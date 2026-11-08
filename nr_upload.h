/*
  nr_upload.h - HTTP uploader for the Night Sound Recorder (v2.6b). Put it next to the .ino.
  v2.6b: filename is copied BEFORE the directory entry is closed (v2.6a passed a dangling pointer ->
         empty/garbage X-Filename -> 400 from the receiver); sole sender; backoff; no restart when idle.
*/
#pragma once

const char*        UPLOAD_HOST = "192.168.12.184";      // PC running nr_receive.py
constexpr uint16_t UPLOAD_PORT = 8082;
constexpr uint32_t CSV_UPLOAD_MS = 3600000UL;           
constexpr uint32_t UPLOAD_STALL_RESTART_MS = 1800000UL;  // only when a clip is pending AND failing  
constexpr uint32_t UPLOAD_SEND_DEADLINE_MS = 25000UL;   
constexpr uint32_t UPLOAD_REPLY_DEADLINE_MS = 8000UL;   

enum UpResult : uint8_t { UP_OK = 0, UP_FILE_BAD = 1, UP_NET_BAD = 2 };
static char upReason[40] = "";                           

// Safely access global variables shared from the main .ino sketch
extern char uploadingName[64];
extern char clipName[64];
extern volatile bool clipOpen;
extern volatile bool eventActive;

UpResult postFile(const char* path, const char* name) {
  upReason[0] = 0;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) { strcpy(upReason, "open failed"); return UP_FILE_BAD; }
  size_t size = f.size();
  if (size == 0) { f.close(); strcpy(upReason, "size 0"); return UP_FILE_BAD; }

  WiFiClient client;
  client.setNoDelay(true);
  if (!client.connect(UPLOAD_HOST, UPLOAD_PORT, 3000)) {
    f.close(); strcpy(upReason, "connect failed");
    Serial.printf("UPLOAD %s %u B -> connect failed\n", name, (unsigned)size);
    return UP_NET_BAD;
  }

  client.printf("POST /upload HTTP/1.1\r\nHost: %s\r\nX-Filename: %s\r\n"
                "Content-Type: application/octet-stream\r\nContent-Length: %u\r\nConnection: close\r\n\r\n",
                UPLOAD_HOST, name, (unsigned)size);

  static uint8_t buf[4096];
  size_t sent = 0;
  uint32_t t0 = millis();
  UpResult res = UP_OK;
  while (sent < size) {
    if (millis() - t0 > UPLOAD_SEND_DEADLINE_MS) {
      snprintf(upReason, sizeof(upReason), "send timeout at %u/%u", (unsigned)sent, (unsigned)size);
      res = UP_NET_BAD; break;
    }
    size_t want = size - sent; if (want > sizeof(buf)) want = sizeof(buf);
    size_t got = f.read(buf, want);
    if (got == 0) {                                  
      snprintf(upReason, sizeof(upReason), "file read 0 at %u/%u", (unsigned)sent, (unsigned)size);
      res = UP_FILE_BAD; break;
    }
    size_t w = 0;
    while (w < got) {
      size_t k = client.write(buf + w, got - w);
      if (k == 0) {
        if (!client.connected() || millis() - t0 > UPLOAD_SEND_DEADLINE_MS) {
          snprintf(upReason, sizeof(upReason), "write stuck at %u/%u", (unsigned)(sent + w), (unsigned)size);
          res = UP_NET_BAD; break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      w += k;
    }
    if (res != UP_OK) break;
    sent += got;
    vTaskDelay(1);                                   
  }
  f.close();

  if (res == UP_OK) {                                
    String resp = "";
    uint32_t t1 = millis();
    while (millis() - t1 < UPLOAD_REPLY_DEADLINE_MS && (client.connected() || client.available())) {
      while (client.available() && resp.length() < 300) resp += (char)client.read();
      if (resp.indexOf("ok ") >= 0 && resp.indexOf("\r\n\r\n") >= 0) break;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    String want = String("ok ") + String((unsigned)size);
    if (resp.indexOf(" 200 ") > 0 && resp.indexOf(want) > 0) {
      res = UP_OK;
    } else {
      snprintf(upReason, sizeof(upReason), "bad reply: %.24s", resp.substring(0, 24).c_str());
      res = (resp.length() == 0) ? UP_NET_BAD : UP_FILE_BAD;   
    }
  }
  client.stop();
  Serial.printf("UPLOAD %s %u B -> %s%s%s\n", name, (unsigned)size,
                res == UP_OK ? "ok" : "FAILED", res == UP_OK ? "" : " : ", res == UP_OK ? "" : upReason);
  return res;
}

// =====================================================
// BACKGROUND TASK & QUEUE MANAGEMENT
// =====================================================
constexpr size_t MIN_UPLOAD_BYTES = 100000;   // skips a clip still being created (pre-roll only ~32 KB)

static bool nameIsClean(const char* nm) {
  if (!nm || !nm[0]) return false;
  for (const char* p = nm; *p; p++) {
    char c = *p;
    bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-';
    if (!ok) return false;
  }
  return true;
}

// Returns true and fills out[] (a private copy) with the oldest finished clip name.
static bool nextClipName(char* out, size_t outSize) {
  bool found = false;
  out[0] = 0;
  fsTake();
  File dir = LittleFS.open("/clips", "r");
  if (dir && dir.isDirectory()) {
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      const char* nm = f.name();
      size_t len = nm ? strlen(nm) : 0;
      bool ok = !f.isDirectory() && len > 4 && len < outSize &&
                strcmp(nm + len - 4, ".wav") == 0 && nameIsClean(nm) &&
                f.size() >= MIN_UPLOAD_BYTES;
      if (ok && (!found || strcmp(nm, out) < 0)) {   // oldest first (names start with the timestamp)
        memcpy(out, nm, len + 1);                     // copy NOW, while f is still open
        found = true;
      }
      f.close();
    }
  }
  if (dir) dir.close();
  fsGive();
  return found;
}

static void uploadTask(void* pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(5000));

  uint32_t lastCsvUpload = millis();
  uint32_t lastProgress = millis();     // last time the queue was empty or an upload succeeded
  uint8_t  fails = 0;
  char     lastBad[48] = "";
  uint8_t  badCnt = 0;

  while (true) {
    uint32_t waitMs = 2000;
    bool pending = false;

    if (WiFi.status() == WL_CONNECTED && !eventActive && !clipOpen) {
      char name[48];
      if (nextClipName(name, sizeof(name))) {
        pending = true;
        char path[64];
        snprintf(path, sizeof(path), "/clips/%s", name);

        fsTake();
        strncpy(uploadingName, name, sizeof(uploadingName) - 1);
        uploadingName[sizeof(uploadingName) - 1] = 0;
        fsGive();

        uploadBusy = true;
        UpResult res = postFile(path, name);   // 'name' is our own copy: safe for the whole transfer
        uploadBusy = false;

        fsTake();
        uploadingName[0] = 0;
        if (res == UP_OK) LittleFS.remove(path);
        fsGive();

        if (res == UP_OK) {
          fails = 0; badCnt = 0; lastBad[0] = 0;
          lastProgress = millis();
          waitMs = 200;                          // drain the queue quickly
        } else {
          if (fails < 250) fails++;
          if (res == UP_FILE_BAD) {              // receiver said no / file unreadable
            if (strcmp(name, lastBad) == 0) badCnt++;
            else { strncpy(lastBad, name, sizeof(lastBad) - 1); lastBad[sizeof(lastBad) - 1] = 0; badCnt = 1; }
            if (badCnt >= 3) {                   // poison file: park it so the queue can move on
              Serial.printf("UPLOAD poison clip %s -> /corrupt\n", name);
              moveToDeadLetter(String(path));
              badCnt = 0; lastBad[0] = 0;
            }
          }
          waitMs = 5000UL << (fails > 4 ? 4 : (fails - 1));   // 5,10,20,40,80 s ...
          if (waitMs > 80000UL) waitMs = 80000UL;
        }
      } else {
        lastProgress = millis();                 // nothing to send is healthy, not a stall
        fails = 0;
      }
    }

    // Hourly CSV logs, only when the queue is empty and the recorder is idle
    if (!pending && WiFi.status() == WL_CONNECTED && !eventActive && !clipOpen &&
        (millis() - lastCsvUpload >= CSV_UPLOAD_MS)) {
      const char* logFiles[] = {"/logs/boot.csv", "/logs/events.csv", "/logs/watch.csv", "/logs/wifi.csv", "/logs/house.csv", "/logs/hvac.csv"};
      bool allSucceeded = true;
      for (const char* path : logFiles) {
        if (eventActive || clipOpen) { allSucceeded = false; break; }
        if (LittleFS.exists(path)) {
          const char* plainName = path + 6;      // skip "/logs/"
          UpResult res = postFile(path, plainName);
          if (res != UP_OK) { allSucceeded = false; break; }
        }
      }
      if (allSucceeded) lastCsvUpload = millis();
      else waitMs = 60000UL;
    }

    // Restart only if a clip has been waiting and failing for a long time while Wi-Fi is up
    if (pending && fails && WiFi.status() == WL_CONNECTED && !eventActive && !clipOpen &&
        millis() - lastProgress > UPLOAD_STALL_RESTART_MS) {
      Serial.println("FATAL: clip upload failing for 30 min with Wi-Fi up. Restarting.");
      houseLog("upload_stalled_restart", "no_successful_upload_30_min");
      vTaskDelay(pdMS_TO_TICKS(200));
      ESP.restart();
    }

    vTaskDelay(pdMS_TO_TICKS(waitMs));
  }
}

void startUploader() {
  xTaskCreatePinnedToCore(
    uploadTask,
    "upload_task",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );
  Serial.println("Background HTTP Uploader Task deployed to Core 0.");
}
