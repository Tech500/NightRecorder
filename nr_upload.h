/*
  nr_upload.h - Patched HTTP uploader with Global Linkage
*/
#pragma once

const char*        UPLOAD_HOST = "192.168.12.184";      
constexpr uint16_t UPLOAD_PORT = 8082;
constexpr uint32_t CSV_UPLOAD_MS = 3600000UL;           
constexpr uint32_t UPLOAD_STALL_RESTART_MS = 600000UL;  
constexpr uint32_t UPLOAD_SEND_DEADLINE_MS = 25000UL;   
constexpr uint32_t UPLOAD_REPLY_DEADLINE_MS = 8000UL;   

static char upReason[128] = "";                           

inline UpResult postFile(const char* path, const char* name) {
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

  static uint8_t buf[1024];
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
    String resp;
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

static void uploadTask(void*) {
  uint32_t lastCsv = 0;
  uint32_t lastOk = millis();
  uint8_t fails = 0;
  uint32_t lastDbg = 0;
  char badName[48] = "";
  uint8_t badCnt = 0;
  Serial.println("UPLOAD task started");

  for (;;) {
    uint32_t waitMs = 5000;
    if (fails) {
      waitMs = 10000UL << (fails > 4 ? 4 : fails);
      if (waitMs > 120000UL) waitMs = 120000UL;
    }
    vTaskDelay(pdMS_TO_TICKS(waitMs));

    if (millis() - lastDbg >= 30000UL) {
      lastDbg = millis();
      Serial.printf("UPLOAD alive: wifi=%d event=%d clipOpen=%d fails=%u ip=%s heap=%u\n",
                    (int)WiFi.status(), (int)eventActive, (int)clipOpen, (unsigned)fails,
                    WiFi.localIP().toString().c_str(), (unsigned)ESP.getFreeHeap());
    }

    if (WiFi.status() != WL_CONNECTED || eventActive || clipOpen) continue;

    char names[8][48];
    uint8_t cnt = 0;
    fsTake();
    File dir = LittleFS.open("/clips");
    if (dir && dir.isDirectory()) {
      for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        if (!f.isDirectory() && f.size() > 0 && cnt < 8) {
          if (strcmp(f.name(), badName) != 0 || badCnt < 3) {
            strncpy(names[cnt], f.name(), sizeof(names[0]) - 1);
            names[cnt][sizeof(names[0]) - 1] = 0;
            cnt++;
          }
        }
        f.close();
      }
    }
    if (dir) dir.close();
    fsGive();

    if (cnt > 0) {
      for (uint8_t i = 0; i < cnt; i++) {
        if (eventActive || clipOpen || WiFi.status() != WL_CONNECTED) break;
        char path[64];
        snprintf(path, sizeof(path), "/clips/%s", names[i]);
        fsTake();
        strncpy(uploadingName, names[i], sizeof(uploadingName) - 1);
        uploadingName[sizeof(uploadingName) - 1] = 0;
        fsGive();

        uploadBusy = true;
        UpResult r = postFile(path, names[i]);
        uploadBusy = false;

        fsTake();
        uploadingName[0] = 0;
        if (r == UP_OK) {
          LittleFS.remove(path);
          fails = 0; badName[0] = 0; badCnt = 0; lastOk = millis();
        } else {
          fsGive();
          if (r == UP_FILE_BAD) {
            if (strcmp(names[i], badName) == 0) badCnt++;
            else { strncpy(badName, names[i], sizeof(badName) - 1); badName[sizeof(badName) - 1] = 0; badCnt = 1; }
          }
          fails++; break;
        }
        fsGive();
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      continue;
    }

    uint32_t now = millis();
    if (now - lastCsv >= CSV_UPLOAD_MS && now - lastOk >= 60000UL) {
      lastCsv = now;
      const char* csvs[] = { "/boot.csv", "/heap.csv", "/wifi.csv", "/house.csv", "/hvac.csv", "/events.csv", "/watch.csv" };
      for (uint8_t i = 0; i < 7; i++) {
        if (eventActive || clipOpen || WiFi.status() != WL_CONNECTED) break;
        if (!LittleFS.exists(csvs[i])) continue;
        const char* nm = csvs[i] + 1;
        uploadBusy = true;
        UpResult r = postFile(csvs[i], nm);
        uploadBusy = false;
        if (r != UP_OK) { fails++; break; }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }

    if (fails && millis() - lastOk > UPLOAD_STALL_RESTART_MS && !eventActive && !clipOpen) {
      houseLog("upload_stalled_restart", "no_successful_upload_10_min");
      if (levelFile) levelFile.flush();
      vTaskDelay(pdMS_TO_TICKS(200));
      ESP.restart();
    }
  }
}

inline void startUploader() {
  xTaskCreatePinnedToCore(uploadTask, "uploadTask", 4096, nullptr, 1, nullptr, 0);
}
