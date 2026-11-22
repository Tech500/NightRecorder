/*
  nr_upload.h - HTTP uploader for the Night Sound Recorder (v2.6g). Put it next to the .ino.
  v2.6g: noteRestart() before each restart (reason shows in the next boot row); first_upload line in house.csv.
  v2.6e (builds on v2.6d) - reliability over capturing every sound:
    1. Uploads start only after the recorder has been quiet for UPLOAD_QUIET_MS (lastEventEndMillis), so a transfer is
       not begun just before the next event of a burst.
    2. Wi-Fi bounce and radio reset are DISABLED (thresholds 250). In the 11-21 logs neither ever cured a stall
       (only a restart did) and three bounces were followed by a 30-min Wi-Fi outage.
    3. Fast restart: UPLOAD_RESTART_FAILS (3) failed sends with the receiver reachable -> ESP.restart().
       At most 3 fast restarts in a row (RTC counter, cleared by any successful upload); after that the old
       10-minute rule applies, so a broken receiver cannot cause a restart loop.
    4. An EMPTY reply (receiver closed the connection at once) is not counted as a stall.
    5. upload_fail reasons for write failures also carry the socket write error code (e=).
  v2.6d (builds on v2.6c):
    1. Log CSVs are DAILY files (events_20261121.csv ...). The day rolls over at NOON local time, so one file =
       one night (same idea as the PC's MM-DD-YYYY night folders). A CLOSED day's files are uploaded and then
       DELETED from the board, but only after the receiver answers "ok <size>". Today's files are re-sent every
       CSV_UPLOAD_MS and kept. levels_*.csv go the same way.
    2. CSV uploads are per file: a file that already went out is not re-sent while a later file keeps failing
       (v2.6c re-sent boot.csv every minute). A closed file the receiver keeps refusing is skipped for the round.
    3. Faster stall recovery. A send that makes no progress for 6 s is abandoned (was: wait out 25 s);
       2 failed sends -> Wi-Fi bounce (was 3); 2 more -> full radio reset; 10 min without success -> restart.
    4. upload_fail lines in house.csv also carry rssi, heap, largest free block and the longest single flash read /
       TCP write, to tell a Wi-Fi/TCP stall from a flash stall.
  v2.6c: stall recovery. (1) A failed CSV upload counts as a stall. (2) 3 failed sends with Wi-Fi up -> one Wi-Fi
         bounce; 10 min with no successful upload -> restart (was 30). (3) 'connect failed' (receiver PC off) never
         counts as a stall. (4) The failure reason is written to house.csv (upload_fail, rate limited).
  v2.6b: filename is copied BEFORE the directory entry is closed (v2.6a passed a dangling pointer ->
         empty/garbage X-Filename -> 400 from the receiver); sole sender; backoff; no restart when idle.
*/
#pragma once
#include <esp_heap_caps.h>

const char*        UPLOAD_HOST = "192.168.12.184";      // PC running nr_receive.py
constexpr uint16_t UPLOAD_PORT = 8082;
constexpr uint32_t CSV_UPLOAD_MS = 900000UL;             // refresh of the current night's logs: every 15 min
constexpr uint32_t UPLOAD_STALL_RESTART_MS = 600000UL;   // 10 min of failed sends (Wi-Fi up, receiver reachable)
constexpr uint8_t  UPLOAD_WIFI_KICK_FAILS  = 250;        // Wi-Fi bounce DISABLED in v2.6e (never cured a stall)
constexpr uint8_t  UPLOAD_WIFI_RESET_FAILS = 250;        // full radio reset DISABLED in v2.6e
constexpr uint8_t  UPLOAD_RESTART_FAILS    = 3;          // failed sends in a row (receiver reachable) -> restart
constexpr uint8_t  UPLOAD_FAST_RESTART_MAX = 3;          // fast restarts in a row before falling back to the 10 min rule
constexpr uint32_t UPLOAD_QUIET_MS = 20000UL;            // recorder must be idle this long before an upload starts
constexpr uint32_t UPLOAD_SEND_DEADLINE_MS = 25000UL;    // whole transfer
constexpr uint32_t UPLOAD_NOPROGRESS_MS = 6000UL;        // no byte accepted by TCP for this long -> give up
constexpr uint32_t UPLOAD_REPLY_DEADLINE_MS = 8000UL;

enum UpResult : uint8_t { UP_OK = 0, UP_FILE_BAD = 1, UP_NET_BAD = 2 };
static char upReason[40] = "";
static bool upEmptyReply = false;                       // last postFile got an empty reply (connection closed at once): not a device stall
RTC_DATA_ATTR static uint8_t fastRestarts = 0;          // survives a software restart, cleared by any successful upload
static bool upConnectFail = false;                      // last postFile failed at connect(): receiver unreachable, not a device stall
static bool firstUploadLogged = false;                  // v2.6g: first_upload line written for this boot
static uint32_t upMaxRdMs = 0, upMaxWrMs = 0, upElapsedMs = 0;   // diagnostics for the last postFile

// Safely access global variables shared from the main .ino sketch
extern char uploadingName[64];
extern char clipName[64];
extern volatile bool clipOpen;
extern volatile bool eventActive;

// True when no event/clip is active AND the last event ended more than UPLOAD_QUIET_MS ago.
// lastEventEndMillis is set in endEvent() in the .ino (no .ino change needed for this).
extern uint32_t lastEventEndMillis;
static inline bool recorderQuiet() {
  if (eventActive || clipOpen) return false;
  return (millis() - lastEventEndMillis) > UPLOAD_QUIET_MS;
}

UpResult postFile(const char* path, const char* name) {
  upReason[0] = 0;
  upConnectFail = false;
  upEmptyReply = false;
  upMaxRdMs = upMaxWrMs = upElapsedMs = 0;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) { strcpy(upReason, "open failed"); return UP_FILE_BAD; }
  size_t size = f.size();
  if (size == 0) { f.close(); strcpy(upReason, "size 0"); return UP_FILE_BAD; }

  WiFiClient client;
  client.setNoDelay(true);
  if (!client.connect(UPLOAD_HOST, UPLOAD_PORT, 3000)) {
    f.close(); strcpy(upReason, "connect failed"); upConnectFail = true;
    Serial.printf("UPLOAD %s %u B -> connect failed\n", name, (unsigned)size);
    return UP_NET_BAD;
  }

  client.printf("POST /upload HTTP/1.1\r\nHost: %s\r\nX-Filename: %s\r\n"
                "Content-Type: application/octet-stream\r\nContent-Length: %u\r\nConnection: close\r\n\r\n",
                UPLOAD_HOST, name, (unsigned)size);

  static uint8_t buf[4096];
  size_t sent = 0;
  uint32_t t0 = millis();
  uint32_t lastProg = t0;
  UpResult res = UP_OK;
  while (sent < size) {
    if (millis() - t0 > UPLOAD_SEND_DEADLINE_MS) {
      snprintf(upReason, sizeof(upReason), "send timeout at %u/%u", (unsigned)sent, (unsigned)size);
      res = UP_NET_BAD; break;
    }
    size_t want = size - sent; if (want > sizeof(buf)) want = sizeof(buf);
    uint32_t tr = millis();
    size_t got = f.read(buf, want);
    uint32_t rd = millis() - tr; if (rd > upMaxRdMs) upMaxRdMs = rd;
    if (got == 0) {
      snprintf(upReason, sizeof(upReason), "file read 0 at %u/%u", (unsigned)sent, (unsigned)size);
      res = UP_FILE_BAD; break;
    }
    size_t w = 0;
    while (w < got) {
      uint32_t tw = millis();
      size_t k = client.write(buf + w, got - w);
      uint32_t wd = millis() - tw; if (wd > upMaxWrMs) upMaxWrMs = wd;
      if (k == 0) {
        if (!client.connected() || millis() - t0 > UPLOAD_SEND_DEADLINE_MS || millis() - lastProg > UPLOAD_NOPROGRESS_MS) {
          snprintf(upReason, sizeof(upReason), "write stuck at %u/%u e=%d", (unsigned)(sent + w), (unsigned)size, (int)client.getWriteError());
          res = UP_NET_BAD; break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
      w += k;
      lastProg = millis();
    }
    if (res != UP_OK) break;
    sent += got;
    vTaskDelay(1);
  }
  f.close();
  upElapsedMs = millis() - t0;

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
      upEmptyReply = (resp.length() == 0);
      res = upEmptyReply ? UP_NET_BAD : UP_FILE_BAD;
    }
  }
  client.stop();
  Serial.printf("UPLOAD %s %u B -> %s%s%s\n", name, (unsigned)size,
                res == UP_OK ? "ok" : "FAILED", res == UP_OK ? "" : " : ", res == UP_OK ? "" : upReason);
  if (res == UP_OK && !firstUploadLogged) {      // v2.6g: once per boot, time from boot to the first acknowledged upload
    firstUploadLogged = true;
    char d[72];
    snprintf(d, sizeof(d), "%lu s after boot (%s)", (unsigned long)(millis() / 1000), name);
    for (char* q = d; *q; q++) if (*q == ',') *q = ';';
    houseLog("first_upload", d);
  }
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

// One house.csv line per 3 minutes at most, so a long stall cannot fill the flash.
static void logUploadFail(const char* name, uint8_t n) {
  static uint32_t lastLog = 0;
  if (lastLog != 0 && millis() - lastLog < 180000UL) return;
  lastLog = millis();
  char d[176];
  snprintf(d, sizeof(d), "%s %s x%u rssi=%d heap=%u blk=%u rd=%u wr=%u ms=%u",
           name, upReason, (unsigned)n, (int)WiFi.RSSI(), (unsigned)ESP.getFreeHeap(),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
           (unsigned)upMaxRdMs, (unsigned)upMaxWrMs, (unsigned)upElapsedMs);
  for (char* p = d; *p; p++) if (*p == ',') *p = ';';
  houseLog("upload_fail", d);
}

// ---------- log/CSV upload (daily files) ----------
constexpr uint8_t CSV_MAX_FILES = 24;
static char    csvSent[CSV_MAX_FILES][40];     // files already delivered in the current round
static uint8_t csvSentN = 0;
static char    csvBad[40] = "";                // file the receiver keeps refusing
static uint8_t csvBadCnt = 0;

static bool csvWasSent(const char* nm) {
  for (uint8_t i = 0; i < csvSentN; i++) if (strcmp(csvSent[i], nm) == 0) return true;
  return false;
}
static void csvMarkSent(const char* nm) {
  if (csvSentN < CSV_MAX_FILES) {
    strncpy(csvSent[csvSentN], nm, 39);
    csvSent[csvSentN][39] = 0;
    csvSentN++;
  }
}

// Copies the names of the non-empty .csv files in /logs (private copies, made while each entry is open).
static int listLogs(char names[][40], int maxN) {
  int n = 0;
  fsTake();
  File dir = LittleFS.open("/logs", "r");
  if (dir && dir.isDirectory()) {
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      const char* nm = f.name();
      size_t len = nm ? strlen(nm) : 0;
      if (n < maxN && !f.isDirectory() && len > 4 && len < 40 &&
          strcmp(nm + len - 4, ".csv") == 0 && nameIsClean(nm) && f.size() > 0) {
        memcpy(names[n], nm, len + 1);
        n++;
      }
      f.close();
    }
  }
  if (dir) dir.close();
  fsGive();
  return n;
}

enum BatchResult : uint8_t { BATCH_DONE = 0, BATCH_FAILED = 1, BATCH_INTERRUPTED = 2 };

// A file whose name contains today's tag (dayTag(): YYYYMMDD of the night's end, rolling at noon) is CURRENT:
// it is uploaded and kept. Everything else (earlier days, legacy un-dated files) is CLOSED: uploaded, then
// deleted once the receiver has confirmed. Closed files go first because deleting them frees flash.
static BatchResult csvBatch(uint8_t* stallFails) {
  static char names[CSV_MAX_FILES][40];
  char tag[16];
  dayTag(tag, sizeof(tag));
  int n = listLogs(names, CSV_MAX_FILES);
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i < n; i++) {
      const char* nm = names[i];
      bool current = strstr(nm, tag) != nullptr;
      if ((pass == 0) == current) continue;          // pass 0: closed files only, pass 1: current files only
      if (csvWasSent(nm)) continue;
      if (!recorderQuiet()) return BATCH_INTERRUPTED;
      char path[64];
      snprintf(path, sizeof(path), "/logs/%s", nm);
      UpResult res = postFile(path, nm);
      if (res == UP_OK) {
        csvMarkSent(nm);
        csvBad[0] = 0; csvBadCnt = 0;
        if (!current) {
          fsTake();
          LittleFS.remove(path);                     // delivered and acknowledged: purge from the ESP32
          fsGive();
        }
        continue;
      }
      if (res == UP_FILE_BAD) {                      // receiver said no / file unreadable
        if (strcmp(nm, csvBad) == 0) csvBadCnt++;
        else { strncpy(csvBad, nm, sizeof(csvBad) - 1); csvBad[sizeof(csvBad) - 1] = 0; csvBadCnt = 1; }
        if (csvBadCnt >= 3) {                        // poison file: leave it for now, do not block the others
          houseLog("csv_skip", nm);
          csvMarkSent(nm);
          csvBad[0] = 0; csvBadCnt = 0;
          continue;
        }
      }
      if (res == UP_NET_BAD && !upConnectFail && !upEmptyReply) {
        if (*stallFails < 250) (*stallFails)++;
        logUploadFail(nm, *stallFails);
      }
      return BATCH_FAILED;
    }
  }
  csvSentN = 0;
  return BATCH_DONE;
}

static void uploadTask(void* pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(5000));

  uint32_t lastCsvUpload = millis() - CSV_UPLOAD_MS + 60000UL;   // first log batch ~1 min after boot
  uint32_t lastProgress = millis();     // last time the queue was empty or an upload succeeded
  uint8_t  fails = 0;
  uint8_t  stallFails = 0;              // consecutive failed sends where the receiver WAS reachable (clips and CSVs)
  bool     kicked = false;              // Wi-Fi bounce already tried in this streak
  bool     hardKicked = false;          // full radio reset already tried in this streak
  char     lastBad[48] = "";
  uint8_t  badCnt = 0;

  while (true) {
    uint32_t waitMs = 2000;
    bool pending = false;

    if (WiFi.status() == WL_CONNECTED && recorderQuiet()) {
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
          stallFails = 0; kicked = false; hardKicked = false;
          fastRestarts = 0;
          lastProgress = millis();
          waitMs = 200;                          // drain the queue quickly
        } else {
          if (fails < 250) fails++;
          if (res == UP_NET_BAD && !upConnectFail && !upEmptyReply) {
            if (stallFails < 250) stallFails++;
            logUploadFail(name, stallFails);
          }
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
        if (stallFails == 0) lastProgress = millis();   // nothing to send is healthy, unless a log upload is failing
        fails = 0;
      }
    }

    // Logs (daily CSVs + levels): only when the clip queue is empty and the recorder is idle
    if (!pending && WiFi.status() == WL_CONNECTED && recorderQuiet() &&
        (millis() - lastCsvUpload >= CSV_UPLOAD_MS)) {
      uploadBusy = true;
      BatchResult br = csvBatch(&stallFails);
      uploadBusy = false;
      if (br == BATCH_DONE) {
        lastCsvUpload = millis();
        stallFails = 0; kicked = false; hardKicked = false;   // only a COMPLETE round clears the streak
        fastRestarts = 0;
        lastProgress = millis();
      } else if (br == BATCH_INTERRUPTED) {
        waitMs = 15000UL;
      } else {
        waitMs = (stallFails > 0 && stallFails <= UPLOAD_WIFI_RESET_FAILS) ? 8000UL : 60000UL;
      }
    }

    // v2.6e fast restart: a restart is the only thing that has cured a stalled send.
    if (stallFails >= UPLOAD_RESTART_FAILS && fastRestarts < UPLOAD_FAST_RESTART_MAX &&
        WiFi.status() == WL_CONNECTED && !eventActive && !clipOpen) {
      fastRestarts++;
      houseLog("upload_fast_restart", "3_failed_sends");
      Serial.println("FATAL: 3 failed sends in a row. Restarting.");
      vTaskDelay(pdMS_TO_TICKS(200));
      noteRestart(RC_FAST_UPLOAD);
      ESP.restart();
    }

    // Recovery ladder (bounce/reset disabled in v2.6e), only for stalls (receiver reachable but a send does not complete):
    //   2 failures in a row -> Wi-Fi bounce; 4 -> full radio reset; 10 min without any success -> restart.
    if (stallFails >= UPLOAD_WIFI_KICK_FAILS && !kicked && WiFi.status() == WL_CONNECTED && !eventActive && !clipOpen) {
      kicked = true;
      houseLog("upload_wifi_bounce", "2_failed_sends");
      WiFi.disconnect();
      vTaskDelay(pdMS_TO_TICKS(1000));
      WiFi.reconnect();
      WiFi.setSleep(false);
      vTaskDelay(pdMS_TO_TICKS(5000));
      fails = 1; waitMs = 3000;
    } else if (stallFails >= UPLOAD_WIFI_RESET_FAILS && kicked && !hardKicked && WiFi.status() == WL_CONNECTED && !eventActive && !clipOpen) {
      hardKicked = true;
      houseLog("upload_wifi_reset", "4_failed_sends");
      WiFi.disconnect(true);                     // radio off
      vTaskDelay(pdMS_TO_TICKS(1000));
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      for (uint8_t i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(500));
      fails = 1; waitMs = 3000;
    }
    if (stallFails > 0 && WiFi.status() == WL_CONNECTED && !eventActive && !clipOpen &&
        millis() - lastProgress > UPLOAD_STALL_RESTART_MS) {
      Serial.println("FATAL: uploads failing for 10 min with Wi-Fi up. Restarting.");
      houseLog("upload_stalled_restart", "no_successful_upload_10_min");
      vTaskDelay(pdMS_TO_TICKS(200));
      noteRestart(RC_STALL_10MIN);
      ESP.restart();
    }

    vTaskDelay(pdMS_TO_TICKS(waitMs));
  }
}

void startUploader() {
  xTaskCreatePinnedToCore(
    uploadTask,
    "upload_task",
    10240,
    nullptr,
    1,
    nullptr,
    0
  );
  Serial.println("Background HTTP Uploader Task deployed to Core 0.");
}
