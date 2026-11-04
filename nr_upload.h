/*
  nr_upload.h - HTTP uploader for the Night Sound Recorder (v2.2). Put it next to the .ino.

  Sends every clip in /clips, and (hourly) every .csv in /, to the PC receiver nr_receive.py.
  A clip is deleted from flash only after the PC answers "ok <same byte count>".
  Runs in its own task on core 0, so the microphone loop is never blocked by the network.
  Included from the .ino just above the SETUP / LOOP section; call startUploader() at the end of setup().
*/
#pragma once

const char*        UPLOAD_HOST = "192.168.12.184";      // PC running nr_receive.py (reserve this address in the router)
constexpr uint16_t UPLOAD_PORT = 8082;
constexpr uint32_t CSV_UPLOAD_MS = 3600000UL;           // CSV files once an hour (and once soon after boot)
constexpr uint32_t UPLOAD_STALL_RESTART_MS = 7200000UL; // clips waiting and nothing sent for 2 h: restart (0 = never)

// POST one file. True only if the PC replies "ok <bytes>" with the size we sent.
static bool postFile(const char* path, const char* name) {
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return false;
  size_t size = f.size();
  if (size == 0) { f.close(); return false; }

  WiFiClient client;
  HTTPClient http;
  http.setReuse(false);
  http.setConnectTimeout(3000);
  http.setTimeout(10000);
  int code = -1;
  bool ok = false;
  if (http.begin(client, UPLOAD_HOST, UPLOAD_PORT, "/upload")) {
    http.addHeader("X-Filename", name);
    http.addHeader("Content-Type", "application/octet-stream");
    code = http.sendRequest("POST", &f, size);          // streams the file from flash
    if (code == 200) {
      String r = http.getString();
      ok = r.startsWith("ok ") && (size_t)r.substring(3).toInt() == size;
    }
    http.end();
  }
  f.close();
  Serial.printf("UPLOAD %s %u B -> %d %s\n", name, (unsigned)size, code, ok ? "ok" : "FAILED");
  return ok;
}

static void uploadTask(void*) {
  uint32_t lastCsv = 0;                 // 0 = send the CSVs soon after boot
  uint32_t lastOk = millis();
  uint8_t  fails = 0;

  for (;;) {
    uint32_t waitMs = 5000;             // after failures: 20 s, 40 s, 80 s, then 120 s
    if (fails) {
      waitMs = 10000UL << (fails > 4 ? 4 : fails);
      if (waitMs > 120000UL) waitMs = 120000UL;
    }
    vTaskDelay(pdMS_TO_TICKS(waitMs));
    if (WiFi.status() != WL_CONNECTED || eventActive || clipOpen) continue;

    // 1) clips: collect names first, then send them one by one
    char names[8][48];
    uint8_t cnt = 0;
    File dir = LittleFS.open("/clips");
    if (dir && dir.isDirectory()) {
      for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        if (!f.isDirectory() && f.size() > 0 && cnt < 8) {       // 0-byte ghosts are left to housekeeping()
          strncpy(names[cnt], f.name(), sizeof(names[0]) - 1);
          names[cnt][sizeof(names[0]) - 1] = 0;
          cnt++;
        }
        f.close();
      }
    }
    if (dir) dir.close();

    bool failed = false;
    for (uint8_t i = 0; i < cnt && !failed; i++) {
      if (eventActive || clipOpen) break;                         // a new event started: finish later
      char path[64];
      snprintf(path, sizeof(path), "/clips/%s", names[i]);
      uploadBusy = true;
      if (postFile(path, names[i])) { LittleFS.remove(path); lastOk = millis(); }
      else failed = true;
      uploadBusy = false;
    }
    if (cnt == 0) lastOk = millis();                              // nothing waiting

    // 2) CSV logs: stay on the ESP32; the PC copy is overwritten each time
    if (!failed && !eventActive && !clipOpen && (lastCsv == 0 || millis() - lastCsv >= CSV_UPLOAD_MS)) {
      char csv[16][40];
      uint8_t nc = 0;
      File root = LittleFS.open("/");
      if (root) {
        for (File f = root.openNextFile(); f; f = root.openNextFile()) {
          const char* nm = f.name();
          size_t L = strlen(nm);
          if (!f.isDirectory() && f.size() > 0 && L > 4 && L < sizeof(csv[0]) &&
              !strcmp(nm + L - 4, ".csv") && nc < 16) {
            strcpy(csv[nc++], nm);
          }
          f.close();
        }
        root.close();
      }
      bool csvOk = true;
      uploadBusy = true;
      for (uint8_t i = 0; i < nc && csvOk; i++) {
        char path[48];
        snprintf(path, sizeof(path), "/%s", csv[i]);
        csvOk = postFile(path, csv[i]);
      }
      uploadBusy = false;
      if (csvOk) lastCsv = millis() | 1;                          // never 0
      else failed = true;
    }

    if (failed) { if (fails < 255) fails++; } else fails = 0;

    // Backstop: clips are waiting and nothing has gone out for 2 h -> restart (set to 0 to disable)
    if (UPLOAD_STALL_RESTART_MS && cnt && millis() - lastOk > UPLOAD_STALL_RESTART_MS &&
        !eventActive && !clipOpen) {
      houseLog("upload_stall_restart", "clips waiting 2h");
      vTaskDelay(pdMS_TO_TICKS(200));
      ESP.restart();
    }
  }
}

static void startUploader() {
  xTaskCreatePinnedToCore(uploadTask, "upload", 8192, nullptr, 1, nullptr, 0);
}
