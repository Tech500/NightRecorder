/*
  ESP32 Night Sound-Event Recorder v2.6b (LittleFS + HTTP upload to the PC, no SD card)
  v2.6b: uploader task is the only sender; no WAV header seek-rewrite (receiver rebuilds header);
         no format on boot; no ESP.restart() on upload failure.
  Board: ESP32 DevKit V1 (DOIT)    Arduino ESP32 core 3.3.x
  Microphone: INMP441 I2S MEMS
*/

#include <Arduino.h>
#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include "secrets.h"
#include <HTTPClient.h>
#include <WebServer.h>
#include <time.h>
#include <math.h>
#include <driver/i2s_std.h>
#include <esp_sntp.h>
#include <esp_system.h>
#include <freertos/semphr.h>
#include <Preferences.h>

WebServer web(8080);

// =====================================================
// SETTINGS & SIGNATURE DEFINITIONS
// =====================================================
volatile bool uploadBusy = false;
void houseLog(const char* ev, const char* detail);
void wifiLog(const char* ev, uint32_t downSec);
void runDeadLetterRecovery();
void checkAndRunScheduledRecovery();
void endEvent(uint32_t endMs, bool forced);
void moveToDeadLetter(String activeFilePath);
bool parseClipEpoch(const char* nm, time_t* out);
uint32_t freeBytes();
bool initMicrophone();
bool readMicrophone(size_t& n);
void onWifiUp();

SemaphoreHandle_t fsLock = nullptr;
char uploadingName[64] = "";
void fsTake();
void fsGive();

void webSetup();
void listDir(const char* path);
bool clipsAllowedNow();
bool localTimeStr(char* out, size_t size, const char* fmt);
uint32_t triggerLevel();
uint32_t releaseLevel();
uint32_t hfTriggerLevel();
uint32_t hfReleaseLevel();
uint32_t lfTriggerLevel();
uint32_t lfReleaseLevel();
void retireOldCsv(const char* path, const char* marker, const char* aside);
const char* resetReasonStr(esp_reset_reason_t r);
void heapTick(uint32_t now);
static bool deleteOldestClipLocked();
bool deleteOldestClip();
bool writeWavHeader(File& f, uint32_t dataBytes);
bool flushWbuf();
bool audioWrite(const int16_t* s, size_t n);
void prPush(const int16_t* s, size_t n);
bool openClip();
void closeClip();
void logEvent(uint32_t eventMs, bool forced);
void startEvent(uint32_t now);
void trackEvent(size_t n, uint32_t now);
void updateBaseline(bool warming);
void updateHfBaseline(bool warming);
void updateLfBaseline(bool warming);
void levelTick(size_t n, uint32_t now);
void watchTick(uint32_t now);
void hvacLog(const char* ev, uint32_t edgeMs, uint32_t low, uint32_t durS, uint32_t peakLow);
void hvacTick(size_t n, uint32_t now);
void housekeeping(uint32_t now);
void serviceNetwork(uint32_t now);
void setup();
void loop();
void onTimeSync(struct timeval* tv);
inline void fsTake() {
  if (fsLock) xSemaphoreTakeRecursive(fsLock, portMAX_DELAY);
}
inline void fsGive() {
  if (fsLock) xSemaphoreGiveRecursive(fsLock);
}

// =====================================================
// PINS & CONSTANTS
// =====================================================
constexpr int I2S_BCLK = 26;
constexpr int I2S_WS = 25;
constexpr int I2S_DIN = 4;

constexpr uint32_t SAMPLE_RATE = 16000;
constexpr uint16_t BITS_PER_SAMPLE = 16;
constexpr uint16_t CHANNELS = 1;

constexpr uint32_t CLIP_MAX_MS = 5000;
constexpr uint32_t PREROLL_SAMPLES = 16384;
constexpr float WRITE_GAIN = 1.0f;
constexpr uint32_t CLIP_MAX_BYTES = (SAMPLE_RATE * 2UL / 1000UL) * CLIP_MAX_MS;

constexpr uint32_t EVENT_LOCKOUT_MS = 5000;
uint32_t lastEventEndMillis = 0;

// Storage Constants
constexpr uint32_t FS_RESERVE_BYTES = 98304;
constexpr bool DELETE_OLDEST_WHEN_FULL = true;
constexpr bool FORMAT_IF_MOUNT_FAILS = true;
constexpr int MAX_ATTEMPTS_BEFORE_ARCHIVE = 3;

// Detection Constants (RMS units)
constexpr float DET_HP_HZ = 100.0f;
constexpr float TRIGGER_RATIO = 6.0f;
constexpr float RELEASE_RATIO = 3.0f;
constexpr uint32_t MIN_TRIGGER_RMS = 300;
constexpr uint32_t MIN_RELEASE_RMS = 180;
constexpr uint8_t DEBOUNCE_CHUNKS = 3;
constexpr uint32_t QUIET_TIME_MS = 1500;
constexpr uint32_t MAX_EVENT_MS = 300000;
constexpr float BASE_RISE = 0.0005f;
constexpr float BASE_FALL = 0.01f;
constexpr float BASE_WARMUP = 0.05f;
constexpr uint32_t WARMUP_MS = 3000;
constexpr float DC_ALPHA = 0.001f;

// Watch log Constants
constexpr bool WATCH_ENABLED = true;
constexpr int32_t WATCH_PEAK = 400;
constexpr uint32_t WATCH_RMS = 120;
constexpr uint32_t WATCH_HF_RMS = 30;
constexpr uint32_t WATCH_LF_RMS = 600;
constexpr uint32_t WATCH_MAX_BYTES = 160000;

// Peak trigger
constexpr int32_t PEAK_TRIGGER = 1000;

// High-frequency band detector
constexpr float HF_HP_HZ = 1500.0f;
constexpr float HF_TRIGGER_RATIO = 6.0f;
constexpr float HF_RELEASE_RATIO = 3.0f;
constexpr uint32_t HF_MIN_TRIGGER_RMS = 60;
constexpr uint32_t HF_MIN_RELEASE_RMS = 20;
constexpr uint8_t HF_DEBOUNCE_CHUNKS = 2;

// Low band Constants
constexpr float LF_LP_HZ = 300.0f;
constexpr float LF_HP_HZ = 20.0f;

// Clip recording hours
constexpr int CLIP_FROM_HOUR = 20;
constexpr int CLIP_UNTIL_HOUR = 8;

// Housekeeping Constants
constexpr uint32_t HOUSEKEEP_MS = 60000UL;
constexpr uint32_t GHOST_AGE_S = 600;
constexpr uint32_t STALE_CLIP_S = 2700;
constexpr uint32_t STALE_RESTART_MIN_UPTIME_MS = 7200000UL;

// Low-band detector Constants
constexpr float LF_TRIGGER_RATIO = 8.0f;
constexpr float LF_RELEASE_RATIO = 3.0f;
constexpr uint32_t LF_MIN_TRIGGER_RMS = 1000;
constexpr uint32_t LF_MIN_RELEASE_RMS = 400;
constexpr uint8_t LF_DEBOUNCE_CHUNKS = 2;

// Loud flags
constexpr int32_t LOUD_PEAK = 3000;
constexpr int32_t VERY_LOUD_PEAK = 5000;

// Level log Constants
constexpr bool LEVEL_LOG_ENABLED = true;
constexpr uint32_t LEVEL_LOG_MS = 60000;
constexpr uint32_t HEAP_LOG_MS = 300000;

// Wi-Fi watchdog Constants
constexpr uint32_t WIFI_LOG_MS = 300000;
constexpr uint32_t WIFI_REJOIN_MS = 600000;
constexpr uint32_t WIFI_RESTART_MS = 1800000;
constexpr uint32_t NTP_SYNC_MS = 6UL * 3600UL * 1000UL;

const char* TZ_INFO = "EST5EDT,M3.2.0/2,M11.1.0/2";

// Subfolder Log Paths
const char* BOOT_LOG_PATH = "/logs/boot.csv";
const char* EVENTS_LOG_PATH = "/logs/events.csv";
const char* WATCH_LOG_PATH = "/logs/watch.csv";
const char* HVAC_LOG_PATH = "/logs/hvac.csv";
const char* WIFI_LOG_PATH = "/logs/wifi.csv";
const char* HOUSE_LOG_PATH = "/logs/house.csv";

// HVAC Engine Variable Definitions
constexpr uint32_t HVAC_WINDOW_MS = 10000;
constexpr uint8_t HVAC_CONFIRM_WINDOWS = 3;
constexpr uint32_t HVAC_ON_MIN_RMS = 450;
constexpr float HVAC_ON_RATIO = 2.2f;
constexpr uint32_t HVAC_OFF_MIN_RMS = 300;
constexpr float HVAC_OFF_RATIO = 1.5f;
constexpr float HVAC_REF_INIT = 200.0f;
constexpr float HVAC_REF_FALL = 0.05f;
constexpr float HVAC_REF_RISE = 0.005f;
constexpr float HVAC_REF_MAX = 600.0f;

double hvRaw = 0, hvDet = 0;
uint32_t hvN = 0, hvLast = 0;
float hvRef = HVAC_REF_INIT;
bool hvOn = false;
uint8_t hvCnt = 0;
uint32_t hvEdgeStart = 0, hvOnStart = 0, hvPeakLow = 0;

#include "nr_upload.h"

i2s_chan_handle_t rxHandle = nullptr;
File audioFile;
File levelFile;

// Globally Visible Arrays (Fully sized to hold 256 frame samples)
int32_t rawSamples[256];
int16_t pcmSamples[256];
uint32_t currentRms = 0;
uint64_t chunkSumSq = 0;
int32_t chunkPeak = 0;
uint32_t detRms = 0;
uint64_t detSumSq = 0;
int32_t detPeak = 0;
uint32_t hfRms = 0;
uint64_t hfSumSq = 0;
int32_t hfPeak = 0;
uint32_t lfRms = 0;

float baseline = 0;
bool baselineInit = false;
float hfBaseline = 0;
bool hfBaselineInit = false;
float lfBaseline = 0;
bool lfBaselineInit = false;
uint32_t warmupEnd = 0;
uint8_t aboveCount = 0;

int16_t preRoll[PREROLL_SAMPLES];
uint32_t prHead = 0;
bool prFull = false;

uint8_t wbuf[4096];
size_t wbufLen = 0;

volatile bool clipOpen = false;
bool clipError = false;
bool noSpace = false;
bool daySkip = false;
uint32_t audioBytes = 0;
uint32_t clipBytesFinal = 0;
uint32_t recordNumber = 1;
char clipName[64] = "";

volatile bool eventActive = false;
bool evHasTime = false;
char evTime[32] = "";
uint32_t evStart = 0, quietStart = 0;
int32_t evPeak = 0;
uint32_t evMaxRms = 0, evBaseline = 0, evTrigger = 0;
int32_t evRawPeak = 0;
uint32_t evRawMaxRms = 0;
uint32_t evHfRms = 0, evLfRms = 0;
int32_t evHfPeak = 0;
const char* evTrigBy = "bb";

int32_t dbPeak = 0, dbRawPeak = 0;
uint32_t dbRms = 0, dbRawRms = 0;
uint32_t dbHfRms = 0, dbLfRms = 0;
int32_t dbHfPeak = 0;
const char* dbTrigBy = "bb";

uint64_t lvSumSq = 0, lvDetSumSq = 0;
double lvLfSq = 0;
uint32_t lvCount = 0, lvLast = 0, lvLines = 0;
int32_t lvPeak = 0, lvDetPeak = 0;
int levelDayKey = -1;

uint32_t wbStart = 0;
int32_t wbPeak = 0, wbRms = 0, wbRawPeak = 0, wbRawRms = 0;
uint32_t wbHfRms = 0, wbLfRms = 0;
int32_t wbHfPeak = 0;
File watchFile;

bool timeConfigured = false;
volatile bool ntpSyncFlag = false;
bool bootLogged = false;
uint32_t wifiDownSince = 0;
volatile bool ntpConfirmed = false;

const int MIN_SIGNAL_STRENGTH = -65;

// Dynamic thresholding evaluation rules
uint32_t triggerLevel() {
  return (uint32_t)(baseline * TRIGGER_RATIO) > MIN_TRIGGER_RMS ? (uint32_t)(baseline * TRIGGER_RATIO) : MIN_TRIGGER_RMS;
}
uint32_t releaseLevel() {
  return (uint32_t)(baseline * RELEASE_RATIO) > MIN_RELEASE_RMS ? (uint32_t)(baseline * RELEASE_RATIO) : MIN_RELEASE_RMS;
}
uint32_t hfTriggerLevel() {
  return (uint32_t)(hfBaseline * HF_TRIGGER_RATIO) > HF_MIN_TRIGGER_RMS ? (uint32_t)(hfBaseline * HF_TRIGGER_RATIO) : HF_MIN_TRIGGER_RMS;
}
uint32_t hfReleaseLevel() {
  return (uint32_t)(hfBaseline * HF_RELEASE_RATIO) > HF_MIN_RELEASE_RMS ? (uint32_t)(hfBaseline * HF_RELEASE_RATIO) : HF_MIN_RELEASE_RMS;
}
uint32_t lfTriggerLevel() {
  return (uint32_t)(lfBaseline * LF_TRIGGER_RATIO) > LF_MIN_TRIGGER_RMS ? (uint32_t)(lfBaseline * LF_TRIGGER_RATIO) : LF_MIN_TRIGGER_RMS;
}
uint32_t lfReleaseLevel() {
  return (uint32_t)(lfBaseline * LF_RELEASE_RATIO) > LF_MIN_RELEASE_RMS ? (uint32_t)(lfBaseline * LF_RELEASE_RATIO) : LF_MIN_RELEASE_RMS;
}
bool clipsAllowedNow() {
  return true;
}
bool localTimeStr(char* out, size_t size, const char* fmt) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) return false;
  return strftime(out, size, fmt, &timeinfo) > 0;
}
void retireOldCsv(const char* path, const char* marker, const char* aside) {
  if (LittleFS.exists(path)) {
    File f = LittleFS.open(path, FILE_READ);
    if (f) {
      String firstLine = f.readStringUntil('\n');
      f.close();
      if (firstLine.indexOf(marker) == -1) {
        Serial.printf("Legacy structure detected in %s. Archiving layout.\n", path);
        LittleFS.rename(path, aside);
      }
    }
  }
}
const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "Vbat_PowerOn";
    case ESP_RST_SW: return "Software_Reset";
    case ESP_RST_PANIC: return "Exception_Panic";
    case ESP_RST_INT_WDT: return "Interrupt_WDT";
    case ESP_RST_TASK_WDT: return "Task_WDT";
    case ESP_RST_WDT: return "Other_WDT";
    case ESP_RST_DEEPSLEEP: return "DeepSleep_Exit";
    default: return "Unknown_Trigger";
  }
}
void heapTick(uint32_t now) {
  static uint32_t lastLog = 0;
  if (now - lastLog >= HEAP_LOG_MS) {
    lastLog = now;
    Serial.printf("[HEAP STATUS] Total Free: %lu Bytes | Min Ever Free: %lu Bytes\n",
                  (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
  }
}
// Server Response Callback Isolation Methods
void handleRootIndex() {
  String s = "";
  File d = LittleFS.open("/clips", "r");
  if (!d || !d.isDirectory()) {
    web.send(200, "text/plain", "No clips directory found.");
    return;
  }
  File f = d.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      s += "" + String(f.name()) + " " + String(f.size()) + " delete";
    }
    f.close();
    f = d.openNextFile();
  }
  d.close();
  web.send(200, "text/html", s);
}
void handleFileGet() {
  File f = LittleFS.open("/clips/" + web.arg("f"), "r");
  if (!f) {
    web.send(404, "text/plain", "not found");
    return;
  }
  web.streamFile(f, "audio/wav");
  f.close();
}
void handleFileDelete() {
  LittleFS.remove("/clips/" + web.arg("f"));
  web.sendHeader("Location", "/");
  web.send(303);
}
void webSetup() {
  web.on("/", handleRootIndex);
  web.on("/get", handleFileGet);
  web.on("/del", handleFileDelete);
  web.begin();
}
void listDir(const char* path) {
  File d = LittleFS.open(path);
  if (!d) {
    Serial.printf("open %s failed\n", path);
    return;
  }
  for (File f = d.openNextFile(); f; f = d.openNextFile()) {
    Serial.printf("%s %s %u\n", f.isDirectory() ? "DIR " : "FILE", f.name(), (unsigned)f.size());
    f.close();
  }
  d.close();
}
static bool deleteOldestClipLocked() {
  File dir = LittleFS.open("/clips", "r");
  if (!dir || !dir.isDirectory()) return false;
  char oldestAny[64] = "";
  char oldestRoutine[64] = "";
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    if (!f.isDirectory() && f.size() > 0 && strcmp(f.name(), uploadingName) != 0) {
      const char* nm = f.name();
      if (oldestAny[0] == 0 || strcmp(nm, oldestAny) < 0) {
        strncpy(oldestAny, nm, sizeof(oldestAny) - 1);
        oldestAny[sizeof(oldestAny) - 1] = 0;
      }
      if (!strstr(nm, "_loud") && (oldestRoutine[0] == 0 || strcmp(nm, oldestRoutine) < 0)) {
        strncpy(oldestRoutine, nm, sizeof(oldestRoutine) - 1);
        oldestRoutine[sizeof(oldestRoutine) - 1] = 0;
      }
    }
    f.close();
  }
  dir.close();
  const char* victim = (oldestRoutine[0] != 0) ? oldestRoutine : oldestAny;
  if (victim[0] == 0) return false;
  char path[64];
  snprintf(path, sizeof(path), "/clips/%s", victim);
  Serial.printf("Space low: deleting oldest clip %s\n", path);
  bool removed = LittleFS.remove(path);
  if (removed) houseLog("rolled_off", victim);
  return removed;
}
bool deleteOldestClip() {
  fsTake();
  bool r = deleteOldestClipLocked();
  fsGive();
  return r;
}
bool writeWavHeader(File& f, uint32_t dataBytes) {
  if (!f) return false;
  f.seek(0);
  uint32_t riffSize = dataBytes + 36;
  uint32_t fmtSize = 16;
  uint16_t audioFmt = 1;
  uint16_t channels = CHANNELS;
  uint32_t sampleRate = SAMPLE_RATE;
  uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
  uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);
  uint16_t bits = BITS_PER_SAMPLE;
  bool ok = true;
  ok &= f.write((const uint8_t*)"RIFF", 4) == 4;
  ok &= f.write((const uint8_t*)&riffSize, 4) == 4;
  ok &= f.write((const uint8_t*)"WAVE", 4) == 4;
  ok &= f.write((const uint8_t*)"fmt ", 4) == 4;
  ok &= f.write((const uint8_t*)&fmtSize, 4) == 4;
  ok &= f.write((const uint8_t*)&audioFmt, 2) == 2;
  ok &= f.write((const uint8_t*)&channels, 2) == 2;
  ok &= f.write((const uint8_t*)&sampleRate, 4) == 4;
  ok &= f.write((const uint8_t*)&byteRate, 4) == 4;
  ok &= f.write((const uint8_t*)&blockAlign, 2) == 2;
  ok &= f.write((const uint8_t*)&bits, 2) == 2;
  ok &= f.write((const uint8_t*)"data", 4) == 4;
  ok &= f.write((const uint8_t*)&dataBytes, 4) == 4;
  return ok;
}
bool flushWbuf() {
  if (!wbufLen) return true;
  size_t w = audioFile.write(wbuf, wbufLen);
  bool ok = (w == wbufLen);
  wbufLen = 0;
  return ok;
}
bool audioWrite(const int16_t* s, size_t n) {
  const uint8_t* p = (const uint8_t*)s;
  size_t bytes = n * sizeof(int16_t);
  bool ok = true;
  audioBytes += bytes;
  while (bytes) {
    size_t space = sizeof(wbuf) - wbufLen;
    size_t c = bytes < space ? bytes : space;
    memcpy(wbuf + wbufLen, p, c);
    wbufLen += c;
    p += c;
    bytes -= c;
    if (wbufLen == sizeof(wbuf)) ok &= flushWbuf();
  }
  return ok;
}
void prPush(const int16_t* s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    preRoll[prHead] = s[i];
    if (++prHead >= PREROLL_SAMPLES) {
      prHead = 0;
      prFull = true;
    }
  }
}
bool openClip() {
  if (!clipsAllowedNow()) {
    daySkip = true;
    return false;
  }
  while (freeBytes() < CLIP_MAX_BYTES + FS_RESERVE_BYTES) {
    if (!DELETE_OLDEST_WHEN_FULL || !deleteOldestClip()) {
      noSpace = true;
      Serial.println("WARNING: not enough free space; clip skipped.");
      return false;
    }
  }
  char stamp[32];
  if (!localTimeStr(stamp, sizeof(stamp), "%Y%m%d_%H%M%S")) {
    snprintf(stamp, sizeof(stamp), "boot%lu", (unsigned long)(millis() / 1000));
  }
  snprintf(clipName, sizeof(clipName), "/clips/%s_%04lu.wav", stamp, (unsigned long)recordNumber++);
  audioFile = LittleFS.open(clipName, FILE_WRITE);
  if (!audioFile) return false;
  if (!writeWavHeader(audioFile, CLIP_MAX_BYTES)) {
    audioFile.close();
    return false;
  }
  audioBytes = 0;
  wbufLen = 0;
  bool ok = true;
  if (prFull) ok &= audioWrite(&preRoll[prHead], PREROLL_SAMPLES - prHead);
  ok &= audioWrite(preRoll, prHead);
  if (!ok) clipError = true;
  return true;
}
void closeClip() {
  if (!clipOpen) return;
  if (!flushWbuf()) clipError = true;
  // v2.6b: no seek-back header rewrite. The header written at open stays as is;
  // nr_receive.py rebuilds the correct header from the byte count.
  audioFile.flush();
  audioFile.close();
  clipOpen = false;
  clipBytesFinal = audioBytes;
  const char* tag = evRawPeak >= VERY_LOUD_PEAK ? "_very_loud" : (evRawPeak >= LOUD_PEAK ? "_loud" : nullptr);
  if (tag) {
    char newName[64];
    int stem = (int)strlen(clipName) - 4;
    snprintf(newName, sizeof(newName), "%.*s%s.wav", stem, clipName, tag);
    fsTake();
    bool renamed = LittleFS.rename(clipName, newName);
    fsGive();
    if (renamed) {
      strncpy(clipName, newName, sizeof(clipName) - 1);
      clipName[sizeof(clipName) - 1] = 0;
    }
  }
}
void logEvent(uint32_t eventMs, bool forced) {
  const char* path = "/logs/events.csv";
  bool isNew = !LittleFS.exists(path);
  File f = LittleFS.open(path, FILE_APPEND);
  if (!f) return;
  if (isNew) f.println("start,file,event_ms,clip_ms,peak,max_rms,baseline,trigger,flags,raw_peak,raw_max_rms,lf_rms,hf_rms,hf_peak,trigger_by");
  char flags[128] = "";
  if (!evHasTime) strcat(flags, "no_ntp ");
  else if (!ntpConfirmed) strcat(flags, "clk_unsynced ");
  if (daySkip) strcat(flags, "day_skip ");
  else if (noSpace) strcat(flags, "no_space ");
  else if (clipName[0] == 0) strcat(flags, "no_clip ");
  if (clipError) strcat(flags, "write_err ");
  if (forced) strcat(flags, "forced_end ");
  if (uploadBusy) strcat(flags, "upl ");
  if (evRawPeak >= VERY_LOUD_PEAK) strcat(flags, "very_loud ");
  else if (evRawPeak >= LOUD_PEAK) strcat(flags, "loud ");
  if (evStart < 90000UL) strcat(flags, "post_boot ");
  f.printf("%s,%s,%lu,%lu,%ld,%lu,%lu,%lu,%s,%ld,%lu,%lu,%lu,%ld,%s\n",
           evTime, clipName, (unsigned long)eventMs,
           (unsigned long)(clipBytesFinal / (SAMPLE_RATE * 2UL / 1000UL)),
           (long)evPeak, (unsigned long)evMaxRms, (unsigned long)evBaseline, (unsigned long)evTrigger,
           flags, (long)evRawPeak, (unsigned long)evRawMaxRms, (unsigned long)evLfRms,
           (unsigned long)evHfRms, (long)evHfPeak, evTrigBy);
  f.close();
}
void startEvent(uint32_t now) {
  evHasTime = localTimeStr(evTime, sizeof(evTime), "%Y-%m-%d %H:%M:%S");
  if (!evHasTime) snprintf(evTime, sizeof(evTime), "boot+%lus", (unsigned long)(now / 1000));
  clipName[0] = 0;
  clipError = false;
  noSpace = false;
  daySkip = false;
  clipBytesFinal = 0;
  evStart = now;
  quietStart = 0;
  evPeak = dbPeak;
  evMaxRms = dbRms;
  evRawPeak = dbRawPeak;
  evRawMaxRms = dbRawRms;
  evHfRms = dbHfRms;
  evHfPeak = dbHfPeak;
  evLfRms = dbLfRms;
  evTrigBy = dbTrigBy;
  evBaseline = (uint32_t)baseline;
  evTrigger = triggerLevel();
  clipOpen = openClip();
  if (!clipOpen) clipName[0] = 0;
  prHead = 0;
  prFull = false;
  aboveCount = 0;
  eventActive = true;
  Serial.printf("EVENT START %s | RMS=%lu base=%lu trig=%lu | %s\n", evTime, (unsigned long)detRms, (unsigned long)evBaseline, (unsigned long)evTrigger, clipName);
}
void trackEvent(size_t n, uint32_t now) {
  if (detPeak > evPeak) evPeak = detPeak;
  if (detRms > evMaxRms) evMaxRms = detRms;
  if (chunkPeak > evRawPeak) evRawPeak = chunkPeak;
  if (currentRms > evRawMaxRms) evRawMaxRms = currentRms;
  if (hfRms > evHfRms) evHfRms = hfRms;
  if (hfPeak > evHfPeak) evHfPeak = hfPeak;
  if (lfRms > evLfRms) evLfRms = lfRms;
  if (clipOpen) {
    if (audioBytes + n * 2 > CLIP_MAX_BYTES) {
      closeClip();
    } else if (!audioWrite(pcmSamples, n)) {
      clipError = true;
      closeClip();
    }
  }
  if (now - evStart >= CLIP_MAX_MS) {
    endEvent(now, false);
    return;
  }
  if (now - evStart >= MAX_EVENT_MS) endEvent(now, true);
}
void endEvent(uint32_t endMs, bool forced) {
  closeClip();
  uint32_t dur = endMs - evStart;
  logEvent(dur, forced);
  Serial.printf("EVENT END   %lu ms | peak=%ld maxRMS=%lu%s\n", (unsigned long)dur, (long)evPeak, (unsigned long)evMaxRms, forced ? " (forced)" : "");
  if (forced) {
    baseline = (float)detRms;
    hfBaseline = (float)hfRms;
    lfBaseline = (float)lfRms;
  }
  // v2.6b: finished clips are picked up and sent by uploadTask (nr_upload.h) only.
  lastEventEndMillis = millis();
  eventActive = false;
  aboveCount = 0;
  quietStart = 0;
}
void moveToDeadLetter(String activeFilePath) {
  if (!LittleFS.exists("/corrupt")) {
    LittleFS.mkdir("/corrupt");
  }
  String rawName = activeFilePath;
  if (rawName.lastIndexOf('/') != -1) {
    rawName = rawName.substring(rawName.lastIndexOf('/') + 1);
  }
  String deadLetterPath = "/corrupt/" + rawName;
  fsTake();
  if (LittleFS.rename(activeFilePath, deadLetterPath)) {
    Serial.printf("Asset isolated safely to dead-letter storage: %s\n", deadLetterPath.c_str());
  } else {
    Serial.println("Critical rename failure. Purging asset immediately to clear queue block.");
    LittleFS.remove(activeFilePath);
  }
  fsGive();
}
void updateBaseline(bool warming) {
  if (!baselineInit) {
    baseline = (float)detRms;
    baselineInit = true;
    return;
  }
  float a = warming ? BASE_WARMUP : ((float)detRms > baseline ? BASE_RISE : BASE_FALL);
  baseline += ((float)detRms - baseline) * a;
}
void updateHfBaseline(bool warming) {
  if (!hfBaselineInit) {
    hfBaseline = (float)hfRms;
    hfBaselineInit = true;
    return;
  }
  float a = warming ? BASE_WARMUP : ((float)hfRms > hfBaseline ? BASE_RISE : BASE_FALL);
  hfBaseline += ((float)hfRms - hfBaseline) * a;
}
void updateLfBaseline(bool warming) {
  if (!lfBaselineInit) {
    lfBaseline = (float)lfRms;
    lfBaselineInit = true;
    return;
  }
  float a = warming ? BASE_WARMUP : ((float)lfRms > lfBaseline ? BASE_RISE : BASE_FALL);
  lfBaseline += ((float)lfRms - lfBaseline) * a;
}
void levelTick(size_t n, uint32_t now) {
  if (!LEVEL_LOG_ENABLED) return;
  lvSumSq += chunkSumSq;
  lvDetSumSq += detSumSq;
  lvLfSq += (double)lfRms * (double)lfRms * (double)n;
  lvCount += n;
  if (chunkPeak > lvPeak) lvPeak = chunkPeak;
  if (detPeak > lvDetPeak) lvDetPeak = detPeak;
  if (now - lvLast < LEVEL_LOG_MS) return;
  lvLast = now;
  struct tm t;
  if (getLocalTime(&t, 0)) {
    int key = (t.tm_mon + 1) * 100 + t.tm_mday;
    if (key != levelDayKey) {
      char path[64];
      strftime(path, sizeof(path), "/logs/levels_%Y%m%d.csv", &t);
      if (levelFile) levelFile.close();
      char aside[64];
      strftime(aside, sizeof(aside), "/logs/levels_%Y%m%d_old.csv", &t);
      retireOldCsv(path, "avg_low", aside);
      bool isNew = !LittleFS.exists(path);
      levelFile = LittleFS.open(path, FILE_APPEND);
      if (levelFile && isNew) levelFile.println("time,avg_rms,peak,baseline,avg_rms_hp,peak_hp,avg_lf,avg_low");
      levelDayKey = key;
    }
    if (levelFile && freeBytes() > FS_RESERVE_BYTES / 2) {
      char ts[32];
      strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &t);
      uint32_t avg = lvCount ? (uint32_t)sqrt((double)lvSumSq / lvCount) : 0;
      uint32_t avgHp = lvCount ? (uint32_t)sqrt((double)detSumSq / lvCount) : 0;
      uint32_t avgLf = lvCount ? (uint32_t)sqrt(lvLfSq / lvCount) : 0;
      double mRaw = lvCount ? (double)lvSumSq / lvCount : 0.0;
      double mHp = lvCount ? (double)detSumSq / lvCount : 0.0;
      uint32_t avgLow = (uint32_t)sqrt(mRaw > mHp ? mRaw - mHp : 0.0);
      levelFile.printf("%s,%lu,%ld,%lu,%lu,%ld,%lu,%lu\n", ts, (unsigned long)avg, (long)lvPeak, (unsigned long)baseline, (unsigned long)avgHp, (long)lvDetPeak, (unsigned long)avgLf, (unsigned long)avgLow);
      if (++lvLines % 5 == 0) levelFile.flush();
    }
  }
  lvSumSq = 0;
  lvDetSumSq = 0;
  lvLfSq = 0;
  lvCount = 0;
  lvPeak = 0;
  lvDetPeak = 0;
}
void watchTick(uint32_t now) {
  if (!WATCH_ENABLED) return;
  if (detPeak > wbPeak) wbPeak = detPeak;
  if (detRms > wbRms) wbRms = detRms;
  if (chunkPeak > wbRawPeak) wbRawPeak = chunkPeak;
  if (currentRms > wbRawRms) wbRawRms = currentRms;
  if (hfRms > wbHfRms) wbHfRms = hfRms;
  if (hfPeak > wbHfPeak) wbHfPeak = hfPeak;
  if (lfRms > wbLfRms) wbLfRms = lfRms;
  if (now - wbStart < 1000) return;
  wbStart = now;
  if (now - lastEventEndMillis < EVENT_LOCKOUT_MS) {
    wbPeak = 0;
    wbRms = 0;
    wbRawPeak = 0;
    wbRawRms = 0;
    wbHfRms = 0;
    wbHfPeak = 0;
    wbLfRms = 0;
    return;
  }
  if (now > warmupEnd && (wbPeak >= WATCH_PEAK || wbRms >= WATCH_RMS || wbHfRms >= WATCH_HF_RMS || wbLfRms >= WATCH_LF_RMS) && freeBytes() > FS_RESERVE_BYTES / 2) {
    char ts[32];
    if (!localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S")) snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(now / 1000));
    const char* path = WATCH_LOG_PATH;
    bool isNew = !LittleFS.exists(path);
    watchFile = LittleFS.open(path, FILE_APPEND);
    if (watchFile) {
      if (watchFile.size() < WATCH_MAX_BYTES) {
        if (isNew) watchFile.println("time,peak_hp,max_rms_hp,raw_peak,raw_max_rms,baseline,in_event,hf_rms,hf_peak,lf_rms");
        watchFile.printf("%s,%ld,%lu,%ld,%lu,%lu,%d,%lu,%ld,%lu\n", ts, (long)wbPeak, (unsigned long)wbRms, (long)wbRawPeak, (unsigned long)wbRawRms, (unsigned long)baseline, eventActive ? 1 : 0, (unsigned long)wbHfRms, (long)wbHfPeak, (unsigned long)wbLfRms);
      }
      watchFile.close();
    }
  }
  wbPeak = 0;
  wbRms = 0;
  wbRawPeak = 0;
  wbRawRms = 0;
  wbHfRms = 0;
  wbHfPeak = 0;
  wbLfRms = 0;
}
void housekeeping(uint32_t now) {
  static uint32_t last = 0, lastFailLog = 0;
  if (last && now - last < HOUSEKEEP_MS) return;
  last = now ? now : 1;
  time_t nowT = time(nullptr);
  bool timeOk = nowT > 1700000000L;
  File dir = LittleFS.open("/clips", "r");
  if (!dir || !dir.isDirectory()) return;
  char ghosts[6][48];
  uint8_t ng = 0;
  time_t oldest = 0;
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    if (!f.isDirectory()) {
      const char* nm = f.name();
      time_t tc = 0;
      bool hasT = parseClipEpoch(nm, &tc);
      if (f.size() == 0) {
        bool old = hasT ? (timeOk && nowT - tc > (time_t)GHOST_AGE_S) : true;
        if (old && ng < 6) {
          strncpy(ghosts[ng], nm, sizeof(ghosts[0]) - 1);
          ghosts[ng][sizeof(ghosts[0]) - 1] = 0;
          ng++;
        }
      } else if (hasT && (!oldest || tc < oldest)) {
        oldest = tc;
      }
    }
    f.close();
  }
  dir.close();
  for (uint8_t i = 0; i < ng; i++) {
    char path[64];
    snprintf(path, sizeof(path), "/clips/%s", ghosts[i]);
    if (LittleFS.remove(path)) {
      houseLog("ghost_removed", ghosts[i]);
    } else if (!lastFailLog || now - lastFailLog > 600000UL) {
      lastFailLog = now ? now : 1;
      houseLog("ghost_remove_failed", ghosts[i]);
    }
  }
  if (!DELETE_OLDEST_WHEN_FULL && timeOk && oldest && nowT - oldest > (time_t)STALE_CLIP_S && now > STALE_RESTART_MIN_UPTIME_MS) {
    char d[64];
    snprintf(d, sizeof(d), "oldest_clip_%lu_min", (unsigned long)((nowT - oldest) / 60));
    houseLog("stale_restart", d);
    if (levelFile) levelFile.flush();
    delay(200);
    ESP.restart();
  }
}
void serviceNetwork(uint32_t now) {
  static uint32_t last = 0, lastOk = 0, lastRejoin = 0;
  if (now - last < 30000 || clipOpen) return;
  last = now;
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiDownSince) {
      wifiLog("up", (now - wifiDownSince) / 1000);
      wifiDownSince = 0;
    }
    onWifiUp();
    if (now - lastOk >= WIFI_LOG_MS) {
      lastOk = now;
      wifiLog("ok", 0);
    }
    return;
  }
  if (!wifiDownSince) {
    wifiDownSince = now ? now : 1;
    wifiLog("down", 0);
  }
  uint32_t down = now - wifiDownSince;
  if (down >= WIFI_RESTART_MS && !eventActive) {
    wifiLog("restart", down / 1000);
    if (levelFile) levelFile.flush();
    delay(200);
    ESP.restart();
  }
  if (down >= WIFI_REJOIN_MS && now - lastRejoin >= WIFI_REJOIN_MS) {
    lastRejoin = now;
    wifiLog("rejoin", down / 1000);
    WiFi.disconnect(false);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  } else {
    WiFi.reconnect();
  }
}
void checkAndRunScheduledRecovery() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  if ((timeinfo.tm_hour % 6 == 0) && (timeinfo.tm_min == 0) && (timeinfo.tm_sec == 0)) {
    Serial.printf("--- NTP CALENDAR TRIGGER [%02d:00:00]: Executing Dead-Letter Sweep ---\n", timeinfo.tm_hour);
    runDeadLetterRecovery();
    delay(1000);
  }
}
void runDeadLetterRecovery() {
  if (WiFi.status() != WL_CONNECTED) return;
  int currentRSSI = WiFi.RSSI();
  if (currentRSSI < -65) {
    Serial.printf("NTP Recovery Aborted: Wireless signal strength token too low (%d dBm)\n", currentRSSI);
    return;
  }
  File root = LittleFS.open("/corrupt", "r");
  if (!root || !root.isDirectory()) return;
  File deadFile = root.openNextFile();
  int processedCount = 0;
  while (deadFile && processedCount < 2) {
    String deadFilePath = deadFile.path();
    String rawName = deadFilePath;
    if (rawName.lastIndexOf('/') != -1) {
      rawName = rawName.substring(rawName.lastIndexOf('/') + 1);
    }
    Serial.printf("NTP Recovery processing archived asset: %s\n", rawName.c_str());
    UpResult r = postFile(deadFilePath.c_str(), rawName.c_str());
    if (r == UP_OK) {
      Serial.println("Archived asset processed successfully. Purging from local flash.");
      deadFile.close();
      LittleFS.remove(deadFilePath);
      processedCount++;
    } else {
      Serial.println("Archived delivery attempt failed. Leaving in folder.");
      deadFile.close();
      break;
    }
    deadFile = root.openNextFile();
  }
  root.close();
}
void setup() {
  Serial.begin(115200);
  delay(500);
  fsLock = xSemaphoreCreateRecursiveMutex();
  setenv("TZ", TZ_INFO, 1);
  tzset();
  Serial.printf("\nESP32 NIGHT SOUND RECORDER v2.6b built %s %s | reset reason: %s\n", __DATE__, __TIME__, resetReasonStr(esp_reset_reason()));

  // begin(true) formats only if the mount fails
  if (!LittleFS.begin(true)) {
    Serial.println("FATAL: LittleFS mount/format failed.");
    while (true) delay(1000);
  }
  Serial.println("LittleFS mounted.");
  
  LittleFS.mkdir("/clips");
  LittleFS.mkdir("/logs");
  Serial.println("Running emergency maintenance sweep on /clips folder...");
  File dir = LittleFS.open("/clips", "r");
  if (dir && dir.isDirectory()) {
    File f = dir.openNextFile();
    while (f) {
      const char* nameStr = f.name();
      bool isCorrupt = false;
      if (nameStr == nullptr || strlen(nameStr) == 0) {
        isCorrupt = true;
      } else {
        char firstChar = nameStr[0];
        if (firstChar < 32 || firstChar > 126) {
          isCorrupt = true;
        }
      }
      if (isCorrupt) {
        String badPath = "/clips/" + String(nameStr ? nameStr : "");
        f.close();
        fsTake();
        LittleFS.remove(badPath);
        Serial.println("EMERGENCY CLEANUP: Purged corrupted ghost entry from file table.");
        fsGive();
      } else {
        f.close();
      }
      f = dir.openNextFile();
    }
    dir.close();
  }
  retireOldCsv("/logs/events.csv", "trigger_by", "/logs/events_v21c.csv");
  retireOldCsv("/logs/watch.csv", "hf_rms", "/logs/watch_v21c.csv");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(500); }
  if (WiFi.status() == WL_CONNECTED) {
    onWifiUp();
    struct tm ti;
    getLocalTime(&ti, 15000);
  }
  if (!initMicrophone()) {
    while (true) delay(1000);
  }
  warmupEnd = millis() + WARMUP_MS;
  startUploader();
}
void loop() {
  uint32_t now = millis();
  serviceNetwork(now);
  if (WiFi.status() == WL_CONNECTED) {
    checkAndRunScheduledRecovery();
  }
  if (ntpSyncFlag) { ntpSyncFlag = false; }
  if (!bootLogged) {
    char ts[32];
    bool haveTime = localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S");
    if (haveTime || millis() > 60000UL) {
      if (!haveTime) snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(millis() / 1000));
      bootLogged = true;
      bool isNew = !LittleFS.exists(BOOT_LOG_PATH);
      File f = LittleFS.open(BOOT_LOG_PATH, FILE_APPEND);
      if (f) {
        if (isNew) f.println("time,reset_reason,code,free_heap");
        f.printf("%s,%s,%d,%lu\n", ts, resetReasonStr(esp_reset_reason()), (int)esp_reset_reason(), (unsigned long)ESP.getFreeHeap());
        f.close();
      }
    }
  }
  if (!eventActive && !clipOpen) housekeeping(now);
  size_t n = 0;
  if (!readMicrophone(n) || n == 0) return;
  now = millis();
  updateBaseline(now < warmupEnd);
  updateHfBaseline(now < warmupEnd);
  updateLfBaseline(now < warmupEnd);
  levelTick(n, now);
  watchTick(now);
  hvacTick(n, now);
  heapTick(now);
  if (eventActive) {
    trackEvent(n, now);
    return;
  }
  prPush(pcmSamples, n);
  if (now < warmupEnd) return;
  bool trigBB = detRms >= triggerLevel();
  bool trigPeak = detPeak >= PEAK_TRIGGER;
  bool trigHF = hfRms >= hfTriggerLevel();
  bool trigLF = lfRms >= lfTriggerLevel();
  if (trigBB || trigPeak || trigHF || trigLF) {
    if (now - lastEventEndMillis < EVENT_LOCKOUT_MS) {
      aboveCount = 0;
      return;
    }
    if (aboveCount == 0) {
      dbPeak = dbRawPeak = 0;
      dbRms = dbRawRms = 0;
      dbHfRms = dbLfRms = 0;
      dbHfPeak = 0;
      dbTrigBy = trigPeak ? "peak" : (trigBB ? "bb" : (trigHF ? "hf" : "lf"));
    }
    if (detPeak > dbPeak) dbPeak = detPeak;
    if (chunkPeak > dbRawPeak) dbRawPeak = chunkPeak;
    if (detRms > dbRms) dbRms = detRms;
    if (currentRms > dbRawRms) dbRawRms = currentRms;
    if (hfRms > dbHfRms) dbHfRms = hfRms;
    if (hfPeak > dbHfPeak) dbHfPeak = hfPeak;
    if (lfRms > dbLfRms) dbLfRms = lfRms;
    uint8_t need = trigPeak ? 1 : (trigBB ? DEBOUNCE_CHUNKS : (trigHF ? HF_DEBOUNCE_CHUNKS : LF_DEBOUNCE_CHUNKS));
    if (++aboveCount >= need) startEvent(now);
  } else {
    aboveCount = 0;
  }
}
uint32_t freeBytes() {
  uint32_t total = LittleFS.totalBytes();
  uint32_t used = LittleFS.usedBytes();
  return total > used ? total - used : 0;
}
bool parseClipEpoch(const char* nm, time_t* out) {
  int Y, M, D, h, m, s;
  if (sscanf(nm, "%4d%2d%2d_%2d%2d%2d", &Y, &M, &D, &h, &m, &s) != 6) return false;
  struct tm t = {};
  t.tm_year = Y - 1900;
  t.tm_mon = M - 1;
  t.tm_mday = D;
  t.tm_hour = h;
  t.tm_min = m;
  t.tm_sec = s;
  t.tm_isdst = -1;
  *out = mktime(&t);
  return true;
}
void wifiLog(const char* ev, uint32_t downSec) {
  if (freeBytes() < FS_RESERVE_BYTES / 2) return;
  char ts[32];
  if (!localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S"))
    snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(millis() / 1000));
  const char* path = "/logs/wifi.csv";
  bool isNew = !LittleFS.exists(path);
  File f = LittleFS.open(path, FILE_APPEND);
  if (!f) return;
  if (isNew) f.println("time,event,rssi_dbm,down_s");
  f.printf("%s,%s,%d,%lu\n", ts, ev,
           WiFi.status() == WL_CONNECTED ? (int)WiFi.RSSI() : 0, (unsigned long)downSec);
  f.close();
}
void houseLog(const char* ev, const char* detail) {
  if (freeBytes() < FS_RESERVE_BYTES / 2) return;
  char ts[32];
  if (!localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S"))
    snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(millis() / 1000));
  fsTake();
  const char* path = "/logs/house.csv";
  bool isNew = !LittleFS.exists(path);
  File f = LittleFS.open(path, FILE_APPEND);
  if (f) {
    if (isNew) f.println("time,event,detail");
    f.printf("%s,%s,%s\n", ts, ev, detail);
    f.close();
  }
  fsGive();
}
void hvacLog(const char* ev, uint32_t edgeMs, uint32_t low, uint32_t durS, uint32_t peakLow) {
  if (freeBytes() < FS_RESERVE_BYTES / 2) return;
  char ts[32];
  struct tm t;
  if (getLocalTime(&t, 0)) {
    time_t tt = time(nullptr) - (time_t)((millis() - edgeMs) / 1000);
    localtime_r(&tt, &t);
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &t);
  } else {
    snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(edgeMs / 1000));
  }
  fsTake();
  bool isNew = !LittleFS.exists(HVAC_LOG_PATH);
  File f = LittleFS.open(HVAC_LOG_PATH, FILE_APPEND);
  if (f) {
    if (isNew) f.println("time,event,low_rms,ref_rms,duration_s,peak_low_rms");
    f.printf("%s,%s,%lu,%lu,%lu,%lu\n", ts, ev, (unsigned long)low, (unsigned long)hvRef, (unsigned long)durS, (unsigned long)peakLow);
    f.close();
  }
  fsGive();
}
void hvacTick(size_t n, uint32_t now) {
  hvRaw += (double)chunkSumSq;
  hvDet += (double)detSumSq;
  hvN += n;
  if (hvLast == 0) hvLast = now ? now : 1;
  if (now - hvLast < HVAC_WINDOW_MS) return;
  uint32_t winStart = hvLast;
  hvLast = now;
  double mRaw = hvN ? hvRaw / hvN : 0.0;
  double mDet = hvN ? hvDet / hvN : 0.0;
  hvRaw = 0;
  hvDet = 0;
  hvN = 0;
  if (now < warmupEnd) return;
  uint32_t low = (uint32_t)sqrt(mRaw > mDet ? mRaw - mDet : 0.0);
  if (!hvOn) {
    float onT = hvRef * HVAC_ON_RATIO;
    if (onT < HVAC_ON_MIN_RMS) onT = (float)HVAC_ON_MIN_RMS;
    if (low > onT) {
      if (hvCnt == 0) {
        hvEdgeStart = winStart;
        hvPeakLow = 0;
      }
      if (low > hvPeakLow) hvPeakLow = low;
      if (++hvCnt >= HVAC_CONFIRM_WINDOWS) {
        hvOn = true;
        hvCnt = 0;
        hvOnStart = hvEdgeStart;
        hvacLog("on", hvEdgeStart, low, 0, hvPeakLow);
      }
    } else {
      hvCnt = 0;
      hvRef += ((float)low - hvRef) * ((float)low > hvRef ? HVAC_REF_RISE : HVAC_REF_FALL);
      if (hvRef < 100.0f) hvRef = 100.0f;
      if (hvRef > HVAC_REF_MAX) hvRef = HVAC_REF_MAX;
    }
  } else {
    if (low > hvPeakLow) hvPeakLow = low;
    float offT = hvRef * HVAC_OFF_RATIO;
    if (offT < HVAC_OFF_MIN_RMS) offT = (float)HVAC_OFF_MIN_RMS;
    if (low < offT) {
      if (hvCnt == 0) hvEdgeStart = winStart;
      if (++hvCnt >= HVAC_CONFIRM_WINDOWS) {
        hvOn = false;
        hvCnt = 0;
        hvacLog("off", hvEdgeStart, low, (hvEdgeStart - hvOnStart) / 1000, hvPeakLow);
      }
    } else {
      hvCnt = 0;
    }
  }
}
void onTimeSync(struct timeval* tv) {
  ntpConfirmed = true;
  ntpSyncFlag = true;
}
void onWifiUp() {
  if (!timeConfigured) {
    sntp_set_sync_interval(NTP_SYNC_MS);
    sntp_set_time_sync_notification_cb(onTimeSync);
    configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");
    timeConfigured = true;
  }
}
bool initMicrophone() {
  i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = 16;
  channelConfig.dma_frame_num = 512;
  esp_err_t err = i2s_new_channel(&channelConfig, nullptr, &rxHandle);
  if (err != ESP_OK) return false;
  i2s_std_config_t standardConfig = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK,
      .ws = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din = (gpio_num_t)I2S_DIN,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false } }
  };
  standardConfig.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  err = i2s_channel_init_std_mode(rxHandle, &standardConfig);
  if (err != ESP_OK) return false;
  err = i2s_channel_enable(rxHandle);
  if (err != ESP_OK) return false;
  Serial.println("INMP441 initialized.");
  return true;
}
bool readMicrophone(size_t& n) {
  size_t bytesRead = 0;
  esp_err_t err = i2s_channel_read(rxHandle, rawSamples, sizeof(rawSamples), &bytesRead, portMAX_DELAY);
  if (err != ESP_OK || bytesRead == 0) {
    n = 0;
    return false;
  }
  n = bytesRead / sizeof(int32_t);
  if (n > 256) n = 256;
  static float dc = 0;
  static bool dcInit = false;
  uint64_t sumSq = 0, dSumSq = 0, hSumSq = 0, lSumSq = 0;
  int32_t peak = 0, dPeak = 0, hPeak = 0;
  static float h1x = 0, h1y = 0, h2x = 0, h2y = 0;
  static bool hInit = false;
  static float f1x = 0, f1y = 0, f2x = 0, f2y = 0, lp1 = 0, lp2 = 0;
  static float l1x = 0, l1y = 0, l2x = 0, l2y = 0;
  const float lkH = 1.0f / (1.0f + 2.0f * (float)M_PI * LF_HP_HZ / (float)SAMPLE_RATE);
  const float hkH = 1.0f / (1.0f + 2.0f * (float)M_PI * HF_HP_HZ / (float)SAMPLE_RATE);
  const float aL = 1.0f - expf(-2.0f * (float)M_PI * LF_LP_HZ / (float)SAMPLE_RATE);
  const float hk = 1.0f / (1.0f + 2.0f * (float)M_PI * DET_HP_HZ / (float)SAMPLE_RATE);
  for (size_t i = 0; i < n; i++) {
    float s = (float)(rawSamples[i] >> 16);
    if (!dcInit) {
      dc = s;
      dcInit = true;
    }
    dc += (s - dc) * DC_ALPHA;
    float x = s - dc;
    int32_t xi = (int32_t)x;
    sumSq += (uint64_t)((int64_t)xi * xi);
    int32_t a = xi < 0 ? -xi : xi;
    if (a > peak) peak = a;
    if (!hInit) {
      h1x = x;
      f1x = x;
      lp1 = x;
      lp2 = x;
      l1x = x;
      hInit = true;
    }
    float y1 = hk * (h1y + x - h1x);
    h1x = x;
    h1y = y1;
    float y2 = hk * (h2y + y1 - h2x);
    h2x = y1;
    h2y = y2;
    int32_t yi = (int32_t)y2;
    dSumSq += (uint64_t)((int64_t)yi * yi);
    int32_t da = yi < 0 ? -yi : yi;
    if (da > dPeak) dPeak = da;
    float g1 = hkH * (f1y + x - f1x);
    f1x = x;
    f1y = g1;
    float g2 = hkH * (f2y + g1 - f2x);
    f2x = g1;
    f2y = g2;
    int32_t hi = (int32_t)g2;
    hSumSq += (uint64_t)((int64_t)hi * hi);
    int32_t ha = hi < 0 ? -hi : hi;
    if (ha > hPeak) hPeak = ha;
    lp1 += aL * (x - lp1);
    lp2 += aL * (lp1 - lp2);
    float q1 = lkH * (l1y + lp2 - l1x);
    l1x = lp2;
    l1y = q1;
    float q2 = lkH * (l2y + q1 - l2x);
    l2x = q1;
    l2y = q2;
    int32_t li = (int32_t)q2;
    lSumSq += (uint64_t)((int64_t)li * li);
    float g = x * WRITE_GAIN;
    if (g > 32767.0f) g = 32767.0f;
    if (g < -32768.0f) g = -32768.0f;
    pcmSamples[i] = (int16_t)g;
  }
  currentRms = n ? (uint32_t)sqrt((double)sumSq / n) : 0;
  chunkSumSq = sumSq;
  chunkPeak = peak;
  detRms = n ? (uint32_t)sqrt((double)dSumSq / n) : 0;
  detSumSq = dSumSq;
  detPeak = dPeak;
  hfRms = n ? (uint32_t)sqrt((double)hSumSq / n) : 0;
  hfSumSq = hSumSq;
  hfPeak = hPeak;
  lfRms = n ? (uint32_t)sqrt((double)lSumSq / n) : 0;
  return true;
}