/*
  ESP32 Night Sound-Event Recorder v2.2  (LittleFS + HTTP upload to the PC, no SD card)
  Board: ESP32 DevKit V1 (DOIT)    Arduino ESP32 core 3.3.x
  Microphone: INMP441 I2S MEMS

  INMP441:  VDD->3.3V  GND->GND  L/R->GND  SCK->GPIO26  WS->GPIO25  SD->GPIO4

  PARTITION SCHEME (Tools menu): choose "No OTA (2MB APP/2MB SPIFFS)".
  The default scheme leaves about 1.4 MB for files; this gives about 1.9 MB.
  Changing the scheme wipes the old filesystem. The first boot after that
  will say "mount failed": set FORMAT_IF_MOUNT_FAILS true, upload, run once,
  then set it back to false.

  Create secrets.h next to this file (add it to .gitignore):
      const char* WIFI_SSID     = "your-ssid";
      const char* WIFI_PASSWORD = "your-password";
  (FTP_USER / FTP_PASSWORD are no longer used.)

  Also next to this file: nr_upload.h (the HTTP uploader). The PC runs nr_receive.py (port 8082).

  v2.2c changes (on top of v2.2), after the 11-04/05 night, when uploads stalled for hours at a time:
    - A recursive mutex (fsLock) guards the places where the recorder (core 1) and the uploader (core 0)
      can touch the same clip: deleteOldestClip(), the clip rename in closeClip(), houseLog(), and the
      uploader's check/remove (nr_upload.h). uploadingName holds the clip being sent right now and the
      rolling buffer never deletes it. Before, both always went for the OLDEST clip, so on a full flash
      the file being sent could be deleted under the uploader.
    - eventActive and clipOpen are volatile (the uploader task on the other core reads them).
    - levels_*.csv gains avg_lf (20-300 Hz band) and avg_low (energy below about 100 Hz, including sub-20 Hz
      swings: sqrt(raw^2 - high-passed^2)). An existing levels file without avg_low is renamed to
      levels_YYYYMMDD_old.csv at first use so rows never mix formats.
    - /hvac.csv: one row when a sustained low-frequency plateau starts ("on") and one when it ends
      ("off", with its length and peak level). Looks at 10 s windows of avg_low; needs 3 windows in a row
      (30 s) to switch, so a single thump or a boot pop is ignored. A STARTING POINT: tune HVAC_* below.
    - Banner and serial output say v2.2c.

  v2.2 changes (on top of v2.1i), after the FTP data transfers wedged for 35 h (11-02 23:15 to 11-04 10:45):
    - The FTP server is gone. The ESP32 now POSTs each clip to nr_receive.py on the PC (nr_upload.h),
      in its own task on core 0, and deletes the clip only after the PC confirms the byte count.
      CSV logs are POSTed once an hour. Nothing on the board listens for connections any more.
    - events.csv flag "ftp" is replaced by "upl": an upload was in progress at the end of the event.
    - The duplicate FtpSrv.begin() in setup() is gone with the rest of the FTP code.
    - Sketch banner and serial output say v2.2.

  v2.1i changes (on top of v2.1h), after the 10-29/30 night:
    - deleteOldestClip() skips 0-byte files. A ghost clip that cannot be removed (10-30: 015654_0003
      survived 2 h) was always the "oldest", the delete failed, and every new clip was dropped as
      no_space although real clips were still on the flash.
    - Each clip removed by the rolling buffer is logged in /house.csv as "rolled_off", so clips lost
      because they were not pulled in time are visible.
    - events.csv flag "ftp" when an FTP client was connected at the end of the event. On 10-30 47% of
      all events fell in the first 2 min after :00/:15/:30/:45, when the PC pull runs; this flag shows
      whether those events happen during a pull.

  v2.1h changes (on top of v2.1g), so the 2 MB filesystem no longer fills up and drops clips
  (10-29: from 21:17 on nearly every event was "no_space"):
    - DELETE_OLDEST_WHEN_FULL is now true. When a new clip will not fit, the oldest clip is
      deleted first (rolling buffer). Routine clips go first; *_loud / *_very_loud clips are only
      deleted when no routine clip is left. Log files (events.csv etc.) are never deleted.
    - The stale-clip restart in housekeeping() is skipped while DELETE_OLDEST_WHEN_FULL is true.
      With a rolling buffer, clips older than STALE_CLIP_S are normal and would otherwise restart
      the ESP32 every 2 h of uptime. Set the switch back to false to restore that watchdog.

  v2.1g changes (on top of v2.1f), after a router power cycle on 10-28 made the clock read UTC:
    - The Indianapolis time zone is set at the very start of setup(), before Wi-Fi. After a
      software restart the ESP32 keeps its clock but forgets the time zone, and until Wi-Fi came
      back it read UTC (4 h ahead), so a daytime event was treated as night and recorded.
    - The recording-hours rule (CLIP_FROM_HOUR..CLIP_UNTIL_HOUR) is only applied once NTP has
      confirmed the time since this boot (ntpConfirmed). Until then clips are recorded, as when
      there is no clock. events.csv flags such rows "clk_unsynced" when they carry a time that
      NTP has not yet confirmed.

  v2.1f changes (on top of v2.1e), after the 10-27/28 night and the daytime pull problems:
    - Clips are recorded only between CLIP_FROM_HOUR (20:00) and CLIP_UNTIL_HOUR (08:00).
      Outside those hours events are still detected and logged (events.csv, flag "day_skip"),
      but no audio is written. This keeps FTP pulls from colliding with recording and keeps
      flash free. If the clock is not set yet, clips are recorded.
    - Housekeeping once a minute while idle (no event, no clip open):
        * zero-byte clips older than GHOST_AGE_S (10 min) are removed ("ghost" clips seen on
          10-28: listed as 0 bytes and not deletable over FTP; a reboot frees them);
        * if the oldest clip is older than STALE_CLIP_S (45 min) the clips are not being
          collected or FTP is stuck: the ESP32 restarts, at most once per
          STALE_RESTART_MIN_UPTIME_MS (2 h) of uptime. Each action is logged in /house.csv.
    - The low band (lf_rms, LF trigger) is now 20-300 Hz: a 2-stage 20 Hz high-pass
      (LF_HP_HZ) removes the 1-3 Hz swings that made up half of the 10-28 night's low-band
      events. Real thumps (40-300 Hz) keep their level. events.csv/watch.csv lf_rms from
      v2.1f on is this narrower band, so it reads lower than before for slow swings.

  v2.1e changes (on top of v2.1d), so low-frequency thumps are not missed:
    - Low-band detector: the below-300 Hz signal (already measured for the lf_rms columns) now
      has its own adaptive baseline and trigger: max(LF_MIN_TRIGGER_RMS, 8 x baseline), 2-chunk
      debounce, and keeps an event alive above its release level. This is what catches
      sub-100 Hz thumps (a door closer, a dryer door) that the 100 Hz detection high-pass hides.
      events.csv trigger_by shows "lf" when it fired first.
    - watch.csv also logs seconds where the low band passes WATCH_LF_RMS, so thumps below the
      clip trigger still leave a row.
    - HF_MIN_TRIGGER_RMS raised from 40 to 60: the 40 floor fired on many marginal sounds
      (18 of 35 events on the first night, most with hf_rms 43-54) and filled the flash.
    All levels are STARTING POINTS. Tune them from lf_rms / hf_rms in watch.csv and events.csv.

  v2.1d changes (on top of v2.1c), so short clicks and high-frequency sounds are not missed:
    - Peak trigger: one chunk with a high-passed peak >= PEAK_TRIGGER starts an event at once
      (no 3-chunk debounce), so a click shorter than 48 ms still makes a clip.
    - High-frequency detector: the signal is also high-passed at HF_HP_HZ (1.5 kHz, 2 stages).
      It has its own adaptive baseline and trigger (max(HF_MIN_TRIGGER_RMS, ratio x baseline),
      2-chunk debounce) and keeps an event alive while it stays above its release level.
    - Band columns: events.csv gains lf_rms (below LF_LP_HZ, 300 Hz), hf_rms, hf_peak and
      trigger_by (bb = broadband, peak, hf). watch.csv gains hf_rms, hf_peak, lf_rms, and a
      second also passes when the high-frequency level passes WATCH_HF_RMS.
    - Old events.csv / watch.csv (without the new columns) are renamed to events_v21c.csv /
      watch_v21c.csv at start-up so rows with different columns never mix in one file.
    - watch.csv stops growing at WATCH_MAX_BYTES so it cannot crowd out clips.
    All new levels are STARTING POINTS. Tune them from hf_rms in watch.csv.

  v2.1c changes (on top of v2.1b), Wi-Fi watchdog:
    - If Wi-Fi stays down for WIFI_REJOIN_MS (10 min) the sketch does a full
      disconnect + WiFi.begin() instead of only WiFi.reconnect(), and repeats that every
      10 min. After WIFI_RESTART_MS (30 min) down it restarts the ESP32 (never during an
      event). Recording continues the whole time; only upload and NTP need the network.
    - /wifi.csv: time,event,rssi_dbm,down_s. An "ok" row with the signal strength every
      5 min while connected; "down", "up" (with the outage length), "rejoin" and "restart"
      rows when the link drops. Separate file, so the existing heap.csv is untouched.
    - levelFile is flushed before a watchdog restart.

  v2.1b changes (on top of v2.1a), for chasing the FTP stalls and unexplained restarts:
    - /boot.csv: one row per start with the reset reason (POWERON, BROWNOUT, PANIC,
      TASK_WDT, SW ...) and free heap. Shows whether a gap in the logs was a power dip.
    - /heap.csv: free heap, minimum free heap and largest free block every 5 minutes.
      A heap that shrinks over hours means a leak (a likely cause of the FTP stalls).
    - events.csv flag "post_boot" for events in the first 90 s after a start
      (usually you handling the unit).

  v2.1a changes (on top of v2.1):
    - Onset fix: peak/RMS are accumulated across the debounce chunks, so events.csv
      no longer drops the first (loudest) chunks of an event.
    - Level flags: events.csv "flags" gets "loud" / "very_loud" when the raw peak
      reaches LOUD_PEAK / VERY_LOUD_PEAK.
    - The same tag is appended to the clip file name: ..._0007_very_loud.wav
      (suffix, so names still sort oldest-first for deleteOldestClip()).

  v2.1 changes:
    - Detection runs on a 100 Hz high-passed copy of the mic signal (2 cascaded
      one-pole stages). Sub-20 Hz thumps and pressure pulses no longer trigger
      clips. Clips are still written from the un-high-passed (DC-blocked) signal.
    - /watch.csv: one row per second whose high-passed peak or RMS passed a low
      "watch" level, whether or not a clip was recorded. Use it to find what
      woke you when no clip exists. Rows are tiny (about 40 bytes).
    - events.csv and levels_*.csv gain columns for the raw (un-filtered) values,
      so old and new nights can be compared. Rename old CSVs before the first run.
    - Time zone fix: configTzTime() replaces setenv(TZ) + configTime(0,0,...).
      configTime() resets TZ to UTC, which is why the logs were in UTC.

  What it does:
    - Adaptive threshold: baseline follows the room (rises slowly, falls fast).
      Trigger = max(floor, baseline * TRIGGER_RATIO); release is lower (hysteresis).
    - Debounce: needs DEBOUNCE_CHUNKS consecutive loud chunks (16 ms each).
    - 1 s pre-roll ring buffer, so the clip includes the onset.
    - WAV clip capped at CLIP_MAX_MS total (pre-roll included).
    - Event length is tracked independently of the clip: /events.csv records
      how long the sound really lasted, even if the WAV was capped or skipped.
    - Free-space guard: if there is no room for a clip, the clip is skipped
      (or the oldest clip is deleted, if DELETE_OLDEST_WHEN_FULL is true) but
      the event is still logged, so timing data is never lost.
    - /levels_YYYYMMDD.csv: avg RMS / peak / baseline every LEVEL_LOG_MS.
    - DC blocker on the mic signal; optional gain on written audio only.
    - Wi-Fi/NTP retry every 30 s; SNTP re-sync every 6 h. Clips still record
      without NTP (named bootNNNN_...), flagged "no_ntp" in the event log.
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

/// =====================================================
// DESIGN SPEC: UNIFIED HEADER DECLARATION
// =====================================================
enum UpResult : uint8_t { UP_OK = 0, UP_FILE_BAD = 1, UP_NET_BAD = 2 };

// Forward Declarations: Tells setup and endEvent these handlers exist
UpResult postFile(const char* path, const char* name); 
void startUploader();

WebServer web(8080);

// =====================================================
// SETTINGS
// =====================================================

volatile bool uploadBusy = false;   // v2.2: true while the uploader (nr_upload.h) is sending a file
void houseLog(const char* ev, const char* detail);   // defined further down

// v2.2c: recorder (core 1) and uploader (core 0) both work on /clips. The mutex is RECURSIVE because
// deleteOldestClip() holds it and calls houseLog(), which takes it again. Hold it only for short operations.
SemaphoreHandle_t fsLock = nullptr;      // created first thing in setup()
char uploadingName[48] = "";             // clip the uploader is sending now (written under fsLock)
inline void fsTake() { if (fsLock) xSemaphoreTakeRecursive(fsLock, portMAX_DELAY); }
inline void fsGive() { if (fsLock) xSemaphoreGiveRecursive(fsLock); }

constexpr uint32_t SAMPLE_RATE      = 16000;
constexpr uint16_t BITS_PER_SAMPLE  = 16;
constexpr uint16_t CHANNELS         = 1;

constexpr uint32_t CLIP_MAX_MS      = 5000;    // Forced 5-second capture for Audacity review
constexpr uint32_t PREROLL_SAMPLES  = 16384;   // Keep the ~1.02s pre-roll buffer
constexpr float    WRITE_GAIN       = 1.0f;    
constexpr uint32_t CLIP_MAX_BYTES   = (SAMPLE_RATE * 2UL / 1000UL) * CLIP_MAX_MS;

// Add these two lines right here under your other constants:
constexpr uint32_t EVENT_LOCKOUT_MS = 5000;    // 5-second silence suppression window
uint32_t lastEventEndMillis = 0;              // Global state lockout timer

// Storage
constexpr uint32_t FS_RESERVE_BYTES        = 98304;  // kept free for logs and LittleFS metadata
constexpr bool     DELETE_OLDEST_WHEN_FULL = true;   // true = rolling buffer of newest clips (oldest are lost!); v2.1h
constexpr bool     FORMAT_IF_MOUNT_FAILS   = false;  // true ONCE after changing partition scheme

// Detection (RMS units, same scale as your old VOX_THRESHOLD)
// All detection levels below are on the HIGH-PASSED signal (see DET_HP_HZ), not the raw one.
// Quiet-room noise measured on the high-passed signal: RMS ~35 (max ~75), peak < ~165.
constexpr float    DET_HP_HZ        = 100.0f;  // detection high-pass corner (2 stages, 12 dB/oct)
constexpr float    TRIGGER_RATIO    = 6.0f;    // ~ +15.5 dB over baseline
constexpr float    RELEASE_RATIO    = 3.0f;    // ~ +9.5 dB over baseline
constexpr uint32_t MIN_TRIGGER_RMS  = 300;     // floor; STARTING POINT, tune from watch.csv and events.csv
constexpr uint32_t MIN_RELEASE_RMS  = 180;
constexpr uint8_t  DEBOUNCE_CHUNKS  = 3;       // 3 x 16 ms
constexpr uint32_t QUIET_TIME_MS    = 1500;    // below release this long = event over
constexpr uint32_t MAX_EVENT_MS     = 300000;  // failsafe: force-end after 5 min
constexpr float    BASE_RISE        = 0.0005f; // per 16 ms chunk, tau ~ 32 s
constexpr float    BASE_FALL        = 0.01f;   // per chunk, tau ~ 1.6 s
constexpr float    BASE_WARMUP      = 0.05f;
constexpr uint32_t WARMUP_MS        = 3000;
constexpr float    DC_ALPHA         = 0.001f;  // ~2.5 Hz high-pass corner

// Watch log: a row for every second whose high-passed peak or RMS passes these (no clip needed)
constexpr bool     WATCH_ENABLED    = true;
constexpr int32_t  WATCH_PEAK       = 400;     // ~2.5x the quiet-room peak
constexpr uint32_t WATCH_RMS        = 120;     // ~3.5x the quiet-room RMS
constexpr uint32_t WATCH_HF_RMS     = 30;      // high-frequency RMS that also earns a watch row
constexpr uint32_t WATCH_LF_RMS     = 600;     // low-band RMS that also earns a watch row
constexpr uint32_t WATCH_MAX_BYTES  = 160000;  // watch.csv stops growing here (keeps flash for clips)

// Peak trigger: a single 16 ms chunk with this high-passed peak starts an event (quiet-room peak < ~165)
constexpr int32_t  PEAK_TRIGGER     = 1000;

// High-frequency band detector (quiet-room RMS in this band is only a few counts)
constexpr float    HF_HP_HZ         = 1500.0f; // high-pass corner, 2 cascaded one-pole stages
constexpr float    HF_TRIGGER_RATIO = 6.0f;
constexpr float    HF_RELEASE_RATIO = 3.0f;
constexpr uint32_t HF_MIN_TRIGGER_RMS = 60;    // floor; STARTING POINT (was 40 in v2.1d)
constexpr uint32_t HF_MIN_RELEASE_RMS = 20;
constexpr uint8_t  HF_DEBOUNCE_CHUNKS = 2;

// Low band (events.csv lf_rms, watch.csv lf_rms, LF trigger): LF_HP_HZ to LF_LP_HZ
constexpr float    LF_LP_HZ         = 300.0f;  // low-pass corner, 2 cascaded one-pole stages
constexpr float    LF_HP_HZ         = 20.0f;   // high-pass corner, 2 cascaded one-pole stages (v2.1f)

// Clip recording hours (local time). Events outside are logged without audio (v2.1f).
constexpr int      CLIP_FROM_HOUR   = 20;      // 20:00 ...
constexpr int      CLIP_UNTIL_HOUR  = 8;       // ... until 08:00

// Housekeeping (v2.1f)
constexpr uint32_t HOUSEKEEP_MS                = 60000UL;     // check once a minute while idle
constexpr uint32_t GHOST_AGE_S                 = 600;         // 0-byte clips older than this are removed
constexpr uint32_t STALE_CLIP_S                = 2700;        // oldest clip older than this => restart
constexpr uint32_t STALE_RESTART_MIN_UPTIME_MS = 7200000UL;   // at most one such restart per 2 h

// Low-band detector (same below-300 Hz signal as lf_rms). Quiet-room level is about 60.
constexpr float    LF_TRIGGER_RATIO = 8.0f;
constexpr float    LF_RELEASE_RATIO = 3.0f;
constexpr uint32_t LF_MIN_TRIGGER_RMS = 1000;  // floor; STARTING POINT. A mild thump measured ~1300
constexpr uint32_t LF_MIN_RELEASE_RMS = 400;
constexpr uint8_t  LF_DEBOUNCE_CHUNKS = 2;

// Loud flags: raw (un-filtered) event peak, so low-frequency thumps count. Used for the
// events.csv flags column and the clip file name suffix (_loud / _very_loud).
constexpr int32_t  LOUD_PEAK        = 3000;
constexpr int32_t  VERY_LOUD_PEAK   = 5000;

// Level log (60 s keeps it near 50 KB/day; set LEVEL_LOG_ENABLED false to save the space)
constexpr bool     LEVEL_LOG_ENABLED = true;
constexpr uint32_t LEVEL_LOG_MS      = 60000;

constexpr uint32_t HEAP_LOG_MS     = 300000;   // /heap.csv interval

// Wi-Fi watchdog (see v2.1c notes)
constexpr uint32_t WIFI_LOG_MS     = 300000;   // /wifi.csv "ok" row interval
constexpr uint32_t WIFI_REJOIN_MS  = 600000;   // down this long: full disconnect + begin(), repeated
constexpr uint32_t WIFI_RESTART_MS = 1800000;  // down this long: restart the ESP32 (not during an event)

constexpr uint32_t NTP_SYNC_MS      = 6UL * 3600UL * 1000UL;  // SNTP re-sync interval (default is 1 h)

const char* TZ_INFO = "EST5EDT,M3.2.0/2,M11.1.0/2";  // Indianapolis

// =====================================================
// PINS
// =====================================================
constexpr int I2S_BCLK = 26;
constexpr int I2S_WS   = 25;
constexpr int I2S_DIN  = 4;

// =====================================================
// GLOBALS
// =====================================================
i2s_chan_handle_t rxHandle = nullptr;

void webSetup() {
  web.on("/", [](){
    String s;
    File d = LittleFS.open("/clips");
    for (File f = d.openNextFile(); f; f = d.openNextFile()) {
      s += "<a href='/get?f=" + String(f.name()) + "'>" + String(f.name()) + "</a> " + String(f.size()) +
           " <a href='/del?f=" + String(f.name()) + "'>delete</a><br>";
      f.close();
    }
    web.send(200, "text/html", s);
  });
  web.on("/get", [](){
    File f = LittleFS.open("/clips/" + web.arg("f"));
    if (!f) { web.send(404, "text/plain", "not found"); return; }
    web.streamFile(f, "audio/wav");
    f.close();
  });
  web.on("/del", [](){
    LittleFS.remove("/clips/" + web.arg("f"));
    web.sendHeader("Location", "/"); web.send(303);
  });
  web.begin();
}

void listDir(const char* path) {
  File d = LittleFS.open(path);
  if (!d) { Serial.printf("open %s failed\n", path); return; }
  for (File f = d.openNextFile(); f; f = d.openNextFile()) {
    Serial.printf("%s %s %u\n", f.isDirectory() ? "DIR " : "FILE", f.name(), (unsigned)f.size());
    f.close();
  }
  d.close();
}

File audioFile;
File levelFile;

int32_t  rawSamples[256];
int16_t  pcmSamples[256];
uint32_t currentRms = 0;   // raw (DC-blocked) RMS: what the old version used
uint64_t chunkSumSq = 0;
int32_t  chunkPeak  = 0;
uint32_t detRms     = 0;   // high-passed RMS: used for detection, baseline and trigger
uint64_t detSumSq   = 0;
int32_t  detPeak    = 0;
uint32_t hfRms      = 0;   // RMS above HF_HP_HZ
uint64_t hfSumSq    = 0;
int32_t  hfPeak     = 0;
uint32_t lfRms      = 0;   // RMS below LF_LP_HZ (logging only)

float    baseline = 0;
bool     baselineInit = false;
float    hfBaseline = 0;
bool     hfBaselineInit = false;
float    lfBaseline = 0;
bool     lfBaselineInit = false;
uint32_t warmupEnd = 0;
uint8_t  aboveCount = 0;

// Pre-roll ring
int16_t  preRoll[PREROLL_SAMPLES];
uint32_t prHead = 0;
bool     prFull = false;

// Flash write buffer (4 KB = one LittleFS block)
uint8_t  wbuf[4096];
size_t   wbufLen = 0;

// Clip state
volatile bool clipOpen = false;
bool     clipError = false;
bool     noSpace = false;
bool     daySkip = false;   // clip skipped because it is outside the recording hours
uint32_t audioBytes = 0;
uint32_t clipBytesFinal = 0;
uint32_t recordNumber = 1;
char     clipName[64];

// Event state
volatile bool eventActive = false;
bool     evHasTime = false;
char     evTime[24];
uint32_t evStart = 0, quietStart = 0;
int32_t  evPeak = 0;
uint32_t evMaxRms = 0, evBaseline = 0, evTrigger = 0;   // evPeak/evMaxRms: high-passed
int32_t  evRawPeak = 0;
uint32_t evRawMaxRms = 0;
uint32_t evHfRms = 0, evLfRms = 0;
int32_t  evHfPeak = 0;
const char* evTrigBy = "bb";

// Peak/RMS accumulated while debouncing, so the onset chunks are not lost
int32_t  dbPeak = 0, dbRawPeak = 0;
uint32_t dbRms = 0, dbRawRms = 0;
uint32_t dbHfRms = 0, dbLfRms = 0;
int32_t  dbHfPeak = 0;
const char* dbTrigBy = "bb";

// Level log
uint64_t lvSumSq = 0, lvDetSumSq = 0;
double   lvLfSq = 0;          // v2.2c: sum of lf_rms^2 x samples
uint32_t lvCount = 0, lvLast = 0, lvLines = 0;
int32_t  lvPeak = 0, lvDetPeak = 0;
int      levelDayKey = -1;

// Watch log (1 s bins)
uint32_t wbStart = 0;
int32_t  wbPeak = 0, wbRawPeak = 0;
uint32_t wbRms = 0, wbRawRms = 0;
uint32_t wbHfRms = 0, wbLfRms = 0;
int32_t  wbHfPeak = 0;
File     watchFile;

// Network
bool timeConfigured = false;
volatile bool ntpSyncFlag = false;
bool bootLogged = false;
uint32_t wifiDownSince = 0;   // millis() when the link was first seen down; 0 = up

// =====================================================
// HELPERS
// =====================================================
// Set by the SNTP callback once NTP has confirmed the time since this boot.
volatile bool ntpConfirmed = false;

// True when clips may be recorded now. If the clock is not set or not NTP-confirmed yet, record.
bool clipsAllowedNow() {
  struct tm t;
  if (!ntpConfirmed || !getLocalTime(&t, 0)) return true;
  int h = t.tm_hour;
  if (CLIP_FROM_HOUR <= CLIP_UNTIL_HOUR) return h >= CLIP_FROM_HOUR && h < CLIP_UNTIL_HOUR;
  return h >= CLIP_FROM_HOUR || h < CLIP_UNTIL_HOUR;      // window crosses midnight
}

bool localTimeStr(char* out, size_t size, const char* fmt) {
  struct tm t;
  if (!getLocalTime(&t, 0)) return false;
  return strftime(out, size, fmt, &t) > 0;
}

inline uint32_t triggerLevel() {
  uint32_t t = (uint32_t)(baseline * TRIGGER_RATIO);
  return t > MIN_TRIGGER_RMS ? t : MIN_TRIGGER_RMS;
}

inline uint32_t releaseLevel() {
  uint32_t r = (uint32_t)(baseline * RELEASE_RATIO);
  return r > MIN_RELEASE_RMS ? r : MIN_RELEASE_RMS;
}

inline uint32_t hfTriggerLevel() {
  uint32_t t = (uint32_t)(hfBaseline * HF_TRIGGER_RATIO);
  return t > HF_MIN_TRIGGER_RMS ? t : HF_MIN_TRIGGER_RMS;
}

inline uint32_t hfReleaseLevel() {
  uint32_t r = (uint32_t)(hfBaseline * HF_RELEASE_RATIO);
  return r > HF_MIN_RELEASE_RMS ? r : HF_MIN_RELEASE_RMS;
}

inline uint32_t lfTriggerLevel() {
  uint32_t t = (uint32_t)(lfBaseline * LF_TRIGGER_RATIO);
  return t > LF_MIN_TRIGGER_RMS ? t : LF_MIN_TRIGGER_RMS;
}

inline uint32_t lfReleaseLevel() {
  uint32_t r = (uint32_t)(lfBaseline * LF_RELEASE_RATIO);
  return r > LF_MIN_RELEASE_RMS ? r : LF_MIN_RELEASE_RMS;
}

// A CSV whose header lacks the new columns is renamed aside, so rows never mix formats.
void retireOldCsv(const char* path, const char* marker, const char* aside) {
  if (!LittleFS.exists(path)) return;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return;
  String head = f.readStringUntil('\n');
  f.close();
  if (head.indexOf(marker) >= 0) return;
  if (LittleFS.exists(aside)) LittleFS.remove(aside);
  LittleFS.rename(path, aside);
}

uint32_t freeBytes() {
  uint32_t total = LittleFS.totalBytes();
  uint32_t used = LittleFS.usedBytes();
  return total > used ? total - used : 0;
}

const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_RESET";
    case ESP_RST_SW:        return "SW_RESET";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

// One row every HEAP_LOG_MS: a heap that keeps shrinking points to a leak.
void heapTick(uint32_t now) {
  static bool started = false;
  static uint32_t last = 0;
  if (started && now - last < HEAP_LOG_MS) return;
  started = true;
  last = now;
  if (freeBytes() < FS_RESERVE_BYTES / 2) return;

  char ts[24];
  if (!localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S"))
    snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(now / 1000));
  bool isNew = !LittleFS.exists("/heap.csv");
  File f = LittleFS.open("/heap.csv", FILE_APPEND);
  if (!f) return;
  if (isNew) f.println("time,free_heap,min_free_heap,max_block");
  f.printf("%s,%lu,%lu,%lu\n", ts, (unsigned long)ESP.getFreeHeap(),
           (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
  f.close();
}

// Names start with YYYYMMDD_HHMMSS, so the lexicographically smallest is the oldest.
// Routine clips are deleted first; *_loud / *_very_loud clips (this suffix also matches
// "_very_loud") only when no routine clip is left. Files outside /clips are never touched.
static bool deleteOldestClipLocked() {
  File dir = LittleFS.open("/clips");
  if (!dir || !dir.isDirectory()) return false;

  char oldestAny[48] = "";
  char oldestRoutine[48] = "";
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    if (!f.isDirectory() && f.size() > 0 && strcmp(f.name(), uploadingName) != 0) {   // v2.1i: skip 0-byte ghosts; v2.2c: and the clip being uploaded
      const char* nm = f.name();
      if (!oldestAny[0] || strcmp(nm, oldestAny) < 0) {
        strncpy(oldestAny, nm, sizeof(oldestAny) - 1);
        oldestAny[sizeof(oldestAny) - 1] = 0;
      }
      if (!strstr(nm, "_loud") && (!oldestRoutine[0] || strcmp(nm, oldestRoutine) < 0)) {
        strncpy(oldestRoutine, nm, sizeof(oldestRoutine) - 1);
        oldestRoutine[sizeof(oldestRoutine) - 1] = 0;
      }
    }
    f.close();
  }
  dir.close();

  const char* victim = oldestRoutine[0] ? oldestRoutine : oldestAny;
  if (!victim[0]) return false;

  char path[64];
  snprintf(path, sizeof(path), "/clips/%s", victim);
  Serial.printf("Space low: deleting oldest clip %s\n", path);
  bool removed = LittleFS.remove(path);
  if (removed) houseLog("rolled_off", victim);       // v2.1i: leave a trace of every clip lost to the rolling buffer
  return removed;
}

// v2.2c: the uploader claims a clip (uploadingName) under the same lock, so the two can never collide.
bool deleteOldestClip() {
  fsTake();
  bool r = deleteOldestClipLocked();
  fsGive();
  return r;
}

// =====================================================
// WAV HEADER
// =====================================================
bool writeWavHeader(File& f, uint32_t dataBytes) {
  if (!f) return false;
  f.seek(0);

  uint32_t riffSize   = dataBytes + 36;
  uint32_t fmtSize    = 16;
  uint16_t audioFmt   = 1;
  uint16_t channels   = CHANNELS;
  uint32_t sampleRate = SAMPLE_RATE;
  uint32_t byteRate   = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
  uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);
  uint16_t bits       = BITS_PER_SAMPLE;

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

// =====================================================
// BUFFERED AUDIO WRITE (4 KB blocks to flash)
// =====================================================
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
    wbufLen += c; p += c; bytes -= c;
    if (wbufLen == sizeof(wbuf)) ok &= flushWbuf();
  }
  return ok;
}

// =====================================================
// PRE-ROLL RING
// =====================================================
void prPush(const int16_t* s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    preRoll[prHead] = s[i];
    if (++prHead >= PREROLL_SAMPLES) { prHead = 0; prFull = true; }
  }
}

// =====================================================
// CLIP OPEN / CLOSE
// =====================================================
bool openClip() {
  if (!clipsAllowedNow()) { daySkip = true; return false; }   // outside recording hours: log only

  // Free-space guard. usedBytes() is cheap on LittleFS.
  while (freeBytes() < CLIP_MAX_BYTES + FS_RESERVE_BYTES) {
    if (!DELETE_OLDEST_WHEN_FULL || !deleteOldestClip()) {
      noSpace = true;
      Serial.println("WARNING: not enough free space; clip skipped, event still logged.");
      return false;
    }
  }

  char stamp[24];
  if (!localTimeStr(stamp, sizeof(stamp), "%Y%m%d_%H%M%S")) {
    snprintf(stamp, sizeof(stamp), "boot%lu", (unsigned long)(millis() / 1000));
  }
  snprintf(clipName, sizeof(clipName), "/clips/%s_%04lu.wav",
           stamp, (unsigned long)recordNumber++);

  audioFile = LittleFS.open(clipName, FILE_WRITE);
  if (!audioFile) {
    Serial.println("ERROR: cannot create clip.");
    return false;
  }

  // Header claims the maximum length until the clip is finalized, so a clip
  // cut short by a power loss still plays instead of showing 0 bytes.
  if (!writeWavHeader(audioFile, CLIP_MAX_BYTES)) {
    Serial.println("ERROR: WAV header write failed.");
    audioFile.close();
    return false;
  }

  audioBytes = 0;
  wbufLen = 0;

  // Pre-roll, oldest sample first.
  bool ok = true;
  if (prFull) ok &= audioWrite(&preRoll[prHead], PREROLL_SAMPLES - prHead);
  ok &= audioWrite(preRoll, prHead);
  if (!ok) clipError = true;
  return true;
}

void closeClip() {
  if (!clipOpen) return;
  if (!flushWbuf()) clipError = true;
  if (!writeWavHeader(audioFile, audioBytes & ~1U)) clipError = true;
  audioFile.close();
  clipOpen = false;
  clipBytesFinal = audioBytes;

  // Tag the file name by level so loud clips can be found without opening them.
  const char* tag = evRawPeak >= VERY_LOUD_PEAK ? "_very_loud"
                  : evRawPeak >= LOUD_PEAK      ? "_loud" : nullptr;
  if (tag) {
    char newName[80];
    int stem = (int)strlen(clipName) - 4;                 // strip ".wav"
    snprintf(newName, sizeof(newName), "%.*s%s.wav", stem, clipName, tag);
    fsTake();                                             // v2.2c: not while the uploader checks/claims files
    bool renamed = LittleFS.rename(clipName, newName);
    fsGive();
    if (renamed) {
      strncpy(clipName, newName, sizeof(clipName) - 1);   // events.csv logs the new name
      clipName[sizeof(clipName) - 1] = 0;
    }
  }
}

// =====================================================
// EVENT LOG
// =====================================================
void logEvent(uint32_t eventMs, bool forced) {
  const char* path = "/events.csv";
  bool isNew = !LittleFS.exists(path);
  File f = LittleFS.open(path, FILE_APPEND);
  if (!f) { Serial.println("ERROR: cannot open events.csv"); return; }
  if (isNew) f.println("start,file,event_ms,clip_ms,peak,max_rms,baseline,trigger,flags,raw_peak,raw_max_rms,lf_rms,hf_rms,hf_peak,trigger_by");

  char flags[72] = "";
  if (!evHasTime)    strcat(flags, "no_ntp ");
  else if (!ntpConfirmed) strcat(flags, "clk_unsynced ");
  if (daySkip)       strcat(flags, "day_skip ");
  else if (noSpace)  strcat(flags, "no_space ");
  else if (!clipName[0]) strcat(flags, "no_clip ");
  if (clipError)     strcat(flags, "write_err ");
  if (forced)        strcat(flags, "forced_end ");
  if (uploadBusy)    strcat(flags, "upl ");            // v2.2: an upload was in progress
  if (evRawPeak >= VERY_LOUD_PEAK)  strcat(flags, "very_loud ");
  else if (evRawPeak >= LOUD_PEAK)  strcat(flags, "loud ");
  if (evStart < 90000UL) strcat(flags, "post_boot ");   // first 90 s after a start

  f.printf("%s,%s,%lu,%lu,%ld,%lu,%lu,%lu,%s,%ld,%lu,%lu,%lu,%ld,%s\n",
           evTime, clipName,
           (unsigned long)eventMs,
           (unsigned long)(clipBytesFinal / (SAMPLE_RATE * 2UL / 1000UL)),
           (long)evPeak,
           (unsigned long)evMaxRms,
           (unsigned long)evBaseline,
           (unsigned long)evTrigger,
           flags,
           (long)evRawPeak,
           (unsigned long)evRawMaxRms,
           (unsigned long)evLfRms,
           (unsigned long)evHfRms,
           (long)evHfPeak,
           evTrigBy);
  f.close();
}

// =====================================================
// EVENT START / TRACK / END
// =====================================================
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
  evPeak = dbPeak;            // includes the debounce chunks (onset), not just the trigger chunk
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

  // Pre-roll has been written; restart it so stale audio can't leak into the next event.
  prHead = 0;
  prFull = false;
  aboveCount = 0;
  eventActive = true;

  Serial.printf("EVENT START %s | RMS=%lu (raw %lu) base=%lu trig=%lu | %s\n",
                evTime, (unsigned long)detRms, (unsigned long)currentRms,
                (unsigned long)evBaseline, (unsigned long)evTrigger, clipName);
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

  // Force close recording immediately at the 5-second mark
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
  Serial.printf("EVENT END   %lu ms | peak=%ld maxRMS=%lu%s\n",
                (unsigned long)dur, (long)evPeak, (unsigned long)evMaxRms,
                forced ? " (forced)" : "");
  if (forced) { baseline = (float)detRms; hfBaseline = (float)hfRms; lfBaseline = (float)lfRms; }   

  // Streams to your PC instantly and removes from LittleFS
  if (!daySkip && !noSpace && !clipError && clipName != 0) {
    const char* rawName = clipName;
    if (strncmp(rawName, "/clips/", 7) == 0) rawName += 7;

        uploadBusy = true;
    uint8_t r = postFile(clipName, rawName);  // Changed UpResult to uint8_t
    uploadBusy = false;

    if (r == 0) {  // 0 corresponds explicitly to UP_OK
      fsTake(); 
      LittleFS.remove(clipName);
      Serial.printf("IMMEDIATE PURGE SUCCESS: Wiped %s from flash\n", rawName);
      fsGive();
    }

  }

  // Engage the 5-second lockout window
  lastEventEndMillis = millis(); 

  eventActive = false;
  aboveCount = 0;
  quietStart = 0;
}


// =====================================================
// BASELINE + LEVEL LOG
// =====================================================
void updateBaseline(bool warming) {
  if (!baselineInit) { baseline = (float)detRms; baselineInit = true; return; }
  float a = warming ? BASE_WARMUP
                    : ((float)detRms > baseline ? BASE_RISE : BASE_FALL);
  baseline += ((float)detRms - baseline) * a;
}

void updateHfBaseline(bool warming) {
  if (!hfBaselineInit) { hfBaseline = (float)hfRms; hfBaselineInit = true; return; }
  float a = warming ? BASE_WARMUP
                    : ((float)hfRms > hfBaseline ? BASE_RISE : BASE_FALL);
  hfBaseline += ((float)hfRms - hfBaseline) * a;
}

void updateLfBaseline(bool warming) {
  if (!lfBaselineInit) { lfBaseline = (float)lfRms; lfBaselineInit = true; return; }
  float a = warming ? BASE_WARMUP
                    : ((float)lfRms > lfBaseline ? BASE_RISE : BASE_FALL);
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
      char path[32];
      strftime(path, sizeof(path), "/levels_%Y%m%d.csv", &t);
      if (levelFile) levelFile.close();
      char aside[40];
      strftime(aside, sizeof(aside), "/levels_%Y%m%d_old.csv", &t);
      retireOldCsv(path, "avg_low", aside);       // v2.2c: a file in the older format is renamed aside
      bool isNew = !LittleFS.exists(path);
      levelFile = LittleFS.open(path, FILE_APPEND);
      if (levelFile && isNew) levelFile.println("time,avg_rms,peak,baseline,avg_rms_hp,peak_hp,avg_lf,avg_low");
      levelDayKey = key;
    }
    if (levelFile && freeBytes() > FS_RESERVE_BYTES / 2) {
      char ts[24];
      strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &t);
      uint32_t avg = lvCount ? (uint32_t)sqrt((double)lvSumSq / lvCount) : 0;
      uint32_t avgHp = lvCount ? (uint32_t)sqrt((double)lvDetSumSq / lvCount) : 0;
      uint32_t avgLf = lvCount ? (uint32_t)sqrt(lvLfSq / lvCount) : 0;            // 20-300 Hz band
      double mRaw = lvCount ? (double)lvSumSq / lvCount : 0.0;
      double mHp  = lvCount ? (double)lvDetSumSq / lvCount : 0.0;
      uint32_t avgLow = (uint32_t)sqrt(mRaw > mHp ? mRaw - mHp : 0.0);            // below ~100 Hz incl. sub-20 Hz
      levelFile.printf("%s,%lu,%ld,%lu,%lu,%ld,%lu,%lu\n", ts, (unsigned long)avg,
                       (long)lvPeak, (unsigned long)baseline,
                       (unsigned long)avgHp, (long)lvDetPeak,
                       (unsigned long)avgLf, (unsigned long)avgLow);
      if (++lvLines % 5 == 0) levelFile.flush();   // about every 5 minutes
    }
  }
  lvSumSq = 0; lvDetSumSq = 0; lvLfSq = 0; lvCount = 0; lvPeak = 0; lvDetPeak = 0;
}

// One row per second in which the high-passed peak or RMS passed the watch levels.
// Independent of events and clips, so it shows what happened when no clip was made.
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
  
// Force drop metrics rows during the lockout countdown window
  if (now - lastEventEndMillis < EVENT_LOCKOUT_MS) {
    wbPeak = 0; wbRms = 0; wbRawPeak = 0; wbRawRms = 0;
    wbHfRms = 0; wbHfPeak = 0; wbLfRms = 0;
    return;
  }


  if (now > warmupEnd && (wbPeak >= WATCH_PEAK || wbRms >= WATCH_RMS || wbHfRms >= WATCH_HF_RMS || wbLfRms >= WATCH_LF_RMS) &&
      freeBytes() > FS_RESERVE_BYTES / 2) {
    char ts[24];
    if (!localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S"))
      snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(now / 1000));
    bool isNew = !LittleFS.exists("/watch.csv");
    watchFile = LittleFS.open("/watch.csv", FILE_APPEND);
    if (watchFile) {
      if (watchFile.size() < WATCH_MAX_BYTES) {
        if (isNew) watchFile.println("time,peak_hp,max_rms_hp,raw_peak,raw_max_rms,baseline,in_event,hf_rms,hf_peak,lf_rms");
        watchFile.printf("%s,%ld,%lu,%ld,%lu,%lu,%d,%lu,%ld,%lu\n", ts,
                         (long)wbPeak, (unsigned long)wbRms,
                         (long)wbRawPeak, (unsigned long)wbRawRms,
                         (unsigned long)baseline, eventActive ? 1 : 0,
                         (unsigned long)wbHfRms, (long)wbHfPeak, (unsigned long)wbLfRms);
      }
      watchFile.close();
    }
  }
  wbPeak = 0; wbRms = 0; wbRawPeak = 0; wbRawRms = 0;
  wbHfRms = 0; wbHfPeak = 0; wbLfRms = 0;
}

// =====================================================
// LOW-FREQUENCY PLATEAU LOG  (v2.2c)  -> /hvac.csv
// =====================================================
// Sustained rumble (an HVAC unit, a blower) sits below the 100 Hz detection high-pass, so it makes no
// clip. This watches the energy below ~100 Hz, sqrt(raw^2 - high-passed^2), in 10 s windows and logs
// when a plateau starts and ends. 11-04 21:42-21:46 looked like this: raw level 4x normal, high-passed
// level unchanged. STARTING POINTS: tune from the avg_low column in levels_*.csv and from hvac.csv.
constexpr uint32_t HVAC_WINDOW_MS       = 10000;   // analysis window
constexpr uint8_t  HVAC_CONFIRM_WINDOWS = 3;       // windows in a row needed to switch (30 s)
constexpr uint32_t HVAC_ON_MIN_RMS      = 450;     // quiet room reads about 200
constexpr float    HVAC_ON_RATIO        = 2.2f;    // on when above max(HVAC_ON_MIN_RMS, 2.2 x reference)
constexpr uint32_t HVAC_OFF_MIN_RMS     = 300;
constexpr float    HVAC_OFF_RATIO       = 1.5f;    // off when below max(HVAC_OFF_MIN_RMS, 1.5 x reference)
constexpr float    HVAC_REF_INIT        = 200.0f;  // quiet-room reference at start
constexpr float    HVAC_REF_FALL        = 0.05f;   // reference follows a quieter room quickly (per 10 s window, tau ~ 3 min)
constexpr float    HVAC_REF_RISE        = 0.005f;  // ... and a noisier room slowly (tau ~ 30 min), so a gradual ramp is not absorbed
constexpr float    HVAC_REF_MAX         = 600.0f;

double   hvRaw = 0, hvDet = 0;       // sums of squares over the window
uint32_t hvN = 0, hvLast = 0;
float    hvRef = HVAC_REF_INIT;
bool     hvOn = false;
uint8_t  hvCnt = 0;
uint32_t hvEdgeStart = 0, hvOnStart = 0, hvPeakLow = 0;

void hvacLog(const char* ev, uint32_t edgeMs, uint32_t low, uint32_t durS, uint32_t peakLow) {
  if (freeBytes() < FS_RESERVE_BYTES / 2) return;
  char ts[24];
  struct tm t;
  if (getLocalTime(&t, 0)) {
    time_t tt = time(nullptr) - (time_t)((millis() - edgeMs) / 1000);   // when the edge began, not when it was confirmed
    localtime_r(&tt, &t);
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &t);
  } else {
    snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(edgeMs / 1000));
  }
  fsTake();
  bool isNew = !LittleFS.exists("/hvac.csv");
  File f = LittleFS.open("/hvac.csv", FILE_APPEND);
  if (f) {
    if (isNew) f.println("time,event,low_rms,ref_rms,duration_s,peak_low_rms");
    f.printf("%s,%s,%lu,%lu,%lu,%lu\n", ts, ev, (unsigned long)low, (unsigned long)hvRef,
             (unsigned long)durS, (unsigned long)peakLow);
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
  hvRaw = 0; hvDet = 0; hvN = 0;
  if (now < warmupEnd) return;                    // ignore start-up transients
  uint32_t low = (uint32_t)sqrt(mRaw > mDet ? mRaw - mDet : 0.0);

  if (!hvOn) {
    float onT = hvRef * HVAC_ON_RATIO;
    if (onT < HVAC_ON_MIN_RMS) onT = (float)HVAC_ON_MIN_RMS;
    if (low > onT) {
      if (hvCnt == 0) { hvEdgeStart = winStart; hvPeakLow = 0; }
      if (low > hvPeakLow) hvPeakLow = low;
      if (++hvCnt >= HVAC_CONFIRM_WINDOWS) {
        hvOn = true; hvCnt = 0; hvOnStart = hvEdgeStart;
        hvacLog("on", hvEdgeStart, low, 0, hvPeakLow);
      }
    } else {
      hvCnt = 0;
      hvRef += ((float)low - hvRef) * ((float)low > hvRef ? HVAC_REF_RISE : HVAC_REF_FALL);   // only while no plateau is active
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
        hvOn = false; hvCnt = 0;
        hvacLog("off", hvEdgeStart, low, (hvEdgeStart - hvOnStart) / 1000, hvPeakLow);
      }
    } else {
      hvCnt = 0;
    }
  }
}

// =====================================================
// MICROPHONE
// =====================================================
bool initMicrophone() {
  i2s_chan_config_t channelConfig =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

  // 16 x 512 frames = 512 ms of DMA headroom against flash write stalls.
  channelConfig.dma_desc_num  = 16;
  channelConfig.dma_frame_num = 512;

  esp_err_t err = i2s_new_channel(&channelConfig, nullptr, &rxHandle);
  if (err != ESP_OK) { Serial.printf("I2S channel error: %d\n", err); return false; }

  i2s_std_config_t standardConfig = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_BCLK,
      .ws   = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)I2S_DIN,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false }
    }
  };
  standardConfig.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  err = i2s_channel_init_std_mode(rxHandle, &standardConfig);
  if (err != ESP_OK) { Serial.printf("I2S config error: %d\n", err); return false; }
  err = i2s_channel_enable(rxHandle);
  if (err != ESP_OK) { Serial.printf("I2S enable error: %d\n", err); return false; }

  Serial.println("INMP441 initialized.");
  return true;
}

bool readMicrophone(size_t& n) {
  size_t bytesRead = 0;
  esp_err_t err = i2s_channel_read(rxHandle, rawSamples, sizeof(rawSamples),
                                   &bytesRead, portMAX_DELAY);
  if (err != ESP_OK || bytesRead == 0) { n = 0; return false; }

  n = bytesRead / sizeof(int32_t);
  if (n > 256) n = 256;

  static float dc = 0;
  static bool dcInit = false;
  uint64_t sumSq = 0, dSumSq = 0, hSumSq = 0, lSumSq = 0;
  int32_t peak = 0, dPeak = 0, hPeak = 0;

  // Detection high-pass: two cascaded one-pole stages.
  static float h1x = 0, h1y = 0, h2x = 0, h2y = 0;
  static bool hInit = false;
  // High band (HF_HP_HZ, 2 one-pole high-pass stages) and low band (LF_LP_HZ, 2 one-pole low-pass stages)
  static float f1x = 0, f1y = 0, f2x = 0, f2y = 0, lp1 = 0, lp2 = 0;
  static float l1x = 0, l1y = 0, l2x = 0, l2y = 0;      // 20 Hz high-pass on the low band
  const float lkH = 1.0f / (1.0f + 2.0f * (float)M_PI * LF_HP_HZ / (float)SAMPLE_RATE);
  const float hkH = 1.0f / (1.0f + 2.0f * (float)M_PI * HF_HP_HZ / (float)SAMPLE_RATE);
  const float aL  = 1.0f - expf(-2.0f * (float)M_PI * LF_LP_HZ / (float)SAMPLE_RATE);
  const float hk = 1.0f / (1.0f + 2.0f * (float)M_PI * DET_HP_HZ / (float)SAMPLE_RATE);

  for (size_t i = 0; i < n; i++) {
    float s = (float)(rawSamples[i] >> 16);       // same conversion as before
    if (!dcInit) { dc = s; dcInit = true; }
    dc += (s - dc) * DC_ALPHA;                    // one-pole DC blocker
    float x = s - dc;

    int32_t xi = (int32_t)x;
    sumSq += (uint64_t)((int64_t)xi * xi);        // RMS and peak: un-gained
    int32_t a = xi < 0 ? -xi : xi;
    if (a > peak) peak = a;

    if (!hInit) { h1x = x; f1x = x; lp1 = x; lp2 = x; l1x = x; hInit = true; }
    float y1 = hk * (h1y + x - h1x);   h1x = x;  h1y = y1;
    float y2 = hk * (h2y + y1 - h2x);  h2x = y1; h2y = y2;
    int32_t yi = (int32_t)y2;
    dSumSq += (uint64_t)((int64_t)yi * yi);
    int32_t da = yi < 0 ? -yi : yi;
    if (da > dPeak) dPeak = da;

    float g1 = hkH * (f1y + x - f1x);   f1x = x;   f1y = g1;
    float g2 = hkH * (f2y + g1 - f2x);  f2x = g1;  f2y = g2;
    int32_t hi = (int32_t)g2;
    hSumSq += (uint64_t)((int64_t)hi * hi);
    int32_t ha = hi < 0 ? -hi : hi;
    if (ha > hPeak) hPeak = ha;

    lp1 += aL * (x - lp1);
    lp2 += aL * (lp1 - lp2);
    float q1 = lkH * (l1y + lp2 - l1x);  l1x = lp2;  l1y = q1;
    float q2 = lkH * (l2y + q1 - l2x);   l2x = q1;   l2y = q2;
    int32_t li = (int32_t)q2;
    lSumSq += (uint64_t)((int64_t)li * li);

    float g = x * WRITE_GAIN;                     // gain applies to written audio only
    if (g > 32767.0f)  g = 32767.0f;
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

// =====================================================
// NETWORK (retry-safe)
// =====================================================

// Runs in the lwIP task: just set a flag, print from loop().
void onTimeSync(struct timeval* tv) {
  ntpConfirmed = true;
  ntpSyncFlag = true;
}

void onWifiUp() {
  if (!timeConfigured) {
    sntp_set_sync_interval(NTP_SYNC_MS);
    sntp_set_time_sync_notification_cb(onTimeSync);
    // configTime(0,0,...) would reset TZ to UTC; configTzTime keeps the Indianapolis rule.
    configTzTime(TZ_INFO, "pool.ntp.org", "time.nist.gov");
    timeConfigured = true;
  }
}

// /wifi.csv: "ok" rows (signal strength) while connected; "down", "up", "rejoin", "restart"
// rows around outages. down_s is the outage length where it applies.
void wifiLog(const char* ev, uint32_t downSec) {
  if (freeBytes() < FS_RESERVE_BYTES / 2) return;
  char ts[24];
  if (!localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S"))
    snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(millis() / 1000));
  bool isNew = !LittleFS.exists("/wifi.csv");
  File f = LittleFS.open("/wifi.csv", FILE_APPEND);
  if (!f) return;
  if (isNew) f.println("time,event,rssi_dbm,down_s");
  f.printf("%s,%s,%d,%lu\n", ts, ev,
           WiFi.status() == WL_CONNECTED ? (int)WiFi.RSSI() : 0, (unsigned long)downSec);
  f.close();
}

// /house.csv: what housekeeping did (ghost clips removed, stale-clip restarts).
void houseLog(const char* ev, const char* detail) {
  if (freeBytes() < FS_RESERVE_BYTES / 2) return;
  char ts[24];
  if (!localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S"))
    snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(millis() / 1000));
  fsTake();                                    // v2.2c: called from both cores
  bool isNew = !LittleFS.exists("/house.csv");
  File f = LittleFS.open("/house.csv", FILE_APPEND);
  if (f) {
    if (isNew) f.println("time,event,detail");
    f.printf("%s,%s,%s\n", ts, ev, detail);
    f.close();
  }
  fsGive();
}

// Clip names start with YYYYMMDD_HHMMSS (local time). False for names without a clock stamp.
bool parseClipEpoch(const char* nm, time_t* out) {
  int Y, M, D, h, m, s;
  if (sscanf(nm, "%4d%2d%2d_%2d%2d%2d", &Y, &M, &D, &h, &m, &s) != 6) return false;
  struct tm t = {};
  t.tm_year = Y - 1900; t.tm_mon = M - 1; t.tm_mday = D;
  t.tm_hour = h; t.tm_min = m; t.tm_sec = s; t.tm_isdst = -1;
  *out = mktime(&t);
  return true;
}

// Once a minute while idle: remove ghost (0-byte) clips; restart if clips are not being collected.
void housekeeping(uint32_t now) {
  static uint32_t last = 0, lastFailLog = 0;
  if (last && now - last < HOUSEKEEP_MS) return;
  last = now ? now : 1;

  time_t nowT = time(nullptr);
  bool timeOk = nowT > 1700000000L;                       // clock has been set by NTP

  File dir = LittleFS.open("/clips");
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
        bool old = hasT ? (timeOk && nowT - tc > (time_t)GHOST_AGE_S) : true;   // no stamp: left from a boot without a clock
        if (old && ng < 6) { strncpy(ghosts[ng], nm, sizeof(ghosts[0]) - 1); ghosts[ng][sizeof(ghosts[0]) - 1] = 0; ng++; }
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
      Serial.printf("Housekeeping: removed 0-byte clip %s\n", ghosts[i]);
      houseLog("ghost_removed", ghosts[i]);
    } else if (!lastFailLog || now - lastFailLog > 600000UL) {   // do not repeat every minute
      lastFailLog = now ? now : 1;
      Serial.printf("Housekeeping: cannot remove %s (reboot may free it)\n", ghosts[i]);
      houseLog("ghost_remove_failed", ghosts[i]);
    }
  }

  if (!DELETE_OLDEST_WHEN_FULL && timeOk && oldest && nowT - oldest > (time_t)STALE_CLIP_S && now > STALE_RESTART_MIN_UPTIME_MS) {
    char d[32];
    snprintf(d, sizeof(d), "oldest_clip_%lu_min", (unsigned long)((nowT - oldest) / 60));
    houseLog("stale_restart", d);
    Serial.printf("Housekeeping: clips not collected (%s); restarting\n", d);
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
    if (now - lastOk >= WIFI_LOG_MS) { lastOk = now; wifiLog("ok", 0); }
    return;
  }

  // Link is down. Plain reconnect first; a full rejoin, then a restart, if it stays down.
  if (!wifiDownSince) { wifiDownSince = now ? now : 1; wifiLog("down", 0); }
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
    WiFi.disconnect(false);          // drop the association but keep the radio on
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  } else {
    WiFi.reconnect();
  }
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  fsLock = xSemaphoreCreateRecursiveMutex();   // v2.2c
  setenv("TZ", TZ_INFO, 1);     // local time from the first second, even before Wi-Fi
  tzset();
  Serial.println("\nESP32 NIGHT SOUND RECORDER v2.2d (LittleFS + HTTP upload)");

  // Never auto-format unless FORMAT_IF_MOUNT_FAILS is set: protects existing recordings.

  if (!LittleFS.begin(FORMAT_IF_MOUNT_FAILS))  {
  //if (!LittleFS.begin(FORMAT_IF_MOUNT_FAILS) == true)  {
    Serial.println("FATAL: LittleFS mount failed; filesystem NOT formatted.");
    Serial.println("After a partition-scheme change, set FORMAT_IF_MOUNT_FAILS true once.");
    while (true) delay(1000);
  }
  LittleFS.mkdir("/clips");
  retireOldCsv("/events.csv", "trigger_by", "/events_v21c.csv");
  retireOldCsv("/watch.csv",  "hf_rms",     "/watch_v21c.csv");
  Serial.printf("LittleFS: %lu bytes total, %lu used, %lu free\n",
                (unsigned long)LittleFS.totalBytes(),
                (unsigned long)LittleFS.usedBytes(),
                (unsigned long)freeBytes());

  listDir("/");
  listDir("/clips");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Wi-Fi connected: ");
    Serial.println(WiFi.localIP());
    onWifiUp();
    struct tm ti;
    Serial.println(getLocalTime(&ti, 15000) ? "NTP synchronized." : "NTP not ready yet (will keep trying).");
  } else {
    Serial.println("Wi-Fi unavailable; recording continues, retry every 30 s.");
  }

  if (!initMicrophone()) {
    Serial.println("FATAL: microphone init failed.");
    while (true) delay(1000);
  }

  warmupEnd = millis() + WARMUP_MS;
  Serial.printf("Floor trigger %lu (high-passed %.0f Hz), ratio %.1f, clip cap %lu ms, pre-roll %lu samples\n",
                (unsigned long)MIN_TRIGGER_RMS, DET_HP_HZ, TRIGGER_RATIO,
                (unsigned long)CLIP_MAX_MS, (unsigned long)PREROLL_SAMPLES);

  startUploader();              // v2.2: HTTP uploader task (nr_upload.h)
}

void loop() {
  uint32_t now = millis();

  serviceNetwork(now);

  if (ntpSyncFlag) {
    ntpSyncFlag = false;
    char s[24];
    if (localTimeStr(s, sizeof(s), "%Y-%m-%d %H:%M:%S")) Serial.printf("NTP sync: %s\n", s);
  }

  // One row per start: when the clock is valid (or after 60 s), log why we restarted.
  if (!bootLogged) {
    char ts[24];
    bool haveTime = localTimeStr(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S");
    if (haveTime || millis() > 60000UL) {
      if (!haveTime) snprintf(ts, sizeof(ts), "boot+%lus", (unsigned long)(millis() / 1000));
      bootLogged = true;
      bool isNew = !LittleFS.exists("/boot.csv");
      File f = LittleFS.open("/boot.csv", FILE_APPEND);
      if (f) {
        if (isNew) f.println("time,reset_reason,code,free_heap");
        f.printf("%s,%s,%d,%lu\n", ts, resetReasonStr(esp_reset_reason()),
                 (int)esp_reset_reason(), (unsigned long)ESP.getFreeHeap());
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
  hvacTick(n, now);                // v2.2c: low-frequency plateau log
  heapTick(now);

  if (eventActive) {
    trackEvent(n, now);
    return;
  }

  prPush(pcmSamples, n);          // includes the current chunk
  if (now < warmupEnd) return;

  static uint32_t lastReport = 0;
  if (now - lastReport >= 1000) {
    lastReport = now;
    Serial.printf("RMS_hp=%lu raw=%lu base=%lu trig=%lu | hf=%lu hfbase=%lu hftrig=%lu lf=%lu\n",
                  (unsigned long)detRms, (unsigned long)currentRms, (unsigned long)baseline,
                  (unsigned long)triggerLevel(), (unsigned long)hfRms,
                  (unsigned long)hfBaseline, (unsigned long)hfTriggerLevel(),
                  (unsigned long)lfRms);
  }
  bool trigBB   = detRms >= triggerLevel();
  bool trigPeak = detPeak >= PEAK_TRIGGER;
  bool trigHF   = hfRms >= hfTriggerLevel();
  bool trigLF   = lfRms >= lfTriggerLevel();

  if (trigBB || trigPeak || trigHF || trigLF) {
    
    // --- DESIGN SPEC: LOCKOUT suppression WINDOW ON CORE TRIGGERS ---
    if (now - lastEventEndMillis < EVENT_LOCKOUT_MS) {
      aboveCount = 0; // Clear out the debounce counter history
      return;         // Exit immediately to blind the trigger logic
    }
    // -----------------------------------------------------------------

    if (aboveCount == 0) {
      dbPeak = dbRawPeak = 0; dbRms = dbRawRms = 0;
      dbHfRms = dbLfRms = 0;  dbHfPeak = 0;
      dbTrigBy = trigPeak ? "peak" : (trigBB ? "bb" : (trigHF ? "hf" : "lf"));   // what fired first
    }
    if (detPeak    > dbPeak)    dbPeak    = detPeak;
    if (chunkPeak  > dbRawPeak) dbRawPeak = chunkPeak;
    if (detRms     > dbRms)     dbRms     = detRms;
    if (currentRms > dbRawRms)  dbRawRms  = currentRms;
    if (hfRms      > dbHfRms)   dbHfRms   = hfRms;
    if (hfPeak     > dbHfPeak)  dbHfPeak  = hfPeak;
    if (lfRms      > dbLfRms)   dbLfRms   = lfRms;
    // A peak trigger starts at once; band-only triggers (high or low) need 2 chunks; broadband needs 3.
    uint8_t need = trigPeak ? 1 : (trigBB ? DEBOUNCE_CHUNKS : (trigHF ? HF_DEBOUNCE_CHUNKS : LF_DEBOUNCE_CHUNKS));
    if (++aboveCount >= need) startEvent(now);
  } else {
    aboveCount = 0;
  }
}

#include "nr_upload.h"