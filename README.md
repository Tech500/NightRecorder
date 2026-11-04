# NightRecorder

An ESP32 night sound-event recorder. An INMP441 I2S microphone listens all night; when a sound rises above an adaptive threshold, the ESP32 saves a short WAV clip (with about one second of pre-roll) to its flash and pushes it over Wi-Fi to a small receiver on a PC. The PC acknowledges each file, and only then does the ESP32 delete its copy.

Current version: **v2.2** (LittleFS + HTTP upload). Version 2.1 and earlier served the clips over FTP; see [Why HTTP upload](#why-http-upload).

## How it works

- **Capture:** INMP441 at 16 kHz, 16-bit mono. Level is tracked with high-pass filtered RMS plus separate high-frequency and low-frequency bands.
- **Event detection:** the trigger is an adaptive baseline times a ratio (6.0), with a floor of 300 on the high-passed RMS. An event ends after 1.5 s below the release level, and is force-ended after 5 minutes.
- **Clips:** about 1 s of pre-roll plus the event, capped at 5 s per clip (roughly 160 KB). Files are named `YYYYMMDD_HHMMSS_NNNN.wav`, with `_loud` or `_very_loud` added when the raw peak reaches 3000 or 5000.
- **Recording hours:** clips are written only between `CLIP_FROM_HOUR` (20:00) and `CLIP_UNTIL_HOUR` (08:00), local time. Events outside those hours are still logged to `events.csv` but no audio is saved.
- **Storage:** LittleFS on a 2 MB partition. If space runs out, the oldest routine clip is deleted first (rolling buffer), so loud clips survive longest.
- **Upload:** a separate FreeRTOS task on core 0 sends each clip with an HTTP POST, so the microphone loop never waits on the network. The PC replies `ok <bytes>`, and the clip is deleted only if the byte count matches. Failed attempts back off from 20 s up to 120 s. CSV logs are sent once an hour. If clips have been waiting and nothing has gone out for 2 hours, the board restarts itself.
- **Logs on the board:** `events.csv`, `heap.csv`, `house.csv`, `watch.csv`, `wifi.csv`, `boot.csv` and daily `levels_YYYYMMDD.csv`.

## Why record sleep sounds (experimental use)

The recorder itself is working and in use. Using it for sleep-related questions, described in this section, is experimental.

The project started as a way to find out what is making noise during the night. It can also help someone who wants a record of nighttime sounds when sleep is fragmented, for example when sleep apnea is suspected or being treated:

- **A timeline, not just audio.** Every detected event is logged to `events.csv` with its start time, duration, peak and RMS level, and frequency-band levels. Plotted over a night, this shows when sounds cluster, how long the quiet gaps between them are, and how loud they are.
- **Recognisable sounds.** Loud snoring, gasping or choking sounds, and loud breathing after a pause tend to stand out as events. Clips are tagged `_loud` and `_very_loud` by peak level so they are easy to find without opening every file.
- **Context for a clinician.** A set of timestamped clips and a night-by-night event count can be shown to a doctor or sleep specialist alongside, not instead of, a proper sleep study.
- **Other noise sources.** The same log helps separate sleep-related sounds from a furnace, plumbing, neighbours or street noise that may be waking someone.

### Comparing with CPAP data (OSCAR) — experimental

> **Experimental.** This comparison is an idea being tried, not a tested method. The OSCAR export formats and the time alignment described below have not been verified end to end, and no results are claimed. Treat anything it shows as a prompt for questions, not as a finding.

For someone on CPAP therapy, the recorder gives a second, independent record of the same night. [OSCAR](https://www.sleepfiles.com/OSCAR/) (Open Source CPAP Analysis Reporter) reads the data a CPAP machine stores on its SD card and shows the machine's own timestamped events: apneas, hypopneas, flow limitation, mask leak and snore. The recorder's `events.csv` and clip file names carry the room's timestamps for the same night.

Laid side by side, the two records can show:

- **Sound with a machine event:** a loud sound at the same minute as a flagged apnea, a leak or a snore burst. Clips from that minute show what the sound actually was.
- **Sound with no machine event:** a bang or a furnace cycle that coincides with a waking, which points away from breathing as the cause.
- **A machine event with no sound:** a leak or arousal that was quiet in the room.
- **Mask-off periods:** gaps in the CPAP session line up with the recorder's log of when sounds continued in the room.

To compare them:

1. Export the night from OSCAR (its CSV export covers events and summaries), and copy `events.csv` from the receiver's `csv` folder.
2. Line both up on time. The ESP32 clock is NTP-synchronised and set to your time zone, but CPAP machines keep their own clock, and it can be minutes off. Find one sound you can identify on both records (a cough or a deliberate loud tap), and use it to measure the offset before comparing anything else.
3. Compare in minutes rather than seconds, because the clocks and the two time resolutions are different.

This is a way to look at correlations. It does not show cause, and it does not replace the interpretation of a sleep clinician or the machine's own reports.

What it cannot do:

- It is **not a medical device** and does not diagnose sleep apnea, measure breathing, oxygen saturation or sleep stages, or detect pauses in breathing. A pause in breathing is silence, and silence is not recorded as an event.
- A microphone alone cannot tell you that sleep was fragmented. It only shows that sound occurred at certain times.
- The threshold is tuned for a quiet room, so soft sounds may be missed and a noisy room may produce many events. Tune the trigger settings to your own room.
- Recordings of people sleeping are personal data. Only record people who know about it, and keep the clips on a machine you control.

If you suspect sleep apnea, talk to a doctor. A sleep study is the reliable test.

## Hardware

| Part | Notes |
|---|---|
| ESP32 DevKit V1 (DOIT) | Arduino-ESP32 core 3.3.x |
| INMP441 I2S MEMS microphone | |

Wiring:

| INMP441 | ESP32 |
|---|---|
| VDD | 3.3 V |
| GND | GND |
| L/R | GND |
| SCK | GPIO 26 |
| WS | GPIO 25 |
| SD | GPIO 4 |

## Files

| File | Where it runs | Purpose |
|---|---|---|
| `NightRecorder_v2_2.ino` | ESP32 | Main sketch |
| `nr_upload.h` | ESP32 | HTTP uploader task, included by the sketch |
| `secrets.h` | ESP32 | Wi-Fi credentials (not in the repo) |
| `nr_receive.py` | PC | Receives and files the uploads |

## ESP32 setup

1. Put `NightRecorder_v2_2.ino` in a folder named `NightRecorder_v2_2`, next to `nr_upload.h` and your own `secrets.h`:
   ```cpp
   const char* WIFI_SSID     = "your-ssid";
   const char* WIFI_PASSWORD = "your-password";
   ```
   `secrets.h` is in `.gitignore`; do not commit it.
2. In `nr_upload.h`, set `UPLOAD_HOST` to the PC's address. Reserve that address in your router, because it is hard-coded.
3. In the Arduino IDE, choose the partition scheme **No OTA (2MB APP/2MB SPIFFS)**. Changing it wipes the old filesystem. On the first boot the mount may fail: set `FORMAT_IF_MOUNT_FAILS` to `true`, upload and run once, then set it back to `false`.
4. Adjust the time zone string `TZ_INFO` and the recording hours for your location.
5. Upload and open the serial monitor at the sketch's baud rate. You should see `ESP32 NIGHT SOUND RECORDER v2.2` and `UPLOAD task started`.

## PC receiver setup

Requires Python 3 on Windows, with no extra packages.

1. Set `BASE` in `nr_receive.py` to the folder where files should land (default `M:\NightRecorder-II`).
2. Allow the port through the Windows firewall (admin prompt). Use `profile=any` if Windows labels your network Public:
   ```
   netsh advfirewall firewall add rule name="NightRecorder 8082" dir=in action=allow protocol=TCP localport=8082 profile=private
   ```
3. Run it:
   ```
   python M:\NightRecorder-II\nr_receive.py
   ```
4. Test locally. This test does not exercise the firewall:
   ```
   curl.exe -X POST -H "X-Filename: 20261104_220000_0001.wav" --data-binary @anyfile.wav http://127.0.0.1:8082/upload
   ```
   The reply is `ok <bytes>`.

Received files are organised like this:

- Clips go into a folder named for the morning the night ends on (`MM-DD-YYYY`). A clip taken at 22:00 on 11-04 lands in `11-05-2026`.
- CSVs go into `csv`, overwritten on each upload.
- Each file is written as `.part` and renamed when complete, so a partial file never has its final name.
- Requests are logged to `receiver.log`.

The receiver accepts only `.wav` and `.csv` names matching `[A-Za-z0-9_.-]{1,80}` and files up to 1 MB. It has no authentication, so run it only on a trusted LAN.

## Troubleshooting

- **`UPLOAD ... -> -1 FAILED`:** the ESP32 could not open a connection. The usual cause is the PC's firewall: a rule with `profile=private` does not apply on a Public network, and a Block rule for `python.exe` overrides an Allow rule. A `curl` test from the PC to its own LAN address does not pass through the firewall, so it cannot catch this.
- **Clips never appear but events are detected:** check the recording hours, and check that the serial monitor shows `UPLOAD found N clip(s) waiting`.
- **Nothing uploads after a PC reboot:** `nr_receive.py` must be running. Register it as a Scheduled Task that starts at logon; clips wait safely on the ESP32 until it returns.

## Why HTTP upload

Through v2.1i the ESP32 ran an FTP server and a PC script pulled the clips every 15 minutes. On 2026-11-02 the server's data connections wedged: directory listings still worked, but every file transfer timed out, and only a power cycle cleared it. About 35 hours of clips rolled off the full flash before the fault was found. v2.2 removes the FTP server entirely. The ESP32 now makes outbound connections only, nothing on the board listens, and a clip is deleted only after the PC confirms receipt.

## Known limitations

- The CSV logs on the board are uploaded but not truncated, so they keep growing and slowly use up flash.
- The root cause of the FTP wedge (heap fragmentation, lwIP or Wi-Fi) was not confirmed. Heap and RSSI logging and the 2-hour restart backstop stay in place.
- If the rolling buffer deletes a clip while it is being uploaded, that upload fails and is not retried (rare).
- While the PC is off, clips accumulate on the ESP32 until flash fills and the oldest are overwritten.

## License

MIT. See [LICENSE](LICENSE).
