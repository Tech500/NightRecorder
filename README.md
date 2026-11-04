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
