r"""nr_receive.py - receives clips and CSVs POSTed by the Night Sound Recorder ESP32.

Run:  python M:\NightRecorder-II\nr_receive.py
Test: curl.exe -X POST -H "X-Filename: 20261104_220000_0001.wav" --data-binary @anyfile.wav http://127.0.0.1:8082/upload
"""
import datetime
import os
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BASE = r"M:\NightRecorder-II"   # set to the same root nr_pull.py saves under
PORT = 8082
NAME_OK = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
WAV_RE = re.compile(r"^(\d{8})_(\d{6})_")
MAX_BYTES = 1_000_000


def night_folder(name):
    """Clip taken the night of 11-03 (evening or after midnight) -> folder 11-04-2026."""
    m = WAV_RE.match(name)
    if not m:
        return "unsorted"
    d = datetime.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
    if d.hour >= 12:                       # evening clip: night ends next morning
        d += datetime.timedelta(days=1)
    return d.strftime("%m-%d-%Y")


class Handler(BaseHTTPRequestHandler):
    timeout = 30                           # drop stalled uploads

    def log_message(self, fmt, *args):
        try:
            with open(os.path.join(BASE, "receiver.log"), "a") as f:
                f.write("%s %s\n" % (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), fmt % args))
        except OSError:
            pass

    def reply(self, code, text):
        body = text.encode()
        self.send_response(code)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        try:
            name = self.headers.get("X-Filename", "")
            n = int(self.headers.get("Content-Length", "0") or 0)
            if not NAME_OK.match(name) or not name.lower().endswith((".wav", ".csv")):
                return self.reply(400, "bad filename")
            if n <= 0 or n > MAX_BYTES:
                return self.reply(400, "bad size")
            data = self.rfile.read(n)
            if len(data) != n:
                return self.reply(400, "short body")

            sub = night_folder(name) if name.lower().endswith(".wav") else "csv"
            folder = os.path.join(BASE, sub)
            os.makedirs(folder, exist_ok=True)
            dest = os.path.join(folder, name)
            tmp = dest + ".part"
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, dest)          # a partial file never has the final name
            self.reply(200, "ok %d" % n)   # ESP32 deletes the clip only on this reply
        except Exception as e:
            self.log_message("error %r", e)
            try:
                self.reply(500, "error")
            except OSError:
                pass


if __name__ == "__main__":
    os.makedirs(BASE, exist_ok=True)
    print("Listening on port", PORT)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
