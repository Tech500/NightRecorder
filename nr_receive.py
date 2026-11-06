"""nr_receive.py - Patched with Transaction Ledger Logging"""
import datetime
import os
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BASE = r"M:\NightRecorder-II"   # Target volume directory
PORT = 8082
NAME_OK = re.compile(r"^[A-Za-z0-9_.-]{1,80}$")
WAV_RE = re.compile(r"^(\d{8})_(\d{6})_")
MAX_BYTES = 1_000_000
BODY_TIMEOUT = 15                 # Dropped to 15s to dump stalled connections cleanly

def night_folder(name):
    m = WAV_RE.match(name)
    if not m:
        return "unsorted"
    d = datetime.datetime.strptime(m.group(1) + m.group(2), "%Y%m%d%H%M%S")
    if d.hour >= 12:                       
        d += datetime.timedelta(days=1)
    return d.strftime("%m-%d-%Y")

def note(text):
    try:
        with open(os.path.join(BASE, "receiver.log"), "a") as f:
            f.write("%s %s\n" % (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), text))
    except OSError:
        pass

class Handler(BaseHTTPRequestHandler):
    timeout = BODY_TIMEOUT

    def log_message(self, fmt, *args):
        note(fmt % args)

    def reply(self, code, text):
        body = text.encode()
        self.send_response(code)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        t0 = time.time()
        name = self.headers.get("X-Filename", "")
        ip = self.client_address[0]
        got = 0
        n = 0
        try:
            n = int(self.headers.get("Content-Length", "0") or 0)
            if not NAME_OK.match(name) or not name.lower().endswith((".wav", ".csv")):
                note("REJECT bad filename %r from %s" % (name, ip))
                return self.reply(400, "bad filename")
            if n <= 0 or n > MAX_BYTES:
                note("REJECT bad size %d for %s from %s" % (n, name, ip))
                return self.reply(400, "bad size")

            buf = bytearray()
            try:
                while len(buf) < n:
                    chunk = self.rfile.read1(min(65536, n - len(buf)))
                    if not chunk:                      
                        break
                    buf += chunk
            except (TimeoutError, OSError) as e:
                got = len(buf)
                note("STALL %s declared=%d got=%d after %.1fs from %s (%r)"
                     % (name, n, got, time.time() - t0, ip, e))
                return self.reply(500, "stall")
            got = len(buf)
            if got != n:
                note("SHORT %s declared=%d got=%d after %.1fs from %s" % (name, n, got, time.time() - t0, ip))
                return self.reply(400, "short body")

            sub = night_folder(name) if name.lower().endswith(".wav") else "csv"
            folder = os.path.join(BASE, sub)
            os.makedirs(folder, exist_ok=True)
            dest = os.path.join(folder, name)
            tmp = dest + ".part"
            with open(tmp, "wb") as f:
                f.write(buf)
            os.replace(tmp, dest)          
            self.reply(200, "ok %d" % n)   
            
            # --- DESIGN SPEC: REAL-TIME TRANSACTION PURGE LEDGER ---
            with open(os.path.join(BASE, "file_transfer_ledger.log"), "a") as ledger:
                ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                ledger.write(f"[{ts}] PURGED_FROM_ESP32_OK -> Name: {name} | Size: {n} Bytes | Source: {ip}\n")
                
            note("OK %s %d B in %.2fs from %s" % (name, n, time.time() - t0, ip))
        except Exception as e:
            note("ERROR %s declared=%d got=%d from %s: %r" % (name, n, got, ip, e))
            try:
                self.reply(500, "error")
            except OSError:
                pass

if __name__ == "__main__":
    os.makedirs(BASE, exist_ok=True)
    note("receiver started on port %d" % PORT)
    print("Listening on port", PORT)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
