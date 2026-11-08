"""nr_receive.py - Transaction ledger + WAV header repair + duplicate handling"""
import datetime
import os
import re
import struct
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BASE = os.environ.get("NR_BASE", r"M:\NightRecorder-II")   # Target volume directory
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

def fix_wav(buf):
    """Return (wav_bytes, notes). Rebuilds the 44-byte header from the real data length and
    zeroes any stray 44-byte WAV header that landed inside the audio (seen at 4096-byte
    boundaries on firmware <= v2.6a). Accepts clips with or without a leading header."""
    notes = []
    rate = 16000
    if len(buf) >= 44 and buf[:4] == b"RIFF" and buf[8:16] == b"WAVEfmt ":
        r = struct.unpack_from("<I", buf, 24)[0]
        if 8000 <= r <= 48000:
            rate = r
        pcm = bytearray(buf[44:])
    else:
        pcm = bytearray(buf)          # raw PCM (header-less firmware)
        notes.append("no header; added")
    i = pcm.find(b"RIFF")
    while i != -1:
        if pcm[i + 8:i + 16] == b"WAVEfmt " and pcm[i + 36:i + 40] == b"data":
            pcm[i:i + 44] = bytes(44)
            notes.append("stray header zeroed at file offset %d" % (i + 44))
        i = pcm.find(b"RIFF", i + 4)
    pcm = pcm[:len(pcm) & ~1]
    n = len(pcm)
    hdr = (b"RIFF" + struct.pack("<I", n + 36) + b"WAVEfmt " + struct.pack("<IHHIIHH", 16, 1, 1, rate, rate * 2, 2, 16)
           + b"data" + struct.pack("<I", n))
    return bytes(hdr) + bytes(pcm), notes

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
            out = bytes(buf)
            if name.lower().endswith(".wav"):
                out, fixes = fix_wav(bytes(buf))
                for fx in fixes:
                    note("FIX %s: %s" % (name, fx))
            if name.lower().endswith(".wav") and os.path.exists(dest):
                # Already stored (duplicate POST / retry after a lost reply): ack so the ESP32 purges it.
                note("DUP %s already stored; acked, not rewritten (from %s)" % (name, ip))
                return self.reply(200, "ok %d" % n)
            tmp = dest + ".part"
            with open(tmp, "wb") as f:
                f.write(out)
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
