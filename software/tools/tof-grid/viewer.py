#!/usr/bin/python3
"""Live 8x8 grid view for the VL53L5CX tof-grid firmware.

Reads frames from the ESP32-S3 over USB serial and serves a page at
http://localhost:8766 that updates in real time (Server-Sent Events).

    /usr/bin/python3 viewer.py [--port /dev/ttyACM0] [--http 8766] [--no-browser]
"""
import argparse
import glob
import json
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

PAGE = Path(__file__).with_name("viewer.html")


class Hub:
    """Holds the latest frame and wakes every SSE client when a new one lands."""

    def __init__(self):
        self.cond = threading.Condition()
        self.seq = 0
        self.msg = json.dumps({"type": "status", "text": "waiting for serial..."})

    def publish(self, obj):
        with self.cond:
            self.seq += 1
            self.msg = json.dumps(obj)
            self.cond.notify_all()

    def wait(self, last_seq, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != last_seq, timeout)
            return self.seq, self.msg


hub = Hub()


def find_port():
    ports = sorted(glob.glob("/dev/serial/by-id/usb-Espressif*")) or sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


def parse_frame(line):
    dist, status = line[1:].split("|")
    d = [int(x) for x in dist.split()]
    s = [int(x) for x in status.split()]
    if len(d) != 64 or len(s) != 64:
        raise ValueError("bad frame length")
    return d, s


def serial_loop(port_arg):
    while True:
        port = port_arg or find_port()
        if not port:
            hub.publish({"type": "status", "text": "no ESP32 serial port found - is it plugged in?"})
            time.sleep(1)
            continue
        try:
            # dtr/rts low so opening the port doesn't reset the board
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout = port, 115200, 1
            ser.dtr = ser.rts = False
            ser.open()
            hub.publish({"type": "status", "text": f"connected to {port}"})
            stamps = []
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if line.startswith("F "):
                    try:
                        d, s = parse_frame(line)
                    except ValueError:
                        continue
                    now = time.time()
                    stamps = [t for t in stamps if now - t < 2] + [now]
                    hz = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0
                    hub.publish({"type": "frame", "d": d, "s": s, "hz": round(hz, 1)})
                elif line.startswith("#"):
                    hub.publish({"type": "status", "text": line[1:].strip()})
        except (serial.SerialException, OSError) as e:
            hub.publish({"type": "status", "text": f"serial error: {e} - retrying"})
            time.sleep(1)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            body = PAGE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            seq = -1
            try:
                while True:
                    new_seq, msg = hub.wait(seq)
                    if new_seq == seq:
                        self.wfile.write(b": keepalive\n\n")
                    else:
                        seq = new_seq
                        self.wfile.write(f"data: {msg}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect Espressif USB)")
    ap.add_argument("--http", type=int, default=8766, help="HTTP port (default 8766)")
    ap.add_argument("--no-browser", action="store_true", help="don't open a browser tab")
    args = ap.parse_args()

    threading.Thread(target=serial_loop, args=(args.port,), daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    server.daemon_threads = True
    url = f"http://localhost:{args.http}"
    print(f"tof-grid viewer at {url}  (Ctrl+C to quit)")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
