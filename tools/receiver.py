#!/usr/bin/env python3
"""Tiny upload receiver for the R2S Capture app (stdlib only).

  python receiver.py --root ~/captures --port 8765 [--token SECRET] [--convert OUT_DIR]

In the app: Settings > Upload receiver = http://<this-computer-ip>:8765#SECRET
The app PUTs every file to /upload/<session>/<path> (HEAD first, so restarted uploads skip finished
files), then PUTs /upload/<session>/.complete. With --convert, the receiver then runs convert.py on
the finished session in the background.

Only listen on a trusted network, or put it behind an SSH tunnel / reverse proxy. Use --token.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

HERE = Path(__file__).resolve().parent


def make_handler(root: Path, token: str | None, convert_out: Path | None):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _target(self):
            if token and self.headers.get("X-R2S-Token") != token:
                self.send_error(403, "bad token")
                return None
            path = unquote(self.path.split("?")[0])
            if not path.startswith("/upload/"):
                self.send_error(404)
                return None
            rel = Path(path[len("/upload/"):])
            if rel.is_absolute() or ".." in rel.parts or len(rel.parts) < 2:
                self.send_error(400, "bad path")
                return None
            return root / rel

        def do_HEAD(self):
            t = self._target()
            if t is None:
                return
            if t.is_file():
                self.send_response(200)
                self.send_header("Content-Length", str(t.stat().st_size))
            else:
                self.send_response(404)
                self.send_header("Content-Length", "0")
            self.end_headers()

        def do_PUT(self):
            t = self._target()
            if t is None:
                return
            n = int(self.headers.get("Content-Length", "0"))
            t.parent.mkdir(parents=True, exist_ok=True)
            tmp = t.with_name(t.name + ".part")
            left = n
            with open(tmp, "wb") as fh:
                while left > 0:
                    chunk = self.rfile.read(min(left, 1 << 20))
                    if not chunk:
                        break
                    fh.write(chunk)
                    left -= len(chunk)
            if left:
                self.send_error(400, "short body")
                return
            os.replace(tmp, t)
            self.send_response(201)
            self.send_header("Content-Length", "0")
            self.end_headers()
            self.log_message("stored %s (%d bytes)", t.relative_to(root), n)
            if t.name == ".complete" and convert_out is not None:
                sess = t.parent
                threading.Thread(target=run_convert, args=(sess, convert_out), daemon=True).start()

        def do_GET(self):
            body = ("r2s receiver OK\n" + "\n".join(sorted(p.name for p in root.iterdir() if p.is_dir())) + "\n").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return H


def run_convert(sess: Path, out: Path):
    log = sess / "convert.log"
    with open(log, "w") as fh:
        subprocess.run([sys.executable, str(HERE / "convert.py"), str(sess), "--out", str(out / sess.name)],
                       stdout=fh, stderr=subprocess.STDOUT)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=Path("captures"))
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--token", default=os.environ.get("R2S_TOKEN"))
    ap.add_argument("--convert", type=Path, default=None, help="run convert.py into this dir when a session completes")
    a = ap.parse_args()
    a.root.mkdir(parents=True, exist_ok=True)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(a.root.resolve(), a.token, a.convert))
    print(f"receiving into {a.root.resolve()} on http://{a.host}:{a.port} (token {'on' if a.token else 'OFF'})")
    srv.serve_forever()


if __name__ == "__main__":
    main()
