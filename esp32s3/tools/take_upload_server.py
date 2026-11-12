#!/usr/bin/env python3
"""Store a vest take in pieces and report the size on disk.

The board sends one file in order and deletes it from the card only when
`stored` equals the size on the card. A piece that was already stored is
answered again without writing it twice. A gap is refused so the board
retries from the byte count in the reply.

    POST /v1/takes/T0003/audio_00.wav
    Authorization: Bearer <token>
    X-Offset: 0
    <raw bytes>

    {"take": "T0003", "name": "audio_00.wav", "stored": 32768}

    GET /v1/takes
    GET /v1/takes/T0003
    GET /v1/takes/T0003/audio_00.wav?offset=0&n=32768
        the raw bytes, so a review app can pull the take. X-Stored is the whole file.

VEST_TAKES_DIR is where folders are written. VEST_TAKES_TOKEN, or the first
line of VEST_TAKES_TOKEN_FILE, is the bearer token. VEST_TAKES_PORT defaults
to 8780.
"""

from __future__ import annotations

import json
import os
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

TAKE_RE = re.compile(r"T[0-9]{4}")
NAME_RE = re.compile(r"audio_[0-9]{2}\.wav|video\.bin|info\.txt|receipt\.txt")
MAX_PIECE = 1024 * 1024

ROOT = Path(os.environ.get("VEST_TAKES_DIR", "/var/lib/vest-takes"))
PORT = int(os.environ.get("VEST_TAKES_PORT", "8780"))
_locks: dict[str, threading.Lock] = {}
_locks_guard = threading.Lock()


def token() -> str:
    direct = os.environ.get("VEST_TAKES_TOKEN", "").strip()
    if direct:
        return direct
    path = os.environ.get("VEST_TAKES_TOKEN_FILE", "")
    if path:
        return Path(path).read_text(encoding="utf-8").strip().splitlines()[0]
    raise SystemExit("set VEST_TAKES_TOKEN or VEST_TAKES_TOKEN_FILE")


TOKEN = token()


def file_lock(key: str) -> threading.Lock:
    with _locks_guard:
        return _locks.setdefault(key, threading.Lock())


def take_dir(take: str) -> Path:
    path = (ROOT / take).resolve()
    if path.parent != ROOT.resolve() or not TAKE_RE.fullmatch(take):
        raise ValueError(take)
    return path


class Handler(BaseHTTPRequestHandler):
    server_version = "vest-takes/1"
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args) -> None:
        print("%s %s" % (self.address_string(), fmt % args), flush=True)

    def _json(self, code: int, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Connection", "close")
        if "stored" in payload:
            self.send_header("X-Stored", str(payload["stored"]))
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self) -> bool:
        if self.headers.get("Authorization", "") == f"Bearer {TOKEN}":
            return True
        self._json(401, {"error": "unauthorized"})
        return False

    def _parts(self) -> tuple[str, str] | None:
        bits = [b for b in self.path.split("?", 1)[0].split("/") if b]
        if len(bits) == 3 and bits[0] == "v1" and bits[1] == "takes":
            return bits[2], ""
        if len(bits) == 4 and bits[0] == "v1" and bits[1] == "takes":
            take, name = bits[2], bits[3]
            if TAKE_RE.fullmatch(take) and NAME_RE.fullmatch(name):
                return take, name
        self._json(404, {"error": "not found"})
        return None

    def do_GET(self) -> None:  # noqa: N802
        if self.path.split("?", 1)[0] == "/health":
            self._json(200, {"ok": True})
            return
        if not self._authorized():
            return
        if self.path.split("?", 1)[0] == "/v1/takes":
            takes = []
            if ROOT.is_dir():
                for folder in sorted(p for p in ROOT.iterdir() if p.is_dir() and TAKE_RE.fullmatch(p.name)):
                    files = {f.name: f.stat().st_size for f in folder.iterdir() if f.is_file()}
                    takes.append({"take": folder.name, "files": files})
            self._json(200, {"takes": takes})
            return
        parsed = self._parts()
        if parsed is None:
            return
        take, name = parsed
        folder = take_dir(take)
        if not name:
            files = {}
            if folder.is_dir():
                files = {f.name: f.stat().st_size for f in folder.iterdir() if f.is_file()}
            self._json(200, {"take": take, "files": files})
            return
        path = folder / name
        query = parse_qs(urlsplit(self.path).query)
        if "n" in query:
            try:
                offset = int(query.get("offset", ["0"])[0])
                n = int(query["n"][0])
            except ValueError:
                self._json(400, {"error": "bad piece"})
                return
            if offset < 0 or n < 0 or n > MAX_PIECE or not path.is_file():
                self._json(404 if not path.is_file() else 400, {"error": "bad piece" if path.is_file() else "not found"})
                return
            with path.open("rb") as f:
                f.seek(offset)
                data = f.read(n)
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("X-Stored", str(path.stat().st_size))
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(data)
            return
        stored = path.stat().st_size if path.is_file() else 0
        self._json(200, {"take": take, "name": name, "stored": stored})

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized():
            return
        parsed = self._parts()
        if parsed is None:
            return
        take, name = parsed
        if not name:
            self._json(404, {"error": "not found"})
            return
        try:
            offset = int(self.headers.get("X-Offset", "0"))
        except ValueError:
            self._json(400, {"error": "bad offset"})
            return
        length = int(self.headers.get("Content-Length", "0"))
        if offset < 0 or length < 0 or length > MAX_PIECE:
            self._json(400, {"error": "bad piece"})
            return
        data = self.rfile.read(length)
        if len(data) != length:
            self._json(400, {"error": "short body"})
            return
        folder = take_dir(take)
        path = folder / name
        with file_lock(f"{take}/{name}"):
            folder.mkdir(parents=True, exist_ok=True)
            stored = path.stat().st_size if path.is_file() else 0
            if offset > stored:
                self._json(409, {"take": take, "name": name, "stored": stored, "error": "gap"})
                return
            if offset + length > stored:
                with path.open("r+b" if path.exists() else "wb") as out:
                    out.seek(offset)
                    out.write(data[stored - offset :])
                    out.truncate()
                stored = path.stat().st_size
        self._json(200, {"take": take, "name": name, "stored": stored})


def main() -> None:
    ROOT.mkdir(parents=True, exist_ok=True)
    httpd = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print(f"vest takes on :{PORT} -> {ROOT}", flush=True)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
