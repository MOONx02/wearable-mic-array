#!/usr/bin/env python3
"""Camera + sound direction view for the XIAO ESP32-S3 Sense (cam_mic_stream firmware).

Run it on any computer with Python 3.9 or newer:

    python3 mic_camera_view.py

The first run installs what it needs (numpy, PySide6, pyserial) into a private
folder (~/.mic-live-view-env, shared with mic_live_view4.py); later runs start at once.
Plug in the board by USB-C, and the window finds it by itself.

Options:
    --port PORT       serial port, if auto-detect picks the wrong one
    --wide CM         spacing of the ch0/ch1 pair in cm (default 14)
    --narrow CM       spacing of the ch2/ch3 pair in cm (default 3.5)
    --demo [JPEG]     no board: fake claps from moving directions over a still picture

Shows the board's camera picture. Loud sounds (claps, bangs, voices) get a glowing
column where they are in the picture; sounds outside the picture get an arrow on that
edge ("off screen, ~70° right"). The mics sit in a line, so they only know left/right:
the glow is a full-height column (no height), and a sound behind the mics shows up at
its mirror position in front.

Setup, once: hold the mic line level, with the camera in the middle facing the same way
the mics face. Clap straight in front of the camera and press "Set centre", then clap at
the right edge of the picture and press "Set right edge" (this measures the camera's
field of view). If the glow lands on the wrong side, tick "Swap left/right".
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

# ------------------------------------------------------------------ first-run setup

ENV_DIR = Path.home() / ".mic-live-view-env"
PACKAGES = ["numpy", "PySide6", "pyserial"]


def _env_python() -> Path:
    return ENV_DIR / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def _have_packages() -> bool:
    try:
        import numpy  # noqa: F401
        import PySide6  # noqa: F401
        import serial  # noqa: F401
        return True
    except ImportError:
        return False


def _bootstrap():
    if _have_packages():
        return
    if sys.version_info < (3, 9):
        sys.exit("Python 3.9 or newer is needed (python.org/downloads).")
    if Path(sys.prefix).resolve() == ENV_DIR.resolve():
        sys.exit(f"Setup is incomplete. Delete {ENV_DIR} and run this file again.")
    py, ready = _env_python(), ENV_DIR / ".ready"
    if not (py.exists() and ready.exists()):
        print(f"First run: setting up {ENV_DIR} (one time, needs internet)…", flush=True)
        if not py.exists():
            subprocess.check_call([sys.executable, "-m", "venv", str(ENV_DIR)])
        subprocess.check_call([str(py), "-m", "pip", "install", "--quiet", "--upgrade", "pip"])
        subprocess.check_call([str(py), "-m", "pip", "install", "--quiet", *PACKAGES])
        ready.write_text("ok\n")
        print("Setup done, starting…", flush=True)
    sys.exit(subprocess.call([str(py), os.path.abspath(__file__), *sys.argv[1:]]))


_bootstrap()

# ------------------------------------------------------------------ the app

import argparse  # noqa: E402
import math  # noqa: E402
import struct  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import wave  # noqa: E402
from collections import deque  # noqa: E402

import numpy as np  # noqa: E402
import serial  # noqa: E402
from serial.tools import list_ports  # noqa: E402
from PySide6.QtCore import QPointF, QRectF, QSettings, Qt, QTimer  # noqa: E402
from PySide6.QtGui import (QColor, QFont, QImage, QLinearGradient, QPainter, QPen,  # noqa: E402
                           QPolygonF, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout,  # noqa: E402
                               QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)

SYNC = b"\xa5\x5a"
MAGIC = SYNC + b"M4"
CH = 4
PAIRS = ((0, 1), (2, 3))  # wide pair, narrow pair
FS = 16000
C = 343.0
TIMING_SIGMA_S = 20e-6  # typical arrival-time error of one pair on a clap
BOARD_VIDS = (0x303A, 0x2886)  # Espressif (USB serial mode), Seeed (native USB mode)
HOLD_S = 2.5  # how long a sound stays on screen
HISTORY_S = 10.0
DEAD_PTP = 8
TAKES = Path(os.path.abspath(__file__)).parent / "mic_takes"

BG, PANEL, EDGE = QColor("#0d1117"), QColor("#161b22"), QColor("#30363d")
INK, DIM = QColor("#e6edf3"), QColor("#8b949e")
CH_COLORS = [QColor("#58a6ff"), QColor("#f0883e"), QColor("#3fb950"), QColor("#d2a8ff")]
PAIR_COLORS = [QColor("#ff7b72"), QColor("#e3b341")]
GLOW, LOUD = QColor("#ffa028"), QColor("#ff5a3c")
ALERT = QColor("#ff7b72")


# ------------------------------------------------------------------ serial reader

class Reader(threading.Thread):
    """Finds the board and parses its packets: mic chunks, JPEG frames and status text."""

    def __init__(self, port: str | None):
        super().__init__(daemon=True)
        self.want_port = port
        self.lock = threading.Lock()
        self.chunks: deque = deque()
        self.frame = None  # (seq, jpeg bytes, width, height)
        self.frame_times: deque = deque()
        self.board_msgs: deque = deque(maxlen=6)
        self.board_stats = ""
        self.commands: deque = deque()
        self.status = "searching for the board…"
        self.port = None
        self.gaps = 0
        self.packets = 0
        self.stop = False

    @staticmethod
    def _open(path: str) -> serial.Serial:
        s = serial.Serial()
        s.port = path
        s.baudrate = 115200
        s.timeout = 0.05
        # Keep DTR/RTS released: on the ESP32-S3's USB serial, toggling them resets the chip.
        s.dtr = False
        s.rts = False
        s.open()
        s.reset_input_buffer()
        return s

    def _candidates(self) -> list[str]:
        if self.want_port:
            return [self.want_port]
        ports = list(list_ports.comports())
        esp = [p.device for p in ports if p.vid in BOARD_VIDS]
        rest = [p.device for p in ports if p.vid not in BOARD_VIDS and p.vid is not None]
        return esp + rest

    def _probe(self, path: str) -> serial.Serial | None:
        try:
            s = self._open(path)
        except (OSError, serial.SerialException):
            return None
        buf, t0 = b"", time.time()
        while time.time() - t0 < 1.5:
            try:
                buf += s.read(65536)
            except (OSError, serial.SerialException):
                break
            if MAGIC in buf:
                return s
        s.close()
        return None

    def run(self):
        while not self.stop:
            s = None
            for p in self._candidates():
                s = self._probe(p)
                if s is not None:
                    self.port = p
                    break
            if s is None:
                self.status = "no mic stream found: plug in the board (cam_mic_stream firmware)"
                time.sleep(1.0)
                continue
            self.status = "streaming"
            self._stream(s)
            self.status = "connection lost, reconnecting…"
            self.port = None

    def send(self, cmd: bytes):
        self.commands.append(cmd)

    def _stream(self, s: serial.Serial):
        buf, last_seq = bytearray(), None
        while not self.stop:
            try:
                while self.commands:
                    s.write(self.commands.popleft())
                data = s.read(max(1, s.in_waiting or 1))
            except (OSError, serial.SerialException):
                break
            if not data:
                continue
            buf += data
            while True:
                i = buf.find(SYNC)
                if i < 0:
                    del buf[:-1]
                    break
                if i:
                    del buf[:i]
                if len(buf) < 4:
                    break
                kind = bytes(buf[2:4])
                if kind == b"M4":
                    if len(buf) < 10:
                        break
                    seq, frames = struct.unpack_from("<IH", buf, 4)
                    if frames > 4096:
                        del buf[:2]
                        continue
                    end = 10 + frames * CH * 2
                    if len(buf) < end:
                        break
                    chunk = np.frombuffer(bytes(buf[10:end]), dtype="<i2").reshape(-1, CH).copy()
                    del buf[:end]
                    if last_seq is not None and seq != (last_seq + 1) & 0xFFFFFFFF:
                        self.gaps += 1
                    last_seq = seq
                    self.packets += 1
                    with self.lock:
                        self.chunks.append(chunk)
                elif kind == b"JP":
                    if len(buf) < 16:
                        break
                    seq, length, w, h = struct.unpack_from("<IIHH", buf, 4)
                    if length > 400_000 or not (0 < w <= 2048 and 0 < h <= 2048):
                        del buf[:2]
                        continue
                    if len(buf) < 16 + length:
                        break
                    jpeg = bytes(buf[16:16 + length])
                    del buf[:16 + length]
                    now = time.time()
                    with self.lock:
                        self.frame = (seq, jpeg, w, h)
                        self.frame_times.append(now)
                        while self.frame_times and now - self.frame_times[0] > 2.0:
                            self.frame_times.popleft()
                elif kind == b"ST":
                    if len(buf) < 6:
                        break
                    (length,) = struct.unpack_from("<H", buf, 4)
                    if length > 512:
                        del buf[:2]
                        continue
                    if len(buf) < 6 + length:
                        break
                    msg = bytes(buf[6:6 + length]).decode("utf-8", "replace")
                    del buf[:6 + length]
                    with self.lock:
                        if msg.startswith("stats: "):
                            self.board_stats = msg[7:]
                        else:
                            self.board_msgs.append((time.time(), msg))
                else:
                    del buf[:2]
        try:
            s.close()
        except (OSError, serial.SerialException):
            pass

    def take(self) -> list:
        with self.lock:
            out = list(self.chunks)
            self.chunks.clear()
        return out

    @property
    def fps(self) -> float:
        with self.lock:
            t = list(self.frame_times)
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 2 and t[-1] > t[0] else 0.0


class DemoReader(Reader):
    """Stands in for the board: background noise plus claps from changing directions."""

    ANGLES = (0, 15, -20, 35, -60, 75, -40, 5)  # + = toward the ch0 side

    def __init__(self, spacings_m: list[float], jpeg: bytes | None):
        super().__init__(None)
        self.spacing = spacings_m
        self.jpeg = jpeg
        self.port = "demo"

    def _clap(self, angle_deg: float, n: int = 2048) -> np.ndarray:
        rng = np.random.default_rng()
        burst = rng.normal(0, 1, n) * np.exp(-np.arange(n) / 250.0)
        burst[:200] = 0
        F = np.fft.rfft(burst)
        f = np.fft.rfftfreq(n, 1 / FS)
        out = np.empty((n, CH))
        s = math.sin(math.radians(angle_deg))
        xs = (self.spacing[0] / 2, -self.spacing[0] / 2, self.spacing[1] / 2, -self.spacing[1] / 2)
        for ch, x in enumerate(xs):
            delay = -x * s / C
            out[:, ch] = np.fft.irfft(F * np.exp(-2j * math.pi * f * delay), n)
        return out / np.max(np.abs(out)) * 12000

    def run(self):
        self.status = "demo (no board)"
        if self.jpeg:
            self.board_msgs.append((time.time(), "demo: still picture instead of the camera"))
        else:
            self.board_msgs.append((time.time(), "demo: no picture given, showing a blank frame"))
        rng = np.random.default_rng()
        pending = np.zeros((0, CH))
        t_next_clap, k, seq = time.time() + 1.0, 0, 0
        t_frame = 0.0
        while not self.stop:
            time.sleep(0.016)
            chunk = rng.normal(0, 25, (256, CH))
            now = time.time()
            if now >= t_next_clap:
                pending = self._clap(self.ANGLES[k % len(self.ANGLES)])
                k += 1
                t_next_clap = now + 1.4
            if len(pending):
                m = min(256, len(pending))
                chunk[:m] += pending[:m]
                pending = pending[m:]
            with self.lock:
                self.chunks.append(chunk.astype(np.int16))
                if self.jpeg and now - t_frame > 0.1:
                    t_frame = now
                    self.frame = (seq, self.jpeg, 0, 0)
                    seq += 1
                    self.frame_times.append(now)
                    while self.frame_times and now - self.frame_times[0] > 2.0:
                        self.frame_times.popleft()
            self.packets += 1


# ------------------------------------------------------------------ analysis

def gcc_phat(a: np.ndarray, b: np.ndarray, max_lag: int, interp: int = 32) -> float:
    """Delay of b relative to a in samples (positive: b hears it later)."""
    n = 1 << int(math.ceil(math.log2(len(a) + len(b))))
    A, B = np.fft.rfft(a, n), np.fft.rfft(b, n)
    f = np.fft.rfftfreq(n, 1 / FS)
    cross = B * np.conj(A)
    cross /= np.abs(cross) + 1e-12
    cross[(f < 300) | (f > 6000)] = 0
    cc = np.fft.irfft(cross, n * interp)
    m = max_lag * interp
    cc = np.concatenate((cc[-m:], cc[: m + 1]))
    return (int(np.argmax(cc)) - m) / interp


class Model:
    def __init__(self, spacings_m: list[float]):
        self.spacing = list(spacings_m)
        self.threshold_db = 12.0
        self.x = np.zeros((FS * 2, CH), np.float32)
        self.total = 0
        self.rate_window: deque = deque()
        self.rate = 0.0
        self.level = np.full(CH, -120.0)
        self.floor = np.full(CH, -70.0)
        self.dead = np.zeros(CH, bool)
        self.detections: deque = deque()  # (time, angle, sigma, pairs_agree, strength, [angle per pair])
        self.last_det = None
        self.dc = np.zeros(CH)
        self.recorder = None
        self.record_path = None

    @property
    def now(self) -> float:
        return self.total / FS

    def push(self, chunks: list):
        if not chunks:
            return
        data = np.concatenate(chunks)
        if self.recorder:
            self.recorder.writeframes(data.astype("<i2").tobytes())
        x = data.astype(np.float32)
        m = x.mean(axis=0)
        self.dc = m if self.total == 0 else 0.95 * self.dc + 0.05 * m
        x -= self.dc
        n = len(x)
        if n >= len(self.x):
            self.x[:] = x[-len(self.x):]
        else:
            self.x[:-n] = self.x[n:]
            self.x[-n:] = x
        self.total += n
        wall = time.time()
        self.rate_window.append((wall, n))
        while self.rate_window and wall - self.rate_window[0][0] > 2.0:
            self.rate_window.popleft()
        span = wall - self.rate_window[0][0]
        if span > 0.5:
            self.rate = sum(k for _, k in list(self.rate_window)[1:]) / span
        if self.total >= FS // 2:
            self.dead = np.ptp(self.x[-FS // 2:], axis=0) < DEAD_PTP
        self._levels()
        self._direction()

    def _levels(self):
        blk = self.x[-int(0.05 * FS):] / 32768.0
        lv = 10 * np.log10(np.var(blk, axis=0) + 1e-12)
        self.level = lv
        if self.total < int(0.5 * FS):
            self.floor = np.minimum(self.floor, lv) if self.total > int(0.1 * FS) else lv
            return
        self.floor = np.where(lv < self.floor, 0.7 * self.floor + 0.3 * lv, self.floor + 0.02)

    def _direction(self):
        live = ~self.dead
        if self.total < int(0.5 * FS) or not live.any():
            return
        strength = float(np.max((self.level - self.floor)[live]))
        if strength < self.threshold_db:
            return
        if self.last_det and self.now - self.last_det[0] < 0.08:
            return
        seg = self.x[-1024:]
        angles, sigmas = [], []
        for (a, b), spacing in zip(PAIRS, self.spacing):
            if self.dead[a] or self.dead[b]:
                angles.append(None)
                sigmas.append(None)
                continue
            lag = gcc_phat(seg[:, a], seg[:, b], int(math.ceil(spacing / C * FS)) + 2)
            s = max(-1.0, min(1.0, lag / FS * C / spacing))
            ang = math.degrees(math.asin(s))  # + = toward the ch0 / ch2 side
            angles.append(ang)
            cos = max(math.cos(math.radians(ang)), 0.2)
            sigmas.append(min(30.0, math.degrees(TIMING_SIGMA_S * C / (spacing * cos))))
        if all(a is None for a in angles):
            return
        agree = None
        if None not in angles:
            tol = 2 * math.hypot(*sigmas) + 5
            agree = abs(angles[0] - angles[1]) <= tol
            if agree:
                w0, w1 = 1 / sigmas[0] ** 2, 1 / sigmas[1] ** 2
                angle = (w0 * angles[0] + w1 * angles[1]) / (w0 + w1)
                sigma = (w0 + w1) ** -0.5
            else:
                angle, sigma = angles[0], sigmas[0]
        else:
            k = 0 if angles[0] is not None else 1
            angle, sigma = angles[k], sigmas[k]
        det = (self.now, angle, sigma, agree, strength, angles)
        self.detections.append(det)
        self.last_det = det
        while self.detections and self.now - self.detections[0][0] > HISTORY_S:
            self.detections.popleft()

    def toggle_record(self):
        if self.recorder:
            self.recorder.close()
            self.recorder = None
            return None
        TAKES.mkdir(parents=True, exist_ok=True)
        self.record_path = TAKES / time.strftime("cam4_%Y%m%d_%H%M%S.wav")
        w = wave.open(str(self.record_path), "wb")
        w.setnchannels(CH)
        w.setsampwidth(2)
        w.setframerate(FS)
        self.recorder = w
        return str(self.record_path)


# ------------------------------------------------------------------ drawing

def font(size, bold=False, mono=False) -> QFont:
    f = QFont()
    f.setStyleHint(QFont.Monospace if mono else QFont.SansSerif)
    f.setFamilies(["Menlo", "Consolas", "DejaVu Sans Mono"] if mono else ["Helvetica", "Segoe UI", "Arial"])
    f.setPixelSize(size)
    f.setBold(bold)
    return f


def panel(p: QPainter, r: QRectF, title: str):
    p.setPen(QPen(EDGE, 1.5))
    p.setBrush(PANEL)
    p.drawRoundedRect(r, 10, 10)
    p.setFont(font(15, True))
    p.setPen(DIM)
    p.drawText(QPointF(r.x() + 14, r.y() + 24), title)


def with_alpha(c: QColor, a: float) -> QColor:
    c = QColor(c)
    c.setAlphaF(max(0.0, min(1.0, a)))
    return c


def label_box(p: QPainter, x: float, y: float, lines: list[str], color: QColor, bounds: QRectF):
    """Dark rounded tag with a coloured border; x, y is its top-left, kept inside bounds."""
    p.setFont(font(15, True))
    fm = p.fontMetrics()
    w = max(fm.horizontalAdvance(lines[0]) + 20, *(fm.horizontalAdvance(t) * 0.87 + 20 for t in lines[1:]))
    h = 26 + 19 * (len(lines) - 1)
    x = min(max(x, bounds.left() + 6), bounds.right() - w - 6)
    y = min(max(y, bounds.top() + 6), bounds.bottom() - h - 6)
    box = QRectF(x, y, w, h)
    p.setPen(QPen(color, 2))
    p.setBrush(QColor(13, 17, 23, 215))
    p.drawRoundedRect(box, 7, 7)
    p.setPen(INK)
    p.drawText(QPointF(x + 10, y + 19), lines[0])
    p.setFont(font(13))
    p.setPen(QColor("#c9d1d9"))
    for i, t in enumerate(lines[1:]):
        p.drawText(QPointF(x + 10, y + 38 + 19 * i), t)


class View(QWidget):
    def __init__(self, model: Model, reader: Reader):
        super().__init__()
        self.m = model
        self.reader = reader
        self.swap = False
        self.hfov = 60.0
        self.offset = 0.0  # mic angle (+ = left) that is the centre of the picture
        self.rotate = 0
        self.mirror = False
        self.image: QImage | None = None
        self.image_seq = None
        self.setMinimumSize(1100, 680)

    def oriented(self, ang):
        """Mic angle -> + = left of the wearer / picture."""
        return None if ang is None else (-ang if self.swap else ang)

    def update_frame(self):
        with self.reader.lock:
            fr = self.reader.frame
        if fr is None or fr[0] == self.image_seq:
            return
        img = QImage.fromData(fr[1], "JPG")
        if img.isNull():
            return
        t = QTransform()
        if self.rotate:
            t.rotate(self.rotate)
        if self.mirror:
            t.scale(-1, 1)
        if self.rotate or self.mirror:
            img = img.transformed(t, Qt.SmoothTransformation)
        self.image, self.image_seq = img, fr[0]

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BG)
        w, h = self.width(), self.height()
        gap = 14
        left_w = w * 0.70
        cam_r = QRectF(gap, gap, left_w - 1.5 * gap, h - 2 * gap)
        x1 = left_w + 0.5 * gap
        gauge_r = QRectF(x1, gap, w - x1 - gap, h * 0.40)
        lvl_r = QRectF(x1, gauge_r.bottom() + gap, w - x1 - gap, h * 0.24)
        info_r = QRectF(x1, lvl_r.bottom() + gap, w - x1 - gap, h - lvl_r.bottom() - 2 * gap)
        self.draw_camera(p, cam_r)
        self.draw_gauge(p, gauge_r)
        self.draw_levels(p, lvl_r)
        self.draw_info(p, info_r)
        p.end()

    # -- camera with overlay

    def draw_camera(self, p, r):
        panel(p, r, "Camera + sound direction")
        area = r.adjusted(12, 36, -12, -34)
        img = self.image
        aspect = img.width() / img.height() if img else 4 / 3
        pw = min(area.width(), area.height() * aspect)
        ph = pw / aspect
        pic = QRectF(area.center().x() - pw / 2, area.center().y() - ph / 2, pw, ph)
        if img:
            p.setRenderHint(QPainter.SmoothPixmapTransform)
            p.drawImage(pic, img)
        else:
            p.setPen(QPen(EDGE, 1.5, Qt.DashLine))
            p.setBrush(QColor("#0f141a"))
            p.drawRect(pic)
            p.setFont(font(18))
            p.setPen(DIM)
            p.drawText(pic, Qt.AlignCenter, "no camera picture yet\n(sound directions still show)")
        p.save()
        p.setClipRect(pic)
        f = (pic.width() / 2) / math.tan(math.radians(self.hfov / 2))
        cx = pic.center().x()

        def x_of(rel):
            return cx - f * math.tan(math.radians(max(-89.0, min(89.0, rel))))

        self.draw_ruler(p, pic, x_of)
        now = self.m.now
        recent = [d for d in self.m.detections if now - d[0] < HOLD_S]
        half = self.hfov / 2
        for det in recent:
            t, angle, sigma, _, strength, _ = det
            fade = 1 - (now - t) / HOLD_S
            rel = self.oriented(angle) - self.offset
            color = LOUD if strength > 30 else GLOW
            latest = det is recent[-1]
            if abs(rel) <= half:
                x = x_of(rel)
                spread = abs(x_of(rel + sigma) - x_of(rel - sigma)) / 2
                hw = max(18.0, spread) * 2
                g = QLinearGradient(x - hw, 0, x + hw, 0)
                g.setColorAt(0.0, with_alpha(color, 0))
                g.setColorAt(0.5, with_alpha(color, 0.55 * fade if latest else 0.25 * fade))
                g.setColorAt(1.0, with_alpha(color, 0))
                p.setPen(Qt.NoPen)
                p.setBrush(g)
                p.drawRect(QRectF(x - hw, pic.top(), 2 * hw, pic.height()))
                p.setPen(QPen(with_alpha(QColor("#fff3d6"), fade), 2.5 if latest else 1.5))
                p.drawLine(QPointF(x, pic.top()), QPointF(x, pic.bottom()))
                if latest:
                    near = abs(rel) + sigma > half
                    lines = [f"{'LOUD · ' if strength > 30 else ''}Sound · in view" if not near else "Sound · near the edge",
                             f"{self.side_text(rel)} · ±{sigma:.0f}° · {strength:.0f} dB over quiet",
                             f"{now - t:.1f} s ago · or behind, at the mirror spot"]
                    label_box(p, x + 14, pic.top() + 0.18 * pic.height(), lines, color, pic)
            elif latest:
                self.draw_edge_arrow(p, pic, rel, sigma, strength, now - t, fade, color)
        p.restore()
        p.setFont(font(12))
        p.setPen(DIM)
        p.drawText(QRectF(r.x() + 14, r.bottom() - 28, r.width() - 28, 20), Qt.AlignLeft | Qt.AlignVCenter,
                   "Mics in a line: left/right only. The glow is a column (no height); "
                   "a sound behind shows at its mirror position.")

    def side_text(self, rel: float) -> str:
        if abs(rel) < 2:
            return "straight ahead"
        return f"{abs(rel):.0f}° {'left' if rel > 0 else 'right'}"

    def draw_ruler(self, p, pic, x_of):
        """Angle ticks along the bottom of the picture (handy for checking calibration)."""
        y = pic.bottom() - 26
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 120))
        p.drawRect(QRectF(pic.left(), y - 4, pic.width(), 30))
        p.setFont(font(11))
        half = int(self.hfov / 2)
        for a in range(-90, 91, 10):
            if abs(a) > half:
                continue
            x = x_of(a)
            p.setPen(QPen(QColor(255, 255, 255, 170), 1.5 if a == 0 else 1))
            p.drawLine(QPointF(x, y - 4), QPointF(x, y + (6 if a % 30 else 10)))
            if a % 20 == 0:
                p.drawText(QRectF(x - 20, y + 9, 40, 14), Qt.AlignCenter, "0°" if a == 0 else f"{abs(a)}°")

    def draw_edge_arrow(self, p, pic, rel, sigma, strength, age, fade, color):
        left = rel > 0
        just = abs(rel) - sigma <= self.hfov / 2
        y = pic.center().y()
        tip = pic.left() + 10 if left else pic.right() - 10
        d = 1 if left else -1
        arrow = QPolygonF([QPointF(tip, y), QPointF(tip + d * 34, y - 30), QPointF(tip + d * 34, y - 13),
                           QPointF(tip + d * 70, y - 13), QPointF(tip + d * 70, y + 13),
                           QPointF(tip + d * 34, y + 13), QPointF(tip + d * 34, y + 30)])
        p.setPen(QPen(with_alpha(QColor("#1a1206"), fade), 2))
        p.setBrush(with_alpha(color, 0.35 + 0.65 * fade))
        p.drawPolygon(arrow)
        side = "left" if left else "right"
        head = "probably just off screen" if just else "off screen"
        lines = [f"{'LOUD · ' if strength > 30 else ''}{head} · ~{abs(rel):.0f}° {side}",
                 f"turn {side} to see it · ±{sigma:.0f}°",
                 f"{age:.1f} s ago · or behind you on that side"]
        bx = tip + d * 80 if left else tip + d * 80 - 300
        label_box(p, bx, y - 40, lines, color, pic)

    # -- right column

    def draw_gauge(self, p, r):
        panel(p, r, "Direction (top view, front half)")
        cx, base = r.center().x(), r.bottom() - 40
        R = min(r.width() * 0.40, r.height() - 110)
        box = QRectF(cx - R, base - R, 2 * R, 2 * R)
        p.setPen(QPen(EDGE, 1.5))
        p.setBrush(QColor("#0f141a"))
        p.drawPie(box, 0, 180 * 16)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(88, 166, 255, 45))
        lo = max(-90.0, self.offset - self.hfov / 2)
        hi = min(90.0, self.offset + self.hfov / 2)
        p.drawPie(box, int((90 + lo) * 16), int((hi - lo) * 16))
        p.setFont(font(11))
        for a in range(-90, 91, 30):
            t = math.radians(a)
            p.setPen(QPen(DIM, 1.5))
            p.drawLine(QPointF(cx - R * math.sin(t), base - R * math.cos(t)),
                       QPointF(cx - (R - 10) * math.sin(t), base - (R - 10) * math.cos(t)))
            p.drawText(QPointF(cx - (R + 16) * math.sin(t) - 10, base - (R + 16) * math.cos(t) + 4), f"{abs(a)}°")
        p.setPen(QColor("#58a6ff"))
        p.setFont(font(11, True))
        p.drawText(QRectF(cx - 60, base - R * 0.55, 120, 16), Qt.AlignCenter, "camera view")
        p.setFont(font(12, True))
        p.setPen(INK)
        p.drawText(QPointF(cx - R - 10, base + 26), "LEFT")
        p.drawText(QPointF(cx + R - 30, base + 26), "RIGHT")

        now = self.m.now
        det = self.m.last_det
        if det and now - det[0] < 4:
            fade = max(0.25, 1 - (now - det[0]) / 4)
            for k in (1, 0):
                a = self.oriented(det[5][k])
                if a is None:
                    continue
                th = math.radians(a)
                p.setPen(QPen(with_alpha(PAIR_COLORS[k], fade), 6 if k == 0 else 3, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(cx, base), QPointF(cx - (R - 8) * math.sin(th), base - (R - 8) * math.cos(th)))
            rel = self.oriented(det[1]) - self.offset
            p.setFont(font(22, True))
            p.setPen(INK)
            where = "in view" if abs(rel) <= self.hfov / 2 else "off screen"
            p.drawText(QRectF(r.x(), r.y() + 30, r.width(), 30), Qt.AlignHCenter, f"{self.side_text(rel)} · {where}")
            p.setFont(font(12, mono=True))
            p.setPen(DIM)
            agree = {True: "pairs agree", False: "pairs disagree (using the 140 mm pair)", None: "one pair only"}[det[3]]
            p.drawText(QRectF(r.x(), r.y() + 62, r.width(), 18), Qt.AlignHCenter, agree)
        else:
            p.setFont(font(15))
            p.setPen(DIM)
            p.drawText(QRectF(r.x(), r.y() + 36, r.width(), 24), Qt.AlignHCenter, "clap, snap or talk")
        for k, name in enumerate(("140 mm pair", "35 mm pair")):
            x = cx - 95 + k * 105
            p.setPen(Qt.NoPen)
            p.setBrush(PAIR_COLORS[k])
            p.drawEllipse(QPointF(x, base + 22), 4, 4)
            p.setFont(font(11))
            p.setPen(PAIR_COLORS[k])
            p.drawText(QPointF(x + 9, base + 26), name)
        p.setPen(Qt.NoPen)
        p.setBrush(INK)
        p.drawEllipse(QPointF(cx, base), 5, 5)

    def draw_levels(self, p, r):
        panel(p, r, "Mic levels (dBFS)")
        inner = r.adjusted(54, 34, -70, -10)
        row_h = inner.height() / CH

        def xf(db):
            return inner.left() + float(np.clip((db + 80) / 80, 0, 1)) * inner.width()

        for ch in range(CH):
            y = inner.top() + ch * row_h + row_h / 2
            p.setFont(font(13, True))
            p.setPen(CH_COLORS[ch])
            p.drawText(QPointF(r.x() + 14, y + 5), f"ch{ch}")
            bh = min(16.0, row_h - 6)
            bar = QRectF(inner.left(), y - bh / 2, inner.width(), bh)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("#21262d"))
            p.drawRoundedRect(bar, 4, 4)
            p.setFont(font(12, mono=True))
            if self.m.dead[ch]:
                p.setPen(ALERT)
                p.drawText(QPointF(bar.left() + 8, y + 5), "no signal: check 3V, GND, CLK, DAT")
                continue
            p.setBrush(CH_COLORS[ch])
            p.drawRoundedRect(QRectF(bar.left(), bar.top(), xf(self.m.level[ch]) - bar.left(), bh), 4, 4)
            p.setPen(QPen(DIM, 1, Qt.DashLine))
            xn = xf(self.m.floor[ch])
            p.drawLine(QPointF(xn, bar.top() - 3), QPointF(xn, bar.bottom() + 3))
            p.setPen(INK)
            p.drawText(QPointF(inner.right() + 10, y + 5), f"{self.m.level[ch]:5.0f}")

    def draw_info(self, p, r):
        panel(p, r, "Board")
        img = self.image
        lines = [
            f"camera: {img.width()}x{img.height()} · {self.reader.fps:4.1f} fps" if img else "camera: no picture",
            f"mics: {self.m.rate / 1000:5.2f} kHz · gaps {self.reader.gaps}",
            f"field of view {self.hfov:.0f}° · centre offset {self.offset:+.0f}°",
        ]
        with self.reader.lock:
            msgs = list(self.reader.board_msgs)[-2:]
            stats = self.reader.board_stats
        p.setFont(font(13, mono=True))
        y = r.y() + 50
        for t in lines:
            p.setPen(INK)
            p.drawText(QPointF(r.x() + 14, y), t)
            y += 22
        p.setFont(font(12))
        if stats:
            p.setPen(DIM)
            p.drawText(QRectF(r.x() + 14, y - 14, r.width() - 28, 36), Qt.TextWordWrap, "board: " + stats)
            y += 36
        for _, msg in msgs:
            p.setPen(ALERT if "fail" in msg or "not available" in msg else DIM)
            p.drawText(QRectF(r.x() + 14, y - 14, r.width() - 28, 36), Qt.TextWordWrap, msg)
            y += 34


# ------------------------------------------------------------------ window

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: find the board automatically)")
    ap.add_argument("--wide", type=float, default=14.0, help="ch0/ch1 spacing in cm")
    ap.add_argument("--narrow", type=float, default=3.5, help="ch2/ch3 spacing in cm")
    ap.add_argument("--demo", nargs="?", const="", metavar="JPEG", help="run without a board")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()

    app = QApplication(sys.argv[:1])
    spacings = [args.wide / 100, args.narrow / 100]
    model = Model(spacings)
    if args.demo is not None:
        reader = DemoReader(spacings, Path(args.demo).read_bytes() if args.demo else None)
    else:
        reader = Reader(args.port)
    reader.start()

    settings = QSettings("CapstoneMicArray", "mic_camera_view")
    view = View(model, reader)
    view.hfov = float(settings.value("hfov", 60.0))
    view.offset = float(settings.value("offset", 0.0))
    view.swap = settings.value("swap", "false") in (True, "true")
    view.rotate = int(settings.value("rotate", 0))
    view.mirror = settings.value("mirror", "false") in (True, "true")

    win = QWidget()
    win.setWindowTitle("Camera + sound direction")
    win.setStyleSheet("QWidget { background: #0d1117; color: #e6edf3; font-size: 14px; }"
                      "QPushButton, QComboBox { background: #21262d; border: 1px solid #30363d;"
                      " border-radius: 6px; padding: 5px 12px; }"
                      "QPushButton:checked { background: #da3633; border-color: #ff7b72; }"
                      "QDoubleSpinBox { background: #21262d; border: 1px solid #30363d; padding: 3px; }")
    status = QLabel()
    status.setStyleSheet("color: #8b949e; font-size: 12px;")
    rec = QPushButton("● Record audio")
    rec.setCheckable(True)
    snap = QPushButton("Snapshot")
    fov = QDoubleSpinBox()
    fov.setRange(20, 140)
    fov.setDecimals(0)
    fov.setSuffix("° view")
    fov.setValue(view.hfov)
    set_centre = QPushButton("Set centre")
    set_centre.setToolTip("Clap straight in front of the camera, then press this")
    set_edge = QPushButton("Set right edge")
    set_edge.setToolTip("Clap at the right edge of the picture, then press this (measures the field of view)")
    swap = QCheckBox("Swap left/right")
    swap.setChecked(view.swap)
    rotate = QComboBox()
    rotate.addItems(["No rotation", "Turn 90° clockwise", "Turn 180°", "Turn 90° counter-clockwise"])
    rotate.setCurrentIndex(view.rotate // 90)
    mirror = QCheckBox("Mirror picture")
    mirror.setChecked(view.mirror)
    res = QComboBox()
    res.addItems(["320x240", "640x480", "800x600"])
    thresh = QSlider(Qt.Horizontal)
    thresh.setRange(4, 30)
    thresh.setValue(int(model.threshold_db))
    thresh.setFixedWidth(120)
    thresh_lbl = QLabel()

    row1 = QHBoxLayout()
    for wdg in (rec, snap, QLabel("Camera"), res, rotate, mirror, QLabel("Trigger"), thresh, thresh_lbl):
        row1.addWidget(wdg)
    row1.addStretch(1)
    row1.addWidget(status)
    row2 = QHBoxLayout()
    for wdg in (QLabel("Calibrate:"), fov, set_centre, set_edge, swap):
        row2.addWidget(wdg)
    hint = QLabel("Clap straight ahead → Set centre. Clap at the right edge of the picture → Set right edge.")
    hint.setStyleSheet("color: #8b949e; font-size: 12px;")
    row2.addWidget(hint)
    row2.addStretch(1)
    lay = QVBoxLayout(win)
    lay.setContentsMargins(8, 8, 8, 8)
    lay.addLayout(row1)
    lay.addLayout(row2)
    lay.addWidget(view, 1)

    def save():
        settings.setValue("hfov", view.hfov)
        settings.setValue("offset", view.offset)
        settings.setValue("swap", view.swap)
        settings.setValue("rotate", view.rotate)
        settings.setValue("mirror", view.mirror)

    def last_angle():
        det = model.last_det
        if not det or model.now - det[0] > 5:
            hint.setText("No recent sound: clap first, then press the button within 5 s.")
            return None
        return view.oriented(det[1])

    def on_centre():
        a = last_angle()
        if a is not None:
            view.offset = a
            hint.setText(f"Centre set: picture centre = {a:+.0f}° on the mics.")
            save()

    def on_edge():
        a = last_angle()
        if a is None:
            return
        half = view.offset - a  # right of centre is a negative angle
        if not 5 <= half <= 70:
            hint.setText(f"That clap was {half:+.0f}° from centre; clap at the RIGHT edge, or try Swap left/right.")
            return
        view.hfov = 2 * half
        fov.setValue(view.hfov)
        hint.setText(f"Field of view set to {view.hfov:.0f}°.")
        save()

    def on_rotate(i):
        view.rotate = 90 * i
        view.image_seq = None
        save()

    def on_mirror(v):
        view.mirror = v
        view.image_seq = None
        save()

    def on_fov(v):
        view.hfov = float(v)
        save()

    def on_swap(v):
        view.swap = v
        save()

    def on_thresh(v):
        model.threshold_db = float(v)
        thresh_lbl.setText(f"{v} dB")

    def on_rec(checked):
        if checked != bool(model.recorder):
            model.toggle_record()
        rec.setText("■ Stop recording" if model.recorder else "● Record audio")

    def on_snap():
        TAKES.mkdir(parents=True, exist_ok=True)
        path = TAKES / time.strftime("cam_%Y%m%d_%H%M%S.png")
        view.grab().save(str(path))
        hint.setText(f"Saved {path.name} in mic_takes.")

    set_centre.clicked.connect(on_centre)
    set_edge.clicked.connect(on_edge)
    rotate.currentIndexChanged.connect(on_rotate)
    mirror.toggled.connect(on_mirror)
    fov.valueChanged.connect(on_fov)
    swap.toggled.connect(on_swap)
    res.currentIndexChanged.connect(lambda i: reader.send(b"123"[i:i + 1]))
    thresh.valueChanged.connect(on_thresh)
    on_thresh(thresh.value())
    rec.toggled.connect(on_rec)
    snap.clicked.connect(on_snap)

    def tick():
        model.push(reader.take())
        view.update_frame()
        recs = f"  ● REC {model.record_path.name}" if model.recorder else ""
        status.setText(f"{reader.port or '-'} · {reader.status}{recs}")
        view.update()

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(33)
    win.resize(1500, 900)
    win.show()
    if args.screenshot:
        QTimer.singleShot(int(os.environ.get("SHOT_MS", "6000")),
                          lambda: (win.grab().save(args.screenshot), app.quit()))
    app.exec()
    reader.stop = True
    if model.recorder:
        model.recorder.close()


if __name__ == "__main__":
    main()
