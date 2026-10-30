#!/usr/bin/env python3
"""Four-mic live view for the XIAO ESP32-S3 mic array (four_mic_stream firmware).

Run it on any computer with Python 3.9 or newer:

    python3 mic_live_view4.py

The first run installs what it needs (numpy, PySide6, pyserial) into a private
folder (~/.mic-live-view-env) and takes a minute or two; later runs start at once.
Plug in the board by USB-C, and the window finds it by itself.

Options:
    --port PORT       serial port, if auto-detect picks the wrong one
    --wide CM         spacing of the ch0/ch1 pair in cm (default 14)
    --narrow CM       spacing of the ch2/ch3 pair in cm (default 3.5)

Shows all four waveforms, a scrolling spectrogram, levels (with a warning when a mic
stops sending), and the direction of loud sounds from each pair's arrival-time
difference (GCC-PHAT). The two pairs face the same way, so their needles should agree;
the wide pair is about four times more precise. Mics in a line can't tell front from
back, so the gauge is a half circle. Record saves 4-channel 16 kHz WAV files to a
"mic_takes" folder next to this file.
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
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer  # noqa: E402
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen  # noqa: E402
from PySide6.QtWidgets import (QApplication, QCheckBox, QDoubleSpinBox, QHBoxLayout, QLabel,  # noqa: E402
                               QPushButton, QSlider, QVBoxLayout, QWidget)

MAGIC = b"\xa5\x5aM4"
CH = 4
PAIRS = ((0, 1), (2, 3))  # wide pair, narrow pair
FS = 16000
C = 343.0
ESPRESSIF_VID = 0x303A
HISTORY_S = 20.0
WAVE_S = 2.0
SPEC_S = 8.0
NFFT = 512
SPEC_HOP = 256
DEAD_PTP = 8  # a channel whose samples move less than this over 0.5 s isn't sending
TAKES = Path(os.path.abspath(__file__)).parent / "mic_takes"

BG, PANEL, EDGE = QColor("#0d1117"), QColor("#161b22"), QColor("#30363d")
INK, DIM = QColor("#e6edf3"), QColor("#8b949e")
CH_COLORS = [QColor("#58a6ff"), QColor("#f0883e"), QColor("#3fb950"), QColor("#d2a8ff")]
PAIR_COLORS = [QColor("#ff7b72"), QColor("#e3b341")]
ALERT = QColor("#ff7b72")
_CMAP = np.array([[0, 0, 4], [40, 11, 84], [101, 21, 110], [159, 42, 99], [212, 72, 66],
                  [245, 125, 21], [250, 193, 39], [252, 255, 164]], float)
_LUT = np.zeros(256, np.uint32)
for _i in range(256):
    _v = _i / 255 * (len(_CMAP) - 1)
    _k = min(int(_v), len(_CMAP) - 2)
    _c = _CMAP[_k] + (_CMAP[_k + 1] - _CMAP[_k]) * (_v - _k)
    _LUT[_i] = 0xFF000000 | (int(_c[0]) << 16) | (int(_c[1]) << 8) | int(_c[2])


# ------------------------------------------------------------------ serial reader

class Reader(threading.Thread):
    """Finds the board, parses packets, and hands int16 (frames, 4) chunks to the UI."""

    def __init__(self, port: str | None):
        super().__init__(daemon=True)
        self.want_port = port
        self.lock = threading.Lock()
        self.chunks: deque = deque()
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
        esp = [p.device for p in ports if p.vid == ESPRESSIF_VID]
        rest = [p.device for p in ports if p.vid != ESPRESSIF_VID and p.vid is not None]
        return esp + rest

    def _probe(self, path: str) -> serial.Serial | None:
        try:
            s = self._open(path)
        except (OSError, serial.SerialException):
            return None
        buf, t0 = b"", time.time()
        while time.time() - t0 < 1.0:
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
                self.status = "no mic stream found: plug in the board (four_mic_stream firmware)"
                time.sleep(1.0)
                continue
            self.status = "streaming"
            self._stream(s)
            self.status = "connection lost, reconnecting…"
            self.port = None

    def _stream(self, s: serial.Serial):
        buf, last_seq = b"", None
        while not self.stop:
            try:
                data = s.read(max(1, s.in_waiting or 1))
            except (OSError, serial.SerialException):
                break
            if not data:
                continue
            buf += data
            while True:
                i = buf.find(MAGIC)
                if i < 0:
                    buf = buf[-3:]
                    break
                if len(buf) < i + 10:
                    buf = buf[i:]
                    break
                seq, frames = struct.unpack_from("<IH", buf, i + 4)
                if frames > 4096:
                    buf = buf[i + 4:]
                    continue
                end = i + 10 + frames * CH * 2
                if len(buf) < end:
                    buf = buf[i:]
                    break
                chunk = np.frombuffer(buf[i + 10:end], dtype="<i2").reshape(-1, CH).copy()
                buf = buf[end:]
                if last_seq is not None and seq != (last_seq + 1) & 0xFFFFFFFF:
                    self.gaps += 1
                last_seq = seq
                self.packets += 1
                with self.lock:
                    self.chunks.append(chunk)
        try:
            s.close()
        except (OSError, serial.SerialException):
            pass

    def take(self) -> list:
        with self.lock:
            out = list(self.chunks)
            self.chunks.clear()
        return out


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
        self.spacing = list(spacings_m)  # one per pair
        self.swap = False
        self.threshold_db = 12.0
        self.x = np.zeros((int(HISTORY_S * FS), CH), np.float32)  # newest at the end
        self.total = 0
        self.rate_window: deque = deque()
        self.rate = 0.0
        self.level = np.full(CH, -120.0)
        self.peak = np.full(CH, -120.0)
        self.floor = np.full(CH, -70.0)
        self.dead = np.zeros(CH, bool)
        self.spec_cols = int(SPEC_S * FS / SPEC_HOP)
        self.spec = np.zeros((NFFT // 2 + 1, self.spec_cols), np.uint8)
        self.spec_pending = 0
        self.window = np.hanning(NFFT).astype(np.float32)
        # (time, [angle per pair or None], [lag_us per pair or None], strength)
        self.detections: deque = deque()
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
        self.spec_pending += n
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
        self._spectrogram()
        self._direction()

    def _levels(self):
        blk = self.x[-int(0.05 * FS):] / 32768.0
        lv = 10 * np.log10(np.var(blk, axis=0) + 1e-12)
        self.level = lv
        self.peak = np.maximum(lv, self.peak - 0.5)
        if self.total < int(0.5 * FS):
            self.floor = np.minimum(self.floor, lv) if self.total > int(0.1 * FS) else lv
            return
        # noise floor: falls quickly to quiet levels, rises slowly
        self.floor = np.where(lv < self.floor, 0.7 * self.floor + 0.3 * lv, self.floor + 0.02)

    def _spectrogram(self):
        cols = self.spec_pending // SPEC_HOP
        if cols <= 0:
            return
        cols = min(cols, self.spec_cols)
        self.spec_pending -= cols * SPEC_HOP
        new = np.empty((NFFT // 2 + 1, cols), np.uint8)
        mix = self.x[:, ~self.dead].mean(axis=1) if (~self.dead).any() else self.x[:, 0]
        for j in range(cols):
            end = len(mix) - self.spec_pending - (cols - 1 - j) * SPEC_HOP
            frame = mix[end - NFFT:end] / 32768.0 * self.window
            p = np.abs(np.fft.rfft(frame)) ** 2 / (self.window.sum() ** 2) * 4
            dbv = 10 * np.log10(p + 1e-14)
            new[:, j] = np.clip((dbv + 115) / 80 * 255, 0, 255).astype(np.uint8)
        self.spec = np.concatenate([self.spec[:, cols:], new], axis=1)

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
        angles, lags = [], []
        for (a, b), spacing in zip(PAIRS, self.spacing):
            if self.dead[a] or self.dead[b]:
                angles.append(None)
                lags.append(None)
                continue
            lag = gcc_phat(seg[:, a], seg[:, b], int(math.ceil(spacing / C * FS)) + 2)
            s = max(-1.0, min(1.0, lag / FS * C / spacing))
            angles.append(math.degrees(math.asin(s)))  # + = toward ch0 / ch2 side
            lags.append(lag / FS * 1e6)
        if all(a is None for a in angles):
            return
        det = (self.now, angles, lags, strength)
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
        self.record_path = TAKES / time.strftime("live4_%Y%m%d_%H%M%S.wav")
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


class View(QWidget):
    def __init__(self, model: Model):
        super().__init__()
        self.m = model
        self.setMinimumSize(1100, 700)

    def pair_name(self, k: int) -> str:
        return f"{self.m.spacing[k] * 1000:.0f} mm pair"

    def oriented(self, ang):
        return None if ang is None else (-ang if self.m.swap else ang)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BG)
        w, h = self.width(), self.height()
        gap = 14
        left_w = w * 0.6
        wave_r = QRectF(gap, gap, left_w - 1.5 * gap, h * 0.5)
        spec_r = QRectF(gap, wave_r.bottom() + gap, left_w - 1.5 * gap, h - wave_r.bottom() - 2 * gap)
        x1 = left_w + 0.5 * gap
        gauge_r = QRectF(x1, gap, w - x1 - gap, h * 0.42)
        lvl_r = QRectF(x1, gauge_r.bottom() + gap, w - x1 - gap, h * 0.25)
        hist_r = QRectF(x1, lvl_r.bottom() + gap, w - x1 - gap, h - lvl_r.bottom() - 2 * gap)
        self.draw_waves(p, wave_r)
        self.draw_spec(p, spec_r)
        self.draw_gauge(p, gauge_r)
        self.draw_levels(p, lvl_r)
        self.draw_history(p, hist_r)
        p.end()

    def draw_waves(self, p, r):
        panel(p, r, f"Waveforms (last {WAVE_S:.0f} s)")
        x = self.m.x[-int(WAVE_S * FS):]
        scale = max(400.0, float(np.max(np.abs(x))) * 1.1)
        inner = r.adjusted(60, 34, -12, -12)
        lane_h = inner.height() / CH
        cols = max(1, int(inner.width()))
        for ch in range(CH):
            top = inner.top() + ch * lane_h
            mid = top + lane_h / 2
            p.setPen(QPen(EDGE, 1))
            p.drawLine(QPointF(inner.left(), mid), QPointF(inner.right(), mid))
            p.setFont(font(13, True))
            p.setPen(CH_COLORS[ch])
            p.drawText(QPointF(r.x() + 14, mid + 5), f"ch{ch}")
            if self.m.dead[ch]:
                p.setFont(font(13, True))
                p.setPen(ALERT)
                p.drawText(QPointF(inner.left() + 10, mid - 6), "no signal")
            seg = x[: len(x) // cols * cols, ch].reshape(cols, -1)
            lo, hi = seg.min(axis=1), seg.max(axis=1)
            path = QPainterPath()
            for i in range(cols):
                xx = inner.left() + i
                path.moveTo(xx, mid - hi[i] / scale * lane_h / 2)
                path.lineTo(xx, mid - lo[i] / scale * lane_h / 2 + 0.5)
            p.setPen(QPen(CH_COLORS[ch], 1))
            p.drawPath(path)
            if ch == 1:
                p.setPen(QPen(DIM, 1, Qt.DashLine))
                p.drawLine(QPointF(r.x() + 8, top + lane_h), QPointF(r.right() - 8, top + lane_h))
        p.setFont(font(12))
        p.setPen(DIM)
        p.drawText(QPointF(inner.right() - 160, r.y() + 24), f"full scale ±{scale:,.0f}")
        p.drawText(QPointF(r.x() + 240, r.y() + 24),
                   f"ch0/ch1: {self.pair_name(0)}   ·   ch2/ch3: {self.pair_name(1)}")

    def draw_spec(self, p, r):
        panel(p, r, f"Spectrogram, all mics (last {SPEC_S:.0f} s)")
        inner = r.adjusted(60, 34, -12, -26)
        rgb = np.ascontiguousarray(_LUT[self.m.spec[::-1]])
        img = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.shape[1] * 4, QImage.Format_RGB32)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        p.drawImage(inner, img)
        p.setFont(font(12))
        p.setPen(DIM)
        for khz in (0, 2, 4, 6, 8):
            y = inner.bottom() - khz / 8 * inner.height()
            p.drawText(QPointF(r.x() + 12, y + 4), f"{khz} kHz")
        p.drawText(QPointF(inner.left(), r.bottom() - 8), f"-{SPEC_S:.0f} s")
        p.drawText(QPointF(inner.right() - 30, r.bottom() - 8), "now")

    def draw_gauge(self, p, r):
        panel(p, r, "Direction (mics in a line: left-right only, can't tell front from back)")
        left, right = ("ch0/ch2", "ch1/ch3") if not self.m.swap else ("ch1/ch3", "ch0/ch2")
        cx, base = r.center().x(), r.bottom() - 46
        R = min(r.width() * 0.38, r.height() - 130)
        p.setPen(QPen(EDGE, 1.5))
        p.setBrush(QColor("#0f141a"))
        p.drawPie(QRectF(cx - R, base - R, 2 * R, 2 * R), 0, 180 * 16)
        p.setFont(font(12))
        for a in range(-90, 91, 30):
            t = math.radians(a)
            p.setPen(QPen(DIM, 1.5))
            p.drawLine(QPointF(cx - R * math.sin(t), base - R * math.cos(t)),
                       QPointF(cx - (R - 12) * math.sin(t), base - (R - 12) * math.cos(t)))
            p.drawText(QPointF(cx - (R + 18) * math.sin(t) - 12, base - (R + 18) * math.cos(t) + 4), f"{abs(a)}°")
        p.setFont(font(13, True))
        p.setPen(INK)
        p.drawText(QPointF(cx - R - 40, base + 30), f"LEFT ({left})")
        p.drawText(QPointF(cx + R - 70, base + 30), f"RIGHT ({right})")
        for k in range(2):
            p.setPen(Qt.NoPen)
            p.setBrush(PAIR_COLORS[k])
            p.drawEllipse(QPointF(r.right() - 150, r.y() + 44 + k * 20), 5, 5)
            p.setFont(font(12))
            p.setPen(PAIR_COLORS[k])
            p.drawText(QPointF(r.right() - 140, r.y() + 48 + k * 20), self.pair_name(k))

        now = self.m.now
        det = self.m.last_det
        if det and now - det[0] < 4:
            fade = max(0.25, 1 - (now - det[0]) / 4)
            for k in (1, 0):  # draw the narrow pair first so the wide one sits on top
                a = self.oriented(det[1][k])
                if a is None:
                    continue
                th = math.radians(a)
                c = QColor(PAIR_COLORS[k])
                c.setAlphaF(fade)
                p.setPen(QPen(c, 6 if k == 0 else 3, Qt.SolidLine, Qt.RoundCap))
                p.drawLine(QPointF(cx, base), QPointF(cx - (R - 8) * math.sin(th), base - (R - 8) * math.cos(th)))
            best = 0 if det[1][0] is not None else 1
            a = self.oriented(det[1][best])
            side = "left" if a > 5 else "right" if a < -5 else "centre"
            p.setFont(font(26, True))
            p.setPen(INK)
            p.drawText(QRectF(r.x(), r.y() + 34, r.width(), 34), Qt.AlignHCenter,
                       f"{abs(a):.0f}° {side}" if side != "centre" else "centre")
            p.setFont(font(14, mono=True))
            p.setPen(DIM)
            if None not in det[1]:
                diff = abs(det[1][0] - det[1][1])
                note = "pairs agree" if diff <= 15 else "pairs disagree"
                line = f"{note} (within {diff:.0f}°) · {now - det[0]:.1f} s ago"
            else:
                line = f"only the {self.pair_name(best)} is live · {now - det[0]:.1f} s ago"
            p.drawText(QRectF(r.x(), r.y() + 70, r.width(), 22), Qt.AlignHCenter, line)
        else:
            p.setFont(font(18))
            p.setPen(DIM)
            p.drawText(QRectF(r.x(), r.y() + 40, r.width(), 30), Qt.AlignHCenter,
                       "clap, snap or talk to get a direction")
        p.setPen(Qt.NoPen)
        p.setBrush(INK)
        p.drawEllipse(QPointF(cx, base), 6, 6)

    def draw_levels(self, p, r):
        panel(p, r, "Level (dBFS, last 50 ms)")
        inner = r.adjusted(60, 34, -110, -10)
        row_h = inner.height() / CH

        def xf(db):
            return inner.left() + float(np.clip((db + 80) / 80, 0, 1)) * inner.width()

        for ch in range(CH):
            y = inner.top() + ch * row_h + row_h / 2
            p.setFont(font(13, True))
            p.setPen(CH_COLORS[ch])
            p.drawText(QPointF(r.x() + 14, y + 5), f"ch{ch}")
            bh = min(18.0, row_h - 6)
            bar = QRectF(inner.left(), y - bh / 2, inner.width(), bh)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("#21262d"))
            p.drawRoundedRect(bar, 4, 4)
            p.setFont(font(13, mono=True))
            if self.m.dead[ch]:
                p.setPen(ALERT)
                p.drawText(QPointF(bar.left() + 10, y + 5), "NO SIGNAL: check this mic's 3V, GND, CLK, DAT")
                p.drawText(QPointF(inner.right() + 12, y + 5), "  dead")
                continue
            p.setBrush(CH_COLORS[ch])
            p.drawRoundedRect(QRectF(bar.left(), bar.top(), xf(self.m.level[ch]) - bar.left(), bh), 4, 4)
            p.setPen(QPen(INK, 2))
            xp = xf(self.m.peak[ch])
            p.drawLine(QPointF(xp, bar.top() - 2), QPointF(xp, bar.bottom() + 2))
            p.setPen(QPen(DIM, 1, Qt.DashLine))
            xn = xf(self.m.floor[ch])
            p.drawLine(QPointF(xn, bar.top() - 3), QPointF(xn, bar.bottom() + 3))
            p.setPen(INK)
            p.drawText(QPointF(inner.right() + 12, y + 5), f"{self.m.level[ch]:6.1f}")
        p.setFont(font(11))
        p.setPen(DIM)
        p.drawText(QPointF(r.right() - 270, r.y() + 24), "white = peak · dashed = noise floor")

    def draw_history(self, p, r):
        panel(p, r, f"Direction history (last {HISTORY_S:.0f} s)")
        inner = r.adjusted(60, 34, -14, -24)
        for a in (-90, -45, 0, 45, 90):
            y = inner.center().y() - a / 90 * inner.height() / 2
            p.setPen(QPen(EDGE, 1))
            p.drawLine(QPointF(inner.left(), y), QPointF(inner.right(), y))
            p.setFont(font(11))
            p.setPen(DIM)
            p.drawText(QPointF(r.x() + 10, y + 4), "left" if a == 90 else "right" if a == -90 else f"{abs(a)}°")
        now = self.m.now
        for t, angles, _, strength in self.m.detections:
            age = now - t
            if age > HISTORY_S:
                continue
            x = inner.right() - age / HISTORY_S * inner.width()
            for k in (1, 0):
                a = self.oriented(angles[k])
                if a is None:
                    continue
                y = inner.center().y() - a / 90 * inner.height() / 2
                c = QColor(PAIR_COLORS[k])
                c.setAlphaF(min(1.0, 0.35 + strength / 40))
                p.setPen(Qt.NoPen)
                p.setBrush(c)
                rad = (4 + min(strength, 30) / 6) * (1.0 if k == 0 else 0.6)
                p.drawEllipse(QPointF(x, y), rad, rad)
        p.setFont(font(11))
        p.setPen(DIM)
        p.drawText(QPointF(inner.left(), r.bottom() - 6), f"-{HISTORY_S:.0f} s")
        p.drawText(QPointF(inner.right() - 24, r.bottom() - 6), "now")


# ------------------------------------------------------------------ window

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: find the board automatically)")
    ap.add_argument("--wide", type=float, default=14.0, help="ch0/ch1 spacing in cm")
    ap.add_argument("--narrow", type=float, default=3.5, help="ch2/ch3 spacing in cm")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()

    app = QApplication(sys.argv[:1])
    model = Model([args.wide / 100, args.narrow / 100])
    reader = Reader(args.port)
    reader.start()

    win = QWidget()
    win.setWindowTitle("Four-mic live view")
    win.setStyleSheet("QWidget { background: #0d1117; color: #e6edf3; font-size: 14px; }"
                      "QPushButton { background: #21262d; border: 1px solid #30363d; border-radius: 6px;"
                      " padding: 6px 14px; }"
                      "QPushButton:checked { background: #da3633; border-color: #ff7b72; }"
                      "QDoubleSpinBox { background: #21262d; border: 1px solid #30363d; padding: 3px; }")
    view = View(model)
    status = QLabel()
    status.setStyleSheet("color: #8b949e; font-size: 12px;")
    rec = QPushButton("● Record")
    rec.setCheckable(True)
    spins = []
    for k, val in enumerate((args.wide, args.narrow)):
        sp = QDoubleSpinBox()
        sp.setRange(1, 60)
        sp.setDecimals(1)
        sp.setSuffix(" cm")
        sp.setValue(val)
        sp.valueChanged.connect(lambda v, k=k: model.spacing.__setitem__(k, v / 100))
        spins.append(sp)
    swap = QCheckBox("Swap left/right")
    thresh = QSlider(Qt.Horizontal)
    thresh.setRange(4, 30)
    thresh.setValue(int(model.threshold_db))
    thresh.setFixedWidth(140)
    thresh_lbl = QLabel()
    clear = QPushButton("Clear history")

    row = QHBoxLayout()
    for wdg in (rec, QLabel("ch0–ch1"), spins[0], QLabel("ch2–ch3"), spins[1], swap,
                QLabel("Trigger"), thresh, thresh_lbl, clear):
        row.addWidget(wdg)
    row.addStretch(1)
    row.addWidget(status)
    lay = QVBoxLayout(win)
    lay.setContentsMargins(8, 8, 8, 8)
    lay.addLayout(row)
    lay.addWidget(view, 1)

    def on_thresh(v):
        model.threshold_db = float(v)
        thresh_lbl.setText(f"{v} dB over floor")

    def on_rec(checked):
        if checked != bool(model.recorder):
            model.toggle_record()
        rec.setText("■ Stop recording" if model.recorder else "● Record")

    thresh.valueChanged.connect(on_thresh)
    on_thresh(thresh.value())
    swap.toggled.connect(lambda v: setattr(model, "swap", v))
    clear.clicked.connect(model.detections.clear)
    rec.toggled.connect(on_rec)

    def tick():
        model.push(reader.take())
        recs = f"  ● REC {model.record_path.name}" if model.recorder else ""
        status.setText(f"{reader.port or '-'} · {reader.status} · {model.rate / 1000:5.2f} kHz · "
                       f"packets {reader.packets} · gaps {reader.gaps}{recs}")
        view.update()

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(33)
    win.resize(1500, 950)
    win.show()
    if args.screenshot:
        QTimer.singleShot(6000, lambda: (win.grab().save(args.screenshot), app.quit()))
    app.exec()
    reader.stop = True
    if model.recorder:
        model.recorder.close()


if __name__ == "__main__":
    main()
