#!/usr/bin/env python3
"""Review recordings from the board's microSD card (cam_mic_record firmware), like the live viewers.

    python3 tools/take_viewer.py                  takes in mic_takes/sd/, takes/ and any card that's plugged in
    python3 tools/take_viewer.py path/to/folder   other folders: processed takes, a card, or copied take folders

The middle shows a take the way mic_caption_view.py shows the live board: the camera with the sound
direction overlay and subtitles, the direction gauge and the mic levels. Under it, every mic channel and
the cleaned voice scroll past, and below that is the whole take: where each sound came from (click to
jump). Listen to the cleaned voice, the raw mic, or any single channel. Takes that haven't been through
tools/sd_import.py yet can be processed from here.

Keys: space = play / pause, left / right arrow = 2 s back / forward, A = voice focus <-> raw mic,
S = subtitles on / off, D = direction overlay on / off.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(os.path.abspath(__file__)).parent
ENV_PY = Path.home() / ".mic-caption-env" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")

try:
    import faster_whisper  # noqa: F401  (needed by sd_import)
    import numpy  # noqa: F401
    from PySide6 import QtMultimedia  # noqa: F401
except ImportError:
    if ENV_PY.exists() and Path(sys.prefix).resolve() != ENV_PY.parent.parent.resolve():
        os.execv(str(ENV_PY), [str(ENV_PY), os.path.abspath(__file__), *sys.argv[1:]])
    sys.exit("missing packages: run mic_caption_view.py once to set up ~/.mic-caption-env")

import argparse  # noqa: E402
import bisect  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import re  # noqa: E402
import signal  # noqa: E402
import struct  # noqa: E402
import subprocess  # noqa: E402
import tempfile  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import wave  # noqa: E402
from dataclasses import dataclass  # noqa: E402

import numpy as np  # noqa: E402
from PySide6.QtCore import QPointF, QProcess, QRectF, Qt, QTimer, QUrl, Signal, qInstallMessageHandler  # noqa: E402
from PySide6.QtGui import QColor, QDesktopServices, QImage, QKeySequence, QPainter, QPen, QPolygonF, QShortcut  # noqa: E402
from PySide6.QtMultimedia import QAudioOutput, QMediaPlayer  # noqa: E402
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QHBoxLayout, QLabel, QListWidget,  # noqa: E402
                               QListWidgetItem, QMainWindow, QPushButton, QSlider, QSplitter, QVBoxLayout,
                               QWidget)

import sd_import as sd  # noqa: E402  (also puts the live viewers on the path)
import mic_camera_view as cv  # noqa: E402
import mic_caption_view as cap  # noqa: E402

FS = cv.FS
CH = cv.CH
PROCESSED = sd.TAKES_OUT
COPIED = HERE.parent / "takes"
SD_IMPORT = HERE / "sd_import.py"
LEVEL_BLOCK = 800  # 50 ms, the live viewer's level window
ENV_BLOCK = 80  # 5 ms per point in the channel view
SUB_HOLD_S = sd.SUB_HOLD_S
ROW_NAMES = ["ch0 · 140 mm pair", "ch1 · 140 mm pair", "ch2 · 35 mm pair", "ch3 · 35 mm pair",
             "voice focus (vs ch1)"]
FOCUS_ROW = CH
# (label, file in the processed folder or None for one raw channel, channel-view row)
LISTEN = [("Voice focus (processed)", "voice_focus.wav", FOCUS_ROW), ("Raw mic (same loudness)", "raw_mic.wav", 1)]
LISTEN += [(f"Raw ch{c} only", None, c) for c in range(CH)]
MUTED = "#8b949e"


# ------------------------------------------------------------------ finding takes

@dataclass
class Take:
    name: str
    raw: Path | None = None  # as recorded on the card
    out: Path | None = None  # after sd_import.py
    duration: float = 0.0
    mode: str = ""

    @property
    def processed(self) -> bool:
        return self.out is not None


def is_raw(d: Path) -> bool:
    return (d / "info.txt").exists() and any(d.glob("audio_*.wav"))


def is_processed(d: Path) -> bool:
    return (d / "voice_focus.wav").exists() and (d / "raw_4mic.wav").exists()


def default_folders() -> list[Path]:
    return [PROCESSED, COPIED, *sorted(Path("/Volumes").glob("*/takes"))]


def scan(folders: list[Path]) -> list[Take]:
    takes: dict[str, Take] = {}

    def add(d: Path):
        t = takes.setdefault(d.name, Take(d.name))
        if is_processed(d):
            t.out = t.out or d
        elif is_raw(d):
            t.raw = t.raw or d

    for f in folders:
        if not f.is_dir():
            continue
        if is_processed(f) or is_raw(f):
            add(f)
            continue
        root = f / "takes" if (f / "takes").is_dir() else f
        for d in sorted(root.glob("T[0-9]*")):
            if d.is_dir():
                add(d)
    for t in takes.values():
        if t.out is None and is_processed(PROCESSED / t.name):
            t.out = PROCESSED / t.name
        if t.raw:
            info = sd.read_info(t.raw)
            t.duration = float(info.get("duration_s") or 0)
            t.mode = info.get("mode", "")
        elif t.out and (t.out / "transcript.txt").exists():
            head = (t.out / "transcript.txt").read_text(errors="replace").splitlines()[:1]
            m = re.match(r"\S+\s+([\d.]+) s\s+(.*)", head[0]) if head else None
            if m:
                t.duration, t.mode = float(m.group(1)), m.group(2).strip()
    return sorted(takes.values(), key=lambda t: t.name, reverse=True)


# ------------------------------------------------------------------ one take, ready to play back

def load_wav(path: Path) -> np.ndarray:
    """(frames, channels) int16, memory-mapped so long takes don't fill the memory."""
    with open(path, "rb") as f:
        head = f.read(65536)
    i = head.find(b"data")
    (nch,) = struct.unpack_from("<H", head, 22)
    n = (path.stat().st_size - i - 8) // (2 * nch)
    return np.memmap(path, "<i2", "r", offset=i + 8, shape=(n, nch))


def level_tracks(audio: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per 50 ms: each mic's level (dBFS) and the live viewer's running noise floor."""
    nb = len(audio) // LEVEL_BLOCK
    lv = np.empty((nb, CH))
    step = 4096
    for b in range(0, nb, step):
        e = min(nb, b + step)
        x = np.asarray(audio[b * LEVEL_BLOCK:e * LEVEL_BLOCK], np.float32).reshape(e - b, LEVEL_BLOCK, CH) / 32768
        lv[b:e] = 10 * np.log10(x.var(axis=1) + 1e-12)
    fl = np.empty_like(lv)
    f = lv[0].copy() if nb else np.full(CH, -70.0)
    for k in range(nb):
        if k < 10:
            f = np.minimum(f, lv[k]) if k > 2 else lv[k].copy()
        else:
            f = np.where(lv[k] < f, 0.7 * f + 0.3 * lv[k], f + 0.03)
        fl[k] = f
    return lv, fl


def envelopes(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Min and max per 5 ms block, for drawing the waveform at any zoom."""
    nb = len(x) // ENV_BLOCK
    mins = np.empty((nb, x.shape[1]), np.int16)
    maxs = np.empty_like(mins)
    step = 65536
    for b in range(0, nb, step):
        e = min(nb, b + step)
        blk = np.asarray(x[b * ENV_BLOCK:e * ENV_BLOCK]).reshape(e - b, ENV_BLOCK, x.shape[1])
        mins[b:e], maxs[b:e] = blk.min(axis=1), blk.max(axis=1)
    return mins, maxs


def read_dets(out: Path) -> list[tuple]:
    """directions.csv back into the live viewer's detections: (t, angle, sigma, agree, strength, [pair angles])."""
    dets = []
    try:
        with open(out / "directions.csv", newline="") as f:
            for r in csv.DictReader(f):
                try:
                    pairs = [float(r[k]) if r[k] else None for k in ("wide_pair_deg", "narrow_pair_deg")]
                    agree = None if r["pairs_agree"] == "" else r["pairs_agree"] == "1"
                    dets.append((float(r["time_s"]), float(r["mic_angle_deg"]), float(r["uncertainty_deg"]), agree,
                                 float(r["loudness_db"]), pairs))
                except (KeyError, ValueError):
                    pass
    except OSError:
        pass
    return dets


LINE = re.compile(r"^(\S+)–(\S+)  (?:(.*?)  )?(.+)$")


def parse_clock(s: str) -> float:
    total = 0.0
    for part in s.split(":"):
        total = total * 60 + float(part)
    return total


def read_captions(out: Path, calib: sd.Calib) -> tuple[list, list]:
    """([(start, end, text, mic angle)], [(time, mic angle)]) from captions.json, or from transcript.txt
    for takes processed before captions.json existed."""
    try:
        d = json.loads((out / "captions.json").read_text())
        return [tuple(c) for c in d["captions"]], [tuple(b) for b in d["bangs"]]
    except (OSError, ValueError, KeyError):
        pass
    caps, bangs = [], []
    try:
        rows = (out / "transcript.txt").read_text(errors="replace").splitlines()[2:]
    except OSError:
        return caps, bangs
    for row in rows:
        m = LINE.match(row.strip())
        if not m:
            continue
        where, text = m.group(3) or "", m.group(4)
        a = re.match(r"(\d+)° (left|right)", where)
        rel = (float(a.group(1)) * (1 if a.group(2) == "left" else -1)) if a else (0.0 if "ahead" in where else None)
        mic = None if rel is None else (-(rel + calib.offset) if calib.swap else rel + calib.offset)
        if text.startswith("[BANG"):
            bangs.append((parse_clock(m.group(1)), mic))
        else:
            caps.append((parse_clock(m.group(1)), parse_clock(m.group(2)), text, mic))
    return caps, bangs


class TakeData:
    def __init__(self, take: Take, calib: sd.Calib):
        out = take.out
        self.take = take
        self.audio = load_wav(out / "raw_4mic.wav")
        focus = load_wav(out / "voice_focus.wav")
        self.duration = len(self.audio) / FS
        cam = out / "camera.bin"
        if not cam.exists() and take.raw:
            cam = take.raw / "video.bin"
        self.frames = sd.read_frames(cam)
        self.frame_times = [s / FS for s, _ in self.frames]
        self.dets = read_dets(out)
        self.det_times = [d[0] for d in self.dets]
        self.captions, self.bangs = read_captions(out, calib)
        self.levels, self.floors = level_tracks(self.audio)
        rmin, rmax = envelopes(self.audio)
        fmin, fmax = envelopes(focus[:len(self.audio)])
        n = min(len(rmin), len(fmin))
        self.env_min = np.concatenate([rmin[:n], fmin[:n, :1]], axis=1)
        self.env_max = np.concatenate([rmax[:n], fmax[:n, :1]], axis=1)
        peak = np.maximum(np.abs(self.env_min.astype(np.int32)), np.abs(self.env_max.astype(np.int32)))
        raw_peak = float(np.percentile(peak[:, :CH], 99.9)) if n else 1.0
        # voice_focus.wav has the same gain as raw_mic.wav, so on raw_mic's scale the removed noise shows
        ref = np.abs(np.asarray(load_wav(out / "raw_mic.wav")[::4, 0], np.int32))
        focus_peak = float(np.percentile(ref, 99.9)) if len(ref) else 1.0
        self.row_scale = [max(raw_peak, 50.0)] * CH + [max(focus_peak, 50.0)]
        info = sd.read_info(take.raw) if take.raw else {}
        self.lost = int(info.get("lost_audio_frames") or 0)
        self.mode = take.mode
        self._tmp = None

    def channel_wav(self, ch: int) -> Path:
        """One raw channel on its own, all four at the same gain so their levels can be compared."""
        if self._tmp is None:
            self._tmp = tempfile.TemporaryDirectory(prefix="take_viewer_")
        path = Path(self._tmp.name) / f"ch{ch}.wav"
        if not path.exists():
            x = np.asarray(self.audio[:, ch], np.float64)
            x -= x.mean()
            gain = min(30.0, 16000 / self.row_scale[0])
            with wave.open(str(path), "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(FS)
                w.writeframes(np.clip(np.round(x * gain), -32767, 32767).astype("<i2").tobytes())
        return path


# ------------------------------------------------------------------ the live viewer, fed from a recording

class PlaybackModel:
    """What mic_camera_view's View reads from its Model, for one moment of a recording."""

    def __init__(self):
        self.now = 0.0
        self.detections: list = []
        self.last_det = None
        self.level = np.full(CH, -120.0)
        self.floor = np.full(CH, -70.0)
        self.dead = np.zeros(CH, bool)
        self.rate = float(FS)


class PlaybackReader:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.fps = 0.0
        self.gaps = 0
        self.board_msgs: list = []
        self.board_stats = ""


class PlaybackCaptions:
    def __init__(self):
        self.lock = threading.Lock()
        self.lines: list = []
        self.partial = None
        self.speaking_angle = None
        self.status = ""


class TakeView(cap.CaptionView):
    def __init__(self, calib: sd.Calib):
        self.pm, self.pr, self.pc = PlaybackModel(), PlaybackReader(), PlaybackCaptions()
        super().__init__(self.pm, self.pr, self.pc)
        self.setMinimumSize(560, 380)
        self.hfov, self.offset, self.swap = calib.hfov, calib.offset, calib.swap
        self.rotate, self.mirror = calib.rotate, calib.mirror
        self.show_overlay = True
        self.data: TakeData | None = None
        self.listening = ""

    def load(self, data: TakeData | None):
        self.data = data
        self.image = self.image_seq = None
        self.pr.frame = None
        if data is None:
            return
        self.pr.fps = len(data.frames) / data.duration if data.duration else 0.0
        self.pr.gaps = data.lost
        if not data.frames:
            note = ("voice-only take: no camera" if data.mode == "voice"
                    else "camera frames not found: process this take again from the card")
            self.image = self.placeholder(note)
        self.seek(0.0)

    @staticmethod
    def placeholder(text: str) -> QImage:
        img = QImage(640, 480, QImage.Format.Format_RGB888)
        img.fill(QColor("#0f141a"))
        p = QPainter(img)
        p.setPen(cv.DIM)
        p.setFont(cv.font(22))
        p.drawText(img.rect(), Qt.AlignmentFlag.AlignCenter, text + "\n(sound directions still show)")
        p.end()
        return img

    def seek(self, t: float):
        d = self.data
        if d is None:
            return
        m = self.pm
        m.now = t
        hi = bisect.bisect_right(d.det_times, t)
        m.detections = d.dets[bisect.bisect_left(d.det_times, t - cv.HOLD_S):hi] if self.show_overlay else []
        m.last_det = d.dets[hi - 1] if hi else None
        k = min(max(int(t * FS) // LEVEL_BLOCK - 1, 0), len(d.levels) - 1)
        if k >= 0:
            m.level, m.floor = d.levels[k], d.floors[k]
        n = int(t * FS)
        seg = d.audio[max(0, n - FS // 2):n]
        m.dead = (np.ptp(seg, axis=0) < cv.DEAD_PTP) if len(seg) >= FS // 4 else np.zeros(CH, bool)
        if d.frames:
            i = max(0, bisect.bisect_right(d.frame_times, t) - 1)
            self.pr.frame = (i, d.frames[i][1])
        wall = time.time()
        lines = [{"wall": wall, "text": txt, "angle": ang, "kind": "speech"}
                 for s, e, txt, ang in d.captions if s <= t <= e + SUB_HOLD_S]
        lines += [{"wall": wall, "text": "", "angle": ang, "kind": "sound"}
                  for tb, ang in d.bangs if tb <= t <= tb + SUB_HOLD_S]
        self.pc.lines = lines
        self.pc.speaking_angle = next((ang for s, e, _, ang in d.captions if s - 0.3 <= t <= e), None)
        self.update_frame()
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), cv.BG)
        w, h = self.width(), self.height()
        gap = 12
        left_w = w * 0.64
        cam_r = QRectF(gap, gap, left_w - 1.5 * gap, h - 2 * gap)
        x1 = left_w + 0.5 * gap
        cw = w - x1 - gap
        gauge_r = QRectF(x1, gap, cw, h * 0.50)
        lvl_r = QRectF(x1, gauge_r.bottom() + gap, cw, h * 0.25)
        info_r = QRectF(x1, lvl_r.bottom() + gap, cw, h - lvl_r.bottom() - 2 * gap)
        self.draw_camera(p, cam_r)
        self.draw_gauge(p, gauge_r)
        self.draw_levels(p, lvl_r)
        self.draw_info(p, info_r)
        p.end()

    def draw_info(self, p, r):
        cv.panel(p, r, "Take")
        d = self.data
        if d is None:
            return
        mode = {"voice+video": "voice + video", "voice": "voice only"}.get(d.mode, d.mode)
        rows = [f"{d.take.name} · {d.duration:.1f} s · {mode}",
                f"camera: {len(d.frames)} frames · {self.pr.fps:.1f} fps" if d.frames else "camera: none",
                f"lost audio: {d.lost} frames · {len(d.dets)} sounds located",
                f"listening to: {self.listening}"]
        p.setFont(cv.font(13, mono=True))
        p.setPen(cv.INK)
        y = r.y() + 48
        for row in rows:
            if y > r.bottom() - 6:
                break
            p.drawText(QPointF(r.x() + 14, y), row)
            y += 21


# ------------------------------------------------------------------ channel view and whole-take strip

class ChannelStream(QWidget):
    """Every mic channel and the cleaned voice scrolling past, like an oscilloscope."""

    seek = Signal(float)
    BEFORE, AFTER = 4.0, 1.0
    LABEL_W = 150

    def __init__(self):
        super().__init__()
        self.data: TakeData | None = None
        self.t = 0.0
        self.listen_row = FOCUS_ROW
        self.setMinimumHeight(220)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Raw mics (same scale, so their levels compare) and the cleaned voice. "
                        "Blue = speech, orange ticks = located sounds. Click to jump.")

    def set_time(self, t: float):
        self.t = t
        self.update()

    def _x(self, r: QRectF, t: float) -> float:
        return r.left() + (t - (self.t - self.BEFORE)) / (self.BEFORE + self.AFTER) * r.width()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), cv.BG)
        outer = QRectF(6, 4, self.width() - 12, self.height() - 8)
        cv.panel(p, outer, "Channels")
        d = self.data
        r = QRectF(outer.left() + self.LABEL_W, outer.top() + 34, outer.width() - self.LABEL_W - 14,
                   outer.height() - 52)
        if d is None or r.width() < 20:
            return
        t0, t1 = self.t - self.BEFORE, self.t + self.AFTER
        p.save()
        p.setClipRect(r)
        for s, e, _, _ in d.captions:
            if e >= t0 and s <= t1:
                band = QColor(cap.SPEAKER)
                band.setAlpha(28)
                p.fillRect(QRectF(self._x(r, s), r.top(), self._x(r, e) - self._x(r, s), r.height()), band)
        p.setPen(QPen(cv.GLOW, 2))
        for td in d.det_times[bisect.bisect_left(d.det_times, t0):bisect.bisect_right(d.det_times, t1)]:
            x = self._x(r, td)
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.top() + 7))
        p.restore()

        rows = CH + 1
        row_h = r.height() / rows
        px = max(int(r.width() / 2), 10)
        nb = len(d.env_min)
        edges = np.floor((t0 + (t1 - t0) * np.arange(px + 1) / px) * FS / ENV_BLOCK).astype(int)
        cols = np.nonzero((edges[:-1] >= 0) & (edges[:-1] < nb))[0]
        for row in range(rows):
            top = r.top() + row * row_h
            mid = top + row_h / 2
            color = cap.SPEAKER if row == FOCUS_ROW else cv.CH_COLORS[row]
            listening = row == self.listen_row
            p.setFont(cv.font(12, listening))
            p.setPen(color if listening else cv.DIM)
            p.drawText(QRectF(outer.left() + 12, top, self.LABEL_W - 16, row_h),
                       Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                       ("▶ " if listening else "") + ROW_NAMES[row])
            p.setPen(QPen(QColor("#21262d"), 1))
            p.drawLine(QPointF(r.left(), mid), QPointF(r.right(), mid))
            if len(cols) == 0:
                continue
            lo = edges[cols]
            start, end = lo[0], max(min(int(edges[cols[-1] + 1]), nb), lo[-1] + 1)
            mn = np.minimum.reduceat(d.env_min[start:end, row], lo - start)
            mx = np.maximum.reduceat(d.env_max[start:end, row], lo - start)
            scale = (row_h / 2 - 2) / d.row_scale[row]
            xs = r.left() + (cols + 0.5) * r.width() / px
            ys_top = np.clip(mid - mx * scale, top + 1, top + row_h - 1)
            ys_bot = np.clip(mid - mn * scale, top + 1, top + row_h - 1)
            poly = QPolygonF([QPointF(x, y) for x, y in zip(xs, ys_top)] +
                             [QPointF(x, y) for x, y in zip(xs[::-1], ys_bot[::-1])])
            fill = QColor(color)
            fill.setAlpha(230 if listening else 150)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(fill)
            p.drawPolygon(poly)
        p.setFont(cv.font(11))
        for s in range(int(np.ceil(t0)), int(t1) + 1):
            if s < 0:
                continue
            x = self._x(r, s)
            p.setPen(QPen(QColor("#30363d"), 1))
            p.drawLine(QPointF(x, r.bottom()), QPointF(x, r.bottom() + 4))
            p.setPen(cv.DIM)
            p.drawText(QRectF(x - 30, r.bottom() + 2, 60, 14), Qt.AlignmentFlag.AlignCenter, sd.clock(s)[:-2])
        x = self._x(r, self.t)
        p.setPen(QPen(cv.INK, 2))
        p.drawLine(QPointF(x, r.top() - 4), QPointF(x, r.bottom()))

    def mousePressEvent(self, e):
        r_left = 6 + self.LABEL_W
        width = self.width() - 12 - self.LABEL_W - 14
        if e.position().x() >= r_left and width > 0:
            frac = (e.position().x() - r_left) / width
            self.seek.emit(max(0.0, self.t - self.BEFORE + frac * (self.BEFORE + self.AFTER)))


class DirectionStrip(QWidget):
    """The whole take: where each located sound came from (left at the top, right at the bottom)."""

    seek = Signal(float)
    LABEL_W = 64

    def __init__(self, calib: sd.Calib):
        super().__init__()
        self.c = calib
        self.points: list[tuple[float, float, float]] = []
        self.speech: list[tuple[float, float]] = []
        self.duration = 0.1
        self.pos = 0.0
        self.setMinimumHeight(96)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Whole take: where each sound came from. Blue = speech, light band = in the picture. "
                        "Click to jump.")

    def set_data(self, data: TakeData | None):
        self.points = [] if data is None else [(t, self.c.rel(a), s) for t, a, _, _, s, _ in data.dets]
        self.speech = [] if data is None else [(s, e) for s, e, _, _ in data.captions]
        self.duration = max(data.duration, 0.1) if data else 0.1
        self.pos = 0.0
        self.update()

    def set_position(self, t: float):
        self.pos = t
        self.update()

    def _plot(self) -> QRectF:
        return QRectF(self.LABEL_W, 6, max(self.width() - self.LABEL_W - 10, 10), max(self.height() - 12, 10))

    def _x(self, r: QRectF, t: float) -> float:
        return r.left() + r.width() * min(max(t / self.duration, 0.0), 1.0)

    def _y(self, r: QRectF, angle: float) -> float:
        return r.top() + r.height() * (90 - min(max(angle, -90), 90)) / 180

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), cv.BG)
        r = self._plot()
        p.fillRect(r, cv.PANEL)
        half = self.c.hfov / 2
        p.fillRect(QRectF(r.left(), self._y(r, half), r.width(), self._y(r, -half) - self._y(r, half)),
                   QColor(255, 255, 255, 14))
        p.setPen(cv.DIM)
        p.setFont(cv.font(11))
        for angle, text in ((90, "left"), (0, "ahead"), (-90, "right")):
            y = min(max(self._y(r, angle), r.top() + 7), r.bottom() - 7)
            p.drawText(QRectF(0, y - 8, self.LABEL_W - 8, 16),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, text)
        for s, e in self.speech:
            band = QColor(cap.SPEAKER)
            band.setAlpha(45)
            p.fillRect(QRectF(self._x(r, s), r.top(), max(self._x(r, e) - self._x(r, s), 2), r.height()), band)
        p.setPen(Qt.PenStyle.NoPen)
        for t, rel, loud in self.points:
            c = QColor(cv.LOUD if loud > 30 else cv.GLOW)
            c.setAlpha(int(min(255, 90 + loud * 6)))
            p.setBrush(c)
            rad = 1.5 + min(loud, 30) / 10
            p.drawEllipse(QPointF(self._x(r, t), self._y(r, rel)), rad, rad)
        x = self._x(r, self.pos)
        p.setPen(QPen(cv.INK, 2))
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))

    def _seek_to(self, x: float):
        r = self._plot()
        self.seek.emit(self.duration * min(max((x - r.left()) / r.width(), 0.0), 1.0))

    def mousePressEvent(self, e):
        self._seek_to(e.position().x())

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.MouseButton.LeftButton:
            self._seek_to(e.position().x())


# ------------------------------------------------------------------ window

class Viewer(QMainWindow):
    def __init__(self, folders: list[Path]):
        super().__init__()
        self.folders = folders
        self.calib = sd.Calib(argparse.Namespace(hfov=None, offset=None, swap=None, rotate=None, mirror=None))
        self.takes: list[Take] = []
        self.current: Take | None = None
        self.data: TakeData | None = None
        self.pending_pos: int | None = None
        self.proc: QProcess | None = None
        self._pos_ms, self._pos_wall = 0, time.monotonic()
        self.target_ms: int | None = None
        self.setWindowTitle("Take viewer")

        self.player = QMediaPlayer(self)
        self.audio_out = QAudioOutput(self)
        self.player.setAudioOutput(self.audio_out)

        # left: takes
        self.take_list = QListWidget()
        self.take_list.currentRowChanged.connect(self._take_selected)
        self.process_btn = QPushButton("Process")
        self.process_btn.setToolTip("Run sd_import.py on this take: voice focus, directions, transcript, video")
        self.process_btn.clicked.connect(self._process)
        refresh = QPushButton("Refresh")
        refresh.setToolTip("Look again for takes (e.g. after plugging in the card)")
        refresh.clicked.connect(lambda: self.refresh())
        files = QPushButton("Show files")
        files.clicked.connect(self._show_files)
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(self._heading("Takes"))
        lv.addWidget(self.take_list, 1)
        row = QHBoxLayout()
        for b in (self.process_btn, refresh, files):
            row.addWidget(b)
        lv.addLayout(row)

        # middle: the live-viewer layout, channels, whole-take strip, controls
        self.view = TakeView(self.calib)
        self.stream = ChannelStream()
        self.stream.seek.connect(self._seek)
        self.strip = DirectionStrip(self.calib)
        self.strip.seek.connect(self._seek)
        self.play_btn = QPushButton("Play")
        self.play_btn.setFixedWidth(80)
        self.play_btn.clicked.connect(self._toggle_play)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.sliderMoved.connect(lambda ms: self._seek(ms / 1000))
        self.time_lbl = QLabel("00:00.0 / 00:00.0")
        self.time_lbl.setStyleSheet(f"color: {MUTED}; font-family: Menlo, monospace;")
        self.time_lbl.setFixedWidth(150)  # a label that changes width relayouts the window every tick
        self.listen_box = QComboBox()
        self.listen_box.addItems([label for label, _, _ in LISTEN])
        self.listen_box.setToolTip("What you hear. A switches between voice focus and raw mic.")
        self.listen_box.currentIndexChanged.connect(self._listen_changed)
        self.subs_box = QCheckBox("Subtitles")
        self.subs_box.setChecked(True)
        self.subs_box.toggled.connect(self._subs_toggled)
        self.overlay_box = QCheckBox("Direction overlay")
        self.overlay_box.setChecked(True)
        self.overlay_box.toggled.connect(self._overlay_toggled)
        controls = QHBoxLayout()
        controls.addWidget(self.play_btn)
        controls.addWidget(self.slider, 1)
        controls.addWidget(self.time_lbl)
        controls.addWidget(QLabel("Listen:"))
        controls.addWidget(self.listen_box)
        controls.addWidget(self.subs_box)
        controls.addWidget(self.overlay_box)
        mid = QWidget()
        mv = QVBoxLayout(mid)
        mv.setContentsMargins(0, 0, 0, 0)
        mv.setSpacing(4)
        mv.addWidget(self.view, 1)
        mv.addWidget(self.stream)
        mv.addWidget(self.strip)
        mv.addLayout(controls)

        # right: transcript
        self.transcript = QListWidget()
        self.transcript.setWordWrap(True)
        self.transcript.itemClicked.connect(self._line_clicked)
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(self._heading("Transcript"))
        rv.addWidget(self.transcript, 1)

        split = QSplitter()
        for wdg in (left, mid, right):
            split.addWidget(wdg)
        split.setStretchFactor(1, 1)
        split.setSizes([230, 1100, 260])
        self.setCentralWidget(split)
        self.status = self.statusBar()

        self.player.playbackStateChanged.connect(
            lambda s: self.play_btn.setText("Pause" if s == QMediaPlayer.PlaybackState.PlayingState else "Play"))
        self.player.mediaStatusChanged.connect(self._media_status)
        self.player.errorOccurred.connect(lambda _, msg: self.status.showMessage(f"Can't play: {msg}"))

        for key, fn in ((Qt.Key.Key_Space, self._toggle_play),
                        (Qt.Key.Key_Left, lambda: self._seek(self._now() - 2)),
                        (Qt.Key.Key_Right, lambda: self._seek(self._now() + 2)),
                        (Qt.Key.Key_A, lambda: self.listen_box.setCurrentIndex(0 if self.listen_box.currentIndex() else 1)),
                        (Qt.Key.Key_S, self.subs_box.toggle),
                        (Qt.Key.Key_D, self.overlay_box.toggle)):
            QShortcut(QKeySequence(key), self, activated=fn)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)
        self.refresh()

    @staticmethod
    def _heading(text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet("font-size: 15px; font-weight: 600; padding: 4px 2px;")
        return lbl

    # ---------------------------------------------------------------- takes

    def refresh(self, select: str | None = None):
        keep = select or (self.current.name if self.current else None)
        self.takes = scan(self.folders)
        self.take_list.blockSignals(True)
        self.take_list.clear()
        for t in self.takes:
            mode = {"voice+video": "voice + video", "voice": "voice only"}.get(t.mode, t.mode)
            item = QListWidgetItem(f"{t.name}   {t.duration:.1f} s\n{mode}" + ("" if t.processed else " · not processed"))
            if not t.processed:
                item.setForeground(QColor(MUTED))
            self.take_list.addItem(item)
        if not self.takes:
            self.take_list.blockSignals(False)
            self.current = None
            self._load(None)
            self.status.showMessage(f"No takes found. Plug in the card or copy takes into {COPIED}, then Refresh.")
            self.process_btn.setEnabled(False)
            return
        names = [t.name for t in self.takes]
        row = names.index(keep) if keep in names else 0
        self.take_list.setCurrentRow(row)
        self.take_list.blockSignals(False)
        self._take_selected(row)

    def _take_selected(self, row: int):
        if not 0 <= row < len(self.takes):
            return
        t = self.current = self.takes[row]
        self.process_btn.setEnabled(t.raw is not None and self.proc is None)
        self.process_btn.setText("Process again" if t.processed else "Process")
        if not t.processed:
            self._load(None)
            self.status.showMessage(f"{t.name} isn't processed yet: press Process (cleans the voice, locates the "
                                    "sounds, transcribes it)")
            return
        try:
            self._load(TakeData(t, self.calib))
            self.status.showMessage(f"{t.name}: {self.data.duration:.1f} s, {len(self.data.dets)} sounds located, "
                                    f"{len(self.data.captions)} lines of speech")
        except Exception as e:  # noqa: BLE001
            self._load(None)
            self.status.showMessage(f"Couldn't open {t.name}: {e}")

    def _load(self, data: TakeData | None):
        self.player.stop()
        self.data = data
        self.view.load(data)
        self.stream.data = data
        self.strip.set_data(data)
        self.transcript.clear()
        self.slider.setRange(0, int(data.duration * 1000) if data else 0)
        if data is None:
            self.player.setSource(QUrl())
            self.view.update()
            self.stream.update()
            return
        for s, _, text, ang in data.captions:
            where = self.calib.side(self.calib.rel(ang))
            self.transcript.addItem(QListWidgetItem(f"{sd.clock(s)}   {where}\n{text}" if where else f"{sd.clock(s)}\n{text}"))
        if not data.captions:
            self.transcript.addItem(QListWidgetItem("(no speech found)"))
        self._set_source(0)
        self.player.play()

    def _process(self):
        t = self.current
        if not t or not t.raw or self.proc:
            return
        self.player.stop()
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._process_output)
        self.proc.finished.connect(lambda code, _: self._process_done(t.name, code))
        self.process_btn.setEnabled(False)
        self.status.showMessage(f"Processing {t.name}…")
        self.proc.start(sys.executable, [str(SD_IMPORT), str(t.raw), "--out", str(PROCESSED)])

    def _process_output(self):
        text = bytes(self.proc.readAllStandardOutput()).decode(errors="replace").strip()
        if text:
            self.status.showMessage(text.splitlines()[-1].strip())

    def _process_done(self, name: str, code: int):
        self.proc = None
        self.refresh(select=name)
        if code:
            self.status.showMessage(f"processing {name} failed (exit {code})")

    def _show_files(self):
        t = self.current
        path = t and (t.out or t.raw)
        if not path:
            return
        if sys.platform == "darwin":
            subprocess.run(["open", str(path)])
        else:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    # ---------------------------------------------------------------- playback

    def _set_source(self, pos_ms: int):
        d = self.data
        label, name, row = LISTEN[self.listen_box.currentIndex()]
        path = d.take.out / name if name else d.channel_wav(row)
        self.pending_pos = pos_ms or None
        self.target_ms, self._pos_wall = pos_ms, time.monotonic()
        self.player.setSource(QUrl.fromLocalFile(str(path)))
        self.stream.listen_row = row
        self.view.listening = label

    def _now(self) -> float:
        pos = self.player.position()
        now = time.monotonic()
        if self.target_ms is not None:
            # the player reports the old position for a moment after a seek or a new source
            if abs(pos - self.target_ms) > 120 and now - self._pos_wall < 1.0:
                return self.target_ms / 1000
            self.target_ms = None
        if pos != self._pos_ms:
            self._pos_ms, self._pos_wall = pos, now
        t = pos / 1000
        if self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            t += min(now - self._pos_wall, 0.25)
        return t

    def _tick(self):
        d = self.data
        if d is None:
            return
        t = min(self._now(), d.duration)
        self.view.seek(t)
        self.stream.set_time(t)
        self.strip.set_position(t)
        if not self.slider.isSliderDown():
            self.slider.setValue(int(t * 1000))
        self.time_lbl.setText(f"{sd.clock(t)} / {sd.clock(d.duration)}")
        now = next((i for i, (s, e, _, _) in enumerate(d.captions) if s <= t <= e + 0.3), -1)
        if now >= 0 and now != self.transcript.currentRow():
            self.transcript.setCurrentRow(now)

    def _toggle_play(self):
        if self.data is None:
            return
        if self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState:
            self.player.pause()
        else:
            if self.player.mediaStatus() == QMediaPlayer.MediaStatus.EndOfMedia:
                self.player.setPosition(0)
            self.player.play()

    def _seek(self, t: float, play: bool = False):
        if self.data is None:
            return
        t = min(max(t, 0.0), self.data.duration)
        self.player.setPosition(int(t * 1000))
        self.target_ms, self._pos_wall = int(t * 1000), time.monotonic()
        if play:
            self.player.play()
        self._tick()

    def _line_clicked(self, item: QListWidgetItem):
        row = self.transcript.row(item)
        if self.data and 0 <= row < len(self.data.captions):
            self._seek(self.data.captions[row][0], play=True)

    def _listen_changed(self, _):
        if self.data is None:
            return
        playing = self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState
        self._set_source(int(self._now() * 1000))
        if playing:
            self.player.play()

    def _media_status(self, status):
        if status == QMediaPlayer.MediaStatus.LoadedMedia and self.pending_pos is not None:
            self.player.setPosition(self.pending_pos)
            self.pending_pos = None

    def _subs_toggled(self, on: bool):
        self.view.show_captions = on
        self.view.update()

    def _overlay_toggled(self, on: bool):
        self.view.show_overlay = on
        self._tick()


def quiet_qt_messages(mode, context, message):
    # macOS Qt repeats this harmless warning on every layout change, and the media backend logs every file
    if "propagateSizeHints" in message or (context.category or "").startswith(("qt.multimedia", "qt.qpa.fonts")):
        return
    sys.stderr.write(message + "\n")


def main():
    folders = [Path(a).expanduser() for a in sys.argv[1:]] or default_folders()
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Ctrl+C in the terminal closes the window
    qInstallMessageHandler(quiet_qt_messages)
    app = QApplication(sys.argv[:1])
    app.setApplicationName("Take viewer")
    win = Viewer(folders)
    win.setStyleSheet("QWidget { background: #0d1117; color: #e6edf3; font-size: 14px; }"
                      "QPushButton, QComboBox { background: #21262d; border: 1px solid #30363d;"
                      " border-radius: 6px; padding: 5px 10px; }"
                      "QPushButton:disabled { color: #484f58; }"
                      "QListWidget { background: #161b22; border: 1px solid #30363d; border-radius: 6px; }"
                      "QListWidget::item { padding: 6px; border-bottom: 1px solid #21262d; }"
                      "QListWidget::item:selected { background: #1f6feb; color: white; }"
                      "QStatusBar { color: #8b949e; font-size: 12px; }")
    screen = app.primaryScreen().availableGeometry()
    win.resize(int(screen.width() * 0.95), int(screen.height() * 0.92))
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
