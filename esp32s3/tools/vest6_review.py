#!/usr/bin/env python3
"""Review 6-mic vest recordings (cam_mic_vest6) offline, and hear and see how well the noise is cancelled.

    python3 tools/vest6_review.py                  takes in mic_takes/vest6/, takes/, mic_takes/vest6_*.wav
                                                   and any card that's plugged in
    python3 tools/vest6_review.py path/to/folder   other folders: a card, copied take folders, recordings

Pick a take on the left (unprocessed ones can be processed from here: tools/vest6_process.py). The camera
shows where the talker and other sounds were, with subtitles. On the right, the noise report: every
version of the take at the same voice loudness, how much background each leaves, and how much better than
the raw mic it is; click one to listen to it. Under the picture, the spectrogram of the raw mic against what
you're listening to, every mic channel, and the whole take (click anywhere to jump). Auto volume raises what
you hear until the voice is comfortable (Boost adds or takes away on top), and Remove taps turns anything
sudden and louder than the voice (taps, bumps, bangs) down to the voice's level; the files aren't changed.

Highlights: Shift+drag on the timeline or the channels (or [ and ] while playing) marks a stretch; Export
saves each one's audio (what you're listening to, at the volume you hear) and exactly what was said in it.
Speech-to-text: Whisper small by default (about 1 GB while it runs); pick a bigger model on a faster
computer. Transcribe again redoes only the transcript. Processing runs at low priority so the computer stays
usable. Remove echo (on by default) takes the room's echo out of the six mics first (WPE) and adds two
versions, "Raw mic, echo removed" and "Voice focus, echo removed"; the transcript is then made from the latter.

Keys: space = play / pause, left / right = 2 s back / forward, A = raw mic <-> voice focus,
1-9 = listen to version 1-9 (in the report's order), S = subtitles, D = direction overlay,
[ / ] = start / end a highlight at the playhead.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(os.path.abspath(__file__)).parent
ENV_PY = Path.home() / ".mic-caption-env" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")

try:
    import faster_whisper  # noqa: F401  (needed by vest6_process)
    import numpy  # noqa: F401
    from PySide6 import QtMultimedia  # noqa: F401
except ImportError:
    if ENV_PY.exists() and Path(sys.prefix).resolve() != ENV_PY.parent.parent.resolve():
        os.execv(str(ENV_PY), [str(ENV_PY), os.path.abspath(__file__), *sys.argv[1:]])
    sys.exit("missing packages: run mic_caption_view.py once to set up ~/.mic-caption-env")

import bisect  # noqa: E402
import csv  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import signal  # noqa: E402
import struct  # noqa: E402
import subprocess  # noqa: E402
import tempfile  # noqa: E402
import time  # noqa: E402
import wave  # noqa: E402
from dataclasses import dataclass  # noqa: E402

import numpy as np  # noqa: E402
from PySide6.QtCore import QLineF, QPointF, QProcess, QRectF, QSettings, Qt, QTimer, QUrl, Signal, qInstallMessageHandler  # noqa: E402
from PySide6.QtGui import QColor, QDesktopServices, QFont, QImage, QKeySequence, QPainter, QPen, QShortcut, QTransform  # noqa: E402
from PySide6.QtMultimedia import QAudioOutput, QMediaPlayer  # noqa: E402
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFileDialog,  # noqa: E402
                               QFormLayout, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QMainWindow,
                               QPushButton, QSlider, QSplitter, QVBoxLayout, QWidget)

sys.path.insert(0, str(HERE))
import vest6_process as vp  # noqa: E402
import sd_import as sd  # noqa: E402

FS = vp.FS
ROOT = HERE.parent
PROCESS = HERE / "vest6_process.py"
BG, PANEL, EDGE, INK, DIM = QColor("#0d1117"), QColor("#161b22"), QColor("#30363d"), QColor("#e6edf3"), QColor("#8b949e")
GLOW, LOUD, TALKER, GOOD = QColor("#ffa657"), QColor("#ff7b72"), QColor("#58a6ff"), QColor("#3fb950")
CH_COLORS = [QColor(c) for c in ("#58a6ff", "#f0883e", "#3fb950", "#d2a8ff", "#ff7b72", "#e3b341")]
MUTED = "#8b949e"
WINDOW_S = 8.0  # seconds shown in the spectrogram and channel views
HOLD_S = 2.5
ENV_BLOCK = 80


def font(px: int, bold: bool = False, mono: bool = False) -> QFont:
    f = QFont()
    f.setFamilies(["Menlo", "Courier"] if mono else ["Helvetica", "Arial"])
    f.setPixelSize(px)
    f.setBold(bold)
    return f


def alpha(c: QColor, a: float) -> QColor:
    q = QColor(c)
    q.setAlphaF(max(0.0, min(1.0, a)))
    return q


def panel(p: QPainter, r: QRectF, title: str = ""):
    p.setPen(QPen(EDGE, 1))
    p.setBrush(PANEL)
    p.drawRoundedRect(r, 8, 8)
    if title:
        p.setPen(DIM)
        p.setFont(font(12, True))
        p.drawText(QRectF(r.x() + 10, r.y() + 4, r.width() - 20, 18), Qt.AlignmentFlag.AlignLeft, title)


HIGHLIGHT = QColor("#e3b341")


def draw_highlights(p: QPainter, spans: list, drag, xs, lo: float, hi: float, top: float, h: float):
    """The highlighted stretches (numbered) and the one being dragged out, where they fall between lo and hi."""
    items = [(s, e, str(i + 1)) for i, (s, e) in enumerate(spans)] + ([(*sorted(drag), "")] if drag else [])
    for s, e, label in items:
        if e < lo or s > hi:
            continue
        x0, x1 = xs(max(s, lo)), xs(min(e, hi))
        p.fillRect(QRectF(x0, top, max(2.0, x1 - x0), h), alpha(HIGHLIGHT, 0.18))
        p.setPen(QPen(alpha(HIGHLIGHT, 0.9), 1))
        p.drawLine(QPointF(x0, top), QPointF(x0, top + h))
        p.drawLine(QPointF(x1, top), QPointF(x1, top + h))
        if label:
            p.setPen(HIGHLIGHT)
            p.setFont(font(10, True))
            p.drawText(QRectF(x0 + 3, top + 1, 30, 14), Qt.AlignmentFlag.AlignLeft, label)


def side(az) -> str:
    if az is None:
        return ""
    return "straight ahead" if abs(az) < 3 else f"{abs(az):.0f}° {'left' if az > 0 else 'right'}"


# ------------------------------------------------------------------ finding takes

@dataclass
class Take:
    name: str
    raw: Path | None = None   # card take folder or 6-channel .wav
    out: Path | None = None   # after vest6_process.py
    duration: float = 0.0

    @property
    def processed(self) -> bool:
        return self.out is not None


def is_processed(d: Path) -> bool:
    return (d / "noise_report.json").exists() and (d / "voice_focus.wav").exists()


def default_folders() -> list[Path]:
    return [vp.OUT_ROOT, ROOT / "takes", ROOT / "mic_takes", *sorted(Path("/Volumes").glob("*/takes"))]


def wav_info(p: Path) -> tuple[int, float]:
    try:
        with wave.open(str(p), "rb") as w:
            return w.getnchannels(), w.getnframes() / max(1, w.getframerate())
    except Exception:  # noqa: BLE001
        return 0, 0.0


def scan(folders: list[Path]) -> list[Take]:
    takes: dict[str, Take] = {}
    for f in folders:
        if not f.exists():
            continue
        cands = [f] if (f.suffix.lower() == ".wav" or (f / "info.txt").exists() or is_processed(f)) else []
        if not cands and f.is_dir():
            cands = sorted(d for d in f.iterdir() if d.is_dir() or d.suffix.lower() == ".wav")
            if (f / "takes").is_dir():
                cands += sorted((f / "takes").iterdir())
        for d in cands:
            if d.is_dir() and is_processed(d):
                t = takes.setdefault(d.name, Take(d.name))
                t.out = t.out or d
            elif d.is_dir() and (d / "info.txt").exists() and sd.read_info(d).get("mics") == "6":
                t = takes.setdefault(d.name, Take(d.name))
                t.raw = t.raw or d
                t.duration = t.duration or float(sd.read_info(d).get("duration_s") or 0)
            elif d.suffix.lower() == ".wav" and d.name.startswith("vest6_"):
                nch, dur = wav_info(d)
                if nch >= 6:
                    t = takes.setdefault(d.stem, Take(d.stem))
                    t.raw, t.duration = t.raw or d, t.duration or dur
    for t in takes.values():
        if t.out is None and is_processed(vp.OUT_ROOT / t.name):
            t.out = vp.OUT_ROOT / t.name
        if t.out and not t.duration:
            t.duration = float(json.loads((t.out / "noise_report.json").read_text()).get("duration_s", 0))
    return sorted(takes.values(), key=lambda t: t.name, reverse=True)


# ------------------------------------------------------------------ one processed take

def load_wav(path: Path) -> np.ndarray:
    """(frames, channels) int16, memory-mapped."""
    with open(path, "rb") as f:
        head = f.read(65536)
    i = head.find(b"data")
    (nch,) = struct.unpack_from("<H", head, 22)
    n = (path.stat().st_size - i - 8) // (2 * nch)
    return np.memmap(path, "<i2", "r", offset=i + 8, shape=(n, nch))


def write_stereo(path: Path, x: np.ndarray):
    """The mono listening signal in both channels, at full level: some players play a mono file quieter
    than the review app does."""
    sd.write_wav(path, np.repeat(np.asarray(x)[:, None], 2, axis=1))


def envelope(x: np.ndarray) -> np.ndarray:
    """(blocks, ch, 2) min/max per 5 ms."""
    nb = len(x) // ENV_BLOCK
    out = np.empty((nb, x.shape[1], 2), np.int16)
    for b in range(0, nb, 20000):
        e = min(nb, b + 20000)
        blk = np.asarray(x[b * ENV_BLOCK:e * ENV_BLOCK]).reshape(e - b, ENV_BLOCK, x.shape[1])
        out[b:e, :, 0], out[b:e, :, 1] = blk.min(axis=1), blk.max(axis=1)
    return out


def model_downloaded(model: str) -> bool:
    """Is this Whisper model already in the Hugging Face cache (so choosing it needs no download)?"""
    hub = Path(os.environ.get("HF_HUB_CACHE") or Path(os.environ.get("HF_HOME", Path.home() / ".cache" / "huggingface")) / "hub")
    return hub.is_dir() and any(p.name.endswith(f"faster-whisper-{model}") for p in hub.iterdir())


LOUD_VOICE_DB = -20.0  # Auto volume puts the voice's louder moments (90th percentile of 50 ms levels) here


def listen_level(x: np.ndarray, speech: list[tuple[float, float]], auto: bool, boost_db: float,
                 taps: bool) -> tuple[np.ndarray, float, int]:
    """What you hear: x raised so the voice is easy to hear, with anything sudden and louder than the voice
    (taps, bangs, claps) cut back down to the voice's own peaks. Returns the int16-scale signal, the gain in
    dB and how many 1 ms blocks were turned down."""
    x = np.asarray(x, np.float64)
    x = x - x.mean()
    blk, nb = 16, len(x) // 16
    if nb < 10:
        return x, 0.0, 0
    pk = np.abs(x[:nb * blk]).reshape(nb, blk).max(axis=1)
    nf = len(x) // 800
    lv = 10 * np.log10(np.mean(x[:nf * 800].reshape(nf, 800) ** 2, axis=1) + 1e-9) - 20 * np.log10(32768)
    in_speech = np.zeros(nb, bool)
    for s, e in speech:
        in_speech[int(s * 1000):int(e * 1000)] = True
    in_speech_f = in_speech[: nf * 50].reshape(nf, 50).any(axis=1) if nf else np.zeros(0, bool)
    if in_speech_f.sum() < 20:  # no speech found: the take's louder half stands in for it
        in_speech_f = lv > np.median(lv)
        in_speech = np.repeat(in_speech_f, 50)[:nb]
        in_speech = np.pad(in_speech, (0, nb - len(in_speech)))
    gain_db = (LOUD_VOICE_DB - float(np.percentile(lv[in_speech_f], 90)) if auto else 0.0) + boost_db
    g = 10 ** (gain_db / 20)
    pk = np.maximum(pk, 1e-9)
    need = np.minimum(1.0, 32767 * 10 ** (-1 / 20) / g / pk)  # nothing past -1 dBFS after the gain
    if taps:
        # past the voice's own peaks, a sound goes down twice as far as it's over: a tap 15 dB over ends
        # 15 dB under, a loud word 1 dB over ends 1 dB under
        voice_peak = float(np.percentile(pk[in_speech], 99)) * 10 ** (3 / 20)
        need = np.minimum(need, np.maximum(10 ** (-40 / 20), np.minimum(1.0, voice_peak / pk) ** 2))
    for k in (1, 2, 3):  # 3 ms look-ahead: the gain is already down when a tap's first edge arrives
        need[:-k] = np.minimum(need[:-k], need[k:])
    gain = np.ones(nb)
    rel = math.exp(-1 / 60)  # comes back up over ~60 ms
    cut = np.flatnonzero(need < 1)
    i, cur = 0, 1.0
    while i < nb:
        if cur > 0.9999:
            j = bisect.bisect_left(cut, i)
            if j == len(cut):
                break
            i, cur = int(cut[j]), 1.0
        cur = min(need[i], 1 - (1 - cur) * rel)
        gain[i] = cur
        i += 1
    t = np.arange(nb) * blk + blk / 2
    per_sample = np.interp(np.arange(len(x)), t, gain)
    return np.clip(x * g * per_sample, -32767, 32767), gain_db, int((gain < 0.9).sum())


class TakeData:
    def __init__(self, take: Take):
        out = take.out
        self.take = take
        self.rep = json.loads((out / "noise_report.json").read_text())
        self.raw = load_wav(out / "raw_6mic.wav")
        self.duration = len(self.raw) / FS
        self.versions = [(r["file"], r["name"]) for r in self.rep["rows"]]
        self.audio = {f: load_wav(out / f) for f, _ in self.versions}
        self.env_raw = envelope(self.raw)
        self._env, self._spec, self._chan_wavs = {}, {}, {}
        self.peak = max(50.0, float(np.percentile(np.abs(self.env_raw), 99.9)))
        self.vpeak = max(50.0, max(float(np.percentile(np.abs(envelope(a)), 99.9)) for a in self.audio.values()))
        fs = self.rep["frame_s"]
        self.speech = [(a * fs, b * fs) for a, b in self.rep["speech_frames"]]
        self.background = [(a * fs, b * fs) for a, b in self.rep["background_frames"]]
        self.aim = [(s * fs, e * fs, az, el, sig) for s, e, az, el, sig in self.rep.get("aim", [])]
        self.captions, self.words, self.model = [], [], None
        if (out / "captions.json").exists():
            cj = json.loads((out / "captions.json").read_text())
            self.model = cj.get("model")
            for c in cj["captions"]:
                self.captions.append(tuple(c[:5]))
                self.words.append([tuple(w) for w in c[5]] if len(c) > 5 else [])
        hl = out / "highlights.json"
        self.highlights = [tuple(h) for h in json.loads(hl.read_text())] if hl.exists() else []
        self.dets = []
        if (out / "directions.csv").exists():
            with open(out / "directions.csv") as f:
                for row in csv.DictReader(f):
                    el = row["elevation_deg"]
                    self.dets.append((float(row["time_s"]), float(row["azimuth_deg"]), float(el) if el else None,
                                      float(row["uncertainty_deg"]), float(row["loudness_db"])))
        self.det_times = [d[0] for d in self.dets]
        self.frames = sd.read_frames(out / "camera.bin")
        self.frame_times = [s / FS for s, _ in self.frames]
        self._img = (None, None)

    def final_index(self) -> int:
        """The version the transcript was made from (voice focus, or voice focus with the echo removed)."""
        names = [f for f, _ in self.versions]
        want = self.rep.get("transcript_from", "voice_focus.wav")
        return names.index(want) if want in names else names.index("voice_focus.wav")

    def save_highlights(self):
        self.highlights.sort()
        (self.take.out / "highlights.json").write_text(json.dumps(self.highlights))

    def said_between(self, s: float, e: float) -> list[tuple]:
        """(start, end, text, az, el) said between s and e: cut to the words inside when the transcript has word
        times (newer takes), else every line that overlaps."""
        out = []
        for (cs, ce, text, az, el), words in zip(self.captions, self.words):
            if ce <= s or cs >= e:
                continue
            if words:
                inside = [w for w in words if s <= (w[0] + w[1]) / 2 <= e]
                if inside:
                    out.append((inside[0][0], inside[-1][1], " ".join(w[2] for w in inside), az, el))
            else:
                out.append((cs, ce, text, az, el))
        return out

    def env(self, fname: str) -> np.ndarray:
        if fname not in self._env:
            self._env[fname] = envelope(self.audio[fname])
        return self._env[fname]

    def spectrogram(self, fname: str) -> QImage:
        """The whole take, 129 rows up to 8 kHz, one column per 16 ms."""
        if fname not in self._spec:
            y = np.asarray(self.audio[fname][:, 0], np.float64)
            cols = max(1, int(self.duration / 0.016))
            if "lim" not in self._spec:
                ref = vp.spectrogram(np.asarray(self.audio["raw_mic.wav"][:, 0], np.float64), cols)
                hi = float(np.percentile(ref, 99.5))
                self._spec["lim"] = (hi - 70, hi)
            S = vp.spectrogram(y, cols)
            rgb = np.ascontiguousarray(vp.colour(S[::-1], *self._spec["lim"]))
            img = QImage(rgb.data, cols, S.shape[0], cols * 3, QImage.Format.Format_RGB888).copy()
            self._spec[fname] = img
        return self._spec[fname]

    def listen_wav(self, fname: str | None, ch: int | None, auto: bool, boost_db: float, taps: bool):
        """A version (fname) or one raw mic (ch) as you'll hear it (listen_level), as a file the player can
        open. Returns (path, gain dB, 1 ms blocks turned down)."""
        key = (fname, ch, auto, round(boost_db), taps)
        if key not in self._chan_wavs:
            src = self.take.out / (fname or "raw_6mic.wav")
            tag = f"{fname[:-4] if fname else f'ch{ch}'}_{'a' if auto else 'm'}{round(boost_db):+d}{'_t' if taps else ''}"
            p = Path(tempfile.gettempdir()) / f"vest6_{self.take.name}_{tag}.wav"
            x = self.audio[fname][:, 0] if fname else self.raw[:, ch]
            y, gain_db, cut = listen_level(x, self.speech, auto, boost_db, taps)
            if not p.exists() or p.stat().st_mtime < src.stat().st_mtime:
                sd.write_wav(p, y)
            self._chan_wavs[key] = (p, gain_db, cut)
        return self._chan_wavs[key]

    def picture(self, t: float, rotate: int, mirror: bool) -> QImage | None:
        if not self.frames:
            return None
        i = max(0, bisect.bisect_right(self.frame_times, t) - 1)
        if self._img[0] != (i, rotate, mirror):
            img = QImage.fromData(self.frames[i][1], "JPG")
            if rotate or mirror:
                tr = QTransform()
                tr.rotate(rotate)
                if mirror:
                    tr.scale(-1, 1)
                img = img.transformed(tr, Qt.TransformationMode.SmoothTransformation)
            self._img = ((i, rotate, mirror), img)
        return self._img[1]

    def aim_at(self, t: float):
        for s, e, az, el, sig in self.aim:
            if s - 0.2 <= t <= e + 0.4:
                return az, el, sig
        return None


class Calib:
    """The 360 viewer's camera settings (Set centre / Right edge / Rotate there)."""

    def __init__(self):
        s = QSettings("CapstoneMicArray", "mic_camera_view360")
        truth = lambda v: v in (True, "true", "1", 1)  # noqa: E731
        self.hfov = float(s.value("hfov", 60.0))
        self.offset = float(s.value("offset", 0.0))
        self.tilt = float(s.value("tilt", 0.0))
        self.swap = truth(s.value("swap", "false"))
        self.rotate = int(s.value("rotate", 0))
        self.mirror = truth(s.value("mirror", "false"))

    def rel(self, az: float) -> float:
        return (-az if self.swap else az) - self.offset


# ------------------------------------------------------------------ views

class CameraView(QWidget):
    def __init__(self, calib: Calib):
        super().__init__()
        self.calib, self.data, self.t = calib, None, 0.0
        self.show_subs = self.show_overlay = True
        self.setMinimumSize(420, 300)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), BG)
        r = QRectF(self.rect()).adjusted(4, 4, -4, -4)
        panel(p, r, "Camera + where the sound came from")
        d, c = self.data, self.calib
        if d is None:
            p.setPen(DIM)
            p.setFont(font(18))
            p.drawText(r, Qt.AlignmentFlag.AlignCenter, "pick a processed take")
            return
        area = r.adjusted(10, 26, -10, -10)
        img = d.picture(self.t, c.rotate, c.mirror)
        aspect = img.width() / img.height() if img is not None else 4 / 3
        w = min(area.width(), area.height() * aspect)
        pic = QRectF(area.center().x() - w / 2, area.center().y() - w / aspect / 2, w, w / aspect)
        if img is not None:
            p.drawImage(pic, img)
        else:
            p.setPen(QPen(EDGE, 1.5, Qt.PenStyle.DashLine))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(pic)
            p.setPen(DIM)
            p.setFont(font(15))
            p.drawText(pic, Qt.AlignmentFlag.AlignCenter, "no camera in this take\n(directions still show)")
        p.save()
        p.setClipRect(pic)
        f = pic.width() / 2 / math.tan(math.radians(c.hfov / 2))
        x_of = lambda rel: pic.center().x() - f * math.tan(math.radians(max(-89.0, min(89.0, rel))))  # noqa: E731
        y_of = lambda el: pic.center().y() - f * math.tan(math.radians(max(-80.0, min(80.0, el - c.tilt))))  # noqa: E731
        t = self.t
        if self.show_overlay:
            lo, hi = bisect.bisect_left(d.det_times, t - HOLD_S), bisect.bisect_right(d.det_times, t)
            recent = d.dets[lo:hi]
            for det in recent:
                td, az, el, sig, strength = det
                fade = 1 - (t - td) / HOLD_S
                rel = c.rel(az)
                color = LOUD if strength > 30 else GLOW
                if abs(rel) > c.hfov / 2:
                    continue
                x = x_of(rel)
                p.setPen(QPen(alpha(QColor("#fff3d6"), fade), 2.5 if det is recent[-1] else 1.2))
                p.drawLine(QPointF(x, pic.top()), QPointF(x, pic.bottom()))
                if el is not None:
                    p.setPen(QPen(alpha(color, fade), 3))
                    p.setBrush(Qt.BrushStyle.NoBrush)
                    p.drawEllipse(QPointF(x, y_of(el)), 12, 12)
            a = d.aim_at(t)
            if a:
                az, el, sig = a
                rel = c.rel(az)
                if abs(rel) <= c.hfov / 2:
                    x, y = x_of(rel), y_of(el)
                    hw = max(16.0, abs(x_of(rel + sig) - x_of(rel - sig)) / 2)
                    p.setPen(Qt.PenStyle.NoPen)
                    p.setBrush(alpha(TALKER, 0.18))
                    p.drawEllipse(QPointF(x, y), hw, hw)
                    p.setPen(QPen(TALKER, 3))
                    p.setBrush(Qt.BrushStyle.NoBrush)
                    p.drawEllipse(QPointF(x, y), 18, 18)
                    self.tag(p, pic, x + 24, y - 34, f"talker · {side(rel)}, {el:+.0f}° up · ±{sig:.0f}°", TALKER)
                else:
                    left = rel > 0
                    self.tag(p, pic, pic.left() + 8 if left else pic.right() - 8, pic.center().y(),
                             f"{'◀ ' if left else ''}talker {side(rel)}{' ▶' if not left else ''}", TALKER, right=not left)
        if self.show_subs:
            lines = [txt for s, e, txt, *_ in d.captions if s <= t <= e + 2.0][-2:]
            if lines:
                p.setFont(font(max(15, int(pic.width() / 32)), True))
                fm = p.fontMetrics()
                width = pic.width() * 0.92
                hs = [fm.boundingRect(QRectF(0, 0, width, 1000).toRect(), Qt.TextFlag.TextWordWrap, s).height() for s in lines]
                total = sum(hs) + 8 * len(lines) + 8
                y = pic.bottom() - 14 - total
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QColor(0, 0, 0, 175))
                p.drawRoundedRect(QRectF(pic.center().x() - width / 2 - 8, y, width + 16, total), 8, 8)
                y += 8
                p.setPen(INK)
                for s, h in zip(lines, hs):
                    p.drawText(QRectF(pic.center().x() - width / 2, y, width, h),
                               Qt.TextFlag.TextWordWrap | Qt.AlignmentFlag.AlignHCenter, s)
                    y += h + 8
        p.restore()

    def tag(self, p, pic, x, y, text, color, right=False):
        p.setFont(font(13, True))
        w = p.fontMetrics().horizontalAdvance(text) + 18
        x = x - w if right else x
        x = min(max(x, pic.left() + 4), pic.right() - w - 4)
        y = min(max(y, pic.top() + 4), pic.bottom() - 30)
        p.setPen(QPen(color, 2))
        p.setBrush(QColor(13, 17, 23, 215))
        p.drawRoundedRect(QRectF(x, y, w, 26), 7, 7)
        p.setPen(INK)
        p.drawText(QRectF(x, y, w, 26), Qt.AlignmentFlag.AlignCenter, text)


class NoisePanel(QWidget):
    """The noise report, as a table to click: what you hear is highlighted."""
    pick = Signal(int)

    def __init__(self):
        super().__init__()
        self.data, self.listening = None, 0
        self.setMinimumSize(380, 300)
        self.rows_y: list[tuple[float, float]] = []

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), BG)
        r = QRectF(self.rect()).adjusted(4, 4, -4, -4)
        panel(p, r, "Noise cancellation · same voice loudness, background left over")
        d = self.data
        self.rows_y = []
        if d is None:
            return
        rep = d.rep
        x0, y = r.x() + 12, r.y() + 30
        p.setFont(font(11))
        p.setPen(DIM)
        p.drawText(QRectF(x0, y, r.width() - 24, 16), Qt.AlignmentFlag.AlignLeft,
                   f"speech {rep['speech_s']:.1f} s · background measured over {rep['background_s']:.1f} s of quiet · "
                   f"voice band 300-3400 Hz")
        y += 22
        cols = [("version", 0.0), ("background", 0.34), ("voice/noise", 0.48), ("echo (SRMR)", 0.62), ("vs raw mic", 0.76)]
        for name, fx in cols:
            p.drawText(QRectF(x0 + fx * (r.width() - 24), y, 120, 16), Qt.AlignmentFlag.AlignLeft, name)
        y += 20
        rows = rep["rows"]
        best = max(1.0, max(row["better_than_raw_db"] for row in rows))
        rh = max(30.0, min(44.0, (r.bottom() - y - 120) / len(rows)))
        for i, row in enumerate(rows):
            box = QRectF(r.x() + 6, y, r.width() - 12, rh - 4)
            self.rows_y.append((y, y + rh))
            if i == self.listening:
                p.setPen(QPen(TALKER, 2))
                p.setBrush(alpha(TALKER, 0.16))
                p.drawRoundedRect(box, 6, 6)
            gain = row["better_than_raw_db"]
            bw = (r.width() - 24) * 0.22
            bx = x0 + 0.76 * (r.width() - 24)
            if i:
                frac = max(0.0, min(1.0, gain / best))
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(alpha(GOOD if gain >= 3 else (GLOW if gain >= 0 else LOUD), 0.55))
                p.drawRoundedRect(QRectF(bx, y + 6, max(2.0, bw * frac), rh - 16), 3, 3)
            name_w = 0.33 * (r.width() - 24)
            p.setPen(INK)
            p.setFont(font(13, i == self.listening))
            p.drawText(QRectF(x0, y, name_w, rh / 2 + 4), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignBottom,
                       p.fontMetrics().elidedText(f"{i + 1}  {row['name']}", Qt.TextElideMode.ElideRight, int(name_w)))
            p.setPen(DIM)
            p.setFont(font(10))
            p.drawText(QRectF(x0 + 16, y + rh / 2 + 2, name_w - 16, 14), Qt.AlignmentFlag.AlignLeft,
                       p.fontMetrics().elidedText(row["about"], Qt.TextElideMode.ElideRight, int(name_w - 16)))
            p.setFont(font(13, False, True))
            p.setPen(INK)
            mid = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
            p.drawText(QRectF(x0 + 0.34 * (r.width() - 24), y, 100, rh - 4), mid, f"{row['background_dbfs']:.1f} dB")
            p.drawText(QRectF(x0 + 0.48 * (r.width() - 24), y, 100, rh - 4), mid, f"{row['voice_to_noise_db']:.1f} dB")
            p.drawText(QRectF(x0 + 0.62 * (r.width() - 24), y, 100, rh - 4), mid,
                       "—" if row.get("srmr") is None else f"{row['srmr']:.2f}")
            p.drawText(QRectF(bx + 6, y, 100, rh - 4), mid, "—" if not i else f"{gain:+.1f} dB")
            y += rh
        row = rows[self.listening] if self.listening < len(rows) else None
        y += 8
        p.setFont(font(12, True))
        p.setPen(DIM)
        if row and self.listening:
            p.drawText(QRectF(x0, y, r.width() - 24, 16), Qt.AlignmentFlag.AlignLeft, f"{row['name']} vs raw mic, by band:")
            y += 20
            p.setFont(font(12, False, True))
            parts = [f"{b} {'' if v is None else f'{v:+.1f} dB'}" for b, v in row["better_by_band_db"].items()]
            full = row.get("background_drop_full_band_db")
            parts.append(f"whole range (rumble, hiss) {'' if full is None else f'{full:+.1f} dB'}")
            p.setPen(INK)
            p.drawText(QRectF(x0, y, r.width() - 24, 40), Qt.TextFlag.TextWordWrap, "  ·  ".join(parts))
            y += 42
        notes = []
        if rep["background_s"] < 1.0:
            notes.append("Under 1 s of quiet between the speech: background numbers are rough. Leave pauses next time.")
        if rep["speech_s"] < 1.0:
            notes.append("Hardly any speech found: voice numbers are rough.")
        notes.append("Higher 'vs raw mic' = the voice stands out more. Listen too: a filter can sound watery.")
        p.setFont(font(11))
        p.setPen(DIM)
        p.drawText(QRectF(x0, y, r.width() - 24, r.bottom() - y - 6), Qt.TextFlag.TextWordWrap, "\n".join(notes))

    def mousePressEvent(self, e):
        y = e.position().y()
        for i, (a, b) in enumerate(self.rows_y):
            if a <= y < b:
                self.pick.emit(i)
                return


class WindowView(QWidget):
    """Spectrogram (raw mic over what you hear) and every channel, the WINDOW_S seconds around now."""
    seek = Signal(float)
    highlight = Signal(float, float)

    def __init__(self):
        super().__init__()
        self.data, self.t, self.listen_file, self.listen_ch = None, 0.0, "voice_focus.wav", None
        self.drag = None
        self.setMinimumHeight(330)

    def span(self) -> tuple[float, float]:
        d = self.data
        a = max(0.0, min(self.t - WINDOW_S / 2, (d.duration if d else 0) - WINDOW_S))
        return a, a + WINDOW_S

    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), BG)
        d = self.data
        r = QRectF(self.rect()).adjusted(4, 2, -4, -2)
        panel(p, r, "Spectrogram and channels (8 s around now) · blue band = speech, grey = background measured")
        if d is None:
            return
        a, b = self.span()
        left, right = r.x() + 120, r.right() - 10
        W = right - left
        xs = lambda t: left + (t - a) / (b - a) * W  # noqa: E731
        y = r.y() + 26
        listen_name = next((n for f, n in d.versions if f == self.listen_file), f"ch{self.listen_ch} raw")
        if self.listen_ch is not None:
            listen_name = f"ch{self.listen_ch} raw"
        spec_h = max(50.0, (r.height() - 40) * 0.24)
        lower = self.listen_file if self.listen_file != "raw_mic.wav" or self.listen_ch is not None else "voice_focus.wav"
        lower = "voice_focus.wav" if self.listen_ch is not None else lower
        lower_name = next(n for f, n in d.versions if f == lower)
        for fname, label in (("raw_mic.wav", "Raw mic"), (lower, lower_name)):
            img = d.spectrogram(fname)
            cols = img.width()
            sx0, sx1 = a / d.duration * cols, b / d.duration * cols
            p.drawImage(QRectF(left, y, W, spec_h), img, QRectF(sx0, 0, max(1.0, sx1 - sx0), img.height()))
            p.setPen(INK)
            p.setFont(font(12, True))
            p.drawText(QRectF(r.x() + 8, y, 110, 18), Qt.AlignmentFlag.AlignLeft, label)
            p.setPen(DIM)
            p.setFont(font(10))
            p.drawText(QRectF(r.x() + 8, y + 18, 110, 14), Qt.AlignmentFlag.AlignLeft, "0-8 kHz")
            y += spec_h + 4
        bar_y = y
        for spans, col in ((d.speech, alpha(TALKER, 0.8)), (d.background, alpha(DIM, 0.7))):
            for s, e in spans:
                if e < a or s > b:
                    continue
                p.fillRect(QRectF(xs(max(s, a)), bar_y, max(1.0, xs(min(e, b)) - xs(max(s, a))), 5), col)
        y += 10
        rows = [(f"ch{c} {vp.v360.LAYOUTS['vest6'][1][c][0]}", c) for c in range(6)] + [(listen_name, -1)]
        rh = (r.bottom() - 18 - y) / len(rows)
        e0, e1 = int(a * FS / ENV_BLOCK), int(b * FS / ENV_BLOCK)
        n = max(1, int(W))
        for i, (label, c) in enumerate(rows):
            yc = y + i * rh + rh / 2
            if c >= 0:
                env, scale, color = d.env_raw[e0:e1, c], d.peak, CH_COLORS[c]
                hearing = self.listen_ch == c
            else:
                env, scale, color = d.env(self.listen_file or "raw_mic.wav")[e0:e1, 0], d.vpeak, TALKER
                hearing = self.listen_ch is None
            p.setPen(color if hearing else DIM)
            p.setFont(font(11, hearing))
            p.drawText(QRectF(r.x() + 8, yc - 8, 110, 16), Qt.AlignmentFlag.AlignLeft, label)
            if len(env) < 2:
                continue
            idx = np.minimum(np.linspace(0, len(env), n + 1).astype(int)[:-1], len(env) - 1)
            mins = np.minimum.reduceat(env[:, 0], idx)
            maxs = np.maximum.reduceat(env[:, 1], idx)
            p.setPen(QPen(color, 1))
            k = rh / 2 * 0.9 / scale
            p.drawLines([QLineF(left + j, yc - float(maxs[j]) * k, left + j, yc - float(mins[j]) * k) for j in range(n)])
        draw_highlights(p, d.highlights, self.drag, xs, a, b, r.y() + 24, r.bottom() - r.y() - 40)
        x = xs(self.t)
        p.setPen(QPen(INK, 2))
        p.drawLine(QPointF(x, r.y() + 24), QPointF(x, r.bottom() - 16))
        p.setPen(DIM)
        p.setFont(font(10))
        for s in range(int(math.ceil(a)), int(b) + 1):
            p.drawText(QRectF(xs(s) - 30, r.bottom() - 15, 60, 13), Qt.AlignmentFlag.AlignCenter, sd.clock(s)[:-2])

    def _time(self, e) -> float | None:
        a, b = self.span()
        r = QRectF(self.rect()).adjusted(4, 2, -4, -2)
        left, right = r.x() + 120, r.right() - 10
        x = e.position().x()
        if self.drag is None and not left <= x <= right:
            return None
        return a + max(0.0, min(1.0, (x - left) / (right - left))) * (b - a)

    def mousePressEvent(self, e):
        t = None if self.data is None else self._time(e)
        if t is None:
            return
        if e.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            self.drag = (t, t)
            self.update()
        else:
            self.seek.emit(t)

    def mouseMoveEvent(self, e):
        if self.drag is not None:
            self.drag = (self.drag[0], self._time(e))
            self.update()

    def mouseReleaseEvent(self, e):
        if self.drag is not None:
            s, t = sorted(self.drag)
            self.drag = None
            if t - s >= 0.3:
                self.highlight.emit(s, t)
            self.update()


class Timeline(QWidget):
    """The whole take: talker direction per stretch (bars), other sounds (dots), speech / background, now."""
    seek = Signal(float)
    highlight = Signal(float, float)

    def __init__(self, calib: Calib):
        super().__init__()
        self.calib, self.data, self.t = calib, None, 0.0
        self.drag = None
        self.setFixedHeight(120)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), BG)
        r = QRectF(self.rect()).adjusted(4, 2, -4, -2)
        panel(p, r, "Whole take · where sounds came from (click to jump)")
        d = self.data
        if d is None:
            return
        plot = QRectF(r.x() + 60, r.y() + 22, r.width() - 70, r.height() - 40)
        xs = lambda t: plot.left() + t / max(d.duration, 1e-6) * plot.width()  # noqa: E731
        ys = lambda az: plot.center().y() - max(-90.0, min(90.0, self.calib.rel(az))) / 90 * plot.height() / 2  # noqa: E731
        p.setPen(DIM)
        p.setFont(font(10))
        for az, lbl in ((90, "90° L"), (0, "ahead"), (-90, "90° R")):
            y = plot.center().y() - az / 90 * plot.height() / 2
            p.drawText(QRectF(r.x() + 4, y - 7, 54, 14), Qt.AlignmentFlag.AlignRight, lbl)
            p.setPen(QPen(EDGE, 1, Qt.PenStyle.DotLine))
            p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))
            p.setPen(DIM)
        half = self.calib.hfov / 2 / 90 * plot.height() / 2
        p.fillRect(QRectF(plot.left(), plot.center().y() - half, plot.width(), 2 * half), alpha(INK, 0.05))
        for spans, col in ((d.speech, alpha(TALKER, 0.7)), (d.background, alpha(DIM, 0.6))):
            for s, e in spans:
                p.fillRect(QRectF(xs(s), plot.bottom() + 3, max(1.0, xs(e) - xs(s)), 4), col)
        p.setPen(Qt.PenStyle.NoPen)
        for td, az, el, sig, strength in d.dets:
            p.setBrush(alpha(LOUD if strength > 30 else GLOW, 0.8))
            p.drawEllipse(QPointF(xs(td), ys(az)), 2.5, 2.5)
        for s, e, az, el, sig in d.aim:
            p.setBrush(alpha(TALKER, 0.85))
            p.drawRoundedRect(QRectF(xs(s), ys(az) - 3, max(3.0, xs(e) - xs(s)), 6), 3, 3)
        draw_highlights(p, d.highlights, self.drag, xs, 0.0, d.duration, plot.top(), plot.height() + 8)
        x = xs(self.t)
        p.setPen(QPen(INK, 2))
        p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom() + 8))

    def _time(self, e) -> float:
        r = QRectF(self.rect()).adjusted(4, 2, -4, -2)
        left, w = r.x() + 60, r.width() - 70
        return max(0.0, min(1.0, (e.position().x() - left) / w)) * self.data.duration

    def mousePressEvent(self, e):
        if self.data is None:
            return
        if e.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            self.drag = (self._time(e), self._time(e))
            self.update()
        else:
            self.seek.emit(self._time(e))

    def mouseMoveEvent(self, e):
        if self.data is None or not e.buttons() & Qt.MouseButton.LeftButton:
            return
        if self.drag is not None:
            self.drag = (self.drag[0], self._time(e))
            self.update()
        else:
            self.seek.emit(self._time(e))

    def mouseReleaseEvent(self, e):
        if self.drag is not None:
            s, t = sorted(self.drag)
            self.drag = None
            if t - s >= 0.3:
                self.highlight.emit(s, t)
            self.update()


# ------------------------------------------------------------------ window

class Reviewer(QMainWindow):
    def __init__(self, folders: list[Path]):
        super().__init__()
        self.folders = folders
        self.calib = Calib()
        self.takes: list[Take] = []
        self.current: Take | None = None
        self.data: TakeData | None = None
        self.proc: QProcess | None = None
        self.pending_pos: int | None = None
        self.mark: float | None = None
        self.target_ms: int | None = None
        self._pos_ms, self._pos_wall = 0, time.monotonic()
        self.setWindowTitle("Vest review · 6 mics")
        self.player = QMediaPlayer(self)
        self.audio_out = QAudioOutput(self)
        self.player.setAudioOutput(self.audio_out)

        self.take_list = QListWidget()
        self.take_list.currentRowChanged.connect(self._take_selected)
        self.process_btn = QPushButton("Process")
        self.process_btn.setToolTip("Run vest6_process.py: the versions, the noise report, directions, transcript")
        self.process_btn.clicked.connect(self._process)
        refresh = QPushButton("Refresh")
        refresh.clicked.connect(lambda: self.refresh())
        files = QPushButton("Show files")
        files.clicked.connect(self._show_files)
        self.prefs = QSettings("CapstoneMicArray", "vest6_review")
        self.stt_box = QComboBox()
        self.stt_box.setToolTip("Which speech-to-text model writes the transcript. Bigger is more accurate but "
                                "slower and uses more memory; Whisper small is the default")
        auto_model, why = vp.auto_transcriber()
        self.stt_box.addItem(f"Auto: {auto_model} ({why})", "auto")
        for m, about, size in vp.TRANSCRIBERS:
            self.stt_box.addItem(f"{about} · {'downloaded' if model_downloaded(m) else size + ' download'}", m)
        self.stt_box.setCurrentIndex(max(0, self.stt_box.findData(self.prefs.value("stt", vp.DEFAULT_MODEL))))
        self.stt_box.currentIndexChanged.connect(lambda _: self.prefs.setValue("stt", self.stt_box.currentData()))
        self.stt_box.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.stt_box.setMinimumContentsLength(12)
        self.stt_box.view().setMinimumWidth(460)
        self.transcribe_btn = QPushButton("Transcribe again")
        self.transcribe_btn.setToolTip("Redo only the transcript of this take with the speech-to-text model above")
        self.transcribe_btn.clicked.connect(self._transcribe)
        self.echo_box = QCheckBox("Remove echo")
        self.echo_box.setToolTip("When processing: take the room's echo out of the six mics first (WPE), then voice "
                                 "focus. Adds 'Raw mic, echo removed' and 'Voice focus, echo removed'; the transcript "
                                 "is made from the latter. About 5 s extra per minute of audio")
        self.echo_box.setChecked(self.prefs.value("dereverb", "true") in (True, "true", "1", 1))
        self.echo_box.toggled.connect(lambda on: self.prefs.setValue("dereverb", on))
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(self._heading("Takes"))
        lv.addWidget(self.take_list, 1)
        row = QHBoxLayout()
        for b in (self.process_btn, refresh, files):
            row.addWidget(b)
        lv.addLayout(row)
        lv.addWidget(self.echo_box)
        lv.addWidget(QLabel("Speech-to-text:"))
        lv.addWidget(self.stt_box)
        lv.addWidget(self.transcribe_btn)

        self.camera = CameraView(self.calib)
        self.noise = NoisePanel()
        self.noise.pick.connect(lambda i: self.listen_box.setCurrentIndex(i))
        self.window_view = WindowView()
        self.window_view.seek.connect(self._seek)
        self.window_view.highlight.connect(self._add_highlight)
        self.timeline = Timeline(self.calib)
        self.timeline.seek.connect(self._seek)
        self.timeline.highlight.connect(self._add_highlight)
        self.play_btn = QPushButton("Play")
        self.play_btn.setFixedWidth(80)
        self.play_btn.clicked.connect(self._toggle_play)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.sliderMoved.connect(lambda ms: self._seek(ms / 1000))
        self.time_lbl = QLabel("00:00.0 / 00:00.0")
        self.time_lbl.setStyleSheet(f"color: {MUTED}; font-family: Menlo, monospace;")
        self.time_lbl.setFixedWidth(150)
        self.listen_box = QComboBox()
        self.listen_box.setToolTip("What you hear. A = raw mic <-> voice focus, 1-6 = the versions")
        self.listen_box.currentIndexChanged.connect(self._listen_changed)
        self.subs_box = QCheckBox("Subtitles")
        self.subs_box.setChecked(True)
        self.subs_box.toggled.connect(lambda on: (setattr(self.camera, "show_subs", on), self.camera.update()))
        self.overlay_box = QCheckBox("Directions")
        self.overlay_box.setChecked(True)
        self.overlay_box.toggled.connect(lambda on: (setattr(self.camera, "show_overlay", on), self.camera.update()))
        truth = lambda v: v in (True, "true", "1", 1)  # noqa: E731
        self.auto_box = QCheckBox("Auto volume")
        self.auto_box.setToolTip("Raise what you hear until the voice is at a comfortable level")
        self.auto_box.setChecked(truth(self.prefs.value("auto", "true")))
        self.boost = QSlider(Qt.Orientation.Horizontal)
        self.boost.setRange(-12, 24)
        self.boost.setValue(int(self.prefs.value("boost", 0)))
        self.boost.setFixedWidth(110)
        self.boost.setToolTip("Louder or quieter on top of that (dB)")
        self.boost_lbl = QLabel()
        self.boost_lbl.setFixedWidth(52)
        self.taps_box = QCheckBox("Remove taps")
        self.taps_box.setToolTip("Turn any sudden sound louder than the voice (taps, bumps, bangs, claps) down "
                                 "to the voice's own level")
        self.taps_box.setChecked(truth(self.prefs.value("taps", "true")))
        self.boost.valueChanged.connect(lambda v: self.boost_lbl.setText(f"{v:+d} dB"))
        self.boost.sliderReleased.connect(self._level_changed)
        self.boost.valueChanged.connect(lambda _: None if self.boost.isSliderDown() else self._level_changed())
        self.auto_box.toggled.connect(self._level_changed)
        self.taps_box.toggled.connect(self._level_changed)
        self.boost_lbl.setText(f"{self.boost.value():+d} dB")
        self.save_btn = QPushButton("Save audio…")
        self.save_btn.setToolTip("Save the whole take as a WAV file, exactly as you hear it "
                                 "(this version, with Auto volume, Boost and Remove taps)")
        self.save_btn.clicked.connect(self._save_audio)
        for w in (self.auto_box, self.boost, self.taps_box, self.save_btn):  # keep space and the arrow keys for playback
            w.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        controls = QHBoxLayout()
        controls.addWidget(self.play_btn)
        controls.addWidget(self.slider, 1)
        controls.addWidget(self.time_lbl)
        controls.addWidget(QLabel("Listen:"))
        controls.addWidget(self.listen_box)
        controls.addWidget(self.auto_box)
        controls.addWidget(QLabel("Boost:"))
        controls.addWidget(self.boost)
        controls.addWidget(self.boost_lbl)
        controls.addWidget(self.taps_box)
        controls.addWidget(self.save_btn)
        controls.addWidget(self.subs_box)
        controls.addWidget(self.overlay_box)
        top = QSplitter()
        top.addWidget(self.camera)
        top.addWidget(self.noise)
        top.setStretchFactor(0, 3)
        top.setStretchFactor(1, 2)
        mid = QWidget()
        mv = QVBoxLayout(mid)
        mv.setContentsMargins(0, 0, 0, 0)
        mv.setSpacing(4)
        mv.addWidget(top, 3)
        mv.addWidget(self.window_view, 2)
        mv.addWidget(self.timeline)
        mv.addLayout(controls)

        self.transcript = QListWidget()
        self.transcript.setWordWrap(True)
        self.transcript.itemClicked.connect(self._line_clicked)
        self.transcript_head = self._heading("Transcript")
        self.hl_list = QListWidget()
        self.hl_list.setWordWrap(True)
        self.hl_list.itemClicked.connect(lambda it: self._seek(self.data.highlights[self.hl_list.row(it)][0])
                                         if self.data else None)
        hl_hint = QLabel("Shift+drag on the timeline or the channels to highlight, or press [ and ] while playing.")
        hl_hint.setWordWrap(True)
        hl_hint.setStyleSheet(f"color: {MUTED}; font-size: 12px;")
        export = QPushButton("Export…")
        export.setToolTip("Save each highlight's audio (what you're listening to, at this volume) and its transcript")
        export.clicked.connect(self._export)
        remove = QPushButton("Remove")
        remove.clicked.connect(self._remove_highlight)
        clear = QPushButton("Clear")
        clear.clicked.connect(self._clear_highlights)
        hl_row = QHBoxLayout()
        for b in (export, remove, clear):
            hl_row.addWidget(b)
        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(self.transcript_head)
        rv.addWidget(self.transcript, 3)
        rv.addWidget(self._heading("Highlights"))
        rv.addWidget(hl_hint)
        rv.addWidget(self.hl_list, 1)
        rv.addLayout(hl_row)

        split = QSplitter()
        for w in (left, mid, right):
            split.addWidget(w)
        split.setStretchFactor(1, 1)
        split.setSizes([220, 1200, 260])
        self.setCentralWidget(split)
        self.status = self.statusBar()
        self.player.playbackStateChanged.connect(
            lambda s: self.play_btn.setText("Pause" if s == QMediaPlayer.PlaybackState.PlayingState else "Play"))
        self.player.mediaStatusChanged.connect(self._media_status)
        self.player.errorOccurred.connect(lambda _, msg: self.status.showMessage(f"Can't play: {msg}"))
        keys = [(Qt.Key.Key_Space, self._toggle_play), (Qt.Key.Key_Left, lambda: self._seek(self._now() - 2)),
                (Qt.Key.Key_Right, lambda: self._seek(self._now() + 2)), (Qt.Key.Key_A, self._ab),
                (Qt.Key.Key_S, self.subs_box.toggle), (Qt.Key.Key_D, self.overlay_box.toggle),
                (Qt.Key.Key_BracketLeft, self._mark_start), (Qt.Key.Key_BracketRight, self._mark_end)]
        keys += [(getattr(Qt.Key, f"Key_{i + 1}"), (lambda i=i: self.listen_box.setCurrentIndex(i))) for i in range(9)]
        for key, fn in keys:
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
            item = QListWidgetItem(f"{t.name}   {t.duration:.1f} s" + ("" if t.processed else "\nnot processed yet"))
            if not t.processed:
                item.setForeground(QColor(MUTED))
            self.take_list.addItem(item)
        self.take_list.blockSignals(False)
        if not self.takes:
            self.current = None
            self._load(None)
            self.process_btn.setEnabled(False)
            self.status.showMessage("No 6-mic takes found. Plug in the card, copy takes into takes/, or record with "
                                    "mic_camera_view360.py, then Refresh.")
            return
        names = [t.name for t in self.takes]
        row = names.index(keep) if keep in names else 0
        self.take_list.setCurrentRow(row)
        self._take_selected(row)

    def _take_selected(self, row: int):
        if not 0 <= row < len(self.takes):
            return
        t = self.current = self.takes[row]
        self.process_btn.setEnabled(t.raw is not None and self.proc is None)
        self.transcribe_btn.setEnabled(t.processed and self.proc is None)
        self.process_btn.setText("Process again" if t.processed else "Process")
        if not t.processed:
            self._load(None)
            self.status.showMessage(f"{t.name} isn't processed yet: press Process (makes the cleaned versions, "
                                    "measures the noise, finds directions, transcribes)")
            return
        try:
            self._load(TakeData(t))
            vf = next(r for r in self.data.rep["rows"] if r["file"] == "voice_focus.wav")
            self.status.showMessage(f"{t.name}: {self.data.duration:.1f} s · voice focus: voice stands out "
                                    f"{vf['better_than_raw_db']:+.1f} dB more than on the raw mic · "
                                    f"{len(self.data.captions)} lines of speech"
                                    + (" · echo removed (WPE)" if self.data.rep.get("dereverb") else ""))
        except Exception as e:  # noqa: BLE001
            self._load(None)
            self.status.showMessage(f"Couldn't open {t.name}: {e}")

    def _load(self, data: TakeData | None):
        self.player.stop()
        self.data = data
        for w in (self.camera, self.noise, self.window_view, self.timeline):
            w.data = data
            w.update()
        self.transcript.clear()
        self.listen_box.blockSignals(True)
        self.listen_box.clear()
        self.slider.setRange(0, int(data.duration * 1000) if data else 0)
        if data is None:
            self.listen_box.blockSignals(False)
            self.player.setSource(QUrl())
            self.transcript_head.setText("Transcript")
            self.hl_list.clear()
            return
        self.listen_box.addItems([f"{i + 1}. {n}" for i, (_, n) in enumerate(data.versions)]
                                 + [f"Raw ch{c} only ({vp.v360.LAYOUTS['vest6'][1][c][0]})" for c in range(6)])
        self.listen_box.setCurrentIndex(data.final_index())  # what the transcript was made from
        self.listen_box.blockSignals(False)
        for s, e, text, az, el in data.captions:
            where = side(None if az is None else self.calib.rel(az))
            self.transcript.addItem(QListWidgetItem(f"{sd.clock(s)}   {where}\n{text}" if where else f"{sd.clock(s)}\n{text}"))
        if not data.captions:
            self.transcript.addItem(QListWidgetItem("(no speech transcribed)"))
        self.transcript_head.setText(f"Transcript · {data.model}" if data.model else "Transcript")
        self.mark = None
        self._show_highlights()
        self._set_source(0)

    def _process(self):
        t = self.current
        if t and t.raw and not self.proc:
            echo = ["--dereverb"] if self.echo_box.isChecked() else []
            self._run_tool(t, [str(t.raw), "--out", str(vp.OUT_ROOT), *echo],
                           f"Processing {t.name}{' with echo removal' if echo else ''}…")

    def _transcribe(self):
        t = self.current
        if t and t.processed and not self.proc:
            self._run_tool(t, [str(t.out), "--transcribe-only"], f"Transcribing {t.name} again…")

    def _run_tool(self, t: Take, args: list[str], note: str):
        self.player.stop()
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._process_output)
        self.proc.finished.connect(lambda code, _: self._process_done(t.name, code))
        self.process_btn.setEnabled(False)
        self.transcribe_btn.setEnabled(False)
        self.status.showMessage(note)
        self.proc.start(sys.executable, [str(PROCESS), *args, "--model", self.stt_box.currentData(), "--low-priority"])

    # ---------------------------------------------------------------- highlights

    def _add_highlight(self, s: float, e: float):
        d = self.data
        if d is None:
            return
        d.highlights.append((round(s, 2), round(min(e, d.duration), 2)))
        d.save_highlights()
        self._show_highlights()
        self.status.showMessage(f"Highlighted {sd.clock(s)} – {sd.clock(e)}. Export saves its audio and transcript.")

    def _show_highlights(self):
        d = self.data
        self.hl_list.clear()
        for i, (s, e) in enumerate(d.highlights if d else []):
            said = " ".join(t for _, _, t, _, _ in d.said_between(s, e))
            said = said if len(said) < 90 else said[:87] + "…"
            self.hl_list.addItem(QListWidgetItem(f"{i + 1}.  {sd.clock(s)} – {sd.clock(e)}  ({e - s:.1f} s)\n"
                                                 + (said or "(no speech here)")))
        for w in (self.timeline, self.window_view):
            w.update()

    def _remove_highlight(self):
        row = self.hl_list.currentRow()
        if self.data and 0 <= row < len(self.data.highlights):
            del self.data.highlights[row]
            self.data.save_highlights()
            self._show_highlights()

    def _clear_highlights(self):
        if self.data and self.data.highlights:
            self.data.highlights.clear()
            self.data.save_highlights()
            self._show_highlights()

    def _mark_start(self):
        if self.data:
            self.mark = self._now()
            self.status.showMessage(f"Highlight starts at {sd.clock(self.mark)}: press ] where it should end")

    def _mark_end(self):
        if self.data and self.mark is not None:
            s, e = sorted((self.mark, self._now()))
            self.mark = None
            if e - s >= 0.3:
                self._add_highlight(s, e)

    def _audio_settings(self, title: str, ok: str) -> bool:
        """Ask which version and volume settings to save with, starting from what's playing. Saying yes also sets
        the app to them, so what's saved is what you hear."""
        dlg = QDialog(self)
        dlg.setWindowTitle(title)
        form = QFormLayout(dlg)
        version = QComboBox()
        version.addItems([self.listen_box.itemText(k) for k in range(self.listen_box.count())])
        version.setCurrentIndex(self.listen_box.currentIndex())
        auto = QCheckBox("raise the voice to a comfortable level")
        auto.setChecked(self.auto_box.isChecked())
        boost = QSlider(Qt.Orientation.Horizontal)
        boost.setRange(self.boost.minimum(), self.boost.maximum())
        boost.setValue(self.boost.value())
        boost_lbl = QLabel(f"{boost.value():+d} dB")
        boost_lbl.setFixedWidth(52)
        boost.valueChanged.connect(lambda v: boost_lbl.setText(f"{v:+d} dB"))
        boost_row = QHBoxLayout()
        boost_row.addWidget(boost, 1)
        boost_row.addWidget(boost_lbl)
        taps = QCheckBox("turn sudden taps and bangs down to the voice's level")
        taps.setChecked(self.taps_box.isChecked())
        form.addRow("Version:", version)
        form.addRow("Auto volume:", auto)
        form.addRow("Boost:", boost_row)
        form.addRow("Remove taps:", taps)
        hint = QLabel("Saved audio sounds exactly like the app with these settings. Listen plays them here first.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color: {MUTED}; font-size: 12px;")
        form.addRow(hint)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Cancel)
        buttons.addButton(ok, QDialogButtonBox.ButtonRole.AcceptRole).setDefault(True)
        listen = buttons.addButton("Listen", QDialogButtonBox.ButtonRole.ActionRole)
        form.addRow(buttons)

        def apply():
            ws = (self.listen_box, self.auto_box, self.boost, self.taps_box)
            for w in ws:
                w.blockSignals(True)
            self.listen_box.setCurrentIndex(version.currentIndex())
            self.auto_box.setChecked(auto.isChecked())
            self.boost.setValue(boost.value())
            self.taps_box.setChecked(taps.isChecked())
            for w in ws:
                w.blockSignals(False)
            self.boost_lbl.setText(f"{self.boost.value():+d} dB")
            self._level_changed()

        listen.clicked.connect(lambda: (apply(), self.player.play()))
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return False
        apply()
        return True

    def _listening(self):
        """(name, path, gain dB) of what the app plays now."""
        d = self.data
        i, nv = self.listen_box.currentIndex(), len(d.versions)
        fname, ch = (d.versions[i][0], None) if i < nv else (None, i - nv)
        path, gain_db, _ = d.listen_wav(fname, ch, self.auto_box.isChecked(), self.boost.value(),
                                        self.taps_box.isChecked())
        return (d.versions[i][1] if i < nv else f"raw ch{ch}"), path, gain_db

    def _export(self):
        d = self.data
        if d is None or not d.highlights:
            self.status.showMessage("Nothing to export: highlight a stretch first (Shift+drag, or [ and ]).")
            return
        if not self._audio_settings(f"Export {len(d.highlights)} highlight(s): {d.take.name}", "Export"):
            return
        what, path, gain_db = self._listening()
        audio = load_wav(path)[:, 0]
        folder = d.take.out / "exports" / f"{d.take.name}_{time.strftime('%Y%m%d_%H%M%S')}"
        folder.mkdir(parents=True, exist_ok=True)
        lines = [f"{d.take.name}: {len(d.highlights)} highlight(s)",
                 f"audio: {what}, {gain_db:+.0f} dB" + (", taps turned down" if self.taps_box.isChecked() else ""),
                 f"speech-to-text: Whisper {d.model}" if d.model else "speech-to-text: (unknown model)"]
        for k, (s, e) in enumerate(d.highlights, 1):
            write_stereo(folder / f"clip_{k:02d}.wav", audio[int(s * FS):int(e * FS)])
            said = d.said_between(s, e)
            lines += ["", f"Highlight {k}: {sd.clock(s)} – {sd.clock(e)} ({e - s:.1f} s), clip_{k:02d}.wav"]
            for a, b, text, az, _ in said:
                where = side(None if az is None else self.calib.rel(az))
                lines.append(f"  {sd.clock(a)}  {where + '  ' if where else ''}{text}")
            if not said:
                lines.append("  (no speech transcribed here)")
            with open(folder / f"clip_{k:02d}.srt", "w") as f:
                for n, (a, b, text, *_) in enumerate(said, 1):
                    f.write(f"{n}\n{sd.clock(max(0.0, a - s), True)} --> {sd.clock(min(e, b + 0.3) - s, True)}\n{text}\n\n")
        (folder / "transcript.txt").write_text("\n".join(lines) + "\n")
        self.status.showMessage(f"Exported {len(d.highlights)} highlight(s) to {folder}")
        if sys.platform == "darwin":
            subprocess.run(["open", str(folder)])
        else:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(folder)))

    def _save_audio(self):
        d = self.data
        if d is None:
            self.status.showMessage("Nothing to save: pick a processed take first.")
            return
        if not self._audio_settings(f"Save audio: {d.take.name}", "Save…"):
            return
        what, path, gain_db = self._listening()
        slug = "".join(c if c.isalnum() else "_" for c in what.lower()).strip("_")
        folder = Path(self.prefs.value("save_dir", str(Path.home() / "Desktop")))
        dest, _ = QFileDialog.getSaveFileName(self, "Save audio", str(folder / f"{d.take.name}_{slug}.wav"), "WAV audio (*.wav)")
        if not dest:
            return
        dest = Path(dest).with_suffix(".wav")
        write_stereo(dest, load_wav(path)[:, 0])
        self.prefs.setValue("save_dir", str(dest.parent))
        self.status.showMessage(f"Saved {what} ({gain_db:+.0f} dB) to {dest}")
        if sys.platform == "darwin":
            subprocess.run(["open", "-R", str(dest)])

    def _process_output(self):
        text = bytes(self.proc.readAllStandardOutput()).decode(errors="replace").strip()
        lines = [ln for ln in text.splitlines() if ln.strip() and "qt.qpa" not in ln]
        if lines:
            self.status.showMessage(lines[-1].strip())

    def _process_done(self, name: str, code: int):
        self.proc = None
        self.refresh(select=name)
        if code:
            self.status.showMessage(f"processing {name} failed (exit {code}): run tools/vest6_process.py in a terminal to see why")

    def _show_files(self):
        t = self.current
        path = t and (t.out or t.raw)
        if path:
            if sys.platform == "darwin":
                subprocess.run(["open", "-R", str(path)] if path.is_file() else ["open", str(path)])
            else:
                QDesktopServices.openUrl(QUrl.fromLocalFile(str(path)))

    # ---------------------------------------------------------------- playback

    def _set_source(self, pos_ms: int):
        d = self.data
        i = self.listen_box.currentIndex()
        nv = len(d.versions)
        fname, ch = (d.versions[i][0], None) if i < nv else (None, i - nv)
        self.window_view.listen_file, self.window_view.listen_ch = fname or "raw_mic.wav", ch
        path, gain_db, cut = d.listen_wav(fname, ch, self.auto_box.isChecked(), self.boost.value(),
                                          self.taps_box.isChecked())
        name = d.versions[i][1] if i < nv else f"raw ch{ch}"
        self.status.showMessage(f"Listening to {name}: {gain_db:+.0f} dB louder than recorded"
                                + (f" · {cut / 1000:.1f} s of taps and bangs turned down" if self.taps_box.isChecked() else ""))
        self.noise.listening = i if i < nv else -1
        self.noise.update()
        self.pending_pos = pos_ms or None
        self.target_ms, self._pos_wall = pos_ms, time.monotonic()
        self.player.setSource(QUrl.fromLocalFile(str(path)))

    def _ab(self):
        if self.data is None:
            return
        names = [f for f, _ in self.data.versions]
        raw, focus = names.index("raw_mic.wav"), self.data.final_index()
        self.listen_box.setCurrentIndex(raw if self.listen_box.currentIndex() == focus else focus)

    def _level_changed(self, *_):
        self.prefs.setValue("auto", self.auto_box.isChecked())
        self.prefs.setValue("boost", self.boost.value())
        self.prefs.setValue("taps", self.taps_box.isChecked())
        self._listen_changed(None)

    def _listen_changed(self, _):
        if self.data is None or self.listen_box.currentIndex() < 0:
            return
        playing = self.player.playbackState() == QMediaPlayer.PlaybackState.PlayingState
        self._set_source(int(self._now() * 1000))
        if playing:
            self.player.play()

    def _media_status(self, status):
        if status == QMediaPlayer.MediaStatus.LoadedMedia and self.pending_pos is not None:
            self.player.setPosition(self.pending_pos)
            self.pending_pos = None

    def _now(self) -> float:
        pos = self.player.position()
        now = time.monotonic()
        if self.target_ms is not None:
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
        for w in (self.camera, self.window_view, self.timeline):
            w.t = t
            w.update()
        if not self.slider.isSliderDown():
            self.slider.setValue(int(t * 1000))
        self.time_lbl.setText(f"{sd.clock(t)} / {sd.clock(d.duration)}")
        now = next((i for i, (s, e, *_) in enumerate(d.captions) if s <= t <= e + 0.3), -1)
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


def quiet_qt_messages(mode, context, message):
    if "propagateSizeHints" in message or (context.category or "").startswith(("qt.multimedia", "qt.qpa.fonts")):
        return
    sys.stderr.write(message + "\n")


def main():
    folders = [Path(a).expanduser() for a in sys.argv[1:]] or default_folders()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    qInstallMessageHandler(quiet_qt_messages)
    app = QApplication(sys.argv[:1])
    app.setApplicationName("Vest review")
    win = Reviewer(folders)
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
