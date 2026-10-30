#!/usr/bin/env python3
"""Live captions on the camera view (experimental; built on mic_camera_view.py).

    python3 mic_caption_view.py

Everything mic_camera_view.py shows, plus subtitles of what people say, placed by where
they are: a "speaking" tag on the talker when they're in the picture, and a ◀ / ▶ marker
when they're off screen. Sudden loud sounds get a [BANG] line, like TV captions.

The speech goes through a live version of tools/voice_focus.py first (the four mics aimed
at the talker, steady background cancelled, hiss removed), then into Whisper, which runs
on this computer: no internet needed after the first run, and no audio leaves the laptop.

The first run sets up its own folder (~/.mic-caption-env, separate from the camera view's)
and downloads the speech model (about 150 MB for base.en, 500 MB for small.en).

Options:
    --model NAME      Whisper model: base.en (default, fast) or small.en (more accurate)
    --port PORT       serial port, if auto-detect picks the wrong one
    --demo-wav WAV    no board: play a 4-channel recording (from Record) in a loop
    --demo-image JPG  picture to show in demo mode

Captions trail the speech by about 1-2 s and will sometimes get words wrong.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

# ------------------------------------------------------------------ first-run setup

ENV_DIR = Path.home() / ".mic-caption-env"
PACKAGES = ["numpy", "PySide6", "pyserial", "faster-whisper"]


def _env_python() -> Path:
    return ENV_DIR / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def _have_packages() -> bool:
    try:
        import faster_whisper  # noqa: F401
        import numpy  # noqa: F401
        import PySide6  # noqa: F401
        import serial  # noqa: F401
        return True
    except ImportError:
        return False


def _base_python() -> str:
    # faster-whisper's engine lags behind the newest Python, so prefer 3.12 / 3.11.
    for v in ("3.12", "3.11", "3.13"):
        p = shutil.which(f"python{v}")
        if p:
            return p
    return sys.executable


def _bootstrap():
    if _have_packages():
        return
    if Path(sys.prefix).resolve() == ENV_DIR.resolve():
        sys.exit(f"Setup is incomplete. Delete {ENV_DIR} and run this file again.")
    py, ready = _env_python(), ENV_DIR / ".ready"
    if not (py.exists() and ready.exists()):
        print(f"First run: setting up {ENV_DIR} (one time, needs internet)…", flush=True)
        if not py.exists():
            subprocess.check_call([_base_python(), "-m", "venv", str(ENV_DIR)])
        subprocess.check_call([str(py), "-m", "pip", "install", "--quiet", "--upgrade", "pip"])
        subprocess.check_call([str(py), "-m", "pip", "install", "--quiet", *PACKAGES])
        ready.write_text("ok\n")
        print("Setup done, starting…", flush=True)
    sys.exit(subprocess.call([str(py), os.path.abspath(__file__), *sys.argv[1:]]))


_bootstrap()

# ------------------------------------------------------------------ the app

os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")

import argparse  # noqa: E402
import math  # noqa: E402
import queue  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import wave  # noqa: E402
from collections import deque  # noqa: E402

import numpy as np  # noqa: E402
from PySide6.QtCore import QPointF, QRectF, QSettings, Qt, QTimer  # noqa: E402
from PySide6.QtGui import QColor, QFont, QPainter, QPen  # noqa: E402
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout,  # noqa: E402
                               QLabel, QPushButton, QSlider, QVBoxLayout, QWidget)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mic_camera_view as cv  # noqa: E402

FS, CH, C = cv.FS, cv.CH, cv.C
NFFT, HOP = 512, 128
FRAME_S = HOP / FS
SPEAKER = QColor("#58a6ff")
CAPTION_HOLD_S = 9.0
END_SILENCE = 125  # frames of quiet (1 s) that end an utterance
HALLUCINATIONS = {"thank you.", "thanks for watching!", "thank you for watching.", "you", "bye.", "."}


def angle_of(raw: np.ndarray) -> float | None:
    """Direction of the strongest voice-band sound in a 4-channel chunk, from the wide pair."""
    if len(raw) < 512:
        return None
    seg = raw[-min(len(raw), 4 * FS):]
    n = 1 << int(math.ceil(math.log2(2 * len(seg))))
    A, B = np.fft.rfft(seg[:, 0], n), np.fft.rfft(seg[:, 1], n)
    f = np.fft.rfftfreq(n, 1 / FS)
    cross = B * np.conj(A)
    cross /= np.abs(cross) + 1e-12
    cross[(f < 300) | (f > 3400)] = 0
    m = (int(math.ceil(0.14 / C * FS)) + 2) * 16
    cc = np.fft.irfft(cross, n * 16)
    cc = np.concatenate((cc[-m:], cc[:m + 1]))
    lag = (int(np.argmax(cc)) - m) / 16
    return math.degrees(math.asin(max(-1.0, min(1.0, lag / FS * C / 0.14))))


class StreamingFocus:
    """Live voice focus: MVDR beam toward the talker + Wiener hiss filter, 8 ms frames.

    Learns the background from quiet frames and the talker from speech frames as it goes,
    so it follows a background that changes or moves (a second or two to catch up)."""

    def __init__(self):
        self.win = np.sqrt(np.hanning(NFFT + 1)[:NFFT])
        self.f = np.fft.rfftfreq(NFFT, 1 / FS)
        self.band = (self.f >= 300) & (self.f <= 3400)
        F = len(self.f)
        self.inbuf = np.zeros((0, CH))
        self.ola = np.zeros(NFFT)
        self.Rn = np.tile(np.eye(CH, dtype=complex) * 1e-6, (F, 1, 1))
        self.Rs = self.Rn.copy()
        self.n_noise = self.n_speech = self.frames = 0
        self.w = None
        self.Pn = np.full(F, 1e-3)
        self.g_prev = np.ones(F)
        self.post_prev = np.ones(F)
        self.floor = None
        self.enabled = True
        self.wide_hist: deque = deque(maxlen=40)

    def _update_weights(self):
        F = len(self.f)
        tr = np.real(np.trace(self.Rn, axis1=1, axis2=2))
        Rn = self.Rn + np.eye(CH)[None] * (1e-3 * tr / CH + 1e-12)[:, None, None]
        vals, vecs = np.linalg.eig(np.linalg.solve(Rn, self.Rs))
        v = vecs[np.arange(F), :, np.argmax(vals.real, axis=1)]
        d = (Rn @ v[..., None])[..., 0]
        d = d / np.where(np.abs(d[:, :1]) > 1e-12, d[:, :1], 1)
        Rd = np.linalg.solve(Rn, d[..., None])[..., 0]
        self.w = Rd / (np.sum(d.conj() * Rd, axis=1, keepdims=True) + 1e-12)

    def process(self, x: np.ndarray):
        """x: (n, 4) float. Yields (out HOP samples, raw HOP x 4, speech?, bang?) per frame."""
        self.inbuf = np.concatenate([self.inbuf, x])
        while len(self.inbuf) >= NFFT:
            seg = self.inbuf[:NFFT]
            raw = self.inbuf[NFFT - HOP:NFFT].copy()
            self.inbuf = self.inbuf[HOP:]
            X = np.fft.rfft(seg * self.win[:, None], axis=0)
            P = np.abs(X) ** 2
            e = 10 * math.log10(float(P[self.band].mean()) + 1e-12)
            if self.floor is None:
                self.floor = e
            elif e < self.floor:
                self.floor = 0.9 * self.floor + 0.1 * e
            else:
                self.floor += 0.002
            speech, quiet = e > self.floor + 8, e < self.floor + 3
            ew = 10 * math.log10(float(P.mean()) + 1e-12)
            bang = (len(self.wide_hist) == self.wide_hist.maxlen
                    and ew - float(np.median(list(self.wide_hist)[:-3])) > 22 and ew - self.floor > 25)
            self.wide_hist.append(ew)
            self.frames += 1
            if self.enabled:
                outer = X[:, :, None] * X[:, None, :].conj()
                if quiet:
                    self.Rn = 0.99 * self.Rn + 0.01 * outer
                    self.n_noise += 1
                elif speech:
                    self.Rs = 0.98 * self.Rs + 0.02 * outer
                    self.n_speech += 1
                if self.frames % 32 == 0 and self.n_noise > 100 and self.n_speech > 30:
                    self._update_weights()
                Y = np.sum(X * self.w.conj(), axis=1) if self.w is not None else X.mean(axis=1)
                Y[self.f < 70] = 0
                Py = np.abs(Y) ** 2
                if quiet:
                    self.Pn = 0.98 * self.Pn + 0.02 * Py
                post = Py / (self.Pn + 1e-12)
                prio = 0.98 * self.g_prev ** 2 * self.post_prev + 0.02 * np.maximum(post - 1, 0)
                g = np.maximum(prio / (1 + prio), 0.12)
                self.g_prev, self.post_prev = g, post
                Y = Y * g
            else:
                Y = X.mean(axis=1)
                Y[self.f < 70] = 0
            self.ola += np.fft.irfft(Y, NFFT) * self.win / 2  # sqrt-Hann in and out, 75% overlap
            out = self.ola[:HOP].copy()
            self.ola = np.concatenate([self.ola[HOP:], np.zeros(HOP)])
            yield out, raw, speech, bang


class Captioner(threading.Thread):
    """Cuts the focused audio into utterances and transcribes them with Whisper."""

    def __init__(self, model_name: str):
        super().__init__(daemon=True)
        self.model_name = model_name
        self.q: queue.Queue = queue.Queue()
        self.lock = threading.Lock()
        self.lines: deque = deque(maxlen=200)  # dicts: wall, text, angle, kind ('speech' | 'sound')
        self.partial = None  # (text, angle)
        self.speaking_angle = None
        self.status = "loading speech model…"
        self.focus = StreamingFocus()
        self.stop = False
        self.transcript_file = None

    def feed(self, chunks: list):
        if chunks:
            self.q.put(np.concatenate(chunks).astype(np.float64))

    def _add(self, text: str, angle, kind: str, dur: float = 0.0):
        now = time.time()
        with self.lock:
            last = self.lines[-1] if self.lines else None
            if (kind == "speech" and last and last["kind"] == "speech" and now - last["wall"] < 2.5
                    and len(last["text"]) + len(text) < 160):
                # a short pause mid-sentence: keep it on the same caption line
                last["text"] = f"{last['text']} {text}"
                last["wall"] = now
                if angle is not None and dur > 0.8:
                    last["angle"] = angle
            else:
                self.lines.append({"wall": now, "text": text, "angle": angle if dur > 0.6 or kind == "sound" else
                                   (last["angle"] if last else angle), "kind": kind})
            if self.transcript_file:
                where = "" if angle is None else f" ({angle:+.0f}° mic angle)"
                self.transcript_file.write(f"{time.strftime('%H:%M:%S')}{where} {text}\n")
                self.transcript_file.flush()

    def _transcribe(self, audio: np.ndarray) -> str:
        a = audio.astype(np.float32) / 32768.0
        peak = float(np.max(np.abs(a))) if len(a) else 0.0
        if peak < 1e-5:
            return ""
        a *= 0.5 / peak
        segs, _ = self.model.transcribe(a, language="en", beam_size=1, vad_filter=True,
                                        condition_on_previous_text=False, without_timestamps=True)
        words = [s.text.strip() for s in segs if s.no_speech_prob < 0.6 and s.avg_logprob > -1.0]
        text = " ".join(w for w in words if w)
        return "" if text.lower() in HALLUCINATIONS else text

    def run(self):
        from faster_whisper import WhisperModel
        try:
            self.model = WhisperModel(self.model_name, device="cpu", compute_type="int8", local_files_only=True)
        except Exception:  # noqa: BLE001
            self.status = f"downloading speech model {self.model_name} (one time)…"
            try:
                self.model = WhisperModel(self.model_name, device="cpu", compute_type="int8")
            except Exception as e:  # noqa: BLE001
                self.status = f"speech model failed to load (needs internet the first time): {e}"
                return
        self.status = f"captions on ({self.model_name})"
        while not self.q.empty():
            self.q.get_nowait()
        preroll: deque = deque(maxlen=40)  # 0.32 s before speech starts
        recent: deque = deque(maxlen=12)
        utt, utt_raw = [], []
        in_utt, silence, last_partial, last_bang = False, 0, 0.0, 0.0
        while not self.stop:
            try:
                x = self.q.get(timeout=0.2)
            except queue.Empty:
                continue
            for out, raw, speech, bang in self.focus.process(x):
                now = time.time()
                if bang and now - last_bang > 0.6:
                    last_bang = now
                    around = [r for _, r in list(preroll)[-8:]] + [raw]
                    self._add("[BANG]", angle_of(np.concatenate(around)), "sound")
                recent.append(speech)
                preroll.append((out, raw))
                if not in_utt:
                    if sum(recent) >= 6:
                        in_utt, silence, last_partial = True, 0, now
                        utt = [o for o, _ in preroll]
                        utt_raw = [r for _, r in preroll]
                    continue
                utt.append(out)
                utt_raw.append(raw)
                silence = 0 if speech else silence + 1
                dur = len(utt) * FRAME_S
                if silence >= END_SILENCE or dur > 12:
                    audio, rawa = np.concatenate(utt), np.concatenate(utt_raw)
                    in_utt = silence < END_SILENCE
                    utt, utt_raw = [], []
                    text = self._transcribe(audio)
                    ang = angle_of(rawa)
                    with self.lock:
                        self.partial = None
                        self.speaking_angle = ang if in_utt else None
                    if text:
                        self._add(text, ang, "speech", dur)
                elif now - last_partial > 1.0 and dur > 0.8 and self.q.qsize() < 3:
                    last_partial = now
                    ang = angle_of(np.concatenate(utt_raw))
                    text = self._transcribe(np.concatenate(utt))
                    with self.lock:
                        self.speaking_angle = ang
                        self.partial = (text, ang) if text else self.partial


class CaptionView(cv.View):
    def __init__(self, model, reader, captioner: Captioner):
        super().__init__(model, reader)
        self.cap = captioner
        self.show_captions = True

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), cv.BG)
        w, h = self.width(), self.height()
        gap = 14
        left_w = w * 0.68
        cam_r = QRectF(gap, gap, left_w - 1.5 * gap, h - 2 * gap)
        x1 = left_w + 0.5 * gap
        cw = w - x1 - gap
        gauge_r = QRectF(x1, gap, cw, h * 0.32)
        tr_r = QRectF(x1, gauge_r.bottom() + gap, cw, h * 0.30)
        lvl_r = QRectF(x1, tr_r.bottom() + gap, cw, h * 0.13)
        info_r = QRectF(x1, lvl_r.bottom() + gap, cw, h - lvl_r.bottom() - 2 * gap)
        self.draw_camera(p, cam_r)
        self.draw_gauge(p, gauge_r)
        self.draw_transcript(p, tr_r)
        self.draw_levels(p, lvl_r)
        self.draw_info(p, info_r)
        p.end()

    def picture_rect(self, r: QRectF) -> QRectF:
        area = r.adjusted(12, 36, -12, -34)
        img = self.image
        aspect = img.width() / img.height() if img else 4 / 3
        pw = min(area.width(), area.height() * aspect)
        ph = pw / aspect
        return QRectF(area.center().x() - pw / 2, area.center().y() - ph / 2, pw, ph)

    def where(self, angle):
        """-> (rel angle, 'in' | 'left' | 'right') for a raw mic angle."""
        if angle is None:
            return None, "in"
        rel = self.oriented(angle) - self.offset
        if abs(rel) <= self.hfov / 2:
            return rel, "in"
        return rel, "left" if rel > 0 else "right"

    def draw_camera(self, p, r):
        super().draw_camera(p, r)
        if not self.show_captions:
            return
        pic = self.picture_rect(r)
        f = (pic.width() / 2) / math.tan(math.radians(self.hfov / 2))
        with self.cap.lock:
            lines = [ln for ln in self.cap.lines if time.time() - ln["wall"] < CAPTION_HOLD_S][-2:]
            partial = self.cap.partial
            speaking = self.cap.speaking_angle
        p.save()
        p.setClipRect(pic)
        if speaking is not None:
            rel, side = self.where(speaking)
            if side == "in":
                x = pic.center().x() - f * math.tan(math.radians(rel))
                self.draw_speaking_tag(p, x, pic.top() + 12)
        rows = [(ln["text"], ln["angle"], ln["kind"], False) for ln in lines]
        if partial:
            rows.append((partial[0], partial[1], "speech", True))
        rows = rows[-3:]
        if rows:
            self.draw_subtitles(p, pic, rows)
        p.restore()

    def draw_speaking_tag(self, p, x, y):
        p.setFont(cv.font(13, True))
        w = p.fontMetrics().horizontalAdvance("speaking") + 30
        box = QRectF(x - w / 2, y, w, 26)
        p.setPen(QPen(SPEAKER, 2))
        p.setBrush(QColor(13, 17, 23, 215))
        p.drawRoundedRect(box, 13, 13)
        p.setPen(Qt.NoPen)
        p.setBrush(SPEAKER)
        p.drawEllipse(QPointF(box.left() + 13, box.center().y()), 4, 4)
        p.setPen(cv.INK)
        p.drawText(box.adjusted(20, 0, 0, 0), Qt.AlignVCenter | Qt.AlignHCenter, "speaking")

    def draw_subtitles(self, p, pic, rows):
        font = cv.font(max(16, int(pic.width() / 34)), True)
        p.setFont(font)
        fm = p.fontMetrics()
        width = pic.width() * 0.9
        blocks = []
        for text, angle, kind, partial in rows:
            rel, side = self.where(angle)
            if kind == "sound":
                label = f"[BANG{'' if rel is None else ' · ' + self.side_text(rel)}]"
                color = cv.LOUD
            else:
                label = text if len(text) < 160 else "…" + text[-158:]
                if side == "left":
                    label = f"◀ {label}"
                elif side == "right":
                    label = f"{label} ▶"
                color = QColor("#c9d1d9") if partial else cv.INK
            br = fm.boundingRect(QRectF(0, 0, width, 1000).toRect(), Qt.TextWordWrap | Qt.AlignHCenter, label)
            blocks.append((label, color, br.height(), partial))
        total = sum(b[2] for b in blocks) + 10 * len(blocks) + 10
        y = pic.bottom() - 34 - total
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 175))
        p.drawRoundedRect(QRectF(pic.center().x() - width / 2 - 14, y, width + 28, total), 10, 10)
        y += 10
        for label, color, hgt, partial in blocks:
            f2 = QFont(font)
            f2.setItalic(partial)
            p.setFont(f2)
            p.setPen(color)
            p.drawText(QRectF(pic.center().x() - width / 2, y, width, hgt), Qt.TextWordWrap | Qt.AlignHCenter, label)
            y += hgt + 10

    def draw_transcript(self, p, r):
        cv.panel(p, r, "Transcript")
        with self.cap.lock:
            lines = list(self.cap.lines)
            status = self.cap.status
        p.setFont(cv.font(11))
        p.setPen(cv.DIM)
        p.drawText(QRectF(r.right() - 230, r.y() + 10, 216, 18), Qt.AlignRight, status)
        inner = r.adjusted(14, 36, -14, -10)
        p.save()
        p.setClipRect(inner)
        y = inner.bottom()
        for ln in reversed(lines):
            rel, side = self.where(ln["angle"])
            tag = time.strftime("%H:%M:%S", time.localtime(ln["wall"]))
            if rel is not None:
                tag += f" · {self.side_text(rel)}{'' if side == 'in' else ' (off screen)'}"
            p.setFont(cv.font(13, ln["kind"] == "sound"))
            fm = p.fontMetrics()
            br = fm.boundingRect(QRectF(0, 0, inner.width(), 1000).toRect(), Qt.TextWordWrap, ln["text"])
            y -= br.height()
            p.setPen(cv.LOUD if ln["kind"] == "sound" else cv.INK)
            p.drawText(QRectF(inner.left(), y, inner.width(), br.height()), Qt.TextWordWrap, ln["text"])
            y -= 16
            p.setFont(cv.font(11))
            p.setPen(cv.DIM)
            p.drawText(QPointF(inner.left(), y + 12), tag)
            y -= 8
            if y < inner.top():
                break
        if not lines:
            p.setFont(cv.font(14))
            p.setPen(cv.DIM)
            p.drawText(inner, Qt.AlignCenter, "say something")
        p.restore()


class WavDemoReader(cv.Reader):
    """Stands in for the board by playing a 4-channel recording in real time, looped."""

    def __init__(self, wav_path: str, jpeg: bytes | None):
        super().__init__(None)
        with wave.open(wav_path) as w:
            if w.getnchannels() != CH or w.getframerate() != FS:
                raise SystemExit("--demo-wav needs a 4-channel 16 kHz recording")
            self.audio = np.frombuffer(w.readframes(w.getnframes()), "<i2").reshape(-1, CH).copy()
        self.jpeg = jpeg
        self.port = "demo"

    def run(self):
        self.status = "demo (playing a recording)"
        pos, seq, t_frame = 0, 0, 0.0
        t_next = time.time()
        while not self.stop:
            t_next += 256 / FS
            time.sleep(max(0.0, t_next - time.time()))
            chunk = self.audio[pos:pos + 256]
            pos += 256
            if len(chunk) < 256:
                pos = 0
                continue
            now = time.time()
            with self.lock:
                self.chunks.append(chunk)
                if self.jpeg and now - t_frame > 0.1:
                    t_frame = now
                    self.frame = (seq, self.jpeg, 0, 0)
                    seq += 1
                    self.frame_times.append(now)
                    while self.frame_times and now - self.frame_times[0] > 2.0:
                        self.frame_times.popleft()
            self.packets += 1


# ------------------------------------------------------------------ window

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="base.en", help="Whisper model (base.en or small.en)")
    ap.add_argument("--port", help="serial port (default: find the board automatically)")
    ap.add_argument("--wide", type=float, default=14.0, help="ch0/ch1 spacing in cm")
    ap.add_argument("--narrow", type=float, default=3.5, help="ch2/ch3 spacing in cm")
    ap.add_argument("--demo-wav", help="play a 4-channel recording instead of the board")
    ap.add_argument("--demo-image", help="picture to show with --demo-wav")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()

    app = QApplication(sys.argv[:1])
    model = cv.Model([args.wide / 100, args.narrow / 100])
    if args.demo_wav:
        reader = WavDemoReader(args.demo_wav, Path(args.demo_image).read_bytes() if args.demo_image else None)
    else:
        reader = cv.Reader(args.port)
    reader.start()
    captioner = Captioner(args.model)
    captioner.start()

    settings = QSettings("CapstoneMicArray", "mic_caption_view")
    if not settings.contains("hfov"):  # start from the camera view's calibration
        cam = QSettings("CapstoneMicArray", "mic_camera_view")
        for k in ("hfov", "offset", "swap", "rotate", "mirror"):
            if cam.contains(k):
                settings.setValue(k, cam.value(k))
    view = CaptionView(model, reader, captioner)
    view.hfov = float(settings.value("hfov", 60.0))
    view.offset = float(settings.value("offset", 0.0))
    view.swap = settings.value("swap", "false") in (True, "true")
    view.rotate = int(settings.value("rotate", 0))
    view.mirror = settings.value("mirror", "false") in (True, "true")

    win = QWidget()
    win.setWindowTitle("Camera + sound direction + captions (experimental)")
    win.setStyleSheet("QWidget { background: #0d1117; color: #e6edf3; font-size: 14px; }"
                      "QPushButton, QComboBox { background: #21262d; border: 1px solid #30363d;"
                      " border-radius: 6px; padding: 5px 12px; }"
                      "QPushButton:checked { background: #da3633; border-color: #ff7b72; }"
                      "QDoubleSpinBox { background: #21262d; border: 1px solid #30363d; padding: 3px; }")
    status = QLabel()
    status.setStyleSheet("color: #8b949e; font-size: 12px;")
    rec = QPushButton("● Record")
    rec.setToolTip("Saves the 4-mic audio and a transcript text file in mic_takes")
    rec.setCheckable(True)
    snap = QPushButton("Snapshot")
    captions = QCheckBox("Captions")
    captions.setChecked(True)
    focus = QCheckBox("Voice focus")
    focus.setToolTip("Clean the audio before captioning (four mics aimed at the talker, background removed)")
    focus.setChecked(True)
    fov = QDoubleSpinBox()
    fov.setRange(20, 140)
    fov.setDecimals(0)
    fov.setSuffix("° view")
    fov.setValue(view.hfov)
    set_centre = QPushButton("Set centre")
    set_edge = QPushButton("Set right edge")
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
    thresh.setFixedWidth(110)
    thresh_lbl = QLabel()

    row1 = QHBoxLayout()
    for wdg in (rec, snap, captions, focus, QLabel("Camera"), res, rotate, mirror, QLabel("Trigger"), thresh, thresh_lbl):
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
        for k in ("hfov", "offset", "swap", "rotate", "mirror"):
            settings.setValue(k, getattr(view, k))

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
        half = view.offset - a
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
        with captioner.lock:
            if model.recorder:
                path = model.record_path.with_name(model.record_path.stem + "_transcript.txt")
                captioner.transcript_file = open(path, "w")
            elif captioner.transcript_file:
                captioner.transcript_file.close()
                captioner.transcript_file = None
        rec.setText("■ Stop recording" if model.recorder else "● Record")

    def on_snap():
        cv.TAKES.mkdir(parents=True, exist_ok=True)
        path = cv.TAKES / time.strftime("captions_%Y%m%d_%H%M%S.png")
        view.grab().save(str(path))
        hint.setText(f"Saved {path.name} in mic_takes.")

    def on_focus(v):
        captioner.focus.enabled = v

    def on_captions(v):
        view.show_captions = v

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
    focus.toggled.connect(on_focus)
    captions.toggled.connect(on_captions)

    def tick():
        chunks = reader.take()
        model.push(chunks)
        captioner.feed(chunks)
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
    captioner.stop = True
    if model.recorder:
        model.recorder.close()


if __name__ == "__main__":
    main()
