#!/usr/bin/env python3
"""Process recordings from the board's microSD card (cam_mic_record firmware) on the laptop.

    python3 tools/sd_import.py /Volumes/<card>            every take on the card
    python3 tools/sd_import.py /Volumes/<card> --take 3   just T0003
    python3 tools/sd_import.py path/to/T0003              a take folder copied off the card

For each take, writes to mic_takes/sd/T0003/ (or --out):
    raw_4mic.wav      all mics, joined into one file (headers repaired if the power was pulled)
    raw_mic.wav       one raw mic, same loudness as voice_focus.wav, for comparison
    voice_focus.wav   the voice, cleaned (same method as the live captions: it keeps relearning the
                      background, so a fan speeding up or a noise moving is followed)
    directions.csv    every sound the mics located: time, angle, uncertainty, loudness
    transcript.txt    what was said, with times and where the talker was
    subtitles.srt     the same as subtitles, for any video player
    video.mp4         voice + video takes: the camera with the direction overlay and subtitles,
                      audio track 1 = voice focus, track 2 = raw mic
    camera.bin, captions.json   the camera frames and caption directions, for tools/take_viewer.py

Everything runs on this computer, offline. Uses the captions viewer's Python setup
(~/.mic-caption-env: run mic_caption_view.py once if it doesn't exist yet) and ffmpeg for the video.
Camera rotation and calibration come from the viewers' saved settings; override with --rotate etc.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(os.path.abspath(__file__)).parent
ENV_PY = Path.home() / ".mic-caption-env" / ("Scripts/python.exe" if os.name == "nt" else "bin/python")

try:
    import faster_whisper  # noqa: F401
    import numpy  # noqa: F401
    import PySide6  # noqa: F401
except ImportError:
    if ENV_PY.exists() and Path(sys.prefix).resolve() != ENV_PY.parent.parent.resolve():
        os.execv(str(ENV_PY), [str(ENV_PY), os.path.abspath(__file__), *sys.argv[1:]])
    sys.exit("Needs the captions viewer's setup: run `python3 mic_caption_view.py` once (with internet), then retry.")

if __name__ == "__main__":  # only draws video frames; take_viewer.py imports this and needs a real window
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")

import argparse  # noqa: E402
import bisect  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import shutil  # noqa: E402
import struct  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402
import wave  # noqa: E402

import numpy as np  # noqa: E402
from PySide6.QtCore import QPointF, QRectF, QSettings, Qt  # noqa: E402
from PySide6.QtGui import (QColor, QFont, QGuiApplication, QImage, QLinearGradient, QPainter,  # noqa: E402
                           QPen, QPolygonF, QTransform)

sys.path.insert(0, str(HERE.parent))
import mic_camera_view as cv  # noqa: E402
import mic_caption_view as cap  # noqa: E402

FS = 16000
HOP = cap.HOP
OUT_FPS = 10
HOLD_S = cv.HOLD_S
SUB_HOLD_S = 2.0
TAKES_OUT = HERE.parent / "mic_takes" / "sd"


# ------------------------------------------------------------------ reading a take

def read_info(take: Path) -> dict:
    info = {}
    p = take / "info.txt"
    if p.exists():
        for line in p.read_text(errors="replace").splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                info[k.strip()] = v.strip()
    return info


def read_audio(take: Path, channels: int) -> tuple[np.ndarray, int]:
    """All audio parts joined. Reads the sample data directly, so unfinished headers don't matter."""
    parts = sorted(take.glob("audio_*.wav"))
    if not parts:
        raise SystemExit(f"{take}: no audio_*.wav")
    out, repaired = [], 0
    for p in parts:
        raw = p.read_bytes()
        data = raw.find(b"data")
        if raw[:4] != b"RIFF" or data < 0:
            raise SystemExit(f"{p}: not a WAV file")
        start = data + 8
        (declared,) = struct.unpack_from("<I", raw, data + 4)
        n = (len(raw) - start) // (2 * channels) * 2 * channels
        if declared != n:
            repaired += 1
        out.append(np.frombuffer(raw[start:start + n], "<i2").reshape(-1, channels))
    return np.concatenate(out), repaired


def read_video(take: Path) -> list[tuple[int, bytes]]:
    return read_frames(take / "video.bin")


def read_frames(p: Path) -> list[tuple[int, bytes]]:
    """(audio sample at capture, JPEG) per camera frame in a video.bin."""
    if not p.exists():
        return []
    raw, frames, i = p.read_bytes(), [], 0
    while i + 22 <= len(raw):
        if raw[i:i + 2] != b"VF":
            j = raw.find(b"VF", i + 1)
            if j < 0:
                break
            i = j
            continue
        _, sample, length, w, h = struct.unpack_from("<IQIHH", raw, i + 2)
        if i + 22 + length > len(raw) or not (0 < w <= 2048 and 0 < h <= 2048):
            break  # last frame cut off by a power loss
        frames.append((sample, raw[i + 22:i + 22 + length]))
        i += 22 + length
    frames.sort(key=lambda f: f[0])
    return frames


def write_wav(path: Path, x: np.ndarray):
    x = x if x.ndim == 2 else x[:, None]
    with wave.open(str(path), "wb") as w:
        w.setnchannels(x.shape[1])
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(np.clip(np.round(x), -32767, 32767).astype("<i2").tobytes())


# ------------------------------------------------------------------ analysis

def directions(audio: np.ndarray, spacings: list[float], threshold_db: float) -> list[tuple]:
    """The live viewer's sound locator run over the recording (same triggers, same math)."""
    model = cv.Model(spacings)
    model.threshold_db = threshold_db
    step, last, out = int(0.033 * FS), None, []
    for i in range(0, len(audio), step):
        model.push([audio[i:i + step]])
        det = model.last_det
        if det is not None and det is not last:
            out.append(det)
            last = det
    return out


def voice_focus(audio: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[float]]:
    """Live voice focus over the whole take. A first pass over the opening 20 s learns the background
    and the talker, so the start of the recording is cleaned as well as the rest."""
    x = audio.astype(np.float64)
    x -= x.mean(axis=0)
    focus = cap.StreamingFocus()
    for _ in focus.process(x[:20 * FS]):
        pass
    focus.inbuf = np.zeros((0, x.shape[1]))
    focus.ola = np.zeros(cap.NFFT)
    focus.wide_hist.clear()
    out, speech, bangs, last_bang = [], [], [], -1.0
    padded = np.concatenate([x, np.zeros((cap.NFFT, x.shape[1]))])
    for k, (o, _, sp, bang) in enumerate(focus.process(padded)):
        out.append(o)
        speech.append(sp)
        t = (k * HOP + cap.NFFT / 2) / FS
        if bang and t - last_bang > 0.6:
            bangs.append(t)
            last_bang = t
    y = np.concatenate(out)[:len(x)]
    return y, np.array(speech, bool), bangs


def level_matched(clean: np.ndarray, ref: np.ndarray, speech: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mask = np.repeat(speech, HOP)[:len(ref)]
    if mask.sum() < FS // 4:
        mask = np.ones(len(ref), bool)
    gain = math.sqrt(np.mean(ref[mask] ** 2) / (np.mean(clean[mask] ** 2) + 1e-12))
    clean = clean * gain
    peak = max(np.max(np.abs(clean)), np.max(np.abs(ref)), 1.0)
    scale = min(4.0, 30000 / peak)
    return clean * scale, ref * scale


def transcribe(clean: np.ndarray, model_name: str) -> list[tuple[float, float, str]]:
    from faster_whisper import WhisperModel
    try:
        model = WhisperModel(model_name, device="cpu", compute_type="int8", local_files_only=True)
    except Exception:  # noqa: BLE001
        print(f"  speech model {model_name} isn't on this computer yet: downloading it once…")
        model = WhisperModel(model_name, device="cpu", compute_type="int8")
    a = clean.astype(np.float32) / 32768.0
    peak = float(np.max(np.abs(a))) if len(a) else 0.0
    if peak < 1e-5:
        return []
    a *= 0.5 / peak
    segs, _ = model.transcribe(a, language="en", beam_size=5, vad_filter=True, condition_on_previous_text=False)
    out = []
    for s in segs:
        text = s.text.strip()
        if s.no_speech_prob < 0.6 and s.avg_logprob > -1.0 and text and text.lower() not in cap.HALLUCINATIONS:
            out.append((s.start, s.end, text))
    return out


# ------------------------------------------------------------------ calibration (from the viewers' settings)

class Calib:
    def __init__(self, args):
        s = QSettings("CapstoneMicArray", "mic_caption_view")
        if s.value("rotate") is None:
            s = QSettings("CapstoneMicArray", "mic_camera_view")
        def pick(name, default, conv):
            v = getattr(args, name)
            return v if v is not None else conv(s.value(name, default))
        truth = lambda v: v in (True, "true", "1", 1)  # noqa: E731
        self.hfov = pick("hfov", 60.0, float)
        self.offset = pick("offset", 0.0, float)
        self.swap = pick("swap", "false", truth)
        self.rotate = pick("rotate", 0, int)
        self.mirror = pick("mirror", "false", truth)

    def rel(self, mic_angle):
        """Mic angle -> angle from the picture centre, + = left."""
        if mic_angle is None:
            return None
        return (-mic_angle if self.swap else mic_angle) - self.offset

    def side(self, rel):
        if rel is None:
            return ""
        where = "straight ahead" if abs(rel) < 2 else f"{abs(rel):.0f}° {'left' if rel > 0 else 'right'}"
        return where + ("" if abs(rel) <= self.hfov / 2 else " (off screen)")


def clock(t: float, srt=False) -> str:
    h, rem = divmod(t, 3600)
    m, s = divmod(rem, 60)
    if srt:
        return f"{int(h):02d}:{int(m):02d}:{int(s):02d},{int(round((s % 1) * 1000)) % 1000:03d}"
    return f"{int(h)}:{int(m):02d}:{s:04.1f}" if h else f"{int(m):02d}:{s:04.1f}"


# ------------------------------------------------------------------ video

def font(px, bold=False):
    f = QFont()
    f.setFamilies(["Helvetica", "Arial", "DejaVu Sans"])
    f.setPixelSize(px)
    f.setBold(bold)
    return f


class Renderer:
    def __init__(self, calib: Calib, dets, captions, bangs, frames, title):
        self.c, self.dets, self.caps, self.bangs, self.frames, self.title = calib, dets, captions, bangs, frames, title
        self.det_times = [d[0] for d in dets]
        self.frame_times = [s / FS for s, _ in frames]
        self._cache = (None, None)
        first = self.picture(0)
        scale = 720 / max(first.width(), first.height())
        self.W = int(first.width() * scale) // 4 * 4
        self.H = int(first.height() * scale) // 2 * 2

    def picture(self, i) -> QImage:
        if self._cache[0] == i:
            return self._cache[1]
        img = QImage.fromData(self.frames[i][1], "JPG")
        t = QTransform()
        if self.c.rotate:
            t.rotate(self.c.rotate)
        if self.c.mirror:
            t.scale(-1, 1)
        if self.c.rotate or self.c.mirror:
            img = img.transformed(t, Qt.SmoothTransformation)
        self._cache = (i, img)
        return img

    def frame(self, t: float) -> bytes:
        i = max(0, bisect.bisect_right(self.frame_times, t) - 1)
        out = QImage(self.W, self.H, QImage.Format_RGB888)
        out.fill(QColor("#000000"))
        p = QPainter(out)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        p.drawImage(QRectF(0, 0, self.W, self.H), self.picture(i))
        f = (self.W / 2) / math.tan(math.radians(self.c.hfov / 2))

        def x_of(rel):
            return self.W / 2 - f * math.tan(math.radians(max(-89.0, min(89.0, rel))))

        lo = bisect.bisect_left(self.det_times, t - HOLD_S)
        hi = bisect.bisect_right(self.det_times, t)
        recent = self.dets[lo:hi]
        for det in recent:
            td, angle, sigma, _, strength, _ = det
            fade = 1 - (t - td) / HOLD_S
            rel = self.c.rel(angle)
            color = cv.LOUD if strength > 30 else cv.GLOW
            latest = det is recent[-1]
            if abs(rel) <= self.c.hfov / 2:
                x = x_of(rel)
                hw = max(14.0, abs(x_of(rel + sigma) - x_of(rel - sigma)) / 2) * 2
                g = QLinearGradient(x - hw, 0, x + hw, 0)
                g.setColorAt(0.0, cv.with_alpha(color, 0))
                g.setColorAt(0.5, cv.with_alpha(color, (0.55 if latest else 0.25) * fade))
                g.setColorAt(1.0, cv.with_alpha(color, 0))
                p.setPen(Qt.NoPen)
                p.setBrush(g)
                p.drawRect(QRectF(x - hw, 0, 2 * hw, self.H))
                p.setPen(QPen(cv.with_alpha(QColor("#fff3d6"), fade), 2.5 if latest else 1.5))
                p.drawLine(QPointF(x, 0), QPointF(x, self.H))
                if latest:
                    self.tag(p, x + 10, self.H * 0.16, f"{self.c.side(rel)} · ±{sigma:.0f}° · {strength:.0f} dB", color)
            elif latest:
                self.arrow(p, rel > 0, fade, color, f"{self.c.side(rel)}")

        for start, end, text, angle in self.caps:
            if start - 0.3 <= t <= end and angle is not None:
                rel = self.c.rel(angle)
                if abs(rel) <= self.c.hfov / 2:
                    self.speaking(p, x_of(rel), 12)
        lines = []
        for start, end, text, angle in self.caps:
            if start <= t <= end + SUB_HOLD_S:
                rel = self.c.rel(angle)
                if rel is not None and abs(rel) > self.c.hfov / 2:
                    text = f"◀ {text}" if rel > 0 else f"{text} ▶"
                lines.append((text, cv.INK))
        for tb, angle in self.bangs:
            if tb <= t <= tb + SUB_HOLD_S:
                lines.append((f"[BANG{'' if angle is None else ' · ' + self.c.side(self.c.rel(angle))}]", cv.LOUD))
        if lines:
            self.subtitles(p, lines[-2:])
        p.setFont(font(13, True))
        p.setPen(QColor(255, 255, 255, 200))
        p.drawText(QRectF(10, self.H - 26, self.W - 20, 20), Qt.AlignRight | Qt.AlignVCenter, f"{self.title}  {clock(t)}")
        p.end()
        b = out.constBits()
        arr = np.frombuffer(b, np.uint8, out.bytesPerLine() * self.H).reshape(self.H, out.bytesPerLine())
        return arr[:, :self.W * 3].tobytes()

    def tag(self, p, x, y, text, color, right=False):
        p.setFont(font(14, True))
        w = p.fontMetrics().horizontalAdvance(text) + 18
        x = min(max(x - w if right else x, 6), self.W - w - 6)
        p.setPen(QPen(color, 2))
        p.setBrush(QColor(13, 17, 23, 215))
        p.drawRoundedRect(QRectF(x, y, w, 26), 7, 7)
        p.setPen(cv.INK)
        p.drawText(QRectF(x, y, w, 26), Qt.AlignCenter, text)

    def speaking(self, p, x, y):
        p.setFont(font(13, True))
        w = p.fontMetrics().horizontalAdvance("speaking") + 30
        box = QRectF(min(max(x - w / 2, 6), self.W - w - 6), y, w, 26)
        p.setPen(QPen(cap.SPEAKER, 2))
        p.setBrush(QColor(13, 17, 23, 215))
        p.drawRoundedRect(box, 13, 13)
        p.setPen(Qt.NoPen)
        p.setBrush(cap.SPEAKER)
        p.drawEllipse(QPointF(box.left() + 13, box.center().y()), 4, 4)
        p.setPen(cv.INK)
        p.drawText(box.adjusted(20, 0, 0, 0), Qt.AlignCenter, "speaking")

    def arrow(self, p, left, fade, color, text):
        y = self.H * 0.45
        tip = 8 if left else self.W - 8
        d = 1 if left else -1
        poly = QPolygonF([QPointF(tip, y), QPointF(tip + d * 30, y - 26), QPointF(tip + d * 30, y - 11),
                          QPointF(tip + d * 60, y - 11), QPointF(tip + d * 60, y + 11),
                          QPointF(tip + d * 30, y + 11), QPointF(tip + d * 30, y + 26)])
        p.setPen(QPen(cv.with_alpha(QColor("#1a1206"), fade), 2))
        p.setBrush(cv.with_alpha(color, 0.35 + 0.65 * fade))
        p.drawPolygon(poly)
        self.tag(p, tip + d * 66, y - 13, text, color, right=not left)

    def subtitles(self, p, lines):
        p.setFont(font(max(16, self.W // 30), True))
        fm = p.fontMetrics()
        width = self.W * 0.92
        boxes = [fm.boundingRect(QRectF(0, 0, width, 1000).toRect(), Qt.TextWordWrap | Qt.AlignHCenter, t).height()
                 for t, _ in lines]
        total = sum(boxes) + 8 * len(lines) + 8
        y = self.H - 34 - total
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 175))
        p.drawRoundedRect(QRectF((self.W - width) / 2 - 8, y, width + 16, total), 8, 8)
        y += 8
        for (text, color), h in zip(lines, boxes):
            p.setPen(color)
            p.drawText(QRectF((self.W - width) / 2, y, width, h), Qt.TextWordWrap | Qt.AlignHCenter, text)
            y += h + 8


def render_video(out_dir: Path, renderer: Renderer, duration: float):
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        print("  ffmpeg not found: skipping video.mp4 (brew install ffmpeg)")
        return False
    cmd = [ffmpeg, "-y", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{renderer.W}x{renderer.H}", "-r", str(OUT_FPS), "-i", "-",
           "-i", str(out_dir / "voice_focus.wav"), "-i", str(out_dir / "raw_mic.wav"),
           "-map", "0:v", "-map", "1:a", "-map", "2:a",
           "-c:v", "libx264", "-preset", "veryfast", "-crf", "22", "-pix_fmt", "yuv420p",
           "-c:a", "aac", "-b:a", "96k",
           "-metadata:s:a:0", "title=Voice focus", "-metadata:s:a:1", "title=Raw mic",
           # MP4 players show the handler name as the track name
           "-metadata:s:a:0", "handler_name=Voice focus", "-metadata:s:a:1", "handler_name=Raw mic",
           "-metadata:s:a", "language=eng",
           "-shortest", str(out_dir / "video.mp4")]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    n = int(math.ceil(duration * OUT_FPS))
    for k in range(n):
        proc.stdin.write(renderer.frame(k / OUT_FPS))
    proc.stdin.close()
    return proc.wait() == 0


# ------------------------------------------------------------------ one take

def process(take: Path, out_root: Path, args, calib: Calib):
    t0 = time.time()
    info = read_info(take)
    channels = int(info.get("mics", 4))
    if channels != cv.CH:
        print(f"{take.name}: {channels} mics; this tool handles {cv.CH} for now, skipping")
        return
    audio, repaired = read_audio(take, channels)
    frames = [] if args.no_video else read_video(take)
    dur = len(audio) / FS
    out = out_root / take.name
    out.mkdir(parents=True, exist_ok=True)
    print(f"{take.name}: {dur:.1f} s, {info.get('mode', '?')}, {len(frames)} camera frames"
          + (f", {repaired} unfinished WAV header(s) repaired" if repaired else "")
          + (f", {info['lost_audio_frames']} audio frames lost on the card" if info.get("lost_audio_frames", "0") != "0" else ""))
    write_wav(out / "raw_4mic.wav", audio)

    spacings = [args.wide / 100, args.narrow / 100]
    dets = directions(audio, spacings, args.threshold)
    with open(out / "directions.csv", "w") as f:
        f.write("time_s,mic_angle_deg,uncertainty_deg,loudness_db,pairs_agree,wide_pair_deg,narrow_pair_deg,"
                "picture_angle_deg,where\n")
        for td, angle, sigma, agree, strength, angles in dets:
            rel = calib.rel(angle)
            a0, a1 = (f"{a:.1f}" if a is not None else "" for a in angles)
            f.write(f"{td:.3f},{angle:.1f},{sigma:.1f},{strength:.1f},{'' if agree is None else int(agree)},"
                    f"{a0},{a1},{rel:.1f},{calib.side(rel)}\n")
    print(f"  directions: {len(dets)} sounds located")

    clean, speech, bang_times = voice_focus(audio)
    clean, ref = level_matched(clean, audio[:, args.ref].astype(np.float64) - audio[:, args.ref].mean(), speech)
    write_wav(out / "voice_focus.wav", clean)
    write_wav(out / "raw_mic.wav", ref)
    print(f"  voice focus: done, speech in {speech.mean() * 100:.0f}% of the take")

    segs = transcribe(clean, args.model)
    caps = []
    for start, end, text in segs:
        a, b = int(start * FS), int(end * FS)
        caps.append((start, end, text, cap.angle_of(audio[a:b].astype(np.float64))))
    bangs = []
    for tb in bang_times:
        a = int(tb * FS)
        bangs.append((tb, cap.angle_of(audio[max(0, a - 1200):a + 400].astype(np.float64))))
    events = [(s, e, txt, ang) for s, e, txt, ang in caps] + [(tb, tb + 0.5, "[BANG]", ang) for tb, ang in bangs]
    events.sort(key=lambda e: e[0])
    with open(out / "transcript.txt", "w") as f:
        f.write(f"{take.name}  {dur:.1f} s  {info.get('mode', '')}\n\n")
        for s, e, txt, ang in events:
            where = calib.side(calib.rel(ang))
            f.write(f"{clock(s)}–{clock(e)}  {where + '  ' if where else ''}{txt}\n")
    with open(out / "subtitles.srt", "w") as f:
        for i, (s, e, txt, ang) in enumerate(events, 1):
            rel = calib.rel(ang)
            if txt != "[BANG]" and rel is not None and abs(rel) > calib.hfov / 2:
                txt = f"◀ {txt}" if rel > 0 else f"{txt} ▶"
            elif txt == "[BANG]" and rel is not None:
                txt = f"[BANG · {calib.side(rel)}]"
            f.write(f"{i}\n{clock(s, True)} --> {clock(e + (SUB_HOLD_S if txt.startswith('[') else 0.3), True)}\n{txt}\n\n")
    with open(out / "captions.json", "w") as f:
        json.dump({"captions": [[s, e, txt, ang] for s, e, txt, ang in caps],
                   "bangs": [[tb, ang] for tb, ang in bangs]}, f)
    print(f"  transcript: {len(caps)} lines, {len(bangs)} sudden loud sounds")
    if (take / "video.bin").exists():
        shutil.copyfile(take / "video.bin", out / "camera.bin")  # so take_viewer.py can redraw the overlay

    if frames:
        r = Renderer(calib, dets, caps, bangs, frames, take.name)
        if render_video(out, r, dur):
            print(f"  video: video.mp4 ({r.W}x{r.H}, {OUT_FPS} fps)")
    print(f"  saved in {out}  ({time.time() - t0:.0f} s)")


def find_takes(path: Path, number: int | None) -> list[Path]:
    if (path / "info.txt").exists() or list(path.glob("audio_*.wav")):
        return [path]
    root = path / "takes" if (path / "takes").is_dir() else path
    takes = sorted(p for p in root.glob("T[0-9]*") if p.is_dir())
    if number is not None:
        takes = [p for p in takes if p.name == f"T{number:04d}"]
    return takes


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="the card (or its takes/ folder), or one take folder")
    ap.add_argument("--take", type=int, help="only this take number")
    ap.add_argument("--out", type=Path, default=TAKES_OUT, help=f"where to write (default {TAKES_OUT})")
    ap.add_argument("--model", default="base.en", help="Whisper model (base.en, small.en, …)")
    ap.add_argument("--no-video", action="store_true", help="skip video.mp4")
    ap.add_argument("--ref", type=int, default=1, help="mic used for raw_mic.wav (default ch1)")
    ap.add_argument("--threshold", type=float, default=12.0, help="direction trigger, dB over quiet (default 12)")
    ap.add_argument("--wide", type=float, default=14.0, help="ch0/ch1 spacing in cm")
    ap.add_argument("--narrow", type=float, default=3.5, help="ch2/ch3 spacing in cm")
    ap.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), help="camera rotation (default: viewer setting)")
    ap.add_argument("--mirror", action="store_true", default=None)
    ap.add_argument("--hfov", type=float, help="camera field of view in degrees (default: viewer setting)")
    ap.add_argument("--offset", type=float, help="mic angle of the picture centre (default: viewer setting)")
    ap.add_argument("--swap", action="store_true", default=None, help="swap left/right")
    args = ap.parse_args()

    takes = find_takes(Path(args.path), args.take)
    if not takes:
        raise SystemExit(f"no takes found in {args.path}")
    app = QGuiApplication.instance() or QGuiApplication([])  # noqa: F841
    calib = Calib(args)
    print(f"camera: rotate {calib.rotate}°, field of view {calib.hfov:.0f}°, centre {calib.offset:+.0f}°"
          f"{', swapped' if calib.swap else ''}")
    for take in takes:
        process(take, args.out, args, calib)


if __name__ == "__main__":
    main()
