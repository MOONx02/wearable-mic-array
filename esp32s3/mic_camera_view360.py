#!/usr/bin/env python3
"""360° camera + sound direction view for the vest mic array (cam_mic_record8 firmware).

    ~/.mic-caption-env/bin/python mic_camera_view360.py
    ~/.mic-caption-env/bin/python mic_camera_view360.py --demo [JPEG]     # no board

Options:
    --port PORT         serial port, if auto-detect picks the wrong one
    --geometry FILE     mic positions as JSON, {"<channel>": ["name", x_mm, y_mm], ...}
                        x = toward the wearer's left, y = toward the front, from the array centre.
                        Default: the 6-mic breadboard (mic-array-6-wiring.png with the outer front
                        pair on D1 and the inner pair on D3).
    --compact / --no-compact
                        full-screen touch layout for small screens such as the Raspberry Pi 7"
                        display (default: on when the screen is under 1100 px wide; Esc or ✕ quits)
    --model NAME        Whisper model for the live subtitles (default base.en)
    --no-captions       no subtitles (they need faster-whisper; see mic_caption_view.py)

Subtitles reuse mic_caption_view.py: the four front mics are aimed at the talker (voice focus),
Whisper turns the speech into text on this computer, and each line is placed in the picture by
the 360° direction of the talker, with ◀ / ▶ when they're out of frame.

Same picture as mic_camera_view.py, but every working mic pair is used at once (SRP-PHAT), and
the back mics tell front from behind. A sound inside the picture glows where it is; outside the
picture an arrow on the left or right edge says how far round it is and whether it's in front or
behind; a sound behind you gets an arrow at the bottom. With no back mic working, it falls back
to front/back-ambiguous like the 4-mic app.

Calibrate as before: clap straight in front of the camera → "Set centre", clap at the right edge
of the picture → "Set right edge". If sounds on your left show on the right, tick "Swap
left/right"; if sounds in front show behind, tick "Swap front/back".
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(os.path.abspath(__file__)).parent))
import mic_camera_view as cv  # noqa: E402  (also runs its first-run package setup)

import argparse  # noqa: E402
import math  # noqa: E402
import struct  # noqa: E402
import time  # noqa: E402
import wave  # noqa: E402

import numpy as np  # noqa: E402
import serial  # noqa: E402
from PySide6.QtCore import QPointF, QRectF, QSettings, Qt, QTimer  # noqa: E402
from PySide6.QtGui import QColor, QKeySequence, QLinearGradient, QPainter, QPen, QPolygonF, QShortcut  # noqa: E402
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QHBoxLayout,  # noqa: E402
                               QLabel, QPushButton, QSizePolicy, QSlider, QVBoxLayout, QWidget)

from mic_camera_view import (BG, DIM, EDGE, FS, GLOW, HISTORY_S, HOLD_S, INK, LOUD, C, ALERT,  # noqa: E402
                             font, label_box, panel, with_alpha)

mcv = None  # mic_caption_view, imported only when captions are on

CH = 8
SEG = 1024                 # samples per direction estimate (64 ms)
INTERP = 8                 # GCC sub-sample resolution
BAND = (300.0, 6000.0)
STUCK_FRACTION = 0.5       # a live mic almost never repeats a sample; a dead data line mostly does
BEHIND_DEG = 135.0         # beyond this from the camera centre, point the arrow down ("behind you")
FB_SURE = 0.25             # front/back margin (share of the peak) needed to call it

# channel: (name, x mm toward the wearer's left, y mm toward the front)
DEFAULT_GEOMETRY = {
    0: ("front outer L", 68.6, 22.0),
    1: ("front outer R", -68.6, 22.0),
    2: ("front inner L", 22.9, 22.0),
    3: ("front inner R", -22.9, 22.0),
    5: ("back R", -68.6, -22.0),
    7: ("back L", 68.6, -22.0),
}
GEOM_COLORS = [QColor("#58a6ff"), QColor("#f0883e"), QColor("#3fb950"), QColor("#d2a8ff"),
               QColor("#ff7b72"), QColor("#e3b341"), QColor("#79c0ff"), QColor("#56d4dd")]


def wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def load_geometry(path: str | None) -> dict:
    if not path:
        return dict(DEFAULT_GEOMETRY)
    raw = json.loads(Path(path).read_text())
    return {int(k): (str(v[0]), float(v[1]), float(v[2])) for k, v in raw.items()}


# ------------------------------------------------------------------ serial reader (M8 + JPEG)

class Reader8(cv.Reader):
    def __init__(self, port):
        super().__init__(port)
        self.wrong_firmware = False

    def _probe(self, path):
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
            if cv.SYNC + b"M8" in buf:
                return s
        if cv.SYNC + b"M4" in buf:
            self.wrong_firmware = True
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
                self.status = ("board is sending 4 mics: flash cam_mic_record8" if self.wrong_firmware
                               else "no 8-mic stream found: plug in the board (cam_mic_record8 firmware)")
                time.sleep(1.0)
                continue
            self.status = "streaming"
            self._stream(s)
            self.status = "connection lost, reconnecting…"
            self.port = None

    def _stream(self, s):
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
                i = buf.find(cv.SYNC)
                if i < 0:
                    del buf[:-1]
                    break
                if i:
                    del buf[:i]
                if len(buf) < 4:
                    break
                kind = bytes(buf[2:4])
                if kind == b"M8":
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


class Demo360Reader(cv.Reader):
    """No board: claps from directions all the way round, through the given geometry."""

    ANGLES = (0, 40, -25, 120, -150, 180, 75, -100, 15, 160)

    def __init__(self, geometry: dict, jpeg: bytes | None):
        super().__init__(None)
        self.geom = geometry
        self.jpeg = jpeg
        self.port = "demo"

    def _clap(self, ang: float, n: int = 2048) -> np.ndarray:
        rng = np.random.default_rng()
        burst = rng.normal(0, 1, n) * np.exp(-np.arange(n) / 250.0)
        burst[:200] = 0
        F = np.fft.rfft(burst)
        f = np.fft.rfftfreq(n, 1 / FS)
        u = np.array([math.sin(math.radians(ang)), math.cos(math.radians(ang))])
        out = np.zeros((n, CH))
        for ch, (_, x, y) in self.geom.items():
            delay = -(np.array([x, y]) / 1000 @ u) / C
            out[:, ch] = np.fft.irfft(F * np.exp(-2j * math.pi * f * delay), n)
        return out / np.max(np.abs(out)) * 12000

    def run(self):
        self.status = "demo (no board)"
        self.board_msgs.append((time.time(), "demo: claps from all round the array"))
        rng = np.random.default_rng()
        pending = np.zeros((0, CH))
        t_next, k, seq, t_frame = time.time() + 1.0, 0, 0, 0.0
        used = list(self.geom)
        while not self.stop:
            time.sleep(0.016)
            chunk = np.full((256, CH), -30935.0)
            chunk[:, used] = rng.normal(0, 25, (256, len(used)))
            now = time.time()
            if now >= t_next:
                pending = self._clap(self.ANGLES[k % len(self.ANGLES)])
                k += 1
                t_next = now + 1.4
            if len(pending):
                m = min(256, len(pending))
                chunk[:m, used] += pending[:m, used]
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

class Model360(cv.Model):
    def __init__(self, geometry: dict):
        super().__init__([0.1, 0.1])
        self.geom = geometry
        self.x = np.zeros((FS * 2, CH), np.float32)
        self.level = np.full(CH, -120.0)
        self.floor = np.full(CH, -70.0)
        self.dead = np.ones(CH, bool)
        self.echo = np.zeros(CH, bool)
        self.dc = np.zeros(CH)
        self.curve = None
        self._steer: dict = {}

    def _levels(self):
        if self.total >= FS // 2:
            recent = self.x[-FS // 2:]
            stuck = np.mean(np.diff(recent, axis=0) == 0, axis=0) > STUCK_FRACTION
            self.dead = (np.ptp(recent, axis=0) < cv.DEAD_PTP) | stuck
            for a in range(0, CH, 2):
                same = not self.dead[a] and np.array_equal(recent[:, a], recent[:, a + 1])
                self.echo[a] = self.echo[a + 1] = same
        super()._levels()

    def source(self, ch: int) -> int | None:
        """Channel that carries the mic at geometry slot ch: itself, or its line partner when the
        mic is actually on the other SEL slot (or the line just echoes it)."""
        if not self.dead[ch]:
            return ch
        other = ch ^ 1
        if not self.dead[other] and other not in self.geom:
            return other
        return None

    def active(self) -> list[tuple[int, int]]:
        out = []
        for ch in sorted(self.geom):
            src = self.source(ch)
            if src is not None:
                out.append((ch, src))
        return out

    def _steering(self, chans: tuple):
        if chans in self._steer:
            return self._steer[chans]
        pos = np.array([[self.geom[c][1], self.geom[c][2]] for c in chans]) / 1000.0
        th = np.radians(np.arange(-180, 180))
        u = np.stack([np.sin(th), np.cos(th)], axis=1)
        t = -(pos @ u.T) / C  # arrival time of each mic for each direction, (M, 360)
        pairs = [(i, j) for i in range(len(chans)) for j in range(i + 1, len(chans))]
        max_lag = int(math.ceil(math.hypot(*np.ptp(pos, axis=0)) / C * FS)) + 2
        lags = np.array([(t[j] - t[i]) * FS * INTERP for i, j in pairs])
        idx = np.round(lags).astype(int) + max_lag * INTERP
        has_back = np.ptp(pos[:, 1]) > 0.01
        self._steer[chans] = (pairs, idx, max_lag, has_back)
        return self._steer[chans]

    def srp(self, seg: np.ndarray, chans: tuple):
        pairs, idx, max_lag, has_back = self._steering(chans)
        n = 1 << int(math.ceil(math.log2(2 * len(seg))))
        F = np.fft.rfft(seg, n, axis=0)
        f = np.fft.rfftfreq(n, 1 / FS)
        out_band = (f < BAND[0]) | (f > BAND[1])
        m = max_lag * INTERP
        curve = np.zeros(360)
        for k, (i, j) in enumerate(pairs):
            cross = F[:, j] * np.conj(F[:, i])
            cross /= np.abs(cross) + 1e-12
            cross[out_band] = 0
            cc = np.fft.irfft(cross, n * INTERP)
            cc = np.concatenate((cc[-m:], cc[: m + 1]))
            curve += cc[np.clip(idx[k], 0, 2 * m)]
        return curve, has_back

    def _direction(self):
        act = self.active()
        if self.total < FS // 2 or len(act) < 2:
            return
        srcs = [s for _, s in act]
        strength = float(np.max((self.level - self.floor)[srcs]))
        if strength < self.threshold_db:
            return
        if self.last_det and self.now - self.last_det[0] < 0.08:
            return
        chans = tuple(c for c, _ in act)
        curve, has_back = self.srp(self.x[-SEG:, srcs], chans)
        k = int(np.argmax(curve))
        angle = float(k - 180)
        peak, base = curve[k], float(np.median(curve))
        span = max(peak - base, 1e-9)
        above = curve >= peak - 0.3 * span
        lo = hi = k
        while above[(lo - 1) % 360] and k - lo < 179:
            lo -= 1
        while above[(hi + 1) % 360] and hi - k < 179:
            hi += 1
        sigma = float(min(60.0, max(3.0, (hi - lo) / 2)))
        fb = None
        if has_back:
            mirror = int(round(wrap(180.0 - angle))) + 180
            fb = float((peak - curve[mirror % 360]) / span)
        elif abs(angle) > 90:
            angle = math.copysign(180.0 - abs(angle), angle)
        self.curve = curve
        det = (self.now, angle, sigma, fb, strength, curve)
        self.detections.append(det)
        self.last_det = det
        while self.detections and self.now - self.detections[0][0] > HISTORY_S:
            self.detections.popleft()

    def toggle_record(self):
        if self.recorder:
            self.recorder.close()
            self.recorder = None
            return None
        cv.TAKES.mkdir(parents=True, exist_ok=True)
        self.record_path = cv.TAKES / time.strftime("cam8_%Y%m%d_%H%M%S.wav")
        w = wave.open(str(self.record_path), "wb")
        w.setnchannels(CH)
        w.setsampwidth(2)
        w.setframerate(FS)
        self.recorder = w
        return str(self.record_path)


# ------------------------------------------------------------------ drawing

class View360(cv.View):
    def __init__(self, model: Model360, reader, compact: bool = False, captioner=None):
        super().__init__(model, reader)
        self.swap_fb = False
        self.compact = compact
        self.cap = captioner
        self.show_captions = captioner is not None
        if compact:
            self.setMinimumSize(0, 0)

    def paintEvent(self, ev):
        if not self.compact and self.cap is None:
            return super().paintEvent(ev)
        if not self.compact:
            p = QPainter(self)
            p.setRenderHint(QPainter.Antialiasing)
            p.fillRect(self.rect(), BG)
            w, h = self.width(), self.height()
            gap = 14
            left_w = w * 0.68
            cam_r = QRectF(gap, gap, left_w - 1.5 * gap, h - 2 * gap)
            x1 = left_w + 0.5 * gap
            cw = w - x1 - gap
            gauge_r = QRectF(x1, gap, cw, h * 0.36)
            tr_r = QRectF(x1, gauge_r.bottom() + gap, cw, h * 0.28)
            lvl_r = QRectF(x1, tr_r.bottom() + gap, cw, h * 0.20)
            info_r = QRectF(x1, lvl_r.bottom() + gap, cw, h - lvl_r.bottom() - 2 * gap)
            self.draw_camera(p, cam_r)
            self.draw_gauge(p, gauge_r)
            mcv.CaptionView.draw_transcript(self, p, tr_r)
            self.draw_levels(p, lvl_r)
            self.draw_info(p, info_r)
            p.end()
            return
        # small screens (Pi 7" touchscreen, 800x480): camera | radar over mic levels, no board panel
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), BG)
        w, h = self.width(), self.height()
        gap = 6
        left_w = w * 0.56
        cam_r = QRectF(gap, gap, left_w - 1.5 * gap, h - 2 * gap)
        x1 = left_w + 0.5 * gap
        gauge_r = QRectF(x1, gap, w - x1 - gap, h * 0.66)
        lvl_r = QRectF(x1, gauge_r.bottom() + gap, w - x1 - gap, h - gauge_r.bottom() - 2 * gap)
        self.draw_camera(p, cam_r)
        self.draw_gauge(p, gauge_r)
        self.draw_levels(p, lvl_r)
        p.end()

    def oriented(self, ang):
        """Array angle -> + = wearer's left, 0 = front, ±180 = behind."""
        if ang is None:
            return None
        a = -ang if self.swap else ang
        return wrap(180.0 - a) if self.swap_fb else wrap(a)

    def side_text(self, rel: float) -> str:
        if abs(rel) < 2:
            return "straight ahead"
        if abs(rel) > 178:
            return "directly behind"
        return f"{abs(rel):.0f}° {'left' if rel > 0 else 'right'}"

    @staticmethod
    def fb_text(det) -> str:
        fb = det[3]
        if fb is None:
            return "front/back unknown (no back mic)"
        return "front/back sure" if fb >= FB_SURE else "front/back unsure"

    def draw_camera(self, p, r):
        panel(p, r, "Camera + sound direction (360°)")
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
            rel = wrap(self.oriented(angle) - self.offset)
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
                    lines = [f"{'LOUD · ' if strength > 30 else ''}Sound · {'near the edge' if near else 'in view'}",
                             f"{self.side_text(rel)} · ±{sigma:.0f}° · {strength:.0f} dB over quiet",
                             f"{now - t:.1f} s ago · {self.fb_text(det)}"]
                    label_box(p, x + 14, pic.top() + 0.18 * pic.height(), lines, color, pic)
            elif latest:
                if abs(rel) >= BEHIND_DEG:
                    self.draw_behind_arrow(p, pic, rel, sigma, strength, now - t, fade, color, det)
                else:
                    self.draw_side_arrow(p, pic, rel, sigma, strength, now - t, fade, color, det)
        p.restore()
        self.draw_captions(p, pic, f)
        p.setFont(font(12))
        p.setPen(DIM)
        p.drawText(QRectF(r.x() + 14, r.bottom() - 28, r.width() - 28, 20), Qt.AlignLeft | Qt.AlignVCenter,
                   "In view: glow at its spot (a column, no height). Out of view: arrow on that edge; "
                   "behind you: arrow at the bottom.")

    def where(self, angle):
        """Array angle -> (angle from the picture centre, 'in' | 'left' | 'right'), for the captions."""
        if angle is None:
            return None, "in"
        rel = wrap(self.oriented(angle) - self.offset)
        if abs(rel) <= self.hfov / 2:
            return rel, "in"
        return rel, "left" if rel > 0 else "right"

    def draw_captions(self, p, pic, f):
        if self.cap is None or not self.show_captions:
            return
        with self.cap.lock:
            lines = [ln for ln in self.cap.lines if time.time() - ln["wall"] < mcv.CAPTION_HOLD_S][-2:]
            partial = self.cap.partial
            speaking = self.cap.speaking_angle
        p.save()
        p.setClipRect(pic)
        if speaking is not None:
            rel, side = self.where(speaking)
            if side == "in":
                mcv.CaptionView.draw_speaking_tag(self, p, pic.center().x() - f * math.tan(math.radians(rel)),
                                                  pic.top() + 12)
        rows = [(ln["text"], ln["angle"], ln["kind"], False) for ln in lines]
        if partial:
            rows.append((partial[0], partial[1], "speech", True))
        rows = rows[-2:] if self.compact else rows[-3:]
        if rows:
            mcv.CaptionView.draw_subtitles(self, p, pic, rows)
        p.restore()

    def draw_side_arrow(self, p, pic, rel, sigma, strength, age, fade, color, det):
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
        where = "behind you" if abs(rel) > 90 else "in front"
        if det[3] is not None and det[3] < FB_SURE:
            where += "?"
        head = "probably just off screen" if just else "off screen"
        lines = [f"{'LOUD · ' if strength > 30 else ''}{head} · ~{abs(rel):.0f}° {side}",
                 f"{where} · turn {side} to see it · ±{sigma:.0f}°",
                 f"{age:.1f} s ago · {self.fb_text(det)}"]
        bx = tip + d * 80 if left else tip + d * 80 - 320
        label_box(p, bx, y - 40, lines, color, pic)

    def draw_behind_arrow(self, p, pic, rel, sigma, strength, age, fade, color, det):
        x = pic.center().x() + max(-1.0, min(1.0, -wrap(180 - rel) / 45.0)) * pic.width() * 0.25
        tip = pic.bottom() - 40
        arrow = QPolygonF([QPointF(x, tip), QPointF(x - 30, tip - 34), QPointF(x - 13, tip - 34),
                           QPointF(x - 13, tip - 70), QPointF(x + 13, tip - 70), QPointF(x + 13, tip - 34),
                           QPointF(x + 30, tip - 34)])
        p.setPen(QPen(with_alpha(QColor("#1a1206"), fade), 2))
        p.setBrush(with_alpha(color, 0.35 + 0.65 * fade))
        p.drawPolygon(arrow)
        off = wrap(180 - abs(rel))
        side = "" if abs(off) < 5 else f" · {abs(off):.0f}° to your {'left' if rel > 0 else 'right'}"
        sure = "" if det[3] is None or det[3] >= FB_SURE else " (unsure)"
        lines = [f"{'LOUD · ' if strength > 30 else ''}BEHIND YOU{sure}{side}",
                 f"turn around to see it · ±{sigma:.0f}°",
                 f"{age:.1f} s ago · {self.fb_text(det)}"]
        label_box(p, x + 40, tip - 90, lines, color, pic)

    def _xy(self, cx, cy, R, ang, frac=1.0):
        th = math.radians(ang)
        return QPointF(cx - R * frac * math.sin(th), cy - R * frac * math.cos(th))

    def draw_gauge(self, p, r):
        panel(p, r, "Direction (top view, full circle)")
        big, small, line1, line2 = (15, 10, 26, 46) if self.compact else (20, 12, 30, 58)
        top = r.y() + (62 if self.compact else 80)
        cx, cy = r.center().x(), top + (r.bottom() - top) / 2
        R = min(r.width() * 0.36, (r.bottom() - top) / 2 - 22)
        box = QRectF(cx - R, cy - R, 2 * R, 2 * R)
        p.setPen(QPen(EDGE, 1.5))
        p.setBrush(QColor("#0f141a"))
        p.drawEllipse(box)
        # camera field of view wedge (Qt angles: 0 = 3 o'clock, counter-clockwise; ours: 0 = up, + = left)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(88, 166, 255, 45))
        p.drawPie(box, int((90 + self.offset - self.hfov / 2) * 16), int(self.hfov * 16))
        p.setFont(font(11))
        for a in range(-150, 181, 30):
            p.setPen(QPen(DIM, 1.5))
            p.drawLine(self._xy(cx, cy, R, a), self._xy(cx, cy, R, a, 0.93))
        p.setFont(font(12, True))
        p.setPen(INK)
        for txt, a in (("FRONT", 0), ("BACK", 180), ("LEFT", 90), ("RIGHT", -90)):
            pt = self._xy(cx, cy, R + 16, a)
            p.drawText(QRectF(pt.x() - 30, pt.y() - 9, 60, 18), Qt.AlignCenter, txt)
        # mics, at a small scale in the middle
        scale = 0.22 * R / max(1.0, max(math.hypot(x, y) for _, x, y in self.m.geom.values()))
        for k, (ch, (name, x, y)) in enumerate(sorted(self.m.geom.items())):
            mx, my = x, y
            if self.swap:
                mx = -mx
            if self.swap_fb:
                my = -my
            pt = QPointF(cx - mx * scale, cy - my * scale)
            alive = self.m.source(ch) is not None
            p.setPen(QPen(QColor("#0d1117"), 1))
            p.setBrush(GEOM_COLORS[k % len(GEOM_COLORS)] if alive else QColor("#484f58"))
            p.drawEllipse(pt, 4.5, 4.5)
        now = self.m.now
        for det in self.m.detections:
            age = now - det[0]
            if age > 6:
                continue
            rel = self.oriented(det[1])
            p.setPen(Qt.NoPen)
            p.setBrush(with_alpha(LOUD if det[4] > 30 else GLOW, 0.6 * (1 - age / 6)))
            p.drawEllipse(self._xy(cx, cy, R, rel, 0.86), 5, 5)
        det = self.m.last_det
        if det and now - det[0] < 4:
            fade = max(0.25, 1 - (now - det[0]) / 4)
            a = self.oriented(det[1])
            curve = det[5]
            if curve is not None:
                c = (curve - np.median(curve)) / max(1e-9, float(curve.max() - np.median(curve)))
                poly = QPolygonF()
                for i in range(0, 360, 3):
                    ang = self.oriented(float(i - 180))
                    poly.append(self._xy(cx, cy, R, ang, 0.30 + 0.55 * max(0.0, float(c[i]))))
                p.setPen(QPen(with_alpha(QColor("#58a6ff"), 0.5 * fade), 1.5))
                p.setBrush(with_alpha(QColor("#58a6ff"), 0.10 * fade))
                p.drawPolygon(poly)
            p.setPen(Qt.NoPen)
            p.setBrush(with_alpha(GLOW, 0.25 * fade))
            p.drawPie(box.adjusted(8, 8, -8, -8), int((90 + a - det[2]) * 16), int(2 * det[2] * 16))
            p.setPen(QPen(with_alpha(QColor("#ff7b72"), fade), 5, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(cx, cy), self._xy(cx, cy, R - 8, a))
            rel = wrap(a - self.offset)
            p.setFont(font(big, True))
            p.setPen(INK)
            where = "in view" if abs(rel) <= self.hfov / 2 else ("behind" if abs(a) > 90 else "off screen")
            p.drawText(QRectF(r.x(), r.y() + line1, r.width(), line2 - line1), Qt.AlignHCenter,
                       f"{self.side_text(a)} · {where}")
            p.setFont(font(small, mono=True))
            p.setPen(DIM)
            p.drawText(QRectF(r.x(), r.y() + line2, r.width(), 18), Qt.AlignHCenter,
                       f"{self.fb_text(det)} · {len(self.m.active())} mics")
        else:
            p.setFont(font(15))
            p.setPen(DIM)
            p.drawText(QRectF(r.x(), r.y() + 36, r.width(), 24), Qt.AlignHCenter, "clap, snap or talk")
        p.setPen(Qt.NoPen)
        p.setBrush(INK)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

    def draw_levels(self, p, r):
        panel(p, r, "Mic levels (dBFS)")
        geom = sorted(self.m.geom.items())
        inner = r.adjusted(52, 30, -48, -6) if self.compact else r.adjusted(150, 34, -64, -8)
        row_h = inner.height() / max(1, len(geom))

        def xf(db):
            return inner.left() + float(np.clip((db + 80) / 80, 0, 1)) * inner.width()

        for k, (ch, (name, _, _)) in enumerate(geom):
            y = inner.top() + k * row_h + row_h / 2
            color = GEOM_COLORS[k % len(GEOM_COLORS)]
            p.setFont(font(11 if self.compact else 12, True))
            p.setPen(color)
            p.drawText(QPointF(r.x() + 14, y + 5), f"ch{ch}" if self.compact else f"ch{ch} {name}")
            bh = min(14.0, row_h - 5)
            bar = QRectF(inner.left(), y - bh / 2, inner.width(), bh)
            p.setPen(Qt.NoPen)
            p.setBrush(QColor("#21262d"))
            p.drawRoundedRect(bar, 4, 4)
            src = self.m.source(ch)
            p.setFont(font(11, mono=True))
            if src is None:
                p.setPen(ALERT)
                p.drawText(QPointF(bar.left() + 8, y + 4),
                           "no signal" if self.compact else "no signal: check 3V, GND, CLK, DAT")
                continue
            p.setBrush(color)
            p.drawRoundedRect(QRectF(bar.left(), bar.top(), xf(self.m.level[src]) - bar.left(), bh), 4, 4)
            p.setPen(QPen(DIM, 1, Qt.DashLine))
            xn = xf(self.m.floor[src])
            p.drawLine(QPointF(xn, bar.top() - 3), QPointF(xn, bar.bottom() + 3))
            p.setPen(INK)
            note = f"{self.m.level[src]:4.0f}" + ("" if src == ch else f" (ch{src})")
            p.drawText(QPointF(inner.right() + 8, y + 4), note)


def load_captioner(model: Model360, geometry: dict, model_name: str):
    """-> (Captioner, the 4 channels it listens to), or (None, why captions are off)."""
    global mcv
    if importlib.util.find_spec("faster_whisper") is None:
        return None, "captions off: faster-whisper isn't installed"
    import mic_caption_view
    mcv = mic_caption_view

    def speech_angle(_raw):
        det = model.last_det
        return det[1] if det and model.now - det[0] < 3.0 else None

    mcv.angle_of = speech_angle  # Captioner.run looks angle_of up in its own module
    chans = [ch for ch, _ in sorted(geometry.items(), key=lambda kv: -kv[1][2])][:mcv.CH]
    chans += chans[:1] * (mcv.CH - len(chans))
    return mcv.Captioner(model_name), chans


# ------------------------------------------------------------------ window

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: find the board automatically)")
    ap.add_argument("--geometry", help="mic positions JSON (see the top of this file)")
    ap.add_argument("--demo", nargs="?", const="", metavar="JPEG", help="run without a board")
    ap.add_argument("--compact", action=argparse.BooleanOptionalAction, default=None,
                    help="small-screen full-screen layout (default: on for screens under 1100 px wide)")
    ap.add_argument("--model", default="base.en", help="Whisper model for the subtitles (base.en or small.en)")
    ap.add_argument("--captions", action=argparse.BooleanOptionalAction, default=True,
                    help="live subtitles (default on)")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()

    app = QApplication(sys.argv[:1])
    screen = app.primaryScreen().availableGeometry() if app.primaryScreen() else None
    compact = args.compact if args.compact is not None else bool(screen and screen.width() < 1100)
    geometry = load_geometry(args.geometry)
    model = Model360(geometry)
    if args.demo is not None:
        reader = Demo360Reader(geometry, Path(args.demo).read_bytes() if args.demo else None)
    else:
        reader = Reader8(args.port)
    reader.start()
    captioner, cap_chans, cap_note = None, [], ""
    if args.captions:
        captioner, cap_chans = load_captioner(model, geometry, args.model)
        if captioner is None:
            cap_note, cap_chans = cap_chans, []
            print(cap_note, file=sys.stderr)
        else:
            captioner.start()

    settings = QSettings("CapstoneMicArray", "mic_camera_view360")
    view = View360(model, reader, compact, captioner)
    view.hfov = float(settings.value("hfov", 60.0))
    view.offset = float(settings.value("offset", 0.0))
    view.swap = settings.value("swap", "false") in (True, "true")
    view.swap_fb = settings.value("swap_fb", "false") in (True, "true")
    view.rotate = int(settings.value("rotate", 0))
    view.mirror = settings.value("mirror", "false") in (True, "true")

    win = QWidget()
    win.setWindowTitle("Camera + sound direction · 360°")
    win.setStyleSheet(f"QWidget {{ background: #0d1117; color: #e6edf3; font-size: {13 if compact else 14}px; }}"
                      "QPushButton, QComboBox { background: #21262d; border: 1px solid #30363d;"
                      f" border-radius: 6px; padding: {'6px 8px' if compact else '5px 12px'}; }}"
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
    swap_fb = QCheckBox("Swap front/back")
    swap_fb.setChecked(view.swap_fb)
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
    captions = QPushButton("CC") if compact else QCheckBox("Captions")
    captions.setCheckable(True)
    if compact:
        captions.setStyleSheet("QPushButton:checked { background: #1f6feb; border-color: #58a6ff; }")
    captions.setChecked(captioner is not None)
    captions.setEnabled(captioner is not None)
    captions.setToolTip(cap_note or f"Live subtitles from ch{', ch'.join(map(str, cap_chans))} (Whisper {args.model})")
    focus = QCheckBox("Voice focus")
    focus.setChecked(True)
    focus.setEnabled(captioner is not None)
    focus.setToolTip("Clean the audio before captioning (four mics aimed at the talker, background removed)")

    hint = QLabel("Clap straight ahead → Set centre. Clap at the right edge of the picture → Set right edge.")
    hint.setStyleSheet("color: #8b949e; font-size: 12px;")
    lay = QVBoxLayout(win)
    if compact:
        rec.setText("● Rec")
        snap.setText("Snap")
        set_centre.setText("Set centre")
        set_edge.setText("Right edge")
        swap.setText("Swap L/R")
        swap_fb.setText("Swap F/B")
        more = QPushButton("More")
        more.setCheckable(True)
        quit_btn = QPushButton("✕")
        quit_btn.clicked.connect(win.close)
        row1 = QHBoxLayout()
        for wdg in (rec, snap, captions, set_centre, set_edge, swap, swap_fb):
            row1.addWidget(wdg)
        row1.addStretch(1)
        row1.addWidget(more)
        row1.addWidget(quit_btn)
        extra = QWidget()
        row2 = QHBoxLayout(extra)
        row2.setContentsMargins(0, 0, 0, 0)
        for wdg in (res, rotate, mirror, fov, focus, QLabel("Trigger"), thresh, thresh_lbl):
            row2.addWidget(wdg)
        row2.addStretch(1)
        extra.setVisible(False)
        more.toggled.connect(extra.setVisible)
        hint.setText("Clap ahead → Set centre · clap at right edge → Right edge")
        row3 = QHBoxLayout()
        for lbl in (hint, status):
            lbl.setStyleSheet("color: #8b949e; font-size: 11px;")
            lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        row3.addWidget(hint, 1)
        row3.addWidget(status, 1)
        lay.setContentsMargins(4, 4, 4, 2)
        lay.setSpacing(3)
        lay.addLayout(row1)
        lay.addWidget(extra)
        lay.addLayout(row3)
    else:
        row1 = QHBoxLayout()
        for wdg in (rec, snap, captions, focus, QLabel("Camera"), res, rotate, mirror,
                    QLabel("Trigger"), thresh, thresh_lbl):
            row1.addWidget(wdg)
        row1.addStretch(1)
        row1.addWidget(status)
        row2 = QHBoxLayout()
        for wdg in (QLabel("Calibrate:"), fov, set_centre, set_edge, swap, swap_fb):
            row2.addWidget(wdg)
        row2.addWidget(hint)
        row2.addStretch(1)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.addLayout(row1)
        lay.addLayout(row2)
    lay.addWidget(view, 1)
    QShortcut(QKeySequence(Qt.Key_Escape), win, win.close)

    def save():
        for k in ("hfov", "offset", "swap", "swap_fb", "rotate", "mirror"):
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
            hint.setText(f"Centre set: picture centre = {a:+.0f}° on the array.")
            save()

    def on_edge():
        a = last_angle()
        if a is None:
            return
        half = wrap(view.offset - a)
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

    def on_swap_fb(v):
        view.swap_fb = v
        save()

    def on_thresh(v):
        model.threshold_db = float(v)
        thresh_lbl.setText(f"{v} dB")

    def on_rec(checked):
        if checked != bool(model.recorder):
            model.toggle_record()
        if captioner is not None:
            with captioner.lock:
                if model.recorder:
                    path = model.record_path.with_name(model.record_path.stem + "_transcript.txt")
                    captioner.transcript_file = open(path, "w")
                elif captioner.transcript_file:
                    captioner.transcript_file.close()
                    captioner.transcript_file = None
        if compact:
            rec.setText("■ Stop" if model.recorder else "● Rec")
        else:
            rec.setText("■ Stop recording" if model.recorder else "● Record audio")

    def on_snap():
        cv.TAKES.mkdir(parents=True, exist_ok=True)
        path = cv.TAKES / time.strftime("cam360_%Y%m%d_%H%M%S.png")
        view.grab().save(str(path))
        hint.setText(f"Saved {path.name} in mic_takes.")

    set_centre.clicked.connect(on_centre)
    set_edge.clicked.connect(on_edge)
    rotate.currentIndexChanged.connect(on_rotate)
    mirror.toggled.connect(on_mirror)
    fov.valueChanged.connect(on_fov)
    swap.toggled.connect(on_swap)
    swap_fb.toggled.connect(on_swap_fb)
    res.currentIndexChanged.connect(lambda i: reader.send(b"123"[i:i + 1]))
    thresh.valueChanged.connect(on_thresh)
    on_thresh(thresh.value())
    rec.toggled.connect(on_rec)
    snap.clicked.connect(on_snap)

    def on_captions(v):
        view.show_captions = v

    def on_focus(v):
        captioner.focus.enabled = v

    captions.toggled.connect(on_captions)
    focus.toggled.connect(on_focus)

    def tick():
        chunks = reader.take()
        model.push(chunks)
        if captioner is not None:
            captioner.feed([c[:, cap_chans] for c in chunks])
        view.update_frame()
        recs = f"  ● REC {model.record_path.name}" if model.recorder else ""
        caps = f" · {captioner.status}" if compact and captioner is not None else ""
        status.setText(f"{reader.port or '-'} · {reader.status}{caps}{recs}")
        view.update()

    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(33)
    if compact and not args.screenshot:
        win.showFullScreen()
    else:
        win.resize(*((800, 480) if compact else (1500, 900)))
        win.show()
    if args.screenshot:
        QTimer.singleShot(int(os.environ.get("SHOT_MS", "6000")),
                          lambda: (win.grab().save(args.screenshot), app.quit()))
    app.exec()
    reader.stop = True
    if captioner is not None:
        captioner.stop = True
        if captioner.transcript_file:
            captioner.transcript_file.close()
    if model.recorder:
        model.recorder.close()


if __name__ == "__main__":
    main()
