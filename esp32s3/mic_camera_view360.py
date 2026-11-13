#!/usr/bin/env python3
"""360° camera + sound direction view for the vest mic array (cam_mic_vest8 or cam_mic_vest6 firmware).

    ~/.mic-caption-env/bin/python mic_camera_view360.py
    ~/.mic-caption-env/bin/python mic_camera_view360.py --layout vest6   # chest plate only
    ~/.mic-caption-env/bin/python mic_camera_view360.py --port /dev/cu.usbmodem1101
    ~/.mic-caption-env/bin/python mic_camera_view360.py --demo [JPEG]     # no board

Default wiring (layout vest8, firmware cam_mic_vest8): XIAO ESP32-S3 Sense, PDM clock on D0, record
button on D2, six mics on the chest plate and two on the back plate, one data line per pair. On every
pair the mic with SEL on 3V3 is on the wearer's RIGHT, the one with SEL on GND on the LEFT:
    D1  ch0 inner R  ch1 inner L     row, 22.9 mm either side of the middle
    D3  ch2 outer R  ch3 outer L     row, 68.6 mm either side
    D4  ch4 top R    ch5 top L       22.9 mm either side, 60 mm above the row
    D5  ch6 back R   ch7 back L      back plate, 68.6 mm either side, level with the row, BACK_DEPTH_MM behind
The board sends 'MK' packets with mask 0xFF (0x3F for cam_mic_vest6) and a "firmware: ..." status
line every 5 s; the window asks for it on connect, shows it, and warns if it doesn't match the layout.
The raised pair lets it search elevation (-30° to 60°) as well as azimuth, and the back pair tells front
from behind. With vest6 all mics are on one line front to back, so a sound behind shows at its mirror
in front.

Options:
    --port PORT         serial port, if auto-detect picks the wrong one (macOS: ls /dev/cu.usbmodem*)
    --layout NAME       mic layout preset: vest8 (default, cam_mic_vest8), vest6 (cam_mic_vest6), six
                        (4 front + 2 back, the cam_mic_record8 breadboard), rear5 (4 front + rear centre), front4
    --geometry FILE     mic positions as JSON instead of a preset,
                        {"<channel>": ["name", x_mm, y_mm] or ["name", x_mm, y_mm, z_mm], ...}
                        x = toward the wearer's left, y = toward the front, z = up, from the array centre.
    --compact / --no-compact
                        full-screen touch layout for small screens such as the Raspberry Pi 7"
                        display (default: on when the screen is under 1100 px wide; Esc or ✕ quits)
    --model NAME        Whisper model for the live subtitles (default base.en)
    --no-captions       no subtitles (they need faster-whisper; see mic_caption_view.py)

Subtitles reuse mic_caption_view.py: four mics (the front row) are aimed at the talker (voice focus),
Whisper turns the speech into text on this computer, and each line is placed in the picture by
the 360° direction of the talker, with ◀ / ▶ when they're out of frame.

Same picture as mic_camera_view.py, but every working mic pair is used at once (SRP-PHAT), and
with back mics (layouts vest8, six) it tells front from behind. A sound inside the picture glows where it
is; outside the picture an arrow on the left or right edge says how far round it is and whether it's
in front or behind; a sound behind you gets an arrow at the bottom. With no back mic (vest6, or a
back mic not working) it is front/back-ambiguous like the 4-mic app. Mic levels and the trigger
look above 150 Hz only, so body thumps and room rumble don't set it off.

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
import termios  # noqa: E402
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

CH = 8                     # columns kept per sample: the widest stream (cam_mic_vest8); vest6 fills ch0-5
SEG = 1024                 # samples per direction estimate (64 ms)
INTERP = 8                 # GCC sub-sample resolution
BAND = (300.0, 6000.0)
HP_HZ = 150.0              # high-pass for the levels and the trigger
ELEV = np.arange(-30.0, 61.0, 10.0)  # elevations searched when the mics aren't all at one height
STUCK_FRACTION = 0.5       # a live mic almost never repeats a sample; a dead data line mostly does
BEHIND_DEG = 135.0         # beyond this from the camera centre, point the arrow down ("behind you")
FB_SURE = 0.25             # front/back margin (share of the peak) needed to call it
# front/back with a body between the chest and back plates (Model360.plate_score)
PLATE_GAP = 0.08           # m front to back beyond which the mics are on two plates with a body between
PLATE_MAX_LAG = 24         # samples (1.5 ms): the longest way round a body from one plate to the other
PLATE_TIME_BAND = (300.0, 3000.0)   # Hz: low enough to bend round the body to the far plate
PLATE_LEVEL_BAND = (1000.0, 4000.0)  # Hz: high enough for the body to shadow the far plate
PLATE_SHADOW_DB = 6.0      # far-plate level drop that counts as a full vote
PLATE_TILT = 0.6           # how far the plate score raises the front or back half of the azimuth curve
VEST_FIRMWARE = {"vest6": "cam_mic_vest6", "vest8": "cam_mic_vest8"}  # layout: the firmware that streams it
BACK_DEPTH_MM = 150.0      # chest plate face to back plate face, as worn

# channel: (name, x mm toward the wearer's left, y mm toward the front, z mm up)
FRONT4 = {
    0: ("front outer L", 68.6, 22.0, 0.0),
    1: ("front outer R", -68.6, 22.0, 0.0),
    2: ("front inner L", 22.9, 22.0, 0.0),
    3: ("front inner R", -22.9, 22.0, 0.0),
}
VEST_CHEST = {
    0: ("inner R", -22.9, 0.0, 0.0),
    1: ("inner L", 22.9, 0.0, 0.0),
    2: ("outer R", -68.6, 0.0, 0.0),
    3: ("outer L", 68.6, 0.0, 0.0),
    4: ("top R", -22.9, 0.0, 60.0),
    5: ("top L", 22.9, 0.0, 60.0),
}
LAYOUTS = {
    "vest8": ("8 mics: vest chest 6 + back pair (cam_mic_vest8)", {
        **VEST_CHEST,
        6: ("back R", -68.6, -BACK_DEPTH_MM, 0.0),
        7: ("back L", 68.6, -BACK_DEPTH_MM, 0.0),
    }),
    "vest6": ("6 mics: vest chest, 4 in a row + 2 above (cam_mic_vest6)", dict(VEST_CHEST)),
    "six": ("6 mics: 4 front + 2 back (cam_mic_record8)", {
        **FRONT4, 5: ("back R", -68.6, -22.0, 0.0), 7: ("back L", 68.6, -22.0, 0.0)}),
    "rear5": ("5 mics: 4 front + rear centre (cam_mic_record8)", {**FRONT4, 4: ("rear centre", 0.0, -22.0, 0.0)}),
    "front4": ("4 front mics, no front/back (cam_mic_record8)", dict(FRONT4)),
}
DEFAULT_LAYOUT = "vest8"
DEFAULT_GEOMETRY = LAYOUTS[DEFAULT_LAYOUT][1]
GEOM_COLORS = [QColor("#58a6ff"), QColor("#f0883e"), QColor("#3fb950"), QColor("#d2a8ff"),
               QColor("#ff7b72"), QColor("#e3b341"), QColor("#79c0ff"), QColor("#56d4dd")]


def wrap(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def load_geometry(path: str | None, layout: str = DEFAULT_LAYOUT) -> dict:
    if not path:
        return dict(LAYOUTS[layout][1])
    raw = json.loads(Path(path).read_text())
    return {int(k): (str(v[0]), float(v[1]), float(v[2]), float(v[3]) if len(v) > 3 else 0.0)
            for k, v in raw.items()}


def _hp_coeffs(fc: float = HP_HZ) -> tuple:
    """2nd-order Butterworth high-pass (b0, b1, b2, a1, a2), as in web_app/app.js."""
    w = 2 * math.pi * fc / FS
    cw, al = math.cos(w), math.sin(w) / math.sqrt(2)
    a0 = 1 + al
    return ((1 + cw) / 2 / a0, -(1 + cw) / a0, (1 + cw) / 2 / a0, -2 * cw / a0, (1 - al) / a0)


class HighPass:
    """The biquad above, run on (n, channels) blocks with its state carried across calls.

    No scipy here, so the recursive part is done as a convolution with its impulse response plus the
    decaying response to the carried-over outputs (exact, and fast in numpy)."""

    BLOCK = 4096

    def __init__(self, nch: int, fc: float = HP_HZ):
        self.b0, self.b1, self.b2, a1, a2 = _hp_coeffs(fc)
        self.zx = np.zeros((2, nch))  # x[-1], x[-2]
        self.zy = np.zeros((2, nch))  # y[-1], y[-2]
        n = self.BLOCK
        self.h, self.g1, self.g2 = np.zeros(n), np.zeros(n), np.zeros(n)
        for seq, y1, y2, x0 in ((self.h, 0.0, 0.0, 1.0), (self.g1, 1.0, 0.0, 0.0), (self.g2, 0.0, 1.0, 0.0)):
            for i in range(n):
                y = (x0 if i == 0 else 0.0) - a1 * y1 - a2 * y2
                seq[i] = y
                y1, y2 = y, y1
        self.nfft = 1 << int(math.ceil(math.log2(2 * n)))
        self.H = np.fft.rfft(self.h, self.nfft)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, np.float64)
        return np.concatenate([self._block(x[i:i + self.BLOCK]) for i in range(0, len(x), self.BLOCK)]) \
            if len(x) else np.zeros((0, self.zx.shape[1]))

    def _block(self, x: np.ndarray) -> np.ndarray:
        n = len(x)
        xp = np.vstack([self.zx[::-1], x])
        w = self.b0 * xp[2:] + self.b1 * xp[1:-1] + self.b2 * xp[:-2]
        y = np.fft.irfft(np.fft.rfft(w, self.nfft, axis=0) * self.H[:, None], self.nfft, axis=0)[:n]
        y += self.g1[:n, None] * self.zy[0] + self.g2[:n, None] * self.zy[1]
        self.zx = np.vstack([x[-1], x[-2] if n > 1 else self.zx[0]])
        self.zy = np.vstack([y[-1], y[-2] if n > 1 else self.zy[0]])
        return y


# ------------------------------------------------------------------ serial reader (MK / M8 + JPEG)

class _Port:
    """A tty opened without touching DTR or RTS. read/write match the bits of pyserial the reader uses."""

    def __init__(self, fd: int):
        self.fd = fd

    def read(self, n: int) -> bytes:
        try:
            return os.read(self.fd, n)
        except BlockingIOError:
            time.sleep(0.02)
            return b""

    def write(self, data: bytes) -> None:
        try:
            os.write(self.fd, data)
        except BlockingIOError:
            pass

    def close(self) -> None:
        os.close(self.fd)

    @property
    def in_waiting(self) -> int:
        return 65536


class Reader8(cv.Reader):
    """cam_mic_vest6 ('MK' packets, any channel mask) or cam_mic_record8 ('M8', all eight).

    Every mic chunk comes out CH (8) columns wide, the channel number as the column; channels the
    packet's mask leaves out are zero (so they read as dead mics)."""

    def __init__(self, port):
        super().__init__(port)
        self.wrong_firmware = False
        self.kind = None          # b"MK" or b"M8": the mic packets the board sends
        self.mask = None          # channels in the latest mic packet (bit n = ch n)
        self.board_fw = ""        # the board's "firmware: ..." line, without the prefix
        self.connected_at = None
        self._last_seq = None
        self.esc = False          # v4 firmware: after "escape: on", FE 00 = 0xFE and FE 01 = 0xFF
        self.esc_half = False

    def _open(self, path: str):
        # Open the tty and leave DTR and RTS alone. pyserial asserts both on open,
        # and on this board that holds the chip in reset, so the web app's stream stops.
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[2] &= ~termios.HUPCL
        attrs[4] = attrs[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        return _Port(fd)

    def run(self):
        while not self.stop:
            path = next(iter(self._candidates()), None)
            if not path:
                self.status = "plug in the board"
                time.sleep(1.0)
                continue
            try:
                s = self._open(path)
            except (OSError, serial.SerialException) as e:
                self.status = f"can't open {path}: {e}"
                time.sleep(1.0)
                continue
            self.port = path
            self.status = "streaming"
            self.kind = self.mask = None
            self.board_fw = ""
            self.wrong_firmware = False
            self._last_seq = None
            self.connected_at = time.time()
            self.commands.append(b"?")
            self.commands.append(b"x")  # same as the web app: macOS drops 0xFF unless the board escapes them
            self._next_x = time.time() + 2.0
            self.esc = False
            self.esc_half = False
            self._stream(s)
            self.status = "connection lost, reconnecting…"
            self.port = None
            self.connected_at = None
            time.sleep(0.5)

    def _unescape(self, data: bytes) -> bytes:
        """FE 00 -> 0xFE, FE 01 -> 0xFF. A trailing FE waits for its partner.

        A reconnect can start in the middle of a plain packet while ``esc`` still reflects the prior
        connection. Treat an invalid FE pair as a literal FE and continue scanning instead of letting a
        byte value above 255 kill the reader thread.
        """
        if not self.esc_half and 0xFE not in data:
            return data
        out = bytearray()
        i = 0
        if self.esc_half and data:
            if data[0] <= 1:
                out.append(0xFE + data[0])
                i = 1
            else:
                out.append(0xFE)
            self.esc_half = False
        while i < len(data):
            if data[i] != 0xFE:
                out.append(data[i])
                i += 1
            elif i + 1 < len(data):
                if data[i + 1] <= 1:
                    out.append(0xFE + data[i + 1])
                    i += 2
                else:
                    out.append(0xFE)
                    i += 1
            else:
                self.esc_half = True
                break
        return bytes(out)

    def _escape(self, data: bytes) -> bytes:
        """Inverse of _unescape, for the bytes that followed an "escape: off" in the same read."""
        out = bytearray()
        for v in data:
            if v >= 0xFE:
                out.append(0xFE)
                out.append(v - 0xFE)
            else:
                out.append(v)
        if self.esc_half:
            out.append(0xFE)
            self.esc_half = False
        return bytes(out)

    def _stream(self, s):
        buf = bytearray()
        while not self.stop:
            try:
                if time.time() >= self._next_x:
                    self.commands.append(b"x")
                    self._next_x = time.time() + 2.0
                while self.commands:
                    s.write(self.commands.popleft())
                data = s.read(max(1, s.in_waiting or 1))
            except (OSError, serial.SerialException):
                break
            if not data:
                continue
            if self.esc:
                data = self._unescape(data)
            buf += data
            self.parse(buf)
        try:
            s.close()
        except (OSError, serial.SerialException):
            pass

    def _mic(self, seq: int, chunk: np.ndarray, kind: bytes, mask: int):
        if self._last_seq is not None and seq != (self._last_seq + 1) & 0xFFFFFFFF:
            self.gaps += 1
        self._last_seq = seq
        self.packets += 1
        self.kind, self.mask = kind, mask
        self.status = "streaming"
        with self.lock:
            self.chunks.append(chunk)

    def parse(self, buf: bytearray):
        """Takes every whole packet off the front of buf; leaves a partial one for the next read."""
        if not self.esc:
            # v4 may already be escaping (the browser asked, or our own 'x') before this
            # packet is framed. "escape: on" itself is sent in the clear.
            mark = buf.find(b"escape: on")
            if mark >= 0:
                end = mark + len(b"escape: on")
                rest = self._unescape(bytes(buf[end:]))
                del buf[end:]
                buf.extend(rest)
                self.esc = True
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
            if kind == b"MK":
                if len(buf) < 12:
                    break
                seq, frames, mask = struct.unpack_from("<IHB", buf, 4)
                chans = [c for c in range(CH) if mask >> c & 1]
                if not chans or not 0 < frames <= 4096:
                    del buf[:2]
                    continue
                end = 12 + frames * len(chans) * 2
                if len(buf) < end:
                    break
                chunk = np.zeros((frames, CH), np.int16)
                chunk[:, chans] = np.frombuffer(bytes(buf[12:end]), dtype="<i2").reshape(frames, len(chans))
                del buf[:end]
                self._mic(seq, chunk, kind, mask)
            elif kind == b"M8":
                if len(buf) < 10:
                    break
                seq, frames = struct.unpack_from("<IH", buf, 4)
                if not 0 < frames <= 4096:
                    del buf[:2]
                    continue
                end = 10 + frames * CH * 2
                if len(buf) < end:
                    break
                chunk = np.frombuffer(bytes(buf[10:end]), dtype="<i2").reshape(-1, CH).copy()
                del buf[:end]
                self._mic(seq, chunk, kind, 0xFF)
            elif kind == b"M4":
                self.wrong_firmware = True
                del buf[:2]
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
                if msg in ("escape: on", "escape: off"):
                    on = msg == "escape: on"
                    if on != self.esc:
                        rest = bytes(buf)
                        del buf[:]
                        if self.esc:
                            rest = self._escape(rest)
                        self.esc = on
                        if on:
                            rest = self._unescape(rest)
                        buf.extend(rest)
                    continue
                with self.lock:
                    if msg.startswith("stats: "):
                        self.board_stats = msg[7:]
                    elif msg.startswith("firmware: "):
                        self.board_fw = msg[10:]
                        # v4 still calls itself cam_mic_vest6. Any version of that name is this board.
                    else:
                        self.board_msgs.append((time.time(), msg))
            else:
                del buf[:2]


def firmware_warning(reader, layout: str) -> str:
    """What's wrong with the board for this layout, in a sentence, or ''."""
    if getattr(reader, "port", None) == "demo":
        return ""
    want = VEST_FIRMWARE.get(layout)
    if reader.wrong_firmware:
        return f"board is sending 4 mics (old firmware): flash {want or VEST_FIRMWARE[DEFAULT_LAYOUT]}"
    fw, kind = reader.board_fw, reader.kind
    board = next((name for name, f in VEST_FIRMWARE.items() if f in fw or f"layout {name}" in fw), None)
    if board and board != layout and layout in LAYOUTS:
        return f"board runs {VEST_FIRMWARE[board]} but the layout is {layout}: run with --layout {board}"
    if want:
        if fw and not board:
            return f"board says “{fw.split(' · ')[0]}”, not {want}: flash it, or pick that board's --layout"
        if kind == b"M8" and not board:
            return f"board is sending 8 mics (cam_mic_record8?): flash {want}, or run with --layout six"
        full = sum(1 << c for c in LAYOUTS[layout][1])
        if kind == b"MK" and reader.mask is not None and reader.mask & full != full:
            missing = [c for c in range(CH) if full >> c & 1 and not reader.mask >> c & 1]
            return (f"board is leaving out ch{', ch'.join(map(str, missing))} "
                    f"(mask 0x{reader.mask:02X}, expected 0x{full:02X})")
    return ""


def unit_vectors(az_deg, el_deg) -> np.ndarray:
    """Directions toward the sound, (..., 3) in (left, front, up): azimuth 0 = front, + = wearer's left."""
    th, ph = np.radians(az_deg), np.radians(el_deg)
    th, ph = np.broadcast_arrays(th, ph)
    return np.stack([np.sin(th) * np.cos(ph), np.cos(th) * np.cos(ph), np.sin(ph)], axis=-1)


def source_signal(geom: dict, sig: np.ndarray, az: float, el: float = 0.0) -> np.ndarray:
    """sig as each mic hears a far-away source at (az, el): (n, CH), peak 1, unused columns zero."""
    n = len(sig)
    F = np.fft.rfft(sig)
    f = np.fft.rfftfreq(n, 1 / FS)
    u = unit_vectors(az, el)
    out = np.zeros((n, CH))
    for ch, (_, x, y, z) in geom.items():
        delay = -(np.array([x, y, z]) / 1000 @ u) / C  # mics nearer the source hear it earlier
        out[:, ch] = np.fft.irfft(F * np.exp(-2j * math.pi * f * delay), n)
    return out / np.max(np.abs(out))


class Demo360Reader(cv.Reader):
    """No board: claps from directions all the way round, through the given geometry."""

    ANGLES = ((0, 0), (40, 20), (-25, 0), (120, 10), (-150, 0), (180, 30), (75, -10), (-100, 40),
              (15, 0), (160, 0))  # (azimuth, elevation)

    def __init__(self, geometry: dict, jpeg: bytes | None):
        super().__init__(None)
        self.geom = geometry
        self.jpeg = jpeg
        self.port = "demo"

    def _clap(self, ang: float, elev: float = 0.0, n: int = 2048) -> np.ndarray:
        rng = np.random.default_rng()
        burst = rng.normal(0, 1, n) * np.exp(-np.arange(n) / 250.0)
        burst[:200] = 0
        return source_signal(self.geom, burst, ang, elev) * 12000

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
                pending = self._clap(*self.ANGLES[k % len(self.ANGLES)])
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
    """Detections are (time, azimuth, sigma, front/back margin or None, strength dB, azimuth curve,
    elevation or None); azimuth + = wearer's left, elevation + = up."""

    def __init__(self, geometry: dict, rec_ch: int = CH, rec_prefix: str = "cam8"):
        super().__init__([0.1, 0.1])
        self.geom = geometry
        self.rec_ch = rec_ch      # channels written when recording (ch0 .. rec_ch-1)
        self.rec_prefix = rec_prefix
        self.x = np.zeros((FS * 2, CH), np.float32)
        self.xh = np.zeros((FS * 2, CH), np.float32)  # the same above HP_HZ, for levels and trigger
        self.hp = HighPass(CH)
        self.level = np.full(CH, -120.0)
        self.floor = np.full(CH, -70.0)
        self.dead = np.ones(CH, bool)
        self.echo = np.zeros(CH, bool)
        self.dc = np.zeros(CH)
        self.curve = None
        self._steer: dict = {}

    def push(self, chunks: list):
        if not chunks:
            return
        data = np.concatenate(chunks)
        if self.recorder:
            self.recorder.writeframes(np.ascontiguousarray(data[:, :self.rec_ch]).astype("<i2").tobytes())
        x = data.astype(np.float32)
        m = x.mean(axis=0)
        self.dc = m if self.total == 0 else 0.95 * self.dc + 0.05 * m
        x -= self.dc
        xh = self.hp(data).astype(np.float32)
        n = len(x)
        for buf, new in ((self.x, x), (self.xh, xh)):
            if n >= len(buf):
                buf[:] = new[-len(buf):]
            else:
                buf[:-n] = buf[n:]
                buf[-n:] = new
        self.total += n
        wall = time.time()
        self.rate_window.append((wall, n))
        while self.rate_window and wall - self.rate_window[0][0] > 2.0:
            self.rate_window.popleft()
        span = wall - self.rate_window[0][0]
        if span > 0.5:
            self.rate = sum(k for _, k in list(self.rate_window)[1:]) / span
        self._levels()
        self._direction()

    def _levels(self):
        if self.total >= FS // 2:
            recent = self.x[-FS // 2:]
            stuck = np.mean(np.diff(recent, axis=0) == 0, axis=0) > STUCK_FRACTION
            self.dead = (np.ptp(recent, axis=0) < cv.DEAD_PTP) | stuck
            for a in range(0, CH, 2):
                same = not self.dead[a] and np.array_equal(recent[:, a], recent[:, a + 1])
                self.echo[a] = self.echo[a + 1] = same
        blk = self.xh[-int(0.05 * FS):] / 32768.0
        lv = 10 * np.log10(np.var(blk, axis=0) + 1e-12)
        self.level = lv
        if self.total < int(0.5 * FS):
            self.floor = np.minimum(self.floor, lv) if self.total > int(0.1 * FS) else lv
            return
        self.floor = np.where(lv < self.floor, 0.7 * self.floor + 0.3 * lv, self.floor + 0.02)

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
        pos = np.array([self.geom[c][1:4] for c in chans]) / 1000.0
        spread = np.ptp(pos, axis=0)
        # directions: 360 azimuths x the elevations (just 0° when the mics are all at one height),
        # index k * len(els) + e
        els = ELEV if spread[2] > 0.01 else np.array([0.0])
        u = unit_vectors(np.arange(-180, 180)[:, None], els[None, :]).reshape(-1, 3)
        t = -(pos @ u.T) / C  # arrival time of each mic for each direction, (M, 360 * len(els))
        pairs = [(i, j) for i in range(len(chans)) for j in range(i + 1, len(chans))]
        plates = None
        if spread[1] > PLATE_GAP:
            # chest and back plates with a body between: sound bends round it, so pairs across the body don't
            # have the straight-line delays. They only vote on front/back (plate_score); the angle comes
            # from pairs on one plate.
            mid = (pos[:, 1].max() + pos[:, 1].min()) / 2
            front = [i for i in range(len(chans)) if pos[i, 1] > mid]
            back = [i for i in range(len(chans)) if pos[i, 1] <= mid]
            pairs = [(i, j) for i, j in pairs if (i in front) == (j in front)]
            plates = (front, back)
            spread = np.ptp(pos[front], axis=0) if len(front) > 1 else spread
            spread = np.maximum(spread, np.ptp(pos[back], axis=0)) if len(back) > 1 else spread
        max_lag = int(math.ceil(float(np.linalg.norm(spread)) / C * FS)) + 2
        lags = np.array([(t[j] - t[i]) * FS * INTERP for i, j in pairs])
        idx = np.round(lags).astype(int) + max_lag * INTERP
        has_back = bool(np.ptp(pos[:, 1]) > 0.01)
        self._steer[chans] = (pairs, idx, max_lag, has_back, els, plates)
        return self._steer[chans]

    @staticmethod
    def plate_score(F: np.ndarray, f: np.ndarray, n: int, plates: tuple) -> float:
        """Front (+1) to back (-1) from the chest and back plates: which plate hears the sound first
        (GCC-PHAT over the delays sound bending round a body can have) and which is louder above 1 kHz
        (the body shadows the far plate)."""
        front, back = plates
        if not front or not back:
            return 0.0
        band = (f >= PLATE_TIME_BAND[0]) & (f <= PLATE_TIME_BAND[1])
        cc = np.zeros(n)
        for i in front:
            for j in back:
                cross = F[:, j] * np.conj(F[:, i])
                cross /= np.abs(cross) + 1e-12
                cross[~band] = 0
                cc += np.fft.irfft(cross, n)
        lw = PLATE_MAX_LAG
        later, earlier = max(float(cc[1:lw + 1].max()), 0.0), max(float(cc[-lw:].max()), 0.0)  # back later = front
        timing = (later - earlier) / (later + earlier + 1e-9)
        hi = (f >= PLATE_LEVEL_BAND[0]) & (f <= PLATE_LEVEL_BAND[1])
        p = 10 * np.log10((np.abs(F[hi]) ** 2).sum(axis=0) + 1e-12)
        shadow = float(np.mean(p[back]) - np.mean(p[front]))
        return 0.5 * timing + 0.5 * math.tanh(-shadow / PLATE_SHADOW_DB)

    @staticmethod
    def tilt(curve: np.ndarray, score: float) -> np.ndarray:
        """Raises the front or back half of a front/back-mirrored azimuth curve by the plate score, so the
        peak and the front/back margin (estimate) follow it; at the sides it barely matters, so it fades."""
        med = float(np.median(curve))
        above = np.maximum(curve - med, 0)
        return curve + above * PLATE_TILT * score * np.cos(np.radians(np.arange(-180, 180)))

    def srp(self, seg: np.ndarray, chans: tuple):
        """-> (azimuth curve, 360 values from -180°, has back mics, elevation in degrees or None)."""
        pairs, idx, max_lag, has_back, els, plates = self._steering(chans)
        n = 1 << int(math.ceil(math.log2(2 * len(seg))))
        F = np.fft.rfft(seg, n, axis=0)
        f = np.fft.rfftfreq(n, 1 / FS)
        out_band = (f < BAND[0]) | (f > BAND[1])
        m = max_lag * INTERP
        score = self.plate_score(F, f, n, plates) if plates else None
        # with two plates, the angle comes from the plate facing the sound: the far one hears it only
        # after it bends round the body, which pulls its delays toward the side
        weight = {True: 1.0, False: 1.0}
        if plates:
            n_front = sum(i in plates[0] for i, _ in pairs)
            n_back = len(pairs) - n_front
            weight = {True: (1 + score) / 2 / n_front if n_front else 0.0,
                      False: (1 - score) / 2 / n_back if n_back else 0.0}
            if not n_back:
                weight[True] = 1.0
            elif not n_front:
                weight[False] = 1.0
        grid = np.zeros(idx.shape[1])
        for k, (i, j) in enumerate(pairs):
            cross = F[:, j] * np.conj(F[:, i])
            cross /= np.abs(cross) + 1e-12
            cross[out_band] = 0
            cc = np.fft.irfft(cross, n * INTERP)
            cc = np.concatenate((cc[-m:], cc[: m + 1]))
            grid += weight[not plates or i in plates[0]] * cc[np.clip(idx[k], 0, 2 * m)]
        if len(els) == 1:
            return (self.tilt(grid, score) if plates else grid), has_back, None
        # azimuth curve = the cut through the peak's elevation (each azimuth's best elevation instead would
        # smear the peak sideways: tilting up and turning toward the row's end change the row's delays alike)
        grid = grid.reshape(360, len(els))
        g = int(np.argmax(grid))
        e = g % len(els)
        elev = float(els[e])
        if 0 < e < len(els) - 1:  # parabola through the peak and its neighbours, between grid steps
            a, b, c = grid[g // len(els), e - 1:e + 2]
            den = a - 2 * b + c
            if den < 0:
                elev += 0.5 * (a - c) / den * float(els[e + 1] - els[e])
        curve = grid[:, e].copy()
        return (self.tilt(curve, score) if plates else curve), has_back, elev

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
        angle, sigma, fb, curve, elev = self.estimate(self.x[-SEG:, srcs], chans)
        self.curve = curve
        det = (self.now, angle, sigma, fb, strength, curve, elev)
        self.detections.append(det)
        self.last_det = det
        while self.detections and self.now - self.detections[0][0] > HISTORY_S:
            self.detections.popleft()

    def estimate(self, seg: np.ndarray, chans: tuple):
        """seg: (n, len(chans)) samples of the mics at geometry slots chans.
        -> (azimuth, sigma, front/back margin or None, azimuth curve, elevation or None)"""
        curve, has_back, elev = self.srp(seg, chans)
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
        return angle, sigma, fb, curve, elev

    def toggle_record(self):
        if self.recorder:
            self.recorder.close()
            self.recorder = None
            return None
        cv.TAKES.mkdir(parents=True, exist_ok=True)
        self.record_path = cv.TAKES / time.strftime(f"{self.rec_prefix}_%Y%m%d_%H%M%S.wav")
        w = wave.open(str(self.record_path), "wb")
        w.setnchannels(self.rec_ch)
        w.setsampwidth(2)
        w.setframerate(FS)
        self.recorder = w
        return str(self.record_path)


# ------------------------------------------------------------------ drawing

class View360(cv.View):
    def __init__(self, model: Model360, reader, compact: bool = False, captioner=None, layout: str = DEFAULT_LAYOUT):
        super().__init__(model, reader)
        self.swap_fb = False
        self.compact = compact
        self.cap = captioner
        self.show_captions = captioner is not None
        self.layout = layout
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

    @staticmethod
    def up_text(det) -> str:
        """' · 20° up' from the raised mics (relative to the chest's horizontal), '' for a flat array."""
        e = det[6] if len(det) > 6 else None
        if e is None:
            return ""
        return " · level" if abs(e) < 5 else f" · {abs(e):.0f}° {'up' if e > 0 else 'down'}"

    def draw_info(self, p, r):
        panel(p, r, "Board")
        img = self.image
        rd = self.reader
        mask = getattr(rd, "mask", None)
        chans = "" if mask is None else f" · {bin(mask).count('1')} ch (mask 0x{mask:02X})"
        lines = [
            f"camera: {img.width()}x{img.height()} · {rd.fps:4.1f} fps" if img else "camera: no picture",
            f"mics: {self.m.rate / 1000:5.2f} kHz{chans} · gaps {rd.gaps}",
            f"layout {self.layout} · view {self.hfov:.0f}° · centre {self.offset:+.0f}°",
        ]
        with rd.lock:
            msgs = list(rd.board_msgs)[-2:]
            stats = rd.board_stats
            fw = getattr(rd, "board_fw", "")
        warn = firmware_warning(rd, self.layout) if isinstance(rd, Reader8) else ""
        p.setFont(font(13, mono=True))
        y = r.y() + 50
        for t in lines:
            p.setPen(INK)
            p.drawText(QPointF(r.x() + 14, y), t)
            y += 22
        p.setFont(font(12))
        for text, color in ((f"firmware: {fw}" if fw else "", DIM), (f"⚠ {warn}" if warn else "", ALERT),
                            ("board: " + stats if stats else "", DIM)):
            if text:
                p.setPen(color)
                p.drawText(QRectF(r.x() + 14, y - 14, r.width() - 28, 36), Qt.TextWordWrap, text)
                y += 36
        for _, msg in msgs:
            p.setPen(ALERT if "fail" in msg or "not available" in msg else DIM)
            p.drawText(QRectF(r.x() + 14, y - 14, r.width() - 28, 36), Qt.TextWordWrap, msg)
            y += 34

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
            t, angle, sigma, _, strength = det[:5]
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
                             f"{self.side_text(rel)}{self.up_text(det)} · ±{sigma:.0f}° · {strength:.0f} dB over quiet",
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
                 f"{where}{self.up_text(det)} · turn {side} to see it · ±{sigma:.0f}°",
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
        # mics, at a small scale in the middle (top view, so a raised mic sits on the one below it)
        scale = 0.22 * R / max(1.0, max(math.hypot(x, y) for _, x, y, _ in self.m.geom.values()))
        for k, (ch, (name, x, y, _)) in enumerate(sorted(self.m.geom.items())):
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
                       f"{self.fb_text(det)}{self.up_text(det)} · {len(self.m.active())} mics")
        else:
            p.setFont(font(15))
            p.setPen(DIM)
            p.drawText(QRectF(r.x(), r.y() + 36, r.width(), 24), Qt.AlignHCenter, "clap, snap or talk")
        p.setPen(Qt.NoPen)
        p.setBrush(INK)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

    def draw_levels(self, p, r):
        panel(p, r, "Mic levels (dBFS, above 150 Hz)")
        geom = sorted(self.m.geom.items())
        inner = r.adjusted(52, 30, -48, -6) if self.compact else r.adjusted(150, 34, -64, -8)
        row_h = inner.height() / max(1, len(geom))

        def xf(db):
            return inner.left() + float(np.clip((db + 80) / 80, 0, 1)) * inner.width()

        for k, (ch, (name, *_)) in enumerate(geom):
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
    # the front-most mics, lowest first (vest6: the row of four)
    chans = [ch for ch, _ in sorted(geometry.items(), key=lambda kv: (-kv[1][2], kv[1][3]))][:mcv.CH]
    chans += chans[:1] * (mcv.CH - len(chans))
    return mcv.Captioner(model_name), chans


# ------------------------------------------------------------------ window

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: find the board automatically)")
    ap.add_argument("--layout", choices=list(LAYOUTS), default=DEFAULT_LAYOUT,
                    help=f"mic layout preset (default {DEFAULT_LAYOUT}: {LAYOUTS[DEFAULT_LAYOUT][0]})")
    ap.add_argument("--geometry", help="mic positions JSON instead of a preset (see the top of this file)")
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
    geometry = load_geometry(args.geometry, args.layout)
    layout = "custom" if args.geometry else args.layout
    if layout == "vest6":
        model = Model360(geometry, rec_ch=6, rec_prefix="vest6")
    else:
        model = Model360(geometry, rec_prefix="vest8" if layout == "vest8" else "cam8")
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
    view = View360(model, reader, compact, captioner, layout)
    view.hfov = float(settings.value("hfov", 60.0))
    view.offset = float(settings.value("offset", 0.0))
    view.swap = settings.value("swap", "false") in (True, "true")
    view.swap_fb = settings.value("swap_fb", "false") in (True, "true")
    view.rotate = int(settings.value("rotate", 0))
    view.mirror = settings.value("mirror", "false") in (True, "true")

    win = QWidget()
    win.setWindowTitle(f"Camera + sound direction · 360° · {layout}")
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
        if not v and captioner is not None:
            captioner.discard_pending()

    def on_focus(v):
        captioner.focus.enabled = v

    captions.toggled.connect(on_captions)
    focus.toggled.connect(on_focus)

    def tick():
        chunks = reader.take()
        model.push(chunks)
        # The board owns the two PWM outputs; direction finding remains here. Refresh at 10 Hz so the
        # firmware's 500 ms watchdog turns both motors off if this app or the USB connection stops.
        now = time.monotonic()
        haptic_firmware = reader.board_fw.startswith(("cam_mic_vest8 v4", "cam_mic_vest8 v5", "cam_mic_vest8 v6", "cam_mic_vest8 v7")) if isinstance(reader, Reader8) else False
        if haptic_firmware:
            det = model.last_det
            angle = view.oriented(det[1]) if det and model.now - det[0] < 0.35 else None
            duties = []
            for motor_angle in (60.0, -60.0):  # first byte: D6 at 60 degrees; second: D7 at 300 degrees
                separation = wrap(angle - motor_angle) if angle is not None else 180.0
                duties.append(round(255 * max(0.0, math.cos(math.radians(separation)))))
            duties = tuple(duties)
            # Send a changed direction on this UI tick; unchanged duties still refresh at 10 Hz for the
            # firmware watchdog. This removes up to 100 ms from sound-to-haptic response.
            if duties != tick.last_haptic or now >= tick.next_haptic:
                reader.send(bytes((ord("h"), *duties)))
                tick.last_haptic = duties
                tick.next_haptic = now + 0.1
        if captioner is not None and captions.isChecked():
            captioner.feed([c[:, cap_chans] for c in chunks])
        view.update_frame()
        recs = f"  ● REC {model.record_path.name}" if model.recorder else ""
        caps = f" · {captioner.status}" if compact and captioner is not None else ""
        fw = getattr(reader, "board_fw", "")
        fw = f" · {fw.split(' · ')[0]}" if fw else ""
        warn = firmware_warning(reader, layout) if isinstance(reader, Reader8) else ""
        if warn != status.property("warn"):
            status.setProperty("warn", warn)
            status.setStyleSheet(f"color: {'#ff7b72' if warn else '#8b949e'}; font-size: {11 if compact else 12}px;")
            status.setToolTip(warn)
        status.setText(f"⚠ {warn}" if warn and compact else
                       f"{reader.port or '-'} · {reader.status}{fw}{caps}{recs}" + (f" · ⚠ {warn}" if warn else ""))
        view.update()

    tick.next_haptic = 0.0
    tick.last_haptic = None

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
