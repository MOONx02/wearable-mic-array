"""Live one-mic-at-a-time checker for the 8-mic test board (eight_mic_stream.ino).

  ~/.mic-caption-env/bin/python tools/mic_test_ui.py               # finds the board
  ~/.mic-caption-env/bin/python tools/mic_test_ui.py --port /dev/cu.usbmodem11101 --channel 1

Wire one mic, pick its channel (DAT on D1: SEL=GND is ch1, SEL=3V3 is ch0), stay quiet
for a couple of seconds, then tap the mic. The banner says whether the mic is sending and
whether it heard the tap. Press "Next mic" before swapping in the next one.
The small meters show all eight slots, so a mic that lands on the wrong slot still shows up.
"""

from __future__ import annotations

import argparse
import glob
import os
import struct
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QApplication, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eight_mic_capture import MAGIC, MAGIC4, MAGIC_ST, SLOTS, is_flat, parse_status  # noqa: E402
from two_mic_capture import FS, open_port  # noqa: E402

CH = 8
HIST_S = 10.0
WAVE_S = 2.0
BLOCK = FS // 10           # 0.1 s level blocks
TAP_JUMP_DB = 15.0         # a tap close to the port is far more than this above the quiet level
LOUD_FLOOR_DB = -30.0      # a quiet room on a working mic sits well below this

BG, PANEL, EDGE = QColor("#0d1117"), QColor("#161b22"), QColor("#30363d")
INK, DIM = QColor("#e6edf3"), QColor("#8b949e")
GOOD, WARN, BAD, ACCENT = QColor("#3fb950"), QColor("#d29922"), QColor("#f85149"), QColor("#58a6ff")


def slot_label(c: int) -> str:
    pin, sel, _ = SLOTS[c].split()
    return f"{pin} · SEL {sel}"


def block_db(x: np.ndarray) -> np.ndarray:
    """Level of each 0.1 s block, in dBFS, DC removed. x is (n_blocks * BLOCK,)."""
    b = x.reshape(-1, BLOCK).astype(float)
    b -= b.mean(axis=1, keepdims=True)
    return 10 * np.log10(np.mean((b / 32768.0) ** 2, axis=1) + 1e-12)


# ------------------------------------------------------------------ serial reader

class Reader(threading.Thread):
    """Finds the board, parses M8 and ST packets, and queues int16 (frames, 8) chunks."""

    def __init__(self, port: str | None):
        super().__init__(daemon=True)
        self.want_port = port
        self.lock = threading.Lock()
        self.chunks: deque[np.ndarray] = deque()
        self.status = "searching for the board…"
        self.board_msg = ""
        self.port: str | None = None
        self.packets = 0
        self.gaps = 0
        self.wrong_firmware = False

    def run(self):
        while True:
            path = self.want_port or next(iter(sorted(glob.glob("/dev/cu.usbmodem*"))), None)
            if not path:
                self.status = "no board on USB: plug it in"
                time.sleep(1)
                continue
            try:
                fd = open_port(path)
            except OSError as e:
                self.status = f"can't open {path}: {e.strerror}"
                time.sleep(1)
                continue
            self.port, self.status = path, f"connected to {path}"
            try:
                self._pump(fd)
            except OSError:
                pass
            finally:
                os.close(fd)
            self.port, self.status = None, "board disconnected, searching…"
            time.sleep(1)

    def _pump(self, fd: int):
        buf, last_seq, last_data = b"", None, time.time()
        while True:
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                data = b""
            if not data:
                if not os.path.exists(self.port):
                    return
                if time.time() - last_data > 3:
                    self.status = f"{self.port}: no data from the board (opening the port resets it; wait a few s)"
                time.sleep(0.005)
                continue
            last_data = time.time()
            self.status = f"connected to {self.port}"
            buf += data
            while True:
                i = buf.find(b"\xa5\x5a")
                if i < 0 or len(buf) < i + 4:
                    buf = buf[max(0, len(buf) - 3):] if i < 0 else buf[i:]
                    break
                kind = buf[i:i + 4]
                if kind == MAGIC_ST:
                    text, buf = parse_status(buf, i)
                    if text is None:
                        break
                    self.board_msg = text
                    continue
                if kind == MAGIC4:
                    self.wrong_firmware = True
                    buf = buf[i + 4:]
                    continue
                if kind != MAGIC:
                    buf = buf[i + 2:]
                    continue
                if len(buf) < i + 10:
                    buf = buf[i:]
                    break
                seq, frames = struct.unpack_from("<IH", buf, i + 4)
                end = i + 10 + frames * CH * 2
                if len(buf) < end:
                    buf = buf[i:]
                    break
                raw = np.frombuffer(buf[i + 10:end], dtype="<i2")
                buf = buf[end:]
                if raw.size != frames * CH:
                    continue
                if last_seq is not None and seq != last_seq + 1:
                    self.gaps += 1
                last_seq = seq
                self.packets += 1
                with self.lock:
                    self.chunks.append(raw.reshape(-1, CH).copy())

    def take(self) -> np.ndarray | None:
        with self.lock:
            if not self.chunks:
                return None
            out = np.concatenate(self.chunks)
            self.chunks.clear()
        return out


# ------------------------------------------------------------------ widgets

class Panel(QWidget):
    def paint_frame(self, p: QPainter, title: str):
        p.fillRect(self.rect(), PANEL)
        p.setPen(QPen(EDGE, 1))
        p.drawRect(self.rect().adjusted(0, 0, -1, -1))
        p.setPen(DIM)
        p.setFont(QFont("Helvetica", 11))
        p.drawText(10, 18, title)


class WaveView(Panel):
    """Last 2 s of the chosen channel, DC removed, auto-scaled."""

    def __init__(self):
        super().__init__()
        self.x = np.zeros(int(WAVE_S * FS))
        self.flat = True
        self.title = ""
        self.setMinimumHeight(220)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        scale = max(300.0, float(np.max(np.abs(self.x))) if self.x.size else 300.0)
        self.paint_frame(p, f"{self.title}   waveform, last {WAVE_S:.0f} s   (±{scale:.0f} counts)")
        r = QRectF(10, 28, self.width() - 20, self.height() - 38)
        mid = r.center().y()
        p.setPen(QPen(EDGE, 1, Qt.DashLine))
        p.drawLine(r.left(), mid, r.right(), mid)
        if self.flat:
            p.setPen(BAD)
            p.setFont(QFont("Helvetica", 22, QFont.Bold))
            p.drawText(r, Qt.AlignCenter, "FLAT: nothing coming from this mic slot")
            return
        cols = int(r.width())
        n = len(self.x) // cols * cols
        if n == 0:
            return
        seg = self.x[-n:].reshape(cols, -1)
        lo, hi = seg.min(axis=1) / scale, seg.max(axis=1) / scale
        p.setPen(QPen(ACCENT, 1))
        half = r.height() / 2
        for k in range(cols):
            xk = r.left() + k
            p.drawLine(xk, mid - hi[k] * half, xk, mid - lo[k] * half)


class LevelHistory(Panel):
    """Level of the chosen channel over the last 10 s, with the quiet level and the tap threshold."""

    def __init__(self):
        super().__init__()
        self.levels = np.array([])
        self.floor = None
        self.setMinimumHeight(170)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        self.paint_frame(p, f"level, last {HIST_S:.0f} s (dBFS)   grey = quiet level, dashed = tap threshold")
        r = QRectF(44, 28, self.width() - 54, self.height() - 38)
        lo, hi = -90.0, 0.0

        def y(v):
            return r.bottom() - (min(max(v, lo), hi) - lo) / (hi - lo) * r.height()

        p.setFont(QFont("Helvetica", 9))
        for v in (-80, -60, -40, -20, 0):
            p.setPen(QPen(EDGE, 1))
            p.drawLine(r.left(), y(v), r.right(), y(v))
            p.setPen(DIM)
            p.drawText(4, int(y(v)) + 4, f"{v}")
        if self.floor is not None:
            p.setPen(QPen(DIM, 2))
            p.drawLine(r.left(), y(self.floor), r.right(), y(self.floor))
            p.setPen(QPen(WARN, 1, Qt.DashLine))
            p.drawLine(r.left(), y(self.floor + TAP_JUMP_DB), r.right(), y(self.floor + TAP_JUMP_DB))
        n = len(self.levels)
        if n < 2:
            return
        total = int(HIST_S * 10)
        dx = r.width() / (total - 1)
        x0 = r.right() - (n - 1) * dx
        p.setPen(QPen(ACCENT, 2))
        for k in range(1, n):
            p.drawLine(x0 + (k - 1) * dx, y(self.levels[k - 1]), x0 + k * dx, y(self.levels[k]))


class Meters(Panel):
    """All eight slots: bar = level over the last 0.5 s, grey FLAT = not sending."""

    def __init__(self):
        super().__init__()
        self.levels = [-120.0] * CH
        self.flat = [True] * CH
        self.selected = 1
        self.setMinimumHeight(170)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        self.paint_frame(p, "all 8 slots (a mic on the wrong pin or SEL shows up here)")
        w = (self.width() - 20) / CH
        top, bottom = 30, self.height() - 40
        for c in range(CH):
            x = 10 + c * w
            cell = QRectF(x + 6, top, w - 12, bottom - top)
            p.fillRect(cell, BG)
            if self.flat[c]:
                p.setPen(DIM)
                p.setFont(QFont("Helvetica", 11, QFont.Bold))
                p.drawText(cell, Qt.AlignCenter, "FLAT")
            else:
                frac = min(max((self.levels[c] + 90) / 90, 0), 1)
                bar = QRectF(cell.left(), cell.bottom() - frac * cell.height(), cell.width(), frac * cell.height())
                p.fillRect(bar, GOOD if self.levels[c] < LOUD_FLOOR_DB else WARN)
                p.setPen(INK)
                p.setFont(QFont("Helvetica", 10))
                p.drawText(cell.adjusted(0, 4, 0, 0), Qt.AlignHCenter | Qt.AlignTop, f"{self.levels[c]:.0f}")
            if c == self.selected:
                p.setPen(QPen(ACCENT, 3))
                p.drawRect(cell.adjusted(-3, -3, 3, 3))
            p.setPen(INK if c == self.selected else DIM)
            p.setFont(QFont("Helvetica", 10, QFont.Bold if c == self.selected else QFont.Normal))
            p.drawText(QRectF(x, bottom + 4, w, 16), Qt.AlignHCenter, f"ch{c}")
            p.setFont(QFont("Helvetica", 9))
            p.drawText(QRectF(x, bottom + 18, w, 16), Qt.AlignHCenter, slot_label(c))


# ------------------------------------------------------------------ window

class Window(QWidget):
    def __init__(self, reader: Reader, channel: int):
        super().__init__()
        self.reader = reader
        self.channel = channel
        self.hist = np.zeros((int(HIST_S * FS), CH), np.int16)
        self.filled = 0
        self.tap_db: float | None = None
        self.setWindowTitle("Mic test: one mic at a time")
        self.resize(1100, 820)
        self.setStyleSheet(f"background:{BG.name()}; color:{INK.name()};")

        self.banner = QLabel()
        self.banner.setWordWrap(True)
        self.banner.setMinimumHeight(90)
        self.banner.setFont(QFont("Helvetica", 20, QFont.Bold))
        self.detail = QLabel()
        self.detail.setWordWrap(True)
        self.detail.setFont(QFont("Helvetica", 13))

        picker = QHBoxLayout()
        lbl = QLabel("Mic slot:")
        lbl.setFont(QFont("Helvetica", 13))
        picker.addWidget(lbl)
        self.buttons = []
        for c in range(CH):
            b = QPushButton(f"ch{c}\n{slot_label(c)}")
            b.setCheckable(True)
            b.clicked.connect(lambda _=False, c=c: self.pick(c))
            picker.addWidget(b)
            self.buttons.append(b)
        nxt = QPushButton("Next mic\n(clear result)")
        nxt.clicked.connect(self.reset)
        picker.addWidget(nxt)

        self.wave = WaveView()
        self.history = LevelHistory()
        self.meters = Meters()
        self.footer = QLabel()
        self.footer.setStyleSheet(f"color:{DIM.name()};")

        lay = QVBoxLayout(self)
        lay.addWidget(self.banner)
        lay.addWidget(self.detail)
        lay.addLayout(picker)
        lay.addWidget(self.wave, 3)
        lay.addWidget(self.history, 2)
        lay.addWidget(self.meters, 2)
        lay.addWidget(self.footer)
        self.pick(channel)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(33)

    def pick(self, c: int):
        self.channel = c
        for k, b in enumerate(self.buttons):
            b.setChecked(k == c)
            b.setStyleSheet(f"padding:6px; border:1px solid {EDGE.name()}; border-radius:6px;"
                            f"background:{(ACCENT if k == c else PANEL).name()};"
                            f"color:{(BG if k == c else INK).name()};")
        self.meters.selected = c
        self.tap_db = None

    def reset(self):
        self.hist[:] = 0
        self.filled = 0
        self.tap_db = None

    def set_banner(self, color: QColor, text: str, detail: str):
        self.banner.setText(text)
        self.banner.setStyleSheet(f"background:{color.name()}; color:#0d1117; padding:14px; border-radius:8px;")
        self.detail.setText(detail)

    def tick(self):
        new = self.reader.take()
        if new is not None:
            n = min(len(new), len(self.hist))
            self.hist = np.concatenate([self.hist[n:], new[-n:]])
            self.filled = min(self.filled + n, len(self.hist))

        r = self.reader
        self.footer.setText(f"{r.status}   ·   packets {r.packets}, gaps {r.gaps}"
                            + (f"   ·   board: {r.board_msg}" if r.board_msg else ""))
        c = self.channel
        recent = self.hist[-FS // 2:] if self.filled >= FS // 2 else self.hist[-max(self.filled, 1):]
        flat_now = [is_flat(recent[:, k]) for k in range(CH)]
        lvl_now = [float(block_db(recent[-BLOCK * (len(recent) // BLOCK):, k]).max())
                   if len(recent) >= BLOCK else -120.0 for k in range(CH)]
        self.meters.levels, self.meters.flat = lvl_now, flat_now

        self.wave.title = f"ch{c} ({slot_label(c)})"
        self.wave.x = self.hist[-int(WAVE_S * FS):, c].astype(float)
        self.wave.x -= self.wave.x.mean()
        self.wave.flat = flat_now[c]

        nblk = self.filled // BLOCK
        levels = block_db(self.hist[len(self.hist) - nblk * BLOCK:, c]) if nblk else np.array([])
        self.history.levels = np.array([]) if flat_now[c] else levels
        floor = float(np.percentile(levels, 20)) if nblk >= 10 else None
        self.history.floor = None if flat_now[c] else floor
        if floor is not None and not flat_now[c]:
            jump = float(levels.max() - floor)
            if jump >= TAP_JUMP_DB and (self.tap_db is None or jump > self.tap_db):
                self.tap_db = jump

        self.wave.update()
        self.history.update()
        self.meters.update()
        self.update_verdict(c, flat_now, lvl_now, floor)

    def update_verdict(self, c, flat_now, lvl_now, floor):
        r = self.reader
        others = [k for k in range(CH) if k != c and not flat_now[k]]
        if r.wrong_firmware:
            self.set_banner(WARN, "Board is running the 4-mic camera firmware",
                            "Flash eight_mic_stream to this board, then reopen this window.")
        elif r.port is None or r.packets == 0:
            self.set_banner(DIM, "Waiting for the board…", r.status)
        elif self.filled < FS:
            self.set_banner(DIM, "Listening…", "Stay quiet for a moment so the quiet level can be measured.")
        elif flat_now[c]:
            hint = ("Check, in order: 3V3 and GND reach the mic, CLK is in the D0 row, DAT is in the "
                    f"{SLOTS[c].split()[0]} row, and the XIAO header pins are soldered. "
                    "If a known-good mic in the same spot is also FLAT, it's the wiring, not the mic.")
            if others:
                hint = (f"But {', '.join(f'ch{k}' for k in others)} has signal, so the mic is alive and "
                        "landing on another slot: click that channel. ch0/ch1 swapped = SEL on the other rail; "
                        "ch2–ch7 = DAT is in the wrong pin's row.")
            self.set_banner(BAD, f"NO SIGNAL on ch{c}", hint)
        elif self.tap_db is not None:
            self.set_banner(GOOD, f"MIC WORKING on ch{c}",
                            f"Heard your tap {self.tap_db:.0f} dB above the quiet level "
                            f"({floor:.0f} dBFS). Press \"Next mic\", swap mics, and repeat.")
        elif floor is not None and floor > LOUD_FLOOR_DB:
            self.set_banner(WARN, f"ch{c} is sending, but very loud while quiet ({floor:.0f} dBFS)",
                            "A working mic in a quiet room sits around −60 to −45 dBFS. This looks like a "
                            "loose DAT wire or a damaged mic. Tap it: if the level doesn't jump, suspect the mic.")
        else:
            quiet = f"quiet level {floor:.0f} dBFS" if floor is not None else "measuring the quiet level"
            self.set_banner(WARN, f"ch{c} is sending: now tap the mic",
                            f"Signal present ({quiet}). Tap near the mic's sound port or snap your fingers "
                            f"next to it; it passes once a sound is {TAP_JUMP_DB:.0f} dB above quiet.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: first /dev/cu.usbmodem*)")
    ap.add_argument("--channel", type=int, default=1, choices=range(CH),
                    help="slot to check first (default 1: DAT on D1, SEL on GND)")
    args = ap.parse_args()
    app = QApplication(sys.argv)
    reader = Reader(args.port)
    reader.start()
    win = Window(reader, args.channel)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
