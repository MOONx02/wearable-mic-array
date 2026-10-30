"""Record the eight-mic stream from eight_mic_stream.ino and check each mic.

  python3 tools/eight_mic_capture.py /dev/cu.usbmodem2101 20 mic_takes/eight.wav

Every half second it prints each channel's level and marks a channel FLAT when its
samples barely move (a mic that isn't sending). When a block is loud (tap a mic),
it prints which channel was loudest, so tapping each mic in turn maps mics to
channels. Writes an 8-channel 16 kHz WAV.

Wiring is the Tier A + B drawing: clock on D0, data on D1, D3, D4, D5.
Channel order is the I2S slot mask, low bit first. On each data pin the
SEL=3V3 mic (RIGHT on the drawing) comes before the SEL=GND mic (LEFT):

  ch0 D1 3V3  Mic 2 F3     ch1 D1 GND  Mic 1 F2
  ch2 D3 3V3  Mic 4 F4     ch3 D3 GND  Mic 3 F5
  ch4 D4 3V3  Mic 6 S_R    ch5 D4 GND  Mic 5 B1
  ch6 D5 3V3  Mic 8 S_L    ch7 D5 GND  Mic 7 B4

A tap is the check. If the loud channel is the other mic of the pair, the
slot order in the buffer is swapped from this list.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import sys
import time
import wave
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from two_mic_capture import FS, db, open_port

MAGIC = b"\xa5\x5aM8"
MAGIC4 = b"\xa5\x5aM4"
MAGIC_ST = b"\xa5\x5aST"
CH = 8
FLAT_PTP = 8  # peak-to-peak counts below which a channel counts as dead
STUCK_FRACTION = 0.5  # a live mic never repeats one exact value this often; a dead data line does
SLOTS = [
    "D1 3V3 F3", "D1 GND F2", "D3 3V3 F4", "D3 GND F5",
    "D4 3V3 S_R", "D4 GND B1", "D5 3V3 S_L", "D5 GND B4",
]


def parse_status(buf: bytes, i: int):
    if len(buf) < i + 6:
        return None, buf[i:]
    (n,) = struct.unpack_from("<H", buf, i + 4)
    end = i + 6 + n
    if len(buf) < end:
        return None, buf[i:]
    text = buf[i + 6:end].decode("utf-8", "replace")
    return text, buf[end:]


def is_flat(col: np.ndarray) -> bool:
    """Not sending: no swing at all, or parked on one value with only per-packet glitches."""
    col = col.astype(np.int32)
    if np.ptp(col) < FLAT_PTP:
        return True
    _, counts = np.unique(col, return_counts=True)
    return counts.max() > STUCK_FRACTION * len(col)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("seconds", type=float)
    ap.add_argument("out", help="WAV file to write")
    args = ap.parse_args()

    print("Tap one mic at a time, close to its port. The channel that jumps is that mic.")
    print("FLAT means that slot is not sending (open wire, SEL floating, or clock on the wrong pin).")
    print("Clock is D0. Data is D1, D3, D4, D5.")
    for c, name in enumerate(SLOTS):
        print(f"  ch{c}  {name}")

    fd = open_port(args.port)
    buf, chunks, seqs = b"", [], []
    t0 = time.time()
    shown = 0
    warned_m4 = False
    while time.time() - t0 < args.seconds:
        try:
            data = os.read(fd, 65536)
        except BlockingIOError:
            data = b""
        if not data:
            time.sleep(0.005)
            continue
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
                print(f"board: {text}", flush=True)
                continue
            if kind == MAGIC4 and not warned_m4:
                warned_m4 = True
                print("board is still sending the 4-mic stream (M4). Flash eight_mic_stream.", flush=True)
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
            if raw.size != frames * CH:
                buf = buf[i + 2:]
                continue
            chunks.append(raw.reshape(-1, CH).copy())
            seqs.append(seq)
            buf = buf[end:]
        total = sum(len(c) for c in chunks)
        while chunks and total >= (shown + 1) * FS // 2:
            raw = np.concatenate(chunks)[shown * FS // 2:(shown + 1) * FS // 2].astype(np.int32)
            x = raw.astype(float) - raw.mean(axis=0)
            levels = [db(x[:, c]) for c in range(CH)]
            flat = [is_flat(raw[:, c]) for c in range(CH)]
            cells = [f"ch{c} {'FLAT ' if flat[c] else f'{levels[c]:5.1f}'}" for c in range(CH)]
            line = f"{shown * 0.5:5.1f} s   " + "  ".join(cells) + "  dBFS"
            live = [c for c in range(CH) if not flat[c]]
            if live and max(levels[c] for c in live) > -45:
                loudest = max(live, key=lambda c: levels[c])
                line += f"\n          loudest ch{loudest} ({SLOTS[loudest]})"
            print(line, flush=True)
            shown += 1
    os.close(fd)

    if not chunks:
        raise SystemExit("no 8-mic packets: is eight_mic_stream flashed, and is the port right?")
    x = np.concatenate(chunks)
    gaps = int(np.sum(np.diff(np.array(seqs, dtype=np.int64)) != 1)) if len(seqs) > 1 else 0
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(out), "wb") as w:
        w.setnchannels(CH)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(x.astype("<i2").tobytes())
    xf = x.astype(float) - x.mean(axis=0)
    print(f"\nwrote {out}: {len(x) / FS:.1f} s, packets {len(seqs)}, sequence gaps {gaps}")
    if gaps:
        print("  gaps mean the USB link dropped mic packets. Levels are still usable; timing across a gap is not.")
    n_flat = 0
    for c in range(CH):
        flat = is_flat(x[:, c])
        n_flat += int(flat)
        state = "FLAT (not sending)" if flat else "ok"
        print(f"  ch{c} {SLOTS[c]:7s}  {db(xf[:, c]):6.1f} dBFS   min/max {int(x[:, c].min()):6d}/{int(x[:, c].max()):6d}   {state}")
    if n_flat == CH:
        print("  every channel is flat. Check 3V3, GND, and that the clock wire is on D0.")
    live = [c for c in range(CH) if not is_flat(x[:, c])]
    if len(live) > 1:
        corr = np.corrcoef(xf[:, live].T)
        print("  correlation (live channels):")
        for r, c in enumerate(live):
            print(f"    ch{c}: " + " ".join(f"{v:+.2f}" for v in corr[r]))
        for r, a in enumerate(live):
            for b in live[r + 1:]:
                if abs(corr[live.index(a), live.index(b)]) > 0.98:
                    print(f"  ch{a} and ch{b} are almost the same signal "
                          "(both SEL pins on that pair may be tied the same way, or the data wires are shorted).")


if __name__ == "__main__":
    main()
