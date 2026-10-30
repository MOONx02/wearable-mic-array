"""Record the four-mic stream from four_mic_stream.ino and check each mic.

  python3 tools/four_mic_capture.py /dev/cu.usbmodem11101 30 mic_takes/four.wav

Every half second it prints each channel's level and marks a channel FLAT when its
samples barely move (a mic that isn't sending). When a block is loud (tap a mic or
clap), it prints which channel was loudest, so tapping each mic in turn maps mics to
channels, and the arrival-time difference across each pair with the direction it
implies. Writes a 4-channel 16 kHz WAV.

Channel order assumed: ch0/ch1 = line 0 on D0 (wide pair), ch2/ch3 = line 1 on D3
(narrow pair). Confirm it with the tap test.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import time
import wave

import numpy as np

from two_mic_capture import C, FS, db, gcc_phat, open_port

MAGIC = b"\xa5\x5aM4"
CH = 4
FLAT_PTP = 8  # peak-to-peak counts below which a channel counts as dead


def pair_report(x: np.ndarray, a: int, b: int, spacing: float) -> str:
    lag = gcc_phat(x[:, a], x[:, b], int(math.ceil(spacing / C * FS)) + 2)
    s = max(-1.0, min(1.0, lag / FS * C / spacing))
    return (f"ch{b} later than ch{a} by {lag / FS * 1e6:+5.0f} us "
            f"-> {math.degrees(math.asin(s)):+4.0f} deg toward ch{a}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("seconds", type=float)
    ap.add_argument("out", help="WAV file to write")
    ap.add_argument("--wide", type=float, default=0.140, help="ch0-ch1 spacing in metres")
    ap.add_argument("--narrow", type=float, default=0.035, help="ch2-ch3 spacing in metres")
    args = ap.parse_args()

    fd = open_port(args.port)
    buf, chunks, seqs = b"", [], []
    t0 = time.time()
    shown = 0
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
            i = buf.find(MAGIC)
            if i < 0 or len(buf) < i + 10:
                buf = buf[max(0, len(buf) - 3):] if i < 0 else buf[i:]
                break
            seq, frames = struct.unpack_from("<IH", buf, i + 4)
            end = i + 10 + frames * CH * 2
            if len(buf) < end:
                buf = buf[i:]
                break
            chunks.append(np.frombuffer(buf[i + 10:end], dtype="<i2").reshape(-1, CH))
            seqs.append(seq)
            buf = buf[end:]
        total = sum(len(c) for c in chunks)
        while total >= (shown + 1) * FS // 2:
            raw = np.concatenate(chunks)[shown * FS // 2:(shown + 1) * FS // 2].astype(np.int32)
            x = raw.astype(float) - raw.mean(axis=0)
            levels = [db(x[:, c]) for c in range(CH)]
            flat = np.ptp(raw, axis=0) < FLAT_PTP
            cells = [f"ch{c} {'FLAT ' if flat[c] else f'{levels[c]:5.1f}'}" for c in range(CH)]
            line = f"{shown * 0.5:5.1f} s   " + "   ".join(cells) + "  dBFS"
            live = [c for c in range(CH) if not flat[c]]
            if live and max(levels[c] for c in live) > -45:
                loudest = max(live, key=lambda c: levels[c])
                k = int(np.argmax(np.abs(x).sum(axis=1)))
                seg = x[max(0, k - 400):k + 400]
                line += f"\n          loudest ch{loudest}"
                if not flat[0] and not flat[1]:
                    line += f" | wide: {pair_report(seg, 0, 1, args.wide)}"
                if not flat[2] and not flat[3]:
                    line += f" | narrow: {pair_report(seg, 2, 3, args.narrow)}"
            print(line, flush=True)
            shown += 1
    os.close(fd)

    if not chunks:
        raise SystemExit("no packets received: is four_mic_stream flashed and the port right?")
    x = np.concatenate(chunks)
    gaps = int(np.sum(np.diff(np.array(seqs, dtype=np.int64)) != 1))
    with wave.open(args.out, "wb") as w:
        w.setnchannels(CH)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(x.astype("<i2").tobytes())
    xf = x.astype(float) - x.mean(axis=0)
    print(f"\nwrote {args.out}: {len(x) / FS:.1f} s, packets {len(seqs)}, sequence gaps {gaps}")
    for c in range(CH):
        state = "FLAT (not sending)" if np.ptp(x[:, c].astype(np.int32)) < FLAT_PTP else "ok"
        print(f"  ch{c}: {db(xf[:, c]):6.1f} dBFS   min/max {x[:, c].min():6d}/{x[:, c].max():6d}   {state}")
    live = [c for c in range(CH) if xf[:, c].std() > 0]
    if len(live) > 1:
        corr = np.corrcoef(xf[:, live].T)
        print("  correlation (live channels):")
        for r, c in enumerate(live):
            print(f"    ch{c}: " + " ".join(f"{v:+.2f}" for v in corr[r]))


if __name__ == "__main__":
    main()
