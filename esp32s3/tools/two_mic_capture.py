"""Record the two-mic stream from two_mic_stream.ino and check it.

  python3 two_mic_capture.py /dev/cu.usbmodem11101 20 take.wav --spacing 0.08

Prints each channel's level every half second while recording. When a block is
loud (a clap), it also prints the arrival-time difference between the two mics
and the direction that implies. Writes a 2-channel 16 kHz WAV. Standard library
plus numpy only.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import termios
import time
import wave

import numpy as np

MAGIC = b"\xa5\x5aM2"
FS = 16000
C = 343.0


def open_port(path: str) -> int:
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def gcc_phat(a: np.ndarray, b: np.ndarray, max_lag: int, interp: int = 16) -> float:
    """Delay of b relative to a, in samples (positive: b hears it later)."""
    n = 1 << int(math.ceil(math.log2(len(a) + len(b))))
    cross = np.fft.rfft(b, n) * np.conj(np.fft.rfft(a, n))
    cross /= np.abs(cross) + 1e-12
    cc = np.fft.irfft(cross, n * interp)
    m = max_lag * interp
    cc = np.concatenate((cc[-m:], cc[: m + 1]))
    return (int(np.argmax(np.abs(cc))) - m) / interp


def db(x: np.ndarray) -> float:
    return 10 * math.log10(np.mean((x / 32768.0) ** 2) + 1e-12)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("seconds", type=float)
    ap.add_argument("out", help="WAV file to write")
    ap.add_argument("--spacing", type=float, default=0.08, help="mic spacing in metres")
    args = ap.parse_args()

    fd = open_port(args.port)
    buf, chunks, seqs = b"", [], []
    t0 = time.time()
    shown = 0
    max_lag = int(math.ceil(args.spacing / C * FS)) + 2
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
            end = i + 10 + frames * 4
            if len(buf) < end:
                buf = buf[i:]
                break
            chunks.append(np.frombuffer(buf[i + 10:end], dtype="<i2").reshape(-1, 2))
            seqs.append(seq)
            buf = buf[end:]
        total = sum(len(c) for c in chunks)
        while total >= (shown + 1) * FS // 2:
            x = np.concatenate(chunks)[shown * FS // 2:(shown + 1) * FS // 2].astype(float)
            x -= x.mean(axis=0)
            line = f"{shown * 0.5:5.1f} s   ch0 {db(x[:, 0]):6.1f} dBFS   ch1 {db(x[:, 1]):6.1f} dBFS"
            loud = max(db(x[:, 0]), db(x[:, 1]))
            if loud > -45:
                k = int(np.argmax(np.abs(x).sum(axis=1)))
                seg = x[max(0, k - 400):k + 400]
                lag = gcc_phat(seg[:, 0], seg[:, 1], max_lag)
                s = max(-1.0, min(1.0, lag / FS * C / args.spacing))
                line += (f"   ch1 later by {lag / FS * 1e6:+6.0f} us"
                         f"  -> {math.degrees(math.asin(s)):+4.0f} deg toward ch0's side")
            print(line, flush=True)
            shown += 1
    os.close(fd)

    if not chunks:
        raise SystemExit("no packets received: is two_mic_stream flashed and the port right?")
    x = np.concatenate(chunks)
    gaps = int(np.sum(np.diff(np.array(seqs, dtype=np.int64)) != 1))
    with wave.open(args.out, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(x.astype("<i2").tobytes())
    xf = x.astype(float) - x.mean(axis=0)
    corr = float(np.corrcoef(xf[:, 0], xf[:, 1])[0, 1]) if xf.std(axis=0).min() > 0 else float("nan")
    print(f"\nwrote {args.out}: {len(x) / FS:.1f} s, packets {len(seqs)}, sequence gaps {gaps}")
    print(f"whole take: ch0 {db(xf[:, 0]):.1f} dBFS, ch1 {db(xf[:, 1]):.1f} dBFS, "
          f"min/max ch0 {x[:, 0].min()}/{x[:, 0].max()}, ch1 {x[:, 1].min()}/{x[:, 1].max()}, "
          f"correlation {corr:.3f}")


if __name__ == "__main__":
    main()
