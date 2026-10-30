"""Compare noise-reduction options on a 2-mic take, offline.

  python3 tools/noise_compare.py mic_takes/take.wav --quiet 3 --play

Record the take with the noise already playing: stay silent for the first few
seconds (--quiet, default 3 s), then talk normally. The quiet start is used to
learn what the noise sounds like, the same way the vest would learn it while
push-to-talk is up.

Writes a folder next to the take:
  raw_ch0.wav       one mic, untouched
  beam.wav          both mics lined up on the voice and averaged (delay-and-sum)
  ns.wav            one mic + noise suppression
  beam_ns.wav       delay-and-sum + noise suppression (what the vest would send)
  before_after.wav  raw_ch0, a short gap, then beam_ns

and prints, for each version, the noise level, voice level and voice-to-noise
ratio in the 300-3400 Hz band a radio carries. numpy only.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import wave
from pathlib import Path

import numpy as np

from two_mic_capture import FS, C

N = 512
HOP = 256
WIN = np.sqrt(np.hanning(N + 1)[:N])   # periodic sqrt-Hann: analysis x synthesis overlap-adds to 1
FREQS = np.fft.rfftfreq(N, 1 / FS)
BAND = (FREQS >= 300) & (FREQS <= 3400)


def read_wav(path: Path) -> np.ndarray:
    with wave.open(str(path), "rb") as w:
        if w.getframerate() != FS or w.getsampwidth() != 2:
            raise SystemExit(f"{path}: expected 16-bit {FS} Hz audio")
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
        x = x.reshape(-1, w.getnchannels()).astype(float) / 32768.0
    return x - x.mean(axis=0)


def write_wav(path: Path, x: np.ndarray):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes((np.clip(x, -1, 1) * 32767).astype("<i2").tobytes())


def stft(x: np.ndarray) -> np.ndarray:
    pad = np.concatenate([np.zeros(N - HOP), x, np.zeros(N)])
    n = 1 + (len(pad) - N) // HOP
    idx = np.arange(N)[None, :] + HOP * np.arange(n)[:, None]
    return np.fft.rfft(pad[idx] * WIN, axis=1)


def istft(X: np.ndarray, length: int) -> np.ndarray:
    frames = np.fft.irfft(X, N, axis=1) * WIN
    out = np.zeros(HOP * (len(frames) - 1) + N)
    for i, f in enumerate(frames):
        out[i * HOP:i * HOP + N] += f
    return out[N - HOP:N - HOP + length]


def band_power(x: np.ndarray) -> np.ndarray:
    """Per-frame power in the radio band, as a fraction of full scale."""
    return 2 * np.sum(np.abs(stft(x)[:, BAND]) ** 2, axis=1) / (N * N * np.mean(WIN ** 2))


def voice_lag(ch0: np.ndarray, ch1: np.ndarray, quiet: np.ndarray, talk: np.ndarray, max_lag: float) -> float:
    """Samples by which the voice reaches ch1 after ch0.

    Uses the cross-spectrum while talking minus the cross-spectrum of the quiet start,
    so a loud noise source from another direction doesn't win.
    """
    X0, X1 = stft(ch0), stft(ch1)
    cross = X1 * np.conj(X0)
    cv = (cross[talk].mean(axis=0) - cross[quiet].mean(axis=0))[BAND]
    cv /= np.abs(cv) + 1e-12
    lags = np.arange(-max_lag, max_lag + 1e-9, 0.05)
    score = np.real(np.exp(2j * np.pi * np.outer(lags, FREQS[BAND]) / FS) @ cv)
    return float(lags[np.argmax(score)])


def shift(x: np.ndarray, lag: float) -> np.ndarray:
    """Advance x by lag samples (fractional), so a signal that arrives lag samples late lines up."""
    n = 1 << int(math.ceil(math.log2(len(x) + 2 * abs(lag) + 64)))
    X = np.fft.rfft(x, n) * np.exp(2j * np.pi * np.fft.rfftfreq(n) * lag)
    return np.fft.irfft(X, n)[:len(x)]


def suppress(x: np.ndarray, quiet: np.ndarray, floor_db: float, alpha: float = 0.98) -> np.ndarray:
    """Wiener noise suppression with a decision-directed SNR estimate.

    The noise spectrum is learned from the quiet frames. The gain never drops below
    floor_db, so noise is turned down rather than gated to silence.
    """
    X = stft(x)
    noise = np.mean(np.abs(X[quiet]) ** 2, axis=0) + 1e-12
    gmin = 10 ** (floor_db / 20)
    prev = np.zeros(X.shape[1])
    for i in range(len(X)):
        gamma = np.abs(X[i]) ** 2 / noise
        xi = alpha * prev + (1 - alpha) * np.maximum(gamma - 1, 0)
        g = np.maximum(xi / (1 + xi), gmin)
        X[i] *= g
        prev = g ** 2 * gamma
    return istft(X, len(x))


def db(p: float) -> float:
    return 10 * math.log10(max(p, 1e-12))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("take", type=Path, help="2-channel 16 kHz WAV (mono works, minus the beam)")
    ap.add_argument("--quiet", type=float, default=3.0, help="seconds of noise-only at the start")
    ap.add_argument("--spacing", type=float, default=15.0, help="mic spacing in cm")
    ap.add_argument("--floor", type=float, default=-15.0, help="most the suppressor turns noise down, dB")
    ap.add_argument("--play", action="store_true", help="play before_after.wav when done (macOS)")
    args = ap.parse_args()

    x = read_wav(args.take)
    stereo = x.shape[1] >= 2
    ch0 = x[:, 0]

    p0 = band_power(ch0)
    n_quiet = max(1, int((args.quiet - 0.25) * FS - (N - HOP)) // HOP)
    quiet = np.zeros(len(p0), bool)
    quiet[:n_quiet] = True
    floor = np.median(p0[quiet])
    talk = (~quiet) & (np.arange(len(p0)) * HOP > args.quiet * FS) & (p0 > floor * 10 ** 0.6)
    if talk.sum() < 10:
        raise SystemExit("hardly any frames are 6 dB above the quiet start: was there talking after "
                         f"the first {args.quiet:g} s, and was the start actually quiet?")

    versions = {"raw_ch0": ch0}
    if stereo:
        ch1 = x[:, 1]
        lag = voice_lag(ch0, ch1, quiet, talk, args.spacing / 100 / C * FS + 1)
        # Weight each mic by voice amplitude / noise power, scaled so the voice keeps ch0's level.
        amp, wts = [], []
        for ch in (ch0, ch1):
            p = band_power(ch)
            noise = np.mean(p[quiet])
            amp.append(math.sqrt(max(np.mean(p[talk]) - noise, 1e-12)))
            wts.append(amp[-1] / noise)
        scale = amp[0] / (wts[0] * amp[0] + wts[1] * amp[1])
        versions["raw_ch1"] = ch1
        versions["beam"] = scale * (wts[0] * ch0 + wts[1] * shift(ch1, lag))
        print(f"voice reaches ch1 {lag / FS * 1e6:+.0f} us after ch0 ({lag:+.2f} samples); "
              f"beam lines up on that, mic weights {wts[0] / sum(wts):.2f} / {wts[1] / sum(wts):.2f}")
    versions["ns"] = suppress(ch0, quiet, args.floor)
    if stereo:
        versions["beam_ns"] = suppress(versions["beam"], quiet, args.floor)

    out = args.take.with_name(args.take.stem + "_nc")
    out.mkdir(exist_ok=True)
    print(f"\n{len(ch0) / FS:.1f} s take, {quiet.sum()} quiet frames, {talk.sum()} talking frames, "
          f"radio band 300-3400 Hz\n")
    print(f"{'version':10s} {'noise':>11s} {'voice':>11s} {'voice/noise':>12s} {'vs raw_ch0':>11s}")
    base = None
    for name, y in versions.items():
        p = band_power(y)
        noise = float(np.mean(p[quiet]))
        voice = float(np.mean(p[talk])) - noise
        snr = db(voice) - db(noise)
        base = snr if base is None else base
        print(f"{name:10s} {db(noise):6.1f} dBFS {db(voice):6.1f} dBFS {snr:9.1f} dB {snr - base:+8.1f} dB")
        if name != "raw_ch1":
            write_wav(out / f"{name}.wav", y)

    final = versions["beam_ns" if stereo else "ns"]
    write_wav(out / "before_after.wav", np.concatenate([ch0, np.zeros(int(0.6 * FS)), final]))
    print(f"\nwrote {out}/")
    if args.play:
        subprocess.run(["afplay", str(out / "before_after.wav")])


if __name__ == "__main__":
    main()
