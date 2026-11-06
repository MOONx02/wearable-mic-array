#!/usr/bin/env python3
"""Process a 6-mic vest recording (cam_mic_vest6) on the laptop, and measure how well the noise is cancelled.

    python3 tools/vest6_process.py /Volumes/<card>             every 6-mic take on the card
    python3 tools/vest6_process.py /Volumes/<card> --take 3    just T0003
    python3 tools/vest6_process.py path/to/T0003               a take folder copied off the card
    python3 tools/vest6_process.py mic_takes/vest6_XXXX.wav    a recording from mic_camera_view360.py
    python3 tools/vest6_process.py mic_takes/vest6/T0003 --transcribe-only --model large-v3-turbo
                                                               only the transcript again, with a bigger model
    python3 tools/vest6_process.py path/to/T0003 --dereverb    also take the room's echo out (WPE) first

--model picks the speech-to-text model: small.en by default (about 1 GB while it runs). "auto" takes the
biggest one this computer runs in reasonable time: large-v3 with an NVIDIA GPU, large-v3-turbo with 24 GB+
and 10+ cores, medium.en with 16 GB+, small.en with 8 GB+, else base.en. A model downloads once on first use.

--dereverb runs WPE echo removal on the six mics before the beam (linear and multi-mic, so it goes first;
the noise filter stays last) and adds two versions: raw_mic_echo_removed.wav and
voice_focus_echo_removed.wav; the transcript is then made from the latter. The other versions are unchanged,
so voice focus with and without echo removal can be compared directly.

For each take, writes to mic_takes/vest6/<take>/ (or --out):
    raw_6mic.wav        all six mics, joined into one file (headers repaired if the power was pulled)
    the same speech at the same loudness, so only the background differs between them:
      raw_mic.wav       one mic, as it was recorded (--ref, default the mic with the typical background)
      mic_average.wav   all six mics averaged, not aimed anywhere
      beam.wav          delay-and-sum beam aimed at the talker (the mic positions + the talker's direction)
      beam_mvdr.wav     MVDR beam: voice focus without its filter (cancels the background's direction)
      mic_filter.wav    one mic + the noise filter, no array: what a single-mic noise suppressor manages
      voice_focus.wav   MVDR + noise filter: what the live captions use
    noise_report.txt    per version: background level, voice level, voice-to-noise ratio, and how much better
    noise_report.json   than the raw mic it is, overall and in three bands; also in noise_report.json:
                        when there was speech and when the background was measured
    spectrogram.png     raw mic over voice focus, the whole take
    directions.csv      every sound the mics located: time, azimuth, elevation, spread, loudness
    transcript.txt, subtitles.srt, captions.json   what was said, when, and where the talker was
    camera.bin          the camera frames, for tools/vest6_review.py

How the numbers are measured: the speech and the quiet moments between it are found by loudness in the
300-3400 Hz band (the same detector as the live voice focus). Each version is scaled so its voice is as
loud as the raw mic's; then the background left in the quiet moments says how much was removed. Talk with
the noise already on, with pauses, and stay quiet for the first second or two so there's background to
measure. Everything runs on this computer, offline (Whisper must have been downloaded once).
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

if __name__ == "__main__":
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")

import argparse  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import shutil  # noqa: E402
import time  # noqa: E402
import wave  # noqa: E402

import numpy as np  # noqa: E402
from PySide6.QtGui import QColor, QFont, QGuiApplication, QImage, QPainter  # noqa: E402

sys.path.insert(0, str(HERE.parent))
import sd_import as sd  # noqa: E402  (card reading, Whisper)
import mic_camera_view360 as v360  # noqa: E402  (vest6 geometry and the 3-D sound locator)

FS = 16000
NFFT, HOP = 512, 128
BAND = (300.0, 3400.0)
SUB_BANDS = (("300-1000 Hz", 300.0, 1000.0), ("1-2 kHz", 1000.0, 2000.0), ("2-3.4 kHz", 2000.0, 3400.0))
OUT_ROOT = HERE.parent / "mic_takes" / "vest6"
# (file, name in the report and the review app, what it is)
VERSIONS = [
    ("raw_mic.wav", "Raw mic", "one mic, as recorded"),
    ("mic_average.wav", "All mics averaged", "six mics added up, not aimed"),
    ("beam.wav", "Beam at talker", "delay-and-sum aimed at the talker"),
    ("beam_mvdr.wav", "MVDR beam", "voice focus without the filter"),
    ("mic_filter.wav", "Mic + filter", "one mic + the noise filter, no array"),
    ("voice_focus_old.wav", "Voice focus (old)", "before the tap guard, voice check and direction filter"),
    ("voice_focus.wav", "Voice focus", "MVDR + direction filter + noise filter (live captions)"),
]
KEYS = ["raw", "avg", "das", "mvdr", "filt", "old", "focus"]
# with --dereverb: the same chain again on the echo-removed mics (key in that run, row key, file, name, about)
ECHO_VERSIONS = [
    ("raw", "raw_wpe", "raw_mic_echo_removed.wav", "Raw mic, echo removed", "one mic after WPE echo removal"),
    ("focus", "focus_wpe", "voice_focus_echo_removed.wav", "Voice focus, echo removed",
     "WPE echo removal first, then voice focus"),
]
DEREVERB_TAPS = 10   # WPE: echo predicted from this many 8 ms frames (in a 0.9 s classroom, 19-31 scored a little
                     # better on SRMR but transcribed no better, at 2-4x the time) ...
DEREVERB_DELAY = 3   # ... starting this many frames back (the first 24 ms: direct sound + early reflections stay)

# What the beam learns from (the same as the live app's voice focus):
ONSET_DB = 12.0      # a bang or tap: the wide band jumps this far over the 80 ms before it
SPREAD_DB = 12.0     # handling: one mic this much louder than the others' median (a tap or rub on the plate)
HOLD, HOLD_MAX = 15, 40  # frames (8 ms) nothing is learnt after one, longer while it rings, at most
VOICED = 0.6         # pitch correlation (80-400 Hz, from the 300-2000 Hz harmonics) that counts as a voice
HANG = 19            # frames after the last voiced one that still count as speech (consonants, word ends)
ON_TARGET = 0.3      # how much a frame must come from the talker's direction for the beam to learn it
POST_FLOOR = 0.18    # the direction filter's deepest cut (-15 dB); with the noise filter, never below -20 dB
TOTAL_FLOOR = 0.1
# speech-to-text models, smallest first: (Whisper model, what it's like, download size)
DEFAULT_MODEL = "small.en"  # ~1 GB while it runs; bigger ones on request (--model, or the review app's list)
TRANSCRIBERS = [
    ("base.en", "Whisper base: fast, fine on any computer", "150 MB"),
    ("small.en", "Whisper small: better, still fine on a laptop", "500 MB"),
    ("medium.en", "Whisper medium: better again, slow on a laptop CPU", "1.5 GB"),
    ("large-v3-turbo", "Whisper large-v3 turbo: close to the best, wants 16 GB+ or a GPU", "1.6 GB"),
    ("large-v3", "Whisper large-v3: the best and biggest, wants a GPU", "3 GB"),
]


def memory_gb() -> float:
    try:
        if os.name == "nt":
            import ctypes

            class Mem(ctypes.Structure):
                _fields_ = [("len", ctypes.c_ulong), ("load", ctypes.c_ulong), ("total", ctypes.c_ulonglong),
                            ("avail", ctypes.c_ulonglong), ("pt", ctypes.c_ulonglong), ("pa", ctypes.c_ulonglong),
                            ("vt", ctypes.c_ulonglong), ("va", ctypes.c_ulonglong), ("ve", ctypes.c_ulonglong)]
            m = Mem()
            m.len = ctypes.sizeof(Mem)
            ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(m))
            return m.total / 2 ** 30
        return os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 2 ** 30
    except (AttributeError, ValueError, OSError):
        return 8.0


def auto_transcriber() -> tuple[str, str]:
    """The biggest model this computer runs in reasonable time, and why."""
    gpu = sd.speech_device()[0] == "cuda"
    gb, cores = memory_gb(), os.cpu_count() or 4
    why = f"{'NVIDIA GPU, ' if gpu else ''}{gb:.0f} GB memory, {cores} cores"
    if gpu:
        return "large-v3", why
    if gb >= 24 and cores >= 10:
        return "large-v3-turbo", why
    if gb >= 16:
        return "medium.en", why
    if gb >= 8:
        return "small.en", why
    return "base.en", why


# ------------------------------------------------------------------ reading takes

def read_take(path: Path) -> tuple[np.ndarray, dict, list, str]:
    """-> (audio (n, 6) int16, info, camera frames, layout name)."""
    if path.suffix.lower() == ".wav":
        with wave.open(str(path), "rb") as w:
            if w.getframerate() != FS or w.getsampwidth() != 2:
                raise SystemExit(f"{path}: expected 16-bit {FS} Hz audio")
            nch = w.getnchannels()
            x = np.frombuffer(w.readframes(w.getnframes()), "<i2").reshape(-1, nch)
        if nch < 6:
            raise SystemExit(f"{path}: {nch} channels; a vest6 recording has 6")
        return x[:, :6].copy(), {"mode": "voice", "mics": str(nch)}, [], "vest6"
    info = sd.read_info(path)
    nch = int(info.get("mics", 6))
    layout = info.get("mic_layout", "vest6")
    if nch != 6:
        raise SystemExit(f"{path.name}: {nch} mics. This tool is for the 6-mic vest; 4-mic takes go through sd_import.py")
    audio, repaired = sd.read_audio(path, nch)
    if repaired:
        info["repaired"] = str(repaired)
    return audio, info, sd.read_frames(path / "video.bin"), layout


def find_takes(path: Path, number: int | None) -> list[Path]:
    if path.suffix.lower() == ".wav" or (path / "info.txt").exists():
        return [path]
    root = path / "takes" if (path / "takes").is_dir() else path
    takes = sorted(p for p in root.glob("T[0-9]*") if p.is_dir() and sd.read_info(p).get("mics", "4") == "6")
    if number is not None:
        takes = [p for p in takes if p.name == f"T{number:04d}"]
    return takes


def write_wav(path: Path, x: np.ndarray):
    sd.write_wav(path, x)


# ------------------------------------------------------------------ speech, quiet and the talker's direction

WIN = np.sqrt(np.hanning(NFFT + 1)[:NFFT])
FREQS = np.fft.rfftfreq(NFFT, 1 / FS)


def frames_fft(pad: np.ndarray, a: int, b: int) -> np.ndarray:
    """Spectra of frames a..b-1 of a padded (n, ch) signal: (frames, bins, ch)."""
    idx = (np.arange(a, b) * HOP)[:, None] + np.arange(NFFT)[None, :]
    return np.fft.rfft(pad[idx] * WIN[None, :, None], axis=1)


def stft_power(x: np.ndarray, band: tuple[float, float]) -> np.ndarray:
    """Power of the signal within a band, per 8 ms frame (frames as the voice focus cuts them), in sample
    units² (so 32768² = full scale); averaged over channels. x (n,) or (n, ch)."""
    x = x if x.ndim == 2 else x[:, None]
    pad = np.concatenate([x, np.zeros((NFFT, x.shape[1]))])
    nfr = (len(pad) - NFFT) // HOP + 1
    sel = (FREQS >= band[0]) & (FREQS <= band[1])
    out = np.empty(nfr)
    for a in range(0, nfr, 2048):
        b = min(nfr, a + 2048)
        X = frames_fft(pad, a, b)
        out[a:b] = 4 * np.sum(np.abs(X[:, sel]) ** 2, axis=1).mean(axis=1) / NFFT ** 2  # Parseval, sqrt-Hann window
    return out


def dereverb(x: np.ndarray, taps: int = DEREVERB_TAPS, delay: int = DEREVERB_DELAY, iterations: int = 3) -> np.ndarray:
    """WPE (weighted prediction error): each mic's late echo is predicted from all mics' past frames and
    subtracted, frequency by frequency. The first `delay` frames after a sound (direct sound + early
    reflections) are kept, so the mics' timing, the directions and the beam still work on the output.
    Linear and multi-mic, so it goes first, before the beam and the (non-linear) noise filter."""
    n, D = x.shape
    pad = np.concatenate([x, np.zeros((NFFT, D))])
    nfr = (len(pad) - NFFT) // HOP + 1
    X = np.empty((nfr, len(FREQS), D), np.complex64)
    for a in range(0, nfr, 2048):
        X[a:a + 2048] = frames_fft(pad, a, min(nfr, a + 2048))
    del pad
    for f in np.flatnonzero((FREQS >= 80) & (FREQS <= 7000)):
        Y = X[:, f, :].T.astype(np.complex128)                  # (mics, frames)
        Yt = np.zeros((D * taps, nfr), np.complex128)            # the past: frames delay .. delay+taps-1 back
        for k in range(taps):
            s = delay + k
            Yt[k * D:(k + 1) * D, s:] = Y[:, :nfr - s]
        Z = Y
        for _ in range(iterations):
            lam = np.mean(np.abs(Z) ** 2, axis=0)
            lam = np.maximum(lam, 1e-4 * np.mean(lam) + 1e-12)   # silent frames (the padding) mustn't dominate
            W = Yt / lam
            R = W @ Yt.conj().T
            R += np.eye(D * taps) * (1e-6 * np.real(np.trace(R)) / (D * taps) + 1e-12)
            G = np.linalg.solve(R, W @ Y.conj().T)
            Z = Y - G.conj().T @ Yt
        X[:, f, :] = Z.T
    out = np.zeros((nfr * HOP + NFFT, D))
    w2 = (WIN / 2)[None, :, None]                               # sqrt-Hann in and out at 75% overlap adds up to 2
    for a in range(0, nfr, 2048):                                # 2048 frames: a multiple of the 4 overlapping phases
        y = np.fft.irfft(X[a:a + 2048], NFFT, axis=1) * w2
        for j in range(4):
            blk = y[j::4]
            s = (a + j) * HOP
            out[s:s + len(blk) * NFFT] += blk.reshape(-1, D)
    return out[:n]


def srmr(x: np.ndarray, fs: int = FS) -> float:
    """SRMR (speech-to-reverberation modulation energy ratio; Falk et al. 2010), as SRMRpy's fast mode works
    it out: 23 gammatone bands from 125 Hz, their envelopes (10 ms windows every 2.5 ms) through 8 modulation
    filters from 4 to 128 Hz, then the energy at speech's own rhythms (4-~20 Hz) over the faster modulations
    echo smears in. Higher = less echo. Needs a second or more of speech."""
    nfft, nwin, nhop, mfs = 512, int(0.010 * fs), int(0.0025 * fs), 400.0
    if len(x) < nfft + fs // 4:
        return float("nan")
    win = np.zeros(nfft)  # SRMRpy's specgram window: Hann nwin wide, centred in nfft
    half = 0.5 * (1 + np.cos(np.pi * np.arange(nwin // 2 + 1) / (nwin // 2)))
    win[nfft // 2:nfft // 2 + nwin // 2] = half[:nwin // 2]
    win[nfft // 2:nfft // 2 - nwin // 2:-1] = half[:nwin // 2]
    starts = np.arange(0, len(x) - nfft, nhop)
    S = np.zeros((nfft // 2 + 1, 1 + (len(x) - nfft) // nhop))
    for a in range(0, len(starts), 4096):
        idx = starts[a:a + 4096, None] + np.arange(nfft)[None, :]
        S[:, a:a + len(idx)] = np.abs(np.fft.rfft(x[idx] * win, axis=1)).T
    ear_q, min_bw, f = 9.26449, 24.7, np.arange(nfft // 2 + 1) * fs / nfft
    cfs = (-ear_q * min_bw + np.exp(np.arange(1, 24) / 23 * (np.log(125 + ear_q * min_bw) - np.log(fs / 2 + ear_q * min_bw)))
           * (fs / 2 + ear_q * min_bw))[::-1]  # ERB-spaced, low to high
    erb = cfs / ear_q + min_bw
    weights = (1 + ((f[None, :] - cfs[:, None]) / (1.019 * erb[:, None])) ** 2) ** -2  # 4th-order gammatone
    env = weights @ S / nfft
    mcf = 4.0 * (128.0 / 4.0) ** (np.arange(8) / 7)
    wlen, winc = int(np.ceil(0.256 * mfs)), int(np.ceil(0.064 * mfs))
    nfr = 1 + (env.shape[1] - wlen) // winc
    if nfr < 1:
        return float("nan")
    L = 1 << int(np.ceil(np.log2(env.shape[1] + 2 * int(mfs))))  # room for the filters to ring out
    z = np.exp(-1j * 2 * np.pi * np.arange(L // 2 + 1) / L)
    E = np.fft.rfft(env, L, axis=1)
    hw = np.hamming(wlen + 1)[:-1]
    fidx = np.arange(nfr)[:, None] * winc + np.arange(wlen)[None, :]
    energy = np.zeros((23, 8))
    for j, w0 in enumerate(2 * np.pi * mcf / mfs):  # 2nd-order band-pass, Q = 2 (SRMRpy's modulation filters)
        W0 = np.tan(w0 / 2)
        B0 = W0 / 2
        H = B0 * (1 - z ** 2) / ((1 + B0 + W0 ** 2) + (2 * W0 ** 2 - 2) * z + (1 - B0 + W0 ** 2) * z ** 2)
        y = np.fft.irfft(E * H[None, :], L, axis=1)[:, :env.shape[1]]
        energy[:, j] = np.sum((y[:, fidx] * hw) ** 2, axis=2).mean(axis=1)
    pct = energy.sum(axis=1) * 100 / energy.sum()
    bw = erb[int(np.flatnonzero(np.cumsum(pct[::-1]) > 90)[0])]
    w0 = 2 * np.pi * mcf / fs  # SRMRpy compares against cutoffs worked out at the audio rate; kept so numbers match
    cut = mcf - np.tan(w0 / 2) / 2 * fs / (2 * np.pi)
    kstar = 8 if bw > cut[7] else 7 if bw > cut[6] else 6 if bw > cut[5] else 5
    return float(energy[:, :4].sum() / energy[:, 4:kstar].sum())


def speech_srmr(y: np.ndarray, speech: np.ndarray) -> float:
    """Mean SRMR over the stretches of speech a second or longer (pauses are background, not echo)."""
    vals = [srmr(y[a * HOP:b * HOP]) for a, b in runs(speech, join=12, shortest=10) if (b - a) * HOP >= FS]
    vals = [v for v in vals if v == v]
    return float(np.mean(vals)) if vals else float("nan")


def speech_quiet(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The live voice focus's detector, frame by frame: speech = 8 dB over the running floor, quiet = under 3 dB."""
    e = 10 * np.log10(stft_power(x, BAND) + 1e-12)
    speech, quiet = np.zeros(len(e), bool), np.zeros(len(e), bool)
    floor = None
    for k, v in enumerate(e):
        if floor is None:
            floor = v
        elif v < floor:
            floor = 0.9 * floor + 0.1 * v
        else:
            floor += 0.002
        speech[k], quiet[k] = v > floor + 8, v < floor + 3
    return speech, quiet


WIDE = FREQS >= 150
HIGH = FREQS >= 2000
VBAND = (FREQS >= 300) & (FREQS <= 2000)  # the voice's harmonics; below 300 Hz fan rumble looks like a pitch
LAG0, LAG1 = 40, 200  # pitch periods searched, 400 Hz down to 80 Hz
_rw = np.correlate(WIN, WIN, "full")[NFFT - 1:]
RW = _rw[LAG0:LAG1] / _rw[0]  # the window's own correlation, divided out


def detect(x: np.ndarray) -> dict:
    """The live voice focus's detectors, frame by frame (bool arrays):
    loud, quiet_e   the loudness-only detector (speech_quiet): what the old version learnt from, and what the
                    measurements use
    hold            a bang, a tap or handling noise is going on (or just was)
    voiced          the summed mics have a pitch (80-400 Hz), two frames running
    speech          loud, voiced within the last 150 ms, nothing held: what the beam may learn the talker from
    quiet           quiet, not voiced, nothing held: what the background is learnt from"""
    x = x if x.ndim == 2 else x[:, None]
    pad = np.concatenate([x, np.zeros((NFFT, x.shape[1]))])
    nfr = (len(pad) - NFFT) // HOP + 1
    sel = (FREQS >= BAND[0]) & (FREQS <= BAND[1])
    names = ("loud", "quiet_e", "knock", "handled", "hold", "voiced", "speech", "quiet")
    out = {k: np.zeros(nfr, bool) for k in names}
    floor, hist, hold, held, ref, since, was_pitched = None, [], 0, 0, 0.0, 99, False
    for a in range(0, nfr, 2048):
        b = min(nfr, a + 2048)
        X = frames_fft(pad, a, b)
        P = np.abs(X) ** 2
        pm = 4 * P[:, sel].sum(axis=1) / NFFT ** 2 + 1e-12  # (frames, mics)
        E = 10 * np.log10(pm.mean(axis=1))
        Wd = 10 * np.log10(4 * P[:, WIDE].sum(axis=1).mean(axis=1) / NFFT ** 2 + 1e-12)
        Hf = 10 * np.log10(4 * P[:, HIGH].sum(axis=1).mean(axis=1) / NFFT ** 2 + 1e-12)
        spread = 10 * np.log10(pm.max(axis=1) / np.median(pm, axis=1)) if x.shape[1] > 2 else np.zeros(b - a)
        S = np.abs(X.sum(axis=2)) ** 2
        for i in range(b - a):
            k, e, w, h = a + i, E[i], Wd[i], Hf[i]
            if floor is None:
                floor = e
            elif e < floor:
                floor = 0.9 * floor + 0.1 * e
            else:
                floor += 0.002
            loud, quiet = e > floor + 8, e < floor + 3
            if len(hist) >= 12:
                med = float(np.median([v[0] for v in hist[-12:-2]]))
                hmed = float(np.median([v[1] for v in hist[-12:-2]]))
                prev = hist[-1][0]
            else:
                med, hmed, prev = w, h, w
            hist.append((w, h))
            if len(hist) > 12:
                hist.pop(0)
            # a knock arrives in one frame and in every band; a word builds up over several, mostly low
            onset = w - prev > 8 and w - med > ONSET_DB and h - hmed > ONSET_DB and e - floor > 12
            handled = loud and spread[i] > SPREAD_DB
            out["knock"][k], out["handled"][k] = onset, handled
            if onset or handled:
                hold, held, ref = HOLD, 0, med if onset else floor
            elif hold:
                held += 1
                hold -= 1
                if w > ref + 6 and held < HOLD_MAX:
                    hold = max(hold, 1)
            r = np.fft.irfft(S[i] * VBAND, NFFT)
            pitched = r[0] > 0 and float(np.max(r[LAG0:LAG1] / RW)) > VOICED * r[0]
            voiced, was_pitched = pitched and was_pitched, pitched  # two frames running: noise rarely manages that
            since = 0 if voiced else since + 1
            out["loud"][k], out["quiet_e"][k], out["hold"][k], out["voiced"][k] = loud, quiet, hold > 0, voiced
            out["speech"][k] = loud and since <= HANG and not hold
            out["quiet"][k] = quiet and not hold and not voiced
    return out


def runs(mask: np.ndarray, join: int, shortest: int) -> list[tuple[int, int]]:
    """[start, end) frame runs of mask, gaps under `join` frames bridged, runs under `shortest` dropped."""
    out: list[list[int]] = []
    for k in map(int, np.flatnonzero(mask)):
        if out and k - out[-1][1] <= join:
            out[-1][1] = k + 1
        else:
            out.append([k, k + 1])
    return [(a, b) for a, b in out if b - a >= shortest]


def live_channels(audio: np.ndarray) -> list[int]:
    """Mics that carry sound (a dead data line is flat or repeats itself)."""
    x = audio[: min(len(audio), 30 * FS)].astype(np.float32)
    flat = np.ptp(x, axis=0) < 8
    stuck = np.mean(np.diff(x, axis=0) == 0, axis=0) > 0.5
    return [c for c in range(audio.shape[1]) if not flat[c] and not stuck[c]]


class Locator:
    """mic_camera_view360's sound locator on a recording."""

    def __init__(self, geom: dict, chans: list[int]):
        self.model = v360.Model360(geom)
        self.chans = tuple(chans)

    def stream(self, audio: np.ndarray, threshold_db: float) -> list[tuple]:
        """Every sound the live viewer would have located: (t, azimuth, elevation, spread, dB over quiet)."""
        m = self.model
        m.threshold_db = threshold_db
        step, last, out = int(0.033 * FS), None, []
        pad = np.zeros((step, v360.CH), np.int16)
        for i in range(0, len(audio), step):
            blk = audio[i:i + step]
            chunk = pad[: len(blk)].copy()
            chunk[:, : audio.shape[1]] = blk
            m.push([chunk])
            d = m.last_det
            if d is not None and d is not last:
                out.append((d[0], d[1], d[6], d[2], d[4]))
                last = d
        return out


class TalkerLocator:
    """Where speech comes from, with the background taken out: the mics' cross-spectra averaged over the
    speech frames, minus their average over the quiet frames nearby (within 10 s), so a steady fan that's
    louder than the voice doesn't win; then SRP-PHAT over azimuth x elevation in the voice band."""

    def __init__(self, x: np.ndarray, geom: dict, chans: list[int], speech: np.ndarray, quiet: np.ndarray):
        self.x = np.concatenate([x, np.zeros((NFFT, x.shape[1]))])
        self.speech, self.quiet = speech, quiet
        self.sel = (FREQS >= BAND[0]) & (FREQS <= BAND[1])
        f = FREQS[self.sel]
        pos = np.array([geom[c][1:4] for c in chans]) / 1000.0
        self.els = v360.ELEV if np.ptp(pos[:, 2]) > 0.01 else np.array([0.0])
        self.planar = np.ptp(pos[:, 1]) < 0.01  # no mic in front of another: front and back look alike
        u = v360.unit_vectors(np.arange(-180, 180)[:, None], self.els[None, :]).reshape(-1, 3)
        tau = -(u @ pos.T) / v360.C  # (directions, mics) arrival times
        self.pairs = [(i, j) for i in range(len(chans)) for j in range(i + 1, len(chans))]
        self.E = [np.exp(2j * np.pi * f[None, :] * (tau[:, i] - tau[:, j])[:, None]).astype(np.complex64)
                  for i, j in self.pairs]
        # quiet frames' cross-spectra summed per second, for the background near any moment
        nsec = len(speech) * HOP // FS + 1
        M = len(chans)
        self.qsum = np.zeros((nsec, int(self.sel.sum()), M, M), np.complex64)
        self.qn = np.zeros(nsec)
        for a in range(0, len(speech), 512):
            b = min(len(speech), a + 512)
            k = np.flatnonzero(quiet[a:b]) + a
            if not len(k):
                continue
            X = frames_fft(self.x, a, b)[k - a][:, self.sel]
            sec = k * HOP // FS
            outer = X[:, :, :, None] * X[:, :, None, :].conj()
            np.add.at(self.qsum, sec, outer.astype(np.complex64))
            np.add.at(self.qn, sec, 1)

    def locate(self, a: int, b: int) -> tuple[float, float | None, float] | None:
        """Talker direction over frames a..b-1: (azimuth + = left, elevation or None, spread), None if no speech."""
        k = np.flatnonzero(self.speech[a:b]) + a
        if len(k) < 8:
            return None
        X = frames_fft(self.x, k[0], k[-1] + 1)[k - k[0]][:, self.sel]
        R = np.einsum("tfi,tfj->fij", X, X.conj()) / len(k)
        s0, s1 = max(0, k[0] * HOP // FS - 10), k[-1] * HOP // FS + 11
        n = self.qn[s0:s1].sum()
        if n >= 20:
            R = R - self.qsum[s0:s1].sum(axis=0) / n
        grid = np.zeros(len(self.E[0]))
        for (i, j), E in zip(self.pairs, self.E):
            r = R[:, i, j]
            grid += np.real(E @ (r / (np.abs(r) + 1e-12)).astype(np.complex64))
        nel = len(self.els)
        grid = grid.reshape(360, nel)
        g = int(np.argmax(grid))
        ka, ke = divmod(g, nel)
        elev = float(self.els[ke])
        if 0 < ke < nel - 1:
            y0, y1, y2 = grid[ka, ke - 1:ke + 2]
            den = y0 - 2 * y1 + y2
            if den < 0:
                elev += 0.5 * (y0 - y2) / den * float(self.els[1] - self.els[0])
        cut = grid[:, ke]
        y0, y1, y2 = cut[(ka - 1) % 360], cut[ka], cut[(ka + 1) % 360]
        den = y0 - 2 * y1 + y2
        az = ka - 180 + (0.5 * (y0 - y2) / den if den < 0 else 0.0)
        span = max(cut[ka] - float(np.median(cut)), 1e-9)
        above = cut >= cut[ka] - 0.3 * span
        lo = hi = ka
        while above[(lo - 1) % 360] and ka - lo < 179:
            lo -= 1
        while above[(hi + 1) % 360] and hi - ka < 179:
            hi += 1
        if self.planar and abs(az) > 90:
            az = math.copysign(180 - abs(az), az)
        return float(az), (elev if nel > 1 else None), float(min(60.0, max(2.0, (hi - lo) / 2)))


def talker_track(speech: np.ndarray, tl: TalkerLocator) -> tuple[list, np.ndarray]:
    """Direction of each stretch of speech (long ones in 3 s pieces, so a talker who moves is followed), and
    per frame the direction to aim the beam: the current stretch's, else the last one's."""
    nfr = len(speech)
    segs = []
    for a, b in runs(speech, join=50, shortest=30):
        for s in range(a, b, 375):
            e = min(b, s + 375)
            found = tl.locate(s, e) if e - s >= 30 else None
            if found:
                az, el, sig = found
                segs.append((s, e, az, el if el is not None else 0.0, sig))
    aim = np.zeros((nfr, 2))
    cur = (segs[0][2], segs[0][3]) if segs else (0.0, 0.0)
    j = 0
    for k in range(nfr):
        while j < len(segs) and segs[j][0] <= k:
            cur = (segs[j][2], segs[j][3])
            j += 1
        aim[k] = cur
    return segs, aim


# ------------------------------------------------------------------ the versions

class Wiener:
    """The live voice focus's noise filter (decision-directed Wiener gain, floor -18 dB)."""

    def __init__(self, nbins: int):
        self.Pn = np.full(nbins, 1e-3)
        self.g = np.ones(nbins)
        self.post = np.ones(nbins)

    def gain(self, Y: np.ndarray, quiet: bool) -> np.ndarray:
        P = np.abs(Y) ** 2
        if quiet:
            self.Pn = 0.98 * self.Pn + 0.02 * P
        post = P / (self.Pn + 1e-12)
        prio = 0.98 * self.g ** 2 * self.post + 0.02 * np.maximum(post - 1, 0)
        g = np.maximum(prio / (1 + prio), 0.12)
        self.g, self.post = g, post
        return g

    def __call__(self, Y: np.ndarray, quiet: bool) -> np.ndarray:
        return Y * self.gain(Y, quiet)


class DirectionFilter:
    """After the beam: how much of each frequency comes from the talker's direction, as a gain from 1 down
    to POST_FLOOR. The measure is how well the mics' phases agree once lined up on the talker (1 = all from
    there). It learns what the speech scores and what the background scores, and only cuts where the two
    clearly differ: not at low frequencies (the array is too small to tell directions apart there), and
    not at all when the talker is far or the room echoes, where speech lines up no better than noise."""

    def __init__(self, pos: np.ndarray, f: np.ndarray):
        self.f, self.n = f, len(pos)
        self.band = (f >= BAND[0]) & (f <= BAND[1])
        self.s_smooth = np.zeros(len(f))
        self.s_talker = self.s_noise = None

    def __call__(self, X: np.ndarray, tau: np.ndarray, speech: bool, quiet: bool) -> tuple[np.ndarray, float | None]:
        """(gain per bin, how much this frame comes from the talker: about 1 = all, 0 = no more than the
        background; None when the talker can't be told apart from the background)"""
        P = np.sum(np.abs(X) ** 2, axis=1)
        lined = np.sum(X * np.exp(2j * np.pi * self.f[:, None] * tau[None, :]), axis=1)
        s = np.abs(lined) ** 2 / (self.n * P + 1e-12)
        if self.s_talker is None:
            self.s_talker, self.s_noise = np.ones(len(s)), np.ones(len(s))
        span = self.s_talker - self.s_noise
        trust = np.clip((span - 0.1) / 0.2, 0.0, 1.0)
        w = P * self.band
        wt = w * trust
        score = float(np.sum(wt * (s - self.s_noise)) / np.sum(wt * span)) if np.sum(wt) > 0.2 * np.sum(w) else None
        s3 = np.convolve(s, (0.25, 0.5, 0.25), "same")
        self.s_smooth = 0.5 * self.s_smooth + 0.5 * s3
        g = np.clip((self.s_smooth - self.s_noise) / np.maximum(span, 0.05), POST_FLOOR, 1.0)
        if speech:
            self.s_talker = 0.99 * self.s_talker + 0.01 * s
        if quiet:
            self.s_noise = 0.99 * self.s_noise + 0.01 * s
        return 1 - trust * (1 - g), score


class Learner:
    """MVDR toward whatever the speech frames have in common, against the background of the quiet frames
    (mic_caption_view.StreamingFocus, any number of mics)."""

    def __init__(self, F: int, nch: int):
        self.nch = nch
        self.Rn = np.tile(np.eye(nch, dtype=complex) * 1e-6, (F, 1, 1))
        self.Rs = self.Rn.copy()
        self.n_noise = self.n_speech = self.frames = 0
        self.w = None

    def step(self, X: np.ndarray, quiet: bool, speech: bool) -> np.ndarray:
        self.frames += 1
        if quiet or speech:
            outer = X[:, :, None] * X[:, None, :].conj()
            if quiet:
                self.Rn = 0.99 * self.Rn + 0.01 * outer
                self.n_noise += 1
            else:
                self.Rs = 0.98 * self.Rs + 0.02 * outer
                self.n_speech += 1
        if self.frames % 32 == 0 and self.n_noise > 100 and self.n_speech > 30:
            self._update_weights()
        return np.sum(X * self.w.conj(), axis=1) if self.w is not None else X.mean(axis=1)

    def _update_weights(self):
        F, n = self.Rn.shape[0], self.nch
        tr = np.real(np.trace(self.Rn, axis1=1, axis2=2))
        Rn = self.Rn + np.eye(n)[None] * (1e-3 * tr / n + 1e-12)[:, None, None]
        vals, vecs = np.linalg.eig(np.linalg.solve(Rn, self.Rs))
        v = vecs[np.arange(F), :, np.argmax(vals.real, axis=1)]
        d = (Rn @ v[..., None])[..., 0]
        d = d / np.where(np.abs(d[:, :1]) > 1e-12, d[:, :1], 1)
        Rd = np.linalg.solve(Rn, d[..., None])[..., 0]
        self.w = Rd / (np.sum(d.conj() * Rd, axis=1, keepdims=True) + 1e-12)


class Versions:
    """The live voice focus (MVDR toward the talker + direction filter + Wiener, learning as it goes), the
    previous one, and the versions they're compared with, all cut from the same 8 ms frames."""

    def __init__(self, nch: int, ref: int, delays: callable, pos: np.ndarray, aimed: bool):
        self.nch, self.ref, self.delays, self.aimed = nch, ref, delays, aimed
        self.win = np.sqrt(np.hanning(NFFT + 1)[:NFFT])
        self.f = np.fft.rfftfreq(NFFT, 1 / FS)
        F = len(self.f)
        self.beam, self.old = Learner(F, nch), Learner(F, nch)
        self.direction = DirectionFilter(pos, self.f)
        self.focus_filter, self.old_filter, self.mic_filter = Wiener(F), Wiener(F), Wiener(F)
        self.learnt = 0

    def run(self, x: np.ndarray, det: dict, keep: bool = True, keys: list[str] = KEYS) -> dict:
        n = len(x)
        pad = np.concatenate([x, np.zeros((NFFT, self.nch))])
        nfr = min(len(det["speech"]), (len(pad) - NFFT) // HOP + 1)
        outs = {k: np.zeros(len(pad)) for k in keys} if keep else None
        low = self.f < 70
        w2 = self.win / 2  # sqrt-Hann in and out at 75% overlap adds up to 2
        for k in range(nfr):
            s = k * HOP
            X = np.fft.rfft(pad[s:s + NFFT] * self.win[:, None], axis=0)
            sp, qu = bool(det["speech"][k]), bool(det["quiet"][k])
            tau = self.delays(k)
            g_dir, score = self.direction(X, tau, sp, qu)
            if not self.aimed:
                g_dir, score = 1.0, None
            learn = sp and (score is None or score > ON_TARGET)
            self.learnt += learn
            Ym = self.beam.step(X, qu, learn)
            Ym[low] = 0
            Yf = Ym * np.maximum(self.focus_filter.gain(Ym, qu) * g_dir, TOTAL_FLOOR)
            Yo = self.old.step(X, bool(det["quiet_e"][k]), bool(det["loud"][k]))
            Yo[low] = 0
            Yo = self.old_filter(Yo, bool(det["quiet_e"][k]))
            Xr = X[:, self.ref].copy()
            Xr[low] = 0
            Yr = self.mic_filter(Xr, qu)
            if not keep:
                continue
            ph = np.exp(2j * np.pi * self.f[:, None] * tau[None, :])
            for key, Y in (("raw", X[:, self.ref]), ("avg", X.mean(axis=1)), ("das", np.mean(X * ph, axis=1)),
                           ("mvdr", Ym), ("filt", Yr), ("old", Yo), ("focus", Yf)):
                if key in outs:
                    outs[key][s:s + NFFT] += np.fft.irfft(Y, NFFT) * w2
        return {k: v[:n] for k, v in outs.items()} if keep else {}


def steering(geom: dict, chans: list[int], aim: np.ndarray) -> callable:
    """Per frame: each mic's arrival time (s) for a far sound from the frame's aim (azimuth, elevation)."""
    pos = np.array([geom[c][1:4] for c in chans]) / 1000.0
    cache: dict = {}

    def delays(k: int) -> np.ndarray:
        key = (round(float(aim[k, 0]), 1), round(float(aim[k, 1]), 1))
        if key not in cache:
            u = v360.unit_vectors(key[0], key[1])
            cache[key] = -(pos @ u) / v360.C
        return cache[key]
    return delays


# ------------------------------------------------------------------ measuring

def measure(y: np.ndarray, speech_m: np.ndarray, noise_m: np.ndarray, band) -> tuple[float, float]:
    """(background power, voice power) in a band: the quiet frames' mean, and the speech frames' mean minus it."""
    p = stft_power(y, band)[: len(speech_m)]
    noise = float(np.mean(p[noise_m])) if noise_m.any() else float("nan")
    voice = max(float(np.mean(p[speech_m])) - noise, 1e-3 * noise) if speech_m.any() else float("nan")
    return noise, voice


def db(p: float) -> float:
    return 10 * math.log10(p / (32768.0 ** 2) + 1e-20)


def report(outs: dict, speech: np.ndarray, quiet: np.ndarray, table: list[tuple] | None = None) -> dict:
    """Scales every version to the raw mic's voice loudness and measures what background is left.
    table: (key, (file, name, about)) per row; default the standard versions."""
    near_speech = np.convolve(speech.astype(float), np.ones(51), "same") > 0  # within 0.2 s of speech
    noise_m = quiet & ~near_speech
    speech_m = speech.copy()
    rows, gains = [], {}
    nr, vr = measure(outs["raw"], speech_m, noise_m, BAND)
    sub_raw = [measure(outs["raw"], speech_m, noise_m, (lo, hi)) for _, lo, hi in SUB_BANDS]
    full_raw = measure(outs["raw"], speech_m, noise_m, (70.0, 8000.0))[0]
    for key, (fname, name, about) in table or list(zip(KEYS, VERSIONS)):
        n, v = measure(outs[key], speech_m, noise_m, BAND)
        g2 = vr / v if v > 0 else 1.0
        gains[key] = math.sqrt(g2)
        snr = 10 * math.log10(v / n) if n > 0 else float("nan")
        bands = {}
        for (bname, lo, hi), (bn_raw, bv_raw) in zip(SUB_BANDS, sub_raw):
            bn, bv = measure(outs[key], speech_m, noise_m, (lo, hi))
            bands[bname] = round(10 * math.log10((bv / bn) / (bv_raw / bn_raw)), 1) if bn > 0 and bn_raw > 0 else None
        full = measure(outs[key], speech_m, noise_m, (70.0, 8000.0))[0] * g2
        echo = speech_srmr(outs[key], speech_m)
        rows.append({"file": fname, "name": name, "about": about, "srmr": round(echo, 2) if echo == echo else None,
                     "background_dbfs": round(db(n * g2), 1), "voice_dbfs": round(db(vr), 1),
                     "voice_to_noise_db": round(snr, 1),
                     "better_than_raw_db": round(snr - 10 * math.log10(vr / nr), 1),
                     "better_by_band_db": bands,
                     "background_drop_full_band_db": round(10 * math.log10(full_raw / full), 1) if full > 0 else None})
    return {"rows": rows, "gains": gains,
            "speech_s": round(float(speech_m.sum()) * HOP / FS, 1), "background_s": round(float(noise_m.sum()) * HOP / FS, 1),
            "speech_frames": runs(speech_m, 0, 1), "background_frames": runs(noise_m, 0, 1), "frame_s": HOP / FS}


def report_text(name: str, dur: float, rep: dict, segs: list, ref: int, geom: dict) -> str:
    lines = [f"{name}  {dur:.1f} s  ·  noise cancellation report",
             f"speech {rep['speech_s']:.1f} s, background measured over {rep['background_s']:.1f} s of quiet; "
             f"raw mic = ch{ref} ({geom[ref][0]})", ""]
    if rep["background_s"] < 1.0:
        lines += ["!! Less than a second of quiet between the speech: the background numbers are rough.",
                  "   Next time leave pauses, or stay quiet for the first couple of seconds.", ""]
    if rep["speech_s"] < 1.0:
        lines += ["!! Hardly any speech found: the voice numbers are rough.", ""]
    lines.append(f"{'version':<26}{'background':>12}{'voice':>9}{'voice/noise':>13}{'vs raw mic':>12}{'echo (SRMR)':>13}"
                 "   what it is")
    for r in rep["rows"]:
        better = "" if r["file"] == "raw_mic.wav" else f"{r['better_than_raw_db']:+.1f} dB"
        echo = "" if r.get("srmr") is None else f"{r['srmr']:.2f}"
        lines.append(f"{r['name']:<26}{r['background_dbfs']:>9.1f} dB{r['voice_dbfs']:>6.1f} dB"
                     f"{r['voice_to_noise_db']:>10.1f} dB{better:>12}{echo:>13}   {r['about']}")
    lines += ["", "levels in dBFS, 300-3400 Hz (the voice band), every version at the raw mic's voice loudness;",
              "'vs raw mic' = how much more the voice stands out from the background (higher is better)",
              "'echo (SRMR)' = speech-to-reverberation modulation ratio over the speech: how much of the voice's",
              "loudness changes at speech's own rhythm (4-20 Hz) versus faster ones that echo smears in; higher = less echo", "",
              f"{'by band, vs raw mic':<26}" + "".join(f"{b:>13}" for b, _, _ in SUB_BANDS) + f"{'all (bkg)':>13}"]
    for r in rep["rows"][1:]:
        cells = "".join(f"{'' if v is None else f'{v:+.1f} dB':>13}" for v in r["better_by_band_db"].values())
        full = r["background_drop_full_band_db"]
        lines.append(f"{r['name']:<26}{cells}{'' if full is None else f'{full:+.1f} dB':>13}")
    lines += ["", "'all (bkg)' = background removed over the whole range (rumble and hiss too)", ""]
    if segs:
        lines.append("talker directions (beam aim):")
        for s, e, az, el, sig in segs[:40]:
            lines.append(f"  {sd.clock(s * HOP / FS)}–{sd.clock(e * HOP / FS)}  {abs(az):.0f}° "
                         f"{'left' if az > 0 else 'right'}, {el:+.0f}° up  (±{sig:.0f}°)")
        if len(segs) > 40:
            lines.append(f"  … {len(segs) - 40} more")
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------------ spectrogram picture

def spectrogram(y: np.ndarray, cols: int) -> np.ndarray:
    """(129 bins up to 8 kHz, cols) dB, columns averaged over time."""
    n = 256
    win = np.hanning(n)
    nfr = max(1, (len(y) - n) // 128 + 1)
    idx = (np.arange(nfr) * 128)[:, None] + np.arange(n)[None, :]
    S = np.abs(np.fft.rfft(np.pad(y, (0, n))[idx] * win, axis=1)) ** 2
    edges = np.linspace(0, nfr, cols + 1).astype(int)
    out = np.stack([S[a:max(b, a + 1)].mean(axis=0) for a, b in zip(edges[:-1], edges[1:])], axis=1)
    return 10 * np.log10(out / (32768.0 ** 2) + 1e-12)


MAGMA = np.array([[0, 0, 4], [40, 11, 84], [101, 21, 110], [159, 42, 99], [212, 72, 66], [245, 125, 21],
                  [250, 193, 39], [252, 255, 164]], float)


def colour(dbv: np.ndarray, lo: float, hi: float) -> np.ndarray:
    t = np.clip((dbv - lo) / (hi - lo), 0, 1) * (len(MAGMA) - 1)
    i = np.minimum(t.astype(int), len(MAGMA) - 2)
    f = (t - i)[..., None]
    return (MAGMA[i] * (1 - f) + MAGMA[i + 1] * f).astype(np.uint8)


def spectrogram_png(path: Path, raw: np.ndarray, focus: np.ndarray, dur: float, rep: dict):
    cols = int(min(1800, max(600, dur * 40)))
    a, b = spectrogram(raw, cols), spectrogram(focus, cols)
    hi = float(np.percentile(a, 99.5))
    lo = hi - 70
    h, top, gap, left = 129 * 2, 30, 34, 70
    W, H = left + cols + 20, top + 2 * h + gap + 40
    img = QImage(W, H, QImage.Format.Format_RGB888)
    img.fill(QColor("#0d1117"))
    p = QPainter(img)
    f = QFont()
    f.setPixelSize(14)
    p.setFont(f)
    for k, (S, label) in enumerate(((a, "Raw mic"), (b, "Voice focus (same voice loudness)"))):
        rgb = np.ascontiguousarray(np.repeat(colour(S[::-1], lo, hi), 2, axis=0))
        q = QImage(rgb.data, cols, h, cols * 3, QImage.Format.Format_RGB888)
        y0 = top + k * (h + gap)
        p.drawImage(left, y0, q)
        p.setPen(QColor("#e6edf3"))
        p.drawText(left, y0 - 8, label)
        p.setPen(QColor("#8b949e"))
        for khz in (0, 2, 4, 6, 8):
            y = y0 + h - khz / 8 * h
            p.drawText(8, int(y) + 5, f"{khz} kHz")
    p.setPen(QColor("#8b949e"))
    for s in range(0, int(dur) + 1, max(1, int(dur / 10))):
        x = left + s / dur * cols
        p.drawText(int(x) - 14, H - 14, sd.clock(s)[:-2])
    vf = next(r for r in rep["rows"] if r["file"] == "voice_focus.wav")
    p.setPen(QColor("#e6edf3"))
    p.drawText(left + 260, top - 8, f"voice focus: voice stands out {vf['better_than_raw_db']:+.1f} dB more than on the raw mic")
    p.end()
    img.save(str(path))


# ------------------------------------------------------------------ one take

def resolve_model(name: str) -> str:
    if name != "auto":
        return name
    model, why = auto_transcriber()
    print(f"  speech-to-text: {model} (auto: {why})", flush=True)
    return model


def side(az) -> str:
    return "" if az is None else ("straight ahead" if abs(az) < 3 else f"{abs(az):.0f}° {'left' if az > 0 else 'right'}")


def write_transcript(out: Path, header: str, caps: list, model: str | None):
    """transcript.txt, subtitles.srt and captions.json from (start, end, text, az, el, words) lines."""
    with open(out / "transcript.txt", "w") as f:
        f.write(f"{header}\n" + (f"speech-to-text: Whisper {model}\n" if model else "") + "\n")
        for s, e, txt, az, el, _ in caps:
            f.write(f"{sd.clock(s)}–{sd.clock(e)}  {side(az)}{'' if el is None else f', {el:+.0f}° up'}  {txt}\n")
    with open(out / "subtitles.srt", "w") as f:
        for i, (s, e, txt, *_) in enumerate(caps, 1):
            f.write(f"{i}\n{sd.clock(s, True)} --> {sd.clock(e + 0.3, True)}\n{txt}\n\n")
    with open(out / "captions.json", "w") as f:
        json.dump({"model": model, "captions": [[s, e, txt, az, el, [list(w) for w in words]]
                                                for s, e, txt, az, el, words in caps]}, f)
    print(f"  transcript: {len(caps)} lines", flush=True)


def retranscribe(out: Path, model: str):
    """Only the transcript again, from a processed take's voice_focus.wav, with another speech model; each
    line's direction comes from the talker track saved in noise_report.json."""
    t0 = time.time()
    rep = json.loads((out / "noise_report.json").read_text())
    with wave.open(str(out / rep.get("transcript_from", "voice_focus.wav")), "rb") as w:
        x = np.frombuffer(w.readframes(w.getnframes()), "<i2").astype(np.float64)
    model = resolve_model(model)
    fs = rep["frame_s"]
    aim = [(s * fs, e * fs, az, el) for s, e, az, el, _ in rep.get("aim", [])]
    caps = []
    for start, end, text_, words in sd.transcribe(x, model, words=True, progress=True):
        over = [(min(end, e) - max(start, s), az, el) for s, e, az, el in aim if min(end, e) > max(start, s)]
        az = el = None
        if over:
            wsum = sum(o for o, _, _ in over)
            az = sum(o * a for o, a, _ in over) / wsum
            els = [(o, e) for o, _, e in over if e is not None]
            el = sum(o * e for o, e in els) / sum(o for o, _ in els) if els else None
        caps.append((start, end, text_, az, el, words))
    write_transcript(out, f"{rep.get('take', out.name)}  {rep.get('duration_s', len(x) / FS):.1f} s", caps, model)
    print(f"  saved in {out}  ({time.time() - t0:.0f} s)", flush=True)


def process(path: Path, out_root: Path, args):
    t0 = time.time()
    audio, info, frames, layout = read_take(path)
    geom = v360.load_geometry(None, layout if layout in v360.LAYOUTS else "vest6")
    name = path.stem if path.suffix.lower() == ".wav" else path.name
    dur = len(audio) / FS
    out = out_root / name
    out.mkdir(parents=True, exist_ok=True)
    chans = [c for c in live_channels(audio) if c in geom]
    dead = sorted(set(geom) - set(chans))
    print(f"{name}: {dur:.1f} s, {len(frames)} camera frames"
          + (f", {info['repaired']} unfinished WAV header(s) repaired" if info.get("repaired") else "")
          + (f", {info['lost_audio_frames']} audio frames lost on the card" if info.get("lost_audio_frames", "0") != "0" else "")
          + (f"\n  !! no sound on {', '.join(f'ch{c} ({geom[c][0]})' for c in dead)}: left out" if dead else ""), flush=True)
    if len(chans) < 2:
        raise SystemExit(f"{name}: fewer than two working mics")
    write_wav(out / "raw_6mic.wav", audio)

    loc = Locator(geom, chans)
    dets = loc.stream(audio, args.threshold)
    with open(out / "directions.csv", "w") as f:
        f.write("time_s,azimuth_deg,elevation_deg,uncertainty_deg,loudness_db\n")
        for t, az, el, sig, strength in dets:
            f.write(f"{t:.3f},{az:.1f},{'' if el is None else f'{el:.1f}'},{sig:.1f},{strength:.1f}\n")
    print(f"  directions: {len(dets)} sounds located", flush=True)

    x = audio[:, chans].astype(np.float64)
    x -= x.mean(axis=0)
    det = detect(x)
    # measured with the loudness detector, as before, so old and new reports compare
    speech, quiet = det["loud"], det["quiet_e"]
    tl = TalkerLocator(x, geom, chans, det["speech"], det["quiet"])
    segs, aim = talker_track(det["speech"], tl)
    print(f"  speech in {det['speech'].mean() * 100:.0f}% of the take ({speech.mean() * 100:.0f}% loud), "
          f"bangs/taps held {det['hold'].mean() * 100:.1f}%, {len(segs)} stretches located", flush=True)
    if args.ref in chans:
        ref = args.ref
    else:
        # the baseline is the mic with the middle background level, so a hissy or lucky mic doesn't skew the gains
        bg = [float(np.mean(stft_power(x[:, i], BAND)[quiet])) if quiet.any() else 0.0 for i in range(len(chans))]
        ref = chans[int(np.argsort(bg)[(len(bg) - 1) // 2])]

    pos = np.array([geom[c][1:4] for c in chans]) / 1000.0
    ver = Versions(len(chans), chans.index(ref), steering(geom, chans, aim), pos, aimed=bool(segs))
    warm = min(len(x), 20 * FS)  # a first pass learns the background and the talker, so the start is cleaned too
    ver.run(x[:warm], det, keep=False)
    ver.learnt = 0
    outs = ver.run(x, det)
    print(f"  the beam learnt the talker from {ver.learnt / max(1, det['speech'].sum()) * 100:.0f}% of the speech", flush=True)
    table = list(zip(KEYS, VERSIONS))
    listen = "focus"
    if args.dereverb:
        t1 = time.time()
        xw = dereverb(x, args.taps, args.delay)
        print(f"  echo removal (WPE, {args.taps * HOP / FS * 1000:.0f} ms of echo, first "
              f"{args.delay * HOP / FS * 1000:.0f} ms kept): {time.time() - t1:.0f} s", flush=True)
        ver_w = Versions(len(chans), chans.index(ref), steering(geom, chans, aim), pos, aimed=bool(segs))
        ver_w.run(xw[:warm], det, keep=False)
        outs_w = ver_w.run(xw, det, keys=[k for k, *_ in ECHO_VERSIONS])
        for k, row_key, *row in ECHO_VERSIONS:
            outs[row_key] = outs_w[k]
            table.append((row_key, tuple(row)))
        listen = "focus_wpe"
        del xw, outs_w, ver_w
    rep = report(outs, speech, quiet, table)
    peak = max(float(np.max(np.abs(outs[k] * rep["gains"][k]))) for k, _ in table) or 1.0
    scale = min(4.0, 30000 / peak)
    for key, (fname, _, _) in table:
        write_wav(out / fname, outs[key] * rep["gains"][key] * scale)
    transcript_from = dict(table)[listen][0]
    rep.update({"take": name, "duration_s": round(dur, 2), "ref_channel": ref, "channels": chans,
                "layout": layout, "aim": [[s, e, az, el, sig] for s, e, az, el, sig in segs],
                "transcript_from": transcript_from,
                "dereverb": {"method": "WPE", "taps": args.taps, "delay": args.delay} if args.dereverb else None})
    (out / "noise_report.json").write_text(json.dumps(rep))
    text = report_text(name, dur, rep, segs, ref, geom)
    (out / "noise_report.txt").write_text(text)
    shown = text.splitlines()
    shown = shown[next(i for i, ln in enumerate(shown) if ln.startswith("version")):]
    print("\n".join("  " + ln for ln in shown[:shown.index("")]), flush=True)
    spectrogram_png(out / "spectrogram.png", outs["raw"] * rep["gains"]["raw"], outs["focus"] * rep["gains"]["focus"], dur, rep)

    clean = outs[listen] * rep["gains"][listen] * scale
    del outs, ver, x, audio  # free the take before the speech model loads
    caps = []
    model = resolve_model(args.model)
    if not args.no_transcript:
        print(f"  transcript from {transcript_from}", flush=True)
        for start, end, text_, words in sd.transcribe(clean, model, words=True, progress=True):
            found = tl.locate(int(start * FS / HOP), int(end * FS / HOP) + 1)
            az, el, _ = found if found else (None, None, None)
            caps.append((start, end, text_, az, el, words))
    write_transcript(out, f"{name}  {dur:.1f} s  {info.get('mode', '')}", caps, None if args.no_transcript else model)
    if not path.suffix and (path / "video.bin").exists():
        shutil.copyfile(path / "video.bin", out / "camera.bin")
    print(f"  saved in {out}  ({time.time() - t0:.0f} s)", flush=True)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="the card (or its takes/ folder), one take folder, or a 6-channel .wav")
    ap.add_argument("--take", type=int, help="only this take number")
    ap.add_argument("--out", type=Path, default=OUT_ROOT, help=f"where to write (default {OUT_ROOT})")
    ap.add_argument("--model", default=DEFAULT_MODEL,
                    help=f"speech-to-text model (default {DEFAULT_MODEL}): auto (the biggest this computer runs "
                         "well), " + ", ".join(m for m, _, _ in TRANSCRIBERS))
    ap.add_argument("--dereverb", action="store_true",
                    help="also remove the room's echo (WPE) before voice focus: adds 'Raw mic, echo removed' and "
                         "'Voice focus, echo removed', and the transcript is made from the latter")
    ap.add_argument("--taps", type=int, default=DEREVERB_TAPS, help="echo removal: 8 ms frames of echo predicted")
    ap.add_argument("--delay", type=int, default=DEREVERB_DELAY, help="echo removal: 8 ms frames kept untouched")
    ap.add_argument("--low-priority", action="store_true", help="run at low CPU priority (the computer stays responsive)")
    ap.add_argument("--transcribe-only", action="store_true",
                    help="path is an already processed take folder: only redo its transcript (e.g. with a bigger --model)")
    ap.add_argument("--ref", type=int, default=None,
                    help="mic for raw_mic.wav and mic_filter.wav (default: the mic with the typical background level)")
    ap.add_argument("--threshold", type=float, default=12.0, help="direction trigger, dB over quiet (default 12)")
    ap.add_argument("--no-transcript", action="store_true", help="skip Whisper")
    args = ap.parse_args()
    if args.low_priority and hasattr(os, "nice"):
        os.nice(10)
    if args.transcribe_only:
        retranscribe(Path(args.path).expanduser(), args.model)
        return
    takes = find_takes(Path(args.path).expanduser(), args.take)
    if not takes:
        raise SystemExit(f"no 6-mic takes found in {args.path}")
    app = QGuiApplication.instance() or QGuiApplication([])  # noqa: F841  (QImage text needs it)
    for t in takes:
        process(t, args.out, args)


if __name__ == "__main__":
    main()
