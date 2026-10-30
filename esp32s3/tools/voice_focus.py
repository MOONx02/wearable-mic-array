"""Clean a 4-mic recording down to the voice: cancel steady background noise and mic hiss.

  ~/.mic-live-view-env/bin/python tools/voice_focus.py mic_takes/cam4_XXXX.wav

Writes next to the input:
  *_voice.wav    the cleaned voice (mono)
  *_mic.wav      one raw mic at the same loudness, for a before/after comparison
  *_voice.png    spectrograms before and after (if PySide6 is installed)

How it works (no mic positions needed, it learns them from the recording):
  1. Finds the voice and the gaps between words by loudness in the 300-3400 Hz band.
  2. Learns what the background sounds like at the four mics from the gaps, and where the
     voice comes from during speech, then combines the four mics with weights that keep the
     voice and cancel the background (an MVDR beamformer). This removes steady sources from
     one direction (a fan, a vent, a computer) best.
  3. Removes what is left, mostly the mics' own hiss, with a smoothed spectral (Wiener) filter.
  4. Turns the gaps between words down further (--gate-db, 0 to switch off).
Works best when the talker and the noise source stay put during the recording.
"""

from __future__ import annotations

import argparse
import math
import os
import wave
from pathlib import Path

import numpy as np

FS = 16000
NFFT = 512
HOP = 128
VOICE_BAND = (300, 3400)


def stft(x: np.ndarray, win: np.ndarray) -> np.ndarray:
    """x (samples, ch) -> (bins, frames, ch)."""
    pad = np.pad(x, ((NFFT, NFFT), (0, 0)))
    frames = (len(pad) - NFFT) // HOP + 1
    idx = np.arange(NFFT)[None, :] + HOP * np.arange(frames)[:, None]
    seg = pad[idx] * win[None, :, None]
    return np.fft.rfft(seg, axis=1).transpose(1, 0, 2)


def istft(Y: np.ndarray, win: np.ndarray, length: int) -> np.ndarray:
    frames = Y.shape[1]
    seg = np.fft.irfft(Y.T, NFFT, axis=1) * win[None, :]
    out = np.zeros(NFFT + HOP * (frames - 1))
    norm = np.zeros_like(out)
    for t in range(frames):
        out[t * HOP:t * HOP + NFFT] += seg[t]
        norm[t * HOP:t * HOP + NFFT] += win ** 2
    out /= np.maximum(norm, 1e-8)
    return out[NFFT:NFFT + length]


def dilate(mask: np.ndarray, k: int) -> np.ndarray:
    return np.convolve(mask.astype(float), np.ones(2 * k + 1), "same") > 0


def find_voice(X: np.ndarray, f: np.ndarray):
    band = (f >= VOICE_BAND[0]) & (f <= VOICE_BAND[1])
    e = 10 * np.log10(np.mean(np.abs(X[band]) ** 2, axis=(0, 2)) + 1e-12)
    floor = np.percentile(e, 20)
    speech = e > floor + 8
    noise = (e < floor + 3) & ~dilate(speech, 12)  # keep ~100 ms away from speech
    return speech, noise, floor


def mvdr(X: np.ndarray, speech: np.ndarray, noise: np.ndarray) -> np.ndarray:
    F, _, M = X.shape
    Y = np.empty(X.shape[:2], complex)
    eye = np.eye(M)
    for k in range(F):
        Xn, Xs = X[k, noise], X[k, speech]
        Rn = Xn.T @ Xn.conj() / max(len(Xn), 1)
        Rs = Xs.T @ Xs.conj() / max(len(Xs), 1)
        Rn += eye * (1e-3 * np.real(np.trace(Rn)) / M + 1e-9)
        # voice direction: the strongest component of speech relative to the background
        vals, vecs = np.linalg.eig(np.linalg.solve(Rn, Rs))
        d = Rn @ vecs[:, np.argmax(vals.real)]
        d = d / d[0] if abs(d[0]) > 1e-12 else d
        Rinv_d = np.linalg.solve(Rn, d)
        w = Rinv_d / (d.conj() @ Rinv_d)
        Y[k] = X[k] @ w.conj()
    return Y


def wiener(Y: np.ndarray, noise: np.ndarray, gmin_db: float) -> np.ndarray:
    Pn = np.mean(np.abs(Y[:, noise]) ** 2, axis=1) + 1e-12
    gmin = 10 ** (gmin_db / 20)
    G = np.empty(Y.shape)
    prev = np.ones(Y.shape[0])
    post_prev = np.ones(Y.shape[0])
    for t in range(Y.shape[1]):
        post = np.abs(Y[:, t]) ** 2 / Pn
        prio = 0.98 * prev ** 2 * post_prev + 0.02 * np.maximum(post - 1, 0)
        g = np.maximum(prio / (1 + prio), gmin)
        G[:, t] = g
        prev, post_prev = g, post
    return Y * G


def gate(Y: np.ndarray, speech: np.ndarray, gate_db: float) -> np.ndarray:
    if gate_db <= 0:
        return Y
    keep = dilate(speech, 25).astype(float)  # hold ~200 ms around speech
    smooth = np.convolve(keep, np.hanning(15) / np.hanning(15).sum(), "same")
    g = 10 ** (-gate_db / 20) + (1 - 10 ** (-gate_db / 20)) * smooth
    return Y * g[None, :]


def level_db(x: np.ndarray, mask_samples: np.ndarray) -> float:
    return 10 * math.log10(np.mean(x[mask_samples] ** 2) + 1e-12)


def write_wav(path: Path, x: np.ndarray):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(np.clip(x, -32767, 32767).astype("<i2").tobytes())


def spectrogram_png(path: Path, before: np.ndarray, after: np.ndarray, title: str):
    try:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtCore import QRectF, Qt
        from PySide6.QtGui import QColor, QFont, QGuiApplication, QImage, QPainter
    except ImportError:
        return False
    app = QGuiApplication.instance() or QGuiApplication([])  # noqa: F841
    cmap = np.array([[0, 0, 4], [40, 11, 84], [101, 21, 110], [159, 42, 99], [212, 72, 66],
                     [245, 125, 21], [250, 193, 39], [252, 255, 164]], float)

    def image(x):
        win = np.hanning(NFFT)
        S = np.abs(stft(x[:, None], win)[:, :, 0]) ** 2
        db = 10 * np.log10(S / (win.sum() * 32768 / 2) ** 2 + 1e-14)
        v = np.clip((db + 110) / 80, 0, 1)[::-1] * (len(cmap) - 1)
        k = np.minimum(v.astype(int), len(cmap) - 2)
        rgb = cmap[k] + (cmap[k + 1] - cmap[k]) * (v - k)[..., None]
        a = (0xFF000000 | (rgb[..., 0].astype(np.uint32) << 16) | (rgb[..., 1].astype(np.uint32) << 8)
             | rgb[..., 2].astype(np.uint32)).astype(np.uint32)
        a = np.ascontiguousarray(a)
        return QImage(a.data, a.shape[1], a.shape[0], a.shape[1] * 4, QImage.Format_RGB32).copy()

    W, H, top = 1400, 820, 50
    out = QImage(W, H, QImage.Format_RGB32)
    out.fill(QColor("#0d1117"))
    p = QPainter(out)
    p.setRenderHint(QPainter.SmoothPixmapTransform)
    f = QFont("Helvetica")
    f.setPixelSize(20)
    f.setBold(True)
    p.setFont(f)
    p.setPen(QColor("#e6edf3"))
    p.drawText(QRectF(20, 10, W - 40, 30), Qt.AlignLeft, title)
    panel_h = (H - top - 40) / 2
    for i, (name, x) in enumerate((("Before: one raw mic", before), ("After: voice focus", after))):
        y0 = top + i * (panel_h + 20)
        r = QRectF(70, y0 + 26, W - 90, panel_h - 26)
        p.drawImage(r, image(x))
        f.setPixelSize(16)
        p.setFont(f)
        p.setPen(QColor("#e6edf3"))
        p.drawText(QRectF(70, y0, 600, 24), Qt.AlignLeft, name)
        f.setPixelSize(12)
        f.setBold(False)
        p.setFont(f)
        p.setPen(QColor("#8b949e"))
        for khz in range(0, 9, 2):
            yy = r.bottom() - khz / 8 * r.height()
            p.drawText(QRectF(10, yy - 8, 55, 16), Qt.AlignRight, f"{khz} kHz")
        f.setBold(True)
    p.end()
    return out.save(str(path))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wav", help="4-channel 16 kHz recording (from Record in the viewers)")
    ap.add_argument("--ref", type=int, default=1, help="mic used for the 'before' file (default ch1)")
    ap.add_argument("--hiss-db", type=float, default=-18, help="how far the hiss filter may turn a band down")
    ap.add_argument("--gate-db", type=float, default=12, help="extra turn-down between words (0 = off)")
    args = ap.parse_args()

    src = Path(args.wav)
    with wave.open(str(src)) as w:
        if w.getframerate() != FS or w.getnchannels() != 4:
            raise SystemExit("expected a 4-channel 16 kHz WAV")
        x = np.frombuffer(w.readframes(w.getnframes()), "<i2").reshape(-1, 4).astype(np.float64)
    x -= x.mean(axis=0)
    n = len(x)
    win = np.sqrt(np.hanning(NFFT + 1)[:NFFT])
    f = np.fft.rfftfreq(NFFT, 1 / FS)
    X = stft(x, win)
    speech, noise, _ = find_voice(X, f)
    if speech.sum() < 20 or noise.sum() < 20:
        raise SystemExit("couldn't find both speech and quiet gaps; the recording needs some of each")

    Y = mvdr(X, speech, noise)
    Y[f < 70] = 0
    beam = istft(Y.copy(), win, n)
    Y = wiener(Y, noise, args.hiss_db)
    Y = gate(Y, speech, args.gate_db)
    out = istft(Y, win, n)

    def samples(mask):
        m = np.zeros(n, bool)
        for t in np.where(mask)[0]:
            a = max(0, (t - NFFT // HOP) * HOP)
            m[a:a + HOP] = True
        return m

    sm, nm = samples(speech), samples(noise)
    ref = x[:, args.ref]
    rows = [("one raw mic", ref), ("four mics aimed at the voice", beam), ("+ hiss filter and gaps turned down", out)]
    print(f"{src.name}: {n / FS:.1f} s, voice found in {speech.mean() * 100:.0f}% of it")
    print(f"{'':36s} voice    background   voice-to-background")
    base = None
    for name, sig in rows:
        v, b = level_db(sig, sm), level_db(sig, nm)
        base = v - b if base is None else base
        print(f"{name:36s} {v - 90.3:6.1f}   {b - 90.3:6.1f} dBFS   {v - b:5.1f} dB ({v - b - base:+.1f})")

    gain = math.sqrt(np.mean(ref[sm] ** 2) / (np.mean(out[sm] ** 2) + 1e-12))
    out *= gain
    peak = max(np.max(np.abs(out)), np.max(np.abs(ref)), 1.0)
    scale = min(4.0, 30000 / peak)  # same loudness for both, up to +12 dB so it's easy to hear
    stem = src.with_suffix("")
    write_wav(Path(f"{stem}_voice.wav"), out * scale)
    write_wav(Path(f"{stem}_mic.wav"), ref * scale)
    print(f"wrote {stem.name}_voice.wav and {stem.name}_mic.wav")
    if spectrogram_png(Path(f"{stem}_voice.png"), ref, out, f"{src.name}: before and after voice focus"):
        print(f"wrote {stem.name}_voice.png")


if __name__ == "__main__":
    main()
