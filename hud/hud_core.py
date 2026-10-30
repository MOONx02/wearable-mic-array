"""Direction finding, beamforming and beam tracking for a wearable mic array.

Pure numpy/scipy, so the same code runs on simulated or real recordings.
Angles are azimuths in degrees in the wearer's frame: 0 = straight ahead,
positive = wearer's left, +-180 = behind.
"""

from __future__ import annotations

import json
import wave
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.signal import istft, stft

C = 343.0
NFFT = 512
HOP = 160  # 10 ms at 16 kHz, so a HUD frame is a whole number of STFT frames
BLOCK_S = 0.1  # one HUD frame
BAND_HZ = (300.0, 5000.0)
GRID = np.arange(-180.0, 180.0, 2.5)
ACTIVE_FLOOR = 0.015  # share of above-noise bins that steady noise alone produces


@dataclass
class Geometry:
    names: list[str]
    pos: np.ndarray  # (mics, 3), metres
    fs: int
    ref: int

    @classmethod
    def load(cls, path: Path) -> "Geometry":
        g = json.loads(Path(path).read_text())
        names = [m["name"] for m in g["mics"]]
        return cls(names, np.array([m["pos_m"] for m in g["mics"]], float), int(g["fs"]),
                   names.index(g.get("reference_mic", names[0])))


def read_wav(path: Path) -> tuple[np.ndarray, int]:
    with wave.open(str(path)) as w:
        if w.getsampwidth() != 2:
            raise ValueError("expected 16-bit WAV")
        data = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)
        return data.reshape(-1, w.getnchannels()).T / 32768.0, w.getframerate()


def write_wav(path: Path, data: np.ndarray, fs: int) -> None:
    data = np.atleast_2d(data)
    pcm = np.clip(np.round(data.T * 32767), -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(pcm.shape[1])
        w.setsampwidth(2)
        w.setframerate(fs)
        w.writeframes(pcm.tobytes())


def unit(az_deg: np.ndarray) -> np.ndarray:
    a = np.radians(np.asarray(az_deg, float))
    return np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=-1)


def steering(geom: Geometry, freqs: np.ndarray, az_deg: np.ndarray) -> np.ndarray:
    """Far-field steering vectors as received, shape (angles, freqs, mics)."""
    lead = unit(az_deg) @ geom.pos.T / C  # (angles, mics): how much earlier each mic hears it
    return np.exp(2j * np.pi * freqs[None, :, None] * lead[:, None, :])


def wrap(az):
    return (np.asarray(az, float) + 180.0) % 360.0 - 180.0


@dataclass
class Analysis:
    geom: Geometry
    x: np.ndarray  # (mics, samples)
    freqs: np.ndarray = field(init=False)
    X: np.ndarray = field(init=False)  # (mics, freqs, frames)
    srp: np.ndarray = field(init=False)  # (blocks, angles): sounds above the steady noise, 0..1
    activity: np.ndarray = field(init=False)  # (blocks,): share of bins above the noise floor
    steady: np.ndarray = field(init=False)  # (angles,): where the steady noise comes from, 0..1
    level_db: np.ndarray = field(init=False)  # reference mic, dBFS per block
    n_blocks: int = field(init=False)
    frames_per_block: int = field(init=False)

    def __post_init__(self):
        fs = self.geom.fs
        self.freqs, _, self.X = stft(self.x, fs, nperseg=NFFT, noverlap=NFFT - HOP)
        self.frames_per_block = int(round(BLOCK_S * fs / HOP))
        self.n_blocks = self.X.shape[2] // self.frames_per_block
        self._srp()
        ref = self.x[self.geom.ref]
        n = int(BLOCK_S * fs)
        blocks = ref[: self.n_blocks * n].reshape(self.n_blocks, n)
        self.level_db = 10 * np.log10(np.mean(blocks ** 2, axis=1) + 1e-12)

    @property
    def duration(self) -> float:
        return self.n_blocks * BLOCK_S

    def _srp(self, rise_db: float = 6.0):
        """SRP-PHAT (steer to every angle, measure power of phase-only signals).

        Two maps: one from all bins (dominated by steady fans), and one from only
        the time-frequency bins that rise rise_db above each frequency's noise
        floor, which is where talkers and sudden events live.
        """
        band = (self.freqs >= BAND_HZ[0]) & (self.freqs <= BAND_HZ[1])
        A = steering(self.geom, self.freqs[band], GRID)  # (angles, f, m)
        Xb = self.X[:, band, :]
        Xn = Xb / (np.abs(Xb) + 1e-12)
        pwr = np.abs(Xb[self.geom.ref]) ** 2
        # Noise power in one bin is roughly exponential, whose 20th percentile is
        # 0.223 x its mean; the percentile stays valid while sound is on 80% of the time.
        noise_mean = np.percentile(pwr, 20, axis=1, keepdims=True) / 0.223
        mask = pwr > noise_mean * 10 ** (rise_db / 10)  # (f, t)
        fpb = self.frames_per_block
        fg = np.zeros((self.n_blocks, len(GRID)))
        allmap = np.zeros((self.n_blocks, len(GRID)))
        self.activity = np.zeros(self.n_blocks)
        for b in range(self.n_blocks):
            sl = slice(b * fpb, (b + 1) * fpb)
            p = np.abs(np.einsum("afm,mft->aft", A.conj(), Xn[:, :, sl])) ** 2
            allmap[b] = p.mean(axis=(1, 2))
            m = mask[:, sl]
            self.activity[b] = m.mean()
            if m.sum() >= 10:
                fg[b] = (p * m).sum(axis=(1, 2)) / m.sum()
        steady = np.median(allmap, axis=0)
        self.steady = (steady - steady.min()) / (np.ptp(steady) + 1e-12)
        lo = fg.min(axis=1, keepdims=True)
        span = np.ptp(fg, axis=1, keepdims=True)
        self.srp = np.where(span > 0, (fg - lo) / (span + 1e-12), 0.0)
        self.srp *= np.clip((self.activity - 0.7 * ACTIVE_FLOOR) / ACTIVE_FLOOR, 0, 1)[:, None]

    def peaks(self, b: int, min_height=0.55, min_sep=20.0, max_peaks=4) -> list[tuple[float, float]]:
        """Local maxima of the normalized map for block b: [(angle, height), ...]."""
        p = self.srp[b]
        cand = [(GRID[i], p[i]) for i in range(len(p))
                if p[i] >= min_height and p[i] >= p[i - 1] and p[i] >= p[(i + 1) % len(p)]]
        cand.sort(key=lambda t: -t[1])
        kept: list[tuple[float, float]] = []
        for a, h in cand:
            if all(abs(wrap(a - k)) >= min_sep for k, _ in kept):
                kept.append((a, h))
            if len(kept) == max_peaks:
                break
        return kept


def beam_weights(geom: Geometry, freqs: np.ndarray, kind: str = "mvdr", loading: float = 0.05) -> np.ndarray:
    """Weights for every grid angle, shape (angles, freqs, mics).

    das:  delay-and-sum.
    mvdr: superdirective MVDR against a diffuse (spread-out) noise field,
          with diagonal loading so it doesn't amplify mic self-noise.
    """
    D = steering(geom, freqs, GRID)
    M = len(geom.names)
    if kind == "das":
        return D / M
    dist = np.linalg.norm(geom.pos[:, None, :] - geom.pos[None, :, :], axis=-1)
    W = np.empty_like(D)
    for k, f in enumerate(freqs):
        gamma = np.sinc(2 * f * dist / C) + loading * np.eye(M)
        gi = np.linalg.inv(gamma)
        d = D[:, k, :]  # (angles, m)
        num = d @ gi.T
        den = np.einsum("am,am->a", d.conj(), num)
        W[:, k, :] = num / den[:, None]
    return W


def beamform(an: Analysis, track_deg: np.ndarray, kind: str = "mvdr", X: np.ndarray | None = None) -> np.ndarray:
    """Steer along track_deg (one angle per HUD block) and return the time signal.

    Pass X to beamform a different set of signals (e.g. one simulated source)
    with exactly the same weights.
    """
    X = an.X if X is None else X
    W = beam_weights(an.geom, an.freqs, kind)
    idx = np.round((wrap(track_deg) + 180.0) / 2.5).astype(int) % len(GRID)
    frame_block = np.minimum(np.arange(X.shape[2]) // an.frames_per_block, len(idx) - 1)
    Wf = W[idx[frame_block]]  # (frames, f, m)
    Y = np.einsum("tfm,mft->ft", Wf.conj(), X)
    _, y = istft(Y, an.geom.fs, nperseg=NFFT, noverlap=NFFT - HOP)
    return y[: an.x.shape[1]]


def follow_track(an: Analysis, hfov: float, yaw: float = 0.0, hold_blocks: int = 3,
                 min_height: float = 0.7, min_activity: float = 2 * ACTIVE_FLOOR) -> np.ndarray:
    """Auto-steer: lock onto the strongest new sound inside the camera view.

    Only re-aims when a new peak wins for hold_blocks blocks in a row, and only
    during blocks with enough sound above the steady noise, so the beam doesn't jitter.
    """
    current = yaw
    challenger, count = None, 0
    track = np.empty(an.n_blocks)
    for b in range(an.n_blocks):
        if an.activity[b] >= min_activity:
            inside = [(a, h) for a, h in an.peaks(b, min_height) if abs(wrap(a - yaw)) <= hfov / 2]
            if inside:
                a = inside[0][0]
                if abs(wrap(a - current)) <= 10:
                    current = current + 0.3 * wrap(a - current)
                    challenger, count = None, 0
                elif challenger is not None and abs(wrap(a - challenger)) <= 10:
                    count += 1
                    if count >= hold_blocks:
                        current, challenger, count = a, None, 0
                else:
                    challenger, count = a, 1
        track[b] = current
    return track


@dataclass
class Event:
    t: float
    az: float
    db: float
    label: str


def detect_events(an: Analysis, min_activity: float = 2 * ACTIVE_FLOOR, min_gap_s: float = 1.5,
                  same_dir_deg: float = 25.0) -> list[Event]:
    """Onsets of new sounds: a fresh peak in the above-the-noise map.

    A sound counts as new when it appears where nothing was active in the
    previous half second, or after a quiet gap. A crude spectral label (tonal
    vs voice-like) stands in for a trained classifier.
    """
    events: list[Event] = []
    recent: list[tuple[float, float]] = []  # (time, angle) of active peaks
    fpb = an.frames_per_block
    band = (an.freqs >= BAND_HZ[0]) & (an.freqs <= BAND_HZ[1])

    def top(b):
        if b >= an.n_blocks or an.activity[b] < min_activity:
            return None
        pk = an.peaks(b, 0.8, max_peaks=1)
        return pk[0][0] if pk else None

    for b in range(an.n_blocks):
        t = b * BLOCK_S
        recent = [(rt, ra) for rt, ra in recent if t - rt <= 1.0]
        az = top(b)
        if az is None:
            continue
        nxt = top(b + 1)
        persists = nxt is not None and abs(wrap(nxt - az)) <= 15
        fresh = all(abs(wrap(az - ra)) > same_dir_deg for _, ra in recent)
        repeat = any(abs(wrap(az - e.az)) <= same_dir_deg and t - e.t < min_gap_s for e in events)
        recent.append((t, az))
        if persists and fresh and not repeat:
            mag = np.mean(np.abs(an.X[an.geom.ref, band, b * fpb:(b + 1) * fpb]) ** 2, axis=1)
            f0 = an.freqs[band][np.argmax(mag)]
            peak_share = np.sort(mag)[-3:].sum() / (mag.sum() + 1e-20)
            label = f"tone {f0 / 1000:.1f} kHz" if peak_share > 0.5 else "voice / broadband"
            events.append(Event(t, az, an.level_db[b], label))
    return events


def direction_name(az: float) -> str:
    a = wrap(az)
    if abs(a) <= 22.5:
        return "front"
    if abs(a) >= 157.5:
        return "behind"
    side = "left" if a > 0 else "right"
    if abs(a) < 67.5:
        return f"front-{side}"
    if abs(a) <= 112.5:
        return side
    return f"behind-{side}"
