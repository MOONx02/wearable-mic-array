"""Simulate an 8-mic shirt array (Tier B) in a noisy server aisle.

Writes, into sim/:
  recording.wav   8-channel, 16 kHz, 16-bit: what the shirt would record
  geometry.json   mic names and positions (channel order = list order)
  scene.json      ground truth: where each source is and when it is active
  premix.npz      each source's contribution at each mic (fans summed), for measuring dB gains

Free-field room simulation (pyroomacoustics image sources). The wearer's body is
not modelled, so rear mics hear front talkers unshadowed; real gains will differ.

Talkers are macOS `say` voices, generated on first run.
"""

from __future__ import annotations

import json
import subprocess
import wave
from pathlib import Path

import numpy as np
import pyroomacoustics as pra

HERE = Path(__file__).resolve().parent
SIM = HERE / "sim"
FS = 16000
C = 343.0

ROOM = [10.0, 7.0, 3.0]
RT60 = 0.5
WEARER = np.array([5.0, 3.5, 1.3])  # chest centre; wearer faces +x

# Body frame: x forward, y wearer's left, z up, metres from chest centre.
# Names and spacing follow shirt-mic-layout.md (Tier B).
MICS = [
    ("F2", [0.11, -0.075, 0.00]),
    ("F3", [0.11, -0.025, 0.00]),
    ("F4", [0.11, 0.025, 0.00]),
    ("F5", [0.11, 0.075, 0.00]),
    ("B1", [-0.11, -0.15, 0.00]),
    ("B4", [-0.11, 0.15, 0.00]),
    ("S_R", [0.00, -0.14, 0.08]),
    ("S_L", [0.00, 0.14, 0.08]),
]

SPEECH = {
    "a1": ("Samantha", "Hey, can you check the temperature on rack B3? The fan sounds a lot louder "
                       "than yesterday. I think the bearing is going."),
    "b1": ("Daniel", "The cooling report says aisle two is running warm. I can pull the logs after "
                     "lunch if you need them."),
    "a2": ("Samantha", "Okay. Let's log it, and swap the fan during the next maintenance window."),
}

TALKER_SNR_DB = -3.0  # talker vs all fan noise, at the reference mic


def read_wav(path: Path) -> np.ndarray:
    with wave.open(str(path)) as w:
        assert w.getframerate() == FS and w.getsampwidth() == 2
        data = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)
        return data.reshape(-1, w.getnchannels()) / 32768.0


def write_wav(path: Path, data: np.ndarray) -> None:
    data = np.atleast_2d(data.T).T
    pcm = np.clip(np.round(data * 32767), -32768, 32767).astype("<i2")
    with wave.open(str(path), "wb") as w:
        w.setnchannels(pcm.shape[1])
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(pcm.tobytes())


def tts(key: str) -> np.ndarray:
    voice, text = SPEECH[key]
    path = SIM / f"tts_{key}.wav"
    if not path.exists():
        subprocess.run(["say", "-v", voice, "-o", str(path), "--data-format=LEI16@16000", text], check=True)
    sig = read_wav(path)[:, 0]
    return sig / (np.sqrt(np.mean(sig[np.abs(sig) > 0.01] ** 2)) + 1e-12)


def fan_noise(n: int, rng: np.random.Generator, hum_hz: float, tone_hz: float) -> np.ndarray:
    """Broadband fan rush tilted toward low frequencies, plus motor hum and a blade tone."""
    white = rng.standard_normal(n)
    spec = np.fft.rfft(white)
    f = np.fft.rfftfreq(n, 1 / FS)
    spec *= 1.0 / np.sqrt(np.maximum(f, 60.0) / 60.0)
    spec[f > 6000] *= 0.3
    noise = np.fft.irfft(spec, n)
    t = np.arange(n) / FS
    noise /= np.std(noise)
    for k, amp in ((1, 0.5), (2, 0.3), (3, 0.15)):
        noise += amp * np.sin(2 * np.pi * hum_hz * k * t + rng.uniform(0, 6.3))
    noise += 0.25 * np.sin(2 * np.pi * tone_hz * t) * (1 + 0.2 * np.sin(2 * np.pi * 0.7 * t))
    return noise / np.std(noise)


def alarm(duration: float) -> np.ndarray:
    t = np.arange(int(duration * FS)) / FS
    phase = np.mod(t, 0.5)
    envelope = np.clip(np.minimum(phase, 0.25 - phase) / 0.01, 0, 1)
    return np.sin(2 * np.pi * 2000 * t) * envelope


def place(az_deg: float, dist: float, height: float) -> list[float]:
    a = np.radians(az_deg)
    return [WEARER[0] + dist * np.cos(a), WEARER[1] + dist * np.sin(a), height]


def main() -> None:
    SIM.mkdir(exist_ok=True)
    rng = np.random.default_rng(7)
    a1, b1, a2 = tts("a1"), tts("b1"), tts("a2")

    t_a1 = 0.5
    t_b1 = t_a1 + len(a1) / FS - 0.5
    t_alarm = t_b1 + len(b1) / FS + 0.5
    t_a2 = t_alarm + 1.0
    total = t_a2 + len(a2) / FS + 1.0
    n = int(total * FS)

    samantha = np.zeros(n)
    samantha[int(t_a1 * FS):int(t_a1 * FS) + len(a1)] = a1
    samantha[int(t_a2 * FS):int(t_a2 * FS) + len(a2)] = a2
    daniel = np.zeros(n)
    daniel[int(t_b1 * FS):int(t_b1 * FS) + len(b1)] = b1
    beeps = np.zeros(n)
    al = alarm(2.0)
    beeps[int(t_alarm * FS):int(t_alarm * FS) + len(al)] = al

    sources = [
        dict(name="Samantha", kind="talker", az=20.0, dist=1.5, height=1.6, signal=samantha,
             active=[[t_a1, t_a1 + len(a1) / FS], [t_a2, t_a2 + len(a2) / FS]]),
        dict(name="Daniel", kind="talker", az=-30.0, dist=2.0, height=1.65, signal=daniel,
             active=[[t_b1, t_b1 + len(b1) / FS]]),
        dict(name="Alarm", kind="event", az=150.0, dist=2.5, height=2.2, signal=beeps,
             active=[[t_alarm, t_alarm + 2.0]]),
    ]
    for i, (az, dist, hum, tone) in enumerate([(-100, 2.0, 120, 1180), (115, 2.2, 118, 1430),
                                               (-160, 3.0, 122, 960), (70, 3.0, 121, 1650)]):
        sources.append(dict(name=f"Rack fan {i + 1}", kind="noise", az=float(az), dist=dist, height=1.0,
                            signal=fan_noise(n, rng, hum, tone), active=[[0.0, total]]))

    e_abs, max_order = pra.inverse_sabine(RT60, ROOM)
    room = pra.ShoeBox(ROOM, fs=FS, materials=pra.Material(e_abs), max_order=min(max_order, 12))
    for s in sources:
        room.add_source(place(s["az"], s["dist"], s["height"]), signal=s["signal"])
    mic_pos = WEARER[:, None] + np.array([p for _, p in MICS]).T
    room.add_microphone_array(mic_pos)
    premix = room.simulate(return_premix=True)[:, :, :n]  # (sources, mics, samples)

    ref = [m for m, _ in MICS].index("F3")

    def power(i, seg=None):
        x = premix[i, ref]
        if seg is not None:
            x = x[int(seg[0] * FS):int(seg[1] * FS)]
        return np.mean(x ** 2)

    talker_p = power(0, sources[0]["active"][0])
    noise_idx = [i for i, s in enumerate(sources) if s["kind"] == "noise"]
    noise_p = sum(power(i) for i in noise_idx)
    gains = np.ones(len(sources))
    gains[noise_idx] = np.sqrt(talker_p / noise_p / 10 ** (TALKER_SNR_DB / 10))
    gains[1] = np.sqrt(talker_p / power(1, sources[1]["active"][0]) * 10 ** (-2 / 10))
    gains[2] = np.sqrt(talker_p / power(2, sources[2]["active"][0]) * 10 ** (3 / 10))
    premix *= gains[:, None, None]

    mix = premix.sum(axis=0)
    mix += 10 ** (-70 / 20) * rng.standard_normal(mix.shape)  # mic self-noise
    scale = 0.5 / np.max(np.abs(mix))
    mix *= scale
    premix *= scale

    write_wav(SIM / "recording.wav", mix.T)
    parts = [premix[i] for i, s in enumerate(sources) if s["kind"] != "noise"]
    parts.append(premix[noise_idx].sum(axis=0))
    parts = np.stack(parts)
    part_scale = np.max(np.abs(parts)) / 32000
    np.savez_compressed(SIM / "premix.npz", premix_i16=np.round(parts / part_scale).astype(np.int16),
                        scale=part_scale,
                        names=np.array([s["name"] for s in sources if s["kind"] != "noise"] + ["Fans"]))
    (SIM / "geometry.json").write_text(json.dumps({
        "fs": FS,
        "frame": "x forward, y wearer's left, z up; metres from chest centre",
        "reference_mic": "F3",
        "mics": [{"name": m, "pos_m": p} for m, p in MICS],
    }, indent=2))
    (SIM / "scene.json").write_text(json.dumps({
        "simulated": True,
        "duration_s": total,
        "room_m": ROOM,
        "rt60_s": RT60,
        "talker_snr_db": TALKER_SNR_DB,
        "camera": {"hfov_deg": 66.0, "yaw_offset_deg": 0.0},
        "sources": [{k: v for k, v in s.items() if k != "signal"} for s in sources],
    }, indent=2))
    print(f"wrote {SIM}/recording.wav  ({total:.1f} s, {len(MICS)} ch)")


if __name__ == "__main__":
    main()
