"""Mic-array HUD: see where sound comes from and where the beam is pointing.

  python hud.py render   # HUD video (beam audio) + WAVs + key frames into out/
  python hud.py view     # interactive: click a person or a radar direction to listen there
  python hud.py eval     # simulated data only: direction accuracy and dB gains

Defaults use the simulated recording in sim/ (run simulate.py first).
For a real recording pass --recording, --geometry and optionally --video.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from hud_core import (BLOCK_S, HOP, NFFT, Analysis, Geometry, beamform, detect_events,  # noqa: E402
                      follow_track, read_wav, wrap, write_wav)


def load(args):
    geom = Geometry.load(args.geometry)
    x, fs = read_wav(args.recording)
    if fs != geom.fs or x.shape[0] != len(geom.names):
        raise SystemExit(f"recording has {x.shape[0]} ch at {fs} Hz; geometry expects "
                         f"{len(geom.names)} ch at {geom.fs} Hz")
    scene = json.loads(Path(args.scene).read_text()) if args.scene and Path(args.scene).exists() else None
    hfov = args.hfov or (scene or {}).get("camera", {}).get("hfov_deg", 66.0)
    yaw = args.yaw if args.yaw is not None else (scene or {}).get("camera", {}).get("yaw_offset_deg", 0.0)
    return geom, x, scene, hfov, yaw


def block_db(y: np.ndarray, fs: int, n_blocks: int) -> np.ndarray:
    n = int(BLOCK_S * fs)
    y = np.pad(y, (0, max(0, n_blocks * n - len(y))))[: n_blocks * n]
    return 10 * np.log10(np.mean(y.reshape(n_blocks, n) ** 2, axis=1) + 1e-12)


def make_track(an, args, hfov, yaw):
    if args.steer is not None:
        return np.full(an.n_blocks, float(args.steer)), f"fixed {args.steer:+.0f}°"
    return follow_track(an, hfov, yaw), "auto-follow"


def build(args):
    from hud_render import Camera, HudData, beam_locks

    geom, x, scene, hfov, yaw = load(args)
    an = Analysis(geom, x)
    track, mode = make_track(an, args, hfov, yaw)
    y = beamform(an, track, args.beam)
    data = HudData(an=an, track=track, beam_db=block_db(y, geom.fs, an.n_blocks), events=detect_events(an),
                   cam=Camera(hfov, yaw), beam_label=f"Beam: {mode} ({args.beam.upper()})", scene=scene,
                   simulated=bool(scene and scene.get("simulated")), locks=beam_locks(track))
    return data, y


class VideoFrames:
    """Reads a camera video at the HUD frame rate, aligned by a clap offset."""

    def __init__(self, path: Path, offset_s: float, size=(1120, 840)):
        from PySide6.QtGui import QImage
        self.QImage, self.size = QImage, size
        w, h = size
        vf = f"fps={1 / BLOCK_S:g},scale={w}:{h}:force_original_aspect_ratio=increase,crop={w}:{h}"
        cmd = ["ffmpeg", "-loglevel", "error"]
        cmd += ["-ss", f"{offset_s}"] if offset_s >= 0 else ["-itsoffset", f"{-offset_s}"]
        cmd += ["-i", str(path), "-vf", vf, "-f", "rawvideo", "-pix_fmt", "bgra", "-"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        self.frame_bytes = w * h * 4

    def next(self):
        buf = self.proc.stdout.read(self.frame_bytes)
        if len(buf) < self.frame_bytes:
            return None
        img = self.QImage(buf, self.size[0], self.size[1], self.QImage.Format_RGB32)
        return img.copy()

    def close(self):
        self.proc.stdout.close()
        self.proc.kill()
        self.proc.wait()


def cmd_render(args):
    from PySide6.QtGui import QGuiApplication
    from hud_render import H, W, render_frame

    app = QGuiApplication.instance() or QGuiApplication([])  # noqa: F841
    data, y = build(args)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    fs = data.an.geom.fs
    tag = "auto" if args.steer is None else f"fixed{args.steer:+.0f}"
    beam_wav = out / f"beam_{tag}.wav"
    write_wav(beam_wav, 0.9 * y / (np.max(np.abs(y)) + 1e-12), fs)
    ref = data.an.x[data.an.geom.ref]
    write_wav(out / f"mic_{data.an.geom.names[data.an.geom.ref]}.wav", 0.9 * ref / np.max(np.abs(ref)), fs)

    video = VideoFrames(Path(args.video), args.video_offset) if args.video else None
    mp4 = out / f"hud_{tag}.mp4"
    enc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgra",
                            "-s", f"{W}x{H}", "-r", f"{1 / BLOCK_S:g}", "-i", "-", "-i", str(beam_wav),
                            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-c:a", "aac",
                            "-shortest", str(mp4)], stdin=subprocess.PIPE)
    stills = {round(float(s) / BLOCK_S) for s in args.stills}
    for b in range(data.an.n_blocks):
        frame = video.next() if video else None
        img = render_frame(data, b, frame)
        if b in stills:
            img.save(str(out / f"hud_{tag}_t{b * BLOCK_S:04.1f}.png"))
        enc.stdin.write(bytes(img.constBits())[: W * H * 4])
    enc.stdin.close()
    enc.wait()
    if video:
        video.close()
    print(f"wrote {mp4}\nwrote {beam_wav}")


def cmd_eval(args):
    """Ground-truth checks that only a simulation can give."""
    from scipy.signal import stft

    geom, x, scene, hfov, yaw = load(args)
    if not scene:
        raise SystemExit("eval needs scene.json (simulated recordings only)")
    an = Analysis(geom, x)
    pm = np.load(Path(args.recording).with_name("premix.npz"))
    parts = pm["premix_i16"].astype(float) * float(pm["scale"])
    names = list(pm["names"])
    fs = geom.fs
    result = {"direction": {}, "beam_gain_db": {}}

    for s in scene["sources"]:
        if s["kind"] == "noise":
            continue
        errs = []
        for a, bnd in s["active"]:
            for b in range(int(a / BLOCK_S) + 2, int(bnd / BLOCK_S) - 2):
                pk = an.peaks(b, 0.5)
                if pk and an.activity[b] > 0:
                    errs.append(abs(float(wrap(pk[0][0] - s["az"]))))
        errs = np.array(errs)
        result["direction"][s["name"]] = {
            "true_deg": s["az"], "median_error_deg": float(np.median(errs)),
            "within_10deg": float(np.mean(errs <= 10)), "blocks": int(len(errs))}

    def gains(track, kind):
        out = {}
        for i, nm in enumerate(names):
            _, _, Xi = stft(parts[i], fs, nperseg=NFFT, noverlap=NFFT - HOP)
            yi = beamform(an, track, kind, X=Xi)
            out[nm] = float(10 * np.log10(np.mean(yi ** 2) / np.mean(parts[i, geom.ref] ** 2)))
        return out

    for kind in ("das", "mvdr"):
        for s in scene["sources"]:
            if s["kind"] == "talker":
                g = gains(np.full(an.n_blocks, s["az"]), kind)
                result["beam_gain_db"][f"{kind}: beam on {s['name']}"] = {
                    k: round(v - g[s["name"]], 1) for k, v in g.items()}
    track = follow_track(an, hfov, yaw)
    result["auto_track"] = {"samples_deg": [round(float(v), 1) for v in track[::10]]}

    print(json.dumps(result, indent=2))
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "sim_results.json").write_text(json.dumps(result, indent=2))
    print("Gains are relative to the beam's target (0 dB); negative = turned down vs one mic.")


def cmd_view(args):
    from PySide6.QtCore import QPointF, Qt, QTimer, QUrl
    from PySide6.QtGui import QPixmap
    from PySide6.QtMultimedia import QAudioOutput, QMediaPlayer
    from PySide6.QtWidgets import (QApplication, QComboBox, QHBoxLayout, QLabel, QPushButton, QSlider,
                                   QVBoxLayout, QWidget)
    from hud_render import CAM, RADAR_BOX, RADAR_CY, W, render_frame

    app = QApplication.instance() or QApplication([])
    data, y = build(args)
    an = data.an
    fs = an.geom.fs
    tmp = Path(tempfile.mkdtemp(prefix="mic-hud-"))
    video = VideoFrames(Path(args.video), args.video_offset) if args.video else None
    frames = [video.next() for _ in range(an.n_blocks)] if video else None
    if video:
        video.close()

    win = QWidget()
    win.setWindowTitle("Mic-array HUD")
    screen = QLabel()
    screen.setMinimumSize(1280, 720)
    screen.setAlignment(Qt.AlignCenter)
    slider = QSlider(Qt.Horizontal)
    slider.setRange(0, an.n_blocks - 1)
    play = QPushButton("Play")
    auto = QPushButton("Auto-follow")
    kind = QComboBox()
    kind.addItems(["mvdr", "das"])
    kind.setCurrentText(args.beam)
    hint = QLabel("Click a person in the view, or a direction on the radar, to steer the beam there.")
    row = QHBoxLayout()
    for wdg in (play, auto, QLabel("Beam:"), kind, slider):
        row.addWidget(wdg)
    lay = QVBoxLayout(win)
    lay.addWidget(screen, 1)
    lay.addLayout(row)
    lay.addWidget(hint)

    player = QMediaPlayer()
    audio = QAudioOutput()
    player.setAudioOutput(audio)
    state = {"b": 0, "version": 0}

    def load_audio(sig):
        state["version"] += 1
        path = tmp / f"beam_{state['version']}.wav"
        write_wav(path, 0.9 * sig / (np.max(np.abs(sig)) + 1e-12), fs)
        pos = player.position()
        playing = player.playbackState() == QMediaPlayer.PlayingState
        player.setSource(QUrl.fromLocalFile(str(path)))
        player.setPosition(pos)
        if playing:
            player.play()

    def steer(track, label):
        data.track = track
        data.beam_label = f"Beam: {label} ({kind.currentText().upper()})"
        from hud_render import beam_locks
        data.locks = beam_locks(track)
        sig = beamform(an, track, kind.currentText())
        data.beam_db = block_db(sig, fs, an.n_blocks)
        data.timeline_img = None
        load_audio(sig)
        show(state["b"])

    def show(b):
        state["b"] = b
        img = render_frame(data, b, frames[b] if frames else None)
        screen.setPixmap(QPixmap.fromImage(img).scaled(screen.size(), Qt.KeepAspectRatio,
                                                       Qt.SmoothTransformation))

    def to_frame(pos):
        pm = screen.pixmap()
        if pm is None or pm.isNull():
            return None
        ox = (screen.width() - pm.width()) / 2
        oy = (screen.height() - pm.height()) / 2
        sx = W / pm.width()
        return QPointF((pos.x() - ox) * sx, (pos.y() - oy) * sx)

    def clicked(event):
        q = to_frame(event.position())
        if q is None:
            return
        if CAM.contains(q):
            az = data.cam.az_of(q.x())
        elif RADAR_BOX.contains(q):
            cx, cy = RADAR_BOX.center().x(), RADAR_CY
            az = float(np.degrees(np.arctan2(-(q.x() - cx), -(q.y() - cy))))
        else:
            return
        steer(np.full(an.n_blocks, az), f"fixed {az:+.0f}° (clicked)")

    screen.mousePressEvent = clicked
    slider.valueChanged.connect(lambda v: (player.setPosition(int(v * BLOCK_S * 1000)), show(v)))
    play.clicked.connect(lambda: player.pause() if player.playbackState() == QMediaPlayer.PlayingState
                         else player.play())
    player.playbackStateChanged.connect(
        lambda s: play.setText("Pause" if s == QMediaPlayer.PlayingState else "Play"))
    auto.clicked.connect(lambda: steer(follow_track(an, data.cam.hfov, data.cam.yaw), "auto-follow"))
    kind.currentTextChanged.connect(lambda _: steer(data.track, data.beam_label.split(": ", 1)[1].rsplit(" (", 1)[0]))

    timer = QTimer()

    def tick():
        if player.playbackState() == QMediaPlayer.PlayingState:
            b = min(int(player.position() / 1000 / BLOCK_S), an.n_blocks - 1)
            if b != state["b"]:
                slider.blockSignals(True)
                slider.setValue(b)
                slider.blockSignals(False)
                show(b)

    timer.timeout.connect(tick)
    timer.start(40)
    load_audio(y)
    win.resize(1600, 1000)
    win.show()
    show(0)
    if args.screenshot:
        QTimer.singleShot(500, lambda: (screen.grab().save(args.screenshot), app.quit()))
    app.exec()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["render", "view", "eval"])
    ap.add_argument("--recording", default=str(HERE / "sim" / "recording.wav"), help="multichannel 16-bit WAV")
    ap.add_argument("--geometry", default=str(HERE / "sim" / "geometry.json"))
    ap.add_argument("--scene", default=str(HERE / "sim" / "scene.json"),
                    help="ground truth (simulations); ignored if missing")
    ap.add_argument("--video", help="camera video recorded with the array")
    ap.add_argument("--video-offset", type=float, default=0.0,
                    help="seconds into the video where the recording starts (line up a clap)")
    ap.add_argument("--hfov", type=float, help="camera horizontal field of view, degrees (default 66)")
    ap.add_argument("--yaw", type=float, help="camera direction in array angles, degrees (default 0)")
    ap.add_argument("--steer", type=float, help="fixed beam angle instead of auto-follow (+ = wearer's left)")
    ap.add_argument("--beam", choices=["mvdr", "das"], default="mvdr")
    ap.add_argument("--stills", nargs="*", default=["2.0", "9.0", "14.0", "16.0"],
                    help="times (s) to also save as PNG")
    ap.add_argument("--out", default=str(HERE / "out"))
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    args = ap.parse_args()
    {"render": cmd_render, "view": cmd_view, "eval": cmd_eval}[args.command](args)


if __name__ == "__main__":
    main()
