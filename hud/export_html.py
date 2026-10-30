"""Export the HUD as one self-contained HTML file that opens in any browser.

  python export_html.py                      # simulated recording -> out/mic-hud-standalone.html
  python export_html.py --recording take.wav --geometry geom.json --scene none \
      --video phone.mp4 --video-offset 2.35 --hfov 66

The page embeds the 8-channel recording and steers the beam in JavaScript,
so click-to-listen works with nothing installed. Only numpy/scipy are needed
to export (no Qt).
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from hud_core import (BLOCK_S, GRID, HOP, NFFT, Analysis, Geometry, detect_events,  # noqa: E402
                      follow_track, read_wav)


def build_payload(args) -> dict:
    geom = Geometry.load(args.geometry)
    x, fs = read_wav(args.recording)
    if fs != geom.fs or x.shape[0] != len(geom.names):
        raise SystemExit(f"recording has {x.shape[0]} ch at {fs} Hz; geometry expects "
                         f"{len(geom.names)} ch at {geom.fs} Hz")
    scene_path = Path(args.scene) if args.scene and args.scene != "none" else None
    scene = json.loads(scene_path.read_text()) if scene_path and scene_path.exists() else None
    cam = (scene or {}).get("camera", {})
    hfov = args.hfov or cam.get("hfov_deg", 66.0)
    yaw = args.yaw if args.yaw is not None else cam.get("yaw_offset_deg", 0.0)

    an = Analysis(geom, x)
    peaks = [[[round(float(a), 1), round(float(h), 3)] for a, h in an.peaks(b, 0.8, max_peaks=3)]
             if an.activity[b] > 0 else [] for b in range(an.n_blocks)]
    pcm = np.clip(np.round(x * 32768), -32768, 32767).astype("<i2")

    def r(v, nd=3):
        return [round(float(t), nd) for t in v]

    return {
        "fs": fs, "nfft": NFFT, "hop": HOP, "block_s": BLOCK_S,
        "frames_per_block": an.frames_per_block, "n_blocks": an.n_blocks, "duration": an.duration,
        "names": geom.names, "pos": geom.pos.round(4).tolist(), "ref": geom.ref,
        "grid": r(GRID, 1), "srp": [r(row) for row in an.srp], "activity": r(an.activity, 4),
        "steady": r(an.steady), "level_db": r(an.level_db, 2), "peaks": peaks,
        "events": [{"t": round(e.t, 2), "az": round(float(e.az), 1), "db": round(float(e.db), 1),
                    "label": e.label} for e in detect_events(an)],
        "auto_track": r(follow_track(an, hfov, yaw), 2),
        "hfov": hfov, "yaw": yaw, "scene": scene, "simulated": bool(scene and scene.get("simulated")),
        "video_offset": args.video_offset,
        "samples": int(pcm.shape[1]),
        "pcm_b64": base64.b64encode(pcm.tobytes()).decode("ascii"),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recording", default=str(HERE / "sim" / "recording.wav"))
    ap.add_argument("--geometry", default=str(HERE / "sim" / "geometry.json"))
    ap.add_argument("--scene", default=str(HERE / "sim" / "scene.json"), help="'none' for real recordings")
    ap.add_argument("--video", help="camera video (mp4) to show in the camera view")
    ap.add_argument("--video-offset", type=float, default=0.0,
                    help="seconds into the video where the recording starts (line up a clap)")
    ap.add_argument("--hfov", type=float)
    ap.add_argument("--yaw", type=float)
    ap.add_argument("--out", default=str(HERE / "out" / "mic-hud-standalone.html"))
    args = ap.parse_args()

    payload = build_payload(args)
    web = HERE / "web"
    page = (web / "template.html").read_text()
    video_tag = ""
    if args.video:
        b64 = base64.b64encode(Path(args.video).read_bytes()).decode("ascii")
        video_tag = f'<video id="cam" muted playsinline preload="auto" src="data:video/mp4;base64,{b64}"></video>'
    page = (page.replace("/*__DSP__*/", (web / "dsp.js").read_text())
                .replace("/*__HUD__*/", (web / "hud.js").read_text())
                .replace("<!--__VIDEO__-->", video_tag)
                .replace("/*__DATA__*/null", json.dumps(payload, separators=(",", ":"))))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(page)
    print(f"wrote {out}  ({out.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
