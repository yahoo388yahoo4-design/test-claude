#!/usr/bin/env python3
"""Convert an iPhone capture to ARKitScenes and/or LiteReality layouts.

Inputs (auto-detected, see readers/__init__.py): r2s raw sessions (our iOS app, FORMAT.md),
Stray Scanner exports, 3D Scanner App "All Data" exports, ARKitScenes raw episodes.

  python convert.py SESSION --to arkitscenes litereality --out OUT
  python convert.py SESSION --to arkitscenes --out OUT --video-id 47000001 --traj-hz 10 --wide --vga
  python convert.py SESSION --to r2s --out OUT          # normalise any input to the raw format

Outputs:
  OUT/arkitscenes/<video_id>/...  + OUT/arkitscenes/metadata.csv  (+ <video_id>/extras/)
  OUT/litereality/<name>/...                                      (+ extras/)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from readers import READERS, open_episode  # noqa: E402
from writers import write_arkitscenes, write_extras, write_litereality, write_r2s_raw  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--from", dest="fmt", default="auto", choices=["auto", *READERS])
    ap.add_argument("--to", nargs="+", default=["arkitscenes", "litereality"],
                    choices=["arkitscenes", "litereality", "r2s"])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--video-id", default=None, help="ARKitScenes video_id (default: session folder name)")
    ap.add_argument("--traj-hz", type=float, default=10.0, help="ARKitScenes traj rate; 0 = every frame")
    ap.add_argument("--wide", action="store_true", help="also write full-res wide/ on traj frames")
    ap.add_argument("--vga", action="store_true", help="also write vga_wide/ 640x480 at 30 Hz")
    ap.add_argument("--lr-fps", type=float, default=10.0, help="LiteReality frame rate")
    ap.add_argument("--smoothed-depth", action="store_true", help="use ARKit smoothedSceneDepth if recorded")
    ap.add_argument("--no-extras", action="store_true")
    ap.add_argument("--fold", default="Training")
    ap.add_argument("--max-frames", type=int, default=0, help="debug: only the first N frames")
    args = ap.parse_args(argv)

    t0 = time.time()
    ep = open_episode(args.session, args.fmt)
    if args.smoothed_depth and hasattr(ep, "use_smoothed"):
        ep.use_smoothed = True
    if args.max_frames:
        ep.frames = ep.frames[:args.max_frames]
    if not ep.frames:
        sys.exit(f"{args.session}: no frames")
    print(json.dumps({"input": ep.summary()}))
    report = {"input": ep.summary(), "outputs": {}}
    if "arkitscenes" in args.to:
        root = args.out / "arkitscenes"
        info = write_arkitscenes(ep, root, args.video_id, args.traj_hz, args.wide, args.vga, fold=args.fold)
        if not args.no_extras:
            write_extras(ep, Path(info["out"]))
        report["outputs"]["arkitscenes"] = info
    if "litereality" in args.to:
        out = args.out / "litereality" / ep.name
        info = write_litereality(ep, out, args.lr_fps)
        if not args.no_extras:
            write_extras(ep, out)
        report["outputs"]["litereality"] = info
    if "r2s" in args.to:
        report["outputs"]["r2s"] = write_r2s_raw(ep, args.out / "r2s" / ep.name)
    report["seconds"] = round(time.time() - t0, 1)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "convert_report.json").write_text(json.dumps(report, indent=2, default=str))
    print(f"done in {report['seconds']} s -> {args.out}")


if __name__ == "__main__":
    main()
