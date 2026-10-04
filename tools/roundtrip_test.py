#!/usr/bin/env python3
"""Round-trip test of the converter on a real ARKitScenes raw episode.

1. Read the ARKitScenes episode and write it as an r2s raw session (what the phone would record:
   ARKit-axes poses on every frame, raw-deflate depth/confidence, y-up mesh, RoomPlan-style boxes).
2. Convert that session back to ARKitScenes (traj on every frame, and again at 10 Hz) and to LiteReality.
3. Compare with the original: poses, intrinsics, RGB, depth, confidence, mesh vertices, mesh colours
   (recomputed by projection), boxes. Optionally load the result with Apple's own
   tenFpsDataLoader-style pose lookup.

  python roundtrip_test.py /path/to/ARKitScenes/raw/Training/40753679 --work /tmp/rt [--video]
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import convert  # noqa: E402
from common import imread_any, imread_rgb, ply_rgb, ply_xyz, read_ply, rot_angle_deg  # noqa: E402
from readers.arkitscenes import ARKitScenesEpisode, read_traj  # noqa: E402
from writers import write_r2s_raw  # noqa: E402


def compare(orig: Path, conv: Path, vid: str, every: bool) -> dict:
    res = {}
    to, tc = read_traj(orig / "lowres_wide.traj"), read_traj(conv / "lowres_wide.traj")
    common_keys = sorted(set(to) & set(tc))
    rerr = [rot_angle_deg(to[k][1][:3, :3], tc[k][1][:3, :3]) for k in common_keys]
    terr = [np.linalg.norm(to[k][1][:3, 3] - tc[k][1][:3, 3]) for k in common_keys]
    res["traj"] = {"orig_lines": len(to), "converted_lines": len(tc), "matched": len(common_keys),
                   "rot_err_deg_max": float(np.max(rerr)) if rerr else None,
                   "trans_err_mm_max": float(np.max(terr) * 1000) if terr else None}
    if every:
        res["traj"]["all_orig_timestamps_present"] = len(common_keys) == len(to)
    # every converted traj key must name an existing frame (the loader's requirement)
    names = {p.stem.split("_")[-1] for p in (conv / "lowres_wide").glob("*.png")}
    res["traj"]["keys_without_frame"] = len(set(tc) - names)
    # images / depth / confidence / intrinsics, on a sample of frames
    o_imgs = sorted((orig / "lowres_wide").glob("*.png"))
    sample = o_imgs[:: max(1, len(o_imgs) // 50)]
    rgb_err, d_eq, c_eq, k_err, missing = [], 0, 0, [], 0
    for p in sample:
        q = conv / "lowres_wide" / p.name
        if not q.exists():
            missing += 1
            continue
        rgb_err.append(np.abs(imread_rgb(p).astype(int) - imread_rgb(q).astype(int)).mean())
        for sub, ctr in (("lowres_depth", "d"), ("confidence", "c")):
            a, b = orig / sub / p.name, conv / sub / p.name
            if a.exists() and b.exists() and np.array_equal(imread_any(a), imread_any(b)):
                if ctr == "d":
                    d_eq += 1
                else:
                    c_eq += 1
        ko = np.array(open(orig / "lowres_wide_intrinsics" / f"{p.stem}.pincam").read().split(), float)
        kc = np.array(open(conv / "lowres_wide_intrinsics" / f"{p.stem}.pincam").read().split(), float)
        k_err.append(np.abs(ko - kc).max())
    res["frames"] = {"sampled": len(sample), "missing": missing, "rgb_mean_abs_err": float(np.mean(rgb_err)),
                     "depth_identical": d_eq, "conf_identical": c_eq, "intrinsics_max_abs_err": float(np.max(k_err))}
    # mesh
    mo, mc = read_ply(orig / f"{vid}_3dod_mesh.ply"), read_ply(conv / f"{vid}_3dod_mesh.ply")
    xo, xc = ply_xyz(mo), ply_xyz(mc)
    res["mesh"] = {"vertices": [len(xo), len(xc)], "faces_equal": bool(np.array_equal(mo["faces"], mc["faces"])),
                   "vertex_max_err_mm": float(np.abs(xo - xc).max() * 1000)}
    co, cc = ply_rgb(mo).astype(int), ply_rgb(mc).astype(int)
    seen = ~np.all(cc == 128, axis=1)
    res["mesh"]["recolored_fraction"] = float(seen.mean())
    res["mesh"]["recolor_mean_abs_err"] = float(np.abs(co[seen] - cc[seen]).mean())
    res["mesh"]["recolor_mean_abs_err_if_random"] = float(np.abs(co[seen] - np.roll(co[seen], 7919, 0)).mean())
    # boxes
    ao = json.loads((orig / f"{vid}_3dod_annotation.json").read_text())["data"]
    ac = json.loads((conv / f"{vid}_3dod_annotation.json").read_text())["data"]
    errs = []
    for a, b in zip(ao, ac):
        oa, ob = a["segments"]["obbAligned"], b["segments"]["obbAligned"]
        errs.append({"label": [a["label"], b["label"]],
                     "centroid_mm": float(np.linalg.norm(np.subtract(oa["centroid"], ob["centroid"])) * 1000),
                     "lengths_mm": float(np.abs(np.subtract(oa["axesLengths"], ob["axesLengths"])).max() * 1000),
                     "axes": float(np.abs(np.subtract(oa["normalizedAxes"], ob["normalizedAxes"])).max())})
    res["boxes"] = {"orig": len(ao), "converted": len(ac), "per_box": errs}
    return res


def tenfps_loader_check(conv: Path, vid: str) -> dict:
    """Mimic threedod/benchmark_scripts/utils/tenFpsDataLoader.py frame/pose matching."""
    import cv2
    poses = {}
    for line in (conv / "lowres_wide.traj").read_text().splitlines():
        tok = line.split()
        R, _ = cv2.Rodrigues(np.asarray([float(x) for x in tok[1:4]]))
        E = np.eye(4)
        E[:3, :3], E[:3, 3] = R, [float(x) for x in tok[4:7]]
        poses[f"{round(float(tok[0]), 3):.3f}"] = np.linalg.inv(E)
    frames = sorted((conv / "lowres_wide").glob("*.png"))
    used = [f for f in frames if f.stem.split("_")[-1] in poses]
    return {"frames": len(frames), "frames_with_pose": len(used), "traj_lines": len(poses)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode", type=Path)
    ap.add_argument("--work", type=Path, required=True)
    ap.add_argument("--video", action="store_true", help="fake session stores RGB in a lossy video.mov")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()
    vid = args.episode.name
    if args.work.exists():
        shutil.rmtree(args.work)
    ep = ARKitScenesEpisode(args.episode)
    if args.max_frames:
        ep.frames = ep.frames[:args.max_frames]
    interp = sum(bool(f.meta.get("interpolated")) for f in ep.frames)
    print(f"episode {vid}: {len(ep.frames)} frames, {interp} with interpolated poses")
    sess = args.work / "session"
    write_r2s_raw(ep, sess, video=args.video)
    report = {"episode": vid, "frames": len(ep.frames), "interpolated_poses_in_fake_session": interp,
              "video": args.video}
    for label, hz in (("every_frame", 0), ("10hz", 10)):
        out = args.work / f"out_{label}"
        convert.main([str(sess), "--to", "arkitscenes", "litereality", "--out", str(out),
                      "--video-id", vid, "--traj-hz", str(hz)] + (["--max-frames", str(args.max_frames)] if args.max_frames else []))
        conv = out / "arkitscenes" / vid
        r = compare(args.episode, conv, vid, every=(hz == 0))
        r["loader"] = tenfps_loader_check(conv, vid)
        lr = out / "litereality" / "session"
        r["litereality"] = {"frames": len(list(lr.glob("frame_*.jpg"))), "depth": len(list(lr.glob("depth_*.png"))),
                            "conf": len(list(lr.glob("conf_*.png"))), "obj": (lr / "textured_output.obj").exists()}
        report[label] = r
    print(json.dumps(report, indent=1))
    (args.work / "roundtrip_report.json").write_text(json.dumps(report, indent=1))


if __name__ == "__main__":
    main()
