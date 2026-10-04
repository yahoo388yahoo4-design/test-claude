#!/usr/bin/env python3
"""Reader test for Stray Scanner and 3D Scanner App exports.

Writes fake exports in each app's documented layout from a real ARKitScenes episode (1 s = 60 frames),
converts them with convert.py, and compares the resulting traj with Apple's. This checks our readers'
axis and pose conventions against the published formats (not against real app files).

  python thirdparty_test.py ARKITSCENES_EPISODE --work /tmp/tp
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
from common import imwrite_any, imwrite_rgb, rot_angle_deg  # noqa: E402
from readers.arkitscenes import ARKitScenesEpisode, read_traj  # noqa: E402


def fake_stray(ep, out: Path, idx: list):
    import cv2
    from scipy.spatial.transform import Rotation
    out.mkdir(parents=True)
    (out / "depth").mkdir()
    (out / "confidence").mkdir()
    f0 = ep.frames[idx[0]]
    np.savetxt(out / "camera_matrix.csv", f0.K, delimiter=",")
    vw = cv2.VideoWriter(str(out / "rgb.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 60, (f0.w, f0.h))
    with open(out / "odometry.csv", "w") as fh:
        fh.write("timestamp, frame, x, y, z, qx, qy, qz, qw\n")
        for j, (k, img) in enumerate(ep.iter_images(idx)):
            f = ep.frames[k]
            vw.write(img[:, :, ::-1].copy())
            q = Rotation.from_matrix(f.T[:3, :3]).as_quat()
            fh.write(f"{f.t}, {j}, {f.T[0, 3]}, {f.T[1, 3]}, {f.T[2, 3]}, {q[0]}, {q[1]}, {q[2]}, {q[3]}\n")
            imwrite_any(out / "depth" / f"{j:06d}.png", ep.depth(k))
            imwrite_any(out / "confidence" / f"{j:06d}.png", ep.conf(k))
    vw.release()


def fake_3dscanner(ep, out: Path, idx: list):
    out.mkdir(parents=True)
    for j, (k, img) in enumerate(ep.iter_images(idx)):
        f = ep.frames[k]
        imwrite_rgb(out / f"frame_{j:05d}.jpg", img, 98)
        (out / f"frame_{j:05d}.json").write_text(json.dumps({
            "cameraPoseARFrame": f.T.reshape(-1).tolist(), "intrinsics": f.K.reshape(-1).tolist(), "time": f.t,
            "frame_index": j}))
        imwrite_any(out / f"depth_{j:05d}.png", ep.depth(k))
        imwrite_any(out / f"conf_{j:05d}.png", ep.conf(k))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("episode", type=Path)
    ap.add_argument("--work", type=Path, required=True)
    ap.add_argument("--frames", type=int, default=60)
    a = ap.parse_args()
    if a.work.exists():
        shutil.rmtree(a.work)
    ep = ARKitScenesEpisode(a.episode)
    orig = read_traj(a.episode / "lowres_wide.traj")
    rep = {}
    for name, fn in (("strayscanner", fake_stray), ("3dscannerapp", fake_3dscanner)):
        src = a.work / name / "export"
        fn(ep, src, [k for k, f in enumerate(ep.frames) if f.T is not None][:a.frames])
        out = a.work / name / "out"
        convert.main([str(src), "--to", "arkitscenes", "litereality", "--out", str(out), "--video-id", "test",
                      "--traj-hz", "0"])
        conv = read_traj(out / "arkitscenes" / "test" / "lowres_wide.traj")
        keys = sorted(set(orig) & set(conv))
        rep[name] = {"detected": json.loads((out / "convert_report.json").read_text())["input"]["source"],
                     "matched_traj": len(keys),
                     "rot_err_deg_max": max(rot_angle_deg(orig[k][1][:3, :3], conv[k][1][:3, :3]) for k in keys),
                     "trans_err_mm_max": max(np.linalg.norm(orig[k][1][:3, 3] - conv[k][1][:3, 3]) for k in keys) * 1000,
                     "depth_frames": len(list((out / "arkitscenes" / "test" / "lowres_depth").glob("*.png"))),
                     "mesh": (out / "arkitscenes" / "test" / "test_3dod_mesh.ply").exists()}
    print(json.dumps(rep, indent=1))
    (a.work / "thirdparty_report.json").write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
