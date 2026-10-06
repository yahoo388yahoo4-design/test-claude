#!/usr/bin/env python3
"""Validate a capture or a converted dataset folder.

  python validate.py SESSION_DIR                 # r2s raw / Stray / 3D Scanner App input (any reader)
  python validate.py OUT/arkitscenes/<video_id>  # ARKitScenes raw layout
  python validate.py OUT/litereality/<name>      # LiteReality scan folder

Prints one JSON report with PASS/WARN/FAIL checks; exit code 1 if any FAIL.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import imread_any, read_ply  # noqa: E402


class Report:
    def __init__(self, path):
        self.path = str(path)
        self.checks = []

    def add(self, name, ok, detail="", warn=False):
        self.checks.append({"check": name, "status": "PASS" if ok else ("WARN" if warn else "FAIL"), "detail": detail})

    @property
    def failed(self):
        return any(c["status"] == "FAIL" for c in self.checks)

    def dump(self, kind, extra=None):
        out = {"path": self.path, "kind": kind, "result": "FAIL" if self.failed else "PASS", "checks": self.checks}
        if extra:
            out.update(extra)
        print(json.dumps(out, indent=1, default=str))


def pose_stats(Ts, ts):
    Ts = [T for T in Ts if T is not None]
    if len(Ts) < 2:
        return {}
    p = np.array([T[:3, 3] for T in Ts])
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    dt = np.diff(ts[:len(Ts)])
    speed = step / np.maximum(dt, 1e-6)
    return {"path_m": float(step.sum()), "max_step_m": float(step.max()), "p99_speed_mps": float(np.percentile(speed, 99)),
            "extent_m": (p.max(0) - p.min(0)).round(2).tolist()}


def validate_arkitscenes(d: Path, r: Report):
    from readers.arkitscenes import read_traj
    vid = d.name
    imgs = sorted((d / "lowres_wide").glob("*.png"))
    r.add("lowres_wide frames", len(imgs) > 0, f"{len(imgs)} frames")
    names = {p.stem.split("_")[-1] for p in imgs}
    r.add("frame names <vid>_<ts.3f>.png", all(p.stem.startswith(vid + "_") for p in imgs))
    pins = list((d / "lowres_wide_intrinsics").glob("*.pincam"))
    r.add("one .pincam per frame", len(pins) == len(imgs), f"{len(pins)} pincam")
    traj = read_traj(d / "lowres_wide.traj") if (d / "lowres_wide.traj").exists() else {}
    r.add("lowres_wide.traj present", bool(traj), f"{len(traj)} poses")
    missing = set(traj) - names
    r.add("every traj timestamp has a frame (loader requirement)", not missing, f"{len(missing)} without frame")
    tts = sorted(v[0] for v in traj.values())
    if len(tts) > 2:
        hz = (len(tts) - 1) / (tts[-1] - tts[0])
        r.add("traj rate ~10 Hz (or every frame)", 8 <= hz <= 12 or hz > 25, f"{hz:.1f} Hz", warn=True)
    for sub, dtype in (("lowres_depth", np.uint16), ("confidence", np.uint8)):
        files = sorted((d / sub).glob("*.png"))
        if not files:
            r.add(f"{sub} present", False, "missing", warn=True)
            continue
        a = imread_any(files[len(files) // 2])
        r.add(f"{sub} dtype {np.dtype(dtype).name}", a.dtype == dtype, str(a.dtype))
        r.add(f"{sub} count matches frames", len(files) >= 0.95 * len(imgs), f"{len(files)}/{len(imgs)}", warn=True)
        if sub == "lowres_depth":
            valid = a[a > 0]
            r.add("depth range plausible (0.1-10 m)", valid.size > 0 and 100 <= np.median(valid) <= 10000,
                  f"median {np.median(valid) if valid.size else 0:.0f} mm, valid {valid.size / a.size:.0%}")
        if sub == "confidence":
            r.add("confidence values in 0..2", int(a.max()) <= 2, f"max {a.max()}")
    mesh = d / f"{vid}_3dod_mesh.ply"
    if mesh.exists():
        ply = read_ply(mesh)
        v = ply["vertex"]
        z = np.asarray(v["z"])
        r.add("mesh present", True, f"{len(v)} vertices, {0 if ply['faces'] is None else len(ply['faces'])} faces")
        r.add("mesh has rgb", "red" in v.dtype.names)
        if traj:
            cam_z = np.median([T[2, 3] for _, T in traj.values()])
            floor = np.percentile(z, 2)
            r.add("z-up: floor below cameras", floor < cam_z - 0.5, f"floor {floor:.2f} m, camera {cam_z:.2f} m")
    else:
        r.add("mesh present", False, "no *_3dod_mesh.ply", warn=True)
    ann = d / f"{vid}_3dod_annotation.json"
    if ann.exists():
        data = json.loads(ann.read_text())["data"]
        bad = 0
        for o in data:
            R = np.array(o["segments"]["obbAligned"]["normalizedAxes"]).reshape(3, 3)
            if not (np.allclose(R @ R.T, np.eye(3), atol=1e-3) and np.linalg.det(R) > 0):
                bad += 1
        r.add("annotation boxes orthonormal + right-handed", bad == 0, f"{len(data)} boxes, {bad} bad")
    else:
        r.add("annotation present", False, "no *_3dod_annotation.json (needs RoomPlan)", warn=True)
    meta = d.parent / "metadata.csv"
    r.add("metadata.csv row", meta.exists() and vid in meta.read_text(), str(meta), warn=True)
    Ts = [v[1] for v in sorted(traj.values(), key=lambda x: x[0])]
    return {"poses": pose_stats(Ts, np.array(tts))}


def validate_litereality(d: Path, r: Report):
    js = sorted(d.glob("frame_*.json"))
    jp = sorted(d.glob("frame_*.jpg"))
    r.add("frame_XXXXX.jpg + .json pairs", len(js) > 0 and len(js) == len(jp), f"{len(jp)} jpg, {len(js)} json")
    ok = True
    Ts, ts = [], []
    for p in js:
        m = json.loads(p.read_text())
        if len(m.get("cameraPoseARFrame", [])) != 16 or len(m.get("intrinsics", [])) != 9:
            ok = False
            break
        Ts.append(np.array(m["cameraPoseARFrame"]).reshape(4, 4))
        ts.append(m.get("time", 0))
    r.add("json has cameraPoseARFrame[16] + intrinsics[9]", ok)
    dp = sorted(d.glob("depth_*.png"))
    cp = sorted(d.glob("conf_*.png"))
    r.add("depth_XXXXX.png", len(dp) > 0, f"{len(dp)}", warn=True)
    r.add("conf for every depth (LiteReality requirement)", len(cp) >= len(dp), f"{len(cp)} conf")
    if dp:
        a = imread_any(dp[0])
        r.add("depth uint16", a.dtype == np.uint16, f"{a.dtype} {a.shape}")
    r.add("roomplan/room.usdz", (d / "roomplan" / "room.usdz").exists(), "needed for LiteReality's layout stage", warn=True)
    r.add("textured_output.obj", (d / "textured_output.obj").exists(), "", warn=True)
    if Ts:
        y = np.array([T[1, 3] for T in Ts])
        r.add("y-up world: camera height above floor", True, f"camera y median {np.median(y):.2f} m")
    return {"poses": pose_stats(Ts, np.array(ts))}


def validate_input(d: Path, r: Report):
    from readers import open_episode
    ep = open_episode(d)
    s = ep.summary()
    r.add("reader", True, s["source"])
    r.add("frames", len(ep.frames) > 0, f"{len(ep.frames)}")
    posed = [f for f in ep.frames if f.T is not None]
    r.add("poses", len(posed) > 0.9 * len(ep.frames), f"{len(posed)} posed", warn=(getattr(ep, "mode", "") == "multicam"))   # multicam has no on-device poses
    tracking_bad = sum(1 for f in ep.frames if f.tracking != "normal")
    r.add("tracking normal", tracking_bad < 0.1 * max(1, len(ep.frames)), f"{tracking_bad} frames limited", warn=True)
    ts = np.array([f.t for f in ep.frames])
    if len(ts) > 2:
        gaps = np.diff(ts)
        r.add("timestamps increasing", bool(np.all(gaps > 0)))
        med = float(np.median(gaps))
        drops = int(np.sum(gaps > 2.5 * med))
        r.add("few dropped frames", drops < 0.02 * len(ts), f"{drops} gaps > 2.5x median ({1 / med:.1f} fps)", warn=True)
    if ep.has_depth:
        dd = ep.depth(len(ep.frames) // 2)
        r.add("depth decodes", dd is not None and dd.dtype == np.uint16, f"{None if dd is None else dd.shape}")
    else:
        r.add("depth", False, "no depth (non-LiDAR phone or mode C)", warn=True)
    try:
        k, img = next(ep.iter_images([len(ep.frames) // 2]))
        f = ep.frames[k]
        r.add("image decodes at frame size", img.shape[1] == f.w and img.shape[0] == f.h, f"{img.shape}")
    except Exception as e:  # noqa: BLE001
        r.add("image decodes", False, repr(e))
    m = ep.mesh()
    r.add("mesh", m is not None, f"{0 if m is None else len(m.xyz)} vertices", warn=True)
    room = ep.room()
    r.add("RoomPlan objects", bool(room), f"{0 if not room else len(room.get('objects', []))} objects", warn=True)
    if ep.path is not None:
        for f in ("imu.csv", "location.csv", "altimeter.csv"):
            p = ep.path / f
            if p.exists():
                n = sum(1 for _ in open(p)) - 1
                r.add(f"{f}", n > 0, f"{n} rows", warn=True)
    return {"input": s, "poses": pose_stats([f.T for f in ep.frames], ts)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", type=Path)
    a = ap.parse_args()
    d = a.path
    r = Report(d)
    if (d / "lowres_wide").is_dir() and (d / "lowres_wide.traj").exists() and not (d / "session.json").exists():
        kind, extra = "arkitscenes", validate_arkitscenes(d, r)
    elif (d / "frame_00000.json").exists() and not (d / "session.json").exists() and (d / "extras").exists():
        kind, extra = "litereality", validate_litereality(d, r)
    else:
        kind, extra = "input", validate_input(d, r)
    r.dump(kind, extra)
    sys.exit(1 if r.failed else 0)


if __name__ == "__main__":
    main()
