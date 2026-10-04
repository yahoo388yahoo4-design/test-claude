#!/usr/bin/env python3
"""Offline 6-DoF poses for captures without ARKit tracking (mode C multicam, or any video).

Structure-from-motion (pycolmap: SIFT + sequential matching + incremental mapping), then
  * metric scale: median ratio of LiDAR depth to SfM point depth (lidar_rgb stream shares the LiDAR camera),
  * gravity: rotate the SfM world so the IMU gravity (imu.csv, device frame) is ARKit -y,
  * axes: output poses are ARKit camera-to-world (y-up), so the result is a normal r2s session that
    convert.py turns into ARKitScenes / LiteReality.

  python recover_poses.py SESSION --cam lidar_rgb --fps 5 --out SESSION_posed
  python recover_poses.py ARKITSCENES_EPISODE --fps 3 --out /tmp/x --eval   # accuracy test vs ARKit poses

Needs: pip install pycolmap opencv-python-headless numpy scipy
"""
from __future__ import annotations

import argparse
import bisect
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import F_CV_ARKIT, K_matrix, deflate, imwrite_rgb, inflate, rot_angle_deg  # noqa: E402
from readers.base import iter_video  # noqa: E402

# iOS device frame (portrait: x right, y up, z out of screen) -> ARKit camera frame of the back camera
# (sensor landscape: x = device -y, y = device +x, z = device +z)
R_DEV_TO_ARCAM = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)


class Sample:
    def __init__(self, t, img, K, depth=None, up_cam_arkit=None):
        self.t, self.img, self.K, self.depth, self.up = t, img, K, depth, up_cam_arkit


def load_imu_gravity(path: Path):
    if not path.exists():
        return None
    a = np.genfromtxt(path, delimiter=",", names=True)
    return a["t"], np.stack([a["grx"], a["gry"], a["grz"]], 1)


def samples_from_multicam(session: Path, cam: str, fps: float) -> list[Sample]:
    cams = session / "cams"
    recs = [json.loads(line) for line in (cams / f"{cam}.jsonl").read_text().splitlines() if line.strip()]
    calib = json.loads((cams / "calibration.json").read_text()) if (cams / "calibration.json").exists() else None
    dl = []
    if cam == "lidar_rgb" and (cams / "lidar_depth.jsonl").exists():
        dl = [json.loads(line) for line in (cams / "lidar_depth.jsonl").read_text().splitlines() if line.strip()]
    dts = [d["t"] for d in dl]
    blob = open(cams / "lidar_depth.zlib.bin", "rb") if dl else None
    imu = load_imu_gravity(session / "imu.csv")
    picked, last = [], -1e9
    for r in recs:
        if r["t"] - last >= 1.0 / fps - 1e-3:
            picked.append(r)
            last = r["t"]
    out = []
    by_index = {r["i"]: r for r in picked}
    v = cams / f"{cam}.mov"
    v = v if v.exists() else cams / f"{cam}.mp4"   # Android writes .mp4
    for vi, img in iter_video(v, by_index.keys()):
        r = by_index[vi]
        h, w = img.shape[:2]
        if r.get("K"):
            K = K_matrix(*r["K"])
        elif calib:
            K = np.array(calib["intrinsics"], float)
            rw, rh = calib["reference_dims"]
            K[0] *= w / rw
            K[1] *= h / rh
        else:
            K = None
        depth = None
        if dts:
            j = bisect.bisect_left(dts, r["t"])
            j = min(range(max(0, j - 1), min(len(dts), j + 1)), key=lambda k: abs(dts[k] - r["t"]))
            if abs(dts[j] - r["t"]) < 0.02:
                d = dl[j]
                blob.seek(d["d"][0])
                depth = np.frombuffer(inflate(blob.read(d["d"][1])), "<u2").reshape(d["h"], d["w"]).copy()
        up = None
        if imu is not None:
            k = int(np.clip(np.searchsorted(imu[0], r["t"]), 0, len(imu[0]) - 1))
            g_dev = imu[1][k]
            up = -(R_DEV_TO_ARCAM @ g_dev)
            up /= np.linalg.norm(up) + 1e-12
        out.append(Sample(r["t"], img, K, depth, up))
    return out


def samples_from_episode(ep, fps: float) -> tuple[list[Sample], list]:
    """Any reader Episode (for testing against ARKit poses). Gravity is taken from the ARKit pose,
    standing in for the IMU (ARKit's world is gravity-aligned)."""
    picked, last = [], -1e9
    for k, f in enumerate(ep.frames):
        if f.t - last >= 1.0 / fps - 1e-3 and f.T is not None:
            picked.append(k)
            last = f.t
    out, gt = [], []
    for k, img in ep.iter_images(picked):
        f = ep.frames[k]
        up = f.T[:3, :3].T @ np.array([0, 1.0, 0])
        out.append(Sample(f.t, img, f.K, ep.depth(k) if ep.has_depth else None, up))
        gt.append(f.T)
    return out, gt


def run_sfm(samples: list[Sample], work: Path):
    import pycolmap
    img_dir = work / "images"
    if work.exists():
        shutil.rmtree(work)
    img_dir.mkdir(parents=True)
    names = []
    for i, s in enumerate(samples):
        n = f"{i:06d}.jpg"
        imwrite_rgb(img_dir / n, s.img, 95)
        names.append(n)
    db = work / "database.db"
    ro = pycolmap.ImageReaderOptions()
    K = samples[0].K
    if K is not None:
        ro.camera_model = "PINHOLE"
        ro.camera_params = f"{K[0, 0]},{K[1, 1]},{K[0, 2]},{K[1, 2]}"
    pycolmap.extract_features(db, img_dir, image_names=names, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=ro)
    po = pycolmap.SequentialPairingOptions()
    po.overlap = 15
    po.quadratic_overlap = True
    pycolmap.match_sequential(db, pairing_options=po)
    opts = pycolmap.IncrementalPipelineOptions()
    if K is not None:
        opts.ba_refine_focal_length = False
        opts.ba_refine_principal_point = False
        opts.ba_refine_extra_params = False
    maps = pycolmap.incremental_mapping(db, img_dir, work / "sparse", options=opts)
    if not maps:
        raise RuntimeError("SfM failed: no reconstruction")
    rec = max(maps.values(), key=lambda r: r.num_reg_images())
    poses = {}
    for img in rec.images.values():
        cfw = img.cam_from_world() if callable(img.cam_from_world) else img.cam_from_world
        M = np.asarray(cfw.matrix())  # 3x4 world->cam (OpenCV)
        hp = img.has_pose
        if not (hp() if callable(hp) else hp):
            continue
        idx = int(Path(img.name).stem)
        pts = []
        for p2 in img.points2D:
            hp3 = p2.has_point3D
            if hp3() if callable(hp3) else hp3:
                X = rec.points3D[p2.point3D_id].xyz
                pts.append((p2.xy, M[:, :3] @ X + M[:, 3]))
        poses[idx] = (M, pts)
    return poses, rec


def _min_rotation(a, b):
    """Smallest rotation taking unit vector a to unit vector b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    s, c = np.linalg.norm(v), float(a @ b)
    if s < 1e-12:
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def run_rgbd_vo(samples: list[Sample]):
    """Metric RGB-D visual odometry for low-texture rooms: corners (GFTT) tracked frame-to-frame with
    pyramidal KLT (forward-backward checked), lifted to 3-D with the previous frame's LiDAR depth,
    current pose from PnP-RANSAC. Registers every frame (constant-velocity fill when PnP fails) and
    drifts slowly; there is no loop closure. Returns {idx: (M world->cam OpenCV 3x4, [])}."""
    import cv2
    lk = dict(winSize=(21, 21), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    def lift(s, pts):
        d = s.depth
        h, w = s.img.shape[:2]
        dh, dw = d.shape
        u = np.clip((pts[:, 0] * dw / w).astype(int), 0, dw - 1)
        v = np.clip((pts[:, 1] * dh / h).astype(int), 0, dh - 1)
        z = d[v, u].astype(np.float64) / 1000.0
        K = s.K
        X = np.stack([(pts[:, 0] - K[0, 2]) / K[0, 0] * z, (pts[:, 1] - K[1, 2]) / K[1, 1] * z, z], 1)
        return X, (z > 0.1) & (z < 8.0)

    poses = {}
    T_wc = np.eye(4)            # camera-to-world, OpenCV axes
    last_rel = np.eye(4)
    fails = 0
    prev_gray = None
    up_world = None
    for i, s in enumerate(samples):
        gray = cv2.cvtColor(s.img, cv2.COLOR_RGB2GRAY)
        if prev_gray is not None:
            ok = False
            p0 = cv2.goodFeaturesToTrack(prev_gray, maxCorners=1500, qualityLevel=0.001, minDistance=6, blockSize=7)
            if p0 is not None and samples[i - 1].depth is not None:
                p1, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None, **lk)
                p0b, st2, _ = cv2.calcOpticalFlowPyrLK(gray, prev_gray, p1, None, **lk)
                good = (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((p0 - p0b).reshape(-1, 2), axis=1) < 1.0)
                a0, a1 = p0.reshape(-1, 2)[good].astype(np.float64), p1.reshape(-1, 2)[good].astype(np.float64)
                X, valid = lift(samples[i - 1], a0)
                if valid.sum() >= 12:
                    okp, rvec, tvec, inl = cv2.solvePnPRansac(X[valid], a1[valid], s.K, None, reprojectionError=2.0,
                                                              iterationsCount=300, confidence=0.999)
                    if okp and inl is not None and len(inl) >= 15:
                        idx = inl.ravel()
                        okr, rvec, tvec = cv2.solvePnP(X[valid][idx], a1[valid][idx], s.K, None, rvec, tvec,
                                                       useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
                        R, _ = cv2.Rodrigues(rvec)
                        T_cp = np.eye(4)
                        T_cp[:3, :3], T_cp[:3, 3] = R, tvec.ravel()
                        last_rel = np.linalg.inv(T_cp)
                        ok = True
            if not ok:
                fails += 1
            T_wc = T_wc @ last_rel
            if s.up is not None and up_world is not None:
                # remove roll/pitch drift: the IMU says where 'up' is in this camera
                T_wc[:3, :3] = _min_rotation(T_wc[:3, :3] @ (F_CV_ARKIT @ s.up), up_world) @ T_wc[:3, :3]
        elif s.up is not None:
            up_world = F_CV_ARKIT @ s.up   # world = first camera frame (OpenCV axes)
        poses[i] = (np.linalg.inv(T_wc)[:3], [])
        prev_gray = gray
    print(f"rgbd-vo: {len(samples)} frames, {fails} PnP failures (constant-velocity fill)")
    return poses


def metric_scale(samples, poses) -> tuple[float, int]:
    ratios = []
    for idx, (M, pts) in poses.items():
        d = samples[idx].depth
        if d is None:
            continue
        h, w = samples[idx].img.shape[:2]
        dh, dw = d.shape
        for xy, pc in pts:
            if pc[2] <= 0:
                continue
            u, v = int(xy[0] * dw / w), int(xy[1] * dh / h)
            if 0 <= u < dw and 0 <= v < dh and d[v, u] > 0:
                ratios.append(d[v, u] / 1000.0 / pc[2])
    if len(ratios) < 20:
        return 1.0, len(ratios)
    return float(np.median(ratios)), len(ratios)


def gravity_rotation(samples, poses) -> np.ndarray:
    """Rotation W (SfM world -> ARKit-like world) so that measured 'up' maps to +y."""
    ups = []
    for idx, (M, _) in poses.items():
        up = samples[idx].up
        if up is None:
            continue
        up_cv = F_CV_ARKIT @ up
        ups.append(M[:, :3].T @ up_cv)  # up direction in SfM world
    if not ups:
        return np.eye(3)
    u = np.mean(ups, 0)
    u /= np.linalg.norm(u)
    y = np.array([0, 1.0, 0])
    v = np.cross(u, y)
    s, c = np.linalg.norm(v), float(u @ y)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def to_arkit_poses(samples, poses, scale, W):
    out = {}
    first = min(poses)
    origin = None
    for idx in sorted(poses):
        M = poses[idx][0]
        R_c2w_cv = M[:, :3].T
        C = -R_c2w_cv @ M[:, 3]
        T = np.eye(4)
        T[:3, :3] = W @ R_c2w_cv @ F_CV_ARKIT
        T[:3, 3] = W @ (C * scale)
        if idx == first:
            origin = T[:3, 3].copy()
        out[idx] = T
    for T in out.values():
        T[:3, 3] -= origin
    return out


def write_posed_session(samples, Ts, out: Path, src_meta: dict):
    out.mkdir(parents=True, exist_ok=True)
    (out / "frames").mkdir(exist_ok=True)
    dfh = open(out / "depth.zlib.bin", "wb")
    with open(out / "frames.jsonl", "w") as jf:
        for n, idx in enumerate(sorted(Ts)):
            s = samples[idx]
            h, w = s.img.shape[:2]
            imwrite_rgb(out / "frames" / f"{n:06d}.png", s.img)
            rec = {"i": n, "t": s.t, "w": w, "h": h, "K": [s.K[0, 0], s.K[1, 1], s.K[0, 2], s.K[1, 2]],
                   "T": Ts[idx].reshape(-1).tolist(), "track": "normal", "d": None, "c": None, "pose_source": "sfm"}
            if s.depth is not None:
                b = deflate(s.depth.astype("<u2").tobytes())
                rec["d"] = [dfh.tell(), len(b)]
                rec["dh"], rec["dw"] = s.depth.shape
                dfh.write(b)
            jf.write(json.dumps(rec) + "\n")
    dfh.close()
    meta = {"format": "r2s-capture", "version": 1, "mode": "arkit_rgbd", "platform": "sfm",
            "derived_from": src_meta, "video": {"file": "video.mov"}, "notes": "poses from recover_poses.py"}
    (out / "session.json").write_text(json.dumps(meta, indent=1))


def evaluate(gt: list, Ts: dict) -> dict:
    """Compare against ARKit poses after removing the free yaw + translation (scale/gravity were measured)."""
    idx = sorted(Ts)
    P = np.array([Ts[i][:3, 3] for i in idx])
    G = np.array([gt[i][:3, 3] for i in idx])
    G0 = G - G[0]
    # best yaw about +y
    A, B = P[:, [0, 2]], G0[:, [0, 2]]
    Hm = A.T @ B
    U, _, Vt = np.linalg.svd(Hm)
    R2 = Vt.T @ U.T
    if np.linalg.det(R2) < 0:
        Vt[-1] *= -1
        R2 = Vt.T @ U.T
    Ry = np.eye(3)
    Ry[0, 0], Ry[0, 2], Ry[2, 0], Ry[2, 2] = R2[0, 0], R2[0, 1], R2[1, 0], R2[1, 1]
    Pa = P @ Ry.T
    err = np.linalg.norm(Pa - G0, axis=1)
    rot = [rot_angle_deg(Ry @ Ts[i][:3, :3], gt[i][:3, :3]) for i in idx]
    path = np.linalg.norm(np.diff(G0, axis=0), axis=1).sum()
    # Sim3-free scale check: ratio of path lengths
    pl = np.linalg.norm(np.diff(P, axis=0), axis=1).sum()
    tilt = [np.degrees(np.arccos(np.clip((Ts[i][:3, :3].T @ [0, 1, 0]) @ (gt[i][:3, :3].T @ [0, 1, 0]), -1, 1))) for i in idx]
    return {"registered": len(idx), "ate_rmse_m": float(np.sqrt((err ** 2).mean())), "ate_max_m": float(err.max()),
            "rot_err_deg_median": float(np.median(rot)), "gravity_tilt_err_deg_median": float(np.median(tilt)),
            "scale_ratio_path": float(pl / path) if path else None, "gt_path_m": float(path)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", type=Path)
    ap.add_argument("--cam", default="lidar_rgb", help="mode C stream to pose (lidar_rgb gives metric scale)")
    ap.add_argument("--fps", type=float, default=5.0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--eval", action="store_true", help="input has ARKit poses: report accuracy")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--method", default="sfm", choices=["sfm", "rgbd"],
                    help="sfm: pycolmap SfM + LiDAR scale (best on textured scenes, may fragment); "
                         "rgbd: LiDAR-depth visual odometry (registers every frame, drifts)")
    ap.add_argument("--asset", default="lowres_wide", help="ARKitScenes input only: lowres_wide | vga_wide | wide")
    a = ap.parse_args()
    gt = None
    if (a.session / "cams").is_dir():
        samples = samples_from_multicam(a.session, a.cam, a.fps)
        src = {"session": str(a.session), "cam": a.cam}
    else:
        from readers import detect, open_episode
        if detect(a.session) == "arkitscenes":
            from readers.arkitscenes import ARKitScenesEpisode
            ep = ARKitScenesEpisode(a.session, asset=a.asset)
        else:
            ep = open_episode(a.session)
        if a.max_frames:
            ep.frames = ep.frames[:a.max_frames]
        samples, gt = samples_from_episode(ep, a.fps)
        src = {"session": str(a.session), "reader": ep.source}
    print(f"{len(samples)} frames at {a.fps} fps; depth on {sum(s.depth is not None for s in samples)}; "
          f"gravity on {sum(s.up is not None for s in samples)}")
    if a.method == "sfm":
        poses, _ = run_sfm(samples, a.out / "_sfm")
        scale, nr = metric_scale(samples, poses)
    else:
        poses = run_rgbd_vo(samples)
        scale, nr = 1.0, 0
    W = gravity_rotation(samples, poses)
    Ts = to_arkit_poses(samples, poses, scale, W)
    write_posed_session(samples, Ts, a.out, src)
    rep = {"method": a.method, "frames": len(samples), "registered": len(poses), "scale": scale, "scale_samples": nr}
    if gt is not None and a.eval:
        rep["eval_vs_arkit"] = evaluate(gt, Ts)
    print(json.dumps(rep, indent=1))
    (a.out / "recover_report.json").write_text(json.dumps(rep, indent=1))


if __name__ == "__main__":
    main()
