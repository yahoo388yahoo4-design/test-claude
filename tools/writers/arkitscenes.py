"""Write an Episode in the ARKitScenes *raw* layout (github.com/apple/ARKitScenes, DATA.md):

<out>/<video_id>/
  lowres_wide/<vid>_<ts>.png            RGB at depth resolution (256x192), every frame
  lowres_wide_intrinsics/<vid>_<ts>.pincam   "w h fx fy cx cy"
  lowres_depth/<vid>_<ts>.png           uint16 mm          (if depth)
  confidence/<vid>_<ts>.png             uint8 0..2         (if confidence)
  lowres_wide.traj                      "ts rx ry rz tx ty tz" world-to-camera, z-up world, OpenCV camera
  wide/ + wide_intrinsics/              full-res RGB on the traj frames          (--wide)
  vga_wide/ + vga_wide_intrinsics/      640x480 at ~30 Hz                        (--vga)
  <vid>_3dod_mesh.ply                   z-up, vertex-coloured mesh (or fused point cloud)
  <vid>_3dod_annotation.json            oriented boxes from RoomPlan objects     (if room)
  extras/                               everything the format has no slot for
<out>/metadata.csv                       one row per video_id (appended)
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from common import (A_YUP_TO_ZUP, arkit_c2w_to_zup_cv, imwrite_any, imwrite_rgb, resize, rodrigues, scale_K,
                    ts_name, write_ply_xyzrgba)
from .meshtools import color_vertices, fuse_points

ARKITSCENES_CLASSES = ["cabinet", "refrigerator", "shelf", "stove", "bed", "sink", "washer", "toilet", "bathtub",
                       "oven", "dishwasher", "fireplace", "stool", "chair", "table", "tv_monitor", "sofa"]
ROOMPLAN_TO_ARKITSCENES = {"storage": "cabinet", "refrigerator": "refrigerator", "stove": "stove", "bed": "bed",
                           "sink": "sink", "washerDryer": "washer", "toilet": "toilet", "bathtub": "bathtub",
                           "oven": "oven", "dishwasher": "dishwasher", "fireplace": "fireplace", "chair": "chair",
                           "table": "table", "television": "tv_monitor", "sofa": "sofa"}


def select_by_rate(ts: list[float], hz: float) -> list[int]:
    """Pick indices so that picked timestamps are ~1/hz apart, always on real frame timestamps."""
    if not ts:
        return []
    if not hz or hz <= 0:
        return list(range(len(ts)))
    dt = 1.0 / hz
    gaps = np.diff(ts)
    eps = 0.5 * float(np.median(gaps)) if len(gaps) else 0.0
    out = [0]
    nxt = ts[0] + dt
    for k in range(1, len(ts)):
        if ts[k] >= nxt - eps:
            out.append(k)
            nxt = max(nxt + dt, ts[k] + dt - eps)
    return out


def traj_line(name: str, T_c2w_arkit: np.ndarray) -> str:
    Tz = arkit_c2w_to_zup_cv(T_c2w_arkit)
    E = np.linalg.inv(Tz)  # world-to-camera
    r = rodrigues(E[:3, :3])
    t = E[:3, 3]
    # name + "00000": the loader keys poses by f"{round(ts,3):.3f}", so this matches file names exactly
    return f"{name}00000 {r[0]:.10f} {r[1]:.10f} {r[2]:.10f} {t[0]:.8f} {t[1]:.8f} {t[2]:.8f}"


def room_to_annotation(room: dict, video_id: str) -> tuple[dict, list]:
    data, skipped = [], []
    for k, o in enumerate(room.get("objects", [])):
        cat = o.get("category", "")
        label = ROOMPLAN_TO_ARKITSCENES.get(cat) or (cat if cat in ARKITSCENES_CLASSES else None)
        if label is None:
            skipped.append(cat)
            continue
        T = np.array(o["T"], dtype=np.float64).reshape(4, 4)
        R = T[:3, :3] / np.linalg.norm(T[:3, :3], axis=0, keepdims=True)
        w, h, d = [float(x) for x in o["dims"]]
        rows = np.stack([A_YUP_TO_ZUP @ R[:, 0], A_YUP_TO_ZUP @ (-R[:, 2]), A_YUP_TO_ZUP @ R[:, 1]])
        obb = {"centroid": (A_YUP_TO_ZUP @ T[:3, 3]).tolist(), "axesLengths": [w, d, h],
               "normalizedAxes": rows.reshape(-1).tolist()}
        data.append({"uid": str(o.get("id", k)), "label": label, "modelId": "roomplan", "children": [],
                     "objectId": k + 1, "partId": k + 1, "attributes": {"roomplan_category": cat,
                                                                         "confidence": o.get("confidence")},
                     "segments": {"obb": obb, "obbAligned": obb}})
    ann = {"data": data, "stats": {"labelCount": len(data)}, "confirm": False, "skipped": False,
           "attributes": {"tags": [], "attributes": {}}, "mesh_quality": [], "skipped_reason": "",
           "label_version": "r2s-roomplan-1", "video_id": video_id}
    return ann, skipped


def sky_direction(frames) -> str:
    """Which image side the sky is on, from gravity: average world-up in OpenCV camera coords."""
    ups = []
    for f in frames:
        if f.T is None:
            continue
        Rz = arkit_c2w_to_zup_cv(f.T)[:3, :3]
        ups.append(Rz.T @ np.array([0, 0, 1.0]))
    if not ups:
        return "NA"
    u = np.mean(ups, 0)
    if abs(u[0]) > abs(u[1]):
        return "Right" if u[0] > 0 else "Left"
    return "Down" if u[1] > 0 else "Up"


def write_arkitscenes(ep, out_root: Path, video_id: str | None = None, traj_hz: float = 10.0, wide: bool = False,
                      vga: bool = False, color_views: int = 120, fold: str = "Training", log=print) -> dict:
    vid = video_id or ep.name
    out = Path(out_root) / vid
    for d in ("lowres_wide", "lowres_wide_intrinsics"):
        (out / d).mkdir(parents=True, exist_ok=True)
    frames = ep.frames
    names = [ts_name(f.t) for f in frames]
    if len(set(names)) != len(names):
        raise ValueError("two frames share a millisecond timestamp; ARKitScenes names would collide")
    posed = [k for k, f in enumerate(frames) if f.T is not None]
    traj_idx = [posed[j] for j in select_by_rate([frames[k].t for k in posed], traj_hz)]
    vga_idx = set(posed[j] for j in select_by_rate([frames[k].t for k in posed], 30.0)) if vga else set()
    wide_set = set(traj_idx) if wide else set()
    has_depth = ep.has_depth
    lw, lh = 256, 192
    if has_depth:
        for k in range(len(frames)):
            d0 = ep.depth(k)
            if d0 is not None:
                lh, lw = d0.shape
                break
    for d in (["lowres_depth", "confidence"] if has_depth else []) + (["wide", "wide_intrinsics"] if wide else []) + \
            (["vga_wide", "vga_wide_intrinsics"] if vga else []):
        (out / d).mkdir(parents=True, exist_ok=True)
    view_set = set(posed[j] for j in np.linspace(0, len(posed) - 1, min(color_views, len(posed))).astype(int)) \
        if posed else set()
    views = []
    n_depth = n_conf = 0
    for k, img in ep.iter_images(range(len(frames))):
        f = frames[k]
        stem = f"{vid}_{names[k]}"
        low = resize(img, lw, lh)
        Kl = scale_K(f.K, lw / f.w, lh / f.h)
        imwrite_rgb(out / "lowres_wide" / f"{stem}.png", low)
        (out / "lowres_wide_intrinsics" / f"{stem}.pincam").write_text(
            f"{lw} {lh} {Kl[0, 0]:.6f} {Kl[1, 1]:.6f} {Kl[0, 2]:.6f} {Kl[1, 2]:.6f}")
        dep = conf = None
        if has_depth:
            dep = ep.depth(k)
            if dep is not None:
                imwrite_any(out / "lowres_depth" / f"{stem}.png", dep.astype(np.uint16))
                n_depth += 1
            conf = ep.conf(k)
            if conf is not None:
                imwrite_any(out / "confidence" / f"{stem}.png", conf.astype(np.uint8))
                n_conf += 1
        if k in wide_set:
            imwrite_rgb(out / "wide" / f"{stem}.png", img)
            (out / "wide_intrinsics" / f"{stem}.pincam").write_text(
                f"{f.w} {f.h} {f.K[0, 0]:.6f} {f.K[1, 1]:.6f} {f.K[0, 2]:.6f} {f.K[1, 2]:.6f}")
        if k in vga_idx:
            vg = resize(img, 640, 480)
            Kv = scale_K(f.K, 640 / f.w, 480 / f.h)
            imwrite_rgb(out / "vga_wide" / f"{stem}.png", vg)
            (out / "vga_wide_intrinsics" / f"{stem}.pincam").write_text(
                f"640 480 {Kv[0, 0]:.6f} {Kv[1, 1]:.6f} {Kv[0, 2]:.6f} {Kv[1, 2]:.6f}")
        if k in view_set:
            views.append((Kl, f.T, low, dep, conf))
    (out / "lowres_wide.traj").write_text("\n".join(traj_line(names[k], frames[k].T) for k in traj_idx) + "\n")

    # mesh
    mesh = ep.mesh()
    mesh_info = None
    if mesh is not None and len(mesh.xyz):
        rgb, seen = (mesh.rgb, 1.0) if mesh.rgb is not None else color_vertices(mesh.xyz, [v[:4] for v in views])
        write_ply_xyzrgba(out / f"{vid}_3dod_mesh.ply", mesh.xyz @ A_YUP_TO_ZUP.T, rgb, mesh.faces)
        mesh_info = {"vertices": len(mesh.xyz), "faces": 0 if mesh.faces is None else len(mesh.faces),
                     "colored_fraction": round(seen, 3), "kind": "mesh" if mesh.faces is not None else "points"}
    elif has_depth and views:
        P, C = fuse_points(views)
        write_ply_xyzrgba(out / f"{vid}_3dod_mesh.ply", P @ A_YUP_TO_ZUP.T, C, None,
                          comment="r2s fused point cloud from LiDAR depth (no faces)")
        mesh_info = {"vertices": len(P), "faces": 0, "kind": "fused_points"}

    # annotation
    room = ep.room()
    ann_info = None
    if room and room.get("objects"):
        ann, skipped = room_to_annotation(room, vid)
        (out / f"{vid}_3dod_annotation.json").write_text(json.dumps(ann))
        ann_info = {"boxes": len(ann["data"]), "skipped_categories": skipped}

    # metadata.csv (append / replace row)
    sky = sky_direction(frames)
    meta_path = Path(out_root) / "metadata.csv"
    cols = ["video_id", "visit_id", "sky_direction", "fold", "has_laser_scanner_point_clouds", "is_in_upsampling",
            "is_in_threedod"]
    rows = []
    if meta_path.exists():
        with open(meta_path) as fh:
            rows = [r for r in csv.DictReader(fh) if r["video_id"] != vid]
    rows.append({"video_id": vid, "visit_id": "NA", "sky_direction": sky, "fold": fold,
                 "has_laser_scanner_point_clouds": "False", "is_in_upsampling": "False",
                 "is_in_threedod": str(ann_info is not None)})
    with open(meta_path, "w", newline="") as fh:
        wr = csv.DictWriter(fh, fieldnames=cols)
        wr.writeheader()
        wr.writerows(rows)
    info = {"out": str(out), "frames": len(frames), "traj_lines": len(traj_idx), "depth": n_depth, "conf": n_conf,
            "lowres": f"{lw}x{lh}", "wide": len(wide_set), "vga": len(vga_idx), "mesh": mesh_info,
            "annotation": ann_info, "sky_direction": sky}
    log(json.dumps(info))
    return info
