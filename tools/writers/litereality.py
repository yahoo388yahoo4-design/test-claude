"""Write an Episode as a LiteReality scan folder (the 3D Scanner App "All Data" layout that
LiteReality's preprocessing reads; github.com/LiteReality/LiteReality "Prepare Data"):

<out>/
  frame_00000.jpg     full-resolution RGB (native landscape)
  frame_00000.json    {"cameraPoseARFrame": 16 floats row-major ARKit camera-to-world, "intrinsics": 9 floats
                       row-major at full resolution, "time", "frame_index", "exposureDuration", ...}
  depth_00000.png     uint16 mm (256x192)
  conf_00000.png      uint8 0..2 (LiteReality requires it whenever depth exists)
  roomplan/room.usdz  RoomPlan CapturedRoom export (copied verbatim)
  textured_output.obj (+ .mtl, .jpg)   scene mesh in the ARKit y-up world (LiteReality reads the vertices)
  extras/             sensor logs etc.
"""
from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np

from common import imwrite_any, imwrite_rgb
from .arkitscenes import select_by_rate
from .meshtools import color_vertices, fuse_points


def write_obj(path: Path, xyz, faces, rgb=None):
    with open(path, "w") as fh:
        fh.write("mtllib textured_output.mtl\nusemtl material0\n")
        if rgb is not None:
            c = rgb / 255.0
            fh.writelines(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.4f} {q[1]:.4f} {q[2]:.4f}\n" for p, q in zip(xyz, c))
        else:
            fh.writelines(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n" for p in xyz)
        if faces is not None:
            fh.writelines(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in faces)
    (path.parent / "textured_output.mtl").write_text("newmtl material0\nKa 1 1 1\nKd 1 1 1\nmap_Kd textured_output.jpg\n")
    imwrite_rgb(path.parent / "textured_output.jpg", np.full((8, 8, 3), 160, np.uint8))


def write_litereality(ep, out: Path, fps: float = 10.0, jpeg_quality: int = 95, color_views: int = 120, log=print) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    posed = [k for k, f in enumerate(ep.frames) if f.T is not None]
    sel = [posed[j] for j in select_by_rate([ep.frames[k].t for k in posed], fps)]
    view_sel = set(sel[j] for j in np.linspace(0, len(sel) - 1, min(color_views, len(sel))).astype(int)) if sel else set()
    views = []
    n_depth = 0
    for n, (k, img) in enumerate(ep.iter_images(sel)):
        f = ep.frames[k]
        imwrite_rgb(out / f"frame_{n:05d}.jpg", img, jpeg_quality)
        meta = {"cameraPoseARFrame": f.T.reshape(-1).tolist(), "intrinsics": f.K.reshape(-1).tolist(),
                "time": f.t, "frame_index": n, "source_index": k, "imageWidth": f.w, "imageHeight": f.h}
        if f.meta.get("exp") is not None:
            meta["exposureDuration"] = f.meta["exp"]
        (out / f"frame_{n:05d}.json").write_text(json.dumps(meta))
        dep = conf = None
        if ep.has_depth:
            dep = ep.depth(k)
            conf = ep.conf(k)
            if dep is not None:
                imwrite_any(out / f"depth_{n:05d}.png", dep.astype(np.uint16))
                if conf is None:  # LiteReality wants a confidence map next to every depth map
                    conf = np.where(dep > 0, 2, 0).astype(np.uint8)
                imwrite_any(out / f"conf_{n:05d}.png", conf.astype(np.uint8))
                n_depth += 1
        if k in view_sel:
            views.append((f.K, f.T, img, dep, conf))
    usdz = ep.roomplan_usdz()
    if usdz:
        (out / "roomplan").mkdir(exist_ok=True)
        shutil.copy2(usdz, out / "roomplan" / "room.usdz")
        js = Path(usdz).parent / "room.json"
        if js.exists():
            shutil.copy2(js, out / "roomplan" / "room.json")
    mesh = ep.mesh()
    mesh_info = None
    if mesh is not None and len(mesh.xyz):
        rgb = mesh.rgb
        if rgb is None and views:
            rgb, _ = color_vertices(mesh.xyz, [v[:4] for v in views])
        write_obj(out / "textured_output.obj", mesh.xyz, mesh.faces, rgb)
        mesh_info = {"vertices": len(mesh.xyz), "faces": 0 if mesh.faces is None else len(mesh.faces)}
    elif views and ep.has_depth:
        P, C = fuse_points(views)
        write_obj(out / "textured_output.obj", P, None, C)
        mesh_info = {"vertices": len(P), "faces": 0, "kind": "fused_points"}
    info = {"out": str(out), "frames": len(sel), "depth": n_depth, "roomplan_usdz": bool(usdz), "mesh": mesh_info}
    if not usdz:
        info["warning"] = "no roomplan/room.usdz: LiteReality needs a RoomPlan capture (mode B, 3D Scanner App or LiteReality app)"
    log(json.dumps(info))
    return info
