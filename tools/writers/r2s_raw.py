"""Write any Episode in the r2s raw session format (FORMAT.md).

Used to fabricate test sessions from ARKitScenes episodes (round-trip test) and as an executable
reference for other writers (e.g. the Android app). Images go to frames/<i>.png (lossless) or, with
video=True, to video.mov via OpenCV (codec mp4v; the phone writes HEVC).
"""
from __future__ import annotations

import json
import shutil
from pathlib import Path

import numpy as np

from common import deflate, imwrite_rgb, write_r2s_mesh_ply


def write_r2s_raw(ep, out: Path, video: bool = False, mode: str = "arkit_rgbd", log=print) -> dict:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    f0 = ep.frames[0]
    vw = None
    if video:
        import cv2
        vw = cv2.VideoWriter(str(out / "video.mov"), cv2.VideoWriter_fourcc(*"mp4v"), 60, (f0.w, f0.h))
    else:
        (out / "frames").mkdir(exist_ok=True)
    dfh = open(out / "depth.zlib.bin", "wb") if ep.has_depth else None
    cfh = open(out / "conf.zlib.bin", "wb") if ep.has_depth else None
    n_depth = 0
    dsize = None
    with open(out / "frames.jsonl", "w") as jf:
        for k, img in ep.iter_images(range(len(ep.frames))):
            f = ep.frames[k]
            if vw is not None:
                vw.write(img[:, :, ::-1].copy())
            else:
                imwrite_rgb(out / "frames" / f"{k:06d}.png", img)
            rec = {"i": k, "t": f.t, "w": f.w, "h": f.h,
                   "K": [f.K[0, 0], f.K[1, 1], f.K[0, 2], f.K[1, 2]],
                   "T": f.T.reshape(-1).tolist() if f.T is not None else None, "track": f.tracking,
                   "d": None, "c": None}
            for key in ("exp", "eo", "iso", "amb", "ct", "wm"):
                if f.meta.get(key) is not None:
                    rec[key] = f.meta[key]
            if dfh is not None:
                dep = ep.depth(k)
                if dep is not None:
                    b = deflate(dep.astype("<u2").tobytes())
                    rec["d"] = [dfh.tell(), len(b)]
                    dfh.write(b)
                    rec["dh"], rec["dw"] = dep.shape
                    dsize = dep.shape
                    n_depth += 1
                    conf = ep.conf(k)
                    if conf is not None:
                        b = deflate(conf.astype(np.uint8).tobytes())
                        rec["c"] = [cfh.tell(), len(b)]
                        cfh.write(b)
            jf.write(json.dumps(rec) + "\n")
    if vw is not None:
        vw.release()
    for fh in (dfh, cfh):
        if fh:
            fh.close()
    mesh = ep.mesh()
    if mesh is not None and mesh.faces is not None:
        write_r2s_mesh_ply(out / "mesh.ply", mesh.xyz, mesh.faces, cls=mesh.face_cls)
    room = ep.room()
    if room:
        (out / "roomplan").mkdir(exist_ok=True)
        (out / "roomplan" / "objects.json").write_text(json.dumps(room, indent=1))
    usdz = ep.roomplan_usdz()
    if usdz:
        (out / "roomplan").mkdir(exist_ok=True)
        shutil.copy2(usdz, out / "roomplan" / "room.usdz")
    t0, t1 = ep.frames[0].t, ep.frames[-1].t
    session = {"format": "r2s-capture", "version": 1, "mode": mode, "platform": "converted",
               "device": f"converted-from-{ep.source}", "start_uptime": t0, "end_uptime": t1,
               "video": {"file": "video.mov", "width": f0.w, "height": f0.h,
                         "fps": round((len(ep.frames) - 1) / max(t1 - t0, 1e-9), 2), "codec": "mp4v" if video else "png"},
               "depth": {"width": dsize[1], "height": dsize[0], "unit": "mm", "dtype": "uint16",
                         "compression": "raw-deflate"} if dsize else None,
               "counts": {"frames": len(ep.frames), "depth": n_depth}, "source": ep.summary()}
    (out / "session.json").write_text(json.dumps(session, indent=1))
    info = {"out": str(out), "frames": len(ep.frames), "depth": n_depth}
    log(json.dumps(info))
    return info
