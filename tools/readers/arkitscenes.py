"""Reader for an ARKitScenes raw episode folder (lowres_wide/, lowres_depth/, confidence/,
lowres_wide_intrinsics/, lowres_wide.traj, <vid>_3dod_mesh.ply, <vid>_3dod_annotation.json).

Poses are given at ~10 Hz in lowres_wide.traj (world-to-camera, z-up world, OpenCV camera). Frames
whose timestamp has a traj line get that exact pose. With interpolate=True, the other frames get a
slerp/lerp pose between neighbouring traj lines and are flagged meta["interpolated"]=True.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from common import (A_YUP_TO_ZUP, K_matrix, imread_any, ply_rgb, ply_xyz, read_ply, rodrigues_inv,
                    zup_cv_to_arkit_c2w)
from .base import Episode, Frame, Mesh, iter_image_files

# ARKitScenes label -> RoomPlan category (inverse of writers.arkitscenes.ROOMPLAN_TO_ARKITSCENES)
TO_ROOMPLAN = {"cabinet": "storage", "refrigerator": "refrigerator", "stove": "stove", "bed": "bed",
               "sink": "sink", "washer": "washerDryer", "toilet": "toilet", "bathtub": "bathtub",
               "oven": "oven", "dishwasher": "dishwasher", "fireplace": "fireplace", "chair": "chair",
               "table": "table", "tv_monitor": "television", "sofa": "sofa"}


def read_traj(path: Path) -> dict[str, tuple[float, np.ndarray]]:
    """{ "ts.3f": (ts, T_c2w z-up/OpenCV) } exactly like ARKitScenes' TrajStringToMatrix + inverse."""
    out = {}
    for line in Path(path).read_text().splitlines():
        tok = line.split()
        if len(tok) < 7:
            continue
        ts = float(tok[0])
        E = np.eye(4)
        E[:3, :3] = rodrigues_inv([float(x) for x in tok[1:4]])
        E[:3, 3] = [float(x) for x in tok[4:7]]
        out[f"{round(ts, 3):.3f}"] = (ts, np.linalg.inv(E))
    return out


def annotation_to_room(ann: dict) -> dict:
    objs = []
    for o in ann.get("data", []):
        ob = o["segments"]["obbAligned"]
        rows = np.array(ob["normalizedAxes"], dtype=np.float64).reshape(3, 3)
        l0, l1, l2 = ob["axesLengths"]
        c = np.array(ob["centroid"], dtype=np.float64)
        # z-up rows (x, -z_obj, y_obj) -> ARKit object axes (columns x, y(up), z)
        R = np.stack([A_YUP_TO_ZUP.T @ rows[0], A_YUP_TO_ZUP.T @ rows[2], A_YUP_TO_ZUP.T @ (-rows[1])], 1)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = A_YUP_TO_ZUP.T @ c
        objs.append({"id": o.get("uid", str(len(objs))), "category": TO_ROOMPLAN.get(o["label"], o["label"]),
                     "confidence": "high", "dims": [l0, l2, l1], "T": T.reshape(-1).tolist()})
    return {"objects": objs, "walls": [], "doors": [], "windows": [], "openings": [], "floors": []}


class ARKitScenesEpisode(Episode):
    source = "arkitscenes"
    interpolate = True

    @staticmethod
    def detect(path: Path) -> bool:
        return (path / "lowres_wide.traj").exists() and (path / "lowres_wide").is_dir()

    def __init__(self, path: Path, asset: str = "lowres_wide"):
        """asset: lowres_wide (256x192, 60 Hz), vga_wide (640x480, 30 Hz) or wide (1920x1440, 10 Hz).
        Depth/confidence always come from lowres_depth/ (nearest timestamp within 10 ms)."""
        super().__init__()
        from scipy.spatial.transform import Rotation, Slerp
        self.path = Path(path)
        self.asset = asset
        self.name = self.path.name
        traj = read_traj(self.path / "lowres_wide.traj")
        tkeys = sorted(traj.values(), key=lambda x: x[0])
        tts = np.array([x[0] for x in tkeys])
        slerp = Slerp(tts, Rotation.from_matrix(np.stack([x[1][:3, :3] for x in tkeys]))) if len(tts) > 1 else None
        self._files = {}
        self._names = []
        imgs = sorted((self.path / asset).glob("*.png"), key=lambda p: float(p.stem.split("_")[-1]))
        dnames = sorted(p.stem for p in (self.path / "lowres_depth").glob("*.png"))
        dts = np.array([float(n.split("_")[-1]) for n in dnames])
        for p in imgs:
            name = p.stem.split("_")[-1]
            t = float(name)
            pin = self.path / f"{asset}_intrinsics" / f"{p.stem}.pincam"
            if not pin.exists():
                continue
            w, h, fx, fy, cx, cy = [float(x) for x in pin.read_text().split()]
            meta = {}
            if name in traj:
                Tz = traj[name][1]
            elif self.interpolate and slerp is not None and tts[0] <= t <= tts[-1]:
                j = int(np.searchsorted(tts, t))
                a = (t - tts[j - 1]) / (tts[j] - tts[j - 1])
                Tz = np.eye(4)
                Tz[:3, :3] = slerp([t]).as_matrix()[0]
                Tz[:3, 3] = (1 - a) * tkeys[j - 1][1][:3, 3] + a * tkeys[j][1][:3, 3]
                meta["interpolated"] = True
            else:
                Tz = None
            T = zup_cv_to_arkit_c2w(Tz) if Tz is not None else None
            self._files[len(self.frames)] = p
            if asset == "lowres_wide" or not len(dts):
                self._names.append(p.stem)
            else:
                j = int(np.argmin(np.abs(dts - t)))
                self._names.append(dnames[j] if abs(dts[j] - t) < 0.010 else None)
            self.frames.append(Frame(t=t, w=int(w), h=int(h), K=K_matrix(fx, fy, cx, cy), T=T, meta=meta))
        vid = self.name
        self._mesh = self.path / f"{vid}_3dod_mesh.ply"
        self._ann = self.path / f"{vid}_3dod_annotation.json"

    def iter_images(self, indices):
        yield from iter_image_files(self._files, indices)

    @property
    def has_depth(self):
        return (self.path / "lowres_depth").is_dir()

    def depth(self, i):
        if self._names[i] is None:
            return None
        p = self.path / "lowres_depth" / f"{self._names[i]}.png"
        return imread_any(p) if p.exists() else None

    def conf(self, i):
        if self._names[i] is None:
            return None
        p = self.path / "confidence" / f"{self._names[i]}.png"
        return imread_any(p) if p.exists() else None

    def mesh(self):
        if not self._mesh.exists():
            return None
        ply = read_ply(self._mesh)
        xyz = ply_xyz(ply) @ A_YUP_TO_ZUP  # z-up -> y-up (row vectors: v_yup = A^T v_zup)
        return Mesh(xyz=xyz, faces=ply["faces"], rgb=ply_rgb(ply))

    def room(self):
        return annotation_to_room(json.loads(self._ann.read_text())) if self._ann.exists() else None
