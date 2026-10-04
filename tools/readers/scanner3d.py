"""Reader for 3D Scanner App (Laan Labs) "All Data" exports, the input LiteReality expects.

Folder: frame_00000.jpg + frame_00000.json ("cameraPoseARFrame": 16 floats row-major ARKit
camera-to-world, "intrinsics": 9 floats row-major at full resolution, "time"), depth_00000.png
(uint16 mm, 256x192), conf_00000.png, textured_output.obj (y-up ARKit world), roomplan/room.usdz.
"""
from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

from common import imread_any
from .base import Episode, Frame, Mesh, iter_image_files


def read_obj(path: Path) -> Mesh:
    vs, cols, fs = [], [], []
    with open(path) as fh:
        for line in fh:
            if line.startswith("v "):
                p = line.split()
                vs.append([float(x) for x in p[1:4]])
                if len(p) >= 7:
                    cols.append([float(x) for x in p[4:7]])
            elif line.startswith("f "):
                idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
                for k in range(1, len(idx) - 1):  # fan-triangulate
                    fs.append([idx[0], idx[k], idx[k + 1]])
    xyz = np.array(vs, dtype=np.float64)
    faces = np.array(fs, dtype=np.int64) if fs else None
    if faces is not None:
        faces = np.where(faces > 0, faces - 1, len(xyz) + faces)
    rgb = None
    if cols and len(cols) == len(vs):
        c = np.array(cols)
        rgb = (c * 255 if c.max() <= 1.0 else c).clip(0, 255).astype(np.uint8)
    return Mesh(xyz=xyz, faces=faces, rgb=rgb)


class Scanner3DEpisode(Episode):
    source = "3dscannerapp"

    @staticmethod
    def detect(path: Path) -> bool:
        return (path / "frame_00000.json").exists() or any(path.glob("frame_0*.json"))

    def __init__(self, path: Path):
        super().__init__()
        from common import imread_rgb
        self.path = Path(path)
        self.name = self.path.name
        js = sorted(self.path.glob("frame_*.json"), key=lambda p: int(re.findall(r"\d+", p.stem)[0]))
        self._ids = []
        self._img = {}
        for k, jp in enumerate(js):
            d = json.loads(jp.read_text())
            fid = int(re.findall(r"\d+", jp.stem)[0])
            ip = jp.with_suffix(".jpg")
            if not ip.exists():
                continue
            if k == 0 or not hasattr(self, "_wh"):
                im = imread_rgb(ip)
                self._wh = (im.shape[1], im.shape[0])
            T = np.array(d["cameraPoseARFrame"], dtype=np.float64).reshape(4, 4) if "cameraPoseARFrame" in d else None
            K = np.array(d["intrinsics"], dtype=np.float64).reshape(3, 3)
            t = float(d.get("time", d.get("timestamp", fid / 30.0)))
            self._img[len(self.frames)] = ip
            self._ids.append(fid)
            self.frames.append(Frame(t=t, w=self._wh[0], h=self._wh[1], K=K, T=T,
                                     meta={"exp": d.get("exposureDuration"), "motionQuality": d.get("motionQuality")}))

    def iter_images(self, indices):
        yield from iter_image_files(self._img, indices)

    @property
    def has_depth(self):
        return bool(self._ids) and (self.path / f"depth_{self._ids[0]:05d}.png").exists()

    def depth(self, i):
        p = self.path / f"depth_{self._ids[i]:05d}.png"
        return imread_any(p) if p.exists() else None

    def conf(self, i):
        p = self.path / f"conf_{self._ids[i]:05d}.png"
        return imread_any(p) if p.exists() else None

    def mesh(self):
        p = self.path / "textured_output.obj"
        return read_obj(p) if p.exists() else None

    def roomplan_usdz(self):
        for p in (self.path / "roomplan" / "room.usdz", self.path / "room.usdz"):
            if p.exists():
                return p
        return None

    def room(self):
        p = self.path / "roomplan" / "objects.json"
        return json.loads(p.read_text()) if p.exists() else None

    def extras(self):
        out = {}
        for p in self.path.iterdir():
            if p.suffix in (".json", ".txt", ".csv") and not p.name.startswith("frame_"):
                out[p.name] = p
        return out
