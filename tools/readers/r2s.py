"""Reader for the r2s-capture raw session format (FORMAT.md)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from common import K_matrix, inflate, ply_xyz, read_ply
from .base import Episode, Frame, Mesh, iter_image_files, iter_video

EXTRA_FILES = ["session.json", "frames.jsonl", "imu.csv", "accel.csv", "gyro.csv", "mag.csv", "location.csv",
               "heading.csv", "altimeter.csv", "altimeter_abs.csv", "status.csv", "clock.csv", "planes.json",
               "worldmap.arworldmap", "mesh.ply"]
EXTRA_DIRS = ["hires", "roomplan", "cams"]


class _Blob:
    def __init__(self, path: Path):
        self.path = path
        self.fh = open(path, "rb") if path.exists() else None

    def read(self, rng):
        if self.fh is None or rng is None:
            return None
        off, n = rng
        self.fh.seek(off)
        return self.fh.read(n)


class R2SEpisode(Episode):
    source = "r2s"

    @staticmethod
    def detect(path: Path) -> bool:
        p = path / "session.json"
        if not p.exists():
            return False
        try:
            return json.loads(p.read_text()).get("format") == "r2s-capture"
        except Exception:
            return False

    def __init__(self, path: Path):
        super().__init__()
        self.path = Path(path)
        self.meta = json.loads((self.path / "session.json").read_text())
        self.name = self.path.name
        self.mode = self.meta.get("mode", "arkit_rgbd")
        self._lines = []
        fj = self.path / "frames.jsonl"
        if fj.exists():
            for line in fj.read_text().splitlines():
                if line.strip():
                    self._lines.append(json.loads(line))
        for d in self._lines:
            T = np.array(d["T"], dtype=np.float64).reshape(4, 4) if d.get("T") else None
            K = K_matrix(*d["K"])
            self.frames.append(Frame(t=float(d["t"]), w=int(d["w"]), h=int(d["h"]), K=K, T=T,
                                     tracking=d.get("track", "normal"),
                                     meta={k: d.get(k) for k in ("exp", "eo", "iso", "amb", "ct", "wm")}))
        self._video_index = [int(d.get("i", k)) for k, d in enumerate(self._lines)]
        self._depth = _Blob(self.path / "depth.zlib.bin")
        self._conf = _Blob(self.path / "conf.zlib.bin")
        self.use_smoothed = False
        self._sdepth = _Blob(self.path / "depth_smooth.zlib.bin")
        self._sconf = _Blob(self.path / "conf_smooth.zlib.bin")
        fdir = self.path / "frames"
        self._frame_files = None
        if fdir.is_dir():
            self._frame_files = {}
            for k, d in enumerate(self._lines):
                for ext in (".png", ".jpg"):
                    p = fdir / f"{int(d.get('i', k)):06d}{ext}"
                    if p.exists():
                        self._frame_files[k] = p
                        break
        self.video_path = self.path / self.meta.get("video", {}).get("file", "video.mov")

    def iter_images(self, indices):
        indices = list(indices)
        if self._frame_files is not None:
            yield from iter_image_files(self._frame_files, indices)
            return
        # frame k is sample _video_index[k] in the video
        want = {self._video_index[k]: k for k in indices}
        for vi, img in iter_video(self.video_path, want.keys()):
            yield want[vi], img

    @property
    def has_depth(self) -> bool:
        return self._depth.fh is not None and any(d.get("d") for d in self._lines)

    def _decode(self, blob, rng, dtype, d):
        raw = blob.read(rng)
        if raw is None:
            return None
        return np.frombuffer(inflate(raw), dtype).reshape(int(d["dh"]), int(d["dw"])).copy()

    def depth(self, i):
        d = self._lines[i]
        if self.use_smoothed and d.get("sd"):
            return self._decode(self._sdepth, d["sd"], "<u2", d)
        return self._decode(self._depth, d.get("d"), "<u2", d)

    def conf(self, i):
        d = self._lines[i]
        if self.use_smoothed and d.get("sd"):
            # confidence of the smoothed map `depth(i)` returns, or None when the recorder wrote none
            # (Android writes no "sc"); never the raw map's confidence
            return self._decode(self._sconf, d.get("sc"), np.uint8, d)
        return self._decode(self._conf, d.get("c"), np.uint8, d)

    def mesh(self):
        p = self.path / "mesh.ply"
        if not p.exists():
            return None
        ply = read_ply(p)
        cls = None
        if ply.get("face") is not None and "cls" in ply["face"].dtype.names:
            cls = np.asarray(ply["face"]["cls"])
        return Mesh(xyz=ply_xyz(ply), faces=ply["faces"], face_cls=cls)

    def room(self):
        p = self.path / "roomplan" / "objects.json"
        return json.loads(p.read_text()) if p.exists() else None

    def roomplan_usdz(self):
        p = self.path / "roomplan" / "room.usdz"
        return p if p.exists() else None

    def extras(self):
        out = {}
        for f in EXTRA_FILES:
            if (self.path / f).exists():
                out[f] = self.path / f
        for d in EXTRA_DIRS:
            if (self.path / d).is_dir():
                out[d] = self.path / d
        return out
