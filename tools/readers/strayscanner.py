"""Reader for Stray Scanner exports (github.com/strayrobots/scanner).

Folder: rgb.mp4, odometry.csv ("timestamp, frame, x, y, z, qx, qy, qz, qw" = ARKit camera-to-world),
camera_matrix.csv (3x3 K of rgb.mp4), depth/000000.png (uint16 mm, 256x192), confidence/000000.png,
imu.csv. Stray Scanner does not export a mesh.
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from common import imread_any
from .base import Episode, Frame, iter_video


class StrayEpisode(Episode):
    source = "strayscanner"

    @staticmethod
    def detect(path: Path) -> bool:
        return (path / "odometry.csv").exists() and (path / "camera_matrix.csv").exists()

    def __init__(self, path: Path):
        super().__init__()
        import cv2
        from scipy.spatial.transform import Rotation
        self.path = Path(path)
        self.name = self.path.name
        K = np.loadtxt(self.path / "camera_matrix.csv", delimiter=",")
        cap = cv2.VideoCapture(str(self.path / "rgb.mp4"))
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        if w == 0:  # fall back to the documented size
            w, h = 1920, 1440
        rows = []
        with open(self.path / "odometry.csv") as fh:
            for r in csv.reader(fh):
                if not r or not r[0].strip()[0].isdigit():
                    continue
                rows.append([float(x) for x in r])
        self._video_index = []
        for r in rows:
            t, fi, x, y, z, qx, qy, qz, qw = r[:9]
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T[:3, 3] = [x, y, z]
            self.frames.append(Frame(t=t, w=w, h=h, K=K.copy(), T=T))
            self._video_index.append(int(fi))

    def iter_images(self, indices):
        want = {self._video_index[k]: k for k in indices}
        for vi, img in iter_video(self.path / "rgb.mp4", want.keys()):
            yield want[vi], img

    @property
    def has_depth(self):
        return (self.path / "depth").is_dir()

    def _png(self, sub, i):
        for ext in (".png",):
            p = self.path / sub / f"{self._video_index[i]:06d}{ext}"
            if p.exists():
                return imread_any(p)
        return None

    def depth(self, i):
        return self._png("depth", i)

    def conf(self, i):
        return self._png("confidence", i)

    def extras(self):
        return {f: self.path / f for f in ("imu.csv", "odometry.csv", "camera_matrix.csv") if (self.path / f).exists()}
