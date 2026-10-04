"""Reader interface: every input format is normalised to an `Episode`.

A reader only has to fill `frames` (timestamps, intrinsics, ARKit camera-to-world poses) and
implement `iter_images` (+ `depth`/`conf` if it has depth). Everything else is optional.

To add a format (e.g. Android): subclass Episode in readers/<name>.py, give it `detect(path)` and
`__init__(path)`, then add one line to READERS in readers/__init__.py.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Iterator

import numpy as np


@dataclass
class Frame:
    t: float                       # seconds, device uptime clock
    w: int                         # image width (native landscape sensor orientation)
    h: int
    K: np.ndarray                  # 3x3 intrinsics at (w, h)
    T: np.ndarray | None           # 4x4 camera-to-world, ARKit world (y-up) + ARKit camera axes
    tracking: str = "normal"
    meta: dict = field(default_factory=dict)


@dataclass
class Mesh:
    xyz: np.ndarray                # (N,3) ARKit world (y-up)
    faces: np.ndarray | None       # (M,3) int or None (point cloud)
    rgb: np.ndarray | None = None  # (N,3) uint8
    face_cls: np.ndarray | None = None


class Episode:
    name: str = "episode"
    source: str = "unknown"

    def __init__(self):
        self.frames: list[Frame] = []
        self.path: Path | None = None

    # ---- required --------------------------------------------------------------------------
    def iter_images(self, indices: Iterable[int]) -> Iterator[tuple[int, np.ndarray]]:
        """Yield (index, RGB uint8 HxWx3) for the given increasing frame indices."""
        raise NotImplementedError

    # ---- optional --------------------------------------------------------------------------
    @property
    def has_depth(self) -> bool:
        return False

    def depth(self, i: int) -> np.ndarray | None:
        """uint16 millimetres (dh, dw), 0 = invalid; aligned with the RGB frame."""
        return None

    def conf(self, i: int) -> np.ndarray | None:
        """uint8 0..2 confidence, same size as depth."""
        return None

    def mesh(self) -> Mesh | None:
        return None

    def room(self) -> dict | None:
        """RoomPlan-style dict (see FORMAT.md roomplan/objects.json) or None."""
        return None

    def roomplan_usdz(self) -> Path | None:
        return None

    def extras(self) -> dict[str, Path]:
        """Files copied verbatim to <out>/extras/ (sensor logs, raw metadata, ...)."""
        return {}

    def gravity_world(self) -> np.ndarray:
        """Unit 'up' vector in the ARKit world. ARKit/ARCore gravity-aligned worlds: +y."""
        return np.array([0.0, 1.0, 0.0])

    def summary(self) -> dict:
        posed = sum(f.T is not None for f in self.frames)
        dur = self.frames[-1].t - self.frames[0].t if len(self.frames) > 1 else 0.0
        return {"source": self.source, "name": self.name, "frames": len(self.frames), "posed": posed,
                "duration_s": round(dur, 3), "fps": round((len(self.frames) - 1) / dur, 2) if dur else None,
                "image": f"{self.frames[0].w}x{self.frames[0].h}" if self.frames else None,
                "depth": self.has_depth}


def iter_video(path: Path, indices: Iterable[int]) -> Iterator[tuple[int, np.ndarray]]:
    """Sequentially decode a video and yield the requested frame indices as RGB."""
    import cv2
    want = sorted(set(int(i) for i in indices))
    if not want:
        return
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise IOError(f"cannot open video {path}")
    k = 0
    cur = 0
    try:
        while k < len(want):
            if cur < want[k]:
                if not cap.grab():
                    break
                cur += 1
                continue
            ok, bgr = cap.read()
            if not ok:
                break
            yield cur, cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            cur += 1
            k += 1
    finally:
        cap.release()
    if k < len(want):
        raise IOError(f"{path}: video ended after {cur} frames; wanted index {want[k]}")


def iter_image_files(paths: dict[int, Path], indices) -> Iterator[tuple[int, np.ndarray]]:
    from common import imread_rgb
    for i in sorted(set(indices)):
        yield i, imread_rgb(paths[i])
