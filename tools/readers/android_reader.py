"""Reader for sessions recorded by the Android app (capture/android/).

The Android app writes the same r2s-capture v1 raw format as the iOS app (FORMAT.md) with
"platform": "android", so the ARCore mode (`arcore_rgbd`) is read by R2SEpisode unchanged. This
subclass adds what is Android-specific:

  * extras/      ARCore per-frame record (display pose, texture intrinsics, light estimate,
                 Camera2 metadata), sensors_raw.csv, GNSS raw/NMEA, camera characteristics,
                 point clouds, plane timeline, ARCore Recording API mp4, ... -> copied to <out>/extras/android/
  * multicam     Camera2 mode (no poses on device): frames come from the main camera's
                 cams/<name>.jsonl + cams/<name>.mp4 with T=None. Poses must be recovered offline
                 (tools/recover_poses.py) before ARKitScenes / LiteReality export.
  * cams/        tof_depth.zlib.bin (DEPTH16 ToF, if the phone exposed it), raw/*.dng, calibration.json.

Depth: ARCore raw depth (depth-from-motion, or ToF-assisted if ARCore uses it) in depth.zlib.bin,
confidence quantised to 0..2 in conf.zlib.bin (full 0..255 in extras/conf255.zlib.bin), smoothed
depth in depth_smooth.zlib.bin (`--smoothed-depth`).

Depth is cleaned on read with tools/depth_filter.py (the same confidence / range / edge-preserving median /
flying-pixel / grazing-angle rules the app applies before fusion); `--no-depth-filter` in convert.py (or
`episode.filter_depth = False`) returns the recorded maps untouched.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from common import K_matrix
from depth_filter import filter_depth
from .base import Frame, iter_video
from .r2s import R2SEpisode


class AndroidEpisode(R2SEpisode):
    source = "android"
    filter_depth = True

    @staticmethod
    def detect(path: Path) -> bool:
        p = Path(path) / "session.json"
        if not p.exists():
            return False
        try:
            m = json.loads(p.read_text())
        except Exception:
            return False
        return m.get("format") == "r2s-capture" and m.get("platform") == "android"

    def __init__(self, path: Path):
        super().__init__(path)
        self.android = self.meta.get("android", {})
        self._cam = None
        if not self.frames and (self.path / "cams").is_dir():
            self._load_multicam()

    # ---- Camera2 multicam: expose the main camera's frames (unposed) ----------------------------
    def _load_multicam(self):
        streams = self.android.get("multicam", {}).get("streams") or []
        names = [s["name"] for s in streams] or sorted(p.stem for p in (self.path / "cams").glob("*.jsonl")
                                                         if not p.stem.startswith("tof"))
        if not names:
            return
        main = next((n for n in names if n.startswith("wide")), names[0])
        self._cam = main
        lines = [json.loads(x) for x in (self.path / "cams" / f"{main}.jsonl").read_text().splitlines() if x.strip()]
        self._lines = []
        for d in lines:
            K = d.get("K")
            if K is None:
                continue
            self.frames.append(Frame(t=float(d["t"]), w=int(d["w"]), h=int(d["h"]), K=K_matrix(*K[:4]), T=None,
                                     tracking="not_available",
                                     meta={"exp": d.get("exp"), "iso": d.get("iso"),
                                           "focus_distance_diopters": d.get("focus_distance_diopters")}))
            self._lines.append(d)
        self._video_index = [int(d["i"]) for d in self._lines]
        self.video_path = self.path / "cams" / f"{main}.mp4"
        self.name = f"{self.path.name}_{main}"

    def set_poses(self, poses: dict[float, np.ndarray], tol: float = 0.002):
        """Attach offline-recovered ARKit-convention camera-to-world poses {t: 4x4} to frames."""
        ts = np.array(sorted(poses))
        for f in self.frames:
            j = int(np.argmin(np.abs(ts - f.t)))
            if abs(ts[j] - f.t) <= tol:
                f.T = np.asarray(poses[ts[j]], dtype=np.float64)
                f.tracking = "recovered"

    @property
    def has_depth(self) -> bool:
        return self._cam is None and super().has_depth

    def depth(self, i):
        d = super().depth(i)
        if d is None or not self.filter_depth or self._cam is not None:
            return d
        f = self.frames[i]
        K = f.K.astype(np.float64).copy()
        K[0] *= d.shape[1] / max(f.w, 1)
        K[1] *= d.shape[0] / max(f.h, 1)
        c = self.conf(i)
        if c is not None and c.shape != d.shape:
            c = None
        return filter_depth(d, c, K)

    def iter_images(self, indices):
        if self._cam is None:
            yield from super().iter_images(indices)
            return
        indices = list(indices)
        want = {self._video_index[k]: k for k in indices}
        for vi, img in iter_video(self.video_path, want.keys()):
            yield want[vi], img

    def extras(self):
        out = super().extras()
        for name in ("extras", "cams", "hires"):
            p = self.path / name
            if p.is_dir() and any(p.iterdir()):
                out["android" if name == "extras" else name] = p
        for name in ("audio.m4a", "video.mp4.pts.csv", "conf255.zlib.bin"):
            if (self.path / name).exists():
                out[name] = self.path / name
        return out

    def summary(self) -> dict:
        s = super().summary()
        s["platform"] = "android"
        s["mode"] = self.mode
        if self._cam:
            s["multicam_main"] = self._cam
        ar = self.android.get("arcore")
        if ar:
            s["arcore"] = {k: ar.get(k) for k in ("image_size", "depth_mode", "depth_sensor_usage", "timestamp_source")}
        return s
