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
confidence quantised to 0..2 in conf.zlib.bin (full 0..255 in extras/conf255.zlib.bin, byte ranges in
extras/arcore_frames.jsonl `raw_depth.c255`), smoothed depth in depth_smooth.zlib.bin (`--smoothed-depth`).
The smoothed map has no confidence of its own (the app writes no `sc`): `conf()` is None for it and it
is filtered without a confidence gate, as DepthFusion.kt does.

Intrinsics: ARCore depth covers the field of view of the camera *texture*, which can differ from the CPU
image's (e.g. 16:9 next to 4:3), so the app writes `dK` = [fx, fy, cx, cy] of the depth map at dw x dh on
every frames.jsonl line with depth. `depth_K(i)` prefers `dK` and falls back to FORMAT.md's `K * dw / w`.
`depth()` / `conf()` return maps aligned with the RGB frame (the Episode contract and what ARKitScenes /
LiteReality need): when `dK` disagrees with `K * dw / w` the map is resampled (nearest neighbour, through
both intrinsics) onto the RGB field of view; `episode.align_depth = False` returns the recorded geometry.

Depth is cleaned on read with tools/depth_filter.py (the same confidence / range / edge-preserving median /
flying-pixel / grazing-angle rules the app applies before fusion: the 0..255 confidence is gated at
session.json `depth_processing.filter.min_confidence` * 255 when extras/conf255.zlib.bin is present, else
the 0..2 levels at >= 1), and `conf()` is zeroed where the filter removed depth; `--no-depth-filter` in
convert.py (or `episode.filter_depth = False`) returns the recorded values untouched.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from common import K_matrix
from depth_filter import filter_depth
from .base import Frame, iter_video
from .r2s import R2SEpisode, _Blob


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
        self.align_depth = True
        self._cache = (None, None, None)       # (key, depth, conf) of the last frame read
        # full 0..255 raw-depth confidence: ranges live in extras/arcore_frames.jsonl, keyed here by the
        # raw depth's own byte range (the same "d" the frames.jsonl line carries)
        self._conf255 = _Blob(self.path / "extras" / "conf255.zlib.bin")
        self._c255_by_d = {}
        af = self.path / "extras" / "arcore_frames.jsonl"
        if self._conf255.fh is not None and af.exists():
            for line in af.read_text().splitlines():
                if not line.strip():
                    continue
                try:
                    rd = json.loads(line).get("raw_depth") or {}
                except Exception:
                    continue
                if rd.get("d") and rd.get("c255"):
                    self._c255_by_d[tuple(rd["d"])] = rd["c255"]
        flt = (self.meta.get("depth_processing") or {}).get("filter") or {}
        self.min_conf255 = int(float(flt.get("min_confidence", 0.5)) * 255)   # DepthFilter.kt: (minConfidence * 255).toInt()
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

    # ---- depth: intrinsics, filtering, alignment with the RGB frame ----------------------------
    def depth_K(self, i, shape=None) -> np.ndarray | None:
        """3x3 intrinsics of the recorded depth map of frame i: `dK` when the app wrote it (texture field of
        view), else FORMAT.md's `K * dw / w`. None without depth."""
        line = self._lines[i]
        if shape is None:
            if not line.get("dw"):
                return None
            shape = (int(line["dh"]), int(line["dw"]))
        dK = line.get("dK")
        if dK:
            return K_matrix(*dK[:4])
        f = self.frames[i]
        K = f.K.astype(np.float64).copy()
        K[0] *= shape[1] / max(f.w, 1)
        K[1] *= shape[0] / max(f.h, 1)
        return K

    def _raw_conf255(self, i):
        """0..255 confidence of frame i's raw depth map (extras/conf255.zlib.bin) or None."""
        line = self._lines[i]
        rng = self._c255_by_d.get(tuple(line.get("d") or ()))
        return self._decode(self._conf255, rng, np.uint8, line) if rng else None

    def _align(self, i, d, c, Kd):
        """Resample a map recorded at the texture field of view (intrinsics Kd) onto the RGB frame's so that
        `K * dw / w` holds: output pixel -> ray through K -> source pixel through Kd (nearest neighbour,
        0 outside the recorded map). Returns the inputs unchanged when both agree."""
        f = self.frames[i]
        dh, dw = d.shape
        ow, oh = dw, int(round(dw * f.h / max(f.w, 1)))
        Ko = f.K.astype(np.float64).copy()
        Ko[0] *= ow / max(f.w, 1)
        Ko[1] *= oh / max(f.h, 1)
        if (oh, ow) == (dh, dw) and np.allclose(Ko, Kd, rtol=1e-3, atol=0.05):
            return d, c
        su = np.rint((np.arange(ow) - Ko[0, 2]) / Ko[0, 0] * Kd[0, 0] + Kd[0, 2]).astype(int)
        sv = np.rint((np.arange(oh) - Ko[1, 2]) / Ko[1, 1] * Kd[1, 1] + Kd[1, 2]).astype(int)
        inside = ((sv >= 0) & (sv < dh))[:, None] & ((su >= 0) & (su < dw))[None, :]
        su, sv = np.clip(su, 0, dw - 1), np.clip(sv, 0, dh - 1)

        def remap(a):
            return np.where(inside, a[sv[:, None], su[None, :]], 0).astype(a.dtype)
        return remap(d), (remap(c) if c is not None and c.shape == (dh, dw) else c)

    def _load(self, i):
        """(depth, conf) of frame i after filtering and alignment, cached for the last frame (the writers
        call depth(k) and conf(k) back to back)."""
        key = (i, self.use_smoothed, self.filter_depth, self.align_depth)
        if self._cache[0] == key:
            return self._cache[1], self._cache[2]
        d, c = super().depth(i), super().conf(i)
        if d is not None:
            line = self._lines[i]
            Kd = self.depth_K(i, d.shape)
            if self.filter_depth:
                if self.use_smoothed and line.get("sd"):
                    d = filter_depth(d, None, Kd)             # DepthFusion.kt: no confidence gate on the smoothed map
                else:
                    c255 = self._raw_conf255(i)
                    if c255 is not None and c255.shape == d.shape:
                        d = filter_depth(d, c255, Kd, conf_levels=False, min_conf255=self.min_conf255)
                    else:
                        d = filter_depth(d, c if c is not None and c.shape == d.shape else None, Kd)
                if c is not None and c.shape == d.shape:
                    c = np.where(d > 0, c, 0).astype(np.uint8)
            if self.align_depth:
                d, c = self._align(i, d, c, Kd)
        self._cache = (key, d, c)
        return d, c

    def depth(self, i):
        if self._cam is not None:
            return super().depth(i)
        return self._load(i)[0]

    def conf(self, i):
        if self._cam is not None:
            return super().conf(i)
        return self._load(i)[1]

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
