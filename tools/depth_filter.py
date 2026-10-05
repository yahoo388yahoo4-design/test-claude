"""Depth cleaning for phone depth-from-motion maps (numpy), same rules as the Android app's DepthFilter.kt.

ARCore depth on phones without a depth sensor (~160x120) smears depth across object silhouettes ("flying
pixels") and has speckle. `filter_depth` removes, in order:

  1. low confidence (ARCore levels 0..2 from conf.zlib.bin, or 0..255) and out-of-range depth;
  2. isolated pixels (fewer than `min_neighbours` same-surface 3x3 neighbours) and replaces the rest by the
     edge-preserving median of their same-surface neighbours (relative difference < 2 * rel_jump);
  3. flying pixels: depth jumps by more than `rel_jump` both towards a nearer and a farther neighbour;
  4. grazing pixels: surface normal more than `max_grazing_deg` from the viewing ray (needs K).

Input / output: uint16 millimetres, 0 = invalid.
"""
from __future__ import annotations

import warnings

import numpy as np

DEFAULTS = dict(min_conf_level=1, min_conf255=128, min_m=0.2, max_m=4.0, rel_jump=0.06,
                max_grazing_deg=80.0, min_neighbours=3)


def _shifts(a: np.ndarray):
    """The 8 neighbours of every pixel (zero padded): list of (dv, du, shifted array)."""
    p = np.pad(a, 1)
    h, w = a.shape
    return [(dv, du, p[1 + dv:1 + dv + h, 1 + du:1 + du + w]) for dv in (-1, 0, 1) for du in (-1, 0, 1)
            if dv or du]


def filter_depth(depth_mm: np.ndarray, conf: np.ndarray | None = None, K: np.ndarray | None = None, *,
                 conf_levels: bool = True, min_conf_level: int = 1, min_conf255: int = 128,
                 min_m: float = 0.2, max_m: float = 4.0, rel_jump: float = 0.06,
                 max_grazing_deg: float = 80.0, min_neighbours: int = 3, median: bool = True) -> np.ndarray:
    """Cleaned copy of `depth_mm` (uint16 mm). `K` is the 3x3 intrinsics at the depth resolution."""
    d = depth_mm.astype(np.float32) / 1000.0
    valid = d > 0
    if conf is not None:
        valid &= conf >= (min_conf_level if conf_levels else min_conf255)
    valid &= (d >= min_m) & (d <= max_m)
    a = np.where(valid, d, 0.0).astype(np.float32)

    # 2. edge-preserving median + isolation
    band = 2.0 * rel_jump
    nb = _shifts(a)
    same = [(s > 0) & (np.abs(s - a) < band * a) for _, _, s in nb]
    count = np.sum(same, axis=0)
    keep = (a > 0) & (count >= min_neighbours)
    if median:
        stack = np.stack([a] + [np.where(m, s, np.nan) for m, (_, _, s) in zip(same, nb)])
        stack[0][a == 0] = np.nan
        with np.errstate(all="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)      # all-NaN windows (invalid pixels)
            med = np.nanmedian(stack, axis=0)
        b = np.where(keep, med, 0.0).astype(np.float32)
    else:
        b = np.where(keep, a, 0.0).astype(np.float32)

    # 3. flying pixels: jump both ways
    near = np.zeros_like(b); far = np.zeros_like(b)
    with np.errstate(all="ignore"):
        for _, _, s in _shifts(b):
            r = np.where((s > 0) & (b > 0), (s - b) / np.where(b > 0, b, 1), 0)
            far = np.maximum(far, r); near = np.maximum(near, -r)
    c = np.where((near > rel_jump) & (far > rel_jump), 0.0, b).astype(np.float32)

    # 4. grazing angle
    if K is not None and max_grazing_deg < 90:
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        h, w = c.shape
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        X = (u - cx) / fx * c; Y = (v - cy) / fy * c
        P = np.stack([X, Y, c], -1)
        def nbr(dv, du):
            s = np.roll(P, (-dv, -du), axis=(0, 1)); sz = np.roll(c, (-dv, -du), axis=(0, 1))
            if dv: (s[-1:] if dv > 0 else s[:1])[...] = 0; (sz[-1:] if dv > 0 else sz[:1])[...] = 0
            if du: (s[:, -1:] if du > 0 else s[:, :1])[...] = 0; (sz[:, -1:] if du > 0 else sz[:, :1])[...] = 0
            return s, sz > 0
        (pr, mr), (pl, ml), (pd, md), (pu, mu) = nbr(0, 1), nbr(0, -1), nbr(1, 0), nbr(-1, 0)
        R = np.where(mr[..., None], pr, P); L = np.where(ml[..., None], pl, P)
        D = np.where(md[..., None], pd, P); U = np.where(mu[..., None], pu, P)
        ok = (mr | ml) & (md | mu)
        n = np.cross(R - L, D - U)
        ray = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], -1)
        with np.errstate(all="ignore"):
            cosang = np.abs(np.sum(n * ray, -1)) / (np.linalg.norm(n, axis=-1) * np.linalg.norm(ray, axis=-1))
        graze = ok & (c > 0) & np.isfinite(cosang) & (cosang < np.cos(np.radians(max_grazing_deg)))
        c = np.where(graze, 0.0, c)

    return np.round(c * 1000.0).astype(np.uint16)
