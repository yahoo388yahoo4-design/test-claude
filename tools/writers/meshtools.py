"""Colour a mesh by projecting it into posed RGB(-D) frames, or fuse a point cloud from depth."""
from __future__ import annotations

import numpy as np


def _project(xyz, T, K, w, h):
    """ARKit c2w T, pinhole K at (w,h) -> u, v, z_cv, valid mask."""
    R, t = T[:3, :3], T[:3, 3]
    pc = (xyz - t) @ R          # ARKit camera coords (row vectors: R^T (x - t))
    z = -pc[:, 2]
    zs = np.where(z > 1e-6, z, 1.0)
    u = K[0, 0] * pc[:, 0] / zs + K[0, 2]
    v = K[1, 1] * (-pc[:, 1]) / zs + K[1, 2]
    ok = (z > 0.05) & (u >= 0) & (u <= w - 1) & (v >= 0) & (v <= h - 1)
    return u, v, z, ok


def color_vertices(xyz: np.ndarray, views: list, depth_tol: float = 0.06) -> tuple[np.ndarray, float]:
    """views: list of (K at image size, T_c2w ARKit, rgb HxWx3, depth_mm or None).

    Returns (rgb uint8 (N,3), fraction of vertices seen). Unseen vertices are grey."""
    acc = np.zeros((len(xyz), 3))
    wsum = np.zeros(len(xyz))
    for K, T, img, dep in views:
        h, w = img.shape[:2]
        u, v, z, ok = _project(xyz, T, K, w, h)
        idx = np.nonzero(ok)[0]
        ui = np.round(u[idx]).astype(int)
        vi = np.round(v[idx]).astype(int)
        if dep is not None:
            dh, dw = dep.shape
            du = np.clip((u[idx] * dw / w).astype(int), 0, dw - 1)
            dv = np.clip((v[idx] * dh / h).astype(int), 0, dh - 1)
            d = dep[dv, du].astype(np.float64) / 1000.0
            vis = (d > 0) & (np.abs(d - z[idx]) < np.maximum(depth_tol, 0.03 * z[idx]))
            idx, ui, vi = idx[vis], ui[vis], vi[vis]
        wt = 1.0 / np.maximum(z[idx], 0.3)
        acc[idx] += img[vi, ui].astype(np.float64) * wt[:, None]
        wsum[idx] += wt
    seen = wsum > 0
    rgb = np.full((len(xyz), 3), 128, np.uint8)
    rgb[seen] = np.clip(acc[seen] / wsum[seen, None], 0, 255).astype(np.uint8)
    return rgb, float(seen.mean()) if len(xyz) else 0.0


def fuse_points(views_with_conf: list, voxel: float = 0.02, max_depth: float = 5.0) -> tuple[np.ndarray, np.ndarray]:
    """Back-project high-confidence depth into a voxel-downsampled coloured point cloud (ARKit world).

    views_with_conf: (K at image size, T, rgb, depth_mm, conf or None)."""
    pts, cols = [], []
    for K, T, img, dep, conf in views_with_conf:
        if dep is None:
            continue
        h, w = img.shape[:2]
        dh, dw = dep.shape
        Kd = K.copy()
        Kd[0] *= dw / w
        Kd[1] *= dh / h
        vv, uu = np.mgrid[0:dh, 0:dw]
        d = dep.astype(np.float64) / 1000.0
        m = (d > 0.1) & (d < max_depth)
        if conf is not None:
            m &= conf >= 2
        x = (uu[m] - Kd[0, 2]) / Kd[0, 0] * d[m]
        y = (vv[m] - Kd[1, 2]) / Kd[1, 1] * d[m]
        pc_cv = np.stack([x, y, d[m]], 1)
        pc_ar = pc_cv * np.array([1, -1, -1])
        pts.append(pc_ar @ T[:3, :3].T + T[:3, 3])
        iu = np.clip((uu[m] * w / dw).astype(int), 0, w - 1)
        iv = np.clip((vv[m] * h / dh).astype(int), 0, h - 1)
        cols.append(img[iv, iu])
    if not pts:
        return np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
    P = np.concatenate(pts)
    C = np.concatenate(cols)
    key = np.floor(P / voxel).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    n = len(first)
    cnt = np.bincount(inv, minlength=n).astype(np.float64)
    Pm = np.stack([np.bincount(inv, P[:, k], n) for k in range(3)], 1) / cnt[:, None]
    Cm = np.stack([np.bincount(inv, C[:, k].astype(np.float64), n) for k in range(3)], 1) / cnt[:, None]
    return Pm, Cm.astype(np.uint8)
