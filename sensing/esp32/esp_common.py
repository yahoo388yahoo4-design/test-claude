"""Shared pieces of the ESP32 sensing kit: the esp_nodes.json schema, tag manifest and geometry.

Conventions (same as tools/common.py):
  session world : the ARKit / ARCore world of a capture session, y up, metres.
  ARKit camera  : x right, y up, looks down -z.   OpenCV camera: x right, y down, z forward.
  Frames        : K at the native (landscape) image size, T = 4x4 camera-to-world.
"""
from __future__ import annotations

import datetime as _dt
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
NODES_FILE = "esp_nodes.json"
F_CV_ARKIT = np.diag([1.0, -1.0, -1.0])

# Printed tag geometry. The apps read the same numbers from esp_tags.json next to the tag images.
TAG_SIZE_M = 0.070          # whole printed square (ARCore / ARKit reference-image width)
ARUCO_SIZE_M = 0.036        # ArUco marker incl. its black border (OpenCV pose)
ARUCO_DICT = "DICT_4X4_50"  # marker id = node id
NODE_COUNT = 6


# ------------------------------------------------------------------------------------ nodes file
def now_iso() -> str:
    return _dt.datetime.now(_dt.timezone.utc).replace(microsecond=0).isoformat()


def empty_nodes() -> dict:
    return {"version": 1, "format": "r2s-esp-nodes", "frame": "session_world", "units": "m", "up": "+y",
            "nodes": []}


def load_nodes(path) -> dict:
    p = Path(path)
    if p.is_dir():
        p = p / NODES_FILE
    if not p.exists():
        return empty_nodes()
    d = json.loads(p.read_text())
    d.setdefault("nodes", [])
    return d


def save_nodes(path, doc: dict) -> Path:
    p = Path(path)
    if p.is_dir():
        p = p / NODES_FILE
    doc["nodes"] = sorted(doc["nodes"], key=lambda n: n["node_id"])
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(doc, indent=1))
    tmp.replace(p)
    return p


# Which result wins when two methods placed the same node: a person's tap beats automatic detection,
# and a multi-view result beats a single view.
METHOD_RANK = {"tap_frame": 4, "tap_live": 4, "triangulated": 4, "auto_tag": 2, "auto_tag_live": 2, "planned": 0}


def upsert_node(doc: dict, node: dict, force: bool = False) -> bool:
    """Insert or replace a node entry. Returns True if the document changed."""
    node = dict(node)
    node.setdefault("label", f"esp-{node['node_id']}")
    node.setdefault("updated", now_iso())
    node["position"] = [round(float(v), 4) for v in node["position"]]
    if node.get("normal") is not None:
        node["normal"] = [round(float(v), 4) for v in node["normal"]]
    for i, old in enumerate(doc["nodes"]):
        if old["node_id"] == node["node_id"]:
            if not force and METHOD_RANK.get(old.get("method"), 1) > METHOD_RANK.get(node.get("method"), 1):
                return False
            doc["nodes"][i] = node
            return True
    doc["nodes"].append(node)
    return True


# ------------------------------------------------------------------------------------ geometry
def unproject(K: np.ndarray, T: np.ndarray, u: float, v: float, depth_m: float) -> np.ndarray:
    """Pixel (u, v) at the native image size + metric depth along the optical axis -> world point."""
    x = (u - K[0, 2]) / K[0, 0] * depth_m
    y = (v - K[1, 2]) / K[1, 1] * depth_m
    p_ar = F_CV_ARKIT @ np.array([x, y, depth_m])
    return T[:3, :3] @ p_ar + T[:3, 3]


def pixel_ray(K: np.ndarray, T: np.ndarray, u: float, v: float) -> tuple[np.ndarray, np.ndarray]:
    """World-space ray (origin, unit direction) through pixel (u, v)."""
    d_cv = np.array([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], 1.0])
    d = T[:3, :3] @ (F_CV_ARKIT @ d_cv)
    return T[:3, 3].copy(), d / np.linalg.norm(d)


def project(K: np.ndarray, T: np.ndarray, p: np.ndarray):
    """World point -> (u, v, depth) at the native image size; depth <= 0 means behind the camera."""
    pc = T[:3, :3].T @ (np.asarray(p, float) - T[:3, 3])
    pcv = F_CV_ARKIT @ pc
    if pcv[2] <= 1e-6:
        return None
    return K[0, 0] * pcv[0] / pcv[2] + K[0, 2], K[1, 1] * pcv[1] / pcv[2] + K[1, 2], pcv[2]


def triangulate(rays: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, float, float]:
    """Least-squares closest point to >= 2 rays. Returns (point, rms distance to rays, max angle deg)."""
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for o, d in rays:
        P = np.eye(3) - np.outer(d, d)
        A += P
        b += P @ o
    p = np.linalg.lstsq(A, b, rcond=None)[0]
    dist = [np.linalg.norm((np.eye(3) - np.outer(d, d)) @ (p - o)) for o, d in rays]
    ang = 0.0
    for i in range(len(rays)):
        for j in range(i + 1, len(rays)):
            c = abs(float(np.dot(rays[i][1], rays[j][1])))
            ang = max(ang, float(np.degrees(np.arccos(min(1.0, c)))))
    return p, float(np.sqrt(np.mean(np.square(dist)))), ang


def depth_at(depth_mm: np.ndarray, w: int, h: int, u: float, v: float, radius_px: int = 2) -> float | None:
    """Median valid depth (m) around RGB pixel (u, v); the depth map may be smaller than the image."""
    dh, dw = depth_mm.shape
    du, dv = int(round(u * dw / w)), int(round(v * dh / h))
    if not (0 <= du < dw and 0 <= dv < dh):
        return None
    patch = depth_mm[max(0, dv - radius_px):dv + radius_px + 1, max(0, du - radius_px):du + radius_px + 1]
    vals = patch[patch > 0]
    if vals.size == 0:
        return None
    return float(np.median(vals)) / 1000.0


def robust_center(points: np.ndarray, weights: np.ndarray | None = None, inlier_m: float = 0.08):
    """Weighted median-ish centre with outlier rejection. Returns (centre, inlier mask, spread m)."""
    P = np.asarray(points, float)
    w = np.ones(len(P)) if weights is None else np.asarray(weights, float)
    c = np.median(P, axis=0)
    for _ in range(3):
        d = np.linalg.norm(P - c, axis=1)
        m = d <= max(inlier_m, 2.5 * np.median(d) if len(d) > 2 else inlier_m)
        if not m.any():
            m = d <= d.min() + 1e-9
        c = (P[m] * w[m, None]).sum(0) / w[m].sum()
    d = np.linalg.norm(P - c, axis=1)
    m = d <= max(inlier_m, 2.5 * np.median(d) if len(d) > 2 else inlier_m)
    spread = float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else float("nan")
    return c, m, spread


def floor_height(points_y: np.ndarray) -> float:
    """Floor = a low percentile of the scanned points' heights (y-up world)."""
    return float(np.percentile(points_y, 2)) if len(points_y) else 0.0
