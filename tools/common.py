"""Shared geometry, PLY and image helpers for the capture converters.

Conventions
-----------
ARKit world   : y up, metres.            ARKit camera : x right, y up, looks down -z.
ARKitScenes   : world z up (Rx(+90deg) of ARKit world), camera OpenCV (x right, y down, z forward).
"""
from __future__ import annotations

import io
import struct
import zlib
from pathlib import Path

import numpy as np

# ARKit world (y-up) -> z-up world: (x, y, z) -> (x, -z, y)
A_YUP_TO_ZUP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
# OpenCV camera <-> ARKit camera axis flip (self-inverse)
F_CV_ARKIT = np.diag([1.0, -1.0, -1.0])


def arkit_c2w_to_zup_cv(T: np.ndarray) -> np.ndarray:
    """ARKit camera-to-world (y-up world, ARKit cam axes) -> z-up world, OpenCV camera."""
    out = np.eye(4)
    out[:3, :3] = A_YUP_TO_ZUP @ T[:3, :3] @ F_CV_ARKIT
    out[:3, 3] = A_YUP_TO_ZUP @ T[:3, 3]
    return out


def zup_cv_to_arkit_c2w(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = A_YUP_TO_ZUP.T @ T[:3, :3] @ F_CV_ARKIT
    out[:3, 3] = A_YUP_TO_ZUP.T @ T[:3, 3]
    return out


def rodrigues(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> axis-angle vector (robust near 0 and pi)."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_rotvec()


def rodrigues_inv(v) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_rotvec(np.asarray(v, dtype=np.float64)).as_matrix()


def rot_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def K_matrix(fx, fy, cx, cy) -> np.ndarray:
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


def scale_K(K: np.ndarray, sx: float, sy: float | None = None) -> np.ndarray:
    sy = sx if sy is None else sy
    K2 = K.copy().astype(np.float64)
    K2[0, :] *= sx
    K2[1, :] *= sy
    return K2


def ts_name(t: float) -> str:
    """ARKitScenes timestamp string (3 decimals) used in file names and traj keys."""
    return f"{t:.3f}"


# ----------------------------------------------------------------------------- raw deflate blobs

def inflate(blob: bytes) -> bytes:
    return zlib.decompress(blob, -15)


def deflate(raw: bytes, level: int = 6) -> bytes:
    c = zlib.compressobj(level, zlib.DEFLATED, -15)
    return c.compress(raw) + c.flush()


# ----------------------------------------------------------------------------- PLY

_PLY_TYPES = {"char": "i1", "uchar": "u1", "int8": "i1", "uint8": "u1", "short": "<i2", "ushort": "<u2",
              "int16": "<i2", "uint16": "<u2", "int": "<i4", "uint": "<u4", "int32": "<i4", "uint32": "<u4",
              "float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8"}


def read_ply(path) -> dict:
    """Minimal binary-little-endian / ascii PLY reader.

    Returns {"vertex": structured array, "faces": (M,3) int array or None, "face": structured extras}.
    Handles faces stored as `list uchar int` with optional extra scalar face properties, assuming all
    faces are triangles (true for ARKit, RoomPlan, VCGLIB exports used here).
    """
    data = Path(path).read_bytes()
    end = data.index(b"end_header") + len(b"end_header")
    while data[end:end + 1] in (b"\r", b"\n"):
        end += 1
        if data[end - 1:end] == b"\n":
            break
    header = data[:end].decode("ascii", "replace").splitlines()
    fmt = None
    elements = []
    for line in header:
        tok = line.split()
        if not tok:
            continue
        if tok[0] == "format":
            fmt = tok[1]
        elif tok[0] == "element":
            elements.append({"name": tok[1], "count": int(tok[2]), "props": []})
        elif tok[0] == "property":
            if tok[1] == "list":
                elements[-1]["props"].append(("list", tok[2], tok[3], tok[4]))
            else:
                elements[-1]["props"].append((tok[1], tok[2]))
    out = {"vertex": None, "faces": None, "face": None}
    body = data[end:]
    if fmt == "ascii":
        lines = body.decode().split("\n")
        li = 0
        for el in elements:
            rows = lines[li:li + el["count"]]
            li += el["count"]
            if el["name"] == "vertex":
                names = [p[1] for p in el["props"]]
                arr = np.array([[float(x) for x in r.split()[:len(names)]] for r in rows])
                dt = np.dtype([(n, _PLY_TYPES[p[0]]) for n, p in zip(names, el["props"])])
                v = np.zeros(len(rows), dtype=dt)
                for k, n in enumerate(names):
                    v[n] = arr[:, k]
                out["vertex"] = v
            elif el["name"] == "face":
                out["faces"] = np.array([[int(x) for x in r.split()[1:4]] for r in rows], dtype=np.int64)
        return out
    if fmt != "binary_little_endian":
        raise ValueError(f"unsupported PLY format {fmt}")
    off = 0
    for el in elements:
        props = el["props"]
        if any(p[0] == "list" for p in props):
            fields = []
            for p in props:
                if p[0] == "list":
                    fields.append((p[3] + "_n", _PLY_TYPES[p[1]]))
                    fields.append((p[3], _PLY_TYPES[p[2]], (3,)))
                else:
                    fields.append((p[1], _PLY_TYPES[p[0]]))
            dt = np.dtype(fields)
            arr = np.frombuffer(body, dtype=dt, count=el["count"], offset=off)
            listname = next(p[3] for p in props if p[0] == "list")
            n = arr[listname + "_n"]
            if el["count"] and not np.all(n == 3):
                raise ValueError("only triangle meshes are supported")
            off += dt.itemsize * el["count"]
            if el["name"] == "face":
                out["faces"] = arr[listname].astype(np.int64)
                out["face"] = arr
        else:
            dt = np.dtype([(p[1], _PLY_TYPES[p[0]]) for p in props])
            arr = np.frombuffer(body, dtype=dt, count=el["count"], offset=off)
            off += dt.itemsize * el["count"]
            if el["name"] == "vertex":
                out["vertex"] = arr
    return out


def ply_xyz(ply: dict) -> np.ndarray:
    v = ply["vertex"]
    return np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)


def ply_rgb(ply: dict):
    v = ply["vertex"]
    names = v.dtype.names
    for r, g, b in (("red", "green", "blue"), ("r", "g", "b")):
        if r in names:
            return np.stack([v[r], v[g], v[b]], 1).astype(np.uint8)
    return None


def write_ply_xyzrgba(path, xyz: np.ndarray, rgb: np.ndarray | None, faces: np.ndarray | None,
                      comment: str = "r2s-capture convert.py"):
    """Binary PLY in the ARKitScenes *_3dod_mesh.ply layout (float xyz, uchar rgba, list uchar int)."""
    n = len(xyz)
    faces = np.zeros((0, 3), np.int32) if faces is None else faces
    if rgb is None:
        rgb = np.full((n, 3), 200, np.uint8)
    v = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"),
                           ("blue", "u1"), ("alpha", "u1")])
    v["x"], v["y"], v["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    v["red"], v["green"], v["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    v["alpha"] = 255
    f = np.zeros(len(faces), dtype=[("n", "u1"), ("v", "<i4", (3,))])
    f["n"] = 3
    f["v"] = faces
    header = (f"ply\nformat binary_little_endian 1.0\ncomment {comment}\nelement vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\nproperty uchar red\n"
              "property uchar green\nproperty uchar blue\nproperty uchar alpha\n"
              f"element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n")
    with open(path, "wb") as fh:
        fh.write(header.encode())
        fh.write(v.tobytes())
        fh.write(f.tobytes())


def write_r2s_mesh_ply(path, xyz, faces, normals=None, cls=None):
    """Write mesh.ply in the r2s raw layout (used by tests and the Android side as a reference)."""
    n = len(xyz)
    normals = np.zeros((n, 3)) if normals is None else normals
    cls = np.zeros(len(faces), np.uint8) if cls is None else cls
    v = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("nx", "<f4"), ("ny", "<f4"), ("nz", "<f4")])
    v["x"], v["y"], v["z"] = xyz.T
    v["nx"], v["ny"], v["nz"] = normals.T
    f = np.zeros(len(faces), dtype=[("n", "u1"), ("v", "<i4", (3,)), ("cls", "u1")])
    f["n"] = 3
    f["v"] = faces
    f["cls"] = cls
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\n"
              "property float z\nproperty float nx\nproperty float ny\nproperty float nz\n"
              f"element face {len(faces)}\nproperty list uchar int vertex_indices\nproperty uchar cls\nend_header\n")
    with open(path, "wb") as fh:
        fh.write(header.encode())
        fh.write(v.tobytes())
        fh.write(f.tobytes())


# ----------------------------------------------------------------------------- images

def imread_rgb(path) -> np.ndarray:
    import cv2
    im = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if im is None:
        raise FileNotFoundError(path)
    return cv2.cvtColor(im, cv2.COLOR_BGR2RGB)


def imwrite_rgb(path, rgb: np.ndarray, jpeg_quality: int = 95):
    import cv2
    p = str(path)
    params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality] if p.lower().endswith((".jpg", ".jpeg")) else []
    if not cv2.imwrite(p, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), params):
        raise IOError(p)


def imread_any(path) -> np.ndarray:
    """Read a PNG keeping its bit depth (uint16 depth / uint8 confidence)."""
    import cv2
    im = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if im is None:
        raise FileNotFoundError(path)
    return im


def imwrite_any(path, arr: np.ndarray):
    import cv2
    if not cv2.imwrite(str(path), arr):
        raise IOError(path)


def resize(img: np.ndarray, w: int, h: int, nearest: bool = False) -> np.ndarray:
    import cv2
    if img.shape[1] == w and img.shape[0] == h:
        return img
    interp = cv2.INTER_NEAREST if nearest else (cv2.INTER_AREA if img.shape[1] > w else cv2.INTER_LINEAR)
    return cv2.resize(img, (w, h), interpolation=interp)
