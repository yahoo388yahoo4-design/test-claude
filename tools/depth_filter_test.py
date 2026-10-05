"""Tests for tools/depth_filter.py and its use in the Android reader.

Run:  python3 tools/depth_filter_test.py      (or: python3 -m pytest tools/depth_filter_test.py)
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from common import deflate  # noqa: E402
from depth_filter import filter_depth  # noqa: E402

W, H = 160, 120
K = np.array([[125.0, 0, 79.5], [0, 125.0, 59.5], [0, 0, 1]])


def scene(rng: np.random.Generator):
    """Wall at 3 m with a box at 1.5 m in the middle; 1 % noise, smeared silhouettes, speckle."""
    truth = np.full((H, W), 3.0, np.float32)
    truth[40:80, 60:100] = 1.5
    d = truth * (1 + 0.01 * rng.standard_normal(truth.shape)).astype(np.float32)
    flying = np.zeros(truth.shape, bool)
    edge = np.zeros(truth.shape, bool)
    edge[39:81, 59:101] = True
    edge[41:79, 61:99] = False
    sel = edge & (rng.random(truth.shape) < 0.8)
    d[sel] = (1.5 + 1.5 * rng.uniform(0.25, 0.75, truth.shape))[sel]
    flying[sel] = True
    speck = rng.random(truth.shape) < 0.004
    d[speck] = rng.uniform(0.4, 3.8, truth.shape)[speck]
    conf = np.full(truth.shape, 2, np.uint8)
    conf[flying] = rng.integers(0, 3, truth.shape)[flying]
    conf[speck] = rng.integers(0, 3, truth.shape)[speck]
    return truth, np.round(d * 1000).astype(np.uint16), conf, flying | speck


def test_flying_pixels_and_speckle_removed():
    truth, mm, conf, bad = scene(np.random.default_rng(1))
    out = filter_depth(mm, conf, K).astype(np.float32) / 1000
    err = np.abs(out - truth) > 0.05 * truth
    bad_left = np.sum(err & (out > 0))
    bad_in = np.sum(np.abs(mm / 1000 - truth) > 0.05 * truth)
    good = ~bad
    kept = np.sum(good & (out > 0)) / np.sum(good)
    print(f"bad pixels {bad_in} -> {bad_left}, good kept {kept:.3f}")
    assert bad_in > 200
    assert bad_left <= 0.05 * bad_in
    assert kept > 0.9


def test_confidence_range_and_no_filter():
    mm = np.full((H, W), 2000, np.uint16)
    conf = np.full((H, W), 2, np.uint8)
    conf[::7, ::7] = 0
    mm[10, 10] = 100
    mm[20, 21] = 6000
    out = filter_depth(mm, conf, K)
    assert out[0, 0] == 0 and out[10, 10] == 0 and out[20, 21] == 0
    assert out[50, 51] == 2000
    # 0..255 confidence
    out2 = filter_depth(mm, np.where(conf == 0, 30, 220).astype(np.uint8), K, conf_levels=False)
    assert out2[0, 0] == 0 and out2[50, 51] == 2000


def test_grazing_floor_rejected():
    # floor seen from 1.4 m: rows below the horizon; the rows near the horizon are at grazing angles
    v = np.arange(H, dtype=np.float32)[:, None] - K[1, 2]
    with np.errstate(divide="ignore"):
        d = np.where(v > 0, 1.4 * K[1, 1] / np.maximum(v, 1e-6), 0)
    mm = np.round(np.broadcast_to(d, (H, W)) * 1000).clip(0, 65535).astype(np.uint16)
    out = filter_depth(mm, None, K, max_m=60.0)
    assert out[110, 80] > 0                    # steep: kept
    assert out[61, 80] == 0                    # ~89 deg from the normal: dropped


def test_android_reader_filters_depth():
    from readers.android_reader import AndroidEpisode
    truth, mm, conf, _ = scene(np.random.default_rng(2))
    with tempfile.TemporaryDirectory() as td:
        p = Path(td)
        (p / "session.json").write_text(json.dumps({"format": "r2s-capture", "platform": "android", "mode": "arcore_rgbd"}))
        dblob = deflate(mm.tobytes()); cblob = deflate(conf.tobytes())
        (p / "depth.zlib.bin").write_bytes(dblob)
        (p / "conf.zlib.bin").write_bytes(cblob)
        line = {"i": 0, "t": 1.0, "w": 640, "h": 480, "K": [500.0, 500.0, 318.0, 238.0], "T": list(np.eye(4).ravel()),
                "d": [0, len(dblob)], "c": [0, len(cblob)], "dw": W, "dh": H}
        (p / "frames.jsonl").write_text(json.dumps(line) + "\n")
        ep = AndroidEpisode(p)
        filt = ep.depth(0)
        ep.filter_depth = False
        raw = ep.depth(0)
    assert np.array_equal(raw, mm)
    assert np.sum(filt > 0) < np.sum(raw > 0)
    err_raw = np.sum((raw > 0) & (np.abs(raw / 1000 - truth) > 0.05 * truth))
    err_f = np.sum((filt > 0) & (np.abs(filt / 1000 - truth) > 0.05 * truth))
    assert err_f <= 0.05 * err_raw


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
