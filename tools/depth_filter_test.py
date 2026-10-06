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
    # the app drops raw confidence < (0.5 * 255).toInt() = 127: 126 goes, 127 stays
    c255 = np.full((H, W), 200, np.uint8)
    c255[30, 30] = 126
    c255[30, 31] = 127
    out3 = filter_depth(mm, c255, K, conf_levels=False)
    assert out3[30, 30] == 0 and out3[30, 31] == 2000


def write_session(p: Path, mm, conf, *, c255=None, min_confidence=None, dK=None, K=(500.0, 500.0, 318.0, 238.0),
                  wh=(640, 480), smoothed=None):
    """A one-frame Android session the way ArRecorder.kt writes it (depth + 0..2 conf, optional
    extras/conf255.zlib.bin + arcore_frames.jsonl, optional dK, optional smoothed depth without conf)."""
    meta = {"format": "r2s-capture", "platform": "android", "mode": "arcore_rgbd"}
    if min_confidence is not None:
        meta["depth_processing"] = {"filter": {"min_confidence": min_confidence}}
    (p / "session.json").write_text(json.dumps(meta))
    dblob = deflate(mm.astype("<u2").tobytes()); cblob = deflate(conf.tobytes())
    (p / "depth.zlib.bin").write_bytes(dblob)
    (p / "conf.zlib.bin").write_bytes(cblob)
    line = {"i": 0, "t": 1.0, "w": wh[0], "h": wh[1], "K": list(K), "T": list(np.eye(4).ravel()),
            "d": [0, len(dblob)], "c": [0, len(cblob)], "dw": mm.shape[1], "dh": mm.shape[0]}
    if dK is not None:
        line["dK"] = list(dK)
    if smoothed is not None:
        sblob = deflate(smoothed.astype("<u2").tobytes())
        (p / "depth_smooth.zlib.bin").write_bytes(sblob)
        line["sd"] = [0, len(sblob)]
    (p / "frames.jsonl").write_text(json.dumps(line) + "\n")
    if c255 is not None:
        (p / "extras").mkdir(exist_ok=True)
        blob = deflate(c255.tobytes())
        (p / "extras" / "conf255.zlib.bin").write_bytes(blob)
        rd = {"t": 1.0, "d": line["d"], "c": line["c"], "c255": [0, len(blob)], "dw": mm.shape[1], "dh": mm.shape[0]}
        (p / "extras" / "arcore_frames.jsonl").write_text(json.dumps({"n": 0, "t": 1.0, "raw_depth": rd}) + "\n")


def test_android_reader_conf255_gate_matches_app():
    """Raw confidence 100 is level 1 (kept by the 0..2 gate) but below the app's 127: with
    extras/conf255.zlib.bin the converter drops it too, and conf() is masked to the filtered depth."""
    from readers.android_reader import AndroidEpisode
    mm = np.full((H, W), 2000, np.uint16)
    c255 = np.full((H, W), 230, np.uint8)
    c255[40:50, 40:50] = 100
    conf = np.where(c255 < 85, 0, np.where(c255 < 170, 1, 2)).astype(np.uint8)   # ArRecorder.kt quantisation
    assert conf[45, 45] == 1
    with tempfile.TemporaryDirectory() as td:
        p = Path(td)
        write_session(p, mm, conf, c255=c255, min_confidence=0.5)
        ep = AndroidEpisode(p)
        assert ep.min_conf255 == 127
        d, c = ep.depth(0), ep.conf(0)
        assert d[45, 45] == 0 and d[60, 60] == 2000
        assert c[45, 45] == 0 and c[60, 60] == 2
        ep.filter_depth = False
        assert ep.depth(0)[45, 45] == 2000 and ep.conf(0)[45, 45] == 1
    with tempfile.TemporaryDirectory() as td:          # no conf255 -> 0..2 levels, level 1 survives
        p = Path(td)
        write_session(p, mm, conf)
        ep = AndroidEpisode(p)
        assert ep.depth(0)[45, 45] == 2000 and ep.conf(0)[45, 45] == 1


def test_android_reader_smoothed_depth_has_no_confidence():
    """The app writes no confidence for the smoothed map: conf() is None and the raw map's confidence
    must not gate the smoothed depth."""
    from readers.android_reader import AndroidEpisode
    mm = np.full((H, W), 2000, np.uint16)
    conf = np.full((H, W), 2, np.uint8)
    conf[40:50, 40:50] = 0
    smooth = np.full((H, W), 2500, np.uint16)
    with tempfile.TemporaryDirectory() as td:
        p = Path(td)
        write_session(p, mm, conf, smoothed=smooth)
        ep = AndroidEpisode(p)
        assert ep.depth(0)[45, 45] == 0 and ep.conf(0)[45, 45] == 0      # raw map: gated
        ep.use_smoothed = True
        assert ep.conf(0) is None
        assert ep.depth(0)[45, 45] == 2500                               # smoothed map: not gated by raw conf


def test_android_reader_dk_alignment():
    """Depth recorded at a 16:9 texture field of view (dK) next to a 4:3 CPU image is resampled onto the
    RGB field of view, so K * dw / w holds for every consumer; without dK nothing changes."""
    from readers.android_reader import AndroidEpisode
    # CPU image 640x480, K = (500, 500, 318, 238); texture = the central 640x360 of the same sensor,
    # depth 160x90 -> dK = texture K * (160/640, 90/360)
    dK = (500 * 0.25, 500 * 0.25, 318 * 0.25, (238 - 60) * 0.25)
    mm = np.full((90, 160), 2000, np.uint16)
    mm[10, 20] = 1234
    conf = np.full((90, 160), 2, np.uint8)
    conf[10, 20] = 1
    with tempfile.TemporaryDirectory() as td:
        p = Path(td)
        write_session(p, mm, conf, dK=dK)
        ep = AndroidEpisode(p)
        ep.filter_depth = False
        assert np.allclose(ep.depth_K(0), [[125, 0, 79.5], [0, 125, 44.5], [0, 0, 1]])
        d, c = ep.depth(0), ep.conf(0)
        assert d.shape == (120, 160) and c.shape == (120, 160)
        assert d[:15].max() == 0 and d[105:].max() == 0 and c[:15].max() == 0     # outside the texture FOV
        assert np.sum(d[15:105] == 2000) == 90 * 160 - 1 and np.sum(c[15:105] == 2) == 90 * 160 - 1
        assert d[25, 20] == 1234 and c[25, 20] == 1                             # row 10 + 15 rows of offset
        ep.align_depth = False
        assert np.array_equal(ep.depth(0), mm)
        ep.align_depth = True
        ep.filter_depth = True
        assert ep.depth(0).shape == (120, 160)
    with tempfile.TemporaryDirectory() as td:                                    # dK == K * dw / w: untouched
        p = Path(td)
        mm4, conf4 = np.resize(mm, (120, 160)), np.resize(conf, (120, 160))
        write_session(p, mm4, conf4, dK=(500 * 0.25, 500 * 0.25, 318 * 0.25, 238 * 0.25))
        ep = AndroidEpisode(p)
        ep.filter_depth = False
        assert np.array_equal(ep.depth(0), mm4) and np.array_equal(ep.conf(0), conf4)


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
