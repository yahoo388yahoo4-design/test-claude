#!/usr/bin/env python3
"""Tests for the ESP32 kit on a synthetic capture session (no hardware, no phone).

Renders a 5 x 2.5 x 4 m room with the six printed tags on the walls, writes it as an r2s session with
depth (tools/writers.write_r2s_raw), then checks that:
  * esp_locate auto finds every node within 2 cm, with the right wall normal,
  * esp_locate click recovers nodes from one click with depth, and from two clicks without depth,
  * esp_place plans 6 positions on the walls from the session geometry and scores them,
  * esp_fleet parses CSI / vitals packets as the firmware README documents them.

  python test_esp.py [--keep DIR]
"""
from __future__ import annotations

import argparse
import json
import shutil
import struct
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent.parent / "tools"))

import esp_fleet  # noqa: E402
import esp_locate  # noqa: E402
import esp_place  # noqa: E402
from esp_common import F_CV_ARKIT, load_nodes, project  # noqa: E402
from readers.base import Episode, Frame  # noqa: E402

ROOM = np.array([5.0, 2.5, 4.0])
W, H, FX = 960, 720, 640.0
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1]])
TAG = 0.070
NODES = {  # node id -> (centre, outward wall normal)
    1: ((0.004, 0.9, 1.2), (1, 0, 0)),
    2: ((2.0, 1.9, 0.004), (0, 0, 1)),
    3: ((4.0, 0.9, 0.004), (0, 0, 1)),
    4: ((4.996, 1.9, 2.2), (-1, 0, 0)),
    5: ((3.3, 0.9, 3.996), (0, 0, -1)),
    6: ((1.0, 1.9, 3.996), (0, 0, -1)),
}


def look_T(eye, yaw, pitch=0.0):
    """Camera-to-world (ARKit axes: looks down -z) at eye with yaw about +y and pitch about camera x."""
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    T = np.eye(4)
    T[:3, :3] = Ry @ Rx
    T[:3, 3] = eye
    return T


class SynthRoom(Episode):
    source = "synthetic"

    def __init__(self, n=240):
        super().__init__()
        import cv2
        self.name = "synth_room"
        self.tags = {i: cv2.imread(str(HERE / "tags" / f"esp_{i}.png"))[:, :, ::-1] for i in NODES}
        self.frames = []
        # Walk a loop 0.75 m in from the walls, facing the nearest wall and sweeping +-35 deg, the way you
        # would film the walls while scanning a room.
        x0, x1, z0, z1 = 0.75, ROOM[0] - 0.75, 0.75, ROOM[2] - 0.75
        per = 2 * ((x1 - x0) + (z1 - z0))
        for k in range(n):
            s = per * k / n
            if s < x1 - x0:
                eye, yaw = (x0 + s, z0), 0.0                      # facing -z wall
            elif s < (x1 - x0) + (z1 - z0):
                eye, yaw = (x1, z0 + s - (x1 - x0)), -np.pi / 2    # facing +x wall
            elif s < 2 * (x1 - x0) + (z1 - z0):
                eye, yaw = (x1 - (s - (x1 - x0) - (z1 - z0)), z1), np.pi   # facing +z wall
            else:
                eye, yaw = (x0, z1 - (s - 2 * (x1 - x0) - (z1 - z0))), np.pi / 2   # facing -x wall
            sweep = 0.6 * np.sin(2 * np.pi * k / 23)
            T = look_T(np.array([eye[0], 1.35, eye[1]]), yaw=yaw + sweep, pitch=-0.1 + 0.3 * np.sin(2 * np.pi * k / 17))
            self.frames.append(Frame(t=k / 30, w=W, h=H, K=K.copy(), T=T))
        self._cache = {}

    def _render(self, i):
        import cv2
        if i in self._cache:
            return self._cache[i]
        f = self.frames[i]
        vv, uu = np.mgrid[0:H, 0:W].astype(np.float64)
        d_cv = np.stack([(uu - K[0, 2]) / FX, (vv - K[1, 2]) / FX, np.ones_like(uu)], -1)
        d = d_cv @ (f.T[:3, :3] @ F_CV_ARKIT).T
        o = f.T[:3, 3]
        t = np.full((H, W), np.inf)
        for ax in range(3):
            for wall in (0.0, ROOM[ax]):
                with np.errstate(divide="ignore", invalid="ignore"):
                    tt = (wall - o[ax]) / d[..., ax]
                tt[~np.isfinite(tt) | (tt <= 0)] = np.inf
                t = np.minimum(t, tt)
        P = o + d * t[..., None]
        chk = ((np.floor(P[..., 0] * 4) + np.floor(P[..., 1] * 4) + np.floor(P[..., 2] * 4)) % 2)
        rgb = np.stack([150 + 60 * chk, 140 + 50 * chk, 120 + 40 * chk], -1).astype(np.uint8)
        depth = t.copy()
        for nid, (c, n) in NODES.items():
            c, n = np.array(c), np.array(n, float)
            up = np.array([0, 1.0, 0])
            r = np.cross(-n, up)
            corners = [c - r * TAG / 2 + up * TAG / 2, c + r * TAG / 2 + up * TAG / 2,
                       c + r * TAG / 2 - up * TAG / 2, c - r * TAG / 2 - up * TAG / 2]
            pr = [project(K, f.T, q) for q in corners]
            if any(p is None for p in pr):
                continue
            # only draw tags whose front faces the camera
            if np.dot(o - c, n) <= 0:
                continue
            dst = np.array([[p[0], p[1]] for p in pr], np.float32)
            if dst[:, 0].max() < 0 or dst[:, 0].min() >= W or dst[:, 1].max() < 0 or dst[:, 1].min() >= H:
                continue
            tag = self.tags[nid]
            src = np.array([[0, 0], [tag.shape[1], 0], [tag.shape[1], tag.shape[0]], [0, tag.shape[0]]], np.float32)
            Hm = cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(tag, Hm, (W, H), flags=cv2.INTER_AREA)
            mask = cv2.warpPerspective(np.full(tag.shape[:2], 255, np.uint8), Hm, (W, H)) > 128
            rgb[mask] = warped[mask]
        self._cache = {i: (rgb, (depth * 1000).clip(0, 65535).astype(np.uint16))}
        return self._cache[i]

    def iter_images(self, indices):
        for i in sorted(set(indices)):
            yield i, self._render(i)[0]

    @property
    def has_depth(self):
        return True

    def depth(self, i):
        import cv2
        return cv2.resize(self._render(i)[1], (256, 192), interpolation=cv2.INTER_NEAREST)

    def conf(self, i):
        return np.full((192, 256), 2, np.uint8)


def make_session(root: Path) -> Path:
    from writers import write_r2s_raw
    sess = root / "synth_session"
    write_r2s_raw(SynthRoom(), sess, log=lambda *a: None)
    m = json.loads((sess / "session.json").read_text())
    assert m.get("format") == "r2s-capture", m
    return sess


def check(cond, msg):
    print(("  ok    " if cond else "  FAIL  ") + msg)
    if not cond:
        check.failed = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep", type=Path)
    a = ap.parse_args()
    check.failed = False
    root = a.keep or Path(tempfile.mkdtemp(prefix="esp_test_"))
    root.mkdir(parents=True, exist_ok=True)
    try:
        print("synthetic session…")
        sess = make_session(root)

        print("esp_locate auto…")
        esp_locate.main(["auto", str(sess), "--stride", "1"])
        doc = load_nodes(sess)
        got = {n["node_id"]: n for n in doc["nodes"]}
        check(set(got) == set(NODES), f"all nodes found: {sorted(got)}")
        for nid, (c, nrm) in NODES.items():
            if nid in got:
                err = np.linalg.norm(np.array(got[nid]["position"]) - np.array(c))
                ang = np.degrees(np.arccos(np.clip(np.dot(got[nid]["normal"], nrm), -1, 1)))
                check(err < 0.02 and ang < 10, f"node {nid}: position error {err * 100:.1f} cm, normal error {ang:.1f} deg")

        print("esp_locate click (one click + depth, two clicks without)…")
        ep = esp_locate.open_session(sess)
        clicks = []
        for nid, (c, _) in NODES.items():
            seen = []
            for i, f in enumerate(ep.frames):
                pr = project(f.K, f.T, np.array(c))
                if pr and 40 < pr[0] < W - 40 and 40 < pr[1] < H - 40 and np.dot(f.T[:3, 3] - c, NODES[nid][1]) > 0:
                    seen.append((i, pr))
            assert len(seen) >= 2, f"node {nid} never in view"
            i, pr = seen[len(seen) // 2]
            clicks.append({"node_id": nid, "frame": i, "u": pr[0] + 0.7, "v": pr[1] - 0.5})
        res = esp_locate.solve_clicks(ep, clicks)
        for nid, (c, _) in NODES.items():
            err = np.linalg.norm(np.array(res[nid]["position"]) - np.array(c))
            check(err < 0.05, f"node {nid} one click + depth: {err * 100:.1f} cm ({res[nid]['method']})")
        ep2 = esp_locate.open_session(sess)
        ep2.depth = lambda i: None
        two = []
        for nid, (c, _) in NODES.items():
            seen = [(i, project(f.K, f.T, np.array(c))) for i, f in enumerate(ep2.frames)]
            seen = [(i, p) for i, p in seen if p and 40 < p[0] < W - 40 and 40 < p[1] < H - 40
                    and np.dot(ep2.frames[i].T[:3, 3] - c, NODES[nid][1]) > 0]
            for i, p in (seen[0], seen[-1]):
                two.append({"node_id": nid, "frame": i, "u": p[0], "v": p[1]})
        res2 = esp_locate.solve_clicks(ep2, two)
        for nid, (c, _) in NODES.items():
            if nid in res2:
                err = np.linalg.norm(np.array(res2[nid]["position"]) - np.array(c))
                check(err < 0.05 and res2[nid]["method"] == "triangulated",
                      f"node {nid} two clicks, no depth: {err * 100:.1f} cm, rays {res2[nid]['ray_angle_deg']} deg apart")
            else:
                check(False, f"node {nid} two clicks, no depth: not solved (views too similar)")

        print("esp_locate show…")
        esp_locate.main(["show", str(sess)])
        check((sess / "esp_nodes_check.jpg").exists(), "check image written")

        print("esp_place from the session geometry…")
        esp_place.main(["--session", str(sess), "--out", str(root / "plan")])
        plan = json.loads((root / "plan" / "esp_plan.json").read_text())
        P = np.array([n["position"] for n in plan["nodes"]])
        near_wall = np.minimum.reduce([P[:, 0], ROOM[0] - P[:, 0], P[:, 2], ROOM[2] - P[:, 2]])
        check(len(P) == 6 and near_wall.max() < 0.35, f"6 nodes on walls (max {near_wall.max():.2f} m from a wall)")
        check(plan["coverage"]["floor_2plus_links"] > 0.8, f"plan covers {plan['coverage']['floor_2plus_links'] * 100:.0f} % with 2+ links")
        esp_place.main(["--session", str(sess), "--out", str(root / "plan"), "--check", str(sess / "esp_nodes.json")])

        print("esp_fleet packet parsing…")
        csi = struct.pack("<IBBHIIbbBB", 0xC5110001, 4, 1, 64, 2437, 77, -61, -92, 0, 0) + bytes(256)
        p = esp_fleet.parse_packet(csi)
        check(p["node"] == 4 and p["n_sub"] == 64 and p["rssi"] == -61 and p["freq_mhz"] == 2437, f"CSI header {p}")
        vit = struct.pack("<IBBHIbBHffII", 0xC5110002, 2, 0b011, 1650, 715000, -48, 1, 0, 0.4, 0.8, 999, 0)
        p = esp_fleet.parse_packet(vit)
        check(p["node"] == 2 and p["presence"] and p["fall"] and abs(p["breathing_bpm"] - 16.5) < 1e-6
              and abs(p["heart_bpm"] - 71.5) < 1e-6 and p["persons"] == 1, f"vitals {p}")
    finally:
        if not a.keep:
            shutil.rmtree(root, ignore_errors=True)
    print("FAILED" if check.failed else "all passed")
    sys.exit(1 if check.failed else 0)


if __name__ == "__main__":
    main()
