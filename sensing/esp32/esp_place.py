#!/usr/bin/env python3
"""Where to put the 6 ESP32 nodes: a placement plan computed from your room scan.

WiFi CSI sees a person when they disturb a radio link, mostly inside the first Fresnel zones around the
straight line between two radios. With 6 nodes there are 15 node-to-node links; this tool picks wall
positions and heights so those links criss-cross the room's floor area from many angles, at the heights
where bodies are, while keeping the nodes apart and off the floor.

  python esp_place.py --session /path/to/capture_session          # mesh or depth from the phone scan
  python esp_place.py --room 5.0x4.0x2.5                            # plain rectangular room, metres
  python esp_place.py --session S --check S/esp_nodes.json          # score where the nodes actually are

Writes <out>/esp_plan.json (positions in the session world, same frame as esp_nodes.json) and
<out>/esp_plan.png (top view: coverage, nodes, links). See PLACEMENT.md for the reasoning and the rules
of thumb this encodes.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from esp_common import NODE_COUNT, REPO, load_nodes, now_iso  # noqa: E402

CELL = 0.10                    # grid cell (m)
LAMBDA = 0.125                 # 2.4 GHz wavelength (m)
EXCESS = 0.20                  # link "sees" a point if the detour via it is <= this (m), ~ +-0.6 m at mid-link
BODY_H = (0.5, 1.0, 1.5)       # sample heights above the floor where a standing / sitting body is
TIERS = (0.9, 1.9)             # mounting heights above the floor (desk / outlet level, top of shelf)
MIN_SEP = 1.5                  # nodes closer than this (horizontally) add little new geometry


# ------------------------------------------------------------------------------------ geometry input
def room_from_box(L: float, W: float, H: float):
    """Synthetic points for a rectangular room: floor at y=0, x in [0, L], z in [0, W]."""
    pts = []
    for y in np.arange(0, H + 1e-6, 0.1):
        pts += [(x, y, z) for x in np.arange(0, L + 1e-6, 0.03) for z in (0.0, W)]
        pts += [(x, y, z) for z in np.arange(0, W + 1e-6, 0.03) for x in (0.0, L)]
    xs, zs = np.meshgrid(np.arange(0, L, 0.1), np.arange(0, W, 0.1))
    pts += [(x, 0.0, z) for x, z in zip(xs.ravel(), zs.ravel())]
    return np.array(pts, float), None


def room_from_session(path: Path, stride: int = 15):
    sys.path.insert(0, str(REPO / "tools"))
    from readers import open_episode  # noqa: E402
    ep = open_episode(path)
    m = ep.mesh()
    if m is not None and len(m.xyz) > 1000:
        pts = m.xyz
    else:
        from writers.meshtools import fuse_points  # noqa: E402
        idx = [i for i, f in enumerate(ep.frames) if f.T is not None][::stride]
        views = []
        for i, img in ep.iter_images(idx):
            f = ep.frames[i]
            views.append((f.K, f.T, img, ep.depth(i), ep.conf(i)))
        pts, _ = fuse_points(views, voxel=0.04, max_depth=6.0)
    if len(pts) < 500:
        sys.exit(f"{path}: not enough geometry in the scan ({len(pts)} points)")
    return np.asarray(pts, float), ep.room()


class Room:
    """Occupancy grid (x/z, y up) with wall cells, interior (target) cells and wall-mount candidates."""

    def __init__(self, pts: np.ndarray, roomplan: dict | None = None):
        from scipy import ndimage
        y = pts[:, 1]
        self.floor = float(np.percentile(y, 2))
        top = float(np.percentile(y, 99))
        self.height = max(2.2, min(3.5, top - self.floor))
        self.x0, self.z0 = pts[:, 0].min() - 0.3, pts[:, 2].min() - 0.3
        nx = int(np.ceil((pts[:, 0].max() + 0.3 - self.x0) / CELL))
        nz = int(np.ceil((pts[:, 2].max() + 0.3 - self.z0) / CELL))
        self.shape = (nz, nx)
        ix = ((pts[:, 0] - self.x0) / CELL).astype(int)
        iz = ((pts[:, 2] - self.z0) / CELL).astype(int)
        h = y - self.floor
        tall = np.zeros(self.shape, int)
        np.add.at(tall, (iz[(h > 1.3) & (h < 2.1)], ix[(h > 1.3) & (h < 2.1)]), 1)
        anyp = np.zeros(self.shape, int)
        np.add.at(anyp, (iz, ix), 1)
        wall = tall >= max(2, np.percentile(tall[tall > 0], 30) if (tall > 0).any() else 2)
        if roomplan:
            for wl in roomplan.get("walls", []):
                T = np.array(wl["T"], float).reshape(4, 4)
                w = float(wl["dims"][0])
                for s in np.linspace(-w / 2, w / 2, max(2, int(w / (CELL / 2)))):
                    p = T[:3, :3] @ np.array([s, 0, 0]) + T[:3, 3]
                    j, i = int((p[0] - self.x0) / CELL), int((p[2] - self.z0) / CELL)
                    if 0 <= i < nz and 0 <= j < nx:
                        wall[i, j] = True
        wall = ndimage.binary_closing(wall, iterations=2)
        # Room interior: inside closed walls; walls with gaps (doors, unscanned bits) fall back to the
        # convex hull of everything scanned.
        closed = ndimage.binary_closing(wall, structure=np.ones((3, 3)), iterations=5)
        inside = ndimage.binary_fill_holes(closed) & ~wall
        import cv2
        hull = np.zeros(self.shape, np.uint8)
        yy, xx = np.nonzero(anyp > 0)
        cv2.fillConvexPoly(hull, cv2.convexHull(np.stack([xx, yy], 1).astype(np.int32)), 1)
        hull = hull.astype(bool) & ~wall
        if inside.sum() < 0.5 * hull.sum():
            inside = hull
        self.wall = wall
        self.target = ndimage.binary_erosion(inside, iterations=2)   # >= 0.2 m from walls
        if self.target.sum() < 20:
            self.target = inside
        # Candidate mounts: inside cells touching a wall, spaced ~0.3 m apart, at two heights.
        edge = inside & ndimage.binary_dilation(wall, iterations=1)
        ii, jj = np.nonzero(edge)
        cand, taken = [], set()
        for i, j in sorted(zip(ii, jj)):
            key = (i // 3, j // 3)
            if key in taken:
                continue
            taken.add(key)
            x = self.x0 + (j + 0.5) * CELL          # the inside cell touching the wall: ~5-15 cm in front of it
            z = self.z0 + (i + 0.5) * CELL
            for hgt in TIERS:
                cand.append((x, self.floor + hgt, z))
        self.cand = np.array(cand, float)
        ti, tj = np.nonzero(self.target)
        cx = self.x0 + (tj + 0.5) * CELL
        cz = self.z0 + (ti + 0.5) * CELL
        self.cells = np.stack([cx, cz], 1)
        self.samples = np.concatenate([np.stack([cx, np.full_like(cx, self.floor + hb), cz], 1) for hb in BODY_H])

    def to_px(self, x, z):
        return (np.asarray(x) - self.x0) / CELL, (np.asarray(z) - self.z0) / CELL


# ------------------------------------------------------------------------------------ scoring
class Scorer:
    def __init__(self, room: Room, positions: np.ndarray):
        self.room = room
        self.P = np.asarray(positions, float)
        self.cache: dict[tuple[int, int], tuple[np.ndarray, int]] = {}
        self.nc = len(room.cells)

    def link(self, a: int, b: int):
        key = (min(a, b), max(a, b))
        if key not in self.cache:
            A, B = self.P[key[0]], self.P[key[1]]
            S = self.room.samples
            ex = np.linalg.norm(S - A, axis=1) + np.linalg.norm(S - B, axis=1) - np.linalg.norm(A - B)
            seen = (ex <= EXCESS).reshape(len(BODY_H), self.nc).any(0)
            d = B - A
            ang = min(5, int((np.degrees(np.arctan2(d[2], d[0])) % 180) // 30))   # 6 direction bins
            self.cache[key] = (seen, ang)
        return self.cache[key]

    def score(self, sel: list[int], detail: bool = False):
        cov = np.zeros(self.nc)
        dirs = np.zeros((6, self.nc), bool)
        for a, b in itertools.combinations(sel, 2):
            seen, ang = self.link(a, b)
            cov += seen
            dirs[ang] |= seen
        ndir = dirs.sum(0)
        per_cell = 0.6 * np.minimum(cov, 4) / 4 + 0.4 * np.minimum(ndir, 3) / 3
        s = float(per_cell.mean())
        pen = 0.0
        for a, b in itertools.combinations(sel, 2):
            d = np.linalg.norm((self.P[a] - self.P[b])[[0, 2]])
            if d < MIN_SEP:
                pen += 0.05 * (MIN_SEP - d) / MIN_SEP
        hs = np.array([self.P[k][1] - self.room.floor for k in sel])
        need = len(sel) // 3                              # mix heights: links then also slope through the room
        pen += 0.03 * max(0, need - int((hs > 1.4).sum())) + 0.03 * max(0, need - int((hs <= 1.4).sum()))
        if detail:
            return s - pen, {"coverage": cov, "directions": ndir, "covered_2plus": float((cov >= 2).mean()),
                             "covered_3dirs": float((ndir >= 3).mean()), "mean_links": float(cov.mean())}
        return s - pen


def optimise(sc: Scorer, n: int, iters: int = 4) -> list[int]:
    cands = list(range(len(sc.P)))
    # Seed with the two most distant candidates, then greedy, then 1-swap local search.
    D = np.linalg.norm(sc.P[:, None, [0, 2]] - sc.P[None, :, [0, 2]], axis=2)
    a, b = np.unravel_index(np.argmax(D), D.shape)
    sel = [int(a), int(b)]
    while len(sel) < n:
        best = max((c for c in cands if c not in sel), key=lambda c: sc.score(sel + [c]))
        sel.append(best)
    cur = sc.score(sel)
    for _ in range(iters):
        improved = False
        for k in range(n):
            for c in cands:
                if c in sel:
                    continue
                trial = sel[:k] + [c] + sel[k + 1:]
                s = sc.score(trial)
                if s > cur + 1e-6:
                    sel, cur, improved = trial, s, True
        if not improved:
            break
    return sel


# ------------------------------------------------------------------------------------ output
def describe(room: Room, p: np.ndarray) -> str:
    """'0.9 m high, 1.3 m from the corner' style hint in the scan's top view coordinates."""
    h = p[1] - room.floor
    x_rel = p[0] - (room.x0 + 0.3)
    z_rel = p[2] - (room.z0 + 0.3)
    return f"{h:.1f} m above the floor, at x={x_rel:.1f} m, z={z_rel:.1f} m on the plan"


def draw(room: Room, P: np.ndarray, labels: list[str], detail: dict, out: Path, title: str):
    import cv2
    nz, nx = room.shape
    s = max(8, int(np.ceil(720 / nx)))
    img = np.full((nz * s, nx * s, 3), 255, np.uint8)
    cov = np.zeros(room.shape)
    ti, tj = np.nonzero(room.target)
    cov[ti, tj] = np.minimum(detail["coverage"], 6) / 6
    big = cv2.resize(cov, (nx * s, nz * s), interpolation=cv2.INTER_NEAREST)
    tgt = cv2.resize(room.target.astype(np.uint8), (nx * s, nz * s), interpolation=cv2.INTER_NEAREST) > 0
    heat = cv2.applyColorMap((big * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    img[tgt] = heat[tgt]
    wall = cv2.resize(room.wall.astype(np.uint8), (nx * s, nz * s), interpolation=cv2.INTER_NEAREST) > 0
    img[wall] = (90, 90, 90)
    pts = [(int(room.to_px(p[0], p[2])[0] * s), int(room.to_px(p[0], p[2])[1] * s)) for p in P]
    for a, b in itertools.combinations(range(len(P)), 2):
        cv2.line(img, pts[a], pts[b], (255, 255, 255), 1, cv2.LINE_AA)
    for (u, v), lab, p in zip(pts, labels, P):
        hi = p[1] - room.floor > 1.4
        cv2.circle(img, (u, v), 11, (0, 0, 220) if hi else (0, 140, 255), -1, cv2.LINE_AA)
        cv2.putText(img, lab, (u - 6, v + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    pad = np.full((70, img.shape[1], 3), 255, np.uint8)
    cv2.putText(pad, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(pad, "colour = links crossing each floor cell (dark 0 .. yellow 6+)   red = high (~1.9 m)   "
                "orange = low (~0.9 m)", (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(pad, f"x right, z down, {CELL * 100:.0f} cm cells", (8, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(str(out), np.vstack([pad, img]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--session", type=Path, help="capture session folder (any format tools/convert.py reads)")
    g.add_argument("--room", help="LxWxH in metres for a plain rectangular room, e.g. 5x4x2.5")
    ap.add_argument("--nodes", type=int, default=NODE_COUNT)
    ap.add_argument("--check", type=Path, help="esp_nodes.json to score instead of planning")
    ap.add_argument("--out", type=Path, help="output folder (default: the session folder, or .)")
    a = ap.parse_args(argv)

    if a.session:
        pts, rp = room_from_session(a.session)
    else:
        L, W, H = (float(v) for v in a.room.lower().split("x"))
        pts, rp = room_from_box(L, W, H)
    out = a.out or (a.session if a.session else Path("."))
    out.mkdir(parents=True, exist_ok=True)
    room = Room(pts, rp)
    if len(room.cand) < a.nodes:
        sys.exit("could not find enough wall positions in the scan; scan the walls too, or use --room")

    if a.check:
        doc = load_nodes(a.check)
        P = np.array([n["position"] for n in doc["nodes"]], float)
        labels = [str(n["node_id"]) for n in doc["nodes"]]
        sc = Scorer(room, P)
        total, det = sc.score(list(range(len(P))), detail=True)
        draw(room, P, labels, det, out / "esp_check.png", f"actual placement: score {total:.2f}")
        print(f"score {total:.2f}: {det['covered_2plus'] * 100:.0f} % of the floor crossed by 2+ links, "
              f"{det['covered_3dirs'] * 100:.0f} % from 3+ directions, {det['mean_links']:.1f} links per spot on average")
        for (i, n), p in zip(enumerate(doc["nodes"]), P):
            h = p[1] - room.floor
            notes = []
            if h < 0.4:
                notes.append("very low: furniture and people's legs will block it, raise to 0.8 m or more")
            if h > room.height - 0.2:
                notes.append("at the ceiling: fine, but keep the antenna away from metal light fittings")
            near = [m["node_id"] for m, q in zip(doc["nodes"], P) if m is not n and np.linalg.norm((q - p)[[0, 2]]) < MIN_SEP]
            if near:
                notes.append(f"close to node(s) {near}: move apart for more independent links")
            print(f"  node {n['node_id']}: {h:.2f} m high{'; ' + '; '.join(notes) if notes else ''}")
        print(f"wrote {out / 'esp_check.png'}")
        return

    sc = Scorer(room, room.cand)
    sel = optimise(sc, a.nodes)
    # Order nodes around the room (angle about the centre) so ids go round the walls.
    c = room.cells.mean(0)
    sel.sort(key=lambda k: np.arctan2(sc.P[k][2] - c[1], sc.P[k][0] - c[0]))
    total, det = sc.score(sel, detail=True)
    P = sc.P[sel]
    plan = {"version": 1, "format": "r2s-esp-nodes", "frame": "session_world", "units": "m", "up": "+y",
            "floor_y": room.floor, "score": round(total, 3),
            "coverage": {"floor_2plus_links": round(det["covered_2plus"], 3),
                         "floor_3plus_directions": round(det["covered_3dirs"], 3)},
            "nodes": [{"node_id": k + 1, "label": f"esp-{k + 1}", "position": [round(float(v), 3) for v in p],
                       "height_above_floor": round(float(p[1] - room.floor), 2), "method": "planned",
                       "updated": now_iso()} for k, p in enumerate(P)]}
    (out / "esp_plan.json").write_text(json.dumps(plan, indent=1))
    draw(room, P, [str(k + 1) for k in range(len(P))], det, out / "esp_plan.png",
         f"suggested placement for {a.nodes} nodes: score {total:.2f}")
    print(f"plan score {total:.2f}: {det['covered_2plus'] * 100:.0f} % of the floor crossed by 2+ links, "
          f"{det['covered_3dirs'] * 100:.0f} % from 3+ directions")
    for k, p in enumerate(P):
        print(f"  node {k + 1}: {describe(room, p)}")
    print(f"wrote {out / 'esp_plan.json'} and {out / 'esp_plan.png'}")


if __name__ == "__main__":
    main()
