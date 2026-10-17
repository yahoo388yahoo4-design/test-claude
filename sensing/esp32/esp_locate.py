#!/usr/bin/env python3
"""Put the ESP32 nodes into the 3D scan: find them in a recorded capture session.

Two ways, both writing <session>/esp_nodes.json (session world: y up, metres; the frame the poses, mesh
and depth of the session use):

  auto   finds the printed node tags (esp_tags.py) in the video. Each sighting gives the tag's 3D pose
         from the ArUco corners and the camera pose; sightings are combined per node with outlier
         rejection, so a few seconds of the tag in view while you scan is enough.
           python esp_locate.py auto  SESSION
  click  you mark the board in a frame. With depth (LiDAR / ARCore depth) one click is enough; without
         depth, click it in two or more frames from different angles and the rays are intersected.
           python esp_locate.py click SESSION                      # window: pick frame, press 1-6, click
           python esp_locate.py click SESSION --clicks clicks.json # [{"node_id":1,"frame":120,"u":812,"v":433}]
  show   prints the nodes and writes esp_nodes_check.jpg: every node reprojected into the frame that saw it best.

A position placed by a click (in the app or here) is not overwritten by `auto` unless you pass --force.
The phone apps write the same file live while scanning (tag tracking, or tap on the ESP).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from esp_common import (F_CV_ARKIT, HERE, NODE_COUNT, REPO, depth_at, load_nodes, now_iso, pixel_ray,  # noqa: E402
                        project, robust_center, save_nodes, triangulate, unproject, upsert_node)

sys.path.insert(0, str(REPO / "tools"))


def open_session(path: Path):
    from readers import open_episode  # noqa: E402
    return open_episode(path)


def tag_manifest() -> dict:
    p = HERE / "tags" / "esp_tags.json"
    if p.exists():
        return json.loads(p.read_text())
    from esp_common import ARUCO_DICT, ARUCO_SIZE_M
    return {"aruco_dict": ARUCO_DICT, "aruco_size_m": ARUCO_SIZE_M}


# ------------------------------------------------------------------------------------ auto
def detect_sightings(ep, stride: int = 2, ids=range(1, NODE_COUNT + 1), max_dist: float = 5.0, log=print):
    """[(node_id, frame_idx, world_pos, world_normal, weight, corners(4,2), reproj_px, depth_check_m)]"""
    import cv2
    man = tag_manifest()
    s = float(man["aruco_size_m"])
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, man["aruco_dict"])),
                                  _params(cv2))
    obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float64)
    idx = [i for i, f in enumerate(ep.frames) if f.T is not None][::max(1, stride)]
    out = []
    ids = set(ids)
    for n, (i, img) in enumerate(ep.iter_images(idx)):
        f = ep.frames[i]
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        corners, found, _ = det.detectMarkers(gray)
        if found is None:
            continue
        for c, mid in zip(corners, found.ravel()):
            mid = int(mid)
            if mid not in ids:
                continue
            c = c.reshape(4, 2).astype(np.float64)
            side = float(np.mean(np.linalg.norm(c - np.roll(c, 1, 0), axis=1)))
            if side < 18:                # too small for a stable pose (and the normal can flip)
                continue
            ok, rv, tv = cv2.solvePnP(obj, c, f.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            proj, _ = cv2.projectPoints(obj, rv, tv, f.K, None)
            err = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - c) ** 2, 1))))
            tv = tv.ravel()
            if err > 3.0 or not (0.05 < tv[2] < max_dist):
                continue
            R = cv2.Rodrigues(rv)[0]
            p_world = f.T[:3, :3] @ (F_CV_ARKIT @ tv) + f.T[:3, 3]
            n_world = f.T[:3, :3] @ (F_CV_ARKIT @ R[:, 2])
            dchk = None
            dep = ep.depth(i) if ep.has_depth else None
            if dep is not None:
                # Depth sensor refines what a small marker's PnP is weakest at: range and wall normal.
                uc, vc, _ = project(f.K, f.T, p_world) or (*c.mean(0), 0)
                dz = depth_at(dep, f.w, f.h, uc, vc, radius_px=1)
                if dz is not None:
                    dchk = dz - float(tv[2])
                    if abs(dchk) < max(0.05, 0.05 * tv[2]):
                        p_world = unproject(f.K, f.T, uc, vc, dz)
                    plane_n = _depth_normal(dep, f, c, side)
                    if plane_n is not None:          # the tag lies flat on that surface
                        n_world = plane_n
            # Bigger, sharper, more frontal sightings count more.
            frontal = abs(float(R[2, 2]))
            out.append((mid, i, p_world, n_world / np.linalg.norm(n_world), side * side * (0.3 + frontal) / (1 + err),
                        c, err, dchk))
        if log and n % 100 == 0:
            log(f"  frame {i}/{len(ep.frames)}: {len(out)} tag sightings so far")
    return out


def _depth_normal(dep: np.ndarray, f, corners: np.ndarray, side: float):
    """Normal of the surface around the tag from the depth map (plane fit), pointing at the camera."""
    dh, dw = dep.shape
    uc, vc = corners.mean(0)
    r = 1.5 * side
    u0, u1 = int(max(0, (uc - r) * dw / f.w)), int(min(dw, (uc + r) * dw / f.w + 1))
    v0, v1 = int(max(0, (vc - r) * dh / f.h)), int(min(dh, (vc + r) * dh / f.h + 1))
    if (u1 - u0) * (v1 - v0) < 9:
        return None
    vv, uu = np.mgrid[v0:v1, u0:u1]
    z = dep[v0:v1, u0:u1].astype(np.float64) / 1000
    m = z > 0.05
    if m.sum() < 9:
        return None
    u = (uu[m] + 0.5) * f.w / dw
    v = (vv[m] + 0.5) * f.h / dh
    P = np.stack([(u - f.K[0, 2]) / f.K[0, 0] * z[m], (v - f.K[1, 2]) / f.K[1, 1] * z[m], z[m]], 1)
    Pc = P - np.median(P, 0)
    keep = np.abs(Pc[:, 2]) < 0.05            # ignore other surfaces in the patch
    if keep.sum() < 9:
        return None
    w, V = np.linalg.eigh(np.cov(Pc[keep].T))
    if w[0] > 0.2 * w[1]:                      # not planar
        return None
    n_cv = V[:, 0] * (-1 if V[2, 0] > 0 else 1)  # face the camera (camera looks along +z in OpenCV)
    n = f.T[:3, :3] @ (F_CV_ARKIT @ n_cv)
    return n / np.linalg.norm(n)


def _params(cv2):
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


def aggregate(sightings) -> dict[int, dict]:
    res = {}
    by = {}
    for s in sightings:
        by.setdefault(s[0], []).append(s)
    for nid, ss in sorted(by.items()):
        P = np.array([s[2] for s in ss])
        W = np.array([s[4] for s in ss])
        c, m, spread = robust_center(P, W, inlier_m=0.05)
        N = np.array([s[3] for s in ss])[m]
        nrm = N.mean(0)
        nrm = nrm / (np.linalg.norm(nrm) + 1e-9)
        best = max((s for s, keep in zip(ss, m) if keep), key=lambda s: s[4])
        dchk = [s[7] for s, keep in zip(ss, m) if keep and s[7] is not None]
        conf = float(min(1.0, m.sum() / 8) * np.exp(-spread / 0.05))
        res[nid] = {"node_id": nid, "position": c.tolist(), "normal": nrm.tolist(), "method": "auto_tag",
                    "frame_index": int(best[1]), "t": None, "confidence": round(conf, 3),
                    "observations": int(m.sum()), "outliers": int((~m).sum()), "spread_m": round(spread, 4),
                    "depth_minus_tag_m": round(float(np.median(dchk)), 3) if dchk else None,
                    "updated": now_iso()}
    return res


def cmd_auto(a):
    ep = open_session(a.session)
    if not any(f.T is not None for f in ep.frames):
        sys.exit("this session has no camera poses (mode B/C?): run tools/recover_poses.py first, or use `click`")
    print(f"{ep.name}: {len(ep.frames)} frames, scanning every {a.stride}…")
    ss = detect_sightings(ep, a.stride, a.ids or range(1, NODE_COUNT + 1))
    found = aggregate(ss)
    doc = load_nodes(a.session)
    for nid, node in found.items():
        node["t"] = ep.frames[node["frame_index"]].t
        changed = upsert_node(doc, node, force=a.force)
        p = node["position"]
        print(f"  node {nid}: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m from {node['observations']} sightings, "
              f"spread {node['spread_m'] * 100:.1f} cm, confidence {node['confidence']:.2f}"
              f"{'' if changed else '  (kept the existing manual position; --force to replace)'}")
    missing = sorted(set(a.ids or range(1, NODE_COUNT + 1)) - set(found))
    if missing:
        print(f"  not seen: nodes {missing} (film each tag from 0.3-1.5 m for a second, or use `click`)")
    print(f"wrote {save_nodes(a.session, doc)}")


# ------------------------------------------------------------------------------------ click
def solve_clicks(ep, clicks: list[dict]) -> dict[int, dict]:
    """Clicks [{node_id, frame, u, v}] at native image pixels -> node entries."""
    res = {}
    by = {}
    for c in clicks:
        by.setdefault(int(c["node_id"]), []).append(c)
    for nid, cs in sorted(by.items()):
        pts, rays, frames = [], [], []
        for c in cs:
            f = ep.frames[int(c["frame"])]
            if f.T is None:
                continue
            u, v = float(c["u"]), float(c["v"])
            rays.append(pixel_ray(f.K, f.T, u, v))
            frames.append(int(c["frame"]))
            dep = ep.depth(int(c["frame"])) if ep.has_depth else None
            if dep is not None:
                z = depth_at(dep, f.w, f.h, u, v, radius_px=1)
                if z is not None:
                    pts.append(unproject(f.K, f.T, u, v, z))
        if len(rays) >= 2:
            p, rms, ang = triangulate(rays)
            if ang >= 5 and rms < 0.1:
                res[nid] = {"node_id": nid, "position": p.tolist(), "normal": None, "method": "triangulated",
                            "frame_index": frames[0], "confidence": round(float(np.exp(-rms / 0.03)), 3),
                            "observations": len(rays), "ray_rms_m": round(rms, 4), "ray_angle_deg": round(ang, 1)}
                continue
        if pts:
            c, _, spread = robust_center(np.array(pts))
            res[nid] = {"node_id": nid, "position": c.tolist(), "normal": None, "method": "tap_frame",
                        "frame_index": frames[0], "confidence": 0.9 if len(pts) > 1 else 0.7,
                        "observations": len(pts), "spread_m": round(spread, 4)}
        else:
            print(f"  node {nid}: no depth at the click{'s' if len(cs) > 1 else ''} and fewer than two views "
                  "at 5 degrees apart: click it in another frame from a different angle")
    for nid, r in res.items():
        r["t"] = ep.frames[r["frame_index"]].t
        r["updated"] = now_iso()
    return res


def click_gui(ep, start: int = 0) -> list[dict]:
    import cv2
    clicks = []
    posed = [i for i, f in enumerate(ep.frames) if f.T is not None]
    state = {"k": max(0, min(len(posed) - 1, start)), "node": 1, "img": None, "scale": 1.0, "cache": {}}

    def load():
        i = posed[state["k"]]
        if i not in state["cache"]:
            state["cache"] = {i: next(iter(ep.iter_images([i])))[1]}
        img = state["cache"][i]
        sc = min(1.0, 1400 / img.shape[1])
        state["img"], state["scale"] = cv2.resize(img, None, fx=sc, fy=sc)[:, :, ::-1].copy(), sc
        return i

    def redraw():
        i = posed[state["k"]]
        vis = state["img"].copy()
        for c in clicks:
            if c["frame"] == i:
                cv2.drawMarker(vis, (int(c["u"] * state["scale"]), int(c["v"] * state["scale"])), (0, 0, 255),
                               cv2.MARKER_CROSS, 24, 2)
                cv2.putText(vis, str(c["node_id"]), (int(c["u"] * state["scale"]) + 8, int(c["v"] * state["scale"]) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        txt = (f"frame {i}  node {state['node']} (keys 1-{NODE_COUNT})  a/d: -/+1  A/D: -/+30  "
               f"click: mark  u: undo  s: save+quit  q: quit")
        cv2.putText(vis, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(vis, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.imshow("esp_locate", vis)

    def on_mouse(ev, x, y, flags, _):
        if ev == cv2.EVENT_LBUTTONDOWN:
            clicks.append({"node_id": state["node"], "frame": posed[state["k"]],
                           "u": x / state["scale"], "v": y / state["scale"]})
            redraw()

    cv2.namedWindow("esp_locate", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("esp_locate", on_mouse)
    load()
    redraw()
    while True:
        key = cv2.waitKey(50) & 0xFF
        if key == 255:
            continue
        ch = chr(key)
        step = {"a": -1, "d": 1, "A": -30, "D": 30}.get(ch)
        if step:
            state["k"] = max(0, min(len(posed) - 1, state["k"] + step))
            load()
        elif ch.isdigit() and 1 <= int(ch) <= NODE_COUNT:
            state["node"] = int(ch)
        elif ch == "u" and clicks:
            clicks.pop()
        elif ch == "s":
            break
        elif ch == "q":
            clicks.clear()
            break
        redraw()
    cv2.destroyAllWindows()
    return clicks


def cmd_click(a):
    ep = open_session(a.session)
    if a.clicks:
        clicks = json.loads(Path(a.clicks).read_text())
    else:
        clicks = click_gui(ep, a.frame)
        if clicks:
            (a.session / "esp_clicks.json").write_text(json.dumps(clicks, indent=1))
    if not clicks:
        print("no clicks")
        return
    res = solve_clicks(ep, clicks)
    doc = load_nodes(a.session)
    for nid, node in res.items():
        upsert_node(doc, node, force=True)
        p = node["position"]
        print(f"  node {nid}: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m  [{node['method']}, {node['observations']} click(s)]")
    print(f"wrote {save_nodes(a.session, doc)}")


# ------------------------------------------------------------------------------------ show
def cmd_show(a):
    import cv2
    ep = open_session(a.session)
    doc = load_nodes(a.session)
    if not doc["nodes"]:
        sys.exit("no esp_nodes.json yet")
    tiles = []
    for n in doc["nodes"]:
        p = np.array(n["position"])
        print(f"  node {n['node_id']}: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}) m  {n.get('method')}  "
              f"confidence {n.get('confidence', '-')}")
        fi = n.get("frame_index")
        if fi is None or fi >= len(ep.frames) or ep.frames[fi].T is None:
            # pick the frame looking most directly at the node
            best, fi = -1, None
            for i, f in enumerate(ep.frames[::5]):
                if f.T is None:
                    continue
                pr = project(f.K, f.T, p)
                if pr and 0 <= pr[0] < f.w and 0 <= pr[1] < f.h and pr[2] < 4:
                    sc = -abs(pr[0] - f.w / 2) - abs(pr[1] - f.h / 2) - 200 * pr[2]
                    if sc > best or fi is None:
                        best, fi = sc, i * 5
            if fi is None:
                continue
        f = ep.frames[fi]
        img = next(iter(ep.iter_images([fi])))[1][:, :, ::-1].copy()
        pr = project(f.K, f.T, p)
        if pr:
            u, v = int(pr[0]), int(pr[1])
            cv2.circle(img, (u, v), 18, (0, 0, 255), 3)
            cv2.putText(img, f"ESP {n['node_id']}", (u + 22, v), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
            x0, y0 = max(0, u - 300), max(0, v - 225)
            img = img[y0:y0 + 450, x0:x0 + 600]
        img = cv2.resize(cv2.copyMakeBorder(img, 0, max(0, 450 - img.shape[0]), 0, max(0, 600 - img.shape[1]),
                                            cv2.BORDER_CONSTANT), (600, 450))
        cv2.putText(img, f"node {n['node_id']} frame {fi}", (8, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        tiles.append(img)
    if tiles:
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        sheet = np.vstack([np.hstack(tiles[k:k + 3]) for k in range(0, len(tiles), 3)])
        out = a.session / "esp_nodes_check.jpg"
        cv2.imwrite(str(out), sheet)
        print(f"wrote {out}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("auto", help="find the printed tags in the video")
    p.add_argument("session", type=Path)
    p.add_argument("--stride", type=int, default=2, help="look at every Nth frame (default 2)")
    p.add_argument("--ids", type=int, nargs="*", help="only these node ids")
    p.add_argument("--force", action="store_true", help="replace positions that were placed by hand")
    p.set_defaults(fn=cmd_auto)
    p = sub.add_parser("click", help="mark nodes by clicking them in frames")
    p.add_argument("session", type=Path)
    p.add_argument("--clicks", help="JSON list of {node_id, frame, u, v} (native image pixels) instead of the window")
    p.add_argument("--frame", type=int, default=0, help="start the window at this posed frame")
    p.set_defaults(fn=cmd_click)
    p = sub.add_parser("show", help="print nodes and write a check image")
    p.add_argument("session", type=Path)
    p.set_defaults(fn=cmd_show)
    a = ap.parse_args(argv)
    a.fn(a)


if __name__ == "__main__":
    main()
