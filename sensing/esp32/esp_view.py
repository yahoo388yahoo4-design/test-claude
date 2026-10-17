#!/usr/bin/env python3
"""Live 3D viewer: the room scan, the six ESP32 nodes, and where a person is with breathing and heart rate.

    python esp_view.py --session /path/to/session        # scan + <session>/esp_nodes.json
    python esp_view.py --nodes esp_plan.json             # no scan: just the nodes in 3D space
    python esp_view.py                                   # nodes at default spots until you place them
    python esp_view.py --demo                            # no boards: simulated person walking around

Then open http://localhost:8080 (also from a phone on the same network: http://<this computer>:8080).
It listens for the nodes' UDP stream itself (port 5005), so stop `esp_fleet.py monitor` first. To run the
RuView sensing server at the same time, start it on another port and add `--forward 127.0.0.1:5006`.
Everything it needs (three.js included) is in this folder; no internet needed.

The person marker is a rough estimate: the presence-weighted centre of the nodes that sense someone,
because the nodes' vitals packets carry no position. RuView's sensing server does proper localisation.
Breathing and heart rate are what the nodes estimate on board (firmware edge tier 2).
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import random
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from esp_common import NODE_COUNT, load_nodes  # noqa: E402
from esp_fleet import VITALS_MAGIC, parse_packet  # noqa: E402

WEB = HERE / "web"
HISTORY_S = 120


# ------------------------------------------------------------------------------------ scene
def default_nodes(n: int = NODE_COUNT) -> list[dict]:
    """Stand-in spots on a 2 m circle, alternating 0.9 / 1.9 m, for nodes nobody has placed yet."""
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        out.append({"node_id": i + 1, "label": f"esp-{i + 1}", "placed": False,
                    "position": [round(2 * math.cos(a), 3), 0.9 if i % 2 == 0 else 1.9, round(2 * math.sin(a), 3)]})
    return out


def load_scene(session: Path | None, nodes_file: Path | None, max_points: int) -> dict:
    scene = {"geometry": None, "source": None, "nodes": [], "floor_y": 0.0}
    src = nodes_file or session
    placed = {}
    if src is not None:
        for n in load_nodes(src)["nodes"]:
            if n.get("position") and len(n["position"]) == 3:
                placed[int(n["node_id"])] = {"node_id": int(n["node_id"]), "label": n.get("label", f"esp-{n['node_id']}"),
                                             "position": [float(v) for v in n["position"]], "placed": True,
                                             "method": n.get("method")}
    ids = sorted(set(placed) | set(range(1, NODE_COUNT + 1)))
    stand_in = {n["node_id"]: n for n in default_nodes(max(ids))}
    scene["nodes"] = [placed.get(i, stand_in[i]) for i in ids]
    if session is not None:
        scene["geometry"], scene["source"] = load_geometry(session, max_points)
    if scene["geometry"] is not None:
        scene["floor_y"] = scene["geometry"]["floor_y"]
    elif placed:
        scene["floor_y"] = min(n["position"][1] for n in placed.values()) - 0.9
    return scene


def load_geometry(session: Path, max_points: int):
    """mesh.ply if the session has one, else depth fused into points. Float32 arrays, base64."""
    import numpy as np
    from esp_place import room_from_session
    sys.path.insert(0, str(HERE.parent.parent / "tools"))
    from readers import open_episode  # noqa: E402

    ep = open_episode(session)
    m = ep.mesh()
    b64 = lambda a: base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()  # noqa: E731
    if m is not None and len(m.xyz) > 1000 and m.faces is not None and len(m.faces) <= 1_500_000:
        xyz = np.asarray(m.xyz, np.float32)
        faces = np.asarray(m.faces, np.uint32)
        floor = float(np.percentile(xyz[:, 1], 2))
        return {"kind": "mesh", "positions": b64(xyz), "index": b64(faces), "floor_y": floor,
                "count": int(len(xyz))}, "mesh.ply"
    pts, _ = room_from_session(session)
    pts = np.asarray(pts, np.float32)
    if len(pts) > max_points:
        pts = pts[np.random.default_rng(0).choice(len(pts), max_points, replace=False)]
    floor = float(np.percentile(pts[:, 1], 2))
    return {"kind": "points", "positions": b64(pts), "floor_y": floor, "count": int(len(pts))}, \
        ("mesh.ply (points)" if m is not None else "depth")


# ------------------------------------------------------------------------------------ live state
class Live:
    """Latest packets per node, the person estimate and a vitals history. Thread-safe."""

    def __init__(self, scene: dict):
        self.scene = scene
        self.lock = threading.Lock()
        self.nodes: dict[int, dict] = {}
        self.history: list[list] = []       # [t, breathing, heart, persons]
        self.person = None
        self.t0 = time.time()

    def packet(self, p: dict, ip: str):
        now = time.time()
        with self.lock:
            st = self.nodes.setdefault(p["node"], {"csi_times": [], "ip": ip})
            st["ip"] = ip
            st["seen"] = now
            if p["type"] == "csi":
                st["csi_times"].append(now)
                st["rssi"] = p["rssi"]
                st["freq_mhz"] = p["freq_mhz"]
            elif p["type"] == "vitals":
                st["vitals"] = p
                st["vitals_t"] = now

    def snapshot(self) -> dict:
        now = time.time()
        pos = {n["node_id"]: n["position"] for n in self.scene["nodes"]}
        with self.lock:
            nodes = []
            for nid, st in sorted(self.nodes.items()):
                st["csi_times"] = [t for t in st["csi_times"] if now - t < 2.0]
                v = st.get("vitals") if now - st.get("vitals_t", 0) < 5 else None
                nodes.append({
                    "node_id": nid, "ip": st["ip"], "age": round(now - st["seen"], 1),
                    "csi_per_s": round(len(st["csi_times"]) / 2.0, 1), "rssi": st.get("rssi"),
                    "presence": bool(v and v["presence"]), "motion": bool(v and v["motion"]),
                    "fall": bool(v and v["fall"]),
                    "presence_score": round(v["presence_score"], 3) if v else None,
                    "motion_energy": round(v["motion_energy"], 3) if v else None,
                    "persons": v["persons"] if v else None,
                    "breathing": round(v["breathing_bpm"], 1) if v and v["breathing_bpm"] > 0 else None,
                    "heart": round(v["heart_bpm"], 1) if v and v["heart_bpm"] > 0 else None,
                })
            live = [n for n in nodes if n["age"] < 5]
            sensing = [n for n in live if n["presence"] and n["node_id"] in pos]
            person = None
            if sensing:
                w = [max(1e-3, n["presence_score"] or 0) ** 2 for n in sensing]
                c = [sum(wi * pos[n["node_id"]][k] for wi, n in zip(w, sensing)) / sum(w) for k in range(3)]
                c[1] = self.scene["floor_y"]
                if self.person is not None:          # smooth the jumps a little
                    c = [0.7 * a + 0.3 * b for a, b in zip(self.person, c)]
                person = c
            self.person = person
            br = sorted(n["breathing"] for n in sensing if n["breathing"])
            hr = sorted(n["heart"] for n in sensing if n["heart"])
            vit = {
                "breathing": br[len(br) // 2] if br else None,
                "heart": hr[len(hr) // 2] if hr else None,
                "persons": max((n["persons"] or 0 for n in live), default=0),
                "fall": any(n["fall"] for n in live),
                "motion": any(n["motion"] for n in live),
            }
            if not self.history or now - self.history[-1][0] >= 1.0:
                self.history.append([now, vit["breathing"], vit["heart"], vit["persons"]])
                self.history = [h for h in self.history if now - h[0] <= HISTORY_S]
            hist = [[round(h[0] - now, 1), h[1], h[2], h[3]] for h in self.history]
        return {"t": now, "nodes": nodes, "person": person, "vitals": vit, "history": hist}


def udp_loop(live: Live, bind: str, port: int, forward: tuple[str, int] | None = None):
    fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if forward else None
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind((bind, port))
    except OSError as e:
        sys.exit(f"cannot listen on UDP {port}: {e} (stop esp_fleet.py monitor / the RuView server first)")
    while True:
        b, addr = s.recvfrom(4096)
        if fwd:
            fwd.sendto(b, forward)
        p = parse_packet(b)
        if p and p["type"] in ("csi", "vitals"):
            live.packet(p, addr[0])


def demo_loop(port: int, scene: dict):
    """Simulated nodes: a person walking a slow loop between the nodes, breathing ~14/min, heart ~68/min."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    nodes = scene["nodes"]
    cx = sum(n["position"][0] for n in nodes) / len(nodes)
    cz = sum(n["position"][2] for n in nodes) / len(nodes)
    r = 0.5 * max(math.hypot(n["position"][0] - cx, n["position"][2] - cz) for n in nodes)
    t0 = time.time()
    seq = 0
    while True:
        t = time.time() - t0
        px, pz = cx + r * math.cos(t / 9), cz + r * math.sin(t / 13)
        for n in nodes:
            d = math.hypot(n["position"][0] - px, n["position"][2] - pz)
            score = max(0.0, min(1.0, 1.4 / (0.6 + d)))
            br = 14 + 1.5 * math.sin(t / 20) + random.uniform(-0.4, 0.4)
            hr = 68 + 4 * math.sin(t / 30) + random.uniform(-1.5, 1.5)
            flags = 1 | (4 if math.sin(t / 5) > 0.3 else 0)
            pkt = struct.pack("<IBBHIbB", VITALS_MAGIC, n["node_id"], flags, int(br * 100), int(hr * 10000),
                              -55 - int(5 * d), 1) + b"\0\0" + struct.pack("<ffI", 0.3, score, int(t * 1000)) + b"\0" * 4
            s.sendto(pkt, ("127.0.0.1", port))
            for _ in range(6):
                seq += 1
                hdr = struct.pack("<IBBHIIbb", 0xC5110001, n["node_id"], 1, 64, 2432, seq, -55 - int(5 * d), -95)
                s.sendto(hdr + b"\0\0" + bytes(256), ("127.0.0.1", port))
        time.sleep(0.2)


# ------------------------------------------------------------------------------------ http
def make_handler(scene: dict, live: Live):
    files = {"/": ("index.html", "text/html; charset=utf-8"), "/index.html": ("index.html", "text/html; charset=utf-8"),
             "/three.min.js": ("three.min.js", "text/javascript"),
             "/OrbitControls.js": ("OrbitControls.js", "text/javascript")}
    scene_json = json.dumps(scene).encode()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, body: bytes, ctype: str):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in files:
                name, ctype = files[path]
                return self.send((WEB / name).read_bytes(), ctype)
            if path == "/api/scene":
                return self.send(scene_json, "application/json")
            if path == "/api/live":
                return self.send(json.dumps(live.snapshot()).encode(), "application/json")
            self.send_error(404)

    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", type=Path, help="capture session folder: shows its scan and its esp_nodes.json")
    ap.add_argument("--nodes", type=Path, help="esp_nodes.json or esp_plan.json with the node positions")
    ap.add_argument("--http", type=int, default=8080, help="web page port (default 8080)")
    ap.add_argument("--udp", type=int, default=5005, help="UDP port the nodes stream to (default 5005)")
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--max-points", type=int, default=300_000, help="point-cloud limit when the scan has no mesh")
    ap.add_argument("--forward", metavar="HOST:PORT",
                    help="also pass every packet on, e.g. 127.0.0.1:5006 for a RuView server started with --udp-bind 127.0.0.1:5006")
    ap.add_argument("--demo", action="store_true", help="simulate the nodes and a person (no boards needed)")
    a = ap.parse_args(argv)

    scene = load_scene(a.session, a.nodes, a.max_points)
    g = scene["geometry"]
    print("scan:", f"{g['kind']} with {g['count']} points from {scene['source']}" if g else "none (nodes only)")
    for n in scene["nodes"]:
        print(f"  node {n['node_id']}: {n['position']}" + ("" if n["placed"] else "  (not placed: stand-in spot)"))
    live = Live(scene)
    fwd = None
    if a.forward:
        h, _, p = a.forward.rpartition(":")
        fwd = (h or "127.0.0.1", int(p))
    threading.Thread(target=udp_loop, args=(live, a.bind, a.udp, fwd), daemon=True).start()
    if a.demo:
        threading.Thread(target=demo_loop, args=(a.udp, scene), daemon=True).start()
    srv = ThreadingHTTPServer((a.bind, a.http), make_handler(scene, live))
    print(f"open http://localhost:{a.http}   (listening for nodes on UDP {a.udp}; Ctrl-C to stop)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
