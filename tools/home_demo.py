#!/usr/bin/env python3
"""Try the Home Assistant side without phones or ESP32 boards: sends demo room maps, simulated RuView CSI
node packets and demo sensors / people to a Home Assistant running the Real2Sim Home integration.

    python3 tools/home_demo.py --ha http://homeassistant.local:8123 --token <long-lived token>
    python3 tools/home_demo.py --ha ... --token ... --room-map session/roommap/room_map.json   # one real scan
    python3 tools/home_demo.py --ha ... --token ... --ruview-only --seconds 600                # just the node

Standard library only. Demo sensors are plain states (POST /api/states) placed in the rooms with the
real2sim_home.place service; real devices are found through their HA area instead (HOME.md).
"""
from __future__ import annotations

import argparse
import json
import math
import pathlib
import random
import socket
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


def room(name, w, h, objects, doors=(), windows=()):
    walls = [((0, 0), (w, 0)), ((w, 0), (w, h)), ((w, h), (0, h)), ((0, h), (0, 0))]
    counts = {}
    objs = []
    for cat, cx, cy, sw, sd, sh, yaw in objects:
        counts[cat] = counts.get(cat, 0) + 1
        n = counts[cat]
        objs.append({"id": f"{cat}_{n}", "category": cat, "label": cat.replace("_", " ").capitalize() + (f" {n}" if n > 1 else ""),
                     "confidence": 0.9, "center": [cx, cy, sh / 2], "size": [sw, sd, sh], "yaw": yaw})
    return {"format": "r2s-roommap", "version": 1, "name": name, "units": "m", "bounds": [0, 0, w, h], "height": 2.5,
            "source": {"platform": "demo", "session": "home_demo", "device": "tools/home_demo.py", "method": "synthetic"},
            "floor": [[0, 0], [w, 0], [w, h], [0, h]],
            "walls": [{"a": list(a), "b": list(b), "height": 2.5} for a, b in walls],
            "doors": [{"a": list(a), "b": list(b), "height": 2.0} for a, b in doors],
            "windows": [{"a": list(a), "b": list(b), "height": 1.2} for a, b in windows],
            "openings": [], "objects": objs}


R = math.pi / 2
DEMO_ROOMS = [
    room("Living room", 5.2, 4.2, [("sofa", 2.6, 0.55, 2.3, 0.9, 0.8, 0), ("tv", 2.6, 4.1, 1.3, 0.1, 0.75, 0),
                                   ("table", 2.6, 2.0, 1.1, 0.6, 0.45, 0), ("chair", 4.6, 2.0, 0.55, 0.55, 0.9, R),
                                   ("plant", 0.4, 3.8, 0.4, 0.4, 1.0, 0)],
         doors=[((5.2, 1.0), (5.2, 1.9))], windows=[((0, 1.4), (0, 2.8))]),
    room("Bedroom", 4.0, 3.6, [("bed", 2.0, 2.4, 1.6, 2.1, 0.55, 0), ("chair", 0.5, 0.5, 0.5, 0.5, 0.9, 0),
                               ("storage", 3.6, 0.4, 0.8, 0.6, 2.0, 0)], doors=[((0, 0.3), (0, 1.2))], windows=[((1.2, 3.6), (2.8, 3.6))]),
    room("Kitchen", 3.4, 3.0, [("refrigerator", 0.4, 2.65, 0.7, 0.7, 1.9, 0), ("oven", 1.3, 2.7, 0.6, 0.6, 0.9, 0),
                               ("sink", 2.2, 2.7, 0.8, 0.6, 0.9, 0), ("table", 1.7, 1.0, 1.2, 0.8, 0.75, 0),
                               ("chair", 1.2, 0.4, 0.45, 0.45, 0.9, 0), ("chair", 2.2, 0.4, 0.45, 0.45, 0.9, 0)],
         doors=[((3.4, 0.4), (3.4, 1.3))]),
    room("Bathroom", 2.4, 2.2, [("toilet", 0.4, 1.8, 0.4, 0.7, 0.8, -R), ("sink", 1.4, 1.95, 0.6, 0.45, 0.9, 0),
                                ("bathtub", 1.2, 0.4, 1.7, 0.75, 0.6, 0)], doors=[((2.4, 0.9), (2.4, 1.7))]),
]
LAYOUT = {"living_room": (0, 3.8, 0), "bedroom": (5.8, 4.4, 0), "kitchen": (0, 0, 0), "bathroom": (3.8, 0.8, 0)}


class HA:
    def __init__(self, base, token):
        self.base = base.rstrip("/")
        self.token = token

    def call(self, method, path, body=None):
        req = urllib.request.Request(self.base + path, method=method, data=None if body is None else json.dumps(body).encode(),
                                     headers={"Authorization": f"Bearer {self.token}", "Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=20) as r:
                txt = r.read().decode()
                return json.loads(txt) if txt else None
        except urllib.error.HTTPError as e:
            raise SystemExit(f"{method} {path}: HTTP {e.code} {e.read().decode()[:300]}")

    def service(self, domain, name, data):
        return self.call("POST", f"/api/services/{domain}/{name}", data)

    def state(self, eid, state, attrs=None):
        return self.call("POST", f"/api/states/{eid}", {"state": str(state), "attributes": attrs or {}})


def configure_people(ha, ids):
    """Options flow of the integration: which sensors give each person's room / heart rate / battery / activity.
    Only for persons that exist (add them under Settings > People, or person: in configuration.yaml)."""
    entries = ha.call("GET", "/api/config/config_entries/entry?domain=real2sim_home")
    if not entries:
        print("Real2Sim Home is not set up: add it under Settings > Devices & services first")
        return
    eid = entries[0]["entry_id"]
    f = ha.call("POST", "/api/config/config_entries/options/flow", {"handler": eid})
    base = {k: v for k, v in {"udp_port": 5005, "mqtt_topic": "real2sim/room_map", "night_start": 22, "night_end": 7}.items()}
    f = ha.call("POST", f"/api/config/config_entries/options/flow/{f['flow_id']}", base)
    if f.get("step_id") != "persons":
        return
    keys = {d["name"] for d in f.get("data_schema", [])}
    data = {}
    for pid in ids:
        pe = f"person.{pid}"
        for fld, ent in (("room", f"sensor.{pid}_phone_area"), ("heart_rate", f"sensor.{pid}_watch_heart_rate"),
                         ("battery", f"sensor.{pid}_phone_battery_level"), ("activity", f"sensor.{pid}_phone_detected_activity")):
            if f"{pe}|{fld}" in keys:
                data[f"{pe}|{fld}"] = ent
    ha.call("POST", f"/api/config/config_entries/options/flow/{f['flow_id']}", data)
    print("people configured:", ", ".join(sorted({k.split('|')[0] for k in data})) or "none (no matching person entities)")


def vitals(node, *, presence=True, fall=False, motion=False, br=14.0, hr=68.0, rssi=-48, n=1, energy=0.1, score=0.9, ts=0):
    flags = (1 if presence else 0) | (2 if fall else 0) | (4 if motion else 0)
    return struct.pack("<IBBHIbBHffII", 0xC5110002, node, flags, int(br * 100), int(hr * 10000), rssi, n, 0, energy, score, ts, 0)


def csi(node, seq, rssi=-50, sub=64):
    return struct.pack("<IBBHIIbbBB", 0xC5110001, node, 1, sub, 2437, seq, rssi, -95, 0, 0) + bytes(2 * sub)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ha", required=True, help="Home Assistant URL, e.g. http://homeassistant.local:8123")
    ap.add_argument("--token", required=True, help="long-lived access token (HA profile > Security)")
    ap.add_argument("--udp-host", help="where RuView packets go (default: the HA host)")
    ap.add_argument("--udp-port", type=int, default=5005)
    ap.add_argument("--seconds", type=float, default=120, help="how long to simulate (0 = only upload)")
    ap.add_argument("--room-map", action="append", default=[], help="upload this room_map.json instead of the demo rooms")
    ap.add_argument("--ruview-only", action="store_true", help="only simulate a RuView node (node 1)")
    ap.add_argument("--no-options", action="store_true", help="do not change the integration's people options")
    a = ap.parse_args()
    ha = HA(a.ha, a.token)
    host = a.udp_host or urllib.parse.urlparse(a.ha).hostname
    if not a.ruview_only:
        maps = [json.loads(pathlib.Path(p).read_text()) for p in a.room_map] or DEMO_ROOMS
        for m in maps:
            ha.service("real2sim_home", "upload_room_map", {"room_map": m})
            print("room", m["name"], len(m.get("objects", [])), "objects")
        if not a.room_map:
            for rid, (x, y, rot) in LAYOUT.items():
                ha.service("real2sim_home", "set_room_layout", {"room_id": rid, "x": x, "y": y, "rotation": rot})
            demo = [
                ("binary_sensor.demo_living_motion", "on", {"device_class": "motion", "friendly_name": "Living room motion"}, "living_room", 5.0, 4.0),
                ("sensor.demo_living_temperature", "21.6", {"device_class": "temperature", "unit_of_measurement": "°C", "friendly_name": "Living room temperature"}, "living_room", 0.3, 0.6),
                ("sensor.demo_living_humidity", "48", {"device_class": "humidity", "unit_of_measurement": "%", "friendly_name": "Living room humidity"}, "living_room", 0.75, 0.6),
                ("camera.demo_living", "idle", {"friendly_name": "Living room camera"}, "living_room", 4.4, 3.95),
                ("sensor.demo_living_sound", "41", {"device_class": "sound_pressure", "unit_of_measurement": "dB", "friendly_name": "Living room microphone"}, "living_room", 2.6, 3.7),
                ("binary_sensor.demo_bedroom_presence", "on", {"device_class": "occupancy", "friendly_name": "Bedroom mmWave presence"}, "bedroom", 3.8, 3.4),
                ("sensor.demo_bedroom_temperature", "19.8", {"device_class": "temperature", "unit_of_measurement": "°C", "friendly_name": "Bedroom temperature"}, "bedroom", 3.7, 1.4),
                ("binary_sensor.demo_kitchen_motion", "off", {"device_class": "motion", "friendly_name": "Kitchen motion"}, "kitchen", 3.2, 2.8),
                ("sensor.demo_kitchen_temperature", "22.4", {"device_class": "temperature", "unit_of_measurement": "°C", "friendly_name": "Kitchen temperature"}, "kitchen", 3.1, 2.2),
                ("light.demo_kitchen", "on", {"friendly_name": "Kitchen light"}, "kitchen", 1.7, 1.6),
                ("binary_sensor.demo_bathroom_motion", "off", {"device_class": "motion", "friendly_name": "Bathroom motion"}, "bathroom", 2.2, 2.0),
                ("sensor.demo_bathroom_humidity", "63", {"device_class": "humidity", "unit_of_measurement": "%", "friendly_name": "Bathroom humidity"}, "bathroom", 0.25, 1.1),
                # LD2450-style radar targets (mm, radar at the living room's bottom wall facing up the map)
                ("sensor.demo_ld2450_target_1_x", "-600", {"unit_of_measurement": "mm"}, None, 0, 0),
                ("sensor.demo_ld2450_target_1_y", "2900", {"unit_of_measurement": "mm"}, None, 0, 0),
                ("sensor.demo_ld2450_target_1_speed", "0", {"unit_of_measurement": "cm/s"}, None, 0, 0),
            ]
            for eid, st, attrs, rid, x, y in demo:
                ha.state(eid, st, attrs)
                if rid:
                    ha.service("real2sim_home", "place", {"item": eid, "room_id": rid, "x": x, "y": y})
            ha.service("real2sim_home", "place", {"item": "sensor.demo_ld2450_target_1_x", "room_id": "living_room", "x": 2.6, "y": 0.05})
            ha.service("real2sim_home", "place", {"item": "radar:demo_ld2450", "room_id": "living_room", "x": 2.6, "y": 0.05})
            # people: phones (companion app) with a room sensor (Bermuda / ESPresense style) and a watch
            people = [
                ("ana", "Living room", "72", "81", "still"), ("bo", "Kitchen", "88", "34", "walking"),
            ]
            for pid, rname, hr, batt, act in people:
                ha.state(f"device_tracker.{pid}_phone", "home", {"source_type": "router"})
                ha.state(f"sensor.{pid}_phone_area", rname, {"friendly_name": f"{pid} phone area"})
                ha.state(f"sensor.{pid}_watch_heart_rate", hr, {"unit_of_measurement": "bpm"})
                ha.state(f"sensor.{pid}_phone_battery_level", batt, {"unit_of_measurement": "%", "device_class": "battery"})
                ha.state(f"sensor.{pid}_phone_detected_activity", act, {})
            if not a.no_options:
                configure_people(ha, [p[0] for p in people])
                time.sleep(3)      # the integration reloads with the new options
            ha.service("real2sim_home", "place", {"item": "ruview_1", "room_id": "bedroom", "x": 0.2, "y": 3.4})
            ha.service("real2sim_home", "place", {"item": "ruview_2", "room_id": "living_room", "x": 5.0, "y": 0.2})
            print("demo sensors placed; RuView nodes 1 (bedroom) and 2 (living room)")
    if a.seconds <= 0:
        return
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0 = time.time()
    seq = 0
    last_vitals = 0.0
    print(f"simulating RuView nodes -> udp://{host}:{a.udp_port} for {a.seconds:.0f} s (Ctrl+C stops)")
    try:
        while time.time() - t0 < a.seconds:
            now = time.time() - t0
            for node in ((1,) if a.ruview_only else (1, 2)):
                sock.sendto(csi(node, seq), (host, a.udp_port))
            seq += 1
            if now - last_vitals >= 1.0:
                last_vitals = now
                # node 1: someone asleep in the bedroom; node 2: someone on the sofa, getting up now and then
                sock.sendto(vitals(1, br=12.5 + 0.6 * math.sin(now / 20), hr=58 + 3 * math.sin(now / 30), energy=0.05, ts=int(now * 1000)), (host, a.udp_port))
                if not a.ruview_only:
                    walking = int(now / 25) % 4 == 3
                    sock.sendto(vitals(2, br=16 + random.uniform(-0.5, 0.5), hr=(84 if walking else 72) + random.uniform(-2, 2),
                                       motion=walking, energy=2.0 if walking else 0.2, n=1, ts=int(now * 1000)), (host, a.udp_port))
                    if int(now) % 5 == 0:
                        x = -600 + 900 * math.sin(now / 15) if walking else -100
                        ha.state("sensor.demo_ld2450_target_1_x", round(x), {"unit_of_measurement": "mm"})
                        ha.state("sensor.demo_ld2450_target_1_y", 2900 if walking else 600, {"unit_of_measurement": "mm"})
                        ha.state("sensor.demo_ld2450_target_1_speed", 40 if walking else 0, {"unit_of_measurement": "cm/s"})
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
