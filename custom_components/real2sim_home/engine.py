"""Presence fusion: where people are in the mapped rooms, their pose and their heart / breathing rate.

Pure Python (no Home Assistant imports): the integration collects a Snapshot from HA states, the RuView hub
and the stored room maps, and `fuse()` turns it into what the floor-plan card and the entities show.

Sources, strongest first, for each room:
  * mmWave radar targets (LD2450-style x / y sensors, placed in the room)  -> positions of people
  * RuView CSI node vitals (presence, person count, motion, fall, heart, breathing)
  * motion / occupancy / presence binary sensors in the room's area
  * people whose room is known (a "room" sensor such as Bermuda / ESPresense, a phone in that area)
Fitness bands and phones (companion app) add heart rate, activity, battery to named people.
Pose (standing / walking / sitting / lying / fallen) is inferred: fall flag, movement, and where the person is
relative to the room's furniture (bed, sofa, chair, toilet) from the room map.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

SEATS = ("sofa", "chair", "toilet", "bench")
BEDS = ("bed",)
VITALS_MAX_AGE = 15.0        # s: RuView vitals older than this are ignored
BAND_MAX_AGE = 15 * 60.0     # s: a band's heart rate older than this is not shown
FALL_HOLD = 120.0            # s: a fall flag keeps the person "fallen" this long
MOTION_HOLD = 30.0           # s: a motion sensor counts as presence this long after it turns off


@dataclass
class Room:
    id: str
    name: str
    map: dict[str, Any]
    area_id: str | None = None

    @property
    def objects(self) -> list[dict[str, Any]]:
        return self.map.get("objects") or []

    @property
    def size(self) -> tuple[float, float]:
        b = self.map.get("bounds") or [0, 0, 4, 4]
        return float(b[2]) - float(b[0]), float(b[3]) - float(b[1])

    def center(self) -> tuple[float, float]:
        w, h = self.size
        return w / 2, h / 2


@dataclass
class Sensor:
    """An HA entity that tells something about a room (kind = presence / motion / temperature / humidity /
    illuminance / co2 / sound / camera / microphone / heart_rate / breathing / light / other)."""
    entity_id: str
    room_id: str
    kind: str
    name: str = ""
    state: Any = None
    unit: str | None = None
    on: bool = False
    last_changed: float = 0.0
    pos: tuple[float, float] | None = None   # placed by the user (room coordinates); None = automatic


@dataclass
class RadarTarget:
    room_id: str
    x: float
    y: float
    speed: float = 0.0
    source: str = ""


@dataclass
class Node:
    """A RuView CSI node with its latest vitals (None = no fresh vitals)."""
    node_id: int
    room_id: str | None
    pos: tuple[float, float] | None
    presence: bool = False
    fall: bool = False
    motion: bool = False
    heart: float | None = None
    breathing: float | None = None
    n_persons: int = 0
    motion_energy: float = 0.0
    presence_score: float = 0.0
    rssi: int | None = None
    vitals_age: float | None = None
    last_fall_age: float | None = None
    csi_rate: float = 0.0


@dataclass
class Person:
    """A named person (HA person entity) with what their devices report."""
    id: str
    name: str
    picture: str | None = None
    home: bool = True
    room_id: str | None = None             # from a room sensor / phone area
    room_source: str | None = None
    heart: float | None = None
    heart_age: float | None = None
    heart_source: str | None = None
    breathing: float | None = None
    breathing_source: str | None = None
    activity: str | None = None            # phone: still / walking / running / in_vehicle / ...
    battery: float | None = None
    charging: bool | None = None
    phone: str | None = None
    steps: float | None = None
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class Snapshot:
    rooms: list[Room]
    sensors: list[Sensor]
    nodes: list[Node]
    radar: list[RadarTarget]
    persons: list[Person]
    now: float
    night: bool = False


# ------------------------------------------------------------------------------------------- geometry
def in_box(obj: dict[str, Any], x: float, y: float, margin: float = 0.15) -> bool:
    c = obj.get("center") or [0, 0, 0]
    s = obj.get("size") or [0, 0, 0]
    yaw = float(obj.get("yaw") or 0)
    dx, dy = x - float(c[0]), y - float(c[1])
    u = dx * math.cos(yaw) + dy * math.sin(yaw)
    v = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return abs(u) <= float(s[0]) / 2 + margin and abs(v) <= float(s[1]) / 2 + margin


def furniture_at(room: Room, x: float, y: float) -> dict[str, Any] | None:
    for o in room.objects:
        if o.get("category") in SEATS + BEDS and in_box(o, x, y):
            return o
    return None


def auto_marker_positions(room: Room, n: int) -> list[tuple[float, float]]:
    """Spots for n unplaced sensor markers: a row along the room's top edge, inside the room."""
    w, h = room.size
    if n <= 0:
        return []
    step = w / (n + 1)
    y = h - min(0.35, h * 0.12)
    return [(step * (i + 1), y) for i in range(n)]


# ------------------------------------------------------------------------------------------- fusion
def _room_sensors(snap: Snapshot, room_id: str) -> list[Sensor]:
    return [s for s in snap.sensors if s.room_id == room_id]


def _num(v: Any) -> float | None:
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def room_summary(snap: Snapshot, room: Room) -> dict[str, Any]:
    sens = _room_sensors(snap, room.id)
    nodes = [n for n in snap.nodes if n.room_id == room.id]
    fresh_nodes = [n for n in nodes if n.vitals_age is not None and n.vitals_age <= VITALS_MAX_AGE]
    radar = [t for t in snap.radar if t.room_id == room.id]
    reasons: list[str] = []
    for s in sens:
        if s.kind in ("presence", "occupancy") and s.on:
            reasons.append(s.entity_id)
        elif s.kind == "motion" and (s.on or snap.now - s.last_changed <= MOTION_HOLD):
            reasons.append(s.entity_id)
    for n in fresh_nodes:
        if n.presence:
            reasons.append(f"ruview:{n.node_id}")
    if radar:
        reasons.append("radar")
    located = [p for p in snap.persons if p.room_id == room.id and p.home]
    if located:
        reasons.append("person")
    count = max([len(radar), len(located)] + [n.n_persons for n in fresh_nodes if n.presence] + [1 if reasons else 0])

    def avg(kind: str) -> float | None:
        vals = [v for v in (_num(s.state) for s in sens if s.kind == kind) if v is not None]
        return round(sum(vals) / len(vals), 1) if vals else None

    return {
        "id": room.id, "name": room.name, "area_id": room.area_id,
        "occupied": bool(reasons), "count": count, "reasons": reasons,
        "temperature": avg("temperature"), "humidity": avg("humidity"), "illuminance": avg("illuminance"),
        "co2": avg("co2"), "sound": avg("sound"),
        "fall": any(n.fall or (n.last_fall_age is not None and n.last_fall_age <= FALL_HOLD) for n in fresh_nodes),
    }


def _pose_at(room: Room, x: float, y: float, moving: bool, fallen: bool) -> tuple[str, str | None]:
    if fallen:
        return "fallen", None
    if moving:
        return "walking", None
    f = furniture_at(room, x, y)
    if f is not None:
        return ("lying" if f.get("category") in BEDS else "sitting"), f.get("id")
    return "standing", None


def _resting_spot(room: Room, taken: set[str], prefer_bed: bool) -> dict[str, Any] | None:
    order = (BEDS + SEATS) if prefer_bed else SEATS
    for cat in order:
        for o in room.objects:
            if o.get("category") == cat and o.get("id") not in taken:
                return o
    return None


def fuse(snap: Snapshot) -> dict[str, Any]:
    rooms = {r.id: r for r in snap.rooms}
    summaries = {r.id: room_summary(snap, r) for r in snap.rooms}
    people: list[dict[str, Any]] = []

    for room in snap.rooms:
        rs = summaries[room.id]
        if not rs["occupied"]:
            continue
        nodes = [n for n in snap.nodes if n.room_id == room.id and n.vitals_age is not None and n.vitals_age <= VITALS_MAX_AGE]
        node = max(nodes, key=lambda n: n.presence_score, default=None)
        fallen = rs["fall"]
        moving_room = bool(node and (node.motion or node.motion_energy > 1.0))
        radar = sorted((t for t in snap.radar if t.room_id == room.id), key=lambda t: (t.x, t.y))
        named = [p for p in snap.persons if p.room_id == room.id and p.home]
        n_people = max(rs["count"], len(named))
        taken: set[str] = set()
        cx, cy = room.center()
        for i in range(n_people):
            p = named[i] if i < len(named) else None
            moving = (p is not None and p.activity in ("walking", "running", "on_bicycle")) or moving_room
            if i < len(radar):
                t = radar[i]
                x, y, conf, pos_src = t.x, t.y, 0.9, "radar"
                moving = moving or t.speed > 0.25
                pose, on = _pose_at(room, x, y, moving, fallen and i == 0)
            else:
                pose, on, pos_src, conf = "standing", None, "estimated", 0.4
                spot = None
                if fallen and i == 0:
                    pose = "fallen"
                elif not moving:
                    spot = _resting_spot(room, taken, prefer_bed=snap.night)
                if spot is not None:
                    taken.add(spot["id"])
                    x, y = float(spot["center"][0]), float(spot["center"][1])
                    pose, on = ("lying" if spot.get("category") in BEDS else "sitting"), spot["id"]
                elif node is not None and node.pos is not None:
                    # near the CSI node, towards the middle of the room
                    nx, ny = node.pos
                    d = math.hypot(cx - nx, cy - ny) or 1.0
                    k = min(1.0, 1.0 / d)
                    x, y = nx + (cx - nx) * k + 0.5 * i, ny + (cy - ny) * k
                    if moving:
                        pose = "walking"
                else:
                    ang = 2 * math.pi * i / max(1, n_people)
                    x, y = cx + (0.6 * math.cos(ang) if n_people > 1 else 0), cy + (0.6 * math.sin(ang) if n_people > 1 else 0)
                    if moving:
                        pose = "walking"
            w, h = room.size
            x, y = min(max(x, 0.1), w - 0.1), min(max(y, 0.1), h - 0.1)

            heart = heart_src = breathing = br_src = None
            if p is not None and p.heart is not None and (p.heart_age is None or p.heart_age <= BAND_MAX_AGE):
                heart, heart_src = p.heart, p.heart_source or "band"
            if p is not None and p.breathing is not None:
                breathing, br_src = p.breathing, p.breathing_source or "band"
            # RuView vitals belong to the person nearest the node; with one person in the room that is them
            if node is not None and (n_people == 1 or i == 0):
                if heart is None and node.heart is not None:
                    heart, heart_src = node.heart, f"ruview:{node.node_id}"
                if breathing is None and node.breathing is not None:
                    breathing, br_src = node.breathing, f"ruview:{node.node_id}"
            if heart is None:
                for s in _room_sensors(snap, room.id):
                    if s.kind == "heart_rate" and _num(s.state):
                        heart, heart_src = _num(s.state), s.entity_id
                        break
            if breathing is None:
                for s in _room_sensors(snap, room.id):
                    if s.kind == "breathing" and _num(s.state):
                        breathing, br_src = _num(s.state), s.entity_id
                        break
            people.append({
                "id": p.id if p else f"{room.id}_guest_{i + 1}",
                "name": p.name if p else "Someone",
                "known": p is not None, "picture": p.picture if p else None,
                "room_id": room.id, "x": round(x, 2), "y": round(y, 2),
                "pose": pose, "on": on, "position_source": pos_src, "confidence": conf,
                "heart_rate": round(heart) if heart else None, "heart_rate_source": heart_src,
                "breathing_rate": round(breathing, 1) if breathing else None, "breathing_rate_source": br_src,
                "activity": p.activity if p else None,
                "phone": ({"name": p.phone, "battery": p.battery, "charging": p.charging, "steps": p.steps}
                          if p and (p.phone or p.battery is not None) else None),
            })

    # named people away or whose room is unknown
    placed = {pp["id"] for pp in people}
    elsewhere = [{
        "id": p.id, "name": p.name, "picture": p.picture, "home": p.home, "room_id": p.room_id,
        "heart_rate": round(p.heart) if p.heart and (p.heart_age is None or p.heart_age <= BAND_MAX_AGE) else None,
        "activity": p.activity,
        "phone": {"name": p.phone, "battery": p.battery, "charging": p.charging} if p.phone or p.battery is not None else None,
    } for p in snap.persons if p.id not in placed]

    markers = []
    for room in snap.rooms:
        sens = [s for s in _room_sensors(snap, room.id) if s.kind not in ("heart_rate", "breathing")]
        auto = [s for s in sens if s.pos is None]
        spots = iter(auto_marker_positions(room, len(auto)))
        for s in sens:
            x, y = s.pos if s.pos is not None else next(spots)
            markers.append({"entity_id": s.entity_id, "room_id": room.id, "kind": s.kind, "name": s.name,
                            "state": s.state, "unit": s.unit, "on": s.on, "x": round(x, 2), "y": round(y, 2),
                            "placed": s.pos is not None})
        for n in snap.nodes:
            if n.room_id == room.id:
                x, y = n.pos if n.pos is not None else (room.center()[0], 0.25)
                markers.append({"entity_id": f"ruview_{n.node_id}", "room_id": room.id, "kind": "ruview",
                                "name": f"RuView node {n.node_id}", "state": "presence" if n.presence else "clear",
                                "on": n.presence, "x": round(x, 2), "y": round(y, 2), "placed": n.pos is not None,
                                "heart_rate": n.heart, "breathing_rate": n.breathing, "rssi": n.rssi,
                                "csi_rate": round(n.csi_rate, 1)})

    return {
        "rooms": list(summaries.values()),
        "people": people,
        "elsewhere": elsewhere,
        "markers": markers,
        "nodes": [{"node_id": n.node_id, "room_id": n.room_id, "presence": n.presence, "fall": n.fall,
                   "heart_rate": n.heart, "breathing_rate": n.breathing, "persons": n.n_persons,
                   "vitals_age": None if n.vitals_age is None else round(n.vitals_age, 1)} for n in snap.nodes],
    }


# ------------------------------------------------------------------------------------------- room maps
def validate_room_map(data: Any) -> dict[str, Any]:
    """Checks / normalises an uploaded room_map.json (HOME.md); raises ValueError."""
    if not isinstance(data, dict):
        raise ValueError("room map must be a JSON object")
    if data.get("format") != "r2s-roommap":
        raise ValueError("format must be 'r2s-roommap'")
    name = str(data.get("name") or "").strip() or "Room"
    floor = data.get("floor") or []
    if not isinstance(floor, list) or any(not isinstance(p, (list, tuple)) or len(p) < 2 for p in floor):
        raise ValueError("floor must be a list of [x, y] points")
    out = dict(data)
    out["name"] = name
    out["id"] = slugify(str(data.get("id") or name))
    for key in ("walls", "doors", "windows", "openings", "objects"):
        v = data.get(key) or []
        if not isinstance(v, list):
            raise ValueError(f"{key} must be a list")
        out[key] = v
    xs = [float(p[0]) for p in floor] + [float(c) for w in out["walls"] for c in (w["a"][0], w["b"][0])]
    ys = [float(p[1]) for p in floor] + [float(c) for w in out["walls"] for c in (w["a"][1], w["b"][1])]
    if not xs:
        xs, ys = [0.0, 4.0], [0.0, 4.0]
    if not data.get("bounds"):
        out["bounds"] = [min(xs), min(ys), max(xs), max(ys)]
    if not floor:
        out["floor"] = [[min(xs), min(ys)], [max(xs), min(ys)], [max(xs), max(ys)], [min(xs), max(ys)]]
    return out


def slugify(s: str) -> str:
    out = "".join(c if c.isascii() and c.isalnum() else "_" for c in s.lower())
    return "_".join(p for p in out.split("_") if p) or "room"


def auto_layout(rooms: list[Room], layout: dict[str, dict[str, float]], gap: float = 0.6) -> dict[str, dict[str, float]]:
    """Home plan position of each room: stored ones kept, the others placed in rows to the right / below."""
    out = {rid: dict(v) for rid, v in layout.items() if rid in {r.id for r in rooms}}
    x = y = row_h = 0.0
    max_w = max(12.0, math.sqrt(sum(r.size[0] * r.size[1] for r in rooms) or 1) * 2)
    if out:
        y = max(v["y"] + _rot_size(next(r for r in rooms if r.id == rid), v.get("rotation", 0))[1] for rid, v in out.items()) + gap
    for r in rooms:
        if r.id in out:
            continue
        w, h = r.size
        if x > 0 and x + w > max_w:
            x, y, row_h = 0.0, y + row_h + gap, 0.0
        out[r.id] = {"x": round(x, 2), "y": round(y, 2), "rotation": 0}
        x += w + gap
        row_h = max(row_h, h)
    return out


def _rot_size(room: Room, rotation: float) -> tuple[float, float]:
    w, h = room.size
    return (h, w) if int(round(rotation / 90)) % 2 else (w, h)
