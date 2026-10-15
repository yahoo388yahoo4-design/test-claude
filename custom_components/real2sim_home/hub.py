"""Real2Sim Home hub: room maps, RuView nodes, HA sensors and people -> engine.fuse() -> entities + card."""
from __future__ import annotations

import logging
import re
import time
from datetime import datetime, timedelta
from typing import Any, Callable

from homeassistant.const import EVENT_STATE_CHANGED, STATE_HOME, STATE_ON, STATE_UNAVAILABLE, STATE_UNKNOWN
from homeassistant.core import Event, HomeAssistant, callback
from homeassistant.helpers import area_registry as ar
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import entity_registry as er
from homeassistant.helpers.dispatcher import async_dispatcher_send
from homeassistant.helpers.event import async_track_time_interval
from homeassistant.helpers.storage import Store
from homeassistant.util import dt as dt_util

from . import engine
from .const import (
    CONF_NIGHT_END, CONF_NIGHT_START, CONF_PERSONS, DOMAIN, SIGNAL_NEW_NODE, SIGNAL_NEW_PERSON, SIGNAL_NEW_ROOM,
    SIGNAL_UPDATE, STORAGE_KEY, STORAGE_VERSION,
)
from .ruview import RuViewHub

_LOGGER = logging.getLogger(__name__)

_RADAR_X = re.compile(r"^sensor\.(.+)_target_(\d+)_x$")
_SENSOR_KINDS = {
    "temperature": "temperature", "humidity": "humidity", "illuminance": "illuminance",
    "carbon_dioxide": "co2", "sound_pressure": "sound",
}
_BINARY_KINDS = {"motion": "motion", "occupancy": "presence", "presence": "presence"}
_OWN_PLATFORM = DOMAIN


class Hub:
    def __init__(self, hass: HomeAssistant, options: dict[str, Any]):
        self.hass = hass
        self.options = options
        self.store: Store = Store(hass, STORAGE_VERSION, STORAGE_KEY)
        self.rooms: dict[str, dict[str, Any]] = {}            # id -> room map (+ "area_id")
        self.layout: dict[str, dict[str, float]] = {}         # room id -> {x, y, rotation}
        self.placements: dict[str, dict[str, Any]] = {}       # item -> {room_id, x, y, rotation}
        self.ruview = RuViewHub(on_update=self._on_node)
        self.state: dict[str, Any] = {"rooms": [], "people": [], "elsewhere": [], "markers": [], "nodes": []}
        self.known_people: set[str] = set()
        self._listeners: list[Callable[[], None]] = []
        self._subscribers: list[Callable[[str, dict[str, Any]], None]] = []
        self._pending: Any = None
        self._seen_nodes: set[int] = set()

    # ------------------------------------------------------------------ lifecycle
    async def async_load(self) -> None:
        data = await self.store.async_load() or {}
        self.rooms = data.get("rooms", {})
        self.layout = data.get("layout", {})
        self.placements = data.get("placements", {})

    async def async_start(self) -> None:
        self._listeners.append(self.hass.bus.async_listen(EVENT_STATE_CHANGED, self._on_state))
        self._listeners.append(async_track_time_interval(self.hass, self._tick, timedelta(seconds=5)))
        self.refresh()

    async def async_stop(self) -> None:
        for unsub in self._listeners:
            unsub()
        self._listeners.clear()
        if self._pending:
            self._pending.cancel()

    def _save(self) -> None:
        self.store.async_delay_save(lambda: {"rooms": self.rooms, "layout": self.layout, "placements": self.placements}, 1)

    # ------------------------------------------------------------------ room maps
    async def async_add_room_map(self, data: Any) -> dict[str, Any]:
        room = engine.validate_room_map(data)
        rid = room["id"]
        # link to (or create) the Home Assistant area with the room's name, so devices assigned there show up
        areas = ar.async_get(self.hass)
        area = None
        if room.get("area_id"):
            area = areas.async_get_area(room["area_id"])
        if area is None:
            area = areas.async_get_or_create(str(room.get("area") or room["name"]))
        room["area_id"] = area.id
        room["received"] = time.time()
        new = rid not in self.rooms
        self.rooms[rid] = room
        self._save()
        if new:
            async_dispatcher_send(self.hass, SIGNAL_NEW_ROOM, rid)
        self.refresh(geometry=True)
        return room

    async def async_remove_room(self, rid: str) -> None:
        self.rooms.pop(rid, None)
        self.layout.pop(rid, None)
        self.placements = {k: v for k, v in self.placements.items() if v.get("room_id") != rid}
        self._save()
        self.refresh(geometry=True)

    def set_layout(self, rid: str, x: float, y: float, rotation: float = 0) -> None:
        self.layout[rid] = {"x": x, "y": y, "rotation": rotation}
        self._save()
        self.refresh(geometry=True)

    def place(self, item: str, rid: str, x: float, y: float, rotation: float = 0) -> None:
        self.placements[item] = {"room_id": rid, "x": x, "y": y, "rotation": rotation}
        self._save()
        self.refresh(geometry=True)

    def unplace(self, item: str) -> None:
        self.placements.pop(item, None)
        self._save()
        self.refresh(geometry=True)

    def geometry(self) -> dict[str, Any]:
        rooms = self._rooms()
        layout = engine.auto_layout(rooms, self.layout)
        return {
            "rooms": [{"id": r.id, "name": r.name, "area_id": r.area_id, "map": r.map, "layout": layout[r.id]} for r in rooms],
            "placements": self.placements,
        }

    def _rooms(self) -> list[engine.Room]:
        return [engine.Room(rid, m["name"], m, m.get("area_id")) for rid, m in sorted(self.rooms.items())]

    # ------------------------------------------------------------------ events
    @callback
    def _on_node(self, node_id: int, new: bool) -> None:
        if node_id not in self._seen_nodes:
            self._seen_nodes.add(node_id)
            async_dispatcher_send(self.hass, SIGNAL_NEW_NODE, node_id)
        self._schedule()

    @callback
    def _on_state(self, event: Event) -> None:
        eid = event.data.get("entity_id", "")
        if eid.startswith(("sensor.", "binary_sensor.", "person.", "device_tracker.", "camera.", "assist_satellite.", "light.")):
            if not er.async_get(self.hass).async_get(eid) or er.async_get(self.hass).async_get(eid).platform != _OWN_PLATFORM:
                self._schedule()

    @callback
    def _tick(self, _now: datetime) -> None:
        self.refresh()

    @callback
    def _schedule(self) -> None:
        if self._pending is None:
            self._pending = self.hass.loop.call_later(0.5, self._run_pending)

    @callback
    def _run_pending(self) -> None:
        self._pending = None
        self.refresh()

    def subscribe(self, cb: Callable[[str, dict[str, Any]], None]) -> Callable[[], None]:
        self._subscribers.append(cb)

        def unsub() -> None:
            if cb in self._subscribers:
                self._subscribers.remove(cb)
        return unsub

    # ------------------------------------------------------------------ fusion
    @callback
    def refresh(self, geometry: bool = False) -> None:
        try:
            snap = self.snapshot()
            self.state = engine.fuse(snap)
        except Exception:  # noqa: BLE001
            _LOGGER.exception("Real2Sim Home: fusion failed")
            return
        for p in self.state["people"] + self.state["elsewhere"]:
            if p.get("known") is not False and p["id"].startswith("person.") and p["id"] not in self.known_people:
                self.known_people.add(p["id"])
                async_dispatcher_send(self.hass, SIGNAL_NEW_PERSON, p["id"])
        async_dispatcher_send(self.hass, SIGNAL_UPDATE)
        if geometry:
            g = self.geometry()
            for cb in list(self._subscribers):
                cb("geometry", g)
        for cb in list(self._subscribers):
            cb("state", self.state)

    def _night(self) -> bool:
        h = dt_util.now().hour
        start, end = int(self.options.get(CONF_NIGHT_START, 22)), int(self.options.get(CONF_NIGHT_END, 7))
        return (h >= start or h < end) if start > end else start <= h < end

    def snapshot(self) -> engine.Snapshot:
        hass = self.hass
        now = time.time()
        rooms = self._rooms()
        area_room = {r.area_id: r.id for r in rooms if r.area_id}
        name_room = {r.name.lower(): r.id for r in rooms}
        ents = er.async_get(hass)
        devs = dr.async_get(hass)
        areas = ar.async_get(hass)
        for a in areas.async_list_areas():
            if a.id in area_room:
                name_room.setdefault(a.name.lower(), area_room[a.id])

        def room_of(entry: er.RegistryEntry | None, eid: str) -> str | None:
            pl = self.placements.get(eid)
            if pl and pl.get("room_id") in self.rooms:
                return pl["room_id"]
            if entry is None:
                return None
            area = entry.area_id
            if area is None and entry.device_id:
                d = devs.async_get(entry.device_id)
                area = d.area_id if d else None
            return area_room.get(area)

        sensors: list[engine.Sensor] = []
        radar: list[engine.RadarTarget] = []
        radar_seen: set[str] = set()
        for st in hass.states.async_all(("sensor", "binary_sensor", "camera", "assist_satellite", "light")):
            eid = st.entity_id
            entry = ents.async_get(eid)
            if entry is not None and entry.platform == _OWN_PLATFORM:
                continue
            rid = room_of(entry, eid)
            if rid is None:
                continue
            dom = st.domain
            dc = st.attributes.get("device_class")
            name = st.attributes.get("friendly_name") or eid
            low = (eid + " " + str(name)).lower()
            kind = None
            if dom == "binary_sensor":
                kind = _BINARY_KINDS.get(dc)
            elif dom == "sensor":
                m = _RADAR_X.match(eid)
                if m:
                    self._radar(st, m, rid, radar, radar_seen)
                    continue
                if re.search(r"_target_\d+_(y|speed|resolution|distance|angle)$", eid):
                    continue
                if "heart" in low and "rate" in low or low.endswith("_bpm"):
                    kind = "heart_rate"
                elif "breath" in low or "respirat" in low:
                    kind = "breathing"
                else:
                    kind = _SENSOR_KINDS.get(dc)
            elif dom == "camera":
                kind = "camera"
            elif dom == "assist_satellite":
                kind = "microphone"
            elif dom == "light":
                kind = "light"
            if kind is None or st.state in (STATE_UNAVAILABLE, STATE_UNKNOWN) and kind not in ("camera", "microphone"):
                continue
            pl = self.placements.get(eid)
            sensors.append(engine.Sensor(
                entity_id=eid, room_id=rid, kind=kind, name=name, state=st.state,
                unit=st.attributes.get("unit_of_measurement"), on=st.state == STATE_ON or (kind == "microphone" and st.state not in ("idle", STATE_UNAVAILABLE)),
                last_changed=st.last_changed.timestamp(), pos=(pl["x"], pl["y"]) if pl else None))

        nodes: list[engine.Node] = []
        for nid, ns in sorted(self.ruview.nodes.items()):
            key = f"ruview_{nid}"
            pl = self.placements.get(key)
            rid = pl["room_id"] if pl and pl.get("room_id") in self.rooms else None
            if rid is None:
                d = devs.async_get_device(identifiers={(DOMAIN, key)})
                rid = area_room.get(d.area_id) if d else None
            if rid is None and len(rooms) == 1:
                rid = rooms[0].id
            v = ns.vitals
            fresh = ns.fresh(now, engine.VITALS_MAX_AGE)
            nodes.append(engine.Node(
                node_id=nid, room_id=rid, pos=(pl["x"], pl["y"]) if pl else None,
                presence=bool(v and fresh and v.presence), fall=bool(v and fresh and v.fall), motion=bool(v and fresh and v.motion),
                heart=v.heart_bpm if v and fresh else None, breathing=v.breathing_bpm if v and fresh else None,
                n_persons=v.n_persons if v and fresh else 0, motion_energy=v.motion_energy if v and fresh else 0.0,
                presence_score=v.presence_score if v and fresh else 0.0, rssi=v.rssi if v else ns.csi_rssi,
                vitals_age=(now - ns.vitals_at) if v else None,
                last_fall_age=(now - ns.last_fall_at) if ns.last_fall_at else None, csi_rate=ns.csi_rate))

        persons = [self._person(st, name_room, ents, devs, now) for st in hass.states.async_all("person")]
        return engine.Snapshot(rooms, sensors, nodes, radar, persons, now, self._night())

    def _radar(self, st, m, rid: str, out: list, seen: set) -> None:
        prefix, n = m.group(1), m.group(2)
        ys = self.hass.states.get(f"sensor.{prefix}_target_{n}_y")
        x, y = engine._num(st.state), engine._num(ys.state) if ys else None
        if x is None or y is None or (x == 0 and y == 0):
            return
        unit = (st.attributes.get("unit_of_measurement") or "mm").lower()
        k = 0.001 if unit == "mm" else 0.01 if unit == "cm" else 1.0
        sp = self.hass.states.get(f"sensor.{prefix}_target_{n}_speed")
        speed = abs(engine._num(sp.state) or 0) * (0.01 if sp and (sp.attributes.get("unit_of_measurement") or "").startswith("cm") else 1) if sp else 0
        room = self.rooms[rid]
        pl = self.placements.get(f"radar:{prefix}") or self.placements.get(st.entity_id)
        b = room.get("bounds") or [0, 0, 4, 4]
        px, py, rot = (pl["x"], pl["y"], pl.get("rotation", 0)) if pl else ((b[2] - b[0]) / 2, 0.05, 0)
        import math
        a = math.radians(rot)
        lx, fy = x * k, y * k          # lateral, forward (radar faces +y of the room at rotation 0)
        out.append(engine.RadarTarget(rid, px + lx * math.cos(a) - fy * math.sin(a), py + lx * math.sin(a) + fy * math.cos(a), speed, prefix))

    def _person(self, st, name_room, ents, devs, now: float) -> engine.Person:
        pid = st.entity_id
        opts = (self.options.get(CONF_PERSONS) or {}).get(pid, {})
        p = engine.Person(id=pid, name=st.attributes.get("friendly_name") or pid, picture=st.attributes.get("entity_picture"),
                          home=st.state == STATE_HOME or st.state not in ("not_home", STATE_UNKNOWN, STATE_UNAVAILABLE))
        # entities of the person's phones / watches (device trackers' devices), found automatically
        auto: dict[str, str] = {}
        for tr in st.attributes.get("device_trackers") or []:
            e = ents.async_get(tr)
            if not e or not e.device_id:
                continue
            d = devs.async_get(e.device_id)
            if d and not p.phone:
                p.phone = d.name_by_user or d.name
            for de in er.async_entries_for_device(ents, e.device_id):
                eid = de.entity_id
                low = eid.lower()
                if eid.startswith("sensor."):
                    if (de.original_device_class or de.device_class) == "battery" or low.endswith("_battery_level"):
                        auto.setdefault("battery", eid)
                    elif low.endswith(("_battery_state", "_charger_type", "_is_charging")):
                        auto.setdefault("charging", eid)
                    elif low.endswith(("_detected_activity", "_activity")):
                        auto.setdefault("activity", eid)
                    elif low.endswith(("_area", "_room", "_ble_room")):
                        auto.setdefault("room", eid)
                    elif "heart_rate" in low or low.endswith("_heart"):
                        auto.setdefault("heart_rate", eid)
                    elif "respirat" in low or "breath" in low:
                        auto.setdefault("breathing", eid)
                    elif low.endswith("_steps"):
                        auto.setdefault("steps", eid)
        src = {**auto, **{k: v for k, v in opts.items() if v}}

        def val(key: str):
            eid = src.get(key)
            s = self.hass.states.get(eid) if eid else None
            if s is None or s.state in (STATE_UNAVAILABLE, STATE_UNKNOWN):
                return None, None
            return s, eid

        s, eid = val("room")
        if s is not None:
            rid = name_room.get(str(s.state).lower()) or name_room.get(str(s.state).replace("_", " ").lower())
            if rid:
                p.room_id, p.room_source = rid, eid
        s, eid = val("heart_rate")
        if s is not None and engine._num(s.state):
            p.heart, p.heart_source, p.heart_age = engine._num(s.state), eid, now - s.last_updated.timestamp()
        s, eid = val("breathing")
        if s is not None and engine._num(s.state):
            p.breathing, p.breathing_source = engine._num(s.state), eid
        s, _ = val("battery")
        if s is not None:
            p.battery = engine._num(s.state)
        s, _ = val("charging")
        if s is not None:
            p.charging = str(s.state).lower() in ("charging", "full", "on", "true", "ac", "usb", "wireless")
        s, _ = val("activity")
        if s is not None:
            p.activity = str(s.state).lower().replace(" ", "_")
        s, _ = val("steps")
        if s is not None:
            p.steps = engine._num(s.state)
        return p
