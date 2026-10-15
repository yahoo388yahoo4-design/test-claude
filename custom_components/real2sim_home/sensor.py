"""Room people count, RuView node vitals, and per-person room / pose / heart / breathing."""
from __future__ import annotations

from homeassistant.components.sensor import SensorDeviceClass, SensorEntity, SensorStateClass
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import SIGNAL_STRENGTH_DECIBELS_MILLIWATT, EntityCategory
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import DOMAIN, SIGNAL_NEW_NODE, SIGNAL_NEW_PERSON, SIGNAL_NEW_ROOM
from .entity import R2SEntity, node_device, person_device, room_device
from .hub import Hub

BPM = "bpm"
BRPM = "breaths/min"


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry, add: AddEntitiesCallback) -> None:
    hub: Hub = entry.runtime_data

    @callback
    def new_room(rid: str) -> None:
        add([RoomCount(hub, rid)])

    @callback
    def new_node(nid: int) -> None:
        add([NodeValue(hub, nid, *spec) for spec in NODE_VALUES])

    @callback
    def new_person(pid: str) -> None:
        st = hass.states.get(pid)
        name = st.attributes.get("friendly_name", pid) if st else pid
        add([PersonRoom(hub, pid, name), PersonVital(hub, pid, name, "heart_rate"), PersonVital(hub, pid, name, "breathing_rate")])

    for rid in list(hub.rooms):
        new_room(rid)
    for nid in list(hub.ruview.nodes):
        new_node(nid)
    for pid in list(hub.known_people):
        new_person(pid)
    entry.async_on_unload(async_dispatcher_connect(hass, SIGNAL_NEW_ROOM, new_room))
    entry.async_on_unload(async_dispatcher_connect(hass, SIGNAL_NEW_NODE, new_node))
    entry.async_on_unload(async_dispatcher_connect(hass, SIGNAL_NEW_PERSON, new_person))


class RoomCount(R2SEntity, SensorEntity):
    _attr_state_class = SensorStateClass.MEASUREMENT
    _attr_icon = "mdi:account-group"

    def __init__(self, hub: Hub, rid: str):
        super().__init__(hub)
        self.rid = rid
        self._attr_unique_id = f"{DOMAIN}_room_{rid}_people"
        self._attr_device_info = room_device(hub, rid)
        self._attr_name = "People"

    @property
    def available(self) -> bool:
        return self.rid in self.hub.rooms

    @property
    def native_value(self):
        r = next((r for r in self.hub.state["rooms"] if r["id"] == self.rid), None)
        return r["count"] if r else None

    @property
    def extra_state_attributes(self):
        return {"people": [{k: p[k] for k in ("name", "pose", "x", "y", "heart_rate", "breathing_rate")}
                           for p in self.hub.state["people"] if p["room_id"] == self.rid],
                "objects": [o.get("label") for o in self.hub.rooms.get(self.rid, {}).get("objects", [])]}


# key, name, unit, device class, state class, entity category, icon
NODE_VALUES = [
    ("heart_bpm", "Heart rate", BPM, None, SensorStateClass.MEASUREMENT, None, "mdi:heart-pulse"),
    ("breathing_bpm", "Breathing rate", BRPM, None, SensorStateClass.MEASUREMENT, None, "mdi:lungs"),
    ("n_persons", "Persons", None, None, SensorStateClass.MEASUREMENT, None, "mdi:account-multiple"),
    ("motion_energy", "Motion energy", None, None, SensorStateClass.MEASUREMENT, EntityCategory.DIAGNOSTIC, "mdi:waveform"),
    ("presence_score", "Presence score", None, None, SensorStateClass.MEASUREMENT, EntityCategory.DIAGNOSTIC, "mdi:radar"),
    ("rssi", "Signal", SIGNAL_STRENGTH_DECIBELS_MILLIWATT, SensorDeviceClass.SIGNAL_STRENGTH, SensorStateClass.MEASUREMENT, EntityCategory.DIAGNOSTIC, None),
    ("csi_rate", "CSI frame rate", "Hz", None, SensorStateClass.MEASUREMENT, EntityCategory.DIAGNOSTIC, "mdi:wifi"),
]


class NodeValue(R2SEntity, SensorEntity):
    def __init__(self, hub: Hub, nid: int, key, name, unit, dc, sc, cat, icon):
        super().__init__(hub)
        self.nid, self.key = nid, key
        self._attr_unique_id = f"{DOMAIN}_ruview_{nid}_{key}"
        self._attr_device_info = node_device(nid)
        self._attr_name = name
        self._attr_native_unit_of_measurement = unit
        self._attr_device_class = dc
        self._attr_state_class = sc
        self._attr_entity_category = cat
        if icon:
            self._attr_icon = icon

    @property
    def available(self) -> bool:
        ns = self.hub.ruview.nodes.get(self.nid)
        return ns is not None and self.hub.ruview.clock() - ns.last_seen < 30

    @property
    def native_value(self):
        ns = self.hub.ruview.nodes.get(self.nid)
        if ns is None:
            return None
        if self.key == "csi_rate":
            return round(ns.csi_rate, 1)
        if self.key == "rssi":
            return ns.vitals.rssi if ns.vitals else ns.csi_rssi
        if ns.vitals is None or not ns.fresh(self.hub.ruview.clock(), 30):
            return None
        v = getattr(ns.vitals, self.key)
        return round(v, 2) if isinstance(v, float) else v


class PersonRoom(R2SEntity, SensorEntity):
    _attr_icon = "mdi:account-search"

    def __init__(self, hub: Hub, pid: str, name: str):
        super().__init__(hub)
        self.pid = pid
        self._attr_unique_id = f"{DOMAIN}_{pid}_room"
        self._attr_device_info = person_device(pid, name)
        self._attr_name = "Room"

    def _p(self):
        return next((p for p in self.hub.state["people"] if p["id"] == self.pid), None)

    @property
    def native_value(self):
        p = self._p()
        if p is None:
            e = next((p for p in self.hub.state["elsewhere"] if p["id"] == self.pid), None)
            return "away" if e and not e.get("home") else "unknown"
        return self.hub.rooms.get(p["room_id"], {}).get("name", p["room_id"])

    @property
    def extra_state_attributes(self):
        p = self._p() or next((p for p in self.hub.state["elsewhere"] if p["id"] == self.pid), {}) or {}
        return {k: p.get(k) for k in ("room_id", "x", "y", "pose", "on", "position_source", "heart_rate",
                                      "heart_rate_source", "breathing_rate", "breathing_rate_source", "activity", "phone")}


class PersonVital(R2SEntity, SensorEntity):
    _attr_state_class = SensorStateClass.MEASUREMENT

    def __init__(self, hub: Hub, pid: str, name: str, key: str):
        super().__init__(hub)
        self.pid, self.key = pid, key
        self._attr_unique_id = f"{DOMAIN}_{pid}_{key}"
        self._attr_device_info = person_device(pid, name)
        self._attr_name = "Heart rate" if key == "heart_rate" else "Breathing rate"
        self._attr_native_unit_of_measurement = BPM if key == "heart_rate" else BRPM
        self._attr_icon = "mdi:heart-pulse" if key == "heart_rate" else "mdi:lungs"

    @property
    def native_value(self):
        p = next((p for p in self.hub.state["people"] if p["id"] == self.pid), None)
        return p.get(self.key) if p else None

    @property
    def extra_state_attributes(self):
        p = next((p for p in self.hub.state["people"] if p["id"] == self.pid), None)
        return {"source": p.get(f"{self.key}_source")} if p else {}
