"""Room occupancy and RuView node presence / fall / motion."""
from __future__ import annotations

from homeassistant.components.binary_sensor import BinarySensorDeviceClass, BinarySensorEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import DOMAIN, SIGNAL_NEW_NODE, SIGNAL_NEW_ROOM
from .entity import R2SEntity, node_device, room_device
from .hub import Hub


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry, add: AddEntitiesCallback) -> None:
    hub: Hub = entry.runtime_data

    @callback
    def new_room(rid: str) -> None:
        add([RoomOccupancy(hub, rid)])

    @callback
    def new_node(nid: int) -> None:
        add([NodeFlag(hub, nid, "presence", BinarySensorDeviceClass.OCCUPANCY),
             NodeFlag(hub, nid, "fall", BinarySensorDeviceClass.SAFETY),
             NodeFlag(hub, nid, "motion", BinarySensorDeviceClass.MOTION)])

    for rid in list(hub.rooms):
        new_room(rid)
    for nid in list(hub.ruview.nodes):
        new_node(nid)
    entry.async_on_unload(async_dispatcher_connect(hass, SIGNAL_NEW_ROOM, new_room))
    entry.async_on_unload(async_dispatcher_connect(hass, SIGNAL_NEW_NODE, new_node))


class RoomOccupancy(R2SEntity, BinarySensorEntity):
    _attr_device_class = BinarySensorDeviceClass.OCCUPANCY
    _attr_translation_key = "occupancy"

    def __init__(self, hub: Hub, rid: str):
        super().__init__(hub)
        self.rid = rid
        self._attr_unique_id = f"{DOMAIN}_room_{rid}_occupancy"
        self._attr_device_info = room_device(hub, rid)
        self._attr_name = "Occupied"

    def _room(self):
        return next((r for r in self.hub.state["rooms"] if r["id"] == self.rid), None)

    @property
    def available(self) -> bool:
        return self.rid in self.hub.rooms

    @property
    def is_on(self) -> bool | None:
        r = self._room()
        return r["occupied"] if r else None

    @property
    def extra_state_attributes(self):
        r = self._room() or {}
        return {"people": r.get("count"), "sources": r.get("reasons"), "fall": r.get("fall")}


class NodeFlag(R2SEntity, BinarySensorEntity):
    def __init__(self, hub: Hub, nid: int, key: str, dc):
        super().__init__(hub)
        self.nid, self.key = nid, key
        self._attr_device_class = dc
        self._attr_unique_id = f"{DOMAIN}_ruview_{nid}_{key}"
        self._attr_device_info = node_device(nid)
        self._attr_name = {"presence": "Presence", "fall": "Fall", "motion": "Motion"}[key]

    @property
    def available(self) -> bool:
        ns = self.hub.ruview.nodes.get(self.nid)
        return ns is not None and ns.fresh(self.hub.ruview.clock(), 30)

    @property
    def is_on(self) -> bool | None:
        ns = self.hub.ruview.nodes.get(self.nid)
        if ns is None or ns.vitals is None:
            return None
        if self.key == "fall":
            return ns.vitals.fall or (ns.last_fall_at and self.hub.ruview.clock() - ns.last_fall_at < 120)
        return getattr(ns.vitals, self.key)
