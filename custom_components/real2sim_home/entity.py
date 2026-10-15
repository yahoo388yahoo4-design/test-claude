"""Shared entity bits."""
from __future__ import annotations

from homeassistant.helpers.device_registry import DeviceInfo
from homeassistant.helpers.dispatcher import async_dispatcher_connect
from homeassistant.helpers.entity import Entity

from .const import DOMAIN, NAME, SIGNAL_UPDATE
from .hub import Hub


class R2SEntity(Entity):
    _attr_should_poll = False
    _attr_has_entity_name = True

    def __init__(self, hub: Hub):
        self.hub = hub

    async def async_added_to_hass(self) -> None:
        self.async_on_remove(async_dispatcher_connect(self.hass, SIGNAL_UPDATE, self.async_write_ha_state))


def room_device(hub: Hub, rid: str) -> DeviceInfo:
    room = hub.rooms.get(rid, {})
    return DeviceInfo(identifiers={(DOMAIN, f"room_{rid}")}, name=f"{room.get('name', rid)} map", manufacturer=NAME,
                      model=f"room map ({room.get('source', {}).get('platform', '?')})", suggested_area=room.get("name"))


def node_device(node_id: int) -> DeviceInfo:
    return DeviceInfo(identifiers={(DOMAIN, f"ruview_{node_id}")}, name=f"RuView node {node_id}", manufacturer="RuView",
                      model="ESP32 CSI node")


def person_device(pid: str, name: str) -> DeviceInfo:
    return DeviceInfo(identifiers={(DOMAIN, f"person_{pid}")}, name=f"{name} (Real2Sim)", manufacturer=NAME, model="person tracking")
