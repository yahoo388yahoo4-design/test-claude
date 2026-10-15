"""Real2Sim Home: rooms mapped by the R2S Capture apps, people located in them with pose and vitals.

* Room maps (HOME.md room_map.json) come from the Android / iPhone app (POST /api/real2sim_home/room_map),
  the real2sim_home.upload_room_map service or MQTT (real2sim/room_map). Each room gets an HA area.
* RuView ESP32 CSI nodes send UDP packets here (port 5005): presence, motion, fall, heart and breathing rate.
* Devices in a room's area (motion / presence / mmWave radar, cameras, temperature, humidity, microphones,
  lights) and people (person entities with their phones / fitness bands) are fused by engine.py.
* The floor plan is shown in the "Home map" sidebar panel and as the custom:real2sim-home-card Lovelace card.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path

import voluptuous as vol

from homeassistant.components import panel_custom, websocket_api
from homeassistant.components.frontend import add_extra_js_url, async_remove_panel
from homeassistant.components.http import StaticPathConfig
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import EVENT_HOMEASSISTANT_STOP, Platform
from homeassistant.core import HomeAssistant, ServiceCall, callback
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers import config_validation as cv

from . import ruview
from .const import (
    CARD_FILE, CONF_FORWARD, CONF_MQTT_TOPIC, CONF_UDP_PORT, DEFAULT_MQTT_TOPIC, DEFAULT_UDP_PORT, DOMAIN, PANEL_PATH,
    URL_BASE,
)
from .api import RoomMapView
from .hub import Hub

_LOGGER = logging.getLogger(__name__)
PLATFORMS = [Platform.BINARY_SENSOR, Platform.SENSOR]
CONFIG_SCHEMA = cv.config_entry_only_config_schema(DOMAIN)


async def async_setup(hass: HomeAssistant, config) -> bool:
    hass.data.setdefault(DOMAIN, {})
    # frontend files + API are registered once per HA run
    await hass.http.async_register_static_paths(
        [StaticPathConfig(URL_BASE, str(Path(__file__).parent / "frontend"), False)])
    add_extra_js_url(hass, f"{URL_BASE}/{CARD_FILE}")
    hass.http.register_view(RoomMapView())
    websocket_api.async_register_command(hass, ws_subscribe)
    websocket_api.async_register_command(hass, ws_place)
    _register_services(hass)
    return True


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    opts = {**entry.data, **entry.options}
    hub = Hub(hass, opts)
    await hub.async_load()
    hass.data[DOMAIN]["hub"] = hub
    entry.runtime_data = hub

    port = int(opts.get(CONF_UDP_PORT, DEFAULT_UDP_PORT) or 0)
    transport = None
    if port:
        fwd = None
        if opts.get(CONF_FORWARD):
            host, _, fport = str(opts[CONF_FORWARD]).rpartition(":")
            fwd = (host or str(opts[CONF_FORWARD]), int(fport or DEFAULT_UDP_PORT))
        try:
            transport = await ruview.listen(hub.ruview, "0.0.0.0", port, fwd)
            _LOGGER.info("Real2Sim Home: listening for RuView CSI nodes on UDP %s", port)
        except OSError as err:
            _LOGGER.error("Real2Sim Home: cannot listen on UDP %s (%s); RuView nodes will not be received", port, err)
    unsubs = []
    if transport is not None:
        unsubs.append(transport.close)
        unsubs.append(hass.bus.async_listen_once(EVENT_HOMEASSISTANT_STOP, lambda _e: transport.close()))

    if "mqtt" in hass.config.components:
        from homeassistant.components import mqtt

        async def _mqtt_map(msg) -> None:
            try:
                await hub.async_add_room_map(json.loads(msg.payload))
            except (ValueError, KeyError, TypeError) as err:
                _LOGGER.warning("Real2Sim Home: bad room map on MQTT %s: %s", msg.topic, err)
        try:
            unsubs.append(await mqtt.async_subscribe(hass, opts.get(CONF_MQTT_TOPIC, DEFAULT_MQTT_TOPIC), _mqtt_map))
        except HomeAssistantError as err:
            _LOGGER.warning("Real2Sim Home: MQTT subscribe failed: %s", err)
    hass.data[DOMAIN]["unsubs"] = unsubs

    await panel_custom.async_register_panel(
        hass, frontend_url_path=PANEL_PATH, webcomponent_name="real2sim-home-panel", sidebar_title="Home map",
        sidebar_icon="mdi:floor-plan", module_url=f"{URL_BASE}/{CARD_FILE}", require_admin=False, config={})
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    await hub.async_start()
    entry.async_on_unload(entry.add_update_listener(_reload))
    return True


async def _reload(hass: HomeAssistant, entry: ConfigEntry) -> None:
    await hass.config_entries.async_reload(entry.entry_id)


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    hub: Hub = entry.runtime_data
    await hub.async_stop()
    await hub.store.async_save({"rooms": hub.rooms, "layout": hub.layout, "placements": hub.placements})
    for u in hass.data[DOMAIN].pop("unsubs", []):
        try:
            u()
        except Exception:  # noqa: BLE001
            pass
    async_remove_panel(hass, PANEL_PATH)
    hass.data[DOMAIN].pop("hub", None)
    return ok


def _hub(hass: HomeAssistant) -> Hub:
    hub = hass.data.get(DOMAIN, {}).get("hub")
    if hub is None:
        raise HomeAssistantError("Real2Sim Home is not set up")
    return hub


def _register_services(hass: HomeAssistant) -> None:
    async def upload(call: ServiceCall) -> None:
        data = call.data["room_map"]
        if isinstance(data, str):
            data = json.loads(data)
        try:
            await _hub(hass).async_add_room_map(data)
        except ValueError as err:
            raise HomeAssistantError(f"Invalid room map: {err}") from err

    async def remove(call: ServiceCall) -> None:
        await _hub(hass).async_remove_room(call.data["room_id"])

    async def layout(call: ServiceCall) -> None:
        _hub(hass).set_layout(call.data["room_id"], call.data["x"], call.data["y"], call.data.get("rotation", 0))

    async def place(call: ServiceCall) -> None:
        hub = _hub(hass)
        if call.data["room_id"] not in hub.rooms:
            raise HomeAssistantError(f"Unknown room {call.data['room_id']}")
        hub.place(call.data["item"], call.data["room_id"], call.data["x"], call.data["y"], call.data.get("rotation", 0))

    async def unplace(call: ServiceCall) -> None:
        _hub(hass).unplace(call.data["item"])

    hass.services.async_register(DOMAIN, "upload_room_map", upload, vol.Schema({vol.Required("room_map"): vol.Any(dict, str)}))
    hass.services.async_register(DOMAIN, "remove_room", remove, vol.Schema({vol.Required("room_id"): cv.string}))
    hass.services.async_register(DOMAIN, "set_room_layout", layout, vol.Schema({
        vol.Required("room_id"): cv.string, vol.Required("x"): vol.Coerce(float), vol.Required("y"): vol.Coerce(float),
        vol.Optional("rotation", default=0): vol.Coerce(float)}))
    hass.services.async_register(DOMAIN, "place", place, vol.Schema({
        vol.Required("item"): cv.string, vol.Required("room_id"): cv.string, vol.Required("x"): vol.Coerce(float),
        vol.Required("y"): vol.Coerce(float), vol.Optional("rotation", default=0): vol.Coerce(float)}))
    hass.services.async_register(DOMAIN, "unplace", unplace, vol.Schema({vol.Required("item"): cv.string}))


@websocket_api.websocket_command({vol.Required("type"): "real2sim_home/subscribe"})
@callback
def ws_subscribe(hass: HomeAssistant, connection: websocket_api.ActiveConnection, msg: dict) -> None:
    """Geometry (rooms, layout, placements) once and on change, live state on every update."""
    hub = hass.data.get(DOMAIN, {}).get("hub")
    if hub is None:
        connection.send_error(msg["id"], "not_ready", "Real2Sim Home is not set up")
        return

    @callback
    def forward(kind: str, data: dict) -> None:
        connection.send_message(websocket_api.event_message(msg["id"], {"kind": kind, "data": data}))

    connection.subscriptions[msg["id"]] = hub.subscribe(forward)
    connection.send_result(msg["id"])
    forward("geometry", hub.geometry())
    forward("state", hub.state)


@websocket_api.websocket_command({
    vol.Required("type"): "real2sim_home/place", vol.Required("item"): str, vol.Required("room_id"): str,
    vol.Required("x"): vol.Coerce(float), vol.Required("y"): vol.Coerce(float), vol.Optional("rotation", default=0): vol.Coerce(float)})
@websocket_api.require_admin
@callback
def ws_place(hass: HomeAssistant, connection: websocket_api.ActiveConnection, msg: dict) -> None:
    """Drag-and-drop from the card: move a sensor / node / radar (item) or a whole room (item 'room')."""
    hub = hass.data.get(DOMAIN, {}).get("hub")
    if hub is None or msg["room_id"] not in hub.rooms:
        connection.send_error(msg["id"], "not_found", "unknown room")
        return
    if msg["item"] == "room":
        hub.set_layout(msg["room_id"], msg["x"], msg["y"], msg["rotation"])
    else:
        hub.place(msg["item"], msg["room_id"], msg["x"], msg["y"], msg["rotation"])
    connection.send_result(msg["id"])
