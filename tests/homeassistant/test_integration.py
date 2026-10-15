"""Real2Sim Home inside a real Home Assistant (pytest-homeassistant-custom-component)."""
import asyncio
import json
import pathlib
import socket

from pytest_homeassistant_custom_component.common import MockConfigEntry

from homeassistant.core import HomeAssistant
from homeassistant.helpers import area_registry as ar
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import entity_registry as er
from homeassistant.setup import async_setup_component

from custom_components.real2sim_home import ruview
from custom_components.real2sim_home.const import DOMAIN

ROOM = json.loads((pathlib.Path(__file__).parent / "sample_room.json").read_text())


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


async def _setup(hass: HomeAssistant, port: int = 0):
    assert await async_setup_component(hass, "http", {})
    entry = MockConfigEntry(domain=DOMAIN, data={"udp_port": port}, unique_id=DOMAIN)
    entry.add_to_hass(hass)
    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()
    return entry


async def test_room_map_upload_creates_area_entities_and_state(hass: HomeAssistant, hass_client, hass_ws_client):
    port = _free_udp_port()
    await _setup(hass, port)
    client = await hass_client()
    # RuView node 7 was placed in the app during the scan (esp_nodes.json -> sensors layer)
    scanned = {**ROOM, "sensors": [{"id": "ruview_7", "kind": "ruview", "node_id": 7, "position": [1.2, 0.8, 1.5]}]}
    resp = await client.post("/api/real2sim_home/room_map", json=scanned)
    assert resp.status == 200, await resp.text()
    body = await resp.json()
    assert body["id"] == "living_room" and body["objects"] == 4
    assert hass.data[DOMAIN]["hub"].placements["ruview_7"]["x"] == 1.2
    await hass.async_block_till_done()
    area = ar.async_get(hass).async_get_area_by_name("Living room")
    assert area is not None and body["area_id"] == area.id
    assert (await client.post("/api/real2sim_home/room_map", json={"format": "nope"})).status == 400

    # a motion sensor and a thermometer in that area
    ents = er.async_get(hass)
    m = ents.async_get_or_create("binary_sensor", "test", "motion1", suggested_object_id="lr_motion")
    t = ents.async_get_or_create("sensor", "test", "temp1", suggested_object_id="lr_temp")
    ents.async_update_entity(m.entity_id, area_id=area.id)
    ents.async_update_entity(t.entity_id, area_id=area.id)
    hass.states.async_set(m.entity_id, "on", {"device_class": "motion", "friendly_name": "LR motion"})
    hass.states.async_set(t.entity_id, "21.4", {"device_class": "temperature", "unit_of_measurement": "°C"})

    # a RuView node sends vitals over UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for _ in range(3):
        sock.sendto(ruview.build_vitals(7, heart=66, breathing=14, n_persons=1), ("127.0.0.1", port))
        sock.sendto(ruview.build_csi(7, 1), ("127.0.0.1", port))
    sock.close()
    for _ in range(50):
        await asyncio.sleep(0.02)
        if 7 in hass.data[DOMAIN]["hub"].ruview.nodes:
            break
    await asyncio.sleep(0.6)
    await hass.async_block_till_done()

    states = {s.entity_id: s for s in hass.states.async_all()}
    occ = [s for e, s in states.items() if e.startswith("binary_sensor.") and "living_room" in e and "occupied" in e]
    assert occ and occ[0].state == "on", sorted(states)
    hr = [s for e, s in states.items() if e.startswith("sensor.ruview_node_7") and "heart" in e]
    assert hr and float(hr[0].state) == 66.0, sorted(states)
    # the node's device can be put in the area like any device; with one room it is assigned automatically
    dev = dr.async_get(hass).async_get_device(identifiers={(DOMAIN, "ruview_7")})
    assert dev is not None

    ws = await hass_ws_client(hass)
    await ws.send_json({"id": 1, "type": "real2sim_home/subscribe"})
    assert (await ws.receive_json())["success"]
    geo = (await ws.receive_json())["event"]
    st = (await ws.receive_json())["event"]
    assert geo["kind"] == "geometry" and geo["data"]["rooms"][0]["id"] == "living_room"
    assert st["kind"] == "state"
    room = st["data"]["rooms"][0]
    assert room["occupied"] and room["temperature"] == 21.4
    (p,) = st["data"]["people"]
    assert p["heart_rate"] == 66 and p["breathing_rate"] == 14 and p["pose"] in ("sitting", "standing")
    kinds = {mk["kind"] for mk in st["data"]["markers"]}
    assert {"motion", "temperature", "ruview"} <= kinds

    # drag a node to a spot from the card
    await ws.send_json({"id": 2, "type": "real2sim_home/place", "item": "ruview_7", "room_id": "living_room", "x": 0.5, "y": 0.5})
    while (msg := await ws.receive_json())["id"] != 2:
        assert msg["type"] == "event"
    assert msg["success"]
    assert hass.data[DOMAIN]["hub"].placements["ruview_7"] == {"room_id": "living_room", "x": 0.5, "y": 0.5, "rotation": 0}
    # a later scan does not move a node placed by hand
    assert (await client.post("/api/real2sim_home/room_map", json=scanned)).status == 200
    assert hass.data[DOMAIN]["hub"].placements["ruview_7"]["x"] == 0.5


async def test_person_with_watch_and_room_sensor(hass: HomeAssistant):
    await _setup(hass)
    await hass.services.async_call(DOMAIN, "upload_room_map", {"room_map": ROOM}, blocking=True)
    assert await async_setup_component(hass, "person", {})
    # phone device with a Bermuda-style area sensor and a watch heart rate on the same device
    devs = dr.async_get(hass)
    cfg = MockConfigEntry(domain="mobile_app")
    cfg.add_to_hass(hass)
    d = devs.async_get_or_create(config_entry_id=cfg.entry_id, identifiers={("mobile_app", "ana")}, name="Ana's phone")
    ents = er.async_get(hass)
    for uid, oid in (("t", "ana_phone"), ("a", "ana_phone_area"), ("h", "ana_phone_heart_rate"), ("b", "ana_phone_battery_level")):
        dom = "device_tracker" if uid == "t" else "sensor"
        ents.async_get_or_create(dom, "mobile_app", uid, suggested_object_id=oid, device_id=d.id, config_entry=cfg)
    hass.states.async_set("device_tracker.ana_phone", "home")
    hass.states.async_set("sensor.ana_phone_area", "Living room")
    hass.states.async_set("sensor.ana_phone_heart_rate", "72", {"unit_of_measurement": "bpm"})
    hass.states.async_set("sensor.ana_phone_battery_level", "81", {"device_class": "battery"})
    hass.states.async_set("person.ana", "home", {"friendly_name": "Ana", "device_trackers": ["device_tracker.ana_phone"]})
    hub = hass.data[DOMAIN]["hub"]
    hub.refresh()
    await hass.async_block_till_done()
    (p,) = hub.state["people"]
    assert p["name"] == "Ana" and p["room_id"] == "living_room" and p["heart_rate"] == 72
    assert p["phone"]["battery"] == 81
    await hass.async_block_till_done()
    assert hass.states.get("sensor.ana_real2sim_room").state == "Living room"
    assert float(hass.states.get("sensor.ana_real2sim_heart_rate").state) == 72


async def test_unload(hass: HomeAssistant):
    entry = await _setup(hass)
    assert await hass.config_entries.async_unload(entry.entry_id)


def test_services_yaml_is_valid():
    from homeassistant.helpers.service import _SERVICES_SCHEMA
    from homeassistant.util.yaml import load_yaml_dict

    _SERVICES_SCHEMA(load_yaml_dict(str(pathlib.Path(__file__).parents[2] / "custom_components" / "real2sim_home" / "services.yaml")))
