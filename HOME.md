# Room maps, presence and Home Assistant

The capture apps map whole rooms with their furniture and appliances, and the **Real2Sim Home** Home
Assistant integration shows those rooms with the people in them: where they are, their pose, and their
heart and breathing rate. It fuses the RuView ESP32 CSI node (Wi-Fi sensing) with the devices already in
Home Assistant: motion / presence / mmWave radar sensors, cameras, temperature and humidity, microphones,
lights, fitness bands and phones.

```
 R2S Capture (Android / iPhone) ──room_map.json──▶ POST /api/real2sim_home/room_map ─┐
 RuView ESP32 CSI node ──UDP 5005 (vitals 1 Hz, CSI 20 Hz)───────────────────────────┤
 HA devices in the room's area (motion, radar, cameras, temperature, mics, lights) ──┼─▶ engine.py ─▶ entities
 people: person entities + their phones / watches (room sensor, heart rate, battery) ─┘                + "Home map" panel / card
```

## 1. Scan a room

**Android (mode A, ARCore).** Settings › *Room map + Home Assistant*: keep *Map the room* on and type the room
name. Record the room slowly, pointing at every wall and piece of furniture. While recording, the object
detector (EfficientDet-Lite2, COCO, on the phone) draws boxes over the preview: green = placed in 3D with the
ARCore depth, yellow = seen but not placed yet. When you stop, the app writes `roommap/room_map.json` with:

* floor outline and walls from the ARCore planes (turned so the main wall direction is along x);
* objects: TV, toilet, chair, sofa, bed, table, refrigerator, oven, microwave, sink, plant, laptop, … with a
  3D box (each needs ≥ 3 sightings) and, when the TSDF mesh has it, a mesh crop
  `roommap/objects/<category>_<n>.ply` (the object's reconstruction, ARCore world coordinates).

**iPhone (mode B, RoomPlan, LiDAR iPhones).** Settings › *Room map + Home Assistant*: room name, Home
Assistant URL and token. RoomPlan finds walls, doors, windows, openings and objects (television, toilet,
chair, table, sofa, bed, sink, bathtub, oven, stove, refrigerator, dishwasher, washer/dryer, storage, fireplace,
stairs). Besides `roomplan/room.usdz` / `room.json` / `objects.json`, the app now writes `roomplan/room_mesh.usdz`
(RoomPlan's mesh export) and `roommap/room_map.json`.

**Send it to Home Assistant.** With the URL (e.g. `http://homeassistant.local:8123`) and a long-lived access
token (HA › your profile › Security › Long-lived access tokens) in Settings, each new room map is sent when the
recording stops. Older sessions: Sessions › long-press › *Send room map to Home Assistant*. Other ways in:
the `real2sim_home.upload_room_map` service, or MQTT (retained JSON on `real2sim/room_map`).

Scan one room per recording; rooms are placed next to each other on the home plan and you drag them into
place once (below).

### room_map.json

```json
{
  "format": "r2s-roommap", "version": 1, "id": "living_room", "name": "Living room",
  "source": {"platform": "android|ios", "session": "...", "device": "...", "method": "..."},
  "created_unix": 1760500000.0, "units": "m",
  "frame": "floor plan: x right, y up the map, z up from the floor; origin = room min corner",
  "world_to_room": [16 numbers, row-major 4x4: capture world (ARKit / ARCore, y up) -> this frame],
  "bounds": [0, 0, 5.2, 4.2], "height": 2.5,
  "floor": [[x, y], ...],
  "walls":    [{"a": [x, y], "b": [x, y], "height": 2.5}],
  "doors":    [{"a": [x, y], "b": [x, y], "height": 2.0, "parent": "<wall id>"}],
  "windows":  [...], "openings": [...],
  "objects":  [{"id": "tv_1", "category": "tv", "label": "Tv", "confidence": 0.9,
                "center": [x, y, z], "size": [width, depth, height], "yaw": 0.0, "mesh": "objects/tv_0.ply"}],
  "sensors":  [{"id": "ruview_1", "kind": "ruview", "node_id": 1, "label": "esp-1", "position": [x, y, z],
                "method": "tap_live", "confidence": 0.9}]
}
```

`sensors` (optional) are devices placed during the scan. The apps fill it from the session's `esp_nodes.json`
(RuView nodes placed with the ESP32 placement tools, session world frame) moved into the room frame with
`world_to_room`, when the room map is written and again whenever it is sent. Home Assistant places those RuView
nodes on the map, unless you have since dragged them by hand. An entry with `entity_id` instead of `node_id`
places that Home Assistant entity.

`yaw` is the rotation (radians, counter-clockwise) of the object's width axis from +x. Android maps COCO
labels to the same names iOS uses (`couch` → `sofa`, `dining table` → `table`, `television` → `tv`).

## 2. Install the integration

* **HACS:** HACS › Integrations › ⋮ › Custom repositories › this repository, category *Integration*; install
  *Real2Sim Home*; restart Home Assistant.
* **By hand:** copy `custom_components/real2sim_home/` (or unzip `real2sim_home.zip`) into
  `<config>/custom_components/` and restart.

Then Settings › Devices & services › Add integration › *Real2Sim Home*:

| option | default | |
| --- | --- | --- |
| RuView UDP port | 5005 | where the CSI nodes send; 0 turns the listener off |
| Forward RuView packets to | – | `host:port` of a RuView sensing server, so its own UI / pose model keeps working |
| MQTT topic for room maps | `real2sim/room_map` | used when the MQTT integration is set up |
| Night starts / ends | 22 / 7 | still people at night are put in bed, by day on seats |

A **Home map** entry appears in the sidebar. The same view is a dashboard card:

```yaml
type: custom:real2sim-home-card
title: Home
rooms: [living_room, bedroom]   # optional, default all
show_objects: true
show_sensors: true
show_list: true
```

## 3. RuView ESP32 CSI node

Flash the node as in RuView's `firmware/esp32-csi-node/README.md` and provision it with **`--target-ip` =
your Home Assistant's address** (port 5005), e.g.

```
python provision.py --port /dev/ttyUSB0 --ssid MyWifi --password-file wifi.txt --target-ip 192.168.1.20
```

The integration decodes the vitals packet (`0xC5110002`: presence, fall, motion, breathing rate, heart rate,
person count, motion energy, presence score) and counts CSI frames (`0xC5110001`). Each node becomes a device
*RuView node N* with presence / fall / motion binary sensors and heart rate, breathing rate, persons, motion
energy, presence score, signal and CSI frame rate sensors. Put the node in a room by giving its device the
room's area (Settings › Devices), or drag it on the map (Edit layout) so its position is known too. Running
RuView's own sensing server as well? Set *Forward RuView packets to* its address, or point the nodes at the
server and use its `--mqtt` Home Assistant entities: entities named *heart rate* / *breathing* in a room's
area are used as that room's vitals.

RuView's vitals are experimental (breathing 6–30, heart 40–120 bpm, not a medical measurement) and its
presence can be fooled by fans or microwaves; the map shows which source each number comes from.

## 4. Other devices

Every room map creates (or reuses) the Home Assistant **area** with the room's name. Devices you put in that
area are picked up by their type:

| in the area | used as |
| --- | --- |
| `binary_sensor` motion | presence (held 30 s after it clears) |
| `binary_sensor` occupancy / presence (PIR, mmWave such as LD2410 / FP2) | presence |
| `sensor.*_target_N_x` + `_y` (+ `_speed`), e.g. ESPHome LD2450 | positions of people (radar placed with *Edit layout*, facing up the map; or `real2sim_home.place` with `item: radar:<prefix>` and a rotation) |
| `camera` | camera marker (tap: live view) |
| temperature, humidity, illuminance, CO₂, sound pressure sensors | room readings |
| `assist_satellite` (voice satellites) | microphone marker, active while listening |
| `light` | light marker |
| sensors named *heart rate* / *breathing* | room vitals |

People are the `person` entities. For each person the integration looks at the devices of their device
trackers (phone with the Companion app, watch) and uses: a room sensor (`*_area` / `*_room`, e.g. Bermuda BLE
or ESPresense), heart rate (`*heart_rate*`: Wear OS / Galaxy Watch through the Companion app, Gadgetbridge,
Fitbit, Garmin, Withings, …), breathing / respiration, battery level and charging, detected activity, steps.
Anything it does not find by itself can be picked per person in the integration's options.

## 5. What the map shows and how it is worked out

For each room (`engine.py`):

* **occupied** if a presence / motion sensor, a RuView node, a radar target or a located person says so;
  **people count** = the largest of radar targets, RuView's person count, located people, 1 if occupied;
* **positions:** radar targets first; otherwise people still in the room are placed on a seat (or in bed at
  night), moving people near the RuView node, else in the middle of the room;
* **pose:** *fallen* (RuView fall flag, held 2 min), *walking* (phone activity, radar speed, RuView motion),
  *lying* (in the bed's box), *sitting* (in a sofa / chair / toilet / bench box), else *standing*;
* **heart / breathing rate:** the person's band / watch first, else the room's RuView node (given to the one
  person in the room, or the first one), else room vital sensors. A heart pulses at the heart rate and a ring
  breathes at the breathing rate.

Entities: per room *Occupied* and *People* (attributes: people with pose and position, objects); per person
*Room* (attributes x, y, pose, sources, phone), *Heart rate*, *Breathing rate*; per RuView node as above.

**Edit layout** (top right of the map, admins): drag a room by its floor to move it on the home plan,
double-click it to turn it 90°, drag a device marker to where it really is in the room. The same from
automations: `real2sim_home.set_room_layout`, `real2sim_home.place`, `real2sim_home.unplace`,
`real2sim_home.remove_room`.

## 6. Try it without hardware

```
python3 tools/home_demo.py --ha http://homeassistant.local:8123 --token <long-lived token>
```

uploads four demo rooms, places demo sensors, configures two people (`person.ana`, `person.bo` if they exist),
and simulates two RuView nodes (someone asleep in the bedroom, someone on the sofa who gets up now and then)
and an LD2450 radar for a few minutes. `--room-map path/to/room_map.json` uploads a real scan instead.

## 7. Tests

```
pip install pytest-homeassistant-custom-component home-assistant-frontend
python -m pytest -q -p no:homeassistant tests/engine                                     # engine, RuView packets
python -m pytest -q -c tests/homeassistant/pytest.ini --rootdir . tests/homeassistant    # in Home Assistant
cd android && ./gradlew testDebugUnitTest --tests com.real2sim.capture.RoomMapTest        # Android room geometry
ios/NavTests/run_tests.sh                                                                 # iOS RoomMapCore (+ nav, voice)
```
