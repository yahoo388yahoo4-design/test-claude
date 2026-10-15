# ESP32 WiFi sensing nodes (RuView CSI) in the room scan

Six ESP32-S3 boards running RuView's
[`esp32-csi-node`](https://github.com/ruvnet/RuView/blob/main/firmware/esp32-csi-node/README.md)
firmware sense people through WiFi Channel State Information. This folder sets them up, says where to
put them, and puts each node's position into the phone's 3D room scan, so the sensing and the scan share
one coordinate frame.

| Step | Tool | Result |
|---|---|---|
| 1. Flash + configure all 6 boards | `esp_fleet.py` | node ids 1-6, WiFi, TDM slots 0-5, streaming to your computer |
| 2. Print the node tags | `esp_tags.py` | `tags/esp_tags_A4.pdf`: tag N goes on board N |
| 3. Decide where they go | `esp_place.py` + [PLACEMENT.md](PLACEMENT.md) | `esp_plan.png`: positions and heights for your room |
| 4. Put them in the 3D scan | phone app (live), or `esp_locate.py` | `<session>/esp_nodes.json` |
| 5. Check | `esp_fleet.py monitor`, `esp_place.py --check`, `esp_locate.py show` | rates per node, coverage, a picture per node |

## What you need

* 6 × ESP32-S3 boards with 4 MB or more flash (DevKitC-1, XIAO ESP32-S3 and similar). Bare boards use
  the **4 MB headless** firmware build (the default here, it also fits 8 MB boards). Only boards with a
  display should use `--variant 8mb`: the firmware README warns that its display build can drop CSI to
  0 packets/s on boards without a panel.
* A USB data cable, 6 USB chargers, a **2.4 GHz** WiFi network (ESP32-S3 has no 5 GHz).
* A computer on that network for the RuView sensing server (its LAN IP goes into every node).
* Python 3.10+: `pip install 'esptool>=5.0' esp-idf-nvs-partition-gen pyserial opencv-contrib-python-headless scipy pillow`

## 1. Flash and configure the boards

```bash
printf '%s\n' 'your-wifi-password' > ~/.wifi-pass && chmod 600 ~/.wifi-pass
python esp_fleet.py init --ssid "YourWiFi" --password-file ~/.wifi-pass --zone living   # writes fleet.json
python esp_fleet.py fetch     # RuView v0.8.14-esp32 4 MB bundle + provision.py, SHA-256 checked
python esp_fleet.py setup     # plug in one board at a time; each is erased, flashed and provisioned
```

`setup` waits for each new USB serial port, reads the board's MAC (so the same board can't become two
nodes), erases it, writes the four firmware images at RuView's offsets (`0x0` bootloader, `0x8000`
partition table, `0xf000` OTA data, `0x20000` app), then runs RuView's own `provision.py` with
`--node-id N --tdm-slot N-1 --tdm-total 6 --edge-tier 2` and your WiFi and target IP. The password
stays in your file; `fleet.json` only stores its path. Resume with `setup --start 4`, redo one board
with `setup --only 3`, and print every command without running it with `--dry-run setup`.

Defaults you may want to change in `init`: `--target-ip` (default: this computer's LAN address),
`--no-tdm` (plain passive capture without TDM slots), `--ota-psk-file` (RuView's OTA key, see its
README), `--variant 8mb`.

Then start the sensing server on the target computer (from the RuView repo):

```bash
cd v2 && cargo run -p wifi-densepose-sensing-server -- --http-port 3000 --source auto \
  --udp-bind 0.0.0.0 --udp-allow 192.168.1.0/24      # your nodes' subnet
```

Before that, `python esp_fleet.py monitor` (it uses the same UDP port 5005, so run it while the server
is stopped) shows every node's CSI packets per second, RSSI, channel and presence flags. RuView's
acceptance floor is 20 packets/s per node.

## 2. Print the tags

`python esp_tags.py` writes `tags/esp_tags_A4.pdf`. Print at 100 % and check the 50 mm bar. Each 70 mm
tag has a textured border, which the phone apps track live as an ARCore / ARKit image (all six score
100/100 in Google's `arcoreimg` quality check), and a 36 mm ArUco marker (id = node id) that
`esp_locate.py` finds in recorded video. Stick tag N flat on board N's case, or on the wall right next
to the board, facing into the room. The tags are already bundled in both apps; rerun
`esp_tags.py --install` only if you change their design.

## 3. Where to put them

Read [PLACEMENT.md](PLACEMENT.md). In short: spread them around the walls, half at about 0.9 m and half
at about 1.9 m, no two closer than 1.5 m, nothing metal in front, not next to fans or the router.
For your actual room, scan it first (any capture mode with poses), then:

```bash
python esp_place.py --session /path/to/session      # or: --room 5x4x2.5
```

It writes `esp_plan.png` (top view with coverage, nodes and their 15 links) and `esp_plan.json`
(positions in the scan's frame, with heights above the floor).

## 4. Put the nodes into the 3D scan

Record a scan with the phone app in mode A (ARCore / ARKit RGB-D) with the boards in place.

**Automatically, while scanning.** Film each tag for a second from about 0.5-1.5 m. The app tracks the
tag and places node N at its centre; a blue ring and "ESP N" appear over the board.

**By hand, while scanning.** Tap **Mark ESP**, pick the node number, and tap the board on screen. The
app uses the depth under your finger (LiDAR on iPhone, ARCore depth or hit test on Android). If there's
no surface there yet, step to the side and tap it again: two taps from different angles are intersected.
Taps (green rings) always win over tag sightings (blue).

**By hand, afterwards.** In the session list (Android: **ESP** button; iPhone: open the session, then
**ESP** at the top), scrub to a frame that shows the board, pick the node, and tap it. One tap is enough
when the frame has depth; otherwise tap it in a second frame from another angle. Zoom helps with small
boards. Every placed node is drawn back into each frame, so scrub around to check it.

**On a computer**, for any session:

```bash
python esp_locate.py auto  SESSION      # finds the ArUco tags, refines with depth, outlier-robust
python esp_locate.py click SESSION      # window: a/d step frames, 1-6 pick node, click the board, s save
python esp_locate.py show  SESSION      # prints the nodes, writes esp_nodes_check.jpg
```

All of these write the same file, `<session>/esp_nodes.json`:

```json
{"version": 1, "format": "r2s-esp-nodes", "frame": "session_world", "units": "m", "up": "+y",
 "nodes": [{"node_id": 1, "label": "esp-1", "position": [0.004, 0.902, 1.199], "normal": [1, 0, 0],
            "method": "auto_tag", "frame_index": 230, "t": 7.67, "confidence": 0.76, "observations": 13}]}
```

`session_world` is the frame of the session's own poses, mesh and depth (ARKit / ARCore world, y up,
metres), so the nodes line up with everything else exported from that session. `method` is
`auto_tag` / `auto_tag_live` (tag), `tap_live` / `tap_frame` (one tap with depth), `triangulated`
(two or more taps without depth) or `planned` (from `esp_place.py`). A tap is never overwritten by a tag
sighting; `esp_locate.py auto --force` overrides that.

## Tests

`python test_esp.py` renders a synthetic 5 × 4 m room with the six tags on its walls, writes it as an
r2s session with depth, and checks: automatic tag location (all six within 0.5 cm, wall normals
exact), one tap with depth (within 1.1 cm), two taps without depth, the placement planner on that
scan, and the CSI / vitals packet parsing. The Android maths has JVM tests in
`android/app/src/test/.../EspNodesTest.kt`.

What is not tested here: real boards (flashing and provisioning were dry-run against the real RuView
v0.8.14 bundle, its checksums and partition table, and `provision.py --dry-run`), and the live tracking
in the apps on a phone.
