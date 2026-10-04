# r2s-capture raw session format (v1)

This is the format the R2S Capture iOS app writes. `tools/convert.py` reads it (reader `r2s`) and
turns it into ARKitScenes and LiteReality layouts. An Android app can write the same format. Fields
that Android can't fill are left out or set to `null`, and the converter treats them as optional.

One capture is one folder, named `YYYYMMDD_HHMMSS_<mode>`:

```
<session>/
  session.json              metadata (required)
  frames.jsonl              one JSON object per camera frame (required for ARKit modes)
  video.mov                 HEVC video, one sample per frame in frames.jsonl, in the same order (required)
  depth.zlib.bin            per-frame LiDAR depth, raw-deflate(uint16 little-endian millimetres)   (optional)
  conf.zlib.bin             per-frame depth confidence, raw-deflate(uint8 0=low 1=medium 2=high)      (optional)
  depth_smooth.zlib.bin     same as depth, ARKit smoothedSceneDepth                                     (optional)
  conf_smooth.zlib.bin      confidence for smoothed depth                                               (optional)
  imu.csv                   fused device motion, ~100-200 Hz
  accel.csv gyro.csv mag.csv  raw sensor streams
  location.csv heading.csv altimeter.csv altimeter_abs.csv status.csv clock.csv
  mesh.ply                  ARKit scene mesh, world frame, per-face classification
  planes.json               ARKit plane anchors
  worldmap.arworldmap       ARWorldMap (NSKeyedArchiver), for relocalisation
  hires/hires_<t>.jpg + .json   optional full-sensor stills (captureHighResolutionFrame)
  roomplan/room.usdz        RoomPlan CapturedRoom export (mode B)
  roomplan/room.json        CapturedRoom as Codable JSON (Apple's encoding)
  roomplan/objects.json     simplified RoomPlan objects and surfaces (see below; the converter uses this one)
  cams/<name>.mov + cams/<name>.jsonl   extra cameras (mode C)
  cams/lidar_depth.zlib.bin + cams/lidar_depth.jsonl   AVFoundation LiDAR depth (mode C)
  cams/calibration.json     intrinsics, lens distortion tables, extrinsics between cameras (mode C)
```

## Clocks

Every `t` is in **seconds on the device's monotonic uptime clock** (iOS: `ARFrame.timestamp`,
`CMLogItem.timestamp`, `CMClockGetHostTimeClock`, all the same timebase; Android:
`SensorEvent.timestamp` / `Image.getTimestamp()` in `SystemClock.elapsedRealtimeNanos` / 1e9). Sensors
that report wall-clock time (GPS) are converted to uptime when they are logged, and the wall time is
kept too. `clock.csv` (`uptime,unix_time`) is written once a second so any stream can be mapped
between the two clocks.

## Coordinate conventions (as recorded, before conversion)

* **World:** ARKit world frame. Y points up (against gravity), X and Z are horizontal, the origin is
  where the session started, units are metres (`worldAlignment = .gravity`).
* **Camera:** ARKit camera frame. +X points right in the sensor's landscape image, +Y up, and the
  camera looks down −Z. `T_c2w` maps camera coordinates to world coordinates.
* **Images:** stored in the sensor's native landscape orientation (e.g. 1920×1440), never rotated.
  `K` is given at that resolution.

Android writers have to convert into these conventions (ARCore already uses the same world and camera
axes as ARKit).

## session.json

```json
{
  "format": "r2s-capture", "version": 1,
  "mode": "arkit_rgbd" | "arkit_roomplan" | "multicam",
  "platform": "ios" | "android",
  "device": "iPhone18,1", "os": "iOS 26.0", "app_version": "0.1.0 (12)",
  "start_unix": 1790000000.123, "start_uptime": 12345.678,
  "end_unix": ..., "end_uptime": ...,
  "video": {"file": "video.mov", "width": 1920, "height": 1440, "fps": 60, "codec": "hevc", "bitrate": 50000000},
  "depth": {"width": 256, "height": 192, "unit": "mm", "dtype": "uint16", "compression": "raw-deflate"},
  "counts": {"frames": 3600, "depth": 3600, "imu": 12000, "location": 60},
  "settings": { ...the options used... },
  "notes": "free text"
}
```

## frames.jsonl (one line per video frame, in video order)

| key | type | meaning |
|---|---|---|
| `i` | int | frame index = sample index in `video.mov` |
| `t` | float | timestamp in seconds (uptime clock) |
| `w`,`h` | int | image size (pixels) |
| `K` | [fx, fy, cx, cy] | pinhole intrinsics at w×h |
| `T` | 16 floats | camera-to-world 4×4, **row-major**, ARKit world and camera axes |
| `track` | string | `normal`, `limited:<reason>`, `not_available` |
| `exp` | float | exposure duration (s) |
| `eo` | float | exposure offset (EV) |
| `iso` | float or null | ISO, if the device reports it |
| `amb`, `ct` | float | ARKit ambient light intensity (lumen) and colour temperature (K) |
| `wm` | string | world mapping status |
| `d` | [offset, length] or null | byte range of this frame's compressed depth in `depth.zlib.bin` |
| `c` | [offset, length] or null | same for `conf.zlib.bin` |
| `sd`, `sc` | [offset, length] or null | smoothed depth / confidence |
| `dw`, `dh` | int | depth map size (e.g. 256, 192) |

Decode depth in Python: `np.frombuffer(zlib.decompress(blob[o:o+n], -15), '<u2').reshape(dh, dw)`
(millimetres, 0 = invalid). Confidence uses the same call with `np.uint8`.

The depth map lines up with the RGB frame (same field of view), so its intrinsics are
`K * dw / w`.

## Sensor CSVs (header row included; `t` = uptime seconds)

* `imu.csv`: `t, ax, ay, az` (user acceleration, g), `gx, gy, gz` (rotation rate, rad/s, bias-corrected),
  `grx, gry, grz` (gravity, g), `qx, qy, qz, qw` (attitude), `mx, my, mz` (calibrated magnetic field, µT),
  `mag_acc` (−1..2), `heading` (deg, −1 if unknown). Device axes are the iOS device frame
  (x right, y top of screen, z out of screen, in portrait).
* `accel.csv`: `t, x, y, z` (raw accelerometer, g). `gyro.csv`: `t, x, y, z` (raw, rad/s).
  `mag.csv`: `t, x, y, z` (raw magnetometer, µT).
* `location.csv`: `t, unix, lat, lon, alt, ellipsoidal_alt, hacc, vacc, speed, speed_acc, course,
  course_acc, floor, simulated`.
* `heading.csv`: `t, true_heading, magnetic_heading, accuracy, x, y, z`.
* `altimeter.csv`: `t, rel_alt_m, pressure_kpa`. `altimeter_abs.csv`: `t, alt_m, accuracy, precision`.
* `status.csv`: `t, unix, thermal (0 nominal..3 critical), battery (0..1), battery_state, low_power`.
* `clock.csv`: `uptime, unix`.

## mesh.ply

Binary little-endian PLY in the ARKit world frame:
`vertex: float x, y, z, nx, ny, nz`, `face: uchar n(=3), int v0, v1, v2, uchar cls`. `cls` is
ARMeshClassification: 0 none, 1 wall, 2 floor, 3 ceiling, 4 table, 5 seat, 6 window, 7 door.

## planes.json

`[{"id", "alignment": "horizontal"|"vertical", "classification", "T": 16 row-major, "extent": [w, h], "boundary": [[x, y, z], ...]}]`
(boundary points are in the plane's local frame).

## roomplan/objects.json

```json
{"objects":  [{"id": "...", "category": "table", "confidence": "high", "dims": [w, h, d], "T": [16 row-major]}],
 "walls":    [{"id": "...", "category": "wall", "dims": [w, h, d], "T": [...]}],
 "doors": [...], "windows": [...], "openings": [...], "floors": [...]}
```

`dims` and `T` are RoomPlan's `dimensions` and `transform` (object-to-world, ARKit axes). The box's
local +Y axis is up, and the box spans `±dims/2` around the origin of `T`.

## Mode C (multicam) files

* `cams/<name>.mov`: HEVC per camera (`wide`, `ultrawide`, `tele`, `front`, `lidar_rgb`).
* `cams/<name>.jsonl`: `{"i", "t", "w", "h", "K": [fx, fy, cx, cy] or null, "exp", "iso"}` per frame.
* `cams/lidar_depth.jsonl`: `{"i", "t", "w", "h", "d": [offset, length], "K": [...], "ref_wh": [W, H], "quality", "accuracy"}`.
  Depth is uint16 mm, raw-deflate, in `cams/lidar_depth.zlib.bin`.
* `cams/calibration.json`: per camera: `intrinsics`, `reference_dims`, `pixel_size_mm`,
  `lens_distortion_center`, `lens_distortion_lut` and `inverse_lut` (float arrays), and
  `extrinsics_to` (4×3 matrices relative to the other cameras, from `AVCaptureDevice.extrinsicMatrix`).

Mode C has **no 6-DoF poses**: ARKit can't run while AVFoundation holds several cameras.
`tools/recover_poses.py` estimates poses offline with structure-from-motion and uses the LiDAR depth
to set the scale.

## Other writers (e.g. Android)

Write `session.json` with `"platform": "android"` and, at minimum, `frames.jsonl` + `video.mov`
(or `frames/<i>.jpg`; the reader accepts a `frames/` folder of JPEGs instead of a video), with `T`
in ARKit/ARCore axes. You can also register your own reader in `tools/readers/__init__.py` (see
`tools/readers/base.py`).
