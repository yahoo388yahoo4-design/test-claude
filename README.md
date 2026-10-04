# iPhone 17 Pro → ARKitScenes / LiteReality datasets

This folder answers two questions: how to capture data like
[ARKitScenes](https://github.com/apple/ARKitScenes) and [LiteReality](https://litereality.github.io/)
with an iPhone 17 Pro, and how to get the **richest** data out of the phone.

| You want | Use | Then |
|---|---|---|
| Capture today with no build step, LiteReality input | **LiteReality app** (free, by the LiteReality authors) or **3D Scanner App** "All Data" export with RoomPlan on | feed it to LiteReality directly, or `tools/convert.py` → ARKitScenes |
| Capture today, ARKitScenes-style RGB-D + poses | **Stray Scanner** (free, open source) | `tools/convert.py --to arkitscenes` |
| Every sensor the phone has (cameras, LiDAR, IMU, GPS, barometer…) in one raw format | **R2S Capture**, the app in `ios/` | `tools/convert.py` → ARKitScenes **and** LiteReality, plus `extras/` |

**New: navigation mode.** The app's **Nav** button turns the phone into a robot's SLAM, obstacle-avoidance
and path-planning stack, with voice and visual guidance and motor control over Wi-Fi or Bluetooth. See
[NAVIGATION.md](NAVIGATION.md) and [robot/PROTOCOL.md](robot/PROTOCOL.md).

Contents: [what the datasets contain](#1-what-the-two-datasets-contain) ·
[ready-made apps](#2-ready-made-apps) · [our app](#3-r2s-capture-our-app) ·
[sensor table](#4-every-iphone-17-pro-sensor-and-where-it-lands) · [build and install](#5-build-and-install-without-a-mac) ·
[converter](#6-converter-and-tools) · [what was verified](#7-what-was-verified) ·
[capture tips](#8-capture-tips) · [limits](#9-honest-limits) · raw format: [FORMAT.md](FORMAT.md) ·
sideloading: [ios/SIDELOAD.md](ios/SIDELOAD.md)

---

## 1. What the two datasets contain

**ARKitScenes** (Apple, 2021): 5,047 captures of 1,661 rooms, recorded with the iPad Pro's ARKit (LiDAR
depth plus 6-DoF poses) and a Faro laser scanner for ground truth. The *raw* layout per `video_id`:

| asset | content | can an iPhone make it? |
|---|---|---|
| `lowres_wide/<vid>_<ts>.png` | 256×192 RGB, 60 Hz | yes (downscaled ARKit frames) |
| `lowres_depth/`, `confidence/` | 256×192 uint16 mm LiDAR depth, uint8 0–2 confidence | yes (ARKit `sceneDepth`) |
| `lowres_wide_intrinsics/*.pincam` | `w h fx fy cx cy` | yes |
| `lowres_wide.traj` | ~10 Hz, `ts rx ry rz tx ty tz` (axis-angle + translation, world-to-camera, z-up world) | yes (ARKit poses, converted) |
| `wide/` 1920×1440 10 Hz, `vga_wide/` 640×480 30 Hz | RGB | yes |
| `ultrawide/` 640×480 | ultrawide RGB | only in a separate pass without ARKit (mode C) |
| `<vid>_3dod_mesh.ply` | coloured mesh, z-up | yes: ARKit scene mesh, coloured by projecting the RGB frames |
| `<vid>_3dod_annotation.json` | oriented 3D boxes for 17 furniture classes | yes, automatically from RoomPlan objects (no human labelling) |
| `highres_depth/`, laser point clouds | Faro laser scanner | **no** |
| `metadata.csv` | `video_id, visit_id, sky_direction, fold, …` | yes (sky_direction computed from gravity) |

**LiteReality** (2025) turns an iPhone scan into a graphics-ready 3D scene (CAD-retrieved objects, PBR
materials, articulation). Its input is a **3D Scanner App "All Data" export with RoomPlan enabled**:
`frame_XXXXX.jpg` + `frame_XXXXX.json` (`cameraPoseARFrame` = 4×4 ARKit camera-to-world, `intrinsics`
3×3), `depth_XXXXX.png`, `conf_XXXXX.png`, `roomplan/room.usdz` (walls, doors, windows, objects) and
`textured_output.obj`. The authors also released a **LiteReality iOS app** that records the same data in
one pass.

## 2. Ready-made apps

All of these run on an iPhone 17 Pro without building anything. "Converter" means `tools/convert.py`
reads that app's export directly.

| App | Cost | RGB | LiDAR depth + confidence | Poses | Mesh | RoomPlan | IMU / GPS | Export | Converter | Best for |
|---|---|---|---|---|---|---|---|---|---|---|
| **LiteReality** (LiteReality authors) | free | ✓ | ✓ | ✓ ARKit | ✓ | ✓ | – | LiteReality scan folder | via the 3D Scanner layout | LiteReality, zero setup |
| **3D Scanner App** (Laan Labs) | free (some extras paid) | ✓ JPG frames | ✓ 256×192 | ✓ per frame | ✓ textured OBJ | ✓ (enable RoomPlan) | – | "All Data" folder | ✓ `3dscannerapp` | LiteReality's reference input |
| **Stray Scanner** | free, open source | ✓ `rgb.mp4` 1920×1440 | ✓ PNG | ✓ `odometry.csv` | – | – | IMU ✓ | folder via Files | ✓ `strayscanner` | ARKitScenes-style RGB-D |
| **Record3D** | free to record; export / streaming is a paid unlock | ✓ | ✓ | ✓ | – | – | – | `.r3d`, USB/Wi-Fi live streaming | – (export to PLY/EXR first) | live RGB-D streaming to a PC |
| **Polycam** | free tier; raw export needs Developer Mode (some exports are paid) | ✓ | ✓ | ✓ | ✓ | ✓ (room mode) | – | images, depth, confidence, cameras, `raw.glb` | – | quick meshes, raw data with Developer Mode |
| **NeRFCapture** | free, open source | ✓ | ✓ | ✓ `transforms.json` | – | – | – | NeRF / Instant-NGP format, live streaming | – | NeRF / Gaussian splatting |
| **Spectacular Rec** (Spectacular AI) | free | ✓ | ✓ | ✓ | – | – | IMU ✓ | video, depth, IMU, poses | – | SLAM / VIO research |
| **Scaniverse** (Niantic) | free | – (no raw frames) | – | – | ✓ mesh / splat | – | – | mesh, splat | – | finished meshes and splats only |
| **R2S Capture** (this repo) | free (sideload or TestFlight) | ✓ HEVC every frame, 1920×1440@60 or 4K@30, optional 48 MP stills | ✓ raw + smoothed | ✓ every frame | ✓ classified mesh | ✓ (mode B) | ✓ IMU, magnetometer, barometer, GPS, compass | r2s raw ([FORMAT.md](FORMAT.md)) | ✓ `r2s` | richest data, both datasets |

What to use:
* **LiteReality:** the LiteReality app or the 3D Scanner App (All Data + RoomPlan). Both give LiteReality
  exactly the folder it expects.
* **ARKitScenes-style RGB-D with poses:** Stray Scanner today, then `convert.py --to arkitscenes`. Stray
  doesn't export a mesh, so the converter fuses a coloured point cloud from the depth maps instead.
* **The richest data, one capture for both datasets:** R2S Capture mode B (RGB-D + RoomPlan) for rooms;
  mode A when you don't need RoomPlan; mode C for the extra cameras.

## 3. R2S Capture, our app

SwiftUI + ARKit + RoomPlan + AVFoundation + CoreMotion + CoreLocation, iOS 17+, in `ios/`
(XcodeGen `project.yml`, 12 Swift files). **Status: not yet compiled.** No Mac was available, so the
GitHub Actions workflow is its first real compile check (see §5).

| Mode | Engine | Records | Converts to |
|---|---|---|---|
| **A · RGB-D + poses** | ARKit world tracking, LiDAR `sceneDepth` + `smoothedSceneDepth`, scene mesh with classes | HEVC video of every ARFrame (1920×1440@60, or 4K@30), per-frame K, pose, tracking state, exposure, ISO, light estimate; depth + confidence (raw and smoothed) per frame; optional full-res stills (`captureHighResolutionFrame`); mesh, planes, ARWorldMap; audio (optional); all sensors | ARKitScenes (+ LiteReality without RoomPlan) |
| **B · RGB-D + RoomPlan** | RoomPlan `RoomCaptureView(arSession:)` **sharing** the ARSession; frames polled from `currentFrame` | everything in A, plus `roomplan/room.usdz`, `room.json` (CapturedRoom) and `objects.json` | LiteReality **and** ARKitScenes with 3D boxes from RoomPlan objects |
| **C · Multi-camera + LiDAR** | `AVCaptureMultiCamSession`: the largest camera set the phone supports at once (LiDAR depth camera + ultrawide + tele, optionally front) | one HEVC file per camera with per-frame intrinsics, exposure and ISO; AVFoundation LiDAR depth (denser than ARKit's 256×192) with calibration and lens-distortion tables; factory extrinsics between cameras; all sensors. **No ARKit poses** | poses recovered offline (`tools/recover_poses.py`), then ARKitScenes `ultrawide/`, `vga_wide/` and extras |

Always recorded (all modes): fused device motion and raw accelerometer / gyroscope (200 Hz requested;
iOS delivers at most ~100 Hz on most iPhones), raw and calibrated magnetometer, barometer (relative
altitude + pressure), absolute altitude, GPS (lat / lon / alt, accuracies, speed, course, floor), compass
heading, thermal state, battery, and a wall-clock ↔ uptime table. Every stream uses the ARFrame uptime
clock.

How the app handles data: the ARSession delegate runs on a background queue; ARFrames are never
retained (pixel buffers are copied into the writer's own pool); depth is written as raw-deflate uint16 mm
blobs with byte ranges in `frames.jsonl`. Captures land in *Files › On My iPhone › R2S Capture › sessions*
(Finder / Files transfer is enabled), or go to a computer over Wi-Fi with `tools/receiver.py`.

## 4. Every iPhone 17 Pro sensor and where it lands

| Sensor | Recorded in | ARKitScenes | LiteReality | extras/ |
|---|---|---|---|---|
| 48 MP Fusion main (wide) camera | A, B (ARKit, 1920×1440@60 or 3840×2880@30; full-sensor stills optional); C (AVFoundation, up to 1920 wide) | `lowres_wide/` (256×192), `wide/`, `vga_wide/` | `frame_XXXXX.jpg` | `video.mov`, `hires/` |
| 48 MP ultrawide | C only | `ultrawide/` (after pose recovery) | – | `cams/ultrawide.mov` + `.jsonl` |
| 48 MP telephoto | C only (if the multi-cam set allows it) | – | – | `cams/tele.mov` |
| 18 MP front camera | C (optional) | – | – | `cams/front.mov` |
| LiDAR scanner | A, B: ARKit depth 256×192 + confidence (raw and smoothed); C: AVFoundation depth (denser) | `lowres_depth/`, `confidence/` | `depth_XXXXX.png`, `conf_XXXXX.png` | `depth_smooth`, `cams/lidar_depth.*` |
| ARKit visual-inertial tracking | A, B | `lowres_wide.traj` | `cameraPoseARFrame` | `poses_all_frames.csv` (every frame) |
| ARKit scene reconstruction | A, B | `<vid>_3dod_mesh.ply` (coloured by projection) | `textured_output.obj` | `mesh.ply` with per-face classes, `planes.json` |
| RoomPlan | B | `<vid>_3dod_annotation.json` (objects → 17 classes) | `roomplan/room.usdz` | `roomplan/room.json`, `objects.json` (walls, doors, windows, openings, floors) |
| Accelerometer, gyroscope (raw + fused) | all | – | – | `imu.csv`, `accel.csv`, `gyro.csv` |
| Magnetometer (raw + calibrated), compass | all | – | – | `mag.csv`, `imu.csv`, `heading.csv` |
| Barometer / altimeter | all | – | – | `altimeter.csv`, `altimeter_abs.csv` |
| GNSS (dual-frequency GPS) | all (if allowed) | – | – | `location.csv` |
| Microphones | A, B (optional AAC track) | – | – | audio track in `video.mov` |
| Exposure, ISO, light estimate | A, B, C | – | `exposureDuration` | `frames.jsonl`, `poses_all_frames.csv` |
| Gravity | all | `metadata.csv` `sky_direction` | – | `imu.csv` |
| Thermal state, battery | all | – | – | `status.csv` |
| Not captured | UWB (needs a second device), Wi-Fi / Bluetooth scans, ambient-light and proximity sensors (no public API), the Faro laser scanner ground truth | | | |

## 5. Build and install without a Mac

The workflow `.github/workflows/ios-capture.yml` (repo root = this `capture/` folder) runs on GitHub's
macOS runners:

1. **`build-unsigned`** (always): `brew install xcodegen` → `xcodegen generate` → `xcodebuild` for a
   generic iOS device with signing off → `R2SCapture-unsigned.ipa` as an Actions artifact (+ the build
   log). This is also the compile check.
2. **`testflight`** (only if the secrets exist): archive with automatic signing through an App Store
   Connect API key, then export and upload to TestFlight. Without the secrets it is skipped with a notice.

Install routes:
* **Free Apple ID (no developer account):** sideload the unsigned `.ipa` with **AltStore** (main route)
  or **Sideloadly**, which re-sign it with your Apple ID. The app expires after 7 days (refresh it),
  a free ID can have at most 3 sideloaded apps (AltStore uses one), and Developer Mode must be on.
  Step by step: [ios/SIDELOAD.md](ios/SIDELOAD.md).
* **Paid Apple Developer Program ($99/yr), the simplest long-term route:** add the secrets below; every
  push uploads a TestFlight build that installs from the TestFlight app and lasts 90 days.

| Secret | Value |
|---|---|
| `ASC_KEY_ID` | App Store Connect API key ID (Users and Access › Integrations › Keys, role App Manager) |
| `ASC_ISSUER_ID` | the issuer ID shown on that page |
| `ASC_KEY_P8_BASE64` | `base64 -w0 AuthKey_XXXX.p8` |
| `APPLE_TEAM_ID` | 10-character team ID (Membership page) |
| variable `APP_BUNDLE_ID` (optional) | defaults to `com.yahoo4.r2scapture`; create an App Store Connect app record with this bundle ID once |

Notes: macOS runner minutes on **private** repos count 10× against the free Actions quota (2,000
min/month on the free plan ≈ 200 macOS minutes; one build takes about 5–10 min). The app uses only
Info.plist permissions (camera, microphone, motion, location, local network), with no entitlements, so a
free Apple ID can sign it.

## 6. Converter and tools

Python 3.10+, `numpy scipy opencv-python-headless` (+ `pycolmap` for `recover_poses.py --method sfm`).

```bash
# any input -> both datasets (+ extras/)
python tools/convert.py SESSION --to arkitscenes litereality --out OUT
#   OUT/arkitscenes/<video_id>/...  OUT/arkitscenes/metadata.csv  OUT/litereality/<name>/...
python tools/convert.py SESSION --to arkitscenes --out OUT --video-id 48000001 --wide --vga --traj-hz 10
python tools/convert.py STRAY_EXPORT --to arkitscenes --out OUT       # Stray Scanner
python tools/convert.py SCANNER3D_EXPORT --to arkitscenes --out OUT   # 3D Scanner App All Data
python tools/validate.py OUT/arkitscenes/<video_id>                  # PASS/WARN/FAIL report
python tools/validate.py OUT/litereality/<name>
python tools/receiver.py --root ~/captures --port 8765 --token SECRET --convert ~/converted   # Wi-Fi upload target
python tools/recover_poses.py MODE_C_SESSION --cam lidar_rgb --method sfm --out SESSION_posed  # then convert.py
python tools/roundtrip_test.py ARKITSCENES_EPISODE --work /tmp/rt   # self-test on a real ARKitScenes episode
python tools/thirdparty_test.py ARKITSCENES_EPISODE --work /tmp/tp  # Stray / 3D Scanner reader test
```

Inputs are auto-detected by the reader registry in `tools/readers/__init__.py` (`r2s`, `strayscanner`,
`3dscannerapp`, `arkitscenes`; an Android reader can register itself with one line). Conversions:

* **Poses:** ARKit camera-to-world (y-up world, camera looks down −z) → ARKitScenes world-to-camera in a
  z-up world with an OpenCV camera: `p_zup = Rx(+90°)·p`, `R_cv = R_arkit·diag(1, −1, −1)`, then inverted
  and written as axis-angle. Traj lines sit exactly on frame timestamps (Apple's loader keys poses by
  `f"{round(ts,3):.3f}"` and drops frames without a pose). Default 10 Hz; `--traj-hz 0` writes every frame.
* **Intrinsics** are scaled from the capture resolution to 256×192 (and 640×480 / full res).
* **Mesh:** ARKit mesh → z-up PLY in the ARKitScenes layout, vertex colours from ~120 posed frames with a
  LiDAR depth visibility test. Without a mesh (Stray Scanner), a 2 cm voxel point cloud is fused from
  high-confidence depth.
* **Boxes:** RoomPlan object `transform` + `dimensions` → `obbAligned` (rows = box axes `[x, −z, y]` in z-up,
  lengths `[w, d, h]`); RoomPlan categories map to ARKitScenes classes (storage→cabinet,
  television→tv_monitor, washerDryer→washer…); unmapped ones (stairs) are listed in the report.
* **sky_direction** comes from the average gravity direction in the image.

## 7. What was verified

All runs were on fleet-3090 (`/root/real2sim-claude/src/capture`, outputs in `/root/real2sim-claude/work/`),
using the real ARKitScenes episode **40753679** (Training, 3,391 frames, 56 s).

**Converter round trip** (`roundtrip_test.py`): ARKitScenes → our raw format (ARKit axes, deflate blobs,
y-up mesh, RoomPlan-style boxes) → ARKitScenes again, compared with Apple's files:

| check | result |
|---|---|
| original traj timestamps present (every-frame mode) | 560 / 560 |
| pose error vs Apple's traj | max 2.7e-6° rotation, 4.0e-6 mm translation (float precision) |
| lowres RGB, depth, confidence, intrinsics (50 sampled frames) | bit-identical / identical |
| mesh vertices and faces | identical (0.0 mm) |
| 3D boxes (5: table, tv_monitor, 3 chairs) | centroid / size error 0.0 mm, axes 2e-16 |
| mesh re-colouring by projection (Apple's Faro colours as reference) | 92% of vertices coloured, mean abs error 12 / 255 (54 / 255 for shuffled colours) |
| loader check (frames with a matching traj key) | 3,355 / 3,391 frames (the rest lie outside the traj time span) |
| video path (RGB through a lossy `.mov`) | mean abs RGB error 2.7 / 255, so frame indices line up |
| `metadata.csv` row | `40753679,NA,Up,Training,False,False,True`, identical to Apple's |
| LiteReality output | 559 frames at 10 Hz, 557 depth + 557 conf, OBJ |

**Readers for other apps** (`thirdparty_test.py`): fake Stray Scanner (`rgb.mp4`, `odometry.csv`,
`camera_matrix.csv`, depth/confidence PNGs) and 3D Scanner App (`frame_*.jpg/json`, depth/conf) exports
built from 60 frames of the same episode, following each app's documented layout, are auto-detected and
converted. Poses match Apple's traj to 1.7e-6° / 2e-6 mm, all 60 depth frames are kept, and a point
cloud is fused (neither export has a mesh here). These are synthetic exports; real app files weren't
available.

**sky_direction** from gravity matches Apple's `metadata.csv` for 7 of 10 sampled videos (all Left /
Right / Up cases). The 3 mismatches are labelled "Down" by Apple, but their images and poses are
upright (checked visually), so that label doesn't describe the lowres images.

**Mode C pose recovery** (`recover_poses.py`, tested on the same episode using ARKit poses as reference;
gravity from the ARKit pose stands in for the IMU; error measured after removing the free yaw +
translation):

| method / input | registered | trajectory RMSE | notes |
|---|---|---|---|
| SfM (pycolmap) + LiDAR scale, 640×480 @ 3 fps | 48 / 168 (largest piece) | 2.7 cm | scale off by only 0.3%, rotation 0.8°; this low-texture meeting room splits into pieces |
| RGB-D odometry (KLT + LiDAR + PnP + IMU tilt), 256×192 @ 10 fps, first 20 s | 195 / 195 | 12 cm on a 2.2 m path | ~5% drift |
| RGB-D odometry, whole 56 s | 559 / 559 | 0.8 m on a 12 m path | drifts on blank walls / fast turns; no loop closure |

So mode C poses are approximate. If a dataset needs reliable poses, record mode A/B, and use mode C as
an extra pass for the other cameras.

**Not verified:** the iOS app has not been compiled or run (no Mac here; the first CI run is the compile
check), and none of the sideloading routes were tried. Converting real 3D Scanner App / Stray Scanner /
R2S exports is covered only by the shared code path and the format documentation, not by real files.

## 8. Capture tips

* Walk slowly (≤ 0.5 m/s), keep 0.5–3 m from surfaces (LiDAR range ~5 m), and avoid pure rotations in
  place. Start by pointing at a textured area for 2 s so tracking starts as "normal".
* Cover the room in one loop and return to the start (ARKit relocalises and corrects drift). For
  RoomPlan, sweep walls at chest height and then tilt down to the floor and furniture; close doors you
  want detected as doors.
* Lighting: turn the lights on, avoid strong backlight, and lock exposure + white balance (Settings) when
  the lighting is even, so frames match across the room.
* Glass, mirrors, black or shiny surfaces give holes in LiDAR depth; the confidence map marks them.
* Each minute of mode A at 1920×1440@60 and 50 Mbps is roughly 375 MB of video + ~130 MB of depth
  (measured on the round-trip test; twice that with smoothed depth on). 4K@30 needs a higher bitrate. Keep the phone cool: thermal state is logged, and hot phones drop frames.
* Allow location for GPS (outdoors / building-level only; it does nothing indoors beyond floor level).
* Validate right after capture: `python tools/validate.py SESSION`.

## 9. Honest limits

* No Faro laser ground truth: ARKitScenes' `highres_depth` and laser point clouds can't be made with a
  phone. Depth is ARKit's 256×192 LiDAR (or the denser AVFoundation map in mode C).
* ARKit owns the back wide camera during world tracking, so ultrawide / tele / front video can't be
  recorded **at the same time** as ARKit poses: that is why mode C exists, and why its poses must be
  recovered offline.
* RoomPlan boxes are automatic, not human annotations; ARKitScenes classes shelf and stool have no
  RoomPlan equivalent.
* The IMU rate is capped by iOS (requests of 200 Hz usually yield ~100 Hz).
* The app is unbuilt and untested on a device; expect a round of compile fixes on the first CI run.
