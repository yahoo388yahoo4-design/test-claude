# Real2Sim Capture for Android (Xiaomi 13 Ultra and other ARCore phones)

This is the Android counterpart of the iOS capture app (`../ios/`). It records everything the phone
exposes that helps reconstruction. It writes the **same r2s-capture v1 raw session format**
(`../FORMAT.md`), with `"platform": "android"`, so both phones feed one converter (`../tools/convert.py`),
which produces **ARKitScenes** raw layout and **LiteReality** scan folders. Android-only data goes in
`extras/`.

* `APPS.md`: a comparison of ready-made Android capture apps, and the one to use today if you can't
  sideload this one.
* `app/`: the Kotlin app (Gradle, minSdk 29, targetSdk/compileSdk 36, ARCore 1.56).
* `tools/android_roundtrip.py`: test that fabricates an Android session from an ARKitScenes episode
  and round-trips it through the converter.
* `../tools/readers/android_reader.py`: the converter's Android reader (auto-detected).
* `ci/android-capture.yml`: GitHub Actions workflow that builds the debug APK (copy it to `.github/workflows/`).

> **Verification status (2026-10-04).** The debug APK compiles, and `lintDebug` reports no errors
> (API-level checks included). The app was **run on Android emulators** (API 30 and 34, ARCore 1.56
> emulator build, virtual-scene camera driven over gRPC). That exercised mode A, mode B, the sensor
> logging and the storage paths; see §8. The converter path is tested end to end on a synthetic
> Android session built from ARKitScenes episode 40753686. **It has not been run on a Xiaomi 13 Ultra
> yet.** On the emulator ARCore never got past "initializing" (the virtual camera delivers only about
> 1 new frame every 4 s under software GL), so real tracking, depth, SharedCamera stills, ToF and
> multi-lens streaming can only be confirmed on the phone. Treat the first capture as a smoke test,
> and send back `session.json` plus the "Dump cams" output.

---

## 1. Install (sideload)

1. On the phone, enable developer options: Settings → About phone → tap **OS version** 7 times
   (HyperOS / MIUI). Then go to Settings → Additional settings → Developer options and turn on
   **USB debugging**. On Xiaomi, also turn on **Install via USB** (it needs a Mi account and a SIM).
2. Make sure **Google Play Services for AR** is installed and up to date (Play Store). The 13 Ultra is
   on Google's ARCore supported-device list with the Depth API.
3. Install the APK with either:
   * `adb install -r real2sim-capture-debug.apk` from a computer, or
   * copy the APK to the phone and open it in Files ("install unknown apps" must be allowed for Files).
4. Start **Real2Sim Capture** and grant camera, location (precise) and microphone.
   Location is needed for GNSS raw measurements, and the microphone only for the optional audio track.
   On first start the app also opens the **"All files access"** screen. Allow it, so sessions are
   written to `/sdcard/Real2SimCapture/sessions/`, where `adb pull` always works. Without it they go
   to the app's private `Android/data/...` folder, which adb could **not** read on the test emulators
   (Android 11+ may block it). Then use the Wi-Fi upload instead.
5. Press **Dump cams** once. It writes `camera_inventory_<model>.json` and shows every camera id
   (including hidden ones), its physical lenses, RAW / DEPTH16 support and the concurrent-camera sets.
   That file decides what mode B can do on your unit.

Build it yourself (Linux, no root): see §7.

## 2. Modes

| Mode | What runs | Poses on device | Depth | Output |
|---|---|---|---|---|
| **A: ARCore RGB-D** (default, best for reconstruction) | ARCore session on the main wide camera, best 4:3 CPU image config, Depth API `AUTOMATIC` (raw + smoothed), HDR light estimation, planes, point cloud, optional Geospatial / SharedCamera hi-res stills / ARCore Recording API mp4 | yes, every frame | ARCore raw depth + confidence, smoothed depth | full r2s session; convert straight to ARKitScenes + LiteReality |
| **B: Camera2 multi-cam** (live preview of the main lens) | logical rear camera with every physical lens the HAL streams at once (one HEVC per lens, stepped down to fit the hardware budget), RAW DNG every 1 s, full CaptureResult per frame, ToF `DEPTH16` if exposed | no (recover offline) | ToF DEPTH16 only if Xiaomi exposes it | `cams/*`; run `recover_poses.py` first |
| **Sensors only** | IMU / GNSS / baro / status only | – | – | sensor logs |

The IMU (accelerometer, gyro, uncalibrated variants, magnetometer, rotation vectors) runs at the HAL
maximum (`SENSOR_DELAY_FASTEST` plus `HIGH_SAMPLING_RATE_SENSORS`). Also recorded: barometer,
fused/GPS/network location, raw GNSS measurements, NMEA, GNSS status, thermal status, thermal headroom
and battery at 1 Hz, `clock.csv`, and optional audio. All of it is on the **same clock**,
`CLOCK_BOOTTIME` (`elapsedRealtimeNanos`). That is also the clock of camera timestamps when
`SENSOR_INFO_TIMESTAMP_SOURCE = REALTIME`, which the app records per camera.

Options (checkboxes):

* **hi-res stills** (mode A): uses the ARCore SharedCamera API to add the camera's largest 4:3 YUV
  stream (≤4096 px wide, e.g. 4000×3000 binned from the 50 MP IMX989). Every 500 ms the app issues one
  capture request that feeds both ARCore and the hi-res reader. The still therefore has the **same
  sensor timestamp as an ARCore frame**, so it gets that frame's pose exactly. Output:
  `hires/hires_<t>.jpg` + `.json` (T, K scaled from the ARCore intrinsics). It is off by default
  because it costs frame rate. Turn it on for texture quality.
* **ARCore mp4** (mode A): the ARCore Recording API writes `extras/arcore_recording.mp4`, a replayable
  dataset with all of ARCore's internal tracks (camera, IMU). Keep it as a fallback: playing it back
  through ARCore on a phone reproduces tracking.
* **RAW DNG** (mode B): `RAW_SENSOR` via `DngCreator` on the first stream's lens, 1 per second.
* **lock AE/AF/AWB**: mode A sets ARCore focus to FIXED (plus AE/AWB lock with hi-res on). Mode B
  auto-exposes for 1.5 s, then locks AE/AWB and fixes the focus distance. Use it for photogrammetry.
* **OIS off** (mode B): sets `LENS_OPTICAL_STABILIZATION_MODE_OFF` so intrinsics stay rigid. EIS and
  distortion correction are always off in mode B, and OIS samples are recorded when the HAL provides
  them.
* **Geospatial** (mode A): needs a build with `-PARCORE_API_KEY=<Google Cloud key with ARCore API>`.
  Otherwise it is skipped. It records the Earth/VPS camera pose (lat/lon/alt/heading quaternion and
  accuracies) per frame.

### Screens (laid out like the iPhone app)

The UI follows the iOS app (`ios/R2SCapture/ContentView.swift`, `SessionViewer.swift`,
`Nav/NavModeView.swift`): dark, full-bleed camera, translucent rounded cards, iOS blue / red / green,
tabular digits for numbers. Shared pieces are in `Ui.kt` (segmented control, record button, round icon
buttons, bottom sheets with inset-grouped forms, swipeable list rows) and `NavWidgets.kt` (radar, speed
gauge, distance chips).

* **Main screen.** Status card on top (with live stats while recording) and a red timer capsule while
  recording; at the bottom the segmented mode picker (RGB-D / Multi-cam / Sensors), the mode's full title,
  and **Sessions** · **Record** (white ring, red disc that turns into a rounded square while recording) ·
  **Settings** · **Nav**. While idle the camera is live, as on iOS (`IdlePreview.kt`, nothing is
  written): mode A runs a preview-only ARCore session, modes B and sensors (and phones without ARCore)
  a plain Camera2 preview of the main rear camera. It is closed right before a recorder opens the
  camera and restarted after the recording stops; it also stops when the screen is left.
* **Settings sheet.** Mode, Capture (audio, full-res stills), Depth and poses (ARCore recording),
  Camera (lock AE/AF/AWB, RAW DNG, OIS off, camera inventory), Location (Geospatial), Upload receiver
  (URL, upload last session). Saved at once in SharedPreferences (`CaptureSettings.kt`).
* **Sessions.** Large title, one inset-grouped list (name, mode · duration · size · frames). Tap to open,
  swipe left for **Upload** / **Delete**, long-press for the same as a menu. The receiver URL is the
  Settings one (tap "Receiver" to change it).
* **Viewer.** Back button and inline title, segmented tabs (Video / 3D / Map / Sensors / Info),
  transport bar (play / pause, scrubber, time, speed) at the bottom.
* **Navigation.** Top bar (close, tracking, map, robot link, settings), instruction banner with a
  direction arrow, speed gauge and stats, voice button, and one bottom panel: radar + minimap, distance
  chips, Guide / Auto / Manual, joystick + move / turn steppers in Manual, and go-to · GO (Auto) · STOP ·
  save map. Map loading / aligning, the robot link, E-stop, robot and safety parameters and guidance
  toggles are in the settings sheet (parameters apply when it closes).

`./gradlew testDebugUnitTest` also renders these screens off-device with Robolectric
(`UiScreenshotTest`) to `app/build/screenshots/`.

### Live view while recording

* **Mode A** draws what has been captured so far on top of the camera image: the live TSDF mesh
  (shaded, coloured by height above the floor, refreshed about once a second; see
  [Depth filtering and 3D reconstruction](#depth-filtering-and-3d-reconstruction)), a top-down occupancy
  map with your path (top right), and a HUD with frames, depth, mesh size, mapped area, distance walked,
  speed and a 72-beam virtual lidar of the nearest surfaces. Long-press the map to cycle
  mesh → mesh + filtered depth (turbo colours, rejected pixels transparent) → depth only → camera only.
  The map is saved with the session (`extras/map/`) and under `maps/` for navigation mode.
* **Mode B** shows the main lens live, with a coverage panorama on top: a yaw × pitch grid filled in by
  how long the phone pointed each way (from the game rotation vector), plus live per-lens frame,
  RAW, ToF and capture-result counts. The preview never disappears: it is an extra preview stream
  on the main lens when the HAL accepts one (`preview_kind: "preview_surface"`); otherwise the
  recorded main stream itself is shown through surface sharing, which costs no extra camera stream
  (`"recorded_stream"`). Only if neither can be configured is it `"none"`.
* **Mode B hardware budget** (like the iOS app): if the HAL refuses the requested lenses at 1920 px /
  30 fps (`isSessionConfigurationSupported`, or a failed session / encoder), the app steps down to
  1280 px, then 24 fps, then drops the lowest-priority lens one at a time until a configuration
  works. The status line shows what it changed (`budget: reduced to 1280 px, dropped tele_4`), and
  `session.json` records it under `android.multicam` (`budget_actions`, `budget_requested`,
  `preview_kind`, `preview_size`, per-stream `w`/`h`/`fps`) and in a top-level `multicam` block in the
  iOS shape (`budget_actions`, `preview`, `streams[].width/height/fps`).

### Navigation mode (phone on a robot)

The **Nav** button opens a separate screen for driving a two-wheeled robot with the phone as its
only sensor. It talks to the robot with the same protocol as the iPhone app
([`robot/PROTOCOL.md`](../robot/PROTOCOL.md)), so the same receivers work with both phones:
`robot/receiver.py` (Python: sim, serial, Raspberry Pi GPIO), `robot/esp32_diffdrive` (ESP32 with Wi-Fi
and BLE) and `robot/serial_motor` (Arduino bridge). See [`NAVIGATION.md`](../NAVIGATION.md).

1. **Map.** Walk or drive around; ARCore depth builds a 5 cm occupancy grid and a point cloud. Or press
   **Load map** (settings sheet) to bring back a map saved by mode A or by an earlier navigation run, then **Align**
   (scan matching of the live map against the saved one) or **Same start** (you started where the
   saved map started).
2. **Goal.** Tap the top-down map, or tap the floor in the camera view. A* plans a path that keeps the
   robot radius away from obstacles and replans every 0.5 s as the map changes.
3. **Guide / Auto / Manual.** *Guide*: arrow, text and voice ("turn left 40 degrees", "go straight
   1.2 meters"), parking-style beeps under 1.2 m and haptics; nothing is sent to the robot. *Auto*: the
   phone drives the robot along the path with `vel` commands at 10 Hz, slows between the slow and stop
   distances, backs up when blocked for 3 s and the way behind is clear, and says "Goal reached".
   *Manual*: an on-screen joystick (with the same front safety stop).
4. **Moves and turns.** "▲ Fwd / ▼ Back X cm at V cm/s" and "⟲ Left / Right ⟳ A° at W°/s". With
   *Moves closed-loop (ARCore)* on (default) the phone measures the motion and streams velocities, so a
   robot without encoders still moves the right distance; off, `move` / `turn` go to the robot as is.
5. **Link.** None, USB: Neato, USB: OpenBot (USB-OTG cable straight into the robot; Android asks for
   permission the first time; ported from OSSDC VisionAI Mobile), Wi-Fi (`ws://192.168.4.1:8777/robot` for the ESP32 access point, or the URL
   `receiver.py` prints) or Bluetooth LE UART (device name prefix `R2S-Robot`). **STOP** stops at once,
   **E-stop latch** / **Release** latch the robot's emergency stop, and an `estop` from the robot (bumper,
   button) cancels whatever the phone was doing. Tracking loss also stops the robot.

The HUD shows phone and robot speed on a dial, distances front / left / right / rear with bars, the
polar virtual lidar, the command being sent, robot link state, round-trip time, battery and wheel
speeds, map size and tracking state. Settings (saved): phone mount height and how far ahead of the
turning centre it sits, robot radius and height, max speed and turn rate, slow / stop distances, goal
tolerance. Each run logs `nav/<time>/nav.jsonl` (pose, command, distances at 5 Hz).

**Voice.** The round mic button takes spoken commands; long-press it for voice settings. By default it
uses the phone's own speech recogniser (on-device when the phone has one), Gemini Nano on phones that
support it (else the built-in command matcher) and the phone's text-to-speech. A private local server
(Whisper + Piper over Wyoming, Ollama) can replace any part. See [`VOICE.md`](../VOICE.md).

### Sessions and playback viewer

**Sessions** (main screen, disabled while recording) lists the recorded sessions, newest first, with
mode, duration, size and frame count. Tap one to open it; swipe left (or long-press) for **Upload** (to
the receiver URL from Settings) and **Delete** (asks first). Opening it plays the session back on
the phone, like the iPhone app's viewer (`ios/R2SCapture/SessionViewer.swift`):

* **Video.** Mode A plays `video.mp4` with the ARCore depth map (raw or smoothed) alpha-blended on
  top. Each depth map is matched to the frame on screen by timestamp (`video.mp4.pts.csv`, then
  `frames.jsonl`). Below the video: frame time, tracking state, position and yaw, intrinsics and
  exposure. Mode B plays every lens video side by side, in sync on the sensor clock (each lens
  starts at its own first timestamp). The ToF depth is shown as an extra tile when there is one,
  with the RAW DNG and ToF counts and each lens's size, codec, K, exposure and ISO at the playhead.
  **Rotate** turns the sensor-oriented image upright. It defaults to the camera's sensor orientation
  and the setting is remembered.
* **3D.** The reconstructed mesh (`mesh.ply`, else `extras/recon/mesh.ply`, else the cleaned points
  in `extras/map/points.ply`), lit and coloured by height (**Colours** switches to the PLY's camera
  colours), the camera trajectory in yellow and the camera at the playhead in red. Drag to orbit, pinch
  to zoom, use two fingers to pan, and double-tap to reset. The video tab's depth overlay shows the
  filtered depth (raw depth with its confidence) in turbo colours with rejected pixels transparent.
* **Map.** The top-down occupancy map (`extras/map`), with the trajectory and an arrow for the camera
  at the playhead.
* **Sensors.** Charts of accelerometer, gyro, magnetometer, the fused IMU (user acceleration,
  gravity, orientation, heading), barometer and relative altitude, GNSS speed, accuracy, altitude and
  satellites, thermal state, battery level and temperature, thermal headroom, camera exposure / ISO,
  and every other sensor in `extras/sensors_raw.csv` (light, proximity, temperature, humidity,
  rotation vectors, …). A cursor follows the playhead, and tapping a chart seeks there. Sessions
  without video (sensors only) get a virtual clock over the recording, so they can be played too.
* **Info.** `session.json`, pretty-printed, and every file with its size.

The transport bar (play/pause, scrubber, 0.25×–4× speed) drives all tabs. Missing files are skipped,
and a tab with nothing to show explains why. For example, mode B has no on-device poses, so it has no
3D or map.

### Depth filtering and 3D reconstruction

Depth-from-motion is ~160×120, noisy and smeared across silhouettes, so nothing uses it unfiltered.
`DepthFusion` takes each new depth map off the GL thread (latest frame wins; recording never waits):

1. **Filter** (`DepthFilter.kt`, pure Kotlin): raw depth + confidence when ARCore delivers it
   (confidence < 0.5 dropped), else smoothed depth; range 0.2–4 m; a 3×3 median over same-surface
   neighbours (relative difference < 12 %), pixels with < 3 such neighbours dropped as speckle;
   **flying pixels** (a jump > 6 % towards both a nearer and a farther neighbour) dropped; surfaces seen
   at > 80° from their normal dropped; a pixel the previous frame saw *through* (free-space violation
   > 10 %) dropped. Settings and per-rule pixel counts go to `session.json` → `depth_processing`.
2. **Occupancy / virtual lidar** (`MapBuilder`) from the filtered dense smoothed depth (keeps blank
   walls that raw depth drops). Rays now only clear cells where they pass below 0.6 m, so a low box
   seen from chest height is not erased by rays to the wall behind it.
3. **TSDF fusion** (`TsdfVolume.kt`) at ≤ 10 Hz: 3 cm voxels in 8³ blocks (voxel hashing), truncation
   4 voxels (12 cm), depth-dependent weights, running average capped at 64, free-space carving through
   existing blocks, colour from the camera image (sampled at depth resolution), ≤ 8000 blocks (~45 MB).
   ~4 ms filter + ~7 ms integration per 160×120 frame on a desktop JVM.
4. **Mesh** by marching cubes (generated, crack-free case table; vertices shared per grid edge,
   area-weighted normals). Live: dirty blocks re-extracted about once a second. At stop: full extraction,
   connected components under 120 triangles dropped (noise blobs), dominant planes (floor, walls) found
   by RANSAC and the rest split into connected **objects** (bounding boxes in `objects.json`), and a
   cleaned point set (one mesh vertex per voxel, statistical outliers removed).

Outputs (mode A): `extras/recon/mesh.ply` (binary LE, normals, camera colours or height colours),
`extras/recon/points.ply`, `extras/recon/objects.json`, `extras/recon/recon.json`; `extras/map/points.ply`
is now the cleaned TSDF surface points (occupancy, trajectory and `map.json` unchanged). Navigation mode
runs the same filter + fusion for its occupancy grid and live mesh. The converter applies the same
confidence / flying-pixel rules in numpy (`tools/depth_filter.py`); `--no-depth-filter` keeps the
recorded depth bit-identical.

Limits: tuned and tested on synthetic scenes (`DepthFusionTest`: a box on a floor before a wall with
noise, smeared edges and speckle → every vertex within one voxel, box as its own object), not yet on
the phone. Thin structures (< 2 voxels, chair legs, cables) disappear, surfaces beyond 4 m are not
fused, the volume stops growing at 8000 blocks (`volume_full` in `recon.json`), and the mesh lives in
one ARCore session's frame (no loop closure: drift shows up as doubled surfaces).

## 3. Capture tips (13 Ultra)

* **Mode A first.** It is the only mode with on-device poses and depth, and the converter turns it
  directly into both formats.
* ARCore depth is **depth-from-motion**, so keep moving sideways (translate, don't just rotate),
  ~0.5–4 m from surfaces, slowly (≤0.5 m/s). Start by sweeping the room for ~5 s until the status
  shows `normal` tracking. Pass each wall twice at different heights, and close the loop back at the
  start.
* Light the room evenly. Avoid large blank walls and mirrors, which break both tracking and depth.
* Hold the phone in **portrait** with the main (1") camera unobstructed. The app records in sensor
  orientation either way.
* For texture-quality stills, enable **hi-res stills** and slow down.
* Keep the phone cool. `status.csv` and `extras/device_status.csv` show thermal throttling. Long
  sessions at 30 fps HEVC plus depth use roughly 0.5–1 GB/min (depending on resolution).
* Mode B: run **Dump cams** first. If only one physical lens streams, Xiaomi is restricting
  third-party access (see limits). Lock exposure/focus and turn OIS off for SfM.

## 4. Getting data off the phone

* **USB:** `adb pull /sdcard/Real2SimCapture/sessions/ ./sessions/` (with "All files access"). The
  status line shows the folder in use. If it is `.../Android/data/com.real2sim.capture/files/sessions`
  and `adb pull` reports "Permission denied", grant All files access in Settings → Apps → Real2Sim
  Capture, or upload over Wi-Fi.
* **Wi-Fi:** run the shared receiver on fleet-3090 or a laptop,
  `python tools/receiver.py --root ~/captures --port 8765 --token SECRET [--convert OUT]`. Then in the
  app enter `http://<host>:8765#SECRET` and press **Upload last**, or **Upload** next to any session under **Sessions**. It is the same protocol as the iOS
  app: per-file `PUT /upload/<session>/<path>`, a HEAD check so a restarted upload resumes, and a
  `.complete` marker at the end. To reach fleet-3090 from outside the LAN, use an SSH tunnel
  (`ssh -L 8765:localhost:8765 ...` from a laptop on the same Wi-Fi as the phone).

* **Crash logs:** if the app crashes, the stack trace, time, device, app version and the mode that was
  recording go to `crash_logs/crash_<unix time>.txt` next to the `sessions/` folder
  (`/sdcard/Real2SimCapture/crash_logs/` with All files access), so `adb pull` or a USB file browser
  finds them. The app still crashes normally afterwards. `session.json` is written with NaN /
  infinite numbers as `null` (org.json would throw on them), and a failing writer no longer takes
  the recording down.

## 5. Convert

```bash
cd capture/tools
python convert.py /path/to/20261004_181500_arcore_rgbd --to arkitscenes litereality --out OUT   # reader "android" auto-detected
python validate.py OUT/arkitscenes/<video_id>
python validate.py OUT/litereality/<session>
# mode B: pose first, then convert the posed session
python recover_poses.py /path/to/..._multicam --cam wide_2 --fps 5 --out posed
```

## 6. Where every sensor lands

| Xiaomi 13 Ultra hardware | Mode | ARKitScenes | LiteReality | raw session / extras |
|---|---|---|---|---|
| Main camera 50 MP 1" IMX989, 23 mm, OIS | A (ARCore CPU image, best 4:3 config) | `lowres_wide/` 256×192, `.pincam`, (`--wide` full-res, `--vga`) | `frame_XXXXX.jpg` + `.json` (`cameraPoseARFrame`, `intrinsics`) | `video.mp4` (HEVC), `frames.jsonl` (`T`, `K`, `exp`, `iso`, `amb`), `extras/arcore_frames.jsonl` (display pose, texture K, focus/aperture/rolling-shutter skew, HDR light) |
| Main camera, full-res stills | A + hi-res | `--wide` source | extra frames for texturing | `hires/hires_<t>.jpg/.json`, `extras/hires_capture_results.jsonl` |
| ARCore 6-DoF tracking (camera + IMU VIO) | A | `lowres_wide.traj` (z-up world, world-to-camera axis-angle) | `cameraPoseARFrame` (y-up ARKit axes) | `frames.jsonl T` |
| ARCore raw depth + confidence (depth-from-motion) | A | `lowres_depth/` (mm), `confidence/` (0..2) | `depth_XXXXX.png`, `conf_XXXXX.png` | `depth.zlib.bin`, `conf.zlib.bin`, `extras/conf255.zlib.bin` (full 0..255) |
| ARCore smoothed depth | A | with `--smoothed-depth` | with `--smoothed-depth` | `depth_smooth.zlib.bin` |
| Fused geometry (from depth + poses) | A | `<vid>_3dod_mesh.ply` (converter fuses; ARCore has no mesh) | `textured_output.obj` (fused, vertex-coloured) | – |
| ARCore planes, feature point cloud | A | – | – | `planes.json`, `extras/planes_timeline.jsonl`, `extras/pointcloud/*.bin` |
| ARCore Geospatial / VPS | A + key | – | – | `extras/arcore_frames.jsonl` `geo` |
| ARCore Recording API | A + option | – | – | `extras/arcore_recording.mp4` |
| Ultrawide 50 MP IMX858 12 mm | B (if exposed) | after `recover_poses.py` | after `recover_poses.py` | `cams/ultrawide_<id>.mp4/.jsonl` |
| Tele 50 MP IMX858 75 mm (3.2×) | B (if exposed) | ″ | ″ | `cams/tele_<id>.*` |
| Periscope 50 MP IMX858 120 mm (5×) | B (if exposed) | ″ | ″ | `cams/tele2_<id>.*` |
| RAW sensor (DNG) | B | – | – | `cams/raw/<name>_<t>.dng` |
| Lens intrinsics, distortion, extrinsics, OIS samples | B (A: characteristics only) | – | – | `cams/calibration.json`, per-frame `lens_*`, `ois_samples` in `cams/*.jsonl`, `extras/camera_characteristics_arcore_<id>.json` |
| ToF 3D depth module (GSMArena lists one; exposure to apps unverified) | B, if `DEPTH16` is exposed | – (not registered to RGB) | – | `cams/tof_depth.zlib.bin`, `cams/tof_conf.zlib.bin`, `cams/tof_depth.jsonl` |
| Accelerometer, gyro (calibrated + uncalibrated), magnetometer | all | – | – | `accel.csv`, `gyro.csv`, `mag.csv`, `imu.csv` (FORMAT.md units: g, iOS sign), `extras/sensors_raw.csv` (Android SI units, every event) |
| Rotation vector / game rotation / gravity / linear accel | all | – | – | `imu.csv` (quat, gravity, user accel, heading), `extras/sensors_raw.csv` |
| Barometer (if present) | all | – | – | `altimeter.csv` (rel. alt, kPa) |
| Dual-band GNSS (L1+L5) | all | – | – | `location.csv`, `extras/gnss_raw.csv` (pseudoranges, carrier phase), `extras/nmea.csv`, `extras/gnss_status.csv` |
| Thermal / battery | all | – | – | `status.csv`, `extras/device_status.csv` |
| Microphones | all (option) | – | – | `audio.m4a` |
| – | – | `metadata.csv` (video_id, sky_direction, ...) | `pointcloud.pcd`, `extras/` | `session.json`, `clock.csv` |

The converter copies everything that has no slot in ARKitScenes or LiteReality to `<out>/extras/`
(Android-only files under `extras/android/`), together with `poses_all_frames.csv`.

## 7. Build

```bash
# one-time local toolchain (no root; ~1.5 GB), here under $T
T=$HOME/android-toolchain; mkdir -p $T && cd $T
curl -sLo jdk.tgz https://github.com/adoptium/temurin17-binaries/releases/download/jdk-17.0.20.1%2B1/OpenJDK17U-jdk_x64_linux_hotspot_17.0.20.1_1.tar.gz
mkdir jdk17 && tar xzf jdk.tgz -C jdk17 --strip-components=1
curl -sLo clt.zip https://dl.google.com/android/repository/commandlinetools-linux-16111833_latest.zip
mkdir -p sdk/cmdline-tools && unzip -q clt.zip -d sdk/cmdline-tools && mv sdk/cmdline-tools/cmdline-tools sdk/cmdline-tools/latest
export JAVA_HOME=$T/jdk17 ANDROID_HOME=$T/sdk PATH=$T/jdk17/bin:$T/sdk/cmdline-tools/latest/bin:$PATH
yes | sdkmanager --licenses >/dev/null; sdkmanager "platforms;android-36" "build-tools;36.0.0" "platform-tools"
# build
cd capture/android && ./gradlew assembleDebug lintDebug     # -> app/build/outputs/apk/debug/app-debug.apk
```

Versions: AGP 8.13.2, Gradle 8.14.3 (wrapper), Kotlin 2.2.20, JDK 17, ARCore 1.56.0. The debug APK is
about 10.2 MB, so `dist/` holds it as ≤8 MB parts. Rebuild it with
`cat dist/real2sim-capture-debug.apk.part* > real2sim-capture-debug.apk && sha256sum -c dist/real2sim-capture-debug.apk.sha256`.
The GitHub Actions workflow (`ci/android-capture.yml`) builds the same APK and uploads it as an artifact.

## 8. Converter round trip (synthetic Android session)

`tools/android_roundtrip.py` takes ARKitScenes episode 40753686 (60 key frames, iPad LiDAR depth) and
writes it exactly as the app's ARCore mode would. That means HEVC 4:2:0 `video.mp4` + pts file,
raw-deflate depth, confidence through ARCore's 0..255 scale and the app's quantisation, smoothed
depth, `extras/arcore_frames.jsonl`, and sensor logs in FORMAT.md units. To mimic ARCore skipping
depth, **every 7th frame has no depth**. The script then converts with `convert.py` (reader detected:
`android`) and compares against the original episode:

| check (traj every frame and 10 Hz runs identical) | result |
|---|---|
| poses: traj lines matched / rotation / translation error | 60 / 60, max 2.4e-6°, 1.1e-7 mm |
| intrinsics (.pincam) max abs error | 0.0 |
| RGB mean abs error (HEVC 4:2:0 chroma) | 1.45 / 255 (worst frame 1.75) |
| depth / confidence PNGs bit-identical | 52 / 52 frames that had depth (8 deliberately dropped) |
| LiteReality: frames / json / depth / conf / textured_output.obj / extras/android | 60 / 60 / 52 / 52 / yes / yes |
| `validate.py` session / arkitscenes / litereality | PASS / PASS / PASS. WARNs: no mesh in session (ARCore has none; converter fuses), depth 52/60 (by design), no 3dod annotation and no room.usdz (no RoomPlan), traj 1.3 Hz (the sample's 60 key frames span 45 s) |
| multicam reader smoke test (`cams/wide_2.*`, empty frames.jsonl) | detected `android`, 60 unposed frames, video decodes, no depth |

**On the emulator, with the real app** (API 30 image for mode A, API 34 for mode B, since the
API 30 camera HAL emits negative timestamps that MediaCodec drops):

| check | result |
|---|---|
| mode A session: files written | all FORMAT.md files + extras (video.mp4, frames.jsonl, imu/accel/gyro/mag/altimeter/location/status/clock, GNSS raw, NMEA, sensors_raw, camera characteristics, ARCore configs, planes) |
| mode A: repeated `session.update()` frames | found as a bug (ARCore returned 15 distinct frames in 486 updates and every repeat was logged), **fixed**: now 8 distinct frames logged and 370 repeats skipped, strictly increasing pts |
| mode A + "ARCore mp4" | `extras/arcore_recording.mp4` written (1.2 MB / 30 s) |
| mode A tracking / depth | not testable: emulator ARCore stays "initializing" at ~0.25 new frames/s; Depth API unsupported on the emulator |
| mode B (API 34) | HEVC 1280×960, every video sample joined to its CaptureResult (exposure, ISO, frame duration, rolling-shutter skew, focus, crop, AE/AF/AWB states); `K` from focal length / sensor size when the HAL has no `LENS_INTRINSIC_CALIBRATION` (fix added after this test) |
| location clock | found as a bug (provider `elapsedRealtimeNanos` was unix time on the emulator), **fixed** with a sanity check |
| reader + validate.py on these sessions | both auto-detected `android`, video decodes. Mode A PASS; mode B FAIL only on "0 posed" (expected until `recover_poses.py`) |

What these tests do **not** cover: ARCore's real depth quality (depth-from-motion is noisier and
sparser than LiDAR), real MediaCodec output, and device timing.

## 9. Honest limits

* **No LiDAR.** ARCore depth on the 13 Ultra is **estimated from motion** (Google lists the phone with
  the Depth API but not as a ToF-using device). Expect roughly 160×120 depth (the actual size is
  recorded in `session.json`), lower accuracy past ~4 m, holes on textureless surfaces, and nothing
  while standing still. Raw confidence is real but means something different from ARKit's LiDAR
  confidence. The on-phone TSDF mesh and the converter's fused mesh are correspondingly coarser than an
  iPhone Pro scene mesh.
* **The ToF module** is listed by GSMArena but is probably used only for autofocus. If "Dump cams"
  shows a camera with `depth16=true`, mode B records it. It is not registered to the RGB stream, so it
  stays in extras (it can supply metric scale for SfM after calibration).
* **ARCore uses one camera at a time** (the main wide). The other three rear lenses are only reachable
  in mode B, without on-device poses. Whether HyperOS lets third-party apps stream the ultrawide and
  telephotos concurrently is **unverified**. Xiaomi often restricts aux cameras to whitelisted
  packages, and mode B then steps down its budget until the HAL accepts a configuration (possibly
  one lens; see `budget_actions` in `session.json`). Whether surface sharing works with a physical
  lens stream (the preview fallback) on HyperOS is also unverified.
* **The ARCore CPU image is not the full sensor.** Typical configs top out at 1920×1080 or 1440×1080;
  the app prefers 4:3 so the 256×192 ARKitScenes frames are not distorted. Full-sensor detail needs
  the hi-res stills option.
* **No RoomPlan on Android.** LiteReality's `roomplan/room.usdz` is **omitted** for Android captures
  (validate reports a WARN). LiteReality's own pipeline needs it for its layout stage. The path is to
  build it offline from the posed RGB-D: fused mesh → floor/wall plane fit (ARCore `planes.json` gives
  vertical and horizontal planes as seeds) → room.usdz via the existing `build_arkit_scan` tooling on
  fleet-3090. That is not automated yet, and ARKitScenes `3dod_annotation.json` (object boxes) is
  equally absent.
* **Navigation runs on motion-estimated depth.** Without LiDAR, ARCore depth needs the phone to
  move, is sparse on blank walls and glass, and misses thin chair legs and cables; obstacles lower
  than about 6 cm above the floor are ignored on purpose (floor noise). Keep max speed low (≤25 cm/s)
  and stay near the robot with the STOP button. There is no relocalisation against a saved map other
  than **Align** / **Same start**, and the map lives in one ARCore session's frame.
* Mode A's hi-res stills, Geospatial, ToF and multi-lens streaming depend on HAL behaviour that has
  only been compiled against, not exercised on the device.
