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
| **B: Camera2 multi-cam** (no live preview) | logical rear camera with every physical lens the HAL streams at once (one HEVC per lens), RAW DNG every 1 s, full CaptureResult per frame, ToF `DEPTH16` if exposed | no (recover offline) | ToF DEPTH16 only if Xiaomi exposes it | `cams/*`; run `recover_poses.py` first |
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

### Live view while recording

* **Mode A** draws what has been captured so far on top of the camera image: a 3D point cloud of the
  ARCore depth (coloured by height above the floor), a top-down occupancy map with your path
  (top right), and a HUD with frames, depth, points, mapped area, distance walked, speed and a
  72-beam virtual lidar of the nearest surfaces. Long-press the map to hide or show the 3D points.
  The map is saved with the session (`extras/map/`) and under `maps/` for navigation mode.
* **Mode B** has no ARCore, so it shows a coverage panorama instead: a yaw × pitch grid filled in by
  how long the phone pointed each way (from the game rotation vector), plus live per-lens frame,
  RAW, ToF and capture-result counts.

### Navigation mode (phone on a robot)

The **Navigate** button opens a separate screen for driving a two-wheeled robot with the phone as its
only sensor. The receiver that runs on the robot and the shared protocol (same as the iPhone app)
are in [`robot/`](../robot/README.md).

1. **Map.** Walk or drive around; ARCore depth builds a 5 cm occupancy grid and a point cloud. Or press
   **Load map** to bring back a map saved by mode A or by an earlier navigation run, then **Align**
   (scan matching of the live map against the saved one) or **Same start** (you started where the
   saved map started).
2. **Goal.** Tap the top-down map, or tap the floor in the camera view. A* plans a path that keeps the
   robot radius (default 20 cm) away from obstacles and replans every 0.5 s as the map changes.
3. **Guidance.** The path and goal are drawn in 3D and on the map; a big arrow and text give the next
   instruction ("turn left 40 degrees", "go straight 1.2 metres"), spoken aloud (Voice) with parking
   style beeps as obstacles get closer than 1.2 m (Beeps).
4. **Drive.** Connect to the robot (`192.168.1.20`, or `192.168.1.20?token=abc`) and switch on
   **Auto-drive**: the phone sends `vel` commands at 10 Hz (pure pursuit), slows near obstacles,
   stops when something is within 15 cm of the robot's front or ARCore tracking is lost, and says
   "Goal reached". **STOP** latches an e-stop on the robot until **Reset e-stop**.
5. **Manual commands.** "Move X cm at V cm/s" and "Turn A° at W°/s", with the phone measuring how far
   it actually moved and turned.

The HUD shows phone and robot speed on a dial, distances front / left / right with bars, the polar
virtual lidar, robot link state, round-trip time and battery, map size and tracking state.
Settings: phone mount height, robot radius, obstacle height, max speed. Each run logs
`nav/<time>/nav.jsonl` (pose, command, distances at 5 Hz).

### Sessions and playback viewer

**Sessions** (main screen, disabled while recording) lists the recorded sessions, newest first, with
mode, duration, size and frame count. Each one has **Open**, **Upload** (the receiver URL field is the
same setting as on the main screen) and **Delete** (asks first). **Open** plays the session back on
the phone, like the iPhone app's viewer (`ios/R2SCapture/SessionViewer.swift`):

* **Video.** Mode A plays `video.mp4` with the ARCore depth map (raw or smoothed) alpha-blended on
  top. Each depth map is matched to the frame on screen by timestamp (`video.mp4.pts.csv`, then
  `frames.jsonl`). Below the video: frame time, tracking state, position and yaw, intrinsics and
  exposure. Mode B plays every lens video side by side, in sync on the sensor clock (each lens
  starts at its own first timestamp). The ToF depth is shown as an extra tile when there is one,
  with the RAW DNG and ToF counts and each lens's size, codec, K, exposure and ISO at the playhead.
  **Rotate** turns the sensor-oriented image upright. It defaults to the camera's sensor orientation
  and the setting is remembered.
* **3D.** The live-map point cloud (`extras/map/points.ply`, or a `mesh.ply` when one exists)
  coloured by height, the camera trajectory in yellow and the camera at the playhead in red. Drag to
  orbit, pinch to zoom, use two fingers to pan, and double-tap to reset.
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
  confidence. The converter's fused mesh is correspondingly coarser than an iPhone Pro scene mesh.
* **The ToF module** is listed by GSMArena but is probably used only for autofocus. If "Dump cams"
  shows a camera with `depth16=true`, mode B records it. It is not registered to the RGB stream, so it
  stays in extras (it can supply metric scale for SfM after calibration).
* **ARCore uses one camera at a time** (the main wide). The other three rear lenses are only reachable
  in mode B, without on-device poses. Whether HyperOS lets third-party apps stream the ultrawide and
  telephotos concurrently is **unverified**. Xiaomi often restricts aux cameras to whitelisted
  packages, and mode B then falls back to whatever subset `isSessionConfigurationSupported` accepts
  (possibly one lens).
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
