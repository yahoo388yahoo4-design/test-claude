# Android capture apps for real2sim (Xiaomi 13 Ultra)

Research date: 2026-10-04. Play Store facts (last-updated dates, download counts, 404s) were scraped from the US Play listings (`hl=en_US&gl=US`) on that date. GitHub facts come from the GitHub API on the same date. Anything marked **unverified** was not confirmed against a primary source.

Targets:

- **(A) ARKitScenes raw**: `lowres_wide/*.png` (256x192 RGB), `lowres_depth/*.png` (uint16 mm), `confidence/*.png`, `*.pincam` intrinsics, `lowres_wide.traj` (world-to-camera, per the task spec), mesh `.ply`.
- **(B) 3D Scanner App "All Data" / LiteReality**: `frame_XXXXX.jpg` + `frame_XXXXX.json` (`cameraPoseARFrame` camera-to-world, `intrinsics`), `depth_XXXXX.png` (uint16 mm), `conf_XXXXX.png`, `textured_output.obj`, and optionally RoomPlan `room.usdz`.

---

## 1. The phone: Xiaomi 13 Ultra

| Question | Answer | Source |
|---|---|---|
| ARCore certified? | **Yes.** "Xiaomi 13 Ultra" is on the Google Play Services for AR supported-devices list. | https://developers.google.com/ar/devices |
| Depth API? | **Yes.** The row's note is "Supports Depth API". | same |
| Raw Depth API? | **Yes, by inference.** "Raw Depth is available on all devices that support the Depth API." | https://developers.google.com/ar/develop/java/depth/raw-depth |
| Geospatial API? | **Probably yes.** The list flags unsupported devices with "Does not support Geospatial API", and the 13 Ultra row has no such flag. Not tested on the device. | https://developers.google.com/ar/devices |
| ToF / depth sensor? | **There is hardware, but ARCore does not use it.** GSMArena lists a fifth rear module "TOF 3D, (depth)", so the assumption of "no ToF" is wrong at the hardware level. However, ARCore's list does **not** mark the 13 Ultra "Supports time-of-flight (ToF) hardware depth sensor", so ARCore depth here is **depth-from-motion only**. Whether the ToF is exposed to apps as a Camera2 `DEPTH16` stream is **unverified**. It is most likely only an autofocus aid. Check on the device with `adb shell dumpsys media.camera \| grep -i -E "depth\|DEPTH16"`. | https://www.gsmarena.com/xiaomi_13_ultra-12236.php |
| LiDAR / RoomPlan | **No.** RoomPlan is an Apple-only framework, so there is no `room.usdz` on Android. | — |
| Third-party access to all 4 cameras via Camera2 | **Unverified.** Xiaomi often exposes only some physical cameras to third-party apps. ARCore always uses the main wide camera. | — |

What ARCore depth actually provides (https://developers.google.com/ar/develop/depth):

- Depth is computed with "a depth-from-motion algorithm", which needs the device to move. It is most accurate from about 0.5 m to 5 m.
- The **full** Depth API (`acquireDepthImage16Bits`) is smoothed and dense, and comes **without** confidence.
- The **Raw** Depth API (`acquireRawDepthImage16Bits` + `acquireRawDepthConfidenceImage`) is sparser and more accurate, and comes **with** a per-pixel confidence image. The ARCore `raw_depth_java` sample uses exactly these two calls.
- Depth resolution depends on the device, commonly around 160x120. The 13 Ultra's depth resolution is **unverified**.
- The CPU image used for tracking is 640x480 (VGA) by default. Higher-resolution CPU image configs exist but cost performance.

---

## 2. App-by-app

### 2.1 RTAB-Map for Android (introlab): the best ready-made posed RGB-D option

- **Android availability:** **Changed.** The Play listing `com.introlab.rtabmap` now returns **404**. The Wayback Machine has it at 200 up to 2024-04-15 and 404 by 2025-04-09, so it was **delisted**. Two ways to install it now:
  - The latest prebuilt APKs are on GitHub release **0.21.4 (2024-02-19)**: `RTABMap-0.21.4-android30.apk` / `-android26.apk` / `-android24.apk` / `-android23-tango.apk`. https://github.com/introlab/rtabmap/releases/tag/0.21.4
  - Newer releases (0.22.1, 0.23.1, 0.23.8 from 2026-07-05) ship no Android APK. You can build one from Docker (`introlab3it/rtabmap:android26` etc.) as described at https://github.com/introlab/rtabmap/wiki/Installation.
  - The iOS app is still live: https://apps.apple.com/app/id1564774365
- **Open source:** yes (BSD-style). The repo is very active, last push 2026-10-04. https://github.com/introlab/rtabmap
- **Camera drivers:** Tango (legacy), ARCore Java/shared-camera, **ARCore NDK**, and Huawei AREngine. Source: `app/android/jni/Camera*.cpp`.
- **What it records** (read from source, `app/android/jni/CameraARCore.cpp`):
  - **RGB:** the ARCore CPU image (YUV to BGR). It picks the **lowest-resolution** camera config (`ArSession_setCameraConfig(... cpu_low_resolution_camera_config ...)`), which typically means 640x480. This exact resolution is **unverified** on the 13 Ultra.
  - **Depth:** off by default. With the "**Depth From Motion**" setting enabled (pref string: "Use ARCore's depth API to compute depth image from motion … Currently supported only with ARCore NDK driver"), it stores `ArFrame_acquireDepthImage`. That is the **smoothed full-depth** image (DEPTH16, mm), not Raw Depth, and has **no confidence**. Without that setting, only ARCore feature points are stored.
  - **Poses:** a per-frame ARCore pose, which becomes RTAB-Map odometry. The graph is then optimized with loop closure.
  - **Intrinsics:** per frame, from `ArCamera_getImageIntrinsics`.
  - **Frame rate stored:** map "Update Rate" defaults to **1 Hz**. Options are Max / 5 / 4 / 3 / 2 / 1 / 0.5 Hz. A "Data Recorder Mode" menu item exists.
  - **Not recorded:** IMU in the exported data (unverified), GPS (optional, unverified), and RAW.
- **Export / getting data off the phone:**
  - On the phone you get a `.db` (SQLite) database, plus mesh / point-cloud export (OBJ+texture or PLY) from the Export menu. Files go to the app's `RTAB-Map/` folder, which you can `adb pull`.
  - On a PC, the `rtabmap-export` tool (in `tools/Export/main.cpp`) provides `--images` (RGB, depth and per-frame `_calib` folders, named by stamp), `--poses_camera` (optimized camera-frame poses, several formats via `--poses_format`), `--mesh --texture` (textured OBJ), and `--cloud` (PLY).
- **Cost:** free.
- **Maps to:**
  - **(A) ARKitScenes: small converter.** Resize 640x480 to 256x192 (same 4:3), resize depth to 256x192 in mm, write `.pincam` from `_calib`, and invert camera-to-world into `.traj` (verify against our existing ARKitScenes loader). Confidence must be **synthesized**, for example 2 where depth > 0 and 0 elsewhere. That is a deviation and should be flagged in run metadata. The mesh comes from `--mesh` as PLY.
  - **(B) LiteReality: small converter.** Write `frame_*.jpg` and json with camera-to-world `cameraPoseARFrame` plus intrinsics, `depth_*.png`, a synthesized `conf_*.png`, and `textured_output.obj` from `--mesh --texture`. RGB is only VGA, so textures will be soft. There is no `room.usdz`.
- **Risks:** the newest APK is from 2024-02, and it has not been confirmed to run on the 13 Ultra with ARCore 1.56 (**unverified**). Depth is smoothed, not raw. Frames are keyframes at the update rate, not every camera frame.

### 2.2 ARCore SDK samples (Google): DIY recorder building blocks

Repo: https://github.com/google-ar/arcore-android-sdk (Apache-2.0). Latest SDK is **1.56.0 (2026-09-04)**. Samples include `recording_playback_java`, `raw_depth_java`, `computervision_java`, `shared_camera_java`, `geospatial_java`, `hello_ar_*`, `hello_eis_kotlin` and `semantics_java`. None are on Play; you build and sideload them.

- **`recording_playback_java`** (`HelloRecordingPlaybackActivity`): writes `arcore-dataset-YYYY-MM-DD-hh-mm-ss.mp4` to app storage.
  - Per https://developers.google.com/ar/develop/recording-and-playback, the MP4 holds the 640x480 CPU image track (or a high-res CPU image if configured), a depth-visualization track (preview only), IMU (gyro + accel), device/SDK metadata, and optional custom data tracks.
  - It does **not** contain poses or metric depth. Those come from **replaying** the MP4 through ARCore on an Android device, and replay may not exactly reproduce live tracking (unverified).
  - Maps to (A)/(B): **not directly.** You would need a playback-and-dump app. It is useful as an archival "raw session" format.
- **`raw_depth_java`**: calls `acquireRawDepthImage16Bits()` and `acquireRawDepthConfidenceImage()`. Together with `computervision_java` (CPU image access) this is everything needed for a recorder that writes **real depth plus real confidence plus per-frame pose and intrinsics**.
- **Note:** this repo already contains an untracked Android project `capture/android/` ("Real2SimCapture", depending on `com.google.ar:core:1.56.0`). That is the natural home for such a recorder.

### 2.3 OpenCamera Sensors (MobileRoboticsSkoltech)

- **Android:** yes. Available on F-Droid (`com.opencamera_sensors.app`) and as an APK from GitHub. https://github.com/MobileRoboticsSkoltech/OpenCamera-Sensors
- **Open source:** GPL-3.0. **Archived**, last push 2022-08-22.
- **Records:** Camera2 video plus accelerometer, gyroscope and magnetometer CSVs (`X,Y,Z,timestamp_ns`), with frame timestamps on the same clock (`_timestamps.csv`). It also has a flash-strobe log and multi-phone RecSync. The IMU rate is the device max (exact value not documented). Video stabilization can be disabled. No poses, no depth, no intrinsics file. RAW is **not** recorded with video (unverified whether the base Open Camera RAW stills still work in this fork).
- **Export:** `DCIM/OpenCamera/` (video + `{VIDEO_DATE}/` CSVs), via `adb pull`.
- **Cost:** free.
- **Maps to (A)/(B): not directly.** It needs offline SfM/VIO (e.g. COLMAP, or an OpenVINS/Basalt-style pipeline) for poses and a monocular depth model for depth. Use it for VIO-grade IMU + video research data, not as the primary scene recorder.

### 2.4 Open Camera (Mark Harman)

- **Android:** yes, Play `net.sourceforge.opencamera`. Updated 2026-05-27, 100M+ downloads. Open source (GPL-3.0). Free; third-party ads only on the website. https://opencamera.org.uk/
- **Records:** photos/video via Camera2 with manual controls, **RAW (DNG)**, burst, focus bracketing, and GPS geotagging (photos, plus SRT subtitles for video). No IMU logs, poses or depth.
- **Export:** `DCIM/OpenCamera` (JPG/DNG/MP4).
- **Maps to (A)/(B): not directly.** Useful only as a high-quality still / DNG source for photogrammetry or texture. It cannot run at the same time as an ARCore session, because ARCore owns the camera.

### 2.5 MARS logger (OSUPCVLab mobile-ar-sensor-logger)

- **Android:** yes (APK/build from GitHub). https://github.com/OSUPCVLab/mobile-ar-sensor-logger
- **Open source:** GPL-3.0. Last push 2022-01-05, effectively unmaintained.
- **Records (Android):** H.264 video at ~30 Hz via Camera2, frame timestamps, per-frame metadata including focal length in pixels and exposure, and IMU at ~100+ Hz. No ARCore poses on Android (the iOS version uses ARKit). No depth. GPS is listed as future work.
- **Export:** MP4 + CSV/TXT on device storage.
- **Maps to:** same as OpenCamera Sensors, **not directly**.

### 2.6 Sensor Logger (Kelvin Choi / tszheichoi)

- **Android:** yes, Play `com.kelvin.sensorapp`. Updated 2026-09-27, 100K+ downloads, in-app purchases. https://www.tszheichoi.com/sensorlogger
- **Open source:** no.
- **Records:** accelerometer, gyroscope, magnetometer (raw and calibrated), orientation, barometer, GPS, audio, camera images/video (front or back), pedometer, light and Bluetooth, at adjustable sampling rates. **Camera depth is iOS only**, and there are no AR poses or intrinsics.
- **Export:** zipped CSV / JSON / SQLite (free); Excel / KML in paid tiers; HTTP/MQTT streaming.
- **Cost:** free core, with Plus / Pro / Ultimate subscriptions or one-off licences.
- **Maps to (A)/(B): not at all** (no pose or depth). Fine for GPS/IMU side-logging.

### 2.7 Spectacular Rec (Spectacular AI)

- **Android:** yes, Play `com.spectacularai.rec`. Updated 2025-09-15, 500+ downloads, no IAP. It "records video and IMU sensor data" for offline processing by Spectacular AI tools. The mapping README says the Android app works "with or without ToF". https://github.com/SpectacularAI/sdk-examples/tree/main/python/mapping
- **Open source:** the app is not. The processing SDK `spectacularAI` (PyPI 1.53.2) is "**Free for non-commercial use**".
- **Pipeline:** `sai-cli process <rec> <out> --device_preset android` runs offline VIO + mapping and writes a **Nerfstudio `transforms.json`** (per-keyframe poses and intrinsics) plus images. It can also write `.ply`/`.pcd` point clouds or `.obj` meshes (`--texturize` is beta). Depth PNGs are written only if the recording has depth frames (ToF). On the 13 Ultra, expect **no depth**, because the ToF is probably not exposed (unverified).
- **Maps to (A)/(B): small converter for poses + RGB + mesh; no depth or confidence.** You would need a monocular metric depth model to fill depth. The licence is a concern for commercial use.

### 2.8 ARCore Depth Lab (Google)

- **Android:** Play `com.google.ar.unity.arcore_depth_lab`, **last updated 2022-05-09**, 100K+ downloads. Source is Apache-2.0 (Unity), last push 2026-05-14. https://github.com/googlesamples/arcore-depth-lab
- **Records:** nothing. It is a demo of depth effects (occlusion, physics, point-cloud viz) with no data export (**unverified** for the newest source).
- **Maps to:** **not at all.** It is useful only as a quick visual check that Depth API quality on the 13 Ultra is acceptable.

### 2.9 Polycam (Android)

- **Android:** yes, Play `ai.polycam`. Updated 2026-09-29, 1M+ downloads, IAP.
- **Open source:** no.
- **Android features:** the Play description is all photo-based photogrammetry ("Transform photos into 3D models", "Runs smoothly on any Android device with 2GB+ RAM"), with cloud processing.
- **Export (Pro):** mesh as OBJ / DAE / FBX / STL / glTF; point cloud as DXF / PLY / LAS / XYZ / PTS.
- **Raw data** (images + camera poses + depth) is a developer-mode feature for iOS **LiDAR / Room** captures only, according to Polycam's help centre (https://learn.poly.cam/hc/en-us/articles/34295907278996; it returned 403 to the fetcher, so this rests on the search snippet and is **unverified** in full).
- **Maps to:** mesh only, giving `textured_output.obj` or an ARKitScenes `.ply`. **No per-frame poses or depth on Android.**

### 2.10 Scaniverse (Niantic Spatial)

- **Android:** yes, Play `com.nianticlabs.scaniverse`. Updated 2026-09-18, 100K+ downloads, free tier plus paid cloud plans. Launched on Android 2024-05-21 (https://radiancefields.com/scaniverse-arrives-on-android). Requires ARCore with Depth API and 4 GB+ RAM (https://radiancefields.com/scaniverse-cloud-comes-to-android).
- **Open source:** no.
- **What you get:** on-device Gaussian splats, plus cloud splats and meshes. Export is SPZ / PLY / GLB / FBX.
- **Raw frames / poses / depth export:** none documented (**unverified**).
- **Maps to:** mesh/splat only (mesh for `textured_output.obj` / `.ply`). Not the per-frame data either target needs.

### 2.11 KIRI Engine

- **Android:** yes, Play `com.kiriengine.app`. Updated 2026-09-18, 1M+ downloads, IAP.
- **Open source:** no.
- **Modes:** Photo Scan (photogrammetry), Featureless Object Scan (NSR), 3D Gaussian Splatting from video. All are cloud-processed.
- **Export:** OBJ / STL / FBX / GLTF / GLB / USDZ / PLY / XYZ. The free tier allows "at least 3 exports weekly"; Pro is a subscription.
- **Camera-pose / COLMAP export on Android:** **unverified**. No depth.
- **Maps to:** mesh only.

### 2.12 Luma AI (3D capture)

- **Android:** launched on Play 2024-04-10 as `ai.lumalabs.polar` (https://radiancefields.com/luma-ai-android-released). On 2026-10-04 that listing returns **404** (US), so it is apparently delisted. Luma has since pivoted to video generation. Status is **unverified**.
- **Open source:** no. Cloud NeRF/splat output, with no raw pose or depth export (unverified).
- **Maps to:** not usable today.

### 2.13 iOS-only apps (confirmed not on Android)

| App | Evidence |
|---|---|
| **Stray Scanner** | README: "raw RGB-D video datasets recorded using LiDAR enabled iOS devices", App Store only (https://github.com/strayrobots/scanner, MIT, last push 2026-04-04). Its format is close to ours, but it is iOS-only. |
| **Record3D** | App Store only (https://apps.apple.com/app/id1477716895). No Android listing found. |
| **3D Scanner App (Laan Labs)** | App Store only (https://apps.apple.com/us/app/-/id1419913995). This is the app whose "All Data" export defines target (B). It is **not available on Android**. |
| **SiteScape (FARO)** | iOS LiDAR app (https://www.faro.com/en/Products/Software/SiteScape). The Play package `com.sitescape` is an **unrelated** solar-survey app by Energy Scape Renewables. |
| **Teleport (Varjo)** | App Store (https://apps.apple.com/us/app/teleport-by-varjo/id6450445339). Android was announced as "eventually" but none is found (`com.varjo.teleport` returns 404, unverified). |
| **SplatCam** | iPhone LiDAR only (https://apps.apple.com/app/id6759800588). |

### 2.14 Small open-source ARCore recorders on GitHub (2019–2026)

These are all hobby or one-off projects with **no Play listing**, so you would build and sideload them. Each is a useful **reference** but not a tool to rely on.

| Repo | Last push | Licence | What it saves | Notes |
|---|---|---|---|---|
| [rupeshbharambe24/vetra-depth-recorder-latest-v3](https://github.com/rupeshbharambe24/vetra-depth-recorder-latest-v3) | 2026-05-05 | none | Per **tap**: `color.jpg`, `depth.bin` (DEPTH16 via `acquireDepthImage16Bits`, stride-correct), `meta.json` (intrinsics + 4x4 pose); ZIP export | Flutter. Manual single-frame capture, not a stream. Full (smoothed) depth, no confidence. |
| [sinhaankur/Structura](https://github.com/sinhaankur/Structura) | 2026-09-29 | MIT | ARCore Depth fused (TSDF) into a mesh; exports USDZ / OBJ / glTF / PLY / STL | Mesh-centric, no per-frame dump documented. |
| [GIitchz/ar-data-recorder-app](https://github.com/GIitchz/ar-data-recorder-app) | 2026-04-28 | none | "Data is stored … under ar_dataset" | Undocumented. |
| [CurvSurf/PointCloudRecorder-GalaxyXR](https://github.com/CurvSurf/PointCloudRecorder-GalaxyXR) | 2025-12-24 | — | ARCore depth to accumulated point cloud | Built for Galaxy XR headset, not phones. |
| [vanjac/ar-recorder](https://github.com/vanjac/ar-recorder) | 2025-03-23 | MIT | Phone 6-DoF trajectory to Blender | Pose only, Unity, APK in releases. |
| [mtee/ArpRec](https://github.com/mtee/ArpRec) | 2020-05-28 | GPL-3.0 | Raw camera MP4 + poses file | Old, no depth. |
| [PyojinKim/ARCore-Data-Logger](https://github.com/PyojinKim/ARCore-Data-Logger) | 2019-09-10 | MIT | ARCore 6-DoF pose + point cloud to text | Old, no images/depth. |
| [drumath2237/ARGeoRecorder](https://github.com/drumath2237/ARGeoRecorder) | 2023-02-14 | — | ARCore recording with Geospatial | Unity. |
| [remmel/recorder-3d](https://github.com/remmel/recorder-3d) | — | — | RGB + ToF depth + pose (CSV), PLY | **Huawei AREngine only**, not applicable. |

I found no maintained, store-published Android app that exports a per-frame stream of posed RGB + metric depth + confidence the way Stray Scanner, Record3D or 3D Scanner App do on iOS.

---

## 3. Summary table

"Small converter" means a Python script of roughly 100–300 lines run on the workstation.

| App | Android? | Records | Per-frame poses? | Depth? | Export | ARKitScenes (A) | LiteReality (B) |
|---|---|---|---|---|---|---|---|
| **RTAB-Map** | Yes, sideload APK 0.21.4 (Play delisted) | ARCore VGA RGB, intrinsics, optimized poses, mesh | **Yes** (ARCore + loop closure) | **Yes**, ARCore smoothed depth (enable "Depth From Motion"), no confidence | `.db` → `rtabmap-export --images --poses_camera --mesh --texture` | **Small converter** (confidence synthesized) | **Small converter** (confidence synthesized, VGA RGB, no room.usdz) |
| ARCore `raw_depth_java` + `computervision_java` (DIY) | Build yourself | Whatever you write: CPU image (VGA or higher), raw depth + confidence, pose, intrinsics, IMU | Yes | **Yes, raw + confidence** | Your own layout | **Direct** if written in that layout | **Direct** (minus room.usdz) |
| ARCore `recording_playback_java` | Build yourself | MP4: VGA image, IMU, metadata, custom tracks | No (regenerate via playback) | Preview track only | `.mp4` | Not directly | Not directly |
| Spectacular Rec + `sai-cli` | Yes (Play) | Video + IMU (+ToF if exposed) | Yes, offline VIO keyframes | Only with ToF (likely none) | Nerfstudio `transforms.json`, `.ply`/`.obj` | Converter, **no depth** | Converter, **no depth/conf** |
| OpenCamera Sensors | Yes (F-Droid / APK), archived 2022 | Video + gyro/accel/mag CSV, synced timestamps | No | No | MP4 + CSV | Not directly (needs SfM + mono depth) | Not directly |
| MARS logger (Android) | Yes (APK), unmaintained | Video ~30 Hz, IMU ~100 Hz, focal length | No | No | MP4 + CSV | Not directly | Not directly |
| Open Camera | Yes (Play), active | Photo/video, **RAW DNG**, GPS | No | No | JPG/DNG/MP4 | Not at all | Not at all (texture/photogrammetry only) |
| Sensor Logger | Yes (Play), active | IMU, GPS, baro, camera image/video | No | No (iOS only) | CSV/JSON/SQLite | Not at all | Not at all |
| ARCore Depth Lab | Yes (Play, 2022) | Nothing (demo) | No | Live only | None | Not at all | Not at all |
| Polycam | Yes (Play) | Photos → cloud photogrammetry | No (raw data is iOS LiDAR only) | No | OBJ/FBX/GLTF/PLY… (Pro) | Mesh only | Mesh only |
| Scaniverse | Yes (Play) | ARCore-depth capture → splat/mesh | Not exported (unverified) | Used internally only | SPZ/PLY/GLB/FBX | Mesh only | Mesh only |
| KIRI Engine | Yes (Play) | Photos/video → cloud mesh/3DGS | Not exported (unverified) | No | OBJ/PLY/GLB/USDZ… | Mesh only | Mesh only |
| Luma AI | Delisted (404) | — | — | — | — | No | No |
| Stray Scanner, Record3D, 3D Scanner App, SiteScape, Teleport, SplatCam | **No (iOS only)** | — | — | — | — | — | — |

---

## 4. Recommendation for today

**Today, capture with RTAB-Map for Android (ARCore NDK driver), then convert on the workstation.** It is the only ready-made Android tool that gives per-frame camera poses (ARCore VIO plus loop closure), per-frame intrinsics, metric ARCore depth and a textured mesh, in an open format we can script.

Setup:

1. Sideload `RTABMap-0.21.4-android30.apk` from GitHub. The Play listing is gone.
2. In Settings, choose the ARCore (NDK) camera driver and enable **Depth From Motion**.
3. Set **Update Rate** to "Max" or 5 Hz (the default 1 Hz is too sparse).
4. Scan, save the `.db`, and `adb pull` it.
5. On fleet, run `rtabmap-export --images --poses_camera --mesh --texture`, then a small converter into both layouts:
   - downscale VGA to 256x192 for ARKitScenes;
   - write camera-to-world json for LiteReality;
   - invert to world-to-camera for `.traj`;
   - write a **synthesized** confidence (valid-depth mask), and record that as a deviation in run metadata.

**Before relying on it, do a 2-minute smoke test** on the 13 Ultra. It is not confirmed that a 2024 APK runs there with ARCore 1.56, and the depth resolution is not known. If RTAB-Map fails, Spectacular Rec plus `sai-cli process --device_preset android` is the fallback for poses, RGB and mesh, but it gives no depth and is non-commercial only.

**Strategically, finish the in-repo ARCore recorder** (`capture/android/`, already on `com.google.ar:core:1.56.0`), built from `raw_depth_java` and `computervision_java`. It is the only route to real **Raw Depth plus confidence**, every-frame poses, high-res CPU images and IMU, written **directly** into the 3D-Scanner-App / ARKitScenes layouts. No ready-made Android app does this.

**OpenCamera Sensors is not needed for scene capture.** Use it only if we want VIO-grade synced video + IMU research logs. Open Camera DNG is useful only for separate high-quality stills, since nothing can record RAW at the same time as an ARCore session.

## 5. Caveats

- **The depth is estimated, not measured.** On the 13 Ultra, ARCore depth comes from depth-from-motion; ARCore does not use the phone's ToF module. It is noisy, needs camera motion, is most accurate at 0.5–5 m, and degrades on textureless walls, glass and dark scenes. It is not comparable to ARKitScenes' LiDAR-derived `lowres_depth`, so label Android scenes as such in any evaluation.
- **No LiDAR, no RoomPlan.** `room.usdz` cannot be produced on Android, so LiteReality's room-layout input must come from elsewhere (e.g. a layout estimator on the mesh) or be skipped.
- **Confidence is synthetic with RTAB-Map.** Only the Raw Depth API gives real confidence, and it uses ARCore's 0–255 scale, not ARKit's 0/1/2. That needs a documented mapping either way.
- **Resolution is low.** ARCore tracks on a VGA CPU image. RTAB-Map uses the lowest config, so LiteReality textures and `frame_*.jpg` will be far below 3D Scanner App's ~1920x1440. This is fine for ARKitScenes' 256x192.
- **Pose conventions differ.** ARCore poses are in the OpenGL camera convention (+Y up, -Z forward). RTAB-Map exports in its own optical/base frames. Verify the axis flips against one known scene before converting at scale.
- **Several items are unverified:** the RTAB-Map APK on this phone, the 13 Ultra depth resolution, the ToF exposure via Camera2, Geospatial support (inferred from the absence of a flag), Luma's current status, and the absence of raw export in Scaniverse and KIRI.
