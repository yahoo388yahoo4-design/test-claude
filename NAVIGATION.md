# Navigation mode: put the iPhone on a robot

R2S Capture's **Nav** button (bottom right of the capture screen) opens a full-screen navigation mode.
The iPhone 17 Pro becomes the robot's whole perception and planning stack: ARKit visual-inertial SLAM for
the pose, the LiDAR for obstacles, a live 2D occupancy map, an A\* planner and a path follower. It drives
a robot over Wi-Fi or Bluetooth with the small JSON protocol in [robot/PROTOCOL.md](robot/PROTOCOL.md),
or just guides a person with arrows and voice.

> **Status:** the navigation core (map, planner, follower, guidance text, protocol) is unit-tested and
> tested in a closed-loop simulation (`ios/NavTests/run_tests.sh`), and the Python robot receiver is
> tested end to end (`robot/test_receiver.py`). The iOS screens compile in the GitHub Actions build, but
> nobody has driven a real robot with it yet. Start slow, on the floor, with a hand near the power switch.

## 1. Quick start

1. **Make a map (optional but recommended).** Capture the room with mode **B (RoomPlan)** or mode **A**,
   with *Settings › Save ARWorldMap* on. You can also skip this and explore with an empty map.
2. **Mount the phone** on the robot with the back camera facing forward, roughly level, high enough that
   the robot's own body is not in view (or measure the robot radius so it is ignored, see §4).
3. **Start a robot receiver** (see §5) and note its address.
4. In the app: **Nav › gear**:
   * *Map*: pick the capture session from step 1 and tap **Load map and restart tracking**.
   * *Robot link*: Wi-Fi with `ws://<robot>:8777/robot`, or Bluetooth LE. Tap **Connect**; the top bar's
     link dot turns green and shows the round-trip time.
   * *Robot and mount*: robot radius, the height of the tallest part of the robot, the camera height and
     how far the camera sits ahead of the turning centre.
5. Walk the phone (or drive the robot) past a mapped area until the status says **relocalised**.
6. **Pick a goal**: tap the floor in the camera view, tap the minimap, or use the **Go to** menu, which
   lists the RoomPlan objects of the map (table, sofa, bed …) and picks a reachable spot next to one.
7. Choose a mode:
   * **Guide**: arrows, voice ("Turn left 45 degrees", "Go straight 1.5 meters, then turn right",
     "Obstacle ahead, 40 centimeters") and proximity beeps. No motor commands. Good for checking the plan
     by carrying the phone, or for driving the robot yourself.
   * **Auto**: press **GO** and the phone drives the robot along the path, slowing down and stopping for
     obstacles, re-planning around new ones, and backing up when stuck.
   * **Manual**: joystick, plus "move X cm at Y cm/s" and "turn N degrees at M °/s" buttons.
8. **STOP** is always on screen. Closing the screen, losing tracking or losing the link also stops the
   robot (the receivers have a 0.5 s watchdog).

## 2. What is on screen

| Where | Indicator |
|---|---|
| Top bar | tracking state (relocalising, initialising, too fast …), map status, robot link state, round trip in ms, robot status (battery, wheel speeds, busy, E-stop) |
| Banner | instruction arrow (rotates to the heading error), spoken instruction text, state (Driving, Rotating, Blocked, Arrived, No path), distance left, ETA, heading error |
| Camera view | the planned path as a cyan ribbon with white direction chevrons, the goal pole, the yellow look-ahead point the follower steers at, and a coloured ball on the nearest obstacle in each 10° sector ahead (red = within stop distance, orange = slow-down zone, yellow, green) |
| Left | speedometer (measured speed from ARKit, white tick = commanded speed), turn-rate bar (measured, white tick = commanded), frame rate, thermal state, LiDAR pixels and obstacle points per frame, camera height above the floor, world-mapping state |
| Right | live LiDAR depth image, false colour 0 to 4 m |
| Radar | 360° "virtual lidar" cast in the map (dots coloured by distance), the live LiDAR sectors ahead as wedges, the robot footprint, a dashed line to the nearest obstacle with its distance and bearing |
| Minimap | top-down occupancy map around the robot (red = seen now, blue = from the saved map, green = free), RoomPlan walls, doors and labelled objects, the path, the goal and the robot; tap to set a goal |
| Distance chips | gap to the nearest obstacle in front, left, right, behind and overall, measured from the robot's edge |
| Bottom row | Go to menu, GO (Auto), manual controls toggle, STOP, record a log, save the map |

## 3. How it works

* **SLAM / localisation:** ARKit world tracking (visual-inertial odometry with LiDAR). With a saved
  `worldmap.arworldmap` it relocalises into that map's coordinate frame, so the saved mesh, RoomPlan room
  and grid line up with the live view. Robot pose = the camera position on the floor plane, offset by the
  mount distance; heading = the camera's viewing direction projected on the floor.
* **Floor:** the lowest horizontal ARKit plane classified as floor, else camera height minus the mount
  height setting.
* **Obstacles:** every 0.1 s the LiDAR depth (256×192, medium or high confidence, every third pixel) is
  back-projected to 3D. Points 4 cm above the floor up to the robot height are obstacles, points well
  below the floor (stairs down, holes) are obstacles too once the floor is known, floor points are free
  space, and points inside the robot's own radius are ignored.
* **Map:** a 30 × 30 m log-odds occupancy grid at 5 cm. Each sweep ray-traces free space from the sensor
  to every point and marks hits; two consistent hits are needed before a cell counts as occupied. A
  static layer comes from the loaded map (mesh.ply faces that are not floor or ceiling, RoomPlan walls
  and objects below the robot height, a saved map.pgm), with RoomPlan doors and openings cut out of the
  walls. Fresh free-space evidence overrides the static layer, so furniture that moved does not block
  forever.
* **Planning:** A\* on the grid at 2 Hz, with cells closer than the robot radius forbidden, a soft
  cost band beyond that, a small penalty for unexplored cells, and line-of-sight smoothing. A goal inside
  an obstacle snaps to the nearest free spot within 1 m.
* **Following:** pure pursuit towards a point 40 cm ahead on the path; rotate in place when the heading
  error is over 35°; speed scaled by heading error, distance to goal and the forward gap. The forward
  gap is the closest live LiDAR point (or map cell) inside the robot's width ahead. Below the slow-down
  distance the speed ramps down, at the stop distance the robot stops (and turns towards the path if
  that clears the way), and after 3 s blocked it backs up 1 s if the rear is clear, then re-plans.
* **Move X cm / turn N degrees:** by default closed-loop: the phone streams velocity commands and stops
  when ARKit has measured the distance or angle, so the robot needs no encoders. Turn off *closed-loop*
  in settings to send `move` / `turn` to the robot instead.

## 4. Settings worth checking

* **Robot radius**: half the robot's widest dimension. Points within it are treated as the robot itself.
* **Obstacle height up to**: the robot's height (plus margin). Table tops above it are ignored; table legs
  are not.
* **Camera ahead of turning centre**: for a differential drive, the distance from the wheel axle to the
  phone along the driving direction.
* **Slow down below / Stop below**: forward gaps from the robot's edge. Defaults 0.6 m and 0.25 m.
* **Max speed / max turn rate**: start at 0.15 m/s and 30°/s.

## 5. Robot side

See [robot/PROTOCOL.md](robot/PROTOCOL.md). Three reference receivers:

* **Any Linux computer on the robot** (Raspberry Pi, Jetson, laptop):
  `pip install websockets && python3 robot/receiver.py --backend sim -v` prints what the robot would do.
  Then `--backend gpio` (Raspberry Pi + H-bridge through gpiozero) or `--backend serial` (an Arduino
  running `robot/serial_motor/serial_motor.ino`). Set `--wheel-base` and `--max-wheel` (wheel speed at
  full power) for your base. `python3 robot/test_receiver.py` checks it end to end.
* **ESP32** straight on the motor driver: `robot/esp32_diffdrive/esp32_diffdrive.ino` (Wi-Fi access point
  `R2S-Robot` / `r2srobot` plus BLE UART). Edit the pins and `WHEEL_BASE` / `MAX_WHEEL` at the top.
* **ROS 2 robots**: `vel` is a `cmd_vel` Twist; bridge the WebSocket to `/cmd_vel`.

## 6. Logs and saved maps

* **Record** (●) writes `sessions/<time>_nav/` with `session.json` and `nav.jsonl`: 10 Hz samples (pose
  `x`, `z`, `theta`, measured and commanded `v` / `w`, front gap, nearest obstacle and bearing, the nine
  sector gaps, goal and distance left, tracking) plus events (goal, mode, stop, arrived, robot messages).
* **Save map** (↓) writes `sessions/<time>_navmap/` with `worldmap.arworldmap`, `mesh.ply`, `planes.json`,
  `map.pgm` + `map.yaml` (ROS map_server format: ROS x = ARKit x, ROS y = −ARKit z) and the RoomPlan room
  of the map it was built on. It then appears in the map list, so the next run starts relocalised with
  everything learned so far.

Both folders sit next to the capture sessions and copy off the phone the same way (Files app, Finder,
the upload receiver).

## 7. Limits

* LiDAR range is about 5 m and it only sees ahead. Glass, mirrors and black or very shiny surfaces can
  be missed. Objects under 4 cm (cables) are not obstacles.
* ARKit needs texture and light to track. Fast spins, blank walls or darkness degrade tracking; the robot
  stops while tracking is not normal.
* Relocalisation works when the room looks like it did at capture time and the phone sees a mapped
  area.
* Phone heat: continuous ARKit + LiDAR warms the phone; the thermal state is in the HUD.
* The planner is 2D. Overhangs lower than the robot height block, which is the safe choice.
