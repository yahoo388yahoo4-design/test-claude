# R2S robot link protocol (v1)

How R2S Capture's navigation mode talks to a robot's motor controller. Small on purpose: anything that
can parse JSON and drive two wheels can implement it.

## Transports

| Transport | Framing | Default |
|---|---|---|
| Wi-Fi WebSocket | one JSON object per text message | `ws://192.168.4.1:8777/robot` (ESP32 access point); the Python receiver prints its own URL |
| Bluetooth LE, Nordic UART Service | one JSON object per line (`\n`); the phone writes to RX `6E400002-…`, the robot notifies on TX `6E400003-…`; service `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` | advertised name starting with `R2S-Robot` |

Lines longer than one BLE packet arrive in pieces; join them until `\n`.

## Units and axes

* `v`: forward speed, **m/s** (negative = reverse). `w`: turn rate, **rad/s**, **positive = turn left**
  (counter-clockwise seen from above, the ROS `cmd_vel` convention).
* Differential drive: `left = v - w * wheel_base / 2`, `right = v + w * wheel_base / 2`.
* `move`: `dist_cm` (negative = backwards) at `speed_cms`. `turn`: `deg` (positive = left) at `speed_dps`.

## Phone → robot

| `type` | Fields | Meaning |
|---|---|---|
| `hello` | `proto`, `client` | sent on connect; answer with `hello` |
| `ping` | `seq`, `t` | sent every second; answer `pong` with the same `t` (the app shows the round trip) |
| `vel` | `v`, `w`, `seq` | drive at this body velocity. **Valid for 0.5 s**: the app re-sends at 10 Hz while moving; the robot must stop when no `vel` arrives in time (watchdog) |
| `stop` | `seq` | stop now; cancels `move` / `turn` |
| `move` | `dist_cm`, `speed_cms`, `seq` | drive straight this far, then stop and send `done`. Only sent when the app's "closed-loop with ARKit" setting is **off**; otherwise the app does the move itself with `vel` and measures the distance with ARKit |
| `turn` | `deg`, `speed_dps`, `seq` | turn in place, then `done`. Same closed-loop note |
| `estop` | `on` | latch (or release) an emergency stop |

Example: `{"seq":42,"type":"vel","v":0.25,"w":-0.1}`

## Robot → phone

| `type` | Fields | Meaning |
|---|---|---|
| `hello` | `proto`, `name`, `caps` (list), optional `wheel_base`, `max_wheel` | marks the link as connected in the app |
| `pong` | `seq`, `t` | echo of `ping` |
| `status` | any of `v`, `w`, `left`, `right` (wheel m/s), `busy`, `estop`, `battery_v` | 1 to 10 Hz; shown in the app's top bar and logged |
| `done` | `seq` | a `move` / `turn` finished |
| `ack` | `seq` | optional, per command |
| `estop` | | the robot stopped itself (bumper, cliff sensor, button); the app cancels its task |
| `error` | `seq`, `error` | bad message |

## Safety rules for an implementation

1. Stop when no `vel` arrives for 0.5 s, and when the connection drops.
2. Clamp speeds to what the base can do safely.
3. Treat `stop` as highest priority and never queue it behind other work.
4. The phone's obstacle avoidance only sees what the LiDAR sees (about 5 m, its field of view, nothing
   below 4 cm or behind the robot). Add a bumper or cliff sensor and send `estop` if the robot can hurt
   itself or someone.

## Reference implementations

* `robot/receiver.py`: Python asyncio WebSocket server with `sim`, `serial` and Raspberry Pi `gpio`
  backends. `robot/test_receiver.py` tests it end to end.
* `robot/esp32_diffdrive/esp32_diffdrive.ino`: ESP32 with Wi-Fi WebSocket and BLE UART, TB6612 / L298N /
  DRV8833 drivers.
* `robot/serial_motor/serial_motor.ino`: Arduino motor bridge for `receiver.py --backend serial`.

For ROS 2: `vel` maps one-to-one onto `geometry_msgs/Twist` (`linear.x = v`, `angular.z = w`); a 30-line
node that subscribes to the WebSocket and publishes `/cmd_vel` is all that is needed.
