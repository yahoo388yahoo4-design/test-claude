# r2s robot protocol v1

One protocol for both capture apps (Android `NavActivity`, iPhone navigation mode), so a single
receiver drives the robot whichever phone sits on it.

* Transport: WebSocket, JSON text frames, one object per frame with a `"type"` field.
* The **robot is the server**: `ws://<robot-ip>:8766/r2s`. The phone connects as a client.
* Units: cm, cm/s, degrees, deg/s, seconds. Angles are **positive to the left** (counter-clockwise seen
  from above). Robot odometry frame: x forward at start, y to the left, theta CCW.
* Unknown fields are ignored; unknown types get `{"type":"error","reason":...}`.

## Phone → robot

| type | fields | meaning |
|---|---|---|
| `hello` | `v`=1, `client`, `token`? | must be first; `token` only if the receiver was started with `--token` |
| `move` | `id`, `dist_cm` (± = fwd/back), `speed_cms` (>0) | drive straight, then `done` |
| `turn` | `id`, `angle_deg` (+ = left), `speed_dps` (>0) | rotate in place, then `done` |
| `vel` | `v_cms`, `w_dps` (+ = left), `ttl_ms` | continuous command used by auto-drive (~10 Hz); expires after `ttl_ms`, capped at 500 ms |
| `stop` | `id`? | cancel the current command, wheels to 0 |
| `estop` | | stop and **latch**: everything is refused until `reset` |
| `reset` | | clear the e-stop latch |
| `ping` | `t` | echoed back as `pong` (round-trip time) |

`move` and `turn` queue and run in order (`status.queue` counts them). A `vel` cancels the queue
(`done` with `ok:false, reason:"superseded by vel"`); a `move`/`turn` after a `vel` ends the `vel`.

## Robot → phone

| type | fields | when |
|---|---|---|
| `hello` | `v`, `robot`, `wheel_base_cm`, `max_speed_cms`, `max_turn_dps` | reply to the phone's hello |
| `ack` | `id` | a `move`/`turn` was queued |
| `done` | `id`, `ok`, `reason`? (`stopped`, `estop`, `estop latched`, `superseded by vel`, `timeout`) | a `move`/`turn` finished or was cancelled; also answers `stop` with an `id` |
| `odom` | `t`, `x_cm`, `y_cm`, `theta_deg`, `v_cms`, `w_dps` | ~20 Hz |
| `status` | `battery_v`?, `estop`, `queue` | ~2 Hz |
| `pong` | `t` | reply to `ping` |
| `error` | `reason` | bad message, missing hello, wrong token |

## Safety rules a receiver must keep

1. Clamp to its own `max_speed_cms` / `max_turn_dps` whatever the phone asks.
2. `vel` expires after `min(ttl_ms, 500)` ms without a new one (the phone's ARCore/ARKit tracking can
   drop, the app can freeze).
3. Stop when the last client disconnects.
4. `move`/`turn` give up (`reason:"timeout"`) after three times their nominal duration + 2 s.
5. `estop` latches until `reset`.

## Example

```
→ {"type":"hello","v":1,"client":"r2s-android"}
← {"type":"hello","v":1,"robot":"r2s-diffdrive","wheel_base_cm":18,"max_speed_cms":40,"max_turn_dps":120}
→ {"type":"move","id":1,"dist_cm":50,"speed_cms":20}
← {"type":"ack","id":1}
← {"type":"odom","t":12.31,"x_cm":4.1,"y_cm":0,"theta_deg":0,"v_cms":19.8,"w_dps":0}
← {"type":"done","id":1,"ok":true}
```
