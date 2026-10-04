# Robot receiver

Put a phone running the capture app's navigation mode on a two-wheeled robot and let it drive.
The phone does the mapping, localisation, path planning and obstacle checks; the robot only has to
follow `move` / `turn` / `vel` commands ([PROTOCOL.md](PROTOCOL.md)).

```
pip install websockets pyserial
python receiver.py --backend sim                      # no hardware, prints what it would do
python receiver.py --backend serial --serial-port /dev/ttyUSB0 --wheel-base 18 --ticks-per-cm 20 --max-speed 30
```

On the phone: Navigate → robot field `192.168.1.20` (port 8766 and `/r2s` are added) → Connect.
With `--token abc` on the receiver, type `192.168.1.20?token=abc`.

## Hardware

* Any computer on the robot that runs Python 3.9+ (Raspberry Pi, Jetson, old laptop) on the same Wi-Fi as the phone.
* A motor board on USB serial running [arduino/r2s_diffdrive](arduino/r2s_diffdrive/r2s_diffdrive.ino):
  it receives `V <left_cms> <right_cms>` and replies `E <left_ticks> <right_ticks>` (encoders, optional) and
  `B <volts>` (battery, optional). Without encoders the odometry is open loop; the phone's own tracking
  still closes the loop for navigation.
* Mount the phone level, camera facing forward, and enter its height in the app (`mount`).

## Tests

```
pip install websockets pytest
python -m pytest -q robot/test_receiver.py
```
The tests connect over a real WebSocket to the sim backend: hello/ping, move forward and back,
left turn sign, `vel` clamp and watchdog, e-stop latch, hello/token enforcement.
