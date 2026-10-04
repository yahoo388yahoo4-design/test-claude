# Robot side of R2S Capture's navigation mode

| File | What |
|---|---|
| [PROTOCOL.md](PROTOCOL.md) | the JSON messages between the phone and the robot (Wi-Fi WebSocket or BLE UART) |
| [receiver.py](receiver.py) | Python receiver for a Linux computer on the robot: `sim`, `serial`, Raspberry Pi `gpio`, `neato` and `openbot` backends |
| [usb_robots.py](usb_robots.py) | Neato XV / Botvac and OpenBot USB serial drivers used by the `neato` / `openbot` backends (ported from OSSDC VisionAI Mobile, see [THIRD_PARTY.md](THIRD_PARTY.md)) |
| [test_usb_robots.py](test_usb_robots.py) | tests of those drivers against a fake robot on a pseudo-terminal, and end to end through `receiver.py` |
| [test_receiver.py](test_receiver.py) | end-to-end test of `receiver.py` (sim backend) speaking the app's protocol |
| [esp32_diffdrive/](esp32_diffdrive/esp32_diffdrive.ino) | ESP32 firmware: Wi-Fi access point + WebSocket + BLE UART, TB6612 / L298N / DRV8833 |
| [serial_motor/](serial_motor/serial_motor.ino) | Arduino motor bridge for `receiver.py --backend serial` |

Fastest check without hardware:

```bash
pip install websockets
python3 robot/receiver.py --backend sim -v      # prints ws://<ip>:8777/robot
```

Put that URL in the app (Nav › gear › Robot link › Wi-Fi), tap Connect, and drive with the Manual
joystick; the receiver prints the simulated pose. Usage of the app side: [../NAVIGATION.md](../NAVIGATION.md).

## Neato and OpenBot over USB

The Android app drives both straight from the phone over USB-OTG (Nav › Robot link › USB: Neato / USB:
OpenBot). An iPhone cannot open a USB serial port, so on iOS put a Raspberry Pi or laptop on the robot,
plug the robot into it and run the receiver; the iPhone then connects over Wi-Fi as usual:

```bash
pip install websockets pyserial
python3 robot/receiver.py --backend neato   -v   # Neato XV / Botvac, /dev/ttyACM0
python3 robot/receiver.py --backend openbot -v   # OpenBot Arduino, /dev/ttyUSB0
```

* **Neato**: the receiver sends `testmode on`, then `setmotor L R S` (mm, mm, mm/s; each command covers
  1 s of motion and is re-sent with every `vel`, capped at `--neato-max-mm-s`, default 300). It reads the
  battery (`getcharger`) and bumpers (`getdigitalsensors`); a bumper press latches the e-stop and is
  sent to the phone as `{"type":"estop","reason":"bumper"}`.
* **OpenBot**: `c<left>,<right>` PWM in −255..255 (capped at `--max-pwm`, default 150, with OpenBot's
  dead band `--deadband 0.3`), heartbeat `h<ms>`; battery (`v`), sonar (`s`) and wheel speed (`w`) are
  shown in the app's HUD. `--openbot-bumper-estop` latches the e-stop on the bumper message. Flip
  `--invert-left` / `--invert-right` if a wheel runs backwards.

`--wheel-base`, `--max-wheel` and `--serial` default to the robot's own values and can be overridden.
Nothing here has been tried on real Neato or OpenBot hardware yet; start with the wheels off the
ground.
