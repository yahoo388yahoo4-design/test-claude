# Robot side of R2S Capture's navigation mode

| File | What |
|---|---|
| [PROTOCOL.md](PROTOCOL.md) | the JSON messages between the phone and the robot (Wi-Fi WebSocket or BLE UART) |
| [receiver.py](receiver.py) | Python receiver for a Linux computer on the robot: `sim`, `serial` and Raspberry Pi `gpio` backends |
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
