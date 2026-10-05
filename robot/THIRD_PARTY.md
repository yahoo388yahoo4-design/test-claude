# Third-party code in the robot drivers

## OSSDC VisionAI Mobile (Apache License 2.0)

<https://github.com/OSSDC/OSSDC-VisionAI-Mobile> (`RaceOSSDCActivity.java`), Copyright (C) 2021 Marius
Slavescu, OSSDC.org, licensed under
the Apache License, Version 2.0 (<https://www.apache.org/licenses/LICENSE-2.0>).

The Neato and OpenBot USB serial drivers are ported from its Android app, which in turn builds on the
OpenBot project (Intel, MIT License) and the Neato XV serial command set:

* `robot/usb_robots.py` (Python, for `receiver.py --backend neato / openbot`)
* `android/app/src/main/java/com/real2sim/capture/robot/UsbRobot.kt` (Android USB-OTG)

What was taken: the Neato command sequence (`testmode on`, `setmotor L R S`, `getcharger`,
`getdigitalsensors`), the OpenBot control message `c<left>,<right>`, its 150 PWM default, dead band
and `h<ms>` heartbeat, and the USB IDs / serial settings (115200 8N1, DTR and RTS on).

What was changed: rewritten in Python and Kotlin; driven by this project's robot protocol v1
(`robot/PROTOCOL.md`) with `vel` / `move` / `turn` / `estop` and a 0.5 s watchdog instead of OSSDC's
joystick and WebRTC control; Neato speeds capped at 300 mm/s with a 1 s distance horizon; battery,
sonar, wheel speed and bumper parsed into protocol `status` and `estop` messages.

## usb-serial-for-android (MIT License)

<https://github.com/mik3y/usb-serial-for-android>, Copyright (c) 2011-2013 Google Inc. and
contributors, used unmodified as a library dependency by the Android app (CDC-ACM, CH34x, CP21xx,
FTDI and PL2303 drivers).
