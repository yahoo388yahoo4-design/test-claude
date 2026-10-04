"""Neato and OpenBot motor backends for robot/receiver.py (USB serial).

Ported from OSSDC VisionAI Mobile (https://github.com/OSSDC/OSSDC-VisionAI-Mobile,
app/src/main/java/org/ossdc/visionai/webrtc/RaceOSSDCActivity.java), Copyright (C) 2021 Marius Slavescu,
OSSDC.org, licensed under the Apache License 2.0 (see robot/THIRD_PARTY.md). Changes: rewritten in Python,
driven by body velocity (m/s, rad/s) instead of a joystick, a 1 s distance horizon for Neato as in the
neato_robot ROS driver, OpenBot heartbeat, battery / bumper / sonar parsing.

Both robots plug into a computer (Raspberry Pi, laptop) over USB; the phone talks to that computer with
robot/PROTOCOL.md. An Android phone can drive them directly over USB-OTG instead (UsbRobot.kt).

Neato XV / Botvac (USB CDC, /dev/ttyACM0, any baud):
    testmode on                   once, so the robot accepts motor commands
    setmotor <L mm> <R mm> <S mm/s>  drive each wheel that far at that speed (re-sent at 10 Hz; each
                                  command replaces the last; distance = 1 s of travel)
    setmotor 0 0 0                stop
    getcharger / getdigitalsensors   polled for battery voltage and the bumper switches
OpenBot (Arduino Nano/ESP32 running the OpenBot firmware, 115200 baud):
    c<left>,<right>               motor PWM, -255..255
    h<ms>                         heartbeat: the firmware stops the motors when none arrives for <ms>
    robot -> computer: v<volts>, s<sonar cm>, b<bumper>, w<left>,<right> (wheel speed)
"""
import math
import re
import threading
import time


# ---------------------------------------------------------------- pure encoders (tested)

def neato_setmotor(left, right, max_mm_s=300.0):
    """Wheel speeds (m/s) -> 'setmotor L R S\\n'. Distances are 1 s of travel at that speed."""
    l_mm, r_mm = left * 1000.0, right * 1000.0
    peak = max(abs(l_mm), abs(r_mm))
    if peak > max_mm_s:
        k = max_mm_s / peak
        l_mm, r_mm, peak = l_mm * k, r_mm * k, max_mm_s
    l_i, r_i, s_i = int(round(l_mm)), int(round(r_mm)), int(round(peak))
    if s_i < 1 or (l_i == 0 and r_i == 0):
        return "setmotor 0 0 0\n"
    return f"setmotor {l_i} {r_i} {s_i}\n"


def apply_deadband(frac, deadband):
    """OSSDC applyDeadband: maps 0..1 onto deadband..1 so small commands still turn the motors."""
    frac = max(-1.0, min(1.0, frac))
    if abs(frac) < 0.005:
        return 0.0
    return math.copysign(deadband + (1.0 - deadband) * abs(frac), frac)


def openbot_ctrl(left, right, max_wheel=0.4, max_pwm=150, deadband=0.3, invert_left=False, invert_right=False):
    """Wheel speeds (m/s) -> 'c<l>,<r>\\n' with PWM in -max_pwm..max_pwm (OSSDC default cap 150)."""
    def pwm(v, inv):
        p = int(math.floor(abs(apply_deadband(v / max_wheel, deadband)) * max_pwm))
        p = min(p, max_pwm, 255)
        p = int(math.copysign(p, v)) if p else 0
        return -p if inv else p
    return f"c{pwm(left, invert_left)},{pwm(right, invert_right)}\n"


def parse_neato_line(line, out):
    """Updates `out` from one line of a getcharger / getdigitalsensors answer ('Label,Value')."""
    parts = line.strip().split(",")
    if len(parts) < 2:
        return
    key, val = parts[0].strip(), parts[1].strip()
    try:
        if key == "VBattV":
            out["battery_v"] = float(val)
        elif key == "FuelPercent":
            out["battery_pct"] = int(float(val))
        elif key in ("LSIDEBIT", "LFRONTBIT", "RSIDEBIT", "RFRONTBIT", "LLDSBIT", "RLDSBIT"):
            out.setdefault("bumpers", {})[key] = int(float(val)) != 0
    except ValueError:
        pass


_OPENBOT_RE = re.compile(r"^([a-z])(.*)$")


def parse_openbot_line(line, out):
    m = _OPENBOT_RE.match(line.strip())
    if not m:
        return
    k, rest = m.group(1), m.group(2)
    try:
        if k == "v":
            out["battery_v"] = float(rest.split(",")[0])
        elif k == "s":
            out["sonar_cm"] = float(rest.split(",")[0])
        elif k == "w":
            l, r = rest.split(",")[:2]
            out["wheel_rps"] = [float(l), float(r)]
        elif k == "b":
            out["bumper"] = rest.strip()
    except ValueError:
        pass


# ---------------------------------------------------------------- serial backends

class _SerialBackend:
    """Shared plumbing: pyserial port, a reader thread that hands each line to `parse`."""

    baud = 115200

    def __init__(self, args, port=None):
        self.args = args
        if port is None:
            import serial  # pyserial
            port = serial.Serial(args.serial, args.baud or self.baud, timeout=0.1)
        self.port = port
        self.lock = threading.Lock()
        self.left = self.right = 0.0
        self.info = {}
        self.bumped = False
        self.alive = True
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def write(self, s):
        with self.lock:
            try:
                self.port.write(s.encode())
                self.port.flush()
            except Exception as e:  # pragma: no cover
                print("serial write failed:", e)

    def _read(self):
        buf = b""
        while self.alive:
            try:
                chunk = self.port.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk.replace(b"\x1a", b"\n")       # Neato ends answers with Ctrl-Z
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").strip("\r ")
                if text:
                    self.parse(text)
            if len(buf) > 4096:
                buf = b""

    def parse(self, line):
        pass

    def status(self):
        s = dict(self.info)
        if self.bumped:
            s["bumped"] = True
            self.bumped = False
        return s

    def close(self):
        self.alive = False
        try:
            self.port.close()
        except Exception:
            pass


class NeatoMotors(_SerialBackend):
    """Neato XV / Botvac over its USB port (OSSDC robotMode "Neato")."""

    def __init__(self, args, port=None):
        super().__init__(args, port)
        self.max_mm_s = getattr(args, "neato_max_mm_s", 300.0)
        self.write("testmode on\n")
        self.poller = threading.Thread(target=self._poll, daemon=True)
        self.poller.start()

    def set(self, left, right):
        self.left, self.right = left, right
        self.write(neato_setmotor(left, right, self.max_mm_s))

    def _poll(self):
        while self.alive:
            self.write("getcharger\n")
            time.sleep(0.15)
            self.write("getdigitalsensors\n")
            time.sleep(0.85)

    def parse(self, line):
        before = dict(self.info.get("bumpers", {}))
        parse_neato_line(line, self.info)
        after = self.info.get("bumpers", {})
        if any(v and not before.get(k) for k, v in after.items()):
            self.bumped = True

    def close(self):
        self.write("setmotor 0 0 0\n")
        self.write("testmode off\n")
        time.sleep(0.05)
        super().close()


class OpenBotMotors(_SerialBackend):
    """OpenBot body (Arduino firmware) over USB serial (OSSDC robotMode "OpenBot")."""

    def __init__(self, args, port=None):
        super().__init__(args, port)
        time.sleep(getattr(args, "openbot_boot_wait", 2.0))  # the Nano resets when the port opens
        self.last_hb = 0.0

    def _cmd(self, left, right):
        a = self.args
        return openbot_ctrl(left, right, a.max_wheel, a.max_pwm, a.deadband, a.invert_left, a.invert_right)

    def set(self, left, right):
        self.left, self.right = left, right
        now = time.monotonic()
        if now - self.last_hb > 0.25:
            self.last_hb = now
            self.write(f"h{int(self.args.openbot_heartbeat_ms)}\n")
        self.write(self._cmd(left, right))

    def parse(self, line):
        had = self.info.get("bumper")
        parse_openbot_line(line, self.info)
        if self.args.openbot_bumper_estop and self.info.get("bumper") not in (None, "", "0") and self.info.get("bumper") != had:
            self.bumped = True

    def close(self):
        self.write("c0,0\n")
        time.sleep(0.05)
        super().close()


def add_args(p):
    """Command-line options for these backends (called from receiver.parse)."""
    g = p.add_argument_group("neato / openbot")
    g.add_argument("--neato-max-mm-s", type=float, default=300.0, help="Neato wheel speed cap, mm/s")
    g.add_argument("--max-pwm", type=int, default=150, help="OpenBot PWM at --max-wheel (OSSDC default 150, max 255)")
    g.add_argument("--deadband", type=float, default=0.3, help="OpenBot: smallest non-zero PWM as a fraction")
    g.add_argument("--openbot-heartbeat-ms", type=int, default=750,
                   help="OpenBot firmware stops the motors when no heartbeat arrives for this long")
    g.add_argument("--openbot-bumper-estop", action="store_true", help="latch an e-stop on an OpenBot bumper message")
    g.add_argument("--openbot-boot-wait", type=float, default=2.0, help="wait after opening the port, s")


# Defaults per backend, applied when the user did not set them.
DEFAULTS = {
    "neato": {"serial": "/dev/ttyACM0", "wheel_base": 0.248, "max_wheel": 0.30, "name": "Neato"},
    "openbot": {"serial": "/dev/ttyUSB0", "wheel_base": 0.15, "max_wheel": 0.40, "name": "OpenBot"},
}
