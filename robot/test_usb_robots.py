#!/usr/bin/env python3
"""Tests for the Neato and OpenBot backends (robot/usb_robots.py), with a pseudo-terminal standing in
for the robot's USB serial port, and end to end through receiver.py's WebSocket.

  pip install websockets pyserial && python3 robot/test_usb_robots.py
"""
import asyncio
import json
import os
import select
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import receiver  # noqa: E402
import usb_robots as ur  # noqa: E402

fails = 0


def check(cond, msg):
    global fails
    print(("  ok  " if cond else "FAIL  ") + msg)
    if not cond:
        fails += 1


class FakeRobot:
    """Master side of a pty: what the robot would read, and a way to answer."""

    def __init__(self):
        import serial
        self.master, slave = os.openpty()
        import tty
        tty.setraw(self.master)
        tty.setraw(slave)
        self.port = serial.Serial(os.ttyname(slave), 115200, timeout=0.05)
        self.got = b""

    def read(self, wait=0.3):
        end = time.time() + wait
        while time.time() < end:
            r, _, _ = select.select([self.master], [], [], 0.02)
            if r:
                self.got += os.read(self.master, 4096)
        return self.got.decode()

    def say(self, s):
        os.write(self.master, s.encode())


print("encoders")
check(ur.neato_setmotor(0.2, 0.2) == "setmotor 200 200 200\n", "neato straight 0.2 m/s")
check(ur.neato_setmotor(-0.1, 0.1) == "setmotor -100 100 100\n", "neato spin left")
check(ur.neato_setmotor(0.6, 0.3) == "setmotor 300 150 300\n", "neato clamps to 300 mm/s keeping the ratio")
check(ur.neato_setmotor(0, 0) == "setmotor 0 0 0\n", "neato stop")
check(ur.openbot_ctrl(0, 0) == "c0,0\n", "openbot stop")
check(ur.openbot_ctrl(0.4, 0.4) == "c150,150\n", "openbot full speed = max pwm 150 (OSSDC cap)")
check(ur.openbot_ctrl(0.04, -0.04) == "c55,-55\n", "openbot deadband lifts small commands to ~0.37 * 150")
check(ur.openbot_ctrl(0.4, 0.4, invert_right=True) == "c150,-150\n", "openbot invert right")
check(ur.openbot_ctrl(1.0, 0, max_pwm=255) == "c255,0\n", "openbot clamps at 255")
info = {}
for line in ["VBattV,14.32", "FuelPercent,87", "LSIDEBIT,0", "LFRONTBIT,1"]:
    ur.parse_neato_line(line, info)
check(info.get("battery_v") == 14.32 and info.get("battery_pct") == 87, "neato getcharger parse")
check(info["bumpers"]["LFRONTBIT"] and not info["bumpers"]["LSIDEBIT"], "neato bumper parse")
info = {}
for line in ["v7.84", "s41", "w1.5,-1.5", "b0"]:
    ur.parse_openbot_line(line, info)
check(info == {"battery_v": 7.84, "sonar_cm": 41.0, "wheel_rps": [1.5, -1.5], "bumper": "0"}, f"openbot parse {info}")


def args_for(backend, *extra):
    a = receiver.parse(["--backend", backend, "--openbot-boot-wait", "0", *extra])
    return a


print("neato backend over a pty")
fr = FakeRobot()
a = args_for("neato")
check(a.wheel_base == 0.248 and a.name == "Neato", "neato defaults (wheel base 248 mm)")
m = ur.NeatoMotors(a, port=fr.port)
out = fr.read()
check(out.startswith("testmode on\n"), "sends testmode on first")
check("getcharger\n" in out, "polls getcharger")
m.set(0.15, 0.15)
check("setmotor 150 150 150\n" in fr.read(0.2), "drives with setmotor")


async def timed_move():
    """move 100 cm at 20 cm/s: the 1 s setmotor horizon must be refreshed for the whole 5 s job."""
    r = receiver.Robot(m, a)
    done = []

    async def send(obj):
        done.append(obj)
    await r.handle({"type": "move", "dist_cm": 100, "speed_cms": 20, "seq": 7}, send)
    fr.read(0.05)
    mark = len(fr.got)
    await asyncio.sleep(1.5)
    repeats = fr.read(0.05)[mark:].count("setmotor 200 200 200\n")
    check(repeats >= 10, f"timed move re-sends setmotor every 0.1 s ({repeats} in 1.5 s)")
    check(not done, "done not sent before the deadline")
    await r.handle({"type": "stop", "seq": 8}, send)
    await asyncio.sleep(0.3)
    mark = len(fr.got)
    await asyncio.sleep(0.3)
    tail = fr.read(0.05)
    last = [l for l in tail.splitlines() if l.startswith("setmotor")][-1]
    check(last == "setmotor 0 0 0" and "setmotor 200" not in tail[mark:], f"stop ends the refresh ({last!r})")

asyncio.run(timed_move())
fr.say("GetCharger\r\nLabel,Value\r\nVBattV,14.10\r\n\x1a")
fr.say("LSIDEBIT,0\r\nLFRONTBIT,1\r\n\x1a")
time.sleep(0.4)
st = m.status()
check(st.get("battery_v") == 14.1, f"battery from getcharger answer ({st.get('battery_v')})")
check(st.get("bumped") is True, "bumper press reported once")
check("bumped" not in m.status(), "bumper event cleared after reading")
m.close()
check(fr.read(0.2).rstrip().endswith("setmotor 0 0 0\ntestmode off"), "close stops and leaves test mode")

print("openbot backend over a pty")
fr = FakeRobot()
a = args_for("openbot", "--openbot-bumper-estop")
m = ur.OpenBotMotors(a, port=fr.port)
m.set(0.2, -0.2)
out = fr.read(0.2)
check(out.startswith("h750\n") and "c" in out, f"heartbeat then control {out!r}")
check("c97,-97\n" in out, "0.2 m/s of 0.4 -> (0.3 + 0.7 * 0.5) * 150 = 97")
fr.say("v8.1\nb1\n")
time.sleep(0.3)
st = m.status()
check(st.get("battery_v") == 8.1 and st.get("bumped"), f"battery and bumper {st}")
m.close()


async def e2e():
    """receiver.py with the OpenBot backend on a pty: phone vel -> c<l>,<r>; watchdog -> c0,0; bumper -> estop."""
    import websockets
    fr = FakeRobot()
    a = args_for("openbot", "--openbot-bumper-estop", "--port", "18778", "--status-hz", "10")
    orig = ur.OpenBotMotors.__init__

    def init(self, args, port=None):
        orig(self, args, port=fr.port)
    ur.OpenBotMotors.__init__ = init
    server = asyncio.create_task(receiver.main(a))
    await asyncio.sleep(0.5)
    try:
        async with websockets.connect("ws://127.0.0.1:18778/robot") as ws:
            await ws.send(json.dumps({"type": "hello", "proto": 1, "client": "test"}))
            hello = json.loads(await ws.recv())
            check(hello.get("name") == "OpenBot", f"hello names the robot {hello.get('name')}")
            await ws.send(json.dumps({"type": "vel", "v": 0.2, "w": 0.0, "seq": 1}))
            await asyncio.sleep(0.2)
            check("c97,97\n" in fr.read(0.1), "vel 0.2 m/s -> c97,97")
            await asyncio.sleep(0.6)
            check(fr.read(0.1).rstrip().endswith("c0,0"), "watchdog stops after 0.5 s without vel")
            fr.say("b1\n")
            got_estop = False
            for _ in range(20):
                msg = json.loads(await asyncio.wait_for(ws.recv(), 2))
                if msg.get("type") == "estop":
                    got_estop = True
                    break
            check(got_estop, "bumper -> estop message to the phone")
            mark = len(fr.read(0.1))
            await ws.send(json.dumps({"type": "vel", "v": 0.2, "w": 0.0, "seq": 2}))
            await asyncio.sleep(0.2)
            check("c97" not in fr.read(0.1)[mark:], "latched e-stop ignores vel")
    finally:
        server.cancel()
        ur.OpenBotMotors.__init__ = orig


print("end to end (receiver.py --backend openbot)")
try:
    asyncio.run(e2e())
except asyncio.CancelledError:
    pass

print("FAILED: %d" % fails if fails else "all passed")
sys.exit(1 if fails else 0)
