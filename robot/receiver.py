#!/usr/bin/env python3
"""Reference robot receiver for R2S Capture's navigation mode (robot/PROTOCOL.md).

Runs on the robot's computer (Raspberry Pi, Jetson, laptop) and turns the phone's JSON commands into
differential-drive wheel speeds. Pick a motor backend:

  sim     no hardware: integrates the motion and prints it (default; good for testing the app)
  serial  sends "M <left> <right>\\n" (-1000..1000) to a microcontroller, e.g. robot/serial_motor.ino
  gpio    Raspberry Pi + an H-bridge (L298N / TB6612 / DRV8833) through gpiozero

  pip install websockets            (and gpiozero / pyserial for those backends)
  python3 robot/receiver.py --backend sim
  python3 robot/receiver.py --backend serial --serial /dev/ttyUSB0 --wheel-base 0.30 --max-wheel 0.5
  python3 robot/receiver.py --backend gpio --gpio-left 17 27 12 --gpio-right 23 24 13

Then set the app's Navigation settings > Robot link > Wi-Fi to ws://<this computer's IP>:8777/robot.

Safety: velocity commands expire after --watchdog seconds (default 0.5 s) unless refreshed, so the robot
stops if the phone, the app or the Wi-Fi goes away. "move" and "turn" run open-loop from timing (no
encoders here); the app's default closed-loop mode measures the motion with ARKit instead.
"""
import argparse
import asyncio
import json
import math
import socket
import sys
import time

try:
    import websockets
except ImportError:  # pragma: no cover
    sys.exit("pip install websockets")

PROTO = 1


# ---------------------------------------------------------------- motor backends

class SimMotors:
    """No hardware: integrates the differential-drive motion so you can watch it."""

    def __init__(self, args):
        self.args = args
        self.x = self.y = self.th = 0.0
        self.left = self.right = 0.0
        self.t = time.monotonic()

    def set(self, left, right):  # wheel speeds, m/s
        self._integrate()
        self.left, self.right = left, right

    def _integrate(self):
        now = time.monotonic()
        dt = now - self.t
        self.t = now
        v = (self.left + self.right) / 2
        w = (self.right - self.left) / self.args.wheel_base
        self.th += w * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

    def status(self):
        self._integrate()
        return {"sim_pose": [round(self.x, 3), round(self.y, 3), round(math.degrees(self.th), 1)]}

    def close(self):
        self.set(0, 0)


class SerialMotors:
    """Sends 'M <left> <right>' (each -1000..1000, fraction of --max-wheel) to a microcontroller."""

    def __init__(self, args):
        import serial  # pyserial
        self.args = args
        self.port = serial.Serial(args.serial, args.baud, timeout=0)
        self.left = self.right = 0.0

    def set(self, left, right):
        self.left, self.right = left, right
        scale = 1000.0 / self.args.max_wheel
        l = int(max(-1000, min(1000, left * scale)))
        r = int(max(-1000, min(1000, right * scale)))
        self.port.write(f"M {l} {r}\n".encode())

    def status(self):
        return {}

    def close(self):
        self.set(0, 0)
        self.port.close()


class GpioMotors:
    """Raspberry Pi: gpiozero Motor per side (forward pin, backward pin, PWM enable pin)."""

    def __init__(self, args):
        from gpiozero import Motor
        lf, lb, le = args.gpio_left
        rf, rb, re = args.gpio_right
        self.args = args
        self.lm = Motor(forward=lf, backward=lb, enable=le, pwm=True)
        self.rm = Motor(forward=rf, backward=rb, enable=re, pwm=True)
        self.left = self.right = 0.0

    @staticmethod
    def _drive(m, frac):
        frac = max(-1.0, min(1.0, frac))
        if frac > 0.02:
            m.forward(frac)
        elif frac < -0.02:
            m.backward(-frac)
        else:
            m.stop()

    def set(self, left, right):
        self.left, self.right = left, right
        self._drive(self.lm, left / self.args.max_wheel * (-1 if self.args.invert_left else 1))
        self._drive(self.rm, right / self.args.max_wheel * (-1 if self.args.invert_right else 1))

    def status(self):
        return {}

    def close(self):
        self.lm.stop()
        self.rm.stop()


# ---------------------------------------------------------------- robot logic

class Robot:
    def __init__(self, motors, args):
        self.m = motors
        self.args = args
        self.vel_expires = 0.0
        self.job = None          # asyncio.Task for move / turn
        self.job_seq = None
        self.v = self.w = 0.0
        self.estop = False

    def wheels(self, v, w):
        """Body velocity (m/s, rad/s, + = left) -> wheel speeds, scaled down together if one saturates."""
        b = self.args.wheel_base
        left, right = v - w * b / 2, v + w * b / 2
        peak = max(abs(left), abs(right))
        if peak > self.args.max_wheel:
            k = self.args.max_wheel / peak
            left, right = left * k, right * k
        return left, right

    def drive(self, v, w):
        if self.estop:
            v = w = 0.0
        self.v, self.w = v, w
        self.m.set(*self.wheels(v, w))

    def cancel_job(self):
        if self.job and not self.job.done():
            self.job.cancel()
        self.job = None

    async def run_timed(self, v, w, duration, seq, send):
        try:
            self.drive(v, w)
            await asyncio.sleep(max(0.0, duration))
        finally:
            self.drive(0, 0)
        await send({"type": "done", "seq": seq})

    async def handle(self, msg, send):
        t = msg.get("type")
        seq = msg.get("seq")
        if t == "hello":
            await send({"type": "hello", "proto": PROTO, "name": self.args.name,
                        "caps": ["vel", "move", "turn", "stop", "status"],
                        "wheel_base": self.args.wheel_base, "max_wheel": self.args.max_wheel})
        elif t == "ping":
            await send({"type": "pong", "seq": seq, "t": msg.get("t")})
        elif t == "vel":
            self.cancel_job()
            v = float(msg.get("v", 0.0))
            w = float(msg.get("w", 0.0))
            v = max(-self.args.max_speed, min(self.args.max_speed, v))
            self.drive(v, w)
            self.vel_expires = time.monotonic() + self.args.watchdog
        elif t == "stop":
            self.cancel_job()
            self.vel_expires = 0.0
            self.drive(0, 0)
        elif t == "move":
            self.cancel_job()
            dist = float(msg.get("dist_cm", 0)) / 100
            speed = abs(float(msg.get("speed_cms", 20))) / 100 or 0.2
            speed = min(speed, self.args.max_speed)
            v = math.copysign(speed, dist)
            self.job = asyncio.create_task(self.run_timed(v, 0.0, abs(dist) / speed, seq, send))
        elif t == "turn":
            self.cancel_job()
            ang = math.radians(float(msg.get("deg", 0)))
            rate = math.radians(abs(float(msg.get("speed_dps", 45)))) or math.radians(45)
            w = math.copysign(rate, ang)
            self.job = asyncio.create_task(self.run_timed(0.0, w, abs(ang) / rate, seq, send))
        elif t == "estop":
            self.estop = bool(msg.get("on", True))
            self.cancel_job()
            self.drive(0, 0)
        else:
            await send({"type": "error", "seq": seq, "error": f"unknown type {t!r}"})
            return
        if t in ("vel", "move", "turn", "stop") and self.args.ack:
            await send({"type": "ack", "seq": seq})

    async def watchdog(self):
        while True:
            await asyncio.sleep(0.05)
            job_running = self.job is not None and not self.job.done()
            if not job_running and self.vel_expires and time.monotonic() > self.vel_expires:
                self.vel_expires = 0.0
                if self.v or self.w:
                    print("watchdog: no command for", self.args.watchdog, "s, stopping")
                self.drive(0, 0)

    def status(self):
        s = {"type": "status", "v": round(self.v, 3), "w": round(self.w, 3),
             "left": round(self.m.left, 3), "right": round(self.m.right, 3),
             "busy": self.job is not None and not self.job.done(), "estop": self.estop}
        s.update(self.m.status())
        return s


# ---------------------------------------------------------------- server

def local_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips) or ["127.0.0.1"]


async def main(args):
    backends = {"sim": SimMotors, "serial": SerialMotors, "gpio": GpioMotors}
    robot = Robot(backends[args.backend](args), args)
    clients = set()

    async def handler(ws, path=None):
        clients.add(ws)
        peer = getattr(ws, "remote_address", "?")
        print("phone connected:", peer)

        async def send(obj):
            try:
                await ws.send(json.dumps(obj))
            except Exception:
                pass

        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    await send({"type": "error", "error": "bad json"})
                    continue
                if args.verbose and msg.get("type") != "ping":
                    print("<-", msg)
                await robot.handle(msg, send)
        except websockets.ConnectionClosed:
            pass
        finally:
            clients.discard(ws)
            robot.cancel_job()
            robot.drive(0, 0)
            print("phone disconnected, motors stopped")

    async def status_loop():
        while True:
            await asyncio.sleep(1.0 / args.status_hz)
            if clients:
                msg = json.dumps(robot.status())
                for c in list(clients):
                    try:
                        await c.send(msg)
                    except Exception:
                        pass
            if args.backend == "sim" and args.verbose and (robot.v or robot.w):
                print("sim:", robot.status())

    async with websockets.serve(handler, args.host, args.port):
        for ip in local_ips():
            print(f"R2S robot receiver ({args.backend}) on ws://{ip}:{args.port}/robot")
        await asyncio.gather(robot.watchdog(), status_loop())


def parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", choices=["sim", "serial", "gpio"], default="sim")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8777)
    p.add_argument("--name", default="R2S diff-drive")
    p.add_argument("--wheel-base", type=float, default=0.30, help="distance between the wheels, m")
    p.add_argument("--max-wheel", type=float, default=0.50, help="wheel speed at full power, m/s")
    p.add_argument("--max-speed", type=float, default=0.50, help="cap on commanded forward speed, m/s")
    p.add_argument("--watchdog", type=float, default=0.5, help="stop if no vel for this long, s")
    p.add_argument("--status-hz", type=float, default=2.0)
    p.add_argument("--ack", action="store_true", help="answer every motion command with an ack")
    p.add_argument("--serial", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--gpio-left", type=int, nargs=3, default=[17, 27, 12], metavar=("FWD", "BACK", "PWM"))
    p.add_argument("--gpio-right", type=int, nargs=3, default=[23, 24, 13], metavar=("FWD", "BACK", "PWM"))
    p.add_argument("--invert-left", action="store_true")
    p.add_argument("--invert-right", action="store_true")
    p.add_argument("-v", "--verbose", action="store_true")
    return p.parse_args(argv)


if __name__ == "__main__":
    try:
        asyncio.run(main(parse()))
    except KeyboardInterrupt:
        pass
