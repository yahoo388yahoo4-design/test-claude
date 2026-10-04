#!/usr/bin/env python3
"""Reference robot receiver for the r2s capture apps' navigation mode (robot/PROTOCOL.md).

Runs on the robot (Raspberry Pi, Jetson, laptop on the robot, ...) as a WebSocket server. The phone
connects to ws://<robot-ip>:8766/r2s and sends move / turn / vel / stop / estop commands; the
receiver turns them into wheel speeds for a two-wheeled (differential drive) base and streams
odometry back.

Backends:
  --backend sim      no hardware: integrates the commanded wheel speeds (default; for testing)
  --backend serial   an Arduino / ESP32 motor driver on a serial port (robot/arduino/r2s_diffdrive):
                     sends "V <left_cms> <right_cms>\\n" at 50 Hz, reads "E <left_ticks> <right_ticks>\\n"
                     encoder lines if the board has encoders (else odometry is open loop)

  python receiver.py --backend sim
  python receiver.py --backend serial --serial-port /dev/ttyUSB0 --wheel-base 18 --ticks-per-cm 20

Safety: speeds are clamped to --max-speed / --max-turn, a "vel" command expires after its ttl
(at most 500 ms), the robot stops when the controlling phone disconnects, and "estop" latches
until "reset".
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from dataclasses import dataclass, field

import websockets

PROTOCOL_VERSION = 1
VEL_TTL_MAX_S = 0.5


# ---------------------------------------------------------------------------------------- backends
class SimBackend:
    """Integrates commanded wheel speeds (optionally with a first-order lag) into odometry."""

    def __init__(self, wheel_base_cm: float, lag_s: float = 0.0):
        self.b = wheel_base_cm
        self.lag = lag_s
        self.vl = self.vr = 0.0          # actual wheel speeds, cm/s
        self.cl = self.cr = 0.0          # commanded
        self.dist_l = self.dist_r = 0.0  # wheel travel, cm

    def set_wheels(self, vl: float, vr: float):
        self.cl, self.cr = vl, vr

    def update(self, dt: float):
        a = 1.0 if self.lag <= 0 else min(1.0, dt / self.lag)
        self.vl += (self.cl - self.vl) * a
        self.vr += (self.cr - self.vr) * a
        self.dist_l += self.vl * dt
        self.dist_r += self.vr * dt

    def wheel_travel(self):
        return self.dist_l, self.dist_r

    def battery_v(self):
        return None

    def close(self):
        pass


class SerialBackend(SimBackend):
    """Arduino/ESP32 over serial. Uses encoder ticks when the board reports them, else open loop."""

    def __init__(self, port: str, baud: int, wheel_base_cm: float, ticks_per_cm: float):
        super().__init__(wheel_base_cm)
        import serial  # pyserial
        self.ser = serial.Serial(port, baud, timeout=0)
        self.tpc = ticks_per_cm
        self.ticks0 = None
        self.ticks = None
        self.batt = None
        self.buf = b""

    def set_wheels(self, vl: float, vr: float):
        super().set_wheels(vl, vr)
        self.ser.write(f"V {vl:.1f} {vr:.1f}\n".encode())

    def update(self, dt: float):
        self.buf += self.ser.read(4096) or b""
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            parts = line.decode(errors="ignore").split()
            if len(parts) == 3 and parts[0] == "E":
                t = (int(parts[1]), int(parts[2]))
                if self.ticks0 is None:
                    self.ticks0 = t
                self.ticks = t
            elif len(parts) == 2 and parts[0] == "B":
                self.batt = float(parts[1])
        if self.ticks is None:
            super().update(dt)                       # open loop

    def wheel_travel(self):
        if self.ticks is None:
            return super().wheel_travel()
        return ((self.ticks[0] - self.ticks0[0]) / self.tpc, (self.ticks[1] - self.ticks0[1]) / self.tpc)

    def battery_v(self):
        return self.batt

    def close(self):
        try:
            self.ser.write(b"V 0 0\n")
            self.ser.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------------------- robot
@dataclass
class Cmd:
    id: int
    kind: str              # "move" | "turn"
    target: float          # cm or deg (signed)
    speed: float           # cm/s or deg/s (positive)
    start: tuple = field(default_factory=tuple)
    t0: float = 0.0
    timeout: float = 0.0


class Robot:
    def __init__(self, backend, name: str, wheel_base_cm: float, max_speed: float, max_turn: float):
        self.be = backend
        self.name = name
        self.b = wheel_base_cm
        self.max_speed = max_speed
        self.max_turn = max_turn
        self.x = self.y = self.th = 0.0          # cm, cm, rad (CCW positive)
        self.v = self.w = 0.0                    # measured cm/s, rad/s
        self.last_travel = (0.0, 0.0)
        self.queue: list[Cmd] = []
        self.active: Cmd | None = None
        self.vel = None                          # (v, w_deg, expires)
        self.estop = False
        self.events: list[dict] = []             # outgoing ack/done messages

    # ---- commands ----
    def handle(self, m: dict) -> list[dict]:
        t = m.get("type")
        out = []
        if t == "move" or t == "turn":
            cid = int(m.get("id", 0))
            if self.estop:
                return [{"type": "done", "id": cid, "ok": False, "reason": "estop latched"}]
            if t == "move":
                tgt = float(m["dist_cm"]); spd = min(abs(float(m.get("speed_cms", 15))), self.max_speed)
            else:
                tgt = float(m["angle_deg"]); spd = min(abs(float(m.get("speed_dps", 45))), self.max_turn)
            if spd <= 0:
                return [{"type": "done", "id": cid, "ok": False, "reason": "zero speed"}]
            self.vel = None
            self.queue.append(Cmd(cid, t, tgt, spd))
            out.append({"type": "ack", "id": cid})
        elif t == "vel":
            if not self.estop:
                ttl = min(float(m.get("ttl_ms", 300)) / 1000.0, VEL_TTL_MAX_S)
                v = max(-self.max_speed, min(self.max_speed, float(m.get("v_cms", 0))))
                w = max(-self.max_turn, min(self.max_turn, float(m.get("w_dps", 0))))
                self.cancel("superseded by vel")
                self.vel = (v, w, time.monotonic() + ttl)
        elif t == "stop":
            self.cancel("stopped")
            self.vel = None
            if "id" in m:
                out.append({"type": "done", "id": int(m["id"]), "ok": True})
        elif t == "estop":
            self.estop = True
            self.cancel("estop")
            self.vel = None
            self.be.set_wheels(0.0, 0.0)
        elif t == "reset":
            self.estop = False
        return out

    def cancel(self, reason: str):
        for c in ([self.active] if self.active else []) + self.queue:
            self.events.append({"type": "done", "id": c.id, "ok": False, "reason": reason})
        self.active = None
        self.queue.clear()

    # ---- control loop ----
    def step(self, dt: float):
        self.be.update(dt)
        l, r = self.be.wheel_travel()
        dl, dr = l - self.last_travel[0], r - self.last_travel[1]
        self.last_travel = (l, r)
        dc = (dl + dr) / 2
        dth = (dr - dl) / self.b
        self.x += dc * math.cos(self.th + dth / 2)
        self.y += dc * math.sin(self.th + dth / 2)
        self.th += dth
        if dt > 0:
            self.v = 0.7 * self.v + 0.3 * dc / dt
            self.w = 0.7 * self.w + 0.3 * dth / dt

        v = w_deg = 0.0
        now = time.monotonic()
        if self.estop:
            pass
        elif self.vel is not None:
            if now > self.vel[2]:
                self.vel = None                   # watchdog: stale velocity command
            else:
                v, w_deg = self.vel[0], self.vel[1]
        else:
            if self.active is None and self.queue:
                c = self.queue.pop(0)
                c.start = (self.x, self.y, self.th)
                c.t0 = now
                c.timeout = 3.0 * abs(c.target) / c.speed + 2.0
                self.active = c
            c = self.active
            if c is not None:
                if c.kind == "move":
                    done = math.hypot(self.x - c.start[0], self.y - c.start[1])
                    rem = abs(c.target) - done
                    # slow down over the last 5 cm
                    v = math.copysign(min(c.speed, max(3.0, rem * 2.0)), c.target) if rem > 0.3 else 0.0
                else:
                    turned = math.degrees(self.th - c.start[2])
                    rem = abs(c.target) - abs(turned)
                    w_deg = math.copysign(min(c.speed, max(10.0, rem * 2.0)), c.target) if rem > 0.5 else 0.0
                finished = (c.kind == "move" and v == 0.0) or (c.kind == "turn" and w_deg == 0.0)
                if finished:
                    self.events.append({"type": "done", "id": c.id, "ok": True})
                    self.active = None
                elif now - c.t0 > c.timeout:
                    self.events.append({"type": "done", "id": c.id, "ok": False, "reason": "timeout"})
                    self.active = None
                    v = w_deg = 0.0
        w = math.radians(w_deg)
        vl = v - w * self.b / 2
        vr = v + w * self.b / 2
        # keep both wheels inside the speed limit, preserving the curvature
        s = max(abs(vl), abs(vr)) / self.max_speed if self.max_speed > 0 else 1.0
        if s > 1.0:
            vl /= s
            vr /= s
        self.be.set_wheels(vl, vr)

    def odom(self) -> dict:
        return {"type": "odom", "t": round(time.monotonic(), 3), "x_cm": round(self.x, 2), "y_cm": round(self.y, 2),
                "theta_deg": round(math.degrees(self.th), 2), "v_cms": round(self.v, 2), "w_dps": round(math.degrees(self.w), 2)}

    def status(self) -> dict:
        return {"type": "status", "estop": self.estop, "queue": len(self.queue) + (1 if self.active else 0),
                "battery_v": self.be.battery_v()}


# ---------------------------------------------------------------------------------------- server
class Server:
    def __init__(self, robot: Robot, token: str | None = None, rate_hz: float = 50.0, verbose: bool = True):
        self.robot = robot
        self.token = token
        self.rate = rate_hz
        self.clients: set = set()
        self.verbose = verbose

    def log(self, *a):
        if self.verbose:
            print(time.strftime("%H:%M:%S"), *a, flush=True)

    async def send_all(self, msg: dict):
        data = json.dumps(msg)
        for ws in list(self.clients):
            try:
                await ws.send(data)
            except Exception:
                self.clients.discard(ws)

    async def handler(self, ws):
        authed = self.token is None
        self.log("phone connected", getattr(ws, "remote_address", ""))
        try:
            async for raw in ws:
                try:
                    m = json.loads(raw)
                except ValueError:
                    await ws.send(json.dumps({"type": "error", "reason": "bad json"}))
                    continue
                t = m.get("type")
                if t == "hello":
                    if self.token is not None and m.get("token") != self.token:
                        await ws.send(json.dumps({"type": "error", "reason": "bad token"}))
                        await ws.close()
                        return
                    authed = True
                    self.clients.add(ws)
                    await ws.send(json.dumps({"type": "hello", "v": PROTOCOL_VERSION, "robot": self.robot.name,
                                              "wheel_base_cm": self.robot.b, "max_speed_cms": self.robot.max_speed,
                                              "max_turn_dps": self.robot.max_turn}))
                    self.log("hello from", m.get("client"))
                    continue
                if not authed:
                    await ws.send(json.dumps({"type": "error", "reason": "send hello first"}))
                    continue
                if t == "ping":
                    await ws.send(json.dumps({"type": "pong", "t": m.get("t")}))
                    continue
                if t != "vel":
                    self.log("cmd", m)
                for reply in self.robot.handle(m):
                    await ws.send(json.dumps(reply))
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)
            if not self.clients:
                self.robot.handle({"type": "stop"})
            self.log("phone disconnected")

    async def loop(self):
        dt = 1.0 / self.rate
        k = 0
        last = time.monotonic()
        while True:
            await asyncio.sleep(dt)
            now = time.monotonic()
            self.robot.step(now - last)
            last = now
            k += 1
            for e in self.robot.events:
                await self.send_all(e)
            self.robot.events.clear()
            if k % max(1, int(self.rate / 20)) == 0:
                await self.send_all(self.robot.odom())
            if k % max(1, int(self.rate / 2)) == 0:
                await self.send_all(self.robot.status())

    async def serve(self, host: str, port: int, ready: asyncio.Event | None = None):
        async with websockets.serve(self.handler, host, port):
            self.log(f"r2s robot receiver on ws://{host}:{port}/r2s ({self.robot.name})")
            if ready:
                ready.set()
            await self.loop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8766)
    ap.add_argument("--name", default="r2s-diffdrive")
    ap.add_argument("--token", default=None, help="require this token in the phone's hello")
    ap.add_argument("--backend", choices=["sim", "serial"], default="sim")
    ap.add_argument("--serial-port", "--port-dev", dest="serial_port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wheel-base", type=float, default=18.0, help="distance between the wheels, cm")
    ap.add_argument("--ticks-per-cm", type=float, default=20.0)
    ap.add_argument("--max-speed", type=float, default=40.0, help="cm/s")
    ap.add_argument("--max-turn", type=float, default=120.0, help="deg/s")
    ap.add_argument("--sim-lag", type=float, default=0.1, help="sim wheel response time constant, s")
    a = ap.parse_args()
    be = SimBackend(a.wheel_base, a.sim_lag) if a.backend == "sim" else SerialBackend(a.serial_port, a.baud, a.wheel_base, a.ticks_per_cm)
    robot = Robot(be, a.name, a.wheel_base, a.max_speed, a.max_turn)
    try:
        asyncio.run(Server(robot, a.token).serve(a.host, a.port))
    except KeyboardInterrupt:
        pass
    finally:
        be.set_wheels(0.0, 0.0)
        be.close()


if __name__ == "__main__":
    main()
