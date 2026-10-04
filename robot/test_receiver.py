"""End-to-end tests of the reference receiver over a real WebSocket, sim backend (pytest)."""
import asyncio
import json
import math

import pytest
import websockets

from receiver import Robot, Server, SimBackend


async def _with_server(fn, **robot_kw):
    be = SimBackend(18.0, lag_s=0.05)
    robot = Robot(be, "test-bot", 18.0, robot_kw.get("max_speed", 40.0), robot_kw.get("max_turn", 120.0))
    srv = Server(robot, token=robot_kw.get("token"), verbose=False)
    ready = asyncio.Event()
    port = 18766
    task = asyncio.create_task(srv.serve("127.0.0.1", port, ready))
    await ready.wait()
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}/r2s") as ws:
            await fn(ws, robot)
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


async def _recv_until(ws, pred, timeout=10.0):
    async def go():
        while True:
            m = json.loads(await ws.recv())
            if pred(m):
                return m
    return await asyncio.wait_for(go(), timeout)


async def _hello(ws, token=None):
    h = {"type": "hello", "v": 1, "client": "test"}
    if token:
        h["token"] = token
    await ws.send(json.dumps(h))
    return await _recv_until(ws, lambda m: m["type"] == "hello")


def run(coro):
    asyncio.run(coro)


def test_hello_and_ping():
    async def body(ws, robot):
        h = await _hello(ws)
        assert h["v"] == 1 and h["robot"] == "test-bot" and h["max_speed_cms"] == 40.0
        await ws.send(json.dumps({"type": "ping", "t": 12.5}))
        p = await _recv_until(ws, lambda m: m["type"] == "pong")
        assert p["t"] == 12.5
    run(_with_server(body))


def test_move_forward_and_back():
    async def body(ws, robot):
        await _hello(ws)
        await ws.send(json.dumps({"type": "move", "id": 7, "dist_cm": 20, "speed_cms": 20}))
        assert (await _recv_until(ws, lambda m: m["type"] == "ack"))["id"] == 7
        d = await _recv_until(ws, lambda m: m["type"] == "done")
        assert d == {"type": "done", "id": 7, "ok": True}
        o = await _recv_until(ws, lambda m: m["type"] == "odom")
        assert abs(o["x_cm"] - 20) < 1.5 and abs(o["y_cm"]) < 0.5
        await ws.send(json.dumps({"type": "move", "id": 8, "dist_cm": -10, "speed_cms": 20}))
        await _recv_until(ws, lambda m: m["type"] == "done" and m["id"] == 8)
        await asyncio.sleep(0.2)
        assert abs(robot.x - 10) < 1.5
    run(_with_server(body))


def test_turn_left_positive():
    async def body(ws, robot):
        await _hello(ws)
        await ws.send(json.dumps({"type": "turn", "id": 1, "angle_deg": 90, "speed_dps": 90}))
        await _recv_until(ws, lambda m: m["type"] == "done" and m["id"] == 1)
        await asyncio.sleep(0.2)
        assert abs(math.degrees(robot.th) - 90) < 3
        # forward after a left turn goes +y
        await ws.send(json.dumps({"type": "move", "id": 2, "dist_cm": 10, "speed_cms": 20}))
        await _recv_until(ws, lambda m: m["type"] == "done" and m["id"] == 2)
        assert robot.y > 8 and abs(robot.x) < 2
    run(_with_server(body))


def test_vel_watchdog_and_clamp():
    async def body(ws, robot):
        await _hello(ws)
        await ws.send(json.dumps({"type": "vel", "v_cms": 500, "w_dps": 0, "ttl_ms": 5000}))
        await asyncio.sleep(0.3)
        assert robot.be.cl <= 40.0 + 1e-6              # clamped to max speed
        await asyncio.sleep(0.6)                       # ttl capped at 500 ms -> stopped
        assert robot.be.cl == 0.0 and robot.be.cr == 0.0
    run(_with_server(body))


def test_estop_latches_until_reset():
    async def body(ws, robot):
        await _hello(ws)
        await ws.send(json.dumps({"type": "move", "id": 3, "dist_cm": 100, "speed_cms": 10}))
        await _recv_until(ws, lambda m: m["type"] == "ack")
        await asyncio.sleep(0.2)
        await ws.send(json.dumps({"type": "estop"}))
        d = await _recv_until(ws, lambda m: m["type"] == "done" and m["id"] == 3)
        assert d["ok"] is False and d["reason"] == "estop"
        await ws.send(json.dumps({"type": "move", "id": 4, "dist_cm": 10, "speed_cms": 10}))
        d = await _recv_until(ws, lambda m: m["type"] == "done" and m["id"] == 4)
        assert d["ok"] is False
        st = await _recv_until(ws, lambda m: m["type"] == "status")
        assert st["estop"] is True
        await ws.send(json.dumps({"type": "reset"}))
        await ws.send(json.dumps({"type": "move", "id": 5, "dist_cm": 5, "speed_cms": 10}))
        d = await _recv_until(ws, lambda m: m["type"] == "done" and m["id"] == 5)
        assert d["ok"] is True
    run(_with_server(body))


def test_stop_on_disconnect_and_token():
    async def body(ws, robot):
        await ws.send(json.dumps({"type": "move", "id": 1, "dist_cm": 5, "speed_cms": 5}))
        e = await _recv_until(ws, lambda m: m["type"] == "error")
        assert "hello" in e["reason"]
        await _hello(ws, token="s3cret")
        await ws.send(json.dumps({"type": "vel", "v_cms": 20, "w_dps": 0, "ttl_ms": 400}))
    run(_with_server(body, token="s3cret"))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
