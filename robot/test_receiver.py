#!/usr/bin/env python3
"""End-to-end test of robot/receiver.py with the sim backend, speaking the app's protocol.

  pip install websockets && python3 robot/test_receiver.py
"""
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import receiver  # noqa: E402
import websockets  # noqa: E402

PORT = 18777
fails = 0


def check(cond, msg):
    global fails
    print(("  ok  " if cond else "FAIL  ") + msg)
    if not cond:
        fails += 1


async def recv_type(ws, t, timeout=3.0):
    end = asyncio.get_event_loop().time() + timeout
    while True:
        left = end - asyncio.get_event_loop().time()
        if left <= 0:
            return None
        try:
            m = json.loads(await asyncio.wait_for(ws.recv(), left))
        except asyncio.TimeoutError:
            return None
        if m.get("type") == t:
            return m


async def fresh_status(ws, settle=0.15):
    """Drops buffered messages, then returns the next status (sent after the drop)."""
    await asyncio.sleep(settle)
    while True:
        try:
            await asyncio.wait_for(ws.recv(), 0.01)
        except asyncio.TimeoutError:
            break
    return await recv_type(ws, "status")


async def run():
    args = receiver.parse(["--backend", "sim", "--port", str(PORT), "--status-hz", "10"])
    server = asyncio.create_task(receiver.main(args))
    await asyncio.sleep(0.5)
    async with websockets.connect(f"ws://127.0.0.1:{PORT}/robot") as ws:
        await ws.send(json.dumps({"type": "hello", "proto": 1, "client": "test"}))
        h = await recv_type(ws, "hello")
        check(h is not None and "vel" in h["caps"], "hello answered with capabilities")

        await ws.send(json.dumps({"type": "ping", "seq": 1, "t": 123.5}))
        p = await recv_type(ws, "pong")
        check(p is not None and p["t"] == 123.5 and p["seq"] == 1, "ping echoes t and seq")

        await ws.send(json.dumps({"type": "vel", "v": 0.2, "w": 0.0, "seq": 2}))
        s = await fresh_status(ws)
        check(s is not None and abs(s["left"] - 0.2) < 1e-6 and abs(s["right"] - 0.2) < 1e-6, "vel sets both wheels")
        await asyncio.sleep(0.9)  # no refresh: the watchdog must stop the robot
        s = await fresh_status(ws)
        check(s is not None and s["v"] == 0 and s["left"] == 0, "watchdog stops after 0.5 s without vel")

        await ws.send(json.dumps({"type": "vel", "v": 0.0, "w": 1.0, "seq": 3}))
        s = await fresh_status(ws)
        check(s is not None and s["left"] < 0 < s["right"], "w > 0 turns left (right wheel forward)")
        await ws.send(json.dumps({"type": "stop", "seq": 4}))

        before = (await fresh_status(ws))["sim_pose"]
        await ws.send(json.dumps({"type": "move", "dist_cm": 20, "speed_cms": 20, "seq": 5}))
        d = await recv_type(ws, "done", timeout=3)
        after = (await fresh_status(ws))["sim_pose"]
        moved = ((after[0] - before[0]) ** 2 + (after[1] - before[1]) ** 2) ** 0.5
        check(d is not None and d["seq"] == 5, "move reports done")
        check(abs(moved - 0.20) < 0.03, f"move 20 cm drove {moved * 100:.1f} cm (sim)")

        await ws.send(json.dumps({"type": "turn", "deg": 90, "speed_dps": 90, "seq": 6}))
        d = await recv_type(ws, "done", timeout=3)
        after2 = (await fresh_status(ws))["sim_pose"]
        turned = after2[2] - after[2]
        check(d is not None and abs(turned - 90) < 8, f"turn 90 deg turned {turned:.1f} deg (sim)")

        await ws.send(json.dumps({"type": "move", "dist_cm": 100, "speed_cms": 10, "seq": 7}))
        await asyncio.sleep(0.3)
        await ws.send(json.dumps({"type": "stop", "seq": 8}))
        s = await fresh_status(ws)
        check(s is not None and not s["busy"] and s["v"] == 0, "stop cancels a running move")

        await ws.send(json.dumps({"type": "bogus", "seq": 9}))
        e = await recv_type(ws, "error")
        check(e is not None, "unknown type answered with an error")
    server.cancel()
    print("\nALL TESTS PASSED" if fails == 0 else f"\n{fails} FAILURE(S)")
    return fails


if __name__ == "__main__":
    sys.exit(1 if asyncio.run(run()) else 0)
