"""RuView ESP32 CSI node packets (firmware/esp32-csi-node, UDP, default port 5005).

Pure Python (no Home Assistant imports) so it is unit tested on its own. Packet layouts follow the firmware
README (ADR-018 CSI frame, edge vitals packet); multi-byte fields are little endian (ESP32).

    0xC5110001  CSI frame   ~20 Hz  magic, node, n_antennas, n_subcarriers u16, freq_mhz u32, seq u32, rssi i8,
                                    noise i8, ppdu, flags, then I/Q int8 pairs
    0xC5110002  vitals      1 Hz    magic, node, flags (bit0 presence, bit1 fall, bit2 motion), breathing u16
                                    (bpm x 100), heart u32 (bpm x 10000), rssi i8, n_persons u8, reserved u16,
                                    motion_energy f32, presence_score f32, timestamp_ms u32, reserved u32
    0xC5110004  WASM output, event driven (counted, not decoded)
"""
from __future__ import annotations

import asyncio
import logging
import struct
import time
from dataclasses import dataclass, field
from typing import Callable

_LOGGER = logging.getLogger(__name__)

MAGIC_CSI = 0xC5110001
MAGIC_VITALS = 0xC5110002
MAGIC_WASM = 0xC5110004

_VITALS = struct.Struct("<IBBHIbBHffII")   # 32 bytes
_CSI_HDR = struct.Struct("<IBBHIIbbBB")    # 20 bytes


@dataclass
class Vitals:
    node_id: int
    presence: bool
    fall: bool
    motion: bool
    breathing_bpm: float | None
    heart_bpm: float | None
    rssi: int
    n_persons: int
    motion_energy: float
    presence_score: float
    timestamp_ms: int


@dataclass
class CsiFrame:
    node_id: int
    antennas: int
    subcarriers: int
    freq_mhz: int
    seq: int
    rssi: int
    noise_floor: int
    flags: int


def _magic(data: bytes) -> int | None:
    if len(data) < 4:
        return None
    le = struct.unpack_from("<I", data)[0]
    if le in (MAGIC_CSI, MAGIC_VITALS, MAGIC_WASM):
        return le
    be = struct.unpack_from(">I", data)[0]       # tolerate a big-endian magic from other senders
    if be in (MAGIC_CSI, MAGIC_VITALS, MAGIC_WASM):
        return be
    return None


def parse(data: bytes) -> Vitals | CsiFrame | tuple[str, int] | None:
    """One UDP datagram -> Vitals, CsiFrame, ("wasm", node_id) or None (unknown / truncated)."""
    magic = _magic(data)
    if magic == MAGIC_VITALS and len(data) >= _VITALS.size:
        (_, node, flags, br, hr, rssi, n, _r1, energy, score, ts, _r2) = _VITALS.unpack_from(data)
        # 0 = not measured; the firmware reports 6..30 bpm breathing and 40..120 bpm heart rate
        breathing = br / 100.0 if br else None
        heart = hr / 10000.0 if hr else None
        if breathing is not None and not 3 <= breathing <= 60:
            breathing = None
        if heart is not None and not 30 <= heart <= 220:
            heart = None
        return Vitals(node, bool(flags & 1), bool(flags & 2), bool(flags & 4), breathing, heart, rssi, n,
                      float(energy), float(score), ts)
    if magic == MAGIC_CSI and len(data) >= _CSI_HDR.size:
        (_, node, ant, sub, freq, seq, rssi, noise, _ppdu, flags) = _CSI_HDR.unpack_from(data)
        return CsiFrame(node, ant, sub, freq, seq, rssi, noise, flags)
    if magic == MAGIC_WASM and len(data) >= 5:
        return ("wasm", data[4])
    return None


def build_vitals(node_id: int, *, presence=True, fall=False, motion=False, breathing=14.0, heart=68.0, rssi=-50,
                 n_persons=1, motion_energy=0.1, presence_score=0.9, timestamp_ms=0) -> bytes:
    """A vitals packet as the firmware sends it (tests and tools/ruview_sim.py)."""
    flags = (1 if presence else 0) | (2 if fall else 0) | (4 if motion else 0)
    return _VITALS.pack(MAGIC_VITALS, node_id, flags, int(round((breathing or 0) * 100)), int(round((heart or 0) * 10000)),
                        rssi, n_persons, 0, motion_energy, presence_score, timestamp_ms, 0)


def build_csi(node_id: int, seq: int, rssi: int = -50, subcarriers: int = 64) -> bytes:
    return _CSI_HDR.pack(MAGIC_CSI, node_id, 1, subcarriers, 2437, seq, rssi, -95, 0, 0) + bytes(subcarriers * 2)


@dataclass
class NodeState:
    """Latest state of one CSI node (what the entities and the presence engine read)."""
    node_id: int
    vitals: Vitals | None = None
    vitals_at: float = 0.0
    last_fall_at: float = 0.0
    csi_frames: int = 0
    csi_rate: float = 0.0
    csi_rssi: int | None = None
    last_seen: float = 0.0
    source: str = ""
    _rate_t0: float = field(default=0.0, repr=False)
    _rate_n: int = field(default=0, repr=False)

    def fresh(self, now: float, max_age: float = 10.0) -> bool:
        return self.vitals is not None and now - self.vitals_at <= max_age


class RuViewHub:
    """Keeps NodeState per node id; feed it datagrams (UDP listener) or decoded entities (MQTT bridge)."""

    def __init__(self, on_update: Callable[[int, bool], None] | None = None, clock: Callable[[], float] = time.time):
        self.nodes: dict[int, NodeState] = {}
        self.on_update = on_update
        self.clock = clock
        self.packets = 0
        self.unknown = 0

    def handle(self, data: bytes, addr: str = "") -> None:
        self.packets += 1
        pkt = parse(data)
        if pkt is None:
            self.unknown += 1
            return
        now = self.clock()
        node_id = pkt.node_id if isinstance(pkt, (Vitals, CsiFrame)) else pkt[1]
        new = node_id not in self.nodes
        st = self.nodes.setdefault(node_id, NodeState(node_id))
        st.last_seen = now
        st.source = addr
        if isinstance(pkt, Vitals):
            st.vitals = pkt
            st.vitals_at = now
            if pkt.fall:
                st.last_fall_at = now
            if self.on_update:
                self.on_update(node_id, new)
        elif isinstance(pkt, CsiFrame):
            st.csi_frames += 1
            st.csi_rssi = pkt.rssi
            if st._rate_t0 == 0:
                st._rate_t0 = now
            st._rate_n += 1
            if now - st._rate_t0 >= 5:
                st.csi_rate = st._rate_n / (now - st._rate_t0)
                st._rate_t0, st._rate_n = now, 0
            if new and self.on_update:
                self.on_update(node_id, True)


class _Protocol(asyncio.DatagramProtocol):
    def __init__(self, hub: RuViewHub, forward: tuple[str, int] | None):
        self.hub = hub
        self.forward = forward
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        try:
            self.hub.handle(data, addr[0])
        except Exception:  # noqa: BLE001 - a bad packet must not stop the listener
            _LOGGER.exception("RuView packet from %s", addr)
        if self.forward and self.transport:
            # pass everything on to a RuView sensing server so its own UI / pose model keeps working
            self.transport.sendto(data, self.forward)


async def listen(hub: RuViewHub, host: str, port: int, forward: tuple[str, int] | None = None):
    """Starts the UDP listener; returns the transport (close() it to stop)."""
    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(lambda: _Protocol(hub, forward), local_addr=(host, port))
    return transport
