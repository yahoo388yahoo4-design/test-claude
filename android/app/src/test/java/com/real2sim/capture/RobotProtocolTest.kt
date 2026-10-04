package com.real2sim.capture

import org.junit.Assert.assertEquals
import org.junit.Test

/** Message shapes must match robot/PROTOCOL.md and robot/receiver.py (the iPhone app sends the same). */
class RobotProtocolTest {
    @Test fun velUsesMetresAndRadians() {
        val m = RobotProtocol.vel(0.25, -0.1234567, 42)
        assertEquals("vel", m.getString("type")); assertEquals(0.25, m.getDouble("v"), 0.0)
        assertEquals(-0.123, m.getDouble("w"), 1e-9); assertEquals(42, m.getInt("seq"))
    }

    @Test fun moveTurnStopEstopHelloPing() {
        val mv = RobotProtocol.move(-30.0, 15.0, 1)
        assertEquals(setOf("type", "dist_cm", "speed_cms", "seq"), mv.keys().asSequence().toSet())
        assertEquals(-30.0, mv.getDouble("dist_cm"), 0.0)
        val t = RobotProtocol.turn(90.0, 45.0, 2)
        assertEquals(setOf("type", "deg", "speed_dps", "seq"), t.keys().asSequence().toSet())
        assertEquals("stop", RobotProtocol.stop(3).getString("type"))
        val e = RobotProtocol.estop(false); assertEquals("estop", e.getString("type")); assertEquals(false, e.getBoolean("on"))
        val h = RobotProtocol.hello("x"); assertEquals(1, h.getInt("proto")); assertEquals("x", h.getString("client"))
        val p = RobotProtocol.ping(4, 12.5); assertEquals(12.5, p.getDouble("t"), 0.0); assertEquals(4, p.getInt("seq"))
        assertEquals(0.0, RobotProtocol.vel(Double.NaN, 0.0, 5).getDouble("v"), 0.0)   // never NaN on the wire
    }

    @Test fun bleLinesSplitAcrossPackets() {
        val s = RobotProtocol.LineSplitter()
        assertEquals(emptyList<String>(), s.feed("{\"type\":\"po".toByteArray()))
        assertEquals(listOf("{\"type\":\"pong\"}", "{\"a\":1}"), s.feed("ng\"}\n{\"a\":1}\n{\"b".toByteArray()))
        assertEquals(listOf("{\"b\":2}"), s.feed("\":2}\n".toByteArray()))
    }
}
