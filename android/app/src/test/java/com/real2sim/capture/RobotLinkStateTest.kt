package com.real2sim.capture

import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.RuntimeEnvironment
import org.robolectric.annotation.Config

/**
 * RobotLink's connection / open-loop motion bookkeeping, driven through a fake transport (connectCustom):
 * a move / turn stays pending (NavActions.busy()) until the robot answers done for that seq, or the
 * command is cancelled by stop, e-stop or a lost link.
 */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [35])
class RobotLinkStateTest {
    /** Records everything the link sends; `robot` feeds a robot -> phone line back. */
    private class Fake : RobotTransport {
        val sent = ArrayList<JSONObject>()
        var closed = false
        lateinit var robot: (String) -> Unit
        override fun send(json: String): Boolean { sent.add(JSONObject(json)); return true }
        override fun close() { closed = true }
        fun last() = sent.last()
        fun types() = sent.map { it.getString("type") }
    }

    private fun connected(): Pair<RobotLink, Fake> {
        val events = ArrayList<String>()
        val link = RobotLink(RuntimeEnvironment.getApplication()) { events.add(it) }
        val fake = Fake()
        link.connectCustom("fake") { incoming -> fake.robot = incoming; fake }
        assertEquals(listOf("hello"), fake.types())
        assertFalse("not connected until the robot answers", link.connected)
        assertFalse("nothing is sent before hello is answered", link.move(10f, 5f))
        assertEquals(0, link.pendingSeq)
        fake.robot("""{"type":"hello","name":"testbot","caps":["move"]}""")
        assertTrue(link.connected); assertEquals("testbot", link.robotName); assertEquals("move", link.caps)
        return link to fake
    }

    @Test fun moveStaysPendingUntilItsDone() {
        val (link, fake) = connected()
        assertFalse(link.motionPending)
        assertTrue(link.move(100f, 20f))
        val seq = fake.last().getInt("seq")
        assertEquals("move", fake.last().getString("type"))
        assertEquals(seq, link.pendingSeq); assertTrue(link.motionPending)
        fake.robot("""{"type":"done","seq":${seq - 1}}""")      // an older command finishing does not clear it
        assertEquals(seq, link.pendingSeq)
        fake.robot("""{"type":"ack","seq":$seq}""")
        assertTrue(link.motionPending)
        fake.robot("""{"type":"done","seq":$seq}""")
        assertEquals(0, link.pendingSeq); assertFalse(link.motionPending)
        link.disconnect()
    }

    @Test fun turnIsClearedByStopEstopAndError() {
        val (link, fake) = connected()
        link.turn(90f, 45f); assertTrue(link.motionPending)
        link.stop(); assertEquals(0, link.pendingSeq); assertEquals("stop", fake.last().getString("type"))

        link.turn(-90f, 45f); assertTrue(link.motionPending)
        fake.robot("""{"type":"estop"}""")
        assertTrue(link.robotEstop); assertFalse(link.motionPending)

        link.turn(45f, 30f); val s = link.pendingSeq; assertTrue(s != 0)
        fake.robot("""{"type":"error","error":"bad json"}""")      // no seq: not about our command
        assertEquals(s, link.pendingSeq)
        fake.robot("""{"type":"error","seq":$s,"error":"unsupported"}""")
        assertEquals(0, link.pendingSeq)

        link.turn(10f, 30f); assertTrue(link.motionPending)
        link.estop(true)
        assertEquals(0, link.pendingSeq)
        assertEquals(listOf("stop", "estop"), fake.types().takeLast(2))
        link.disconnect()
    }

    @Test fun robotBusyCountsAsMotionAndDisconnectClearsEverything() {
        val (link, fake) = connected()
        fake.robot("""{"type":"status","busy":true,"estop":false,"v":0.1,"w":0.0}""")
        assertTrue(link.busy); assertTrue(link.motionPending)
        fake.robot("""{"type":"status","busy":false,"estop":false}""")
        assertFalse(link.motionPending)

        link.move(50f, 10f); assertTrue(link.motionPending)
        fake.robot("""{"type":"status","busy":true,"estop":false}""")
        link.disconnect()
        assertFalse(link.connected); assertTrue(fake.closed)
        assertEquals("a stop is sent to a connected robot on disconnect", "stop", fake.types().last())
        assertEquals(0, link.pendingSeq); assertFalse(link.busy); assertFalse(link.motionPending)
        assertEquals(RobotLink.Kind.NONE, link.kind)
        assertFalse("sends are refused after disconnect", link.move(10f, 5f))
    }

    @Test fun lostTransportStateDropsPendingMotion() {
        // A custom transport (USB) reports its death by having NavActivity call disconnect(); the pending
        // seq must not survive a reconnect either: the new robot never saw that command.
        val (link, fake) = connected()
        link.move(100f, 20f); assertTrue(link.motionPending)
        link.disconnect()
        val fake2 = Fake()
        link.connectCustom("fake2") { incoming -> fake2.robot = incoming; fake2 }
        assertFalse(link.motionPending)
        fake2.robot("""{"type":"pong","seq":1,"t":0.0}""")       // pong also counts as "the robot answered"
        assertTrue(link.connected)
        assertEquals(1, fake.sent.count { it.getString("type") == "move" })
        link.disconnect()
    }
}
