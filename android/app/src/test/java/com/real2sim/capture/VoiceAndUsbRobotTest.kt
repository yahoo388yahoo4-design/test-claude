package com.real2sim.capture

import com.real2sim.capture.robot.RobotCore
import com.real2sim.capture.robot.RobotDrivers
import com.real2sim.capture.robot.UsbRobotKind
import com.real2sim.capture.voice.EnergyVAD
import com.real2sim.capture.voice.IntentParser
import com.real2sim.capture.voice.VoiceAction
import com.real2sim.capture.voice.VoiceTools
import com.real2sim.capture.voice.VoiceURL
import com.real2sim.capture.voice.WAV
import com.real2sim.capture.voice.WyomingDecoder
import com.real2sim.capture.voice.WyomingEvent
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.Collections
import kotlin.math.sin

/** Same cases as ios/NavTests/voice/main.swift and robot/test_usb_robots.py. */
class VoiceAndUsbRobotTest {
    private val labels = listOf("table", "chair 2", "sofa", "kitchen counter", "door")
    private fun p(s: String) = IntentParser.parse(s, labels)

    @Test fun localIntents() {
        assertEquals(listOf(VoiceAction.Stop), p("Stop!"))
        assertEquals(listOf(VoiceAction.Stop), p("hey robot, freeze"))
        assertEquals(listOf(VoiceAction.Stop), p("go to the table and stop"))
        assertFalse(IntentParser.containsStop("please go to the st"))
        assertEquals(listOf(VoiceAction.Move(1.5)), p("move forward 1.5 meters"))
        assertEquals(listOf(VoiceAction.Move(0.5)), p("go forward fifty centimeters"))
        assertEquals(listOf(VoiceAction.Move(-0.2)), p("go back a little"))
        assertEquals(listOf(VoiceAction.Move(0.5)), p("move forward 50"))
        assertEquals(listOf(VoiceAction.Move(1.5)), p("move forward one and a half meters"))
        assertEquals(listOf(VoiceAction.Turn(90.0)), p("turn left"))
        assertEquals(listOf(VoiceAction.Turn(-45.0)), p("turn right forty five degrees"))
        assertEquals(listOf(VoiceAction.Turn(30.0)), p("rotate left 30°"))
        assertEquals(listOf(VoiceAction.Turn(180.0)), p("turn around"))
        assertEquals(listOf(VoiceAction.GoTo("kitchen counter")), p("go to the kitchen counter"))
        assertEquals(listOf(VoiceAction.GoTo("chair 2")), p("take me to the chair"))
        assertNull(p("go to the garage"))
        assertEquals(listOf(VoiceAction.Move(1.0), VoiceAction.Turn(90.0), VoiceAction.GoTo("door")), p("move forward 1 meter then turn left and go to the door"))
        assertEquals(listOf(VoiceAction.Status), p("where are you"))
        assertEquals(listOf(VoiceAction.SetMode("auto")), p("switch to auto mode"))
        assertEquals(listOf(VoiceAction.ClearGoal), p("cancel"))
        assertEquals(listOf(VoiceAction.SavePlace("kitchen")), p("remember this place as the kitchen"))
        assertNull(p("what's the meaning of life"))
    }

    @Test fun toolCalls() {
        val resp = """{"choices":[{"message":{"role":"assistant","content":"On my way.","tool_calls":[
            {"id":"c1","type":"function","function":{"name":"move","arguments":"{\"distance_m\": 12}"}},
            {"id":"c2","type":"function","function":{"name":"turn","arguments":{"degrees":-90}}}]}}]}"""
        val (text, calls) = VoiceTools.parseChatResponse(resp)!!
        assertEquals("On my way.", text)
        assertEquals(listOf(VoiceAction.Move(5.0), VoiceAction.Turn(-90.0)), calls.mapNotNull { VoiceTools.action(it.name, it.arguments) })
        assertNull(VoiceTools.action("set_mode", """{"mode":"warp"}"""))
        assertNull(VoiceTools.action("rm_rf", null))
        val j = VoiceTools.parseJSONReply("Sure! {\"actions\":[{\"tool\":\"turn\",\"degrees\":45}],\"say\":\"Turning {left}.\"} done")!!
        assertEquals(listOf(VoiceAction.Turn(45.0)), j.first); assertEquals("Turning {left}.", j.second)
        assertEquals(9, VoiceTools.openAITools().length())
    }

    @Test fun wyomingRoundTrip() {
        val bytes = WyomingEvent("audio-chunk", WyomingEvent.audioFormat(16000), byteArrayOf(1, 2, 3, 4)).encode()
        val dec = WyomingDecoder()
        val got = bytes.flatMap { dec.feed(byteArrayOf(it)) }
        assertEquals(1, got.size); assertEquals("audio-chunk", got[0].type)
        assertEquals(16000, got[0].data.getInt("rate")); assertTrue(got[0].payload.contentEquals(byteArrayOf(1, 2, 3, 4)))
        val inline = "{\"type\":\"transcript\",\"data\":{\"text\":\"hi\"}}\n".toByteArray()
        assertEquals("hi", WyomingDecoder().feed(inline)[0].data.getString("text"))
    }

    @Test fun vadAndWav() {
        val vad = EnergyVAD(16000)
        val quiet = ShortArray(480) { 50 }
        val loud = ShortArray(480) { (4000 * sin(it * 0.3)).toInt().toShort() }
        repeat(20) { vad.feed(quiet) }
        var r = EnergyVAD.Result.WAITING
        repeat(20) { r = vad.feed(loud) }
        assertEquals(EnergyVAD.Result.SPEAKING, r)
        var n = 0
        while (r == EnergyVAD.Result.SPEAKING && n < 100) { r = vad.feed(quiet); n++ }
        assertEquals(EnergyVAD.Result.DONE, r); assertTrue(n in 25..28)
        val pcm = byteArrayOf(1, 0, 2, 0)
        val (rate, ch, back) = WAV.decode(WAV.encode(pcm, 22050))!!
        assertEquals(22050, rate); assertEquals(1, ch); assertTrue(back.contentEquals(pcm))
        assertEquals("http://192.168.1.5:11434/v1", VoiceURL.openAIBase("192.168.1.5:11434"))
        assertTrue(VoiceURL.isLocal("http://192.168.1.5:11434")); assertFalse(VoiceURL.isLocal("https://api.openai.com/v1"))
    }

    @Test fun robotEncoders() {
        assertEquals("setmotor 200 200 200\n", RobotDrivers.neatoSetMotor(0.2, 0.2))
        assertEquals("setmotor 300 150 300\n", RobotDrivers.neatoSetMotor(0.6, 0.3))
        assertEquals("setmotor 0 0 0\n", RobotDrivers.neatoSetMotor(0.0, 0.0))
        assertEquals("c0,0\n", RobotDrivers.openbotCtrl(0.0, 0.0))
        assertEquals("c150,150\n", RobotDrivers.openbotCtrl(0.4, 0.4))
        assertEquals("c55,-55\n", RobotDrivers.openbotCtrl(0.04, -0.04))
        val info = HashMap<String, Any>()
        assertFalse(RobotDrivers.parseNeato("VBattV,14.32", info)); assertEquals(14.32, info["battery_v"])
        assertTrue(RobotDrivers.parseNeato("LFRONTBIT,1", info)); assertFalse(RobotDrivers.parseNeato("LFRONTBIT,1", info))
        RobotDrivers.parseOpenBot("v7.84", info); assertEquals(7.84, info["battery_v"])
    }

    @Test fun robotCoreSpeaksProtocol() {
        val wrote = Collections.synchronizedList(ArrayList<String>())
        val out = Collections.synchronizedList(ArrayList<JSONObject>())
        val core = RobotCore(UsbRobotKind.OPENBOT, { wrote.add(it) }, { out.add(it) })
        core.handleNow("""{"type":"hello","proto":1}""")
        assertEquals("OpenBot (USB)", out.last().getString("name"))
        core.handleNow("""{"type":"ping","seq":3,"t":1.5}""")
        assertEquals("pong", out.last().getString("type")); assertEquals(1.5, out.last().getDouble("t"), 0.0)
        core.handleNow("""{"type":"vel","v":0.2,"w":0.0,"seq":4}""")
        assertEquals("c97,97\n", wrote.last())
        core.handleNow("""{"type":"vel","v":0.0,"w":1.0,"seq":5}""")   // spin left: right wheel forward
        assertEquals("c-64,64\n", wrote.last())
        core.handleNow("""{"type":"estop","on":true}""")
        core.handleNow("""{"type":"vel","v":0.2,"w":0.0,"seq":6}""")
        assertEquals("c0,0\n", wrote.last())
        assertTrue(core.status().getBoolean("estop"))
        core.handleNow("""{"type":"bogus"}""")
        assertEquals("error", out.last().getString("type"))
        val neato = RobotCore(UsbRobotKind.NEATO, { wrote.add(it) }, { out.add(it) })
        neato.handleNow("""{"type":"vel","v":0.1,"w":0.0,"seq":1}""")
        assertEquals("setmotor 100 100 100\n", wrote.last())
    }
}
