/*
 * Neato and OpenBot robots over USB-OTG.
 *
 * Command formats, scaling (deadband, PWM cap) and the USB device list are ported from OSSDC VisionAI
 * Mobile (https://github.com/OSSDC/OSSDC-VisionAI-Mobile: RaceOSSDCActivity.java, UsbService.java,
 * usb_device_filter.xml), Copyright (C) 2021 Marius Slavescu - OSSDC.org, licensed under the Apache
 * License, Version 2.0 (robot/THIRD_PARTY.md). Changes: Kotlin rewrite; driven by robot/PROTOCOL.md body
 * velocities instead of a joystick; Neato 1 s distance horizon (as in the neato_robot ROS driver); OpenBot
 * heartbeat; battery / bumper parsing; serial access through usb-serial-for-android (MIT) instead of
 * felhr UsbSerial. The same logic for a Linux computer is robot/usb_robots.py.
 */
package com.real2sim.capture.robot

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver
import com.hoho.android.usbserial.driver.ProbeTable
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.driver.UsbSerialProber
import com.hoho.android.usbserial.util.SerialInputOutputManager
import com.real2sim.capture.RobotTransport
import org.json.JSONObject
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.round
import kotlin.math.sign

/** The serial dialects (OSSDC robotMode "OpenBot" / "Neato"). */
enum class UsbRobotKind(val title: String, val wheelBase: Double, val maxWheel: Double) {
    NEATO("Neato", 0.248, 0.30),
    OPENBOT("OpenBot", 0.15, 0.40),
}

/** Pure command encoders and parsers (unit-tested; mirror robot/usb_robots.py). */
object RobotDrivers {
    /** Wheel speeds (m/s) -> "setmotor L R S\n" (mm, mm, mm/s); distance = 1 s of travel. */
    fun neatoSetMotor(left: Double, right: Double, maxMmS: Double = 300.0): String {
        var l = left * 1000; var r = right * 1000
        var peak = max(abs(l), abs(r))
        if (peak > maxMmS) { val k = maxMmS / peak; l *= k; r *= k; peak = maxMmS }
        val li = round(l).toInt(); val ri = round(r).toInt(); val si = round(peak).toInt()
        return if (si < 1 || (li == 0 && ri == 0)) "setmotor 0 0 0\n" else "setmotor $li $ri $si\n"
    }

    /** OSSDC applyDeadband: 0..1 -> deadband..1 so small commands still turn the motors. */
    fun applyDeadband(frac: Double, deadband: Double): Double {
        val f = frac.coerceIn(-1.0, 1.0)
        if (abs(f) < 0.005) return 0.0
        return sign(f) * (deadband + (1 - deadband) * abs(f))
    }

    /** Wheel speeds (m/s) -> "c<l>,<r>\n", PWM up to maxPwm (OSSDC default cap 150). */
    fun openbotCtrl(left: Double, right: Double, maxWheel: Double = 0.4, maxPwm: Int = 150, deadband: Double = 0.3,
                    invertLeft: Boolean = false, invertRight: Boolean = false): String {
        fun pwm(v: Double, inv: Boolean): Int {
            var p = floor(abs(applyDeadband(v / maxWheel, deadband)) * maxPwm).toInt().coerceAtMost(min(maxPwm, 255))
            if (v < 0) p = -p
            return if (inv) -p else p
        }
        return "c${pwm(left, invertLeft)},${pwm(right, invertRight)}\n"
    }

    /** One line of a Neato getcharger / getdigitalsensors answer. Returns true on a new bumper press. */
    fun parseNeato(line: String, out: MutableMap<String, Any>): Boolean {
        val p = line.trim().split(",")
        if (p.size < 2) return false
        val k = p[0].trim(); val v = p[1].trim()
        when (k) {
            "VBattV" -> v.toDoubleOrNull()?.let { out["battery_v"] = it }
            "FuelPercent" -> v.toDoubleOrNull()?.let { out["battery_pct"] = it.toInt() }
            "LSIDEBIT", "LFRONTBIT", "RSIDEBIT", "RFRONTBIT", "LLDSBIT", "RLDSBIT" -> {
                val pressed = (v.toDoubleOrNull() ?: 0.0) != 0.0
                val was = out["bump_$k"] == true
                out["bump_$k"] = pressed
                return pressed && !was
            }
        }
        return false
    }

    /** One line from the OpenBot firmware: v<volts>, s<cm>, w<l>,<r>, b<bumper>. */
    fun parseOpenBot(line: String, out: MutableMap<String, Any>) {
        val t = line.trim()
        if (t.isEmpty() || !t[0].isLetter()) return
        val rest = t.substring(1)
        when (t[0]) {
            'v' -> rest.split(",")[0].toDoubleOrNull()?.let { out["battery_v"] = it }
            's' -> rest.split(",")[0].toDoubleOrNull()?.let { out["sonar_cm"] = it }
            'w' -> rest.split(",").let { if (it.size >= 2) { val l = it[0].toDoubleOrNull(); val r = it[1].toDoubleOrNull(); if (l != null && r != null) out["wheel_rps"] = listOf(l, r) } }
            'b' -> out["bumper"] = rest.trim()
        }
    }

    /** USB IDs the robot boards use (from OSSDC's usb_device_filter.xml). */
    val NEATO_VID = 0x2108; val NEATO_PID = 0x780B
}

/**
 * The robot side of robot/PROTOCOL.md, running on the phone: takes the JSON the nav screen sends (hello,
 * ping, vel, stop, move, turn, estop), drives the motors through [write], and answers through [emit]
 * (hello, pong, status, done, estop). Same rules as robot/receiver.py: vel expires after 0.5 s, stop wins,
 * move / turn run open-loop from timing. Hardware-independent so it can be unit-tested.
 */
class RobotCore(
    val kind: UsbRobotKind,
    private val write: (String) -> Unit,
    private val emit: (JSONObject) -> Unit,
    var maxPwm: Int = 150,
    var invertLeft: Boolean = false,
    var invertRight: Boolean = false,
) {
    val wheelBase = kind.wheelBase
    val maxWheel = kind.maxWheel
    val watchdogS = 0.5
    private val ex = Executors.newSingleThreadScheduledExecutor { r -> Thread(r, "usb-robot").apply { isDaemon = true } }
    private var velExpires = 0L
    private var job: ScheduledFuture<*>? = null
    private var v = 0.0; private var w = 0.0; private var left = 0.0; private var right = 0.0
    @Volatile var estop = false; private set
    val info = HashMap<String, Any>()
    private var lastHeartbeat = 0L
    private val tickers = ArrayList<ScheduledFuture<*>>()

    fun start(statusHz: Double = 2.0) {
        if (kind == UsbRobotKind.NEATO) ex.execute { write("testmode on\n") }
        tickers += ex.scheduleWithFixedDelay({ watchdog() }, 50, 50, TimeUnit.MILLISECONDS)
        tickers += ex.scheduleWithFixedDelay({ emit(status()) }, 500, (1000 / statusHz).toLong(), TimeUnit.MILLISECONDS)
        if (kind == UsbRobotKind.NEATO) tickers += ex.scheduleWithFixedDelay({ write("getcharger\ngetdigitalsensors\n") }, 300, 1000, TimeUnit.MILLISECONDS)
    }

    fun close() {
        ex.execute {
            drive(0.0, 0.0)
            if (kind == UsbRobotKind.NEATO) write("testmode off\n")
        }
        ex.shutdown()
        try { ex.awaitTermination(300, TimeUnit.MILLISECONDS) } catch (_: InterruptedException) {}
    }

    /** One message from the phone (any thread). */
    fun handle(json: String) = ex.execute { handleNow(json) }

    /** A line from the robot's serial port (any thread). */
    fun serialLine(line: String) = ex.execute {
        when (kind) {
            UsbRobotKind.NEATO -> if (RobotDrivers.parseNeato(line, info)) bump()
            UsbRobotKind.OPENBOT -> {
                val had = info["bumper"]
                RobotDrivers.parseOpenBot(line, info)
                val b = info["bumper"] as? String
                if (b != null && b != "" && b != "0" && b != had) info["bumper_hits"] = ((info["bumper_hits"] as? Int) ?: 0) + 1
            }
        }
    }

    private fun bump() {
        if (estop) return
        estop = true; cancelJob(); drive(0.0, 0.0)
        emit(JSONObject().put("type", "estop").put("reason", "bumper"))
    }

    internal fun handleNow(json: String) {
        val m = try { JSONObject(json) } catch (_: Exception) { emit(JSONObject().put("type", "error").put("error", "bad json")); return }
        val seq = m.opt("seq")
        when (m.optString("type")) {
            "hello" -> emit(JSONObject().put("type", "hello").put("proto", 1).put("name", "${kind.title} (USB)")
                .put("caps", org.json.JSONArray(listOf("vel", "move", "turn", "stop", "status")))
                .put("wheel_base", wheelBase).put("max_wheel", maxWheel))
            "ping" -> emit(JSONObject().put("type", "pong").put("seq", seq).put("t", m.opt("t")))
            "vel" -> {
                cancelJob()
                drive(m.optDouble("v", 0.0).coerceIn(-maxWheel, maxWheel), m.optDouble("w", 0.0))
                velExpires = System.nanoTime() + (watchdogS * 1e9).toLong()
            }
            "stop" -> { cancelJob(); velExpires = 0; drive(0.0, 0.0) }
            "move" -> {
                cancelJob()
                val dist = m.optDouble("dist_cm", 0.0) / 100
                val speed = min(abs(m.optDouble("speed_cms", 20.0)) / 100, maxWheel).takeIf { it > 0 } ?: 0.2
                timed(sign(dist) * speed, 0.0, abs(dist) / speed, seq)
            }
            "turn" -> {
                cancelJob()
                val ang = Math.toRadians(m.optDouble("deg", 0.0))
                val rate = Math.toRadians(abs(m.optDouble("speed_dps", 45.0))).takeIf { it > 0 } ?: Math.toRadians(45.0)
                timed(0.0, sign(ang) * rate, abs(ang) / rate, seq)
            }
            "estop" -> { estop = m.optBoolean("on", true); cancelJob(); drive(0.0, 0.0) }
            else -> emit(JSONObject().put("type", "error").put("seq", seq).put("error", "unknown type ${m.optString("type")}"))
        }
    }

    private fun timed(v: Double, w: Double, seconds: Double, seq: Any?) {
        drive(v, w)
        job = ex.schedule({ drive(0.0, 0.0); job = null; emit(JSONObject().put("type", "done").put("seq", seq)) },
            (seconds * 1000).toLong().coerceAtLeast(0), TimeUnit.MILLISECONDS)
    }

    private fun cancelJob() { job?.cancel(false); job = null }

    private fun watchdog() {
        if (job == null && velExpires != 0L && System.nanoTime() > velExpires) { velExpires = 0; drive(0.0, 0.0) }
        // OpenBot firmware heartbeat: it stops the motors itself if the phone (or the cable) goes away.
        if (kind == UsbRobotKind.OPENBOT && System.nanoTime() - lastHeartbeat > 250_000_000L) {
            lastHeartbeat = System.nanoTime(); write("h750\n")
        }
    }

    /** Body velocity -> wheel speeds, scaled down together if one saturates, -> serial command. */
    internal fun drive(vIn: Double, wIn: Double) {
        var vv = vIn; var ww = wIn
        if (estop) { vv = 0.0; ww = 0.0 }
        v = vv; w = ww
        var l = vv - ww * wheelBase / 2; var r = vv + ww * wheelBase / 2
        val peak = max(abs(l), abs(r))
        if (peak > maxWheel) { val k = maxWheel / peak; l *= k; r *= k }
        left = l; right = r
        write(when (kind) {
            UsbRobotKind.NEATO -> RobotDrivers.neatoSetMotor(l, r, maxWheel * 1000)
            UsbRobotKind.OPENBOT -> RobotDrivers.openbotCtrl(l, r, maxWheel, maxPwm, 0.3, invertLeft, invertRight)
        })
    }

    internal fun status(): JSONObject {
        val s = JSONObject().put("type", "status").put("v", r3(v)).put("w", r3(w)).put("left", r3(left)).put("right", r3(right))
            .put("busy", job != null).put("estop", estop)
        (info["battery_v"] as? Double)?.let { s.put("battery_v", it) }
        (info["sonar_cm"] as? Double)?.let { s.put("sonar_cm", it) }
        return s
    }

    private fun r3(x: Double) = round(x * 1000) / 1000
}

/**
 * USB-OTG transport for RobotLink.connectCustom: finds the robot's USB serial adapter, asks for permission,
 * opens it (115200 8N1) and runs a [RobotCore] on it.
 */
class UsbRobot(private val ctx: Context, private val kind: UsbRobotKind, private val incoming: (String) -> Unit,
               private val onState: (String) -> Unit) : RobotTransport {
    private val usb = ctx.getSystemService(Context.USB_SERVICE) as UsbManager
    private var port: UsbSerialPort? = null
    private var io: SerialInputOutputManager? = null
    private var core: RobotCore? = null
    private val pending = ArrayList<String>()
    private val lineBuf = StringBuilder()
    private var receiver: BroadcastReceiver? = null
    @Volatile private var closed = false

    companion object {
        const val ACTION_PERMISSION = "com.real2sim.capture.USB_ROBOT_PERMISSION"

        private val prober: UsbSerialProber by lazy {
            val t = UsbSerialProber.getDefaultProbeTable()
            t.addProduct(RobotDrivers.NEATO_VID, RobotDrivers.NEATO_PID, CdcAcmSerialDriver::class.java)
            UsbSerialProber(t)
        }

        /** Serial adapters plugged in now (Neato, Arduino / CH340 / CP210x / FTDI / PL2303 / ESP32 boards). */
        fun find(usb: UsbManager): List<UsbSerialDriver> {
            val found = prober.findAllDrivers(usb).toMutableList()
            // Boards that only announce a CDC ACM interface (Teensy, STM32, ESP32-S3 native USB …).
            for (d in usb.deviceList.values) if (found.none { it.device == d } && (0 until d.interfaceCount).any { d.getInterface(it).interfaceClass == 2 }) {
                found.add(CdcAcmSerialDriver(d))
            }
            return found
        }

        fun describe(d: UsbDevice) = "%04X:%04X %s".format(d.vendorId, d.productId, d.productName ?: d.deviceName)
    }

    /** Finds the device, requests permission if needed, then opens it. */
    fun start() {
        val drivers = find(usb)
        val d = drivers.firstOrNull { it.device.vendorId == RobotDrivers.NEATO_VID } .takeIf { kind == UsbRobotKind.NEATO }
            ?: drivers.firstOrNull()
        if (d == null) { onState("USB: no serial device found (plug the robot in with an OTG adapter)"); return }
        if (usb.hasPermission(d.device)) { open(d); return }
        onState("USB: allow access to ${describe(d.device)}")
        val r = object : BroadcastReceiver() {
            override fun onReceive(c: Context, i: Intent) {
                if (i.action != ACTION_PERMISSION) return
                try { ctx.unregisterReceiver(this) } catch (_: Exception) {}
                receiver = null
                if (closed) return
                if (usb.hasPermission(d.device)) open(d) else onState("USB: permission denied")
            }
        }
        receiver = r
        ContextCompat.registerReceiver(ctx, r, IntentFilter(ACTION_PERMISSION), ContextCompat.RECEIVER_NOT_EXPORTED)
        val flags = if (Build.VERSION.SDK_INT >= 31) PendingIntent.FLAG_MUTABLE else 0
        val pi = PendingIntent.getBroadcast(ctx, 0, Intent(ACTION_PERMISSION).setPackage(ctx.packageName), flags)
        usb.requestPermission(d.device, pi)
    }

    private fun open(d: UsbSerialDriver) {
        try {
            val conn = usb.openDevice(d.device) ?: run { onState("USB: could not open ${describe(d.device)}"); return }
            val p = d.ports[0]
            p.open(conn)
            p.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
            try { p.dtr = true; p.rts = true } catch (_: Exception) {}
            port = p
            val c = RobotCore(kind, ::writeSerial, { incoming(it.toString()) })
            core = c
            val m = SerialInputOutputManager(p, object : SerialInputOutputManager.Listener {
                override fun onNewData(data: ByteArray) = onSerial(data)
                override fun onRunError(e: Exception) { if (!closed) onState("USB: ${e.message ?: "disconnected"}") }
            })
            io = m
            m.start()
            onState("USB: ${kind.title} on ${describe(d.device)}")
            // OpenBot boards reset when the port opens; give the bootloader a moment.
            val delay = if (kind == UsbRobotKind.OPENBOT) 2000L else 100L
            Thread {
                try { Thread.sleep(delay) } catch (_: InterruptedException) {}
                if (closed) return@Thread
                c.start()
                synchronized(pending) { pending.forEach(c::handle); pending.clear() }
            }.apply { isDaemon = true }.start()
        } catch (e: Exception) {
            onState("USB: ${e.message}")
            close()
        }
    }

    private fun onSerial(data: ByteArray) {
        val c = core ?: return
        for (b in data) {
            val ch = b.toInt().toChar()
            if (ch == '\n' || ch == '\u001a' || ch == '\r') {
                if (lineBuf.isNotEmpty()) { c.serialLine(lineBuf.toString()); lineBuf.setLength(0) }
            } else lineBuf.append(ch)
        }
        if (lineBuf.length > 4096) lineBuf.setLength(0)
    }

    private fun writeSerial(s: String) {
        try { port?.write(s.toByteArray(), 200) } catch (_: Exception) {}
    }

    override fun send(json: String): Boolean {
        val c = core
        if (c == null) { synchronized(pending) { if (pending.size < 20) pending.add(json) }; return true }
        c.handle(json)
        return true
    }

    override fun close() {
        closed = true
        receiver?.let { try { ctx.unregisterReceiver(it) } catch (_: Exception) {} }
        receiver = null
        core?.close(); core = null
        io?.stop(); io = null
        try { port?.close() } catch (_: Exception) {}
        port = null
    }
}
