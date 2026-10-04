package com.real2sim.capture

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.ParcelUuid
import android.os.SystemClock
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.util.UUID
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger
import kotlin.math.round

/**
 * Messages of the robot link protocol shared with the iPhone app (robot/PROTOCOL.md, v1).
 * v in m/s, w in rad/s (+ = left), move in cm and cm/s, turn in degrees (+ = left) and deg/s.
 */
object RobotProtocol {
    const val VERSION = 1
    private fun r3(x: Double) = if (x.isFinite()) round(x * 1000) / 1000 else 0.0
    fun hello(client: String) = JSONObject().put("type", "hello").put("proto", VERSION).put("client", client)
    fun ping(seq: Int, t: Double) = JSONObject().put("type", "ping").put("seq", seq).put("t", t)
    fun vel(v: Double, w: Double, seq: Int) = JSONObject().put("type", "vel").put("v", r3(v)).put("w", r3(w)).put("seq", seq)
    fun stop(seq: Int) = JSONObject().put("type", "stop").put("seq", seq)
    fun move(cm: Double, speedCms: Double, seq: Int) = JSONObject().put("type", "move").put("dist_cm", r3(cm)).put("speed_cms", r3(speedCms)).put("seq", seq)
    fun turn(deg: Double, speedDps: Double, seq: Int) = JSONObject().put("type", "turn").put("deg", r3(deg)).put("speed_dps", r3(speedDps)).put("seq", seq)
    fun estop(on: Boolean) = JSONObject().put("type", "estop").put("on", on)

    /** Splits a BLE byte stream into newline-terminated lines (packets may cut a line anywhere). */
    class LineSplitter {
        private val buf = ByteArrayOutputStream()
        fun feed(bytes: ByteArray): List<String> {
            val out = ArrayList<String>()
            for (b in bytes) {
                if (b == '\n'.code.toByte()) {
                    val s = buf.toString(Charsets.UTF_8.name()).trim(); buf.reset()
                    if (s.isNotEmpty()) out.add(s)
                } else buf.write(b.toInt())
            }
            if (buf.size() > 8192) buf.reset()
            return out
        }
    }
}

/**
 * Connection to the robot's motor controller, same JSON messages over two transports (robot/PROTOCOL.md):
 *  - Wi-Fi: one JSON object per WebSocket text message (default ws://192.168.4.1:8777/robot, the ESP32
 *    access point; robot/receiver.py prints its own URL).
 *  - Bluetooth LE: newline-terminated JSON over the Nordic UART Service (ESP32, nRF52, HM-10 style modules).
 * The link counts as connected once the robot answers (hello or pong) on Wi-Fi, or once the UART
 * characteristics are found on BLE. send() may be called from any thread.
 */
class RobotLink(private val ctx: Context, private val onEvent: (String) -> Unit) {
    enum class Kind { NONE, WIFI, BLE }

    @Volatile var kind = Kind.NONE; private set
    @Volatile var stateText = "robot: off"; private set
    @Volatile var connected = false; private set
    @Volatile var robotName = ""; private set
    @Volatile var caps = ""; private set
    @Volatile var rttMs = Float.NaN; private set
    @Volatile var robotEstop = false; private set
    @Volatile var busy = false; private set
    @Volatile var batteryV = Float.NaN; private set
    /** Robot-reported body velocity (m/s, rad/s) and wheel speeds (m/s); NaN when not reported. */
    @Volatile var robotV = Float.NaN; private set
    @Volatile var robotW = Float.NaN; private set
    @Volatile var wheelL = Float.NaN; private set
    @Volatile var wheelR = Float.NaN; private set
    @Volatile var statusRxMs = 0L; private set
    @Volatile var sent = 0; private set
    @Volatile var received = 0; private set

    /** Called for every message from the robot (any thread). */
    var onMessage: ((JSONObject) -> Unit)? = null

    private val seq = AtomicInteger(0)
    fun nextSeq() = seq.incrementAndGet()

    private var ws: WebSocket? = null
    private val client = OkHttpClient.Builder().connectTimeout(4, TimeUnit.SECONDS).build()
    private var pingTicker: Thread? = null
    private var ble: BleUart? = null

    fun connect(kind: Kind, wsUrl: String, bleName: String) {
        disconnect()
        this.kind = kind
        when (kind) {
            Kind.NONE -> setState("robot: off (guide only)", false)
            Kind.WIFI -> {
                var url = wsUrl.trim()
                if (!url.startsWith("ws://") && !url.startsWith("wss://")) url = "ws://$url"
                if (url.count { it == ':' } < 2) url += ":8777"
                if (!url.substringAfter("://").contains('/')) url += "/robot"
                setState("connecting $url", false)
                ws = try {
                    client.newWebSocket(Request.Builder().url(url).build(), object : WebSocketListener() {
                        override fun onOpen(webSocket: WebSocket, response: Response) { sendRaw(RobotProtocol.hello("R2S Capture Android")) }
                        override fun onMessage(webSocket: WebSocket, text: String) = handle(text)
                        override fun onClosed(webSocket: WebSocket, code: Int, reason: String) { if (ws === webSocket) setState("robot: closed $reason", false) }
                        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) { if (ws === webSocket) setState("Wi-Fi: ${t.message}", false) }
                    })
                } catch (e: IllegalArgumentException) { setState("bad URL $url", false); null }
            }
            Kind.BLE -> {
                setState("BLE $bleName", false)
                ble = BleUart(ctx, bleName, { s, ok -> setState(s, ok); if (ok) sendRaw(RobotProtocol.hello("R2S Capture Android")) }, ::handle).also { it.start() }
            }
        }
        if (kind != Kind.NONE) pingTicker = Thread {
            try {
                while (true) {
                    Thread.sleep(1000)
                    sendRaw(RobotProtocol.ping(nextSeq(), SystemClock.elapsedRealtime() / 1000.0))
                }
            } catch (_: InterruptedException) {}
        }.apply { isDaemon = true; start() }
    }

    fun disconnect() {
        pingTicker?.interrupt(); pingTicker = null
        if (connected) sendRaw(RobotProtocol.stop(nextSeq()))
        ws?.close(1000, "bye"); ws = null
        ble?.stop(); ble = null
        kind = Kind.NONE
        setState("robot: off", false)
    }

    private fun setState(s: String, ok: Boolean) { stateText = s; connected = ok; onEvent(s) }

    /** Sends only while the robot is connected. */
    fun send(o: JSONObject): Boolean = connected && sendRaw(o)

    private fun sendRaw(o: JSONObject): Boolean {
        val s = o.toString()
        val ok = when (kind) {
            Kind.WIFI -> ws?.send(s) ?: false
            Kind.BLE -> ble?.sendLine(s) ?: false
            Kind.NONE -> false
        }
        if (ok) sent++
        return ok
    }

    fun vel(v: Float, w: Float) = send(RobotProtocol.vel(v.toDouble(), w.toDouble(), nextSeq()))
    fun stop() = send(RobotProtocol.stop(nextSeq()))
    fun move(cm: Float, speedCms: Float) = send(RobotProtocol.move(cm.toDouble(), speedCms.toDouble(), nextSeq()))
    fun turn(deg: Float, speedDps: Float) = send(RobotProtocol.turn(deg.toDouble(), speedDps.toDouble(), nextSeq()))
    fun estop(on: Boolean) { if (on) stop(); send(RobotProtocol.estop(on)) }

    private fun handle(text: String) {
        val m = try { JSONObject(text) } catch (_: Exception) { return }
        received++
        when (m.optString("type")) {
            "hello" -> {
                robotName = m.optString("name", "robot")
                caps = m.optJSONArray("caps")?.let { a -> (0 until a.length()).joinToString(",") { a.optString(it) } } ?: ""
                setState(if (kind == Kind.BLE) "BLE $robotName" else robotName, true)
            }
            "pong" -> {
                if (m.has("t")) rttMs = ((SystemClock.elapsedRealtime() / 1000.0 - m.optDouble("t")) * 1000).toFloat()
                if (!connected) setState(robotName.ifEmpty { "robot" }, true)
            }
            "status" -> {
                fun f(k: String) = if (m.has(k) && !m.isNull(k)) m.optDouble(k).toFloat() else Float.NaN
                robotV = f("v"); robotW = f("w"); wheelL = f("left"); wheelR = f("right")
                if (m.has("battery_v")) batteryV = f("battery_v")
                busy = m.optBoolean("busy"); robotEstop = m.optBoolean("estop")
                statusRxMs = SystemClock.elapsedRealtime()
            }
            "estop" -> { robotEstop = true; onEvent("robot: emergency stop") }
            "done" -> onEvent("robot: done #${m.optInt("seq")}")
            "error" -> onEvent("robot error: ${m.optString("error")}")
        }
        onMessage?.invoke(m)
    }

    /** Condensed robot status for the HUD. */
    fun statusLine(): String {
        val parts = ArrayList<String>()
        parts.add(stateText)
        if (connected && rttMs.isFinite()) parts.add("rtt %.0f ms".format(rttMs))
        if (batteryV.isFinite()) parts.add("%.1f V".format(batteryV))
        if (wheelL.isFinite() && wheelR.isFinite()) parts.add("L %.2f R %.2f".format(wheelL, wheelR))
        if (busy) parts.add("busy")
        if (robotEstop) parts.add("E-STOP")
        return parts.joinToString("  ")
    }
}

/**
 * Minimal Nordic UART Service central: scans for the service (optionally filtered by name prefix),
 * connects to the first match, subscribes to TX notifications and writes RX in MTU-sized chunks.
 */
@SuppressLint("MissingPermission")   // NavActivity requests BLUETOOTH_SCAN / CONNECT (or location below API 31) first
class BleUart(
    private val ctx: Context,
    private val namePrefix: String,
    private val onState: (String, Boolean) -> Unit,
    private val onLine: (String) -> Unit,
) {
    companion object {
        val SERVICE: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        val RX: UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")   // phone -> robot (write)
        val TX: UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")   // robot -> phone (notify)
        val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
    }

    private val adapter = (ctx.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager).adapter
    private var gatt: BluetoothGatt? = null
    private var rx: BluetoothGattCharacteristic? = null
    private var mtu = 23
    private val splitter = RobotProtocol.LineSplitter()
    private val writeQueue = ArrayDeque<ByteArray>()
    private var writing = false
    @Volatile private var stopped = false

    private val scanCb = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val name = result.device.name ?: result.scanRecord?.deviceName ?: ""
            if (namePrefix.isNotEmpty() && !name.startsWith(namePrefix)) return
            adapter?.bluetoothLeScanner?.stopScan(this)
            onState("BLE connecting $name", false)
            connectTo(result.device)
        }
        override fun onScanFailed(errorCode: Int) = onState("BLE scan failed ($errorCode)", false)
    }

    fun start() {
        if (adapter == null) { onState("no Bluetooth on this phone", false); return }
        if (!adapter.isEnabled) { onState("Bluetooth is off", false); return }
        val scanner = adapter.bluetoothLeScanner ?: run { onState("BLE unavailable", false); return }
        onState("BLE scanning", false)
        try {
            scanner.startScan(listOf(ScanFilter.Builder().setServiceUuid(ParcelUuid(SERVICE)).build()),
                ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(), scanCb)
        } catch (e: SecurityException) { onState("Bluetooth permission denied", false) }
    }

    fun stop() {
        stopped = true
        try { adapter?.bluetoothLeScanner?.stopScan(scanCb) } catch (_: Exception) {}
        try { gatt?.disconnect(); gatt?.close() } catch (_: Exception) {}
        gatt = null; rx = null
    }

    private fun connectTo(d: BluetoothDevice) {
        gatt = d.connectGatt(ctx, false, object : BluetoothGattCallback() {
            override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
                if (newState == BluetoothProfile.STATE_CONNECTED) { g.requestMtu(247) }
                else if (newState == BluetoothProfile.STATE_DISCONNECTED && !stopped) {
                    rx = null; onState("BLE reconnecting", false); g.connect()
                }
            }
            override fun onMtuChanged(g: BluetoothGatt, m: Int, status: Int) { mtu = m; g.discoverServices() }
            override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
                val s = g.getService(SERVICE) ?: run { onState("BLE: no UART service", false); return }
                rx = s.getCharacteristic(RX)
                s.getCharacteristic(TX)?.let { tx ->
                    g.setCharacteristicNotification(tx, true)
                    tx.getDescriptor(CCCD)?.let { desc ->
                        if (Build.VERSION.SDK_INT >= 33) g.writeDescriptor(desc, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
                        else @Suppress("DEPRECATION") { desc.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE; g.writeDescriptor(desc) }
                    }
                }
                if (rx != null) onState("BLE ${d.name ?: "robot"}", true)
            }
            override fun onDescriptorWrite(g: BluetoothGatt, descriptor: BluetoothGattDescriptor, status: Int) { pump() }
            @Deprecated("API < 33")
            override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
                @Suppress("DEPRECATION") c.value?.let { v -> splitter.feed(v).forEach(onLine) }
            }
            override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
                splitter.feed(value).forEach(onLine)
            }
            override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
                synchronized(writeQueue) { writing = false }
                pump()
            }
        }, BluetoothDevice.TRANSPORT_LE)
    }

    fun sendLine(line: String): Boolean {
        if (rx == null) return false
        val data = (line + "\n").toByteArray()
        val chunk = (mtu - 3).coerceAtLeast(20)
        synchronized(writeQueue) {
            if (writeQueue.size > 64) writeQueue.clear()        // the robot is not keeping up: drop stale commands
            var i = 0
            while (i < data.size) { writeQueue.addLast(data.copyOfRange(i, minOf(data.size, i + chunk))); i += chunk }
        }
        pump()
        return true
    }

    private fun pump() {
        val g = gatt ?: return; val c = rx ?: return
        val next = synchronized(writeQueue) {
            if (writing || writeQueue.isEmpty()) return
            writing = true; writeQueue.removeFirst()
        }
        val type = if (c.properties and BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE != 0)
            BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE else BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
        val ok = if (Build.VERSION.SDK_INT >= 33) g.writeCharacteristic(c, next, type) == android.bluetooth.BluetoothStatusCodes.SUCCESS
        else @Suppress("DEPRECATION") run { c.writeType = type; c.value = next; g.writeCharacteristic(c) }
        if (!ok) synchronized(writeQueue) { writing = false }
    }
}
