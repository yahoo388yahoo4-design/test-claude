package com.real2sim.capture

import android.os.SystemClock
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.Response
import okhttp3.WebSocket
import okhttp3.WebSocketListener
import org.json.JSONObject
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicInteger

/**
 * Phone -> robot link, shared protocol with the iOS app (robot/PROTOCOL.md): JSON text frames over a
 * WebSocket, robot = server at ws://<robot>:8766/r2s, units cm, cm/s, deg, deg/s, +theta = left.
 */
class RobotLink(private val onEvent: (String) -> Unit) {
    enum class State { DISCONNECTED, CONNECTING, CONNECTED }

    @Volatile var state = State.DISCONNECTED; private set
    @Volatile var robotName = ""; private set
    @Volatile var maxSpeedCms = 30f; private set
    @Volatile var maxTurnDps = 90f; private set
    @Volatile var rttMs = -1f; private set
    @Volatile var estopLatched = false; private set
    @Volatile var batteryV = Float.NaN; private set
    @Volatile var queue = 0; private set
    /** Latest robot odometry: x_cm, y_cm, theta_deg, v_cms, w_dps, t (robot clock), received (elapsedRealtime ms). */
    @Volatile var odom: FloatArray? = null; private set
    @Volatile var odomRxMs = 0L; private set
    @Volatile var lastDone = ""; private set

    private val ids = AtomicInteger(1)
    private var ws: WebSocket? = null
    private val client = OkHttpClient.Builder().pingInterval(2, TimeUnit.SECONDS).connectTimeout(4, TimeUnit.SECONDS).build()
    private var pingTicker: Thread? = null

    fun connect(urlIn: String, tokenIn: String? = null) {
        close()
        // "192.168.1.20?token=abc" -> token sent in hello (receiver --token abc)
        val token = tokenIn ?: urlIn.substringAfter("token=", "").substringBefore('&').ifBlank { null }
        var url = urlIn.trim().substringBefore('?')
        if (!url.startsWith("ws://") && !url.startsWith("wss://")) url = "ws://$url"
        if (url.count { it == ':' } < 2) url += ":8766"
        if (!url.substringAfter("://").contains('/')) url += "/r2s"
        state = State.CONNECTING
        onEvent("robot: connecting $url")
        ws = client.newWebSocket(Request.Builder().url(url).build(), object : WebSocketListener() {
            override fun onOpen(webSocket: WebSocket, response: Response) {
                state = State.CONNECTED
                send(JSONObject().put("type", "hello").put("v", 1).put("client", "r2s-android").apply { token?.let { put("token", it) } })
                onEvent("robot: connected")
                pingTicker = Thread {
                    try {
                        while (state == State.CONNECTED) {
                            send(JSONObject().put("type", "ping").put("t", SystemClock.elapsedRealtime() / 1000.0))
                            Thread.sleep(1000)
                        }
                    } catch (_: InterruptedException) {}
                }.apply { isDaemon = true; start() }
            }
            override fun onMessage(webSocket: WebSocket, text: String) = handle(text)
            override fun onClosed(webSocket: WebSocket, code: Int, reason: String) { state = State.DISCONNECTED; onEvent("robot: closed $reason") }
            override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) { state = State.DISCONNECTED; onEvent("robot: ${t.message}") }
        })
    }

    private fun handle(text: String) {
        val m = try { JSONObject(text) } catch (_: Exception) { return }
        when (m.optString("type")) {
            "hello" -> {
                robotName = m.optString("robot"); maxSpeedCms = m.optDouble("max_speed_cms", 30.0).toFloat()
                maxTurnDps = m.optDouble("max_turn_dps", 90.0).toFloat()
                onEvent("robot: $robotName (max ${maxSpeedCms.toInt()} cm/s, ${maxTurnDps.toInt()} deg/s)")
            }
            "pong" -> rttMs = ((SystemClock.elapsedRealtime() / 1000.0 - m.optDouble("t")) * 1000).toFloat()
            "odom" -> {
                odom = floatArrayOf(m.optDouble("x_cm").toFloat(), m.optDouble("y_cm").toFloat(), m.optDouble("theta_deg").toFloat(),
                    m.optDouble("v_cms").toFloat(), m.optDouble("w_dps").toFloat(), m.optDouble("t").toFloat())
                odomRxMs = SystemClock.elapsedRealtime()
            }
            "status" -> {
                estopLatched = m.optBoolean("estop"); queue = m.optInt("queue")
                if (m.has("battery_v") && !m.isNull("battery_v")) batteryV = m.optDouble("battery_v").toFloat()
            }
            "done" -> { lastDone = "#${m.optInt("id")} ${if (m.optBoolean("ok")) "done" else "failed: " + m.optString("reason")}"; onEvent("robot: $lastDone") }
            "error" -> onEvent("robot error: ${m.optString("reason")}")
        }
    }

    private fun send(o: JSONObject): Boolean = ws?.send(o.toString()) ?: false

    val connected get() = state == State.CONNECTED

    fun move(distCm: Float, speedCms: Float): Int {
        val id = ids.getAndIncrement()
        send(JSONObject().put("type", "move").put("id", id).put("dist_cm", distCm.toDouble()).put("speed_cms", speedCms.toDouble()))
        return id
    }

    fun turn(angleDeg: Float, speedDps: Float): Int {
        val id = ids.getAndIncrement()
        send(JSONObject().put("type", "turn").put("id", id).put("angle_deg", angleDeg.toDouble()).put("speed_dps", speedDps.toDouble()))
        return id
    }

    /** Continuous velocity (cm/s, deg/s CCW); the robot stops by itself if the next one is late. */
    fun vel(vCms: Float, wDps: Float, ttlMs: Int = 300) =
        send(JSONObject().put("type", "vel").put("v_cms", vCms.toDouble()).put("w_dps", wDps.toDouble()).put("ttl_ms", ttlMs))

    fun stop() = send(JSONObject().put("type", "stop").put("id", ids.getAndIncrement()))
    fun estop() = send(JSONObject().put("type", "estop"))
    fun reset() = send(JSONObject().put("type", "reset"))

    fun close() {
        try { if (connected) stop() } catch (_: Exception) {}
        ws?.close(1000, "bye"); ws = null
        pingTicker?.interrupt(); pingTicker = null
        state = State.DISCONNECTED
    }
}
