package com.real2sim.capture

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.view.View
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONObject
import java.io.File
import java.util.concurrent.TimeUnit

/**
 * Sends a room map (roommap/room_map.json) to the Home Assistant "Real2Sim Home" integration
 * (homeassistant/custom_components/real2sim_home): POST <ha>/api/real2sim_home/room_map with a long-lived
 * access token. The integration stores it and shows the room on its floor-plan card.
 */
object HomeAssistant {
    private val client by lazy { OkHttpClient.Builder().connectTimeout(10, TimeUnit.SECONDS).readTimeout(30, TimeUnit.SECONDS).build() }

    fun endpoint(base: String): String = base.trim().trimEnd('/') + "/api/real2sim_home/room_map"

    /** Blocking; returns a status line for the UI. */
    fun sendRoomMap(base: String, token: String, json: JSONObject): String {
        if (base.isBlank() || token.isBlank()) return "Home Assistant: set the URL and token in Settings"
        val req = Request.Builder().url(endpoint(base))
            .header("Authorization", "Bearer ${token.trim()}")
            .post(json.toString().toRequestBody("application/json".toMediaType()))
            .build()
        return try {
            client.newCall(req).execute().use { r ->
                if (r.isSuccessful) "sent room \"${json.optString("name")}\" (${json.optJSONArray("objects")?.length() ?: 0} objects) to Home Assistant"
                else "Home Assistant answered ${r.code}: ${r.body?.string()?.take(200)}"
            }
        } catch (e: Exception) { "Home Assistant unreachable: ${e.message}" }
    }

    /** The session's room map, with the RuView nodes of esp_nodes.json (when placed) as its `sensors` layer. */
    fun roomMapOf(sessionDir: File): JSONObject? {
        val f = File(sessionDir, "roommap/room_map.json").takeIf { it.isFile } ?: return null
        val json = try { JSONObject(f.readText()) } catch (_: Exception) { return null }
        if (RoomMapJson.withEspNodes(json, sessionDir)) try { f.writeText(json.toString(1)) } catch (_: Exception) {}
        return json
    }
}

/** Live boxes of the object detector over the camera preview (upright image assumed to fill the view, centre-cropped). */
class DetectionOverlay(ctx: Context) : View(ctx) {
    private var dets: List<RoomScanner.Det2D> = emptyList()
    private var aspect = 0.75f      // width / height of the upright camera image
    private val box = Paint().apply { style = Paint.Style.STROKE; strokeWidth = 4f; isAntiAlias = true }
    private val text = Paint().apply { color = Color.WHITE; textSize = 34f; isAntiAlias = true; isFakeBoldText = true }
    private val bg = Paint().apply { style = Paint.Style.FILL }

    fun set(d: List<RoomScanner.Det2D>, imageAspect: Float) { dets = d; aspect = imageAspect; invalidate() }

    override fun onDraw(c: Canvas) {
        if (dets.isEmpty() || width == 0) return
        val vw = width.toFloat(); val vh = height.toFloat()
        // centre-crop: image scaled to cover the view
        val s = maxOf(vw / aspect, vh)
        val iw = aspect * s; val ih = s
        val ox = (vw - iw) / 2; val oy = (vh - ih) / 2
        for (d in dets) {
            val col = if (d.placed) Color.rgb(52, 199, 89) else Color.rgb(255, 204, 0)
            box.color = col; bg.color = (col and 0x00ffffff) or (0xc0 shl 24)
            val r = RectF(ox + d.l * iw, oy + d.t * ih, ox + d.r * iw, oy + d.b * ih)
            c.drawRoundRect(r, 10f, 10f, box)
            val label = "${d.label} ${(d.score * 100).toInt()}%"
            val tw = text.measureText(label)
            c.drawRoundRect(RectF(r.left, r.top - 44f, r.left + tw + 16f, r.top), 8f, 8f, bg)
            c.drawText(label, r.left + 8f, r.top - 12f, text)
        }
    }
}
