package com.real2sim.capture

import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Positions of the ESP32 sensing nodes in a capture session (sensing/esp32/README.md).
 *
 * File: <session>/esp_nodes.json, written live while recording (tag tracking / tap on the ESP) and from
 * the session viewer (tap the ESP in a video frame). Same schema as sensing/esp32/esp_common.py:
 * {"version":1,"format":"r2s-esp-nodes","frame":"session_world","units":"m","up":"+y",
 *  "nodes":[{"node_id":1,"label":"esp-1","position":[x,y,z],"normal":[..]|null,"method":"...",...}]}
 * session_world = the ARCore world of the recording (y up, metres), the frame frames.jsonl poses use.
 */
class EspNode(
    val id: Int,
    val pos: DoubleArray,
    val normal: DoubleArray?,
    val method: String,
    val frameIndex: Int? = null,
    val t: Double? = null,
    val confidence: Double = 1.0,
    val observations: Int = 1,
    val updated: String = EspMath.nowIso(),
)

object EspMath {
    const val NODE_COUNT = 6
    const val TAG_SIZE_M = 0.070f

    /** A person's tap beats automatic detection; nothing beats a tap except another tap. */
    fun rank(method: String?): Int = when (method) {
        "tap_frame", "tap_live", "triangulated" -> 4
        "auto_tag", "auto_tag_live" -> 2
        "planned" -> 0
        else -> 1
    }

    fun nowIso(): String = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ssXXX", Locale.US).apply {
        timeZone = TimeZone.getTimeZone("UTC")
    }.format(Date())

    /** K = fx,fy,cx,cy at (w,h); T = 4x4 row-major camera-to-world with ARKit/ARCore camera axes (looks down -z). */
    fun unproject(K: DoubleArray, T: DoubleArray, u: Double, v: Double, depthM: Double): DoubleArray {
        val x = (u - K[2]) / K[0] * depthM
        val y = -(v - K[3]) / K[1] * depthM
        val z = -depthM
        return DoubleArray(3) { r -> T[r * 4] * x + T[r * 4 + 1] * y + T[r * 4 + 2] * z + T[r * 4 + 3] }
    }

    /** World ray (origin, unit direction) through pixel (u, v). */
    fun ray(K: DoubleArray, T: DoubleArray, u: Double, v: Double): Pair<DoubleArray, DoubleArray> {
        val dc = doubleArrayOf((u - K[2]) / K[0], -(v - K[3]) / K[1], -1.0)
        val d = DoubleArray(3) { r -> T[r * 4] * dc[0] + T[r * 4 + 1] * dc[1] + T[r * 4 + 2] * dc[2] }
        val n = norm(d)
        return Pair(doubleArrayOf(T[3], T[7], T[11]), DoubleArray(3) { d[it] / n })
    }

    /** World point -> (u, v, depth) in the image of K/T, or null when behind the camera. */
    fun project(K: DoubleArray, T: DoubleArray, p: DoubleArray): DoubleArray? {
        val q = doubleArrayOf(p[0] - T[3], p[1] - T[7], p[2] - T[11])
        // R^T q
        val cx = T[0] * q[0] + T[4] * q[1] + T[8] * q[2]
        val cy = T[1] * q[0] + T[5] * q[1] + T[9] * q[2]
        val cz = T[2] * q[0] + T[6] * q[1] + T[10] * q[2]
        val depth = -cz
        if (depth <= 1e-6) return null
        return doubleArrayOf(K[0] * cx / depth + K[2], K[1] * (-cy) / depth + K[3], depth)
    }

    /** Closest point to >= 2 rays: (point, rms distance to the rays, largest angle between rays in degrees). */
    fun triangulate(rays: List<Pair<DoubleArray, DoubleArray>>): Triple<DoubleArray, Double, Double>? {
        if (rays.size < 2) return null
        val a = Array(3) { DoubleArray(3) }
        val b = DoubleArray(3)
        for ((o, d) in rays) {
            for (r in 0 until 3) for (c in 0 until 3) {
                val p = (if (r == c) 1.0 else 0.0) - d[r] * d[c]
                a[r][c] += p
                b[r] += p * o[c]
            }
        }
        val p = solve3(a, b) ?: return null
        var ss = 0.0
        for ((o, d) in rays) {
            val w = doubleArrayOf(p[0] - o[0], p[1] - o[1], p[2] - o[2])
            val t = dot(w, d)
            ss += sq(w[0] - t * d[0]) + sq(w[1] - t * d[1]) + sq(w[2] - t * d[2])
        }
        var ang = 0.0
        for (i in rays.indices) for (j in i + 1 until rays.size) {
            val c = min(1.0, abs(dot(rays[i].second, rays[j].second)))
            ang = max(ang, Math.toDegrees(acos(c)))
        }
        return Triple(p, sqrt(ss / rays.size), ang)
    }

    /** Median valid depth (m) around RGB pixel (u, v) of a (w, h) image; the depth map is (dw, dh) uint16 mm. */
    fun depthAt(mm: ShortArray, dw: Int, dh: Int, w: Int, h: Int, u: Double, v: Double, r: Int = 1): Double? {
        val du = Math.round(u * dw / w).toInt(); val dv = Math.round(v * dh / h).toInt()
        if (du !in 0 until dw || dv !in 0 until dh) return null
        val vals = ArrayList<Int>()
        for (y in max(0, dv - r)..min(dh - 1, dv + r)) for (x in max(0, du - r)..min(dw - 1, du + r)) {
            val z = mm[y * dw + x].toInt() and 0xffff
            if (z > 0) vals.add(z)
        }
        if (vals.isEmpty()) return null
        vals.sort()
        return vals[vals.size / 2] / 1000.0
    }

    /** Per-axis median: robust centre of repeated sightings. */
    fun median(points: List<DoubleArray>): DoubleArray = DoubleArray(3) { k ->
        val s = points.map { it[k] }.sorted()
        if (s.size % 2 == 1) s[s.size / 2] else (s[s.size / 2 - 1] + s[s.size / 2]) / 2
    }

    fun dot(a: DoubleArray, b: DoubleArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    fun norm(a: DoubleArray) = sqrt(dot(a, a))
    private fun sq(x: Double) = x * x

    private fun solve3(a: Array<DoubleArray>, b: DoubleArray): DoubleArray? {
        fun det(m: Array<DoubleArray>) =
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        val d = det(a)
        if (abs(d) < 1e-12) return null
        return DoubleArray(3) { c -> det(Array(3) { r -> DoubleArray(3) { k -> if (k == c) b[r] else a[r][k] } }) / d }
    }
}

/** esp_nodes.json of one session. Thread-safe; call [save] after changes. */
class EspNodeStore(private val file: File?) {
    private val nodes = sortedMapOf<Int, EspNode>()
    private var extra = JSONObject()
    @Volatile var dirty = false
        private set

    init { file?.let { if (it.isFile) load(it) } }

    @Synchronized fun all(): List<EspNode> = nodes.values.toList()
    @Synchronized fun get(id: Int): EspNode? = nodes[id]

    /** Insert or replace; a lower-ranked method never replaces a higher-ranked one unless [force]. */
    @Synchronized fun put(n: EspNode, force: Boolean = false): Boolean {
        val old = nodes[n.id]
        if (old != null && !force && EspMath.rank(old.method) > EspMath.rank(n.method)) return false
        nodes[n.id] = n
        dirty = true
        return true
    }

    @Synchronized fun remove(id: Int) { if (nodes.remove(id) != null) dirty = true }

    @Synchronized fun toJson(): JSONObject = JSONObject(extra.toString()).apply {
        put("version", 1); put("format", "r2s-esp-nodes"); put("frame", "session_world"); put("units", "m"); put("up", "+y")
        put("nodes", JSONArray().apply {
            for (n in nodes.values) put(JSONObject().apply {
                put("node_id", n.id); put("label", "esp-${n.id}")
                put("position", JSONArray(n.pos.map { Math.round(it * 10000) / 10000.0 }))
                put("normal", n.normal?.let { v -> JSONArray(v.map { Math.round(it * 10000) / 10000.0 }) } ?: JSONObject.NULL)
                put("method", n.method)
                put("frame_index", n.frameIndex ?: JSONObject.NULL)
                put("t", n.t ?: JSONObject.NULL)
                put("confidence", Math.round(n.confidence * 1000) / 1000.0)
                put("observations", n.observations)
                put("updated", n.updated)
            })
        })
    }

    @Synchronized fun save(): Boolean {
        val f = file ?: return false
        return try {
            val tmp = File(f.parentFile, f.name + ".tmp")
            tmp.writeText(toJson().toString(1))
            if (!tmp.renameTo(f)) { f.delete(); tmp.renameTo(f) }
            dirty = false
            true
        } catch (_: Exception) { false }
    }

    @Synchronized private fun load(f: File) {
        try {
            val o = JSONObject(f.readText())
            extra = JSONObject(o.toString()).apply { remove("nodes") }
            val arr = o.optJSONArray("nodes") ?: return
            for (k in 0 until arr.length()) {
                val n = arr.getJSONObject(k)
                val p = n.getJSONArray("position")
                val nv = n.optJSONArray("normal")
                nodes[n.getInt("node_id")] = EspNode(
                    n.getInt("node_id"), DoubleArray(3) { p.getDouble(it) },
                    nv?.let { a -> DoubleArray(3) { a.getDouble(it) } }, n.optString("method", "unknown"),
                    if (n.isNull("frame_index")) null else n.optInt("frame_index"),
                    if (n.isNull("t")) null else n.optDouble("t"),
                    n.optDouble("confidence", 1.0), n.optInt("observations", 1), n.optString("updated", ""),
                )
            }
        } catch (_: Exception) {}
    }

    companion object {
        const val FILE = "esp_nodes.json"
        fun forSession(dir: File) = EspNodeStore(File(dir, FILE))
    }
}
