package com.real2sim.capture

import org.json.JSONObject
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sqrt

/**
 * Cleans one ARCore depth map (DEPTH16 millimetres, rows top-down, optional 0..255 confidence) before it is
 * used for anything (TSDF fusion, occupancy grid, live view). Pure Kotlin, no Android types.
 *
 * Depth-from-motion (no LiDAR / ToF) is ~160x120, noisy, and smears depth across object silhouettes
 * ("flying pixels" half way between the foreground and the background). Steps, in order:
 *  1. confidence + range gate: raw-depth confidence < [Config.minConfidence] and depth outside
 *     [[Config.minRangeM], [Config.maxRangeM]] are dropped;
 *  2. edge-preserving median: each pixel becomes the median of its 3x3 neighbours that lie on the same
 *     surface (relative difference < 2 x [Config.edgeRelJump]); pixels with fewer than
 *     [Config.minNeighbours] such neighbours are isolated speckle and dropped;
 *  3. flying pixels: a pixel whose depth jumps by more than [Config.edgeRelJump] (relative) both towards a
 *     nearer and towards a farther neighbour lies inside a depth ramp across a silhouette -> dropped
 *     (true silhouette pixels only jump one way and survive);
 *  4. grazing angle: the local surface normal (from neighbouring back-projected pixels) nearly
 *     perpendicular to the viewing ray (> [Config.maxGrazingDeg]) -> dropped (needs intrinsics);
 *  5. temporal free-space check: the previous frame's depth is re-projected; a pixel that the previous
 *     view saw *through* (previous depth farther by > [Config.temporalRelTol]) is dropped. A previous depth
 *     that is nearer is an occlusion, not a contradiction, and is ignored.
 *
 * Not thread safe (scratch buffers + temporal state): use one instance per producer thread.
 */
class DepthFilter(val cfg: Config = Config()) {
    data class Config(
        val minConfidence: Float = 0.5f,
        val minRangeM: Float = 0.2f,
        val maxRangeM: Float = 4.0f,
        val edgeRelJump: Float = 0.06f,
        val maxGrazingDeg: Float = 80f,
        val median: Boolean = true,
        val minNeighbours: Int = 3,
        val temporal: Boolean = true,
        val temporalRelTol: Float = 0.10f,
    ) {
        fun toJson(): JSONObject = JSONObject().apply {
            put("min_confidence", minConfidence.toDouble()); put("min_range_m", minRangeM.toDouble()); put("max_range_m", maxRangeM.toDouble())
            put("edge_rel_jump", edgeRelJump.toDouble()); put("max_grazing_deg", maxGrazingDeg.toDouble())
            put("median_3x3_edge_preserving", median); put("min_neighbours", minNeighbours)
            put("temporal", temporal); put("temporal_rel_tol", temporalRelTol.toDouble())
        }
    }

    /** Pixel counts of the last [filter] call (cumulative counts are in [total]). */
    class Stats {
        var input = 0; var lowConf = 0; var range = 0; var isolated = 0; var flying = 0; var grazing = 0; var temporal = 0; var output = 0
        fun add(o: Stats) { input += o.input; lowConf += o.lowConf; range += o.range; isolated += o.isolated; flying += o.flying; grazing += o.grazing; temporal += o.temporal; output += o.output }
        fun clear() { input = 0; lowConf = 0; range = 0; isolated = 0; flying = 0; grazing = 0; temporal = 0; output = 0 }
        fun toJson(): JSONObject = JSONObject().apply {
            put("input_px", input); put("low_confidence", lowConf); put("out_of_range", range); put("isolated", isolated)
            put("flying", flying); put("grazing", grazing); put("temporal", temporal); put("output_px", output)
        }
    }

    val last = Stats()
    val total = Stats()

    private var a = FloatArray(0)
    private var b = FloatArray(0)
    private val win = FloatArray(9)

    // temporal state: previous spatially-filtered depth + its camera
    private var prev: FloatArray? = null
    private var prevW = 0; private var prevH = 0
    private val prevPose = FloatArray(16)
    private var prevK = FloatArray(4)

    fun resetTemporal() { prev = null }

    /**
     * [depthMm] w*h DEPTH16 values (0 = none), [conf] optional w*h confidence 0..255. Intrinsics at the depth
     * resolution (fx <= 0 disables the grazing test). [pose] column-major camera-to-world (ARCore OpenGL
     * camera: +x right, +y up, -z forward) or null (disables the temporal test).
     * Returns metres, 0 = invalid, in [out] if it has the right size.
     */
    fun filter(depthMm: ShortArray, conf: ByteArray?, w: Int, h: Int, fx: Float, fy: Float, cx: Float, cy: Float,
               pose: FloatArray?, out: FloatArray? = null): FloatArray {
        val n = w * h
        if (a.size != n) { a = FloatArray(n); b = FloatArray(n) }
        val st = last; st.clear()
        val minC = (cfg.minConfidence * 255f).toInt()
        // 1. confidence + range
        for (i in 0 until n) {
            val raw = depthMm[i].toInt() and 0xffff
            if (raw == 0) { a[i] = 0f; continue }
            st.input++
            if (conf != null && (conf[i].toInt() and 0xff) < minC) { a[i] = 0f; st.lowConf++; continue }
            val d = raw / 1000f
            if (d < cfg.minRangeM || d > cfg.maxRangeM) { a[i] = 0f; st.range++; continue }
            a[i] = d
        }
        // 2. edge-preserving median + isolation
        val band = 2f * cfg.edgeRelJump
        for (v in 0 until h) for (u in 0 until w) {
            val i = v * w + u
            val d = a[i]
            if (d == 0f) { b[i] = 0f; continue }
            var k = 0
            for (dv in -1..1) {
                val vv = v + dv; if (vv < 0 || vv >= h) continue
                for (du in -1..1) {
                    val uu = u + du; if (uu < 0 || uu >= w) continue
                    val e = a[vv * w + uu]
                    if (e != 0f && abs(e - d) < band * d) win[k++] = e
                }
            }
            if (k - 1 < cfg.minNeighbours) { b[i] = 0f; st.isolated++; continue }
            b[i] = if (cfg.median) median(win, k) else d
        }
        // 3. flying pixels (on b) -> a
        val t = cfg.edgeRelJump
        for (v in 0 until h) for (u in 0 until w) {
            val i = v * w + u
            val d = b[i]
            if (d == 0f) { a[i] = 0f; continue }
            var near = 0f; var far = 0f
            for (dv in -1..1) {
                val vv = v + dv; if (vv < 0 || vv >= h) continue
                for (du in -1..1) {
                    val uu = u + du; if (uu < 0 || uu >= w || (du == 0 && dv == 0)) continue
                    val e = b[vv * w + uu]
                    if (e == 0f) continue
                    val r = (e - d) / d
                    if (r > far) far = r
                    if (-r > near) near = -r
                }
            }
            if (near > t && far > t) { a[i] = 0f; st.flying++ } else a[i] = d
        }
        // 4. grazing angle (on a, written to b)
        if (fx > 0f && fy > 0f && cfg.maxGrazingDeg < 90f) {
            val cosMin = cos(Math.toRadians(cfg.maxGrazingDeg.toDouble())).toFloat()
            for (v in 0 until h) for (u in 0 until w) {
                val i = v * w + u
                val d = a[i]
                if (d == 0f) { b[i] = 0f; continue }
                // horizontal and vertical neighbours (central difference, one-sided at holes / borders)
                val il = if (u > 0 && a[i - 1] != 0f) i - 1 else i
                val ir = if (u < w - 1 && a[i + 1] != 0f) i + 1 else i
                val iu = if (v > 0 && a[i - w] != 0f) i - w else i
                val id = if (v < h - 1 && a[i + w] != 0f) i + w else i
                if (il == ir || iu == id) { b[i] = d; continue }
                // back-project (camera frame, z forward positive for simplicity: the angle is invariant)
                fun px(j: Int) = ((j % w) - cx) / fx * a[j]
                fun py(j: Int) = ((j / w) - cy) / fy * a[j]
                val ax = px(ir) - px(il); val ay = py(ir) - py(il); val az = a[ir] - a[il]
                val bx = px(id) - px(iu); val by = py(id) - py(iu); val bz = a[id] - a[iu]
                val nx = ay * bz - az * by; val ny = az * bx - ax * bz; val nz = ax * by - ay * bx
                val rx = (u - cx) / fx; val ry = (v - cy) / fy; val rz = 1f
                val nn = sqrt(nx * nx + ny * ny + nz * nz); val rn = sqrt(rx * rx + ry * ry + rz * rz)
                if (nn < 1e-12f) { b[i] = d; continue }
                val c = abs(nx * rx + ny * ry + nz * rz) / (nn * rn)
                if (c < cosMin) { b[i] = 0f; st.grazing++ } else b[i] = d
            }
        } else System.arraycopy(a, 0, b, 0, n)
        // 5. temporal free-space check against the previous frame
        val res = if (out != null && out.size == n) out else FloatArray(n)
        val p = prev
        if (cfg.temporal && pose != null && p != null && fx > 0f) {
            val m = pose; val q = prevPose
            val pfx = prevK[0]; val pfy = prevK[1]; val pcx = prevK[2]; val pcy = prevK[3]
            for (v in 0 until h) for (u in 0 until w) {
                val i = v * w + u
                val d = b[i]
                res[i] = d
                if (d == 0f) continue
                // current camera -> world
                val xc = (u - cx) / fx * d; val yc = -(v - cy) / fy * d; val zc = -d
                val wx = m[0] * xc + m[4] * yc + m[8] * zc + m[12]
                val wy = m[1] * xc + m[5] * yc + m[9] * zc + m[13]
                val wz = m[2] * xc + m[6] * yc + m[10] * zc + m[14]
                // world -> previous camera (R^T (p - t))
                val dx = wx - q[12]; val dy = wy - q[13]; val dz = wz - q[14]
                val px = q[0] * dx + q[1] * dy + q[2] * dz
                val py = q[4] * dx + q[5] * dy + q[6] * dz
                val pz = -(q[8] * dx + q[9] * dy + q[10] * dz)
                if (pz < 0.05f) continue
                val pu = Math.round(pfx * px / pz + pcx); val pv = Math.round(pcy - pfy * py / pz)
                if (pu < 0 || pv < 0 || pu >= prevW || pv >= prevH) continue
                val e = p[pv * prevW + pu]
                if (e > 0f && e - pz > cfg.temporalRelTol * pz) { res[i] = 0f; st.temporal++ }
            }
        } else System.arraycopy(b, 0, res, 0, n)
        // keep the spatially filtered map (not the temporal result: no cascades) for the next frame
        if (cfg.temporal && pose != null) {
            if (p == null || p.size != n) prev = b.copyOf() else System.arraycopy(b, 0, p, 0, n)
            prevW = w; prevH = h
            System.arraycopy(pose, 0, prevPose, 0, 16)
            prevK = floatArrayOf(fx, fy, cx, cy)
        }
        for (i in 0 until n) if (res[i] > 0f) st.output++
        total.add(st)
        return res
    }

    companion object {
        private fun median(v: FloatArray, k: Int): Float {
            // insertion sort of <= 9 values
            for (i in 1 until k) {
                val x = v[i]; var j = i - 1
                while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j-- }
                v[j + 1] = x
            }
            return if (k % 2 == 1) v[k / 2] else 0.5f * (v[k / 2 - 1] + v[k / 2])
        }

        /** Little-endian DEPTH16 bytes (as packed by SessionWriter.packPlane) -> shorts. */
        fun shortsLE(bytes: ByteArray, n: Int): ShortArray {
            val s = ShortArray(n)
            java.nio.ByteBuffer.wrap(bytes).order(java.nio.ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(s, 0, minOf(n, bytes.size / 2))
            return s
        }
    }
}
