package com.real2sim.capture

import android.graphics.Bitmap
import android.graphics.Color
import org.json.JSONObject
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.zip.Deflater
import java.util.zip.DeflaterOutputStream
import java.util.zip.Inflater
import java.util.zip.InflaterInputStream
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Live 3D/2D map built from ARCore depth + camera poses, in ARCore world coordinates
 * (y up, gravity aligned; x/z horizontal).
 *
 *  - occupancy grid on the floor plane (x right, z towards the viewer at session start), log-odds per
 *    cell: depth points between [floorY + floorBand, floorY + maxObstacleHeight] mark a cell occupied,
 *    the ray from the camera to each point (in x/z) marks free space, points near the floor mark free.
 *  - a voxel-downsampled point cloud of everything seen (for the live 3D view).
 *  - the camera trajectory.
 *  - a "virtual lidar": per-bearing nearest obstacle distance around the robot, from the newest depth
 *    map (front) merged with ray casts in the grid (all around).
 *
 * All public methods are synchronized; callers are the GL thread (integrate) and the UI / planner
 * threads (snapshots).
 */
class MapBuilder(val res: Float = 0.05f, val size: Int = 800) {
    /** World x/z of grid cell (0,0)'s corner. Set by [reset] around the first pose. */
    var originX = 0f; private set
    var originZ = 0f; private set
    var initialized = false; private set

    /** Floor height in ARCore world y. Updated from planes or from the mount height. */
    @Volatile var floorY = Float.NaN
    @Volatile var floorBand = 0.06f
    @Volatile var maxObstacleHeight = 1.8f
    @Volatile var maxRange = 4.0f

    val occ = ByteArray(size * size)          // log-odds, -LMAX..LMAX, 0 unknown
    private val seen = BooleanArray(size * size)
    var knownCells = 0; private set

    // ---- point cloud (voxel hash) ----
    val voxel = 0.04f
    val maxPoints = 250_000
    val points = FloatArray(maxPoints * 3)
    var pointCount = 0; private set
    var version = 0; private set
    private val voxels = HashSet<Long>(1 shl 16)

    // ---- trajectory ----
    val traj = ArrayList<FloatArray>()            // [x, y, z, t]
    var travelled = 0f; private set

    // ---- virtual lidar ----
    val bins = 72                                  // 5 degrees each, bin 0 = straight ahead, CCW positive
    val depthScan = FloatArray(bins) { Float.POSITIVE_INFINITY }
    var depthScanT = 0L; private set

    fun reset(x: Float, z: Float) = synchronized(this) {
        originX = x - size * res / 2; originZ = z - size * res / 2
        occ.fill(0); seen.fill(false); knownCells = 0
        voxels.clear(); pointCount = 0; version++
        traj.clear(); travelled = 0f
        initialized = true
    }

    fun cellI(x: Float) = floor((x - originX) / res).toInt()
    fun cellJ(z: Float) = floor((z - originZ) / res).toInt()
    fun cellX(i: Int) = originX + (i + 0.5f) * res
    fun cellZ(j: Int) = originZ + (j + 0.5f) * res
    fun inside(i: Int, j: Int) = i in 0 until size && j in 0 until size

    fun isOcc(i: Int, j: Int) = inside(i, j) && occ[j * size + i] > OCC_T
    fun isFree(i: Int, j: Int) = inside(i, j) && occ[j * size + i] < FREE_T
    fun isUnknown(i: Int, j: Int) = !inside(i, j) || (occ[j * size + i] in FREE_T..OCC_T)

    private fun bump(i: Int, j: Int, d: Int) {
        if (!inside(i, j)) return
        val k = j * size + i
        if (!seen[k]) { seen[k] = true; knownCells++ }
        occ[k] = (occ[k] + d).coerceIn(-LMAX, LMAX).toByte()
    }

    fun knownAreaM2() = knownCells * res * res

    fun addPose(x: Float, y: Float, z: Float, tNs: Long) = synchronized(this) {
        if (!initialized) reset(x, z)
        val last = traj.lastOrNull()
        if (last == null || hypot(x - last[0], z - last[2]) > 0.05f) {
            if (last != null) travelled += hypot(x - last[0], z - last[2])
            traj.add(floatArrayOf(x, y, z, tNs / 1e9f))
            if (traj.size > 20000) traj.subList(0, 5000).clear()
        }
    }

    /**
     * Integrate one DEPTH16 map (mm, 0 = no data). [m] is the camera pose (column-major 4x4, ARCore
     * OpenGL camera: +x right, +y up, -z forward relative to the image). Intrinsics are at depth
     * resolution. [heading] is the robot heading (rad, atan2(z, x) of the forward direction) used to
     * fill [depthScan]. Returns the number of points used.
     */
    fun integrateDepth(buf: ByteBuffer, w: Int, h: Int, rowStride: Int, fx: Float, fy: Float, cx: Float, cy: Float,
                       m: FloatArray, heading: Float, tNs: Long, step: Int = 2): Int = synchronized(this) {
        val b = buf.duplicate().order(ByteOrder.LITTLE_ENDIAN)
        val camX = m[12]; val camY = m[13]; val camZ = m[14]
        if (!initialized) reset(camX, camZ)
        val floor = if (floorY.isNaN()) camY - 1.4f else floorY
        val ci = cellI(camX); val cj = cellJ(camZ)
        depthScan.fill(Float.POSITIVE_INFINITY)
        var used = 0
        var ray = 0
        var v = 0
        while (v < h) {
            var u = 0
            while (u < w) {
                val raw = b.getShort(v * rowStride + u * 2).toInt() and 0xffff
                val d = raw / 1000f
                if (raw > 0 && d < maxRange) {
                    val xc = (u - cx) / fx * d
                    val yc = -(v - cy) / fy * d
                    val zc = -d
                    val wx = m[0] * xc + m[4] * yc + m[8] * zc + camX
                    val wy = m[1] * xc + m[5] * yc + m[9] * zc + camY
                    val wz = m[2] * xc + m[6] * yc + m[10] * zc + camZ
                    val hgt = wy - floor
                    val pi = cellI(wx); val pj = cellJ(wz)
                    if (hgt < floorBand) {
                        bump(pi, pj, -2)
                    } else if (hgt < maxObstacleHeight) {
                        bump(pi, pj, 4)
                        if ((ray++ and 3) == 0) carve(ci, cj, pi, pj)
                        val dx = wx - camX; val dz = wz - camZ
                        val r = hypot(dx, dz)
                        var a = atan2(dz, dx) - heading
                        while (a > Math.PI) a -= (2 * Math.PI).toFloat()
                        while (a < -Math.PI) a += (2 * Math.PI).toFloat()
                        // bearing: CCW positive seen from above. Grid z grows "down" in the top view, so negate.
                        val bin = binOf(-a)
                        if (r < depthScan[bin]) depthScan[bin] = r
                    }
                    addVoxel(wx, wy, wz)
                    used++
                }
                u += step
            }
            v += step
        }
        depthScanT = tNs
        version++
        used
    }

    fun binOf(bearing: Float): Int {
        val deg = Math.toDegrees(bearing.toDouble())
        return (((Math.round(deg / (360.0 / bins)) % bins) + bins) % bins).toInt()
    }

    fun binBearing(bin: Int): Float = Math.toRadians((if (bin > bins / 2) bin - bins else bin) * 360.0 / bins).toFloat()

    private fun carve(i0: Int, j0: Int, i1: Int, j1: Int) {
        // Bresenham from the camera cell to (excluding) the hit cell.
        var x = i0; var y = j0
        val dx = abs(i1 - i0); val dy = -abs(j1 - j0)
        val sx = if (i0 < i1) 1 else -1; val sy = if (j0 < j1) 1 else -1
        var err = dx + dy
        var n = 0
        while (!(x == i1 && y == j1) && n++ < 200) {
            bump(x, y, -1)
            val e2 = 2 * err
            if (e2 >= dy) { err += dy; x += sx }
            if (e2 <= dx) { err += dx; y += sy }
        }
    }

    private fun addVoxel(x: Float, y: Float, z: Float) {
        if (pointCount >= maxPoints) return
        val key = (floor(x / voxel).toLong() and 0x1fffff) or
            ((floor(y / voxel).toLong() and 0x1fffff) shl 21) or
            ((floor(z / voxel).toLong() and 0x1fffff) shl 42)
        if (!voxels.add(key)) return
        val o = pointCount * 3
        points[o] = x; points[o + 1] = y; points[o + 2] = z
        pointCount++
    }

    /** Copy of the point cloud for rendering. */
    fun copyPoints(dst: FloatArray): Int = synchronized(this) {
        val n = min(pointCount * 3, dst.size)
        System.arraycopy(points, 0, dst, 0, n)
        n / 3
    }

    /**
     * Nearest obstacle per bearing bin around (x, z), CCW positive from [heading], merging the newest
     * depth scan (if fresher than [maxAgeNs]) with grid ray casts up to [range].
     */
    fun scan(x: Float, z: Float, heading: Float, nowNs: Long, out: FloatArray, range: Float = 4f, maxAgeNs: Long = 500_000_000L) = synchronized(this) {
        val fresh = nowNs - depthScanT < maxAgeNs
        for (b in 0 until bins) {
            val a = heading - binBearing(b)          // grid z is "down": CCW bearing -> subtract
            val ca = cos(a); val sa = sin(a)
            var r = Float.POSITIVE_INFINITY
            var s = res
            while (s <= range) {
                val i = cellI(x + ca * s); val j = cellJ(z + sa * s)
                if (isOcc(i, j)) { r = s; break }
                s += res
            }
            out[b] = if (fresh) min(r, depthScan[b]) else r
        }
    }

    /** Top-down image: unknown dark, free light, occupied red. Crops to the known area (+margin). */
    fun bitmap(): Pair<Bitmap, FloatArray>? = synchronized(this) {
        if (!initialized) return null
        var i0 = size; var j0 = size; var i1 = -1; var j1 = -1
        for (j in 0 until size) for (i in 0 until size) if (seen[j * size + i]) {
            if (i < i0) i0 = i; if (i > i1) i1 = i; if (j < j0) j0 = j; if (j > j1) j1 = j
        }
        traj.forEach { p -> val i = cellI(p[0]); val j = cellJ(p[2])
            if (inside(i, j)) { i0 = min(i0, i); i1 = max(i1, i); j0 = min(j0, j); j1 = max(j1, j) } }
        if (i1 < 0) return null
        val mgn = 20
        i0 = max(0, i0 - mgn); j0 = max(0, j0 - mgn); i1 = min(size - 1, i1 + mgn); j1 = min(size - 1, j1 + mgn)
        val w = i1 - i0 + 1; val h = j1 - j0 + 1
        val px = IntArray(w * h)
        for (j in 0 until h) for (i in 0 until w) {
            val v = occ[(j + j0) * size + (i + i0)].toInt()
            px[j * w + i] = when {
                v > OCC_T -> Color.rgb(230, 60, 50)
                v < FREE_T -> Color.rgb(205, 215, 225)
                v != 0 -> Color.rgb(120, 125, 130)
                else -> Color.rgb(45, 48, 52)
            }
        }
        val bmp = Bitmap.createBitmap(px, w, h, Bitmap.Config.ARGB_8888)
        // world rect of the bitmap: x0, z0, x1, z1
        Pair(bmp, floatArrayOf(originX + i0 * res, originZ + j0 * res, originX + (i1 + 1) * res, originZ + (j1 + 1) * res))
    }

    fun occCopy(): ByteArray = synchronized(this) { occ.copyOf() }

    // ------------------------------------------------------------------ save / load / merge
    fun save(dir: File, extra: JSONObject = JSONObject()) = synchronized(this) {
        dir.mkdirs()
        DeflaterOutputStream(FileOutputStream(File(dir, "occupancy.bin")), Deflater(6, true)).use { it.write(occ) }
        val meta = JSONObject(extra.toString()).apply {
            put("format", "r2s-occupancy"); put("version", 1)
            put("res", res.toDouble()); put("size", size); put("origin_x", originX.toDouble()); put("origin_z", originZ.toDouble())
            put("floor_y", if (floorY.isNaN()) JSONObject.NULL else floorY.toDouble())
            put("encoding", "occupancy.bin = raw-deflate int8 log-odds, row-major [z][x], >$OCC_T occupied, <$FREE_T free")
            put("frame", "ARCore world of the recording session (y up); cell (i,j) covers x = origin_x + i*res, z = origin_z + j*res")
            put("known_area_m2", knownAreaM2().toDouble()); put("travelled_m", travelled.toDouble())
            put("points", pointCount)
        }
        File(dir, "map.json").writeText(meta.toString(1))
        bitmap()?.first?.let { b -> FileOutputStream(File(dir, "occupancy.png")).use { b.compress(Bitmap.CompressFormat.PNG, 100, it) } }
        // trajectory + point cloud for offline use
        File(dir, "trajectory.csv").bufferedWriter().use { w ->
            w.write("t,x,y,z\n"); traj.forEach { p -> w.write("${p[3]},${p[0]},${p[1]},${p[2]}\n") }
        }
        FileOutputStream(File(dir, "points.ply")).use { o ->
            o.write("ply\nformat binary_little_endian 1.0\nelement vertex $pointCount\nproperty float x\nproperty float y\nproperty float z\nend_header\n".toByteArray())
            val bb = ByteBuffer.allocate(pointCount * 12).order(ByteOrder.LITTLE_ENDIAN)
            for (k in 0 until pointCount * 3) bb.putFloat(points[k])
            o.write(bb.array())
        }
    }

    /**
     * Rigid 2D transform q = R(theta) p + t (x/z plane) from this map's frame into another map's frame,
     * and the alignment score in [0, 1].
     */
    data class Align(val theta: Float, val tx: Float, val tz: Float, val score: Float)

    /** Copy cells of [other] (whose frame relates to ours by [a]: other = R p + t) into cells we don't know yet. */
    fun mergeFrom(other: MapBuilder, a: Align) = synchronized(this) {
        val c = cos(a.theta); val s = sin(a.theta)
        val o = other.occCopy()
        for (j in 0 until other.size) for (i in 0 until other.size) {
            val v = o[j * other.size + i]
            if (v.toInt() == 0) continue
            // other cell centre q -> our p = R^T (q - t)
            val qx = other.cellX(i) - a.tx; val qz = other.cellZ(j) - a.tz
            val px = c * qx + s * qz; val pz = -s * qx + c * qz
            val pi = cellI(px); val pj = cellJ(pz)
            if (!inside(pi, pj)) continue
            val k = pj * size + pi
            if (occ[k].toInt() == 0) { occ[k] = v; if (!seen[k]) { seen[k] = true; knownCells++ } }
        }
        version++
    }

    /** Occupied cell centres (subsampled to at most [max]). */
    fun occupiedPoints(max: Int): List<FloatArray> = synchronized(this) {
        val all = ArrayList<FloatArray>()
        for (j in 0 until size) for (i in 0 until size) if (occ[j * size + i] > OCC_T) all.add(floatArrayOf(cellX(i), cellZ(j)))
        if (all.size <= max) all else { val stride = all.size.toFloat() / max; List(max) { all[(it * stride).toInt()] } }
    }

    /**
     * Likelihood field of occupied cells: 255 on an obstacle, falling off with distance (sigma ~ 10 cm).
     * Used to score candidate alignments.
     */
    fun likelihood(): ByteArray = synchronized(this) {
        val inf = 1e9f
        val d = FloatArray(size * size) { if (occ[it] > OCC_T) 0f else inf }
        val d1 = 1f; val d2 = 1.4142f
        for (j in 0 until size) for (i in 0 until size) {
            val k = j * size + i
            var v = d[k]
            if (i > 0) v = min(v, d[k - 1] + d1)
            if (j > 0) { v = min(v, d[k - size] + d1); if (i > 0) v = min(v, d[k - size - 1] + d2); if (i < size - 1) v = min(v, d[k - size + 1] + d2) }
            d[k] = v
        }
        for (j in size - 1 downTo 0) for (i in size - 1 downTo 0) {
            val k = j * size + i
            var v = d[k]
            if (i < size - 1) v = min(v, d[k + 1] + d1)
            if (j < size - 1) { v = min(v, d[k + size] + d1); if (i < size - 1) v = min(v, d[k + size + 1] + d2); if (i > 0) v = min(v, d[k + size - 1] + d2) }
            d[k] = v
        }
        val sigmaCells = 0.10f / res
        ByteArray(size * size) { val x = d[it] / sigmaCells; (255 * kotlin.math.exp(-0.5f * x * x)).toInt().toByte() }
    }

    companion object {
        const val LMAX = 60
        const val OCC_T: Byte = 12
        const val FREE_T: Byte = -6

        fun load(dir: File): MapBuilder {
            val meta = JSONObject(File(dir, "map.json").readText())
            val m = MapBuilder(meta.getDouble("res").toFloat(), meta.getInt("size"))
            m.originX = meta.getDouble("origin_x").toFloat(); m.originZ = meta.getDouble("origin_z").toFloat()
            if (!meta.isNull("floor_y")) m.floorY = meta.getDouble("floor_y").toFloat()
            InflaterInputStream(File(dir, "occupancy.bin").inputStream(), Inflater(true)).use { inp ->
                var o = 0
                while (o < m.occ.size) { val n = inp.read(m.occ, o, m.occ.size - o); if (n < 0) break; o += n }
            }
            for (k in m.occ.indices) if (m.occ[k].toInt() != 0) { m.seen[k] = true; m.knownCells++ }
            m.initialized = true
            return m
        }

        /**
         * Correlative scan matching of [live] against [saved]: brute-force over rotation and translation
         * (coarse 4 deg / 10 cm over +-[searchM], then fine 1 deg / 2.5 cm). Returns live -> saved transform.
         */
        fun align(live: MapBuilder, saved: MapBuilder, searchM: Float = 3f, progress: (String) -> Unit = {}): Align? {
            val pts = live.occupiedPoints(800)
            if (pts.size < 40) return null
            val field = saved.likelihood()
            fun score(p: List<FloatArray>, th: Float, tx: Float, tz: Float): Float {
                val c = cos(th); val s = sin(th)
                var sum = 0
                for (q in p) {
                    val x = c * q[0] - s * q[1] + tx; val z = s * q[0] + c * q[1] + tz
                    val i = saved.cellI(x); val j = saved.cellJ(z)
                    if (saved.inside(i, j)) sum += field[j * saved.size + i].toInt() and 0xff
                }
                return sum / (255f * p.size)
            }
            val coarse = if (pts.size > 300) pts.filterIndexed { k, _ -> k % (pts.size / 300 + 1) == 0 } else pts
            val lc = floatArrayOf(pts.sumOf { it[0].toDouble() }.toFloat() / pts.size, pts.sumOf { it[1].toDouble() }.toFloat() / pts.size)
            val sp = saved.occupiedPoints(2000)
            if (sp.isEmpty()) return null
            val sc = floatArrayOf(sp.sumOf { it[0].toDouble() }.toFloat() / sp.size, sp.sumOf { it[1].toDouble() }.toFloat() / sp.size)
            var best = Align(0f, 0f, 0f, -1f)
            val steps = (searchM / 0.1f).toInt()
            for (a in 0 until 90) {
                val th = Math.toRadians(a * 4.0).toFloat()
                val c = cos(th); val s = sin(th)
                val t0x = sc[0] - (c * lc[0] - s * lc[1]); val t0z = sc[1] - (s * lc[0] + c * lc[1])
                for (dx in -steps..steps) for (dz in -steps..steps) {
                    val tx = t0x + dx * 0.1f; val tz = t0z + dz * 0.1f
                    val v = score(coarse, th, tx, tz)
                    if (v > best.score) best = Align(th, tx, tz, v)
                }
                if (a % 15 == 0) progress("aligning ${a * 4}/360 deg, best ${"%.2f".format(best.score)}")
            }
            var fine = best
            for (da in -4..4) for (dx in -8..8) for (dz in -8..8) {
                val th = best.theta + Math.toRadians(da.toDouble()).toFloat()
                val tx = best.tx + dx * 0.025f; val tz = best.tz + dz * 0.025f
                val v = score(pts, th, tx, tz)
                if (v > fine.score) fine = Align(th, tx, tz, v)
            }
            return fine
        }

        fun dist(a: FloatArray, b: FloatArray) = sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))
    }
}
