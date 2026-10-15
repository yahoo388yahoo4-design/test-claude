package com.real2sim.capture

import org.json.JSONArray
import org.json.JSONObject
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Room map (HOME.md "room_map.json"): floor outline, walls and detected objects of one room in a floor-plan
 * frame (x right, y "up the map", z up from the floor, metres, origin at the room's min corner), built from
 * ARCore planes + the object detector's 3D boxes. Pure Kotlin (org.json only) so the geometry is unit tested.
 *
 * World = ARCore world of the session: y up, right handed, metres.
 */
object RoomCategories {
    /** Detector (COCO) label -> room-map category; labels not listed here are ignored. */
    val FROM_COCO = mapOf(
        "tv" to "tv", "toilet" to "toilet", "chair" to "chair", "couch" to "sofa", "bed" to "bed",
        "dining table" to "table", "refrigerator" to "refrigerator", "oven" to "oven", "microwave" to "microwave",
        "sink" to "sink", "potted plant" to "plant", "laptop" to "laptop", "toaster" to "toaster", "bench" to "bench",
        "clock" to "clock", "vase" to "vase", "book" to "book", "bottle" to "bottle", "cup" to "cup",
        "keyboard" to "keyboard", "mouse" to "mouse", "cell phone" to "phone", "remote" to "remote",
        "teddy bear" to "toy", "hair drier" to "hair_dryer", "suitcase" to "suitcase", "backpack" to "bag",
        "umbrella" to "umbrella", "bicycle" to "bicycle",
    )

    /** Small things are detected but not kept in the map (they move around). */
    val MAPPED = setOf("tv", "toilet", "chair", "sofa", "bed", "table", "refrigerator", "oven", "microwave", "sink",
        "plant", "laptop", "toaster", "bench", "clock", "bicycle")

    /** Typical depth (front to back, metres) when the camera only sees the front of the object. */
    fun depthPrior(cat: String, width: Float): Float = when (cat) {
        "tv", "clock", "laptop" -> 0.12f
        "bed" -> max(1.9f, width)
        "sofa" -> 0.9f
        "table", "bench" -> max(0.7f, width * 0.6f)
        "toilet" -> 0.7f
        "chair" -> 0.55f
        "refrigerator" -> 0.7f
        "oven", "microwave", "toaster" -> 0.5f
        "sink" -> 0.5f
        "plant" -> max(0.3f, width)
        "bicycle" -> 0.6f
        else -> max(0.2f, width * 0.5f)
    }

    /** Stands on the floor: its box is extended down to the floor. */
    fun onFloor(cat: String) = cat in setOf("chair", "sofa", "bed", "table", "refrigerator", "toilet", "bench", "plant", "bicycle", "oven")
}

/** One 3D object observation in world coordinates (axis = horizontal unit vector along the object's width). */
class ObjectObservation(
    val category: String, val score: Float,
    val cx: Float, val cy: Float, val cz: Float,
    val width: Float, val height: Float, val depth: Float,
    val axisX: Float, val axisZ: Float,
)

/** Merges per-frame observations into persistent objects (same category, overlapping footprint). */
class ObjectFusion(private val minHits: Int = 3) {
    class Track(var category: String) {
        var cx = 0f; var cy = 0f; var cz = 0f
        var width = 0f; var height = 0f; var depth = 0f
        var ax = 1f; var az = 0f            // doubled-angle accumulator of the width axis (axis is sign-free)
        var weight = 0f; var hits = 0; var bestScore = 0f
        val yaw get() = 0.5f * atan2(az, ax)
        fun add(o: ObjectObservation) {
            val w = o.score; val tw = weight + w
            cx = (cx * weight + o.cx * w) / tw; cy = (cy * weight + o.cy * w) / tw; cz = (cz * weight + o.cz * w) / tw
            width = (width * weight + o.width * w) / tw; height = (height * weight + o.height * w) / tw
            depth = (depth * weight + o.depth * w) / tw
            val a = atan2(o.axisZ, o.axisX) * 2
            if (hits == 0) { ax = 0f; az = 0f }
            ax += cos(a) * w; az += sin(a) * w
            weight = tw; hits++; bestScore = max(bestScore, o.score)
        }
    }

    val tracks = ArrayList<Track>()

    fun add(o: ObjectObservation) = synchronized(this) {
        var best: Track? = null; var bestD = Float.MAX_VALUE
        for (t in tracks) {
            if (t.category != o.category) continue
            val d = hypot(t.cx - o.cx, t.cz - o.cz)
            val gate = max(0.5f, 0.6f * max(max(t.width, t.depth), max(o.width, o.depth)))
            if (d < gate && d < bestD && abs(t.cy - o.cy) < max(0.6f, t.height)) { best = t; bestD = d }
        }
        (best ?: Track(o.category).also { tracks.add(it) }).add(o)
    }

    /** Tracks seen often enough, overlapping duplicates of the same category merged. */
    fun confirmed(): List<Track> = synchronized(this) {
        val keep = tracks.filter { it.hits >= minHits || (it.hits >= 2 && it.bestScore >= 0.7f) }
            .sortedByDescending { it.weight }.toMutableList()
        val out = ArrayList<Track>()
        for (t in keep) {
            val dup = out.firstOrNull { it.category == t.category && hypot(it.cx - t.cx, it.cz - t.cz) < 0.5f * max(max(it.width, it.depth), 0.6f) }
            if (dup == null) out.add(t) else { dup.hits += t.hits }
        }
        out
    }

    val count get() = synchronized(this) { tracks.count { it.hits >= minHits } }
}

/** An ARCore plane reduced to what the room geometry needs (world coordinates). */
class PlaneInfo(val vertical: Boolean, val upward: Boolean, val center: FloatArray, val normal: FloatArray, val polygon: List<FloatArray>)

object RoomGeometry {
    class Wall(val ax: Float, val ay: Float, val bx: Float, val by: Float, val height: Float) {
        val length get() = hypot(bx - ax, by - ay)
    }

    class Plan(
        val theta: Float,              // rotation about world up applied before the flip (radians)
        val ox: Float, val oy: Float,  // origin shift (plan coordinates of the world origin are -ox, -oy)
        val floorY: Float,
        val floor: List<FloatArray>,   // plan [x, y]
        val walls: List<Wall>,
        val height: Float,
    ) {
        /** World (x, y, z) -> plan (x, y, z). */
        fun toPlan(x: Float, y: Float, z: Float): FloatArray {
            val c = cos(theta); val s = sin(theta)
            val rx = c * x + s * z; val rz = -s * x + c * z
            return floatArrayOf(rx - ox, -rz - oy, y - floorY)
        }
        /** World horizontal direction (dx, dz) -> plan yaw (radians, from plan +x towards +y). */
        fun yawToPlan(dx: Float, dz: Float): Float {
            val c = cos(theta); val s = sin(theta)
            val rx = c * dx + s * dz; val rz = -s * dx + c * dz
            return atan2(-rz, rx)
        }
        /** Row-major 4x4 world -> plan. */
        fun worldToPlan(): DoubleArray {
            val c = cos(theta).toDouble(); val s = sin(theta).toDouble()
            return doubleArrayOf(c, 0.0, s, -ox.toDouble(), s, 0.0, -c, -oy.toDouble(), 0.0, 1.0, 0.0, -floorY.toDouble(), 0.0, 0.0, 0.0, 1.0)
        }
    }

    /**
     * Floor outline, walls and orientation from ARCore planes, the walked trajectory and (optionally) occupied
     * map cells. Walls = vertical planes at least 0.4 m wide; the room is turned so the dominant wall direction
     * lies along x. The outline is the convex hull of floor planes, wall ends, trajectory and occupied cells,
     * replaced by its bounding rectangle when that is nearly the same area.
     */
    fun build(planes: List<PlaneInfo>, floorY0: Float, traj: List<FloatArray>, occupied: List<FloatArray> = emptyList()): Plan {
        val floorY = if (!floorY0.isNaN()) floorY0 else planes.filter { !it.vertical && it.upward }.minOfOrNull { it.center[1] }
            ?: traj.firstOrNull()?.let { it[1] - 1.4f } ?: 0f
        // walls in world xz: segment along the plane's horizontal in-plane direction
        data class WSeg(val ax: Float, val az: Float, val bx: Float, val bz: Float, val top: Float)
        val segs = ArrayList<WSeg>()
        for (p in planes) {
            if (!p.vertical || p.polygon.size < 3) continue
            var hx = -p.normal[2]; var hz = p.normal[0]
            val hn = hypot(hx, hz); if (hn < 1e-3f) continue
            hx /= hn; hz /= hn
            var lo = Float.MAX_VALUE; var hi = -Float.MAX_VALUE; var top = -Float.MAX_VALUE
            for (q in p.polygon) {
                val t = (q[0] - p.center[0]) * hx + (q[2] - p.center[2]) * hz
                lo = min(lo, t); hi = max(hi, t); top = max(top, q[1])
            }
            if (hi - lo < 0.4f) continue
            segs.add(WSeg(p.center[0] + hx * lo, p.center[2] + hz * lo, p.center[0] + hx * hi, p.center[2] + hz * hi, top))
        }
        // dominant direction (mod 90 deg), length weighted
        var sc = 0.0; var ss = 0.0
        for (w in segs) {
            val a = atan2((w.bz - w.az).toDouble(), (w.bx - w.ax).toDouble()) * 4
            val l = hypot(w.bx - w.ax, w.bz - w.az).toDouble()
            sc += cos(a) * l; ss += sin(a) * l
        }
        val theta = if (segs.isEmpty()) 0f else (atan2(ss, sc) / 4).toFloat()
        val c = cos(theta); val s = sin(theta)
        fun rot(x: Float, z: Float) = floatArrayOf(c * x + s * z, -(-s * x + c * z))   // plan before shift

        val pts = ArrayList<FloatArray>()
        for (p in planes) if (!p.vertical && p.upward && abs(p.center[1] - floorY) < 0.25f) p.polygon.forEach { pts.add(rot(it[0], it[2])) }
        for (w in segs) { pts.add(rot(w.ax, w.az)); pts.add(rot(w.bx, w.bz)) }
        traj.forEach { pts.add(rot(it[0], it[2])) }
        occupied.forEach { pts.add(rot(it[0], it[1])) }
        if (pts.isEmpty()) pts.add(floatArrayOf(0f, 0f))
        var hull = convexHull(pts)
        val minX = pts.minOf { it[0] }; val maxX = pts.maxOf { it[0] }
        val minY = pts.minOf { it[1] }; val maxY = pts.maxOf { it[1] }
        val rectA = (maxX - minX) * (maxY - minY)
        if (hull.size < 3 || rectA <= 0f || polygonArea(hull) / rectA > 0.85f)
            hull = listOf(floatArrayOf(minX, minY), floatArrayOf(maxX, minY), floatArrayOf(maxX, maxY), floatArrayOf(minX, maxY))
        val floor = hull.map { floatArrayOf(it[0] - minX, it[1] - minY) }

        // walls in plan coordinates, snapped to the axes when within 10 deg, collinear pieces merged
        val height = segs.maxOfOrNull { it.top - floorY }?.takeIf { it > 1.8f }?.coerceAtMost(4f) ?: 2.5f
        val raw = segs.map { w ->
            val a = rot(w.ax, w.az); val b = rot(w.bx, w.bz)
            snap(Wall(a[0] - minX, a[1] - minY, b[0] - minX, b[1] - minY, height))
        }
        return Plan(theta, minX, minY, floorY, floor, mergeWalls(raw), height)
    }

    fun snap(w: Wall): Wall {
        val ang = atan2(w.by - w.ay, w.bx - w.ax)
        val q = (ang / (PI / 2).toFloat()).roundToInt() * (PI / 2).toFloat()
        if (abs(ang - q) > 10f * PI.toFloat() / 180f) return w
        val mx = (w.ax + w.bx) / 2; val my = (w.ay + w.by) / 2
        return if (abs(cos(q)) > 0.5f) Wall(min(w.ax, w.bx), my, max(w.ax, w.bx), my, w.height)
        else Wall(mx, min(w.ay, w.by), mx, max(w.ay, w.by), w.height)
    }

    /** Axis-aligned walls on the same line (within 0.15 m) that overlap or touch are merged into one. */
    fun mergeWalls(walls: List<Wall>): List<Wall> {
        val out = ArrayList<Wall>()
        val used = BooleanArray(walls.size)
        for (i in walls.indices) {
            if (used[i]) continue
            var w = walls[i]
            val horiz = w.ay == w.by; val vert = w.ax == w.bx
            if (horiz || vert) {
                var changed = true
                while (changed) {
                    changed = false
                    for (j in walls.indices) {
                        if (used[j] || j == i) continue
                        val o = walls[j]
                        if (horiz && o.ay == o.by && abs(o.ay - w.ay) < 0.15f && o.ax <= w.bx + 0.2f && o.bx >= w.ax - 0.2f) {
                            val y = (w.ay * w.length + o.ay * o.length) / (w.length + o.length)
                            w = Wall(min(w.ax, o.ax), y, max(w.bx, o.bx), y, max(w.height, o.height)); used[j] = true; changed = true
                        } else if (vert && o.ax == o.bx && abs(o.ax - w.ax) < 0.15f && o.ay <= w.by + 0.2f && o.by >= w.ay - 0.2f) {
                            val x = (w.ax * w.length + o.ax * o.length) / (w.length + o.length)
                            w = Wall(x, min(w.ay, o.ay), x, max(w.by, o.by), max(w.height, o.height)); used[j] = true; changed = true
                        }
                    }
                }
            }
            used[i] = true
            out.add(w)
        }
        return out
    }

    fun convexHull(points: List<FloatArray>): List<FloatArray> {
        val p = points.distinctBy { Pair((it[0] * 100).roundToInt(), (it[1] * 100).roundToInt()) }.sortedWith(compareBy({ it[0] }, { it[1] }))
        if (p.size < 3) return p
        fun cross(o: FloatArray, a: FloatArray, b: FloatArray) = (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        val lower = ArrayList<FloatArray>()
        for (q in p) { while (lower.size >= 2 && cross(lower[lower.size - 2], lower.last(), q) <= 0) lower.removeAt(lower.size - 1); lower.add(q) }
        val upper = ArrayList<FloatArray>()
        for (q in p.asReversed()) { while (upper.size >= 2 && cross(upper[upper.size - 2], upper.last(), q) <= 0) upper.removeAt(upper.size - 1); upper.add(q) }
        lower.removeAt(lower.size - 1); upper.removeAt(upper.size - 1)
        return lower + upper
    }

    fun polygonArea(p: List<FloatArray>): Float {
        var a = 0f
        for (i in p.indices) { val q = p[i]; val r = p[(i + 1) % p.size]; a += q[0] * r[1] - r[0] * q[1] }
        return abs(a) / 2
    }
}

/** room_map.json writer (HOME.md). */
object RoomMapJson {
    fun slug(name: String): String = name.lowercase().replace(Regex("[^a-z0-9]+"), "_").trim('_').ifEmpty { "room" }

    fun build(name: String, session: String, device: String, plan: RoomGeometry.Plan, objects: List<ObjectFusion.Track>,
              meshes: Map<Int, String> = emptyMap(), nowUnix: Double = System.currentTimeMillis() / 1000.0): JSONObject {
        fun arr(vararg v: Float) = JSONArray(v.map { r(it) })
        val objs = JSONArray()
        val counts = HashMap<String, Int>()
        objects.forEachIndexed { i, t ->
            val n = (counts[t.category] ?: 0) + 1; counts[t.category] = n
            val c = plan.toPlan(t.cx, t.cy, t.cz)
            var yaw = plan.yawToPlan(cos(t.yaw), sin(t.yaw))
            // furniture stands along the walls: snap to the room axes when close
            val q = (yaw / (PI / 2).toFloat()).roundToInt() * (PI / 2).toFloat()
            if (abs(yaw - q) < 20f * PI.toFloat() / 180f) yaw = q
            yaw = normYaw(yaw)
            objs.put(JSONObject().apply {
                put("id", "${t.category}_$n"); put("category", t.category)
                put("label", t.category.replace('_', ' ').replaceFirstChar { it.uppercase() } + if (n > 1) " $n" else "")
                put("confidence", r(min(1f, t.bestScore)))
                put("center", arr(c[0], c[1], max(c[2], t.height / 2)))
                put("size", arr(t.width, t.depth, t.height))
                put("yaw", r(yaw)); put("observations", t.hits)
                meshes[i]?.let { put("mesh", it) }
            })
        }
        val maxX = plan.floor.maxOfOrNull { it[0] } ?: 0f; val maxY = plan.floor.maxOfOrNull { it[1] } ?: 0f
        return JSONObject().apply {
            put("format", "r2s-roommap"); put("version", 1)
            put("id", slug(name)); put("name", name)
            put("source", JSONObject().put("platform", "android").put("session", session).put("device", device)
                .put("method", "ARCore planes + EfficientDet-Lite (COCO) boxes lifted with ARCore depth"))
            put("created_unix", nowUnix)
            put("units", "m")
            put("frame", "floor plan: x right, y up the map, z up from the floor; origin = room min corner")
            put("world_to_room", JSONArray(plan.worldToPlan().map { (it * 1e4).roundToInt() / 1e4 }))
            put("bounds", arr(0f, 0f, maxX, maxY))
            put("height", r(plan.height))
            put("floor", JSONArray(plan.floor.map { arr(it[0], it[1]) }))
            put("walls", JSONArray(plan.walls.map { w -> JSONObject().put("a", arr(w.ax, w.ay)).put("b", arr(w.bx, w.by)).put("height", r(w.height)) }))
            put("doors", JSONArray()); put("windows", JSONArray()); put("openings", JSONArray())
            put("objects", objs)
        }
    }

    fun normYaw(a: Float): Float { var y = a; while (y > PI) y -= (2 * PI).toFloat(); while (y <= -PI) y += (2 * PI).toFloat(); return y }
    private fun r(v: Float): Double = (v * 1000).roundToInt() / 1000.0
}

/** 3D lifting of a 2D detection (pure maths; ARCore camera: x right, y up, z backwards; image v down). */
object DetectionLift {
    /**
     * [box] = left, top, right, bottom in pixels of the image with intrinsics [k] (fx, fy, cx, cy); [frontM] =
     * distance along the optical axis of the object's front; [pose] column-major camera-to-world. Returns null
     * when the box is degenerate.
     */
    fun lift(category: String, score: Float, box: FloatArray, k: FloatArray, frontM: Float, pose: FloatArray, floorY: Float): ObjectObservation? {
        if (frontM <= 0.1f || frontM > 10f) return null
        fun world(u: Float, v: Float): FloatArray {
            val x = (u - k[2]) / k[0] * frontM; val y = -(v - k[3]) / k[1] * frontM; val z = -frontM
            return floatArrayOf(pose[0] * x + pose[4] * y + pose[8] * z + pose[12], pose[1] * x + pose[5] * y + pose[9] * z + pose[13],
                pose[2] * x + pose[6] * y + pose[10] * z + pose[14])
        }
        val corners = listOf(world(box[0], box[1]), world(box[2], box[1]), world(box[2], box[3]), world(box[0], box[3]))
        // forward (camera -z) projected on the floor plane
        var fx = -pose[8]; var fz = -pose[10]
        val fn = hypot(fx, fz); if (fn < 1e-3f) return null
        fx /= fn; fz /= fn
        val rx = -fz; val rz = fx     // horizontal right of the viewing direction
        var lo = Float.MAX_VALUE; var hi = -Float.MAX_VALUE
        var ymin = Float.MAX_VALUE; var ymax = -Float.MAX_VALUE
        var mx = 0f; var mz = 0f
        for (p in corners) {
            val t = p[0] * rx + p[2] * rz
            lo = min(lo, t); hi = max(hi, t); ymin = min(ymin, p[1]); ymax = max(ymax, p[1]); mx += p[0] / 4; mz += p[2] / 4
        }
        val width = hi - lo
        if (width < 0.05f || ymax - ymin < 0.05f) return null
        if (RoomCategories.onFloor(category) && !floorY.isNaN() && ymin > floorY) ymin = floorY
        val depth = RoomCategories.depthPrior(category, width)
        return ObjectObservation(category, score, mx + fx * depth / 2, (ymin + ymax) / 2, mz + fz * depth / 2,
            width, ymax - ymin, depth, rx, rz)
    }

    /** Distance of the object's front: [pct] percentile of valid depths (mm) in the central part of the box. */
    fun frontDepth(depth: ShortArray, dw: Int, dh: Int, box: FloatArray, kImg: FloatArray, kDepth: FloatArray, pct: Float = 0.3f): Float {
        val shrink = 0.2f
        val bw = box[2] - box[0]; val bh = box[3] - box[1]
        val u0 = box[0] + bw * shrink; val u1 = box[2] - bw * shrink
        val v0 = box[1] + bh * shrink; val v1 = box[3] - bh * shrink
        fun du(u: Float) = ((u - kImg[2]) / kImg[0] * kDepth[0] + kDepth[2])
        fun dv(v: Float) = ((v - kImg[3]) / kImg[1] * kDepth[1] + kDepth[3])
        val i0 = floor(du(u0)).toInt().coerceIn(0, dw - 1); val i1 = floor(du(u1)).toInt().coerceIn(0, dw - 1)
        val j0 = floor(dv(v0)).toInt().coerceIn(0, dh - 1); val j1 = floor(dv(v1)).toInt().coerceIn(0, dh - 1)
        val vals = ArrayList<Int>()
        val step = max(1, ((i1 - i0 + 1) * (j1 - j0 + 1) / 400.0).let { sqrt(it).toInt() })
        var j = j0
        while (j <= j1) {
            var i = i0
            while (i <= i1) { val d = depth[j * dw + i].toInt() and 0xffff; if (d in 100..8000) vals.add(d); i += step }
            j += step
        }
        if (vals.size < 5) return 0f
        vals.sort()
        return vals[(vals.size * pct).toInt().coerceAtMost(vals.size - 1)] / 1000f
    }
}
