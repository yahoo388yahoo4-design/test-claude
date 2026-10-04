package com.real2sim.capture

import java.util.PriorityQueue
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Grid path planning and path following for the navigation mode.
 *
 *  - [plan]: A* (8-connected) on a snapshot of the occupancy grid, with obstacles inflated by the robot
 *    radius plus a soft cost near obstacles; unknown cells are allowed but cost more. The result is
 *    shortened by line-of-sight smoothing and returned as world (x, z) waypoints.
 *  - [follow]: pure pursuit on that path, with speed scaled down by the nearest obstacle in front and a
 *    hard stop inside the stop distance.
 */
object Planner {

    class Grid(val occ: ByteArray, val size: Int, val res: Float, val originX: Float, val originZ: Float) {
        fun i(x: Float) = kotlin.math.floor((x - originX) / res).toInt()
        fun j(z: Float) = kotlin.math.floor((z - originZ) / res).toInt()
        fun x(i: Int) = originX + (i + 0.5f) * res
        fun z(j: Int) = originZ + (j + 0.5f) * res
    }

    /** Cost grid: -1 blocked, else 1 + extra cost. */
    private fun costs(g: Grid, radius: Float, unknownCost: Int): IntArray {
        val n = g.size
        val r = max(1, kotlin.math.ceil(radius / g.res).toInt())
        val soft = r + max(2, (0.15f / g.res).toInt())
        val cost = IntArray(n * n) { if (g.occ[it] < MapBuilder.FREE_T) 1 else if (g.occ[it] > MapBuilder.OCC_T) 1 else unknownCost }
        // distance (in cells, chessboard approx refined by two passes) to the nearest occupied cell
        val big = 1_000_000
        val d = IntArray(n * n) { if (g.occ[it] > MapBuilder.OCC_T) 0 else big }
        for (j in 0 until n) for (i in 0 until n) {
            val k = j * n + i; var v = d[k]
            if (i > 0) v = min(v, d[k - 1] + 10); if (j > 0) v = min(v, d[k - n] + 10)
            if (i > 0 && j > 0) v = min(v, d[k - n - 1] + 14); if (i < n - 1 && j > 0) v = min(v, d[k - n + 1] + 14)
            d[k] = v
        }
        for (j in n - 1 downTo 0) for (i in n - 1 downTo 0) {
            val k = j * n + i; var v = d[k]
            if (i < n - 1) v = min(v, d[k + 1] + 10); if (j < n - 1) v = min(v, d[k + n] + 10)
            if (i < n - 1 && j < n - 1) v = min(v, d[k + n + 1] + 14); if (i > 0 && j < n - 1) v = min(v, d[k + n - 1] + 14)
            d[k] = v
        }
        for (k in cost.indices) {
            val dc = d[k] / 10f
            if (dc <= r) cost[k] = -1
            else if (dc < soft) cost[k] += ((soft - dc) * 4).toInt()
        }
        return cost
    }

    /** Returns world waypoints [x, z] from start to goal, or null if unreachable. */
    fun plan(g: Grid, sx: Float, sz: Float, gx: Float, gz: Float, radius: Float, unknownCost: Int = 6): List<FloatArray>? {
        val n = g.size
        val cost = costs(g, radius, unknownCost)
        val si = g.i(sx); val sj = g.j(sz); var gi = g.i(gx); var gj = g.j(gz)
        if (si !in 0 until n || sj !in 0 until n || gi !in 0 until n || gj !in 0 until n) return null
        // the robot's own cell can look blocked (it is close to things): clear a small disc around it
        val rr = 3
        for (dj in -rr..rr) for (di in -rr..rr) { val i = si + di; val j = sj + dj
            if (i in 0 until n && j in 0 until n && cost[j * n + i] < 0) cost[j * n + i] = 20 }
        // goal inside an obstacle: move it to the nearest traversable cell
        if (cost[gj * n + gi] < 0) {
            var found = false
            loop@ for (rad in 1..40) for (dj in -rad..rad) for (di in -rad..rad) {
                if (max(abs(di), abs(dj)) != rad) continue
                val i = gi + di; val j = gj + dj
                if (i in 0 until n && j in 0 until n && cost[j * n + i] > 0) { gi = i; gj = j; found = true; break@loop }
            }
            if (!found) return null
        }
        val gScore = FloatArray(n * n) { Float.POSITIVE_INFINITY }
        val parent = IntArray(n * n) { -1 }
        val closed = BooleanArray(n * n)
        val open = PriorityQueue<Pair<Float, Int>>(1024, compareBy { it.first })
        val start = sj * n + si; val goal = gj * n + gi
        gScore[start] = 0f
        open.add(Pair(0f, start))
        val di = intArrayOf(1, -1, 0, 0, 1, 1, -1, -1); val dj = intArrayOf(0, 0, 1, -1, 1, -1, 1, -1)
        val dl = floatArrayOf(1f, 1f, 1f, 1f, 1.4142f, 1.4142f, 1.4142f, 1.4142f)
        var expanded = 0
        while (open.isNotEmpty()) {
            val (_, k) = open.poll()!!
            if (closed[k]) continue
            closed[k] = true
            if (k == goal) break
            if (++expanded > 400_000) return null
            val ci = k % n; val cj = k / n
            for (m in 0 until 8) {
                val ni = ci + di[m]; val nj = cj + dj[m]
                if (ni !in 0 until n || nj !in 0 until n) continue
                val nk = nj * n + ni
                val c = cost[nk]
                if (c < 0 || closed[nk]) continue
                val ng = gScore[k] + dl[m] * c
                if (ng < gScore[nk]) {
                    gScore[nk] = ng; parent[nk] = k
                    open.add(Pair(ng + hypot((ni - gi).toFloat(), (nj - gj).toFloat()), nk))
                }
            }
        }
        if (parent[goal] < 0 && goal != start) return null
        val cells = ArrayList<Int>()
        var k = goal
        while (k >= 0) { cells.add(k); if (k == start) break; k = parent[k] }
        cells.reverse()
        // line-of-sight smoothing over traversable cells
        val out = ArrayList<FloatArray>()
        var a = 0
        out.add(floatArrayOf(g.x(cells[0] % n), g.z(cells[0] / n)))
        while (a < cells.size - 1) {
            var b = cells.size - 1
            while (b > a + 1 && !lineFree(cost, n, cells[a], cells[b])) b--
            out.add(floatArrayOf(g.x(cells[b] % n), g.z(cells[b] / n)))
            a = b
        }
        return out
    }

    private fun lineFree(cost: IntArray, n: Int, a: Int, b: Int): Boolean {
        var x = a % n; var y = a / n; val x1 = b % n; val y1 = b / n
        val dx = abs(x1 - x); val dy = -abs(y1 - y); val sx = if (x < x1) 1 else -1; val sy = if (y < y1) 1 else -1
        var err = dx + dy
        while (true) {
            val c = cost[y * n + x]
            if (c < 0 || c > 8) return false
            if (x == x1 && y == y1) return true
            val e2 = 2 * err
            if (e2 >= dy) { err += dy; x += sx }
            if (e2 <= dx) { err += dx; y += sy }
        }
    }

    fun length(path: List<FloatArray>): Float {
        var s = 0f
        for (k in 1 until path.size) s += MapBuilder.dist(path[k - 1], path[k])
        return s
    }

    data class Cmd(
        val v: Float,            // m/s forward
        val w: Float,            // rad/s, CCW (left) positive
        val bearing: Float,      // rad to the lookahead point, CCW positive
        val target: FloatArray?, // lookahead point
        val remaining: Float,    // m along the path
        val blocked: Boolean,    // obstacle inside stop distance ahead
        val arrived: Boolean,
    )

    /**
     * Pure pursuit. [heading] = atan2(fz, fx) in grid coordinates; CCW (left) bearings are heading - angle
     * because grid z points "down" in the top view (see MapBuilder).
     */
    fun follow(path: List<FloatArray>, x: Float, z: Float, heading: Float, frontClear: Float,
               vMax: Float, wMax: Float, lookahead: Float = 0.4f, stopDist: Float = 0.25f, slowDist: Float = 0.8f,
               goalTol: Float = 0.15f): Cmd {
        val goal = path.last()
        val dGoal = hypot(goal[0] - x, goal[1] - z)
        if (dGoal < goalTol) return Cmd(0f, 0f, 0f, goal, 0f, false, true)
        // closest segment, then walk forward by the lookahead distance
        var bestK = 0; var bestD = Float.POSITIVE_INFINITY; var bestT = 0f
        for (k in 0 until path.size - 1) {
            val a = path[k]; val b = path[k + 1]
            val vx = b[0] - a[0]; val vz = b[1] - a[1]
            val l2 = vx * vx + vz * vz
            val t = if (l2 < 1e-9f) 0f else (((x - a[0]) * vx + (z - a[1]) * vz) / l2).coerceIn(0f, 1f)
            val d = hypot(a[0] + t * vx - x, a[1] + t * vz - z)
            if (d < bestD) { bestD = d; bestK = k; bestT = t }
        }
        var remaining = 0f
        var px = path[bestK][0] + bestT * (path[bestK + 1][0] - path[bestK][0])
        var pz = path[bestK][1] + bestT * (path[bestK + 1][1] - path[bestK][1])
        var need = lookahead
        var k = bestK
        var tx = px; var tz = pz
        var reached = false
        while (k < path.size - 1) {
            val b = path[k + 1]
            val seg = hypot(b[0] - px, b[1] - pz)
            if (!reached && seg >= need) { val f = need / seg; tx = px + (b[0] - px) * f; tz = pz + (b[1] - pz) * f; reached = true }
            if (!reached) need -= seg
            remaining += seg
            px = b[0]; pz = b[1]; k++
        }
        if (!reached) { tx = goal[0]; tz = goal[1] }
        var bearing = heading - atan2(tz - z, tx - x)
        while (bearing > Math.PI) bearing -= (2 * Math.PI).toFloat()
        while (bearing < -Math.PI) bearing += (2 * Math.PI).toFloat()
        val blocked = frontClear < stopDist
        var v: Float
        var w: Float
        if (abs(bearing) > Math.toRadians(40.0)) {
            v = 0f; w = (bearing * 1.5f).coerceIn(-wMax, wMax)          // rotate in place towards the path
        } else {
            val scale = ((frontClear - stopDist) / (slowDist - stopDist)).coerceIn(0f, 1f)
            v = vMax * scale * cos(bearing).coerceAtLeast(0f) * min(1f, dGoal / 0.5f + 0.3f)
            val l = hypot(tx - x, tz - z).coerceAtLeast(0.05f)
            w = (2 * max(v, 0.05f) * sin(bearing) / l).coerceIn(-wMax, wMax)
        }
        if (blocked) v = 0f
        return Cmd(v, w, bearing, floatArrayOf(tx, tz), remaining, blocked, false)
    }
}
