package com.real2sim.capture

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * JVM tests of the navigation maths with synthetic depth: coordinate conventions (depth back-projection,
 * bearings, left/right), occupancy, the virtual lidar, A* around an obstacle, pure pursuit and map
 * alignment.
 */
class NavMathTest {
    private val w = 160; private val h = 120
    private val fx = 120f; private val fy = 120f; private val cx = 80f; private val cy = 60f

    /** Camera at (px, camY, pz) looking along world direction yaw (rad, atan2(z, x) of forward), level. */
    private fun pose(px: Float, camY: Float, pz: Float, yaw: Float): FloatArray {
        // camera axes in world: forward f = (cos yaw, 0, sin yaw) = -Z_cam; up = +Y; right = f x up
        val fxw = cos(yaw); val fzw = sin(yaw)
        val rx = -fzw; val rz = fxw          // right = f x (0,1,0) = (-fz, 0, fx)... check: (fx,0,fz) x (0,1,0) = (0*0-fz*1, fz*0-fx*0, fx*1-0) = (-fz, 0, fx)
        // column-major: col0 = X_cam (right), col1 = Y_cam (up), col2 = Z_cam (-forward), col3 = t
        return floatArrayOf(rx, 0f, rz, 0f, 0f, 1f, 0f, 0f, -fxw, 0f, -fzw, 0f, px, camY, pz, 1f)
    }

    /** Depth map of a vertical wall at world x = wallX (camera looking +x), floor at y = 0. */
    private fun depthOfScene(m: FloatArray, wallX: Float, camY: Float, box: FloatArray? = null): ByteBuffer {
        val b = ByteBuffer.allocate(w * h * 2).order(ByteOrder.LITTLE_ENDIAN)
        for (v in 0 until h) for (u in 0 until w) {
            // ray in camera frame (z = -1 forward)
            val dxc = (u - cx) / fx; val dyc = -(v - cy) / fy
            // to world
            val rx = m[0] * dxc + m[4] * dyc + m[8] * -1f
            val ry = m[1] * dxc + m[5] * dyc + m[9] * -1f
            val rz = m[2] * dxc + m[6] * dyc + m[10] * -1f
            var t = Float.POSITIVE_INFINITY
            if (rx > 1e-4f) t = (wallX - m[12]) / rx
            if (ry < -1e-4f) t = minOf(t, (0f - camY) / ry)                      // floor
            if (box != null) {                                                     // axis-aligned box x0,z0,x1,z1 (height 0.5)
                for (tx in listOf((box[0] - m[12]) / rx, (box[2] - m[12]) / rx)) if (tx > 0 && tx.isFinite()) {
                    val y = camY + ry * tx; val z = m[14] + rz * tx
                    if (y in 0f..0.5f && z in box[1]..box[3]) t = minOf(t, tx)
                }
            }
            val depth = t            // ray has z_cam = -1 so t == depth along the optical axis
            b.putShort((v * w + u) * 2, if (depth.isFinite() && depth < 10f) (depth * 1000).toInt().toShort() else 0)
        }
        return b
    }

    @Test fun wallAheadIsOccupiedAndScannedInFront() {
        val map = MapBuilder()
        map.floorY = 0f
        val camY = 0.3f
        val m = pose(0f, camY, 0f, 0f)               // facing +x
        val heading = atan2(-m[10], -m[8])
        assertEquals(0f, heading, 1e-5f)
        map.integrateDepth(depthOfScene(m, 2f, camY), w, h, w * 2, fx, fy, cx, cy, m, heading, 1L)
        assertTrue("wall cell occupied", map.isOcc(map.cellI(2.02f), map.cellJ(0f)) || map.isOcc(map.cellI(1.98f), map.cellJ(0f)))
        assertTrue("space before wall free", map.isFree(map.cellI(1.0f), map.cellJ(0f)))
        val scan = FloatArray(map.bins)
        map.scan(0f, 0f, heading, 1L, scan)
        assertEquals("front distance", 2f, scan[0], 0.08f)
        assertTrue("nothing behind", !scan[36].isFinite())
    }

    @Test fun leftIsCounterClockwiseFromAbove() {
        // Facing +x, an obstacle at +z or -z? ARCore: y up, so looking down from +y with x right, z points down
        // on screen; the robot's left (CCW from above) is -z.
        val map = MapBuilder(); map.floorY = 0f
        val m = pose(0f, 0.3f, 0f, 0f)
        map.addPose(0f, 0.3f, 0f, 0L)
        // mark an obstacle at x=0, z=-1 (should be to the LEFT, bearing +90)
        val i = map.cellI(0f); val j = map.cellJ(-1f)
        repeat(10) { map.occ[j * map.size + i] = 40 }
        val scan = FloatArray(map.bins)
        map.scan(0f, 0f, atan2(-m[10], -m[8]), 0L, scan, maxAgeNs = 0)
        assertEquals("left (bearing +90)", 1f, scan[18], 0.08f)
        assertTrue("right empty", !scan[54].isFinite())
    }

    @Test fun planAroundObstacleAndFollow() {
        val map = MapBuilder(); map.floorY = 0f
        map.addPose(0f, 0.3f, 0f, 0L)
        // free square 4x4 m, wall segment across x=1 for z in [-1, 1]
        for (j in map.cellJ(-2f)..map.cellJ(2f)) for (i in map.cellI(-0.5f)..map.cellI(3f)) map.occ[j * map.size + i] = -30
        for (j in map.cellJ(-1f)..map.cellJ(1f)) for (i in map.cellI(1f)..map.cellI(1.1f)) map.occ[j * map.size + i] = 50
        val g = Planner.Grid(map.occ.copyOf(), map.size, map.res, map.originX, map.originZ)
        val path = Planner.plan(g, 0f, 0f, 2f, 0f, 0.2f)
        assertNotNull(path)
        path!!
        // path must go around the wall: some waypoint with |z| > 1
        assertTrue("detour", path.any { abs(it[1]) > 1.0f })
        assertTrue("length > straight", Planner.length(path) > 2.2f)
        // first command: target is off to one side -> turn in place or bear
        val c = Planner.follow(path, 0f, 0f, 0f, Float.POSITIVE_INFINITY, 0.3f, 1f)
        assertTrue(!c.arrived)
        val ahead = Planner.follow(listOf(floatArrayOf(0f, 0f), floatArrayOf(2f, 0f)), 0f, 0f, 0f, Float.POSITIVE_INFINITY, 0.3f, 1f)
        assertEquals(0f, ahead.bearing, 1e-3f); assertTrue(ahead.v > 0.1f)
        val leftGoal = Planner.follow(listOf(floatArrayOf(0f, 0f), floatArrayOf(0f, -2f)), 0f, 0f, 0f, Float.POSITIVE_INFINITY, 0.3f, 1f)
        assertTrue("goal at -z is to the left: positive bearing and CCW turn", leftGoal.bearing > 1.4f && leftGoal.w > 0f)
        val blocked = Planner.follow(listOf(floatArrayOf(0f, 0f), floatArrayOf(2f, 0f)), 0f, 0f, 0f, 0.1f, 0.3f, 1f)
        assertTrue(blocked.blocked); assertEquals(0f, blocked.v, 0f)
        val arrived = Planner.follow(listOf(floatArrayOf(0f, 0f), floatArrayOf(2f, 0f)), 1.95f, 0f, 0f, Float.POSITIVE_INFINITY, 0.3f, 1f)
        assertTrue(arrived.arrived)
    }

    /**
     * NavActivity's closed-loop move: the task stores the phone pose at the start and its progress must be
     * measured with the phone pose too. Measuring from the robot's turning centre (phone minus mountForward
     * along the heading) would begin at -mountForward and finish the move mountForward late (or, for a
     * short move, report it done before the robot moved).
     */
    @Test fun closedLoopMoveProgressUsesTheSamePointAsItsStart() {
        fun travelled(px: Float, pz: Float, sx: Float, sz: Float, h: Float) = NavMath.travelledAlong(px, pz, sx, sz, h)
        val h = 0.7f; val mountForward = 0.15f; val dist = 0.10f
        val sx = 1.0f; val sz = -2.0f                         // phone pose at the start (what Task.Move records)
        // phone after driving `dist` along the heading
        val px = sx + dist * cos(h); val pz = sz + dist * sin(h)
        assertEquals(dist, travelled(px, pz, sx, sz, h), 1e-6f)
        assertEquals(0f, travelled(sx, sz, sx, sz, h), 0f)   // nothing travelled yet: the move must not finish on its first frame
        // robot centre against the phone start: off by mountForward from the first frame on
        val rx0 = sx - mountForward * cos(h); val rz0 = sz - mountForward * sin(h)
        assertEquals(-mountForward, travelled(rx0, rz0, sx, sz, h), 1e-6f)
        assertTrue("10 cm move would look done (|travelled| >= dist) before the robot moved", abs(travelled(rx0, rz0, sx, sz, h)) >= dist)
        val rx1 = px - mountForward * cos(h); val rz1 = pz - mountForward * sin(h)
        assertEquals(dist - mountForward, travelled(rx1, rz1, sx, sz, h), 1e-6f)
        assertEquals(0f, NavMath.moveRemaining(dist, px, pz, sx, sz, h), 1e-6f)   // arrived exactly when the phone moved dist
        assertEquals(dist, NavMath.moveRemaining(dist, sx, sz, sx, sz, h), 1e-6f)  // nothing done at the start
    }

    @Test fun unreachableGoalReturnsNull() {
        val map = MapBuilder(); map.addPose(0f, 0f, 0f, 0L)
        // closed ring of walls around the robot
        for (deg in 0 until 360) {
            val i = map.cellI(cos(Math.toRadians(deg.toDouble())).toFloat()); val j = map.cellJ(sin(Math.toRadians(deg.toDouble())).toFloat())
            for (di in -2..2) for (dj in -2..2) map.occ[(j + dj) * map.size + i + di] = 50
        }
        val g = Planner.Grid(map.occ.copyOf(), map.size, map.res, map.originX, map.originZ)
        assertNull(Planner.plan(g, 0f, 0f, 3f, 0f, 0.1f, unknownCost = 2))
    }

    @Test fun alignRecoversRotationAndShift() {
        val saved = MapBuilder(); saved.addPose(0f, 0f, 0f, 0L)
        val live = MapBuilder(); live.addPose(0f, 0f, 0f, 0L)
        // an L-shaped room with a pillar, in the saved frame
        val walls = ArrayList<FloatArray>()
        var s = -2f; while (s <= 3f) { walls.add(floatArrayOf(s, -2f)); walls.add(floatArrayOf(-2f, s * 0.8f)); s += 0.05f }
        s = 0f; while (s <= 1f) { walls.add(floatArrayOf(1f + s * 0.3f, 1f)); walls.add(floatArrayOf(1f, 1f + s * 0.4f)); s += 0.05f }
        for (p in walls) { val i = saved.cellI(p[0]); val j = saved.cellJ(p[1]); saved.occ[j * saved.size + i] = 50 }
        // live sees the same walls, rotated by 30 deg and shifted: live p = R^T (q - t)
        val th = Math.toRadians(30.0).toFloat(); val tx = 0.7f; val tz = -0.4f
        for (q in walls) {
            val qx = q[0] - tx; val qz = q[1] - tz
            val px = cos(th) * qx + sin(th) * qz; val pz = -sin(th) * qx + cos(th) * qz
            val i = live.cellI(px); val j = live.cellJ(pz); live.occ[j * live.size + i] = 50
        }
        val a = MapBuilder.align(live, saved)
        assertNotNull(a); a!!
        var dth = Math.toDegrees((a.theta - th).toDouble()); while (dth > 180) dth -= 360.0; while (dth < -180) dth += 360.0
        assertEquals(0.0, dth, 2.5)
        assertEquals(tx, a.tx, 0.1f); assertEquals(tz, a.tz, 0.1f)
        assertTrue(a.score > 0.6f)
        // merging puts saved walls where live walls are
        val fresh = MapBuilder(); fresh.addPose(0f, 0f, 0f, 0L)
        fresh.mergeFrom(saved, a)
        val q = walls[10]; val qx = q[0] - a.tx; val qz = q[1] - a.tz
        val px = cos(a.theta) * qx + sin(a.theta) * qz; val pz = -sin(a.theta) * qx + cos(a.theta) * qz
        assertTrue(hypot(px, pz) > 0f)
        var near = false
        for (dj in -2..2) for (di in -2..2) if (fresh.isOcc(fresh.cellI(px) + di, fresh.cellJ(pz) + dj)) near = true
        assertTrue("merged wall lands in live frame", near)
    }
}
