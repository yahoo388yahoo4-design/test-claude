package com.real2sim.capture

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class RoomMapTest {
    /** A 4 x 3 m room turned by [rot] about world up, floor at y = -1.2, walls as ARCore vertical planes. */
    private fun room(rot: Float): Pair<List<PlaneInfo>, List<FloatArray>> {
        val c = cos(rot); val s = sin(rot)
        fun w(x: Float, z: Float) = floatArrayOf(c * x - s * z, 0f, s * x + c * z)
        fun dir(x: Float, z: Float) = floatArrayOf(c * x - s * z, 0f, s * x + c * z)
        val planes = ArrayList<PlaneInfo>()
        // wall: centre (x, z) in room coordinates, normal, half length along the wall
        fun wall(x: Float, z: Float, nx: Float, nz: Float, half: Float) {
            val ctr = w(x, z).also { it[1] = 0f }
            val n = dir(nx, nz)
            val hx = -n[2]; val hz = n[0]
            val poly = listOf(-half to -1.2f, half to -1.2f, half to 1.3f, -half to 1.3f).map { (t, y) -> floatArrayOf(ctr[0] + hx * t, y, ctr[2] + hz * t) }
            planes.add(PlaneInfo(true, false, ctr, n, poly))
        }
        wall(0f, -1.5f, 0f, 1f, 1.8f); wall(0f, 1.5f, 0f, -1f, 2f)
        wall(-2f, 0f, 1f, 0f, 1.4f); wall(2f, 0f, -1f, 0f, 1.5f)
        val floor = listOf(-1.8f to -1.3f, 1.8f to -1.3f, 1.8f to 1.3f, -1.8f to 1.3f).map { (x, z) -> w(x, z).also { it[1] = -1.2f } }
        planes.add(PlaneInfo(false, true, floatArrayOf(0f, -1.2f, 0f), floatArrayOf(0f, 1f, 0f), floor))
        val traj = listOf(floatArrayOf(0f, 0f, 0f, 0f), w(1f, 0.5f).also { it[1] = 0f })
        return planes to traj
    }

    @Test fun roomOutlineAndWallsFromPlanes() {
        for (rot in listOf(0f, 0.5f, -1.1f)) {
            val (planes, traj) = room(rot)
            val plan = RoomGeometry.build(planes, -1.2f, traj)
            val xs = plan.floor.map { it[0] }; val ys = plan.floor.map { it[1] }
            val dims = listOf(xs.max() - xs.min(), ys.max() - ys.min()).sorted()
            assertEquals("rot $rot", 3f, dims[0], 0.05f)
            assertEquals("rot $rot", 4f, dims[1], 0.05f)
            assertEquals("rot $rot walls", 4, plan.walls.size)
            plan.walls.forEach { assertTrue("axis aligned", it.ax == it.bx || it.ay == it.by) }
            assertEquals(2.5f, plan.height, 0.01f)
            // the world origin (room centre) lands in the middle of the plan
            val o = plan.toPlan(0f, -1.2f, 0f)
            assertEquals(xs.max() / 2, o[0], 0.05f); assertEquals(ys.max() / 2, o[1], 0.05f); assertEquals(0f, o[2], 1e-4f)
        }
    }

    @Test fun collinearWallPiecesMerge() {
        val w = RoomGeometry.mergeWalls(listOf(
            RoomGeometry.Wall(0f, 0f, 1.5f, 0f, 2.4f), RoomGeometry.Wall(1.6f, 0.05f, 3f, 0.05f, 2.4f),
            RoomGeometry.Wall(0f, 0f, 0f, 2f, 2.4f)))
        assertEquals(2, w.size)
        assertEquals(3f, w[0].length, 0.01f)
    }

    @Test fun detectionLiftsToWorldBox() {
        // camera at (0, 1.5, 0) looking down -z (identity rotation), 640x480, f = 500
        val pose = floatArrayOf(1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 0f, 0f, 1f, 0f, 0f, 1.5f, 0f, 1f)
        val k = floatArrayOf(500f, 500f, 320f, 240f)
        // a 1 m wide, 0.6 m tall TV whose front is 2.5 m away, centred at height 1.5
        val box = floatArrayOf(320f - 100f, 240f - 60f, 320f + 100f, 240f + 60f)
        val o = DetectionLift.lift("tv", 0.9f, box, k, 2.5f, pose, 0f)
        assertNotNull(o); o!!
        assertEquals(1f, o.width, 0.01f); assertEquals(0.6f, o.height, 0.01f)
        assertEquals(0f, o.cx, 0.01f); assertEquals(1.5f, o.cy, 0.01f)
        assertEquals(-2.5f - 0.06f, o.cz, 0.01f)
        // a chair is extended down to the floor
        val ch = DetectionLift.lift("chair", 0.9f, floatArrayOf(300f, 300f, 340f, 400f), k, 2f, pose, 0f)!!
        assertEquals(0f, ch.cy - ch.height / 2, 1e-3f)
    }

    @Test fun frontDepthUsesNearPercentile() {
        val dw = 160; val dh = 120
        val d = ShortArray(dw * dh) { 3000 }
        for (j in 40 until 80) for (i in 60 until 100) d[j * dw + i] = 2000
        val kImg = floatArrayOf(500f, 500f, 320f, 240f); val kD = floatArrayOf(125f, 125f, 80f, 60f)
        val f = DetectionLift.frontDepth(d, dw, dh, floatArrayOf(240f, 160f, 400f, 320f), kImg, kD)
        assertEquals(2f, f, 0.01f)
    }

    @Test fun fusionMergesRepeatedSightingsAndDropsSingletons() {
        val f = ObjectFusion(minHits = 3)
        repeat(5) { i -> f.add(ObjectObservation("sofa", 0.8f, 1f + i * 0.05f, 0.4f, 2f, 2f, 0.8f, 0.9f, 1f, 0f)) }
        f.add(ObjectObservation("sofa", 0.6f, 4f, 0.4f, 2f, 2f, 0.8f, 0.9f, 1f, 0f))      // seen once
        f.add(ObjectObservation("chair", 0.9f, 1f, 0.4f, 2f, 0.5f, 0.9f, 0.5f, 0f, 1f))
        val c = f.confirmed()
        assertEquals(1, c.size)
        assertEquals("sofa", c[0].category); assertEquals(5, c[0].hits); assertEquals(1.1f, c[0].cx, 0.01f)
        assertEquals(0f, c[0].yaw, 1e-3f)
    }

    @Test fun roomMapJson() {
        val (planes, traj) = room(0.3f)
        val plan = RoomGeometry.build(planes, -1.2f, traj)
        val f = ObjectFusion(minHits = 1)
        f.add(ObjectObservation("tv", 0.9f, 0f, 0f, -1.4f, 1.2f, 0.7f, 0.1f, cos(0.3f + 0.05f), sin(0.3f + 0.05f)))
        val j = RoomMapJson.build("Living Room", "s1", "phone", plan, f.confirmed(), nowUnix = 1.0)
        assertEquals("r2s-roommap", j.getString("format")); assertEquals("living_room", j.getString("id"))
        val o = j.getJSONArray("objects").getJSONObject(0)
        assertEquals("tv_1", o.getString("id"))
        // snapped to the room axes
        val yaw = o.getDouble("yaw")
        assertTrue(abs(yaw) < 1e-6 || abs(abs(yaw) - PI) < 1e-6 || abs(abs(yaw) - PI / 2) < 1e-6)
        assertEquals(16, j.getJSONArray("world_to_room").length())
        assertEquals(4, j.getJSONArray("walls").length())
    }
}
