package com.real2sim.capture

import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.nio.file.Files
import kotlin.math.cos
import kotlin.math.sin

/** ESP node maths and esp_nodes.json: same conventions as sensing/esp32/esp_common.py (row-major T, ARKit camera axes). */
class EspNodesTest {
    private val K = doubleArrayOf(640.0, 640.0, 480.0, 360.0)
    private val w = 960; private val h = 720

    /** Row-major camera-to-world at eye, yawed about +y (camera looks down its -z). */
    private fun pose(ex: Double, ey: Double, ez: Double, yaw: Double): DoubleArray {
        val c = cos(yaw); val s = sin(yaw)
        return doubleArrayOf(c, 0.0, s, ex, 0.0, 1.0, 0.0, ey, -s, 0.0, c, ez, 0.0, 0.0, 0.0, 1.0)
    }

    private fun close(a: DoubleArray, b: DoubleArray, tol: Double) =
        assertTrue("${a.toList()} vs ${b.toList()}", (0..2).all { kotlin.math.abs(a[it] - b[it]) < tol })

    @Test fun projectUnprojectRoundTrip() {
        val T = pose(1.0, 1.4, 2.0, 0.4)
        val p = doubleArrayOf(0.3, 1.1, 0.2)
        val uvz = EspMath.project(K, T, p)!!
        close(EspMath.unproject(K, T, uvz[0], uvz[1], uvz[2]), p, 1e-9)
        // the camera looks down -z: a point straight ahead of an un-rotated camera projects to the centre
        val c = EspMath.project(K, pose(0.0, 0.0, 0.0, 0.0), doubleArrayOf(0.0, 0.0, -2.0))!!
        assertEquals(480.0, c[0], 1e-9); assertEquals(360.0, c[1], 1e-9); assertEquals(2.0, c[2], 1e-9)
        // +y world is up in the image (smaller v)
        assertTrue(EspMath.project(K, pose(0.0, 0.0, 0.0, 0.0), doubleArrayOf(0.0, 0.5, -2.0))!![1] < 360.0)
        assertNull(EspMath.project(K, pose(0.0, 0.0, 0.0, 0.0), doubleArrayOf(0.0, 0.0, 2.0)))
    }

    @Test fun triangulateTwoViews() {
        val p = doubleArrayOf(0.0, 0.9, 0.004)
        val views = listOf(pose(-0.5, 1.35, 1.2, -0.4), pose(0.8, 1.35, 1.0, 0.6))
        val rays = views.map { T -> val uv = EspMath.project(K, T, p)!!; EspMath.ray(K, T, uv[0], uv[1]) }
        val (q, rms, ang) = EspMath.triangulate(rays)!!
        close(q, p, 1e-6)
        assertTrue(rms < 1e-6); assertTrue(ang > 5)
        assertNull(EspMath.triangulate(rays.take(1)))
    }

    @Test fun depthAtScalesToDepthMap() {
        val dw = 256; val dh = 192
        val mm = ShortArray(dw * dh) { 1500 }
        mm[96 * dw + 128] = 0                     // a hole is ignored
        assertEquals(1.5, EspMath.depthAt(mm, dw, dh, w, h, 480.0, 360.0)!!, 1e-9)
        val far = ShortArray(dw * dh) { 40000.toShort() }  // > 32.767 m: read unsigned
        assertEquals(40.0, EspMath.depthAt(far, dw, dh, w, h, 10.0, 10.0)!!, 1e-9)
        assertNull(EspMath.depthAt(ShortArray(dw * dh), dw, dh, w, h, 480.0, 360.0))
        assertNull(EspMath.depthAt(mm, dw, dh, w, h, -50.0, 10.0))
    }

    @Test fun storeRanksAndRoundTrips() {
        val dir = Files.createTempDirectory("esp").toFile()
        val st = EspNodeStore.forSession(dir)
        assertTrue(st.put(EspNode(2, doubleArrayOf(1.0, 0.9, 0.0), null, "tap_live")))
        assertFalse("a tag sighting must not replace a tap", st.put(EspNode(2, doubleArrayOf(5.0, 5.0, 5.0), null, "auto_tag_live")))
        assertTrue(st.put(EspNode(1, doubleArrayOf(0.0, 1.9, 2.0), doubleArrayOf(1.0, 0.0, 0.0), "auto_tag_live", 12, 3.5, 0.6, 8)))
        assertTrue(st.save())
        val o = JSONObject(File(dir, EspNodeStore.FILE).readText())
        assertEquals("r2s-esp-nodes", o.getString("format")); assertEquals("session_world", o.getString("frame"))
        val nodes = o.getJSONArray("nodes")
        assertEquals(1, nodes.getJSONObject(0).getInt("node_id"))   // sorted by id
        assertEquals("esp-2", nodes.getJSONObject(1).getString("label"))
        assertTrue(nodes.getJSONObject(1).isNull("normal"))
        val back = EspNodeStore.forSession(dir)
        assertEquals(2, back.all().size)
        assertEquals("tap_live", back.get(2)!!.method)
        assertEquals(12, back.get(1)!!.frameIndex)
        assertNotNull(back.get(1)!!.normal)
        dir.deleteRecursively()
    }

    @Test fun medianRejectsAnOutlier() {
        val pts = listOf(doubleArrayOf(1.0, 1.0, 1.0), doubleArrayOf(1.01, 0.99, 1.0), doubleArrayOf(9.0, 9.0, 9.0))
        close(EspMath.median(pts), doubleArrayOf(1.01, 1.0, 1.0), 1e-9)
    }
}
