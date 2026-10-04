package com.real2sim.capture

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.nio.file.Files

/** Loads fabricated mode A / mode B / sensors-only session folders the way the viewer does and checks the time sync. */
class ViewerDataTest {
    private fun tmp() = Files.createTempDirectory("viewer").toFile()

    private fun pose(x: Double, z: Double) = "[1,0,0,$x,0,1,0,1.2,0,0,1,$z,0,0,0,1]"

    @Test fun modeASyncPosesDepthAndMap() {
        val dir = File(tmp(), "20260301_120000_arcore_rgbd").apply { mkdirs() }
        try {
            File(dir, "session.json").writeText("""{"mode":"arcore_rgbd","start_uptime":99.5,"end_uptime":102.0,"video":{"file":"video.mp4","width":640,"height":480,"codec":"hevc"}}""")
            File(dir, "video.mp4").writeBytes(ByteArray(16))
            // video starts at boottime 100.0; 3 samples 0.5 s apart
            File(dir, "video.mp4.pts.csv").writeText("frame,pts_us,t_ns\n0,100000000,100000000000\n1,100500000,100500000000\n2,101000000,101000000000\n")
            File(dir, "frames.jsonl").writeText(
                """{"i":0,"t":100.0,"w":640,"h":480,"K":[500,500,320,240],"T":${pose(0.0, 0.0)},"track":"normal","d":[0,10],"dw":160,"dh":120}""" + "\n" +
                """{"i":1,"t":100.5,"w":640,"h":480,"K":[500,500,320,240],"T":${pose(1.0, 0.0)},"track":"normal","d":null}""" + "\n" +
                """{"i":2,"t":101.0,"w":640,"h":480,"K":[500,500,320,240],"T":${pose(2.0, -1.0)},"track":"normal","d":[10,12],"sd":[0,9],"dw":160,"dh":120}""" + "\n")
            File(dir, "depth.zlib.bin").writeBytes(ByteArray(22))
            File(dir, "extras").mkdirs()
            File(dir, "extras/camera_characteristics_arcore_0.json").writeText("""{"android.sensor.orientation": 90}""")
            File(dir, "extras/map").mkdirs()
            File(dir, "extras/map/points.ply").writeText("ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nproperty float z\nend_header\n0 0 0\n")
            File(dir, "extras/map/occupancy.png").writeBytes(ByteArray(4))
            File(dir, "imu.csv").writeText("t,ax,ay,az,gx,gy,gz,grx,gry,grz,qx,qy,qz,qw,mx,my,mz,mag_acc,heading\n" +
                (0 until 30).joinToString("\n") { k -> "${99.8 + k * 0.1},0.1,0.2,0.3,0,0,$k,0,-1,0,0,0,0,1,20,0,-40,3,90" } + "\n")
            File(dir, "extras/sensors_raw.csv").writeText("t,sensor,v0,v1,v2,v3,v4,v5,accuracy\n100.1,light,50,,,,,,3\n100.2,accel,1,2,3,,,,3\n100.9,light,70,,,,,,3\n")

            val d = ViewerData.load(dir)
            assertEquals("arcore_rgbd", d.mode)
            assertEquals(1, d.videos.size)
            assertEquals(100.0, d.t0, 1e-9)
            assertEquals(1.0, d.duration, 1e-9)                       // pts span
            assertEquals(640, d.videos[0].w)
            assertEquals(100.5, d.sessionTime(0, 0.7), 1e-9)          // sample shown at 0.7 s is the one at 0.5 s
            assertEquals(101.0, d.sessionTime(0, 1.0), 1e-9)
            assertEquals(2, d.frames.depth.size)
            assertEquals(1, d.frames.smooth.size)
            assertNotNull(d.depthBlob); assertNull(d.smoothBlob)
            assertEquals(1, nearestIndex(d.frames.depthTimes, d.sessionTime(0, 1.0)))
            assertEquals(1.0, d.poseAt(100.7)!!.x(), 1e-9)
            assertEquals(0.0, d.poseAt(50.0)!!.x(), 1e-9)             // before the first pose: clamp
            assertArrayEquals(floatArrayOf(0f, 1.2f, 0f, 1f, 1.2f, 0f, 2f, 1.2f, -1f), d.trajectory, 1e-6f)
            assertEquals("points.ply", d.plyFile!!.name)
            assertEquals("map", d.mapDir!!.name)
            assertEquals(90, d.defaultRotation)
            assertTrue(d.files.any { it.first == "extras/map/points.ply" })

            val charts = SensorCharts.build(d)
            val titles = charts.charts.map { it.title }
            assertTrue(titles.toString(), titles.any { it.startsWith("User acceleration") })
            assertTrue(titles.toString(), titles.any { it.startsWith("light") })
            assertTrue(titles.none { it.startsWith("accel ") })     // covered by accel.csv, skipped from sensors_raw
            assertTrue(charts.x0 <= -0.2f + 1e-4f && charts.x0 >= -5f)
            // series x are seconds since the first video frame
            val gyro = charts.charts.first { it.title.startsWith("Rotation rate") }.series.first { it.name == "gz" }
            assertEquals(-0.2f, gyro.xy[0], 1e-4f)
        } finally { dir.parentFile!!.deleteRecursively() }
    }

    @Test fun modeBLensOffsetsToFRawCounts() {
        val dir = File(tmp(), "20260301_130000_multicam").apply { mkdirs() }
        try {
            File(dir, "session.json").writeText("""{"mode":"multicam","start_uptime":9.0,"end_uptime":20.0,
                "android":{"multicam":{"logical_id":"0","streams":[
                  {"name":"wide_23mm","physical_id":"2","w":1920,"h":1440,"file":"cams/wide_23mm.mp4","codec":"video/hevc"},
                  {"name":"tele_75mm","physical_id":"3","w":1920,"h":1440,"file":"cams/tele_75mm.mp4","codec":"video/hevc"}]}}}""")
            val cams = File(dir, "cams").apply { mkdirs() }
            for ((n, t0) in listOf("wide_23mm" to 10.0, "tele_75mm" to 10.2)) {
                File(cams, "$n.mp4").writeBytes(ByteArray(8))
                val us = (t0 * 1e6).toLong()
                File(cams, "$n.mp4.pts.csv").writeText("frame,pts_us,t_ns\n0,$us,${us * 1000}\n1,${us + 2_000_000},${(us + 2_000_000) * 1000}\n")
                File(cams, "$n.jsonl").writeText("""{"i":0,"t":$t0,"w":1920,"h":1440,"K":[1500,1500,960,720],"exp":0.004,"iso":100}""" + "\n")
            }
            File(cams, "tof_depth.jsonl").writeText("""{"i":0,"t":10.5,"w":240,"h":180,"d":[0,5],"c":[0,2],"K":null}""" + "\n")
            File(cams, "tof_depth.zlib.bin").writeBytes(ByteArray(5))
            File(cams, "raw").mkdirs()
            File(cams, "raw/wide_23mm_11.000000.dng").writeBytes(ByteArray(1))
            File(cams, "raw/wide_23mm_10.000000.dng").writeBytes(ByteArray(1))
            File(dir, "extras").mkdirs()
            File(dir, "extras/camera_inventory.json").writeText("""{"cameras":{"0":{"android.sensor.orientation":270}}}""")

            val d = ViewerData.load(dir)
            assertEquals(listOf("wide_23mm", "tele_75mm"), d.videos.map { it.name })   // session.json stream order
            assertEquals(10.0, d.t0, 1e-9)
            assertEquals(0.2, d.videos[1].firstT - d.t0, 1e-9)                         // tele starts 0.2 s later
            assertEquals(2.2, d.duration, 1e-9)
            assertEquals("hevc", d.videos[0].codec); assertEquals("2", d.videos[0].physicalId)
            assertEquals(10.1, d.sessionTime(1, 0.1), 1e-9)                             // before tele first frame: session clock
            assertEquals(10.2, d.sessionTime(1, 0.3), 1e-9)
            assertEquals(1, d.tof!!.depth.size); assertEquals(240, d.tof!!.depth[0].w)
            assertEquals(listOf("wide_23mm_10.000000.dng", "wide_23mm_11.000000.dng"), d.rawFiles.map { it.name })
            assertArrayEquals(doubleArrayOf(10.0, 11.0), d.rawTimes, 1e-9)
            assertEquals(270, d.defaultRotation)
            assertTrue(d.frames.poses.isEmpty()); assertNull(d.plyFile); assertNull(d.mapDir)
        } finally { dir.parentFile!!.deleteRecursively() }
    }

    @Test fun sensorsOnlyUsesSessionClock() {
        val dir = File(tmp(), "20260301_140000_sensors").apply { mkdirs() }
        try {
            File(dir, "session.json").writeText("""{"mode":"sensors","start_uptime":50.0,"end_uptime":80.0}""")
            File(dir, "altimeter.csv").writeText("t,rel_alt_m,pressure_kpa\n51,0,101.3\n79,0.5,101.29\n")
            val d = ViewerData.load(dir)
            assertTrue(d.videos.isEmpty())
            assertEquals(50.0, d.t0, 1e-9)
            assertEquals(30.0, d.duration, 1e-9)
            assertEquals(55.0, d.sessionTime(0, 5.0), 1e-9)
            val c = SensorCharts.build(d)
            assertEquals(listOf("Relative altitude (m)", "Air pressure (kPa)"), c.charts.map { it.title })
            assertEquals(1f, c.charts[0].series[0].xy[0], 1e-6f)
        } finally { dir.parentFile!!.deleteRecursively() }
    }
}
