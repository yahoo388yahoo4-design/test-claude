package com.real2sim.capture

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.ByteArrayOutputStream
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.file.Files
import java.util.zip.Deflater

/** JVM tests of the session viewer's readers: JSON, CSV, frames/depth, pts sync, PLY, decimation, session list. */
class SessionDataTest {

    // ---------------------------------------------------------------- JSON

    @Test fun jsonParsesNestedValues() {
        val o = MiniJson.obj("""{"a": 1, "b": [1.5, -2e3, true, null], "c": {"s": "x\"y\\né"}, "e": {}}""")!!
        assertEquals(1.0, o.num("a")!!, 0.0)
        assertEquals(listOf(1.5, -2000.0, true, null), o["b"])
        assertEquals("x\"y\\né", o.obj("c")!!.str("s"))
        assertTrue(o.obj("e")!!.isEmpty())
        assertNull(MiniJson.parseOrNull("{\"a\": }"))
        assertNull(MiniJson.parseOrNull("[1, 2"))
        assertNull(MiniJson.parseOrNull("{} x"))
    }

    @Test fun jsonPrettyRoundTrips() {
        val src = """{"mode":"arcore_rgbd","version":1,"t":12.25,"K":[500,500,320,240],"n":{"x":null,"s":"a\nb"},"l":[{"a":1}]}"""
        val o = MiniJson.parse(src)
        val p = MiniJson.pretty(o)
        assertTrue(p.contains("\"version\": 1,"))          // whole numbers without .0
        assertTrue(p.contains("\"K\": [500, 500, 320, 240]"))
        assertTrue(p.contains("\"s\": \"a\\nb\""))
        assertEquals(o, MiniJson.parse(p))
        // key order preserved
        assertTrue(p.indexOf("\"mode\"") < p.indexOf("\"version\""))
    }

    // ---------------------------------------------------------------- CSV

    @Test fun csvParsesNumbersBooleansAndMissingCells() {
        val t = CsvTable.parse(sequenceOf("t,unix,thermal,battery,battery_state,low_power", "1.5,100,0,0.5,charging,false", "", "2.5,101,1,0.49,unplugged,true", "3.5,102"))
        assertEquals(listOf("t", "unix", "thermal", "battery", "battery_state", "low_power"), t.columns)
        assertEquals(3, t.rows)
        assertArrayEquals(doubleArrayOf(1.5, 2.5, 3.5), t.col("t"), 0.0)
        assertTrue(t.col("battery_state")!!.all { it.isNaN() })
        assertEquals(0.0, t.col("low_power")!![0], 0.0)
        assertEquals(1.0, t.col("low_power")!![1], 0.0)
        assertTrue(t.col("battery")!![2].isNaN())
        assertNull(t.col("nope"))
    }

    @Test fun csvStrideKeepsEveryNthRow() {
        val lines = sequenceOf("t,x") + (0 until 10).map { "$it,${it * 2}" }.asSequence()
        val t = CsvTable.parse(lines, stride = 3)
        assertArrayEquals(doubleArrayOf(0.0, 3.0, 6.0, 9.0), t.col("t"), 0.0)
        assertArrayEquals(doubleArrayOf(0.0, 6.0, 12.0, 18.0), t.col("x"), 0.0)
    }

    @Test fun csvReadSubsamplesLargeFiles() {
        val f = File.createTempFile("imu", ".csv")
        try {
            f.writeText("t,x\n" + (0 until 1000).joinToString("\n") { "${1000 + it},$it" } + "\n")
            val t = CsvTable.read(f, maxRows = 100)!!
            assertTrue("rows ${t.rows}", t.rows in 70..110)   // stride is estimated from the first lines
            assertEquals(1000.0, t.col("t")!![0], 0.0)
            assertNull(CsvTable.read(File(f.path + ".missing")))
        } finally { f.delete() }
    }

    @Test fun rawSensorsSplitPerSensorAndDropEmptyColumns() {
        val lines = sequenceOf(
            "t,sensor,v0,v1,v2,v3,v4,v5,accuracy",
            "1.0,light,120.5,,,,,,3",
            "1.1,accel,0.1,9.8,0.2,,,,3",
            "1.2,light,130.0,,,,,,3",
            "1.3,rotation_vector,0.1,0.2,0.3,0.9,0.01,,3",
        )
        val m = RawSensors.parse(lines, skip = setOf("accel"))
        assertEquals(setOf("light", "rotation_vector"), m.keys)
        assertEquals(listOf("t", "v0"), m["light"]!!.columns)
        assertArrayEquals(doubleArrayOf(120.5, 130.0), m["light"]!!.col("v0"), 0.0)
        assertEquals(listOf("t", "v0", "v1", "v2", "v3", "v4"), m["rotation_vector"]!!.columns)
        assertEquals(mapOf("light" to 2, "accel" to 1, "rotation_vector" to 1), RawSensors.counts(lines))
        val strided = RawSensors.parse(lines, strides = mapOf("light" to 2))
        assertEquals(1, strided["light"]!!.rows)
    }

    // ---------------------------------------------------------------- frames, depth, sync

    private fun frameLine(i: Int, t: Double, d: String = "null") =
        """{"i":$i,"t":$t,"w":640,"h":480,"K":[500,501,320,240],"T":[1,0,0,${i * 0.1},0,1,0,1.5,0,0,1,-${i * 0.2},0,0,0,1],"track":"normal","exp":0.01,"iso":200,"d":$d,"c":null,"dw":160,"dh":120}"""

    @Test fun framesParsePosesIntrinsicsAndDepthRanges() {
        val lines = sequenceOf(
            frameLine(0, 100.0, "[0,50]"),
            frameLine(1, 100.033),
            "not json",
            frameLine(2, 100.066, "[50,40]").replace("\"c\":null", "\"c\":null,\"sd\":[0,70]"),
            """{"i":3,"t":100.1,"w":640,"h":480,"K":null,"T":null,"track":"not_available","d":null}""",
        )
        val f = Frames.parse(lines)
        assertEquals(4, f.frames.size)
        assertEquals(3, f.poses.size)
        assertEquals(2, f.depth.size)
        assertEquals(DepthRef(100.066, 50, 40, 160, 120), f.depth[1])
        assertEquals(1, f.smooth.size)
        val r = f.frames[1]
        assertEquals(1, r.i); assertEquals(640, r.w)
        assertArrayEquals(doubleArrayOf(500.0, 501.0, 320.0, 240.0), r.K, 0.0)
        assertEquals(0.1, r.x(), 1e-9); assertEquals(1.5, r.y(), 1e-9); assertEquals(-0.2, r.z(), 1e-9)
        assertEquals(0.01, r.exp, 0.0); assertEquals(200.0, r.iso, 0.0)
        assertFalse(f.frames[3].hasPose)
        // identity rotation: forward = -Z -> heading atan2(-1, -0) = -90 deg
        assertEquals(-Math.PI / 2, r.heading(), 1e-9)
    }

    @Test fun tofLinesUseWidthHeightKeys() {
        val f = Frames.parse(sequenceOf("""{"i":0,"t":5.0,"w":320,"h":240,"d":[0,99],"c":[0,10],"K":null}"""), "w" to "h")
        assertEquals(DepthRef(5.0, 0, 99, 320, 240), f.depth.single())
    }

    @Test fun nearestAndFloorIndex() {
        val ts = doubleArrayOf(1.0, 2.0, 3.0, 10.0)
        assertEquals(-1, nearestIndex(DoubleArray(0), 1.0))
        assertEquals(0, nearestIndex(ts, -5.0))
        assertEquals(0, nearestIndex(ts, 1.4))
        assertEquals(1, nearestIndex(ts, 1.6))
        assertEquals(2, nearestIndex(ts, 6.4))
        assertEquals(3, nearestIndex(ts, 6.6))
        assertEquals(3, nearestIndex(ts, 99.0))
        assertEquals(0, floorIndex(ts, 0.5))
        assertEquals(1, floorIndex(ts, 2.0))
        assertEquals(2, floorIndex(ts, 9.9))
        assertEquals(3, floorIndex(ts, 11.0))
    }

    @Test fun ptsMapsPlaybackPositionToSessionTime() {
        val p = PtsTable.parse(sequenceOf("frame,pts_us,t_ns", "0,5000000,5000000000", "1,5033000,5033000000", "2,5066000,5066000000", "3,5100000,5100000000"))
        assertEquals(4, p.size)
        assertEquals(5.0, p.firstT!!, 1e-9)
        assertEquals(0.1, p.durationS, 1e-9)
        assertEquals(5.0, p.sessionTime(0.0)!!, 1e-9)
        assertEquals(5.033, p.sessionTime(0.04)!!, 1e-9)   // sample shown at 40 ms is the one at 33 ms
        assertEquals(5.1, p.sessionTime(5.0)!!, 1e-9)
        assertNull(PtsTable.parse(emptySequence()).sessionTime(0.0))
    }

    private fun deflateRaw(b: ByteArray): ByteArray {
        val d = Deflater(1, true); d.setInput(b); d.finish()
        val out = ByteArrayOutputStream(); val tmp = ByteArray(4096)
        while (!d.finished()) { val n = d.deflate(tmp); out.write(tmp, 0, n) }
        d.end(); return out.toByteArray()
    }

    @Test fun depthBlobRoundTripAndColours() {
        val w = 4; val h = 2
        val mm = shortArrayOf(0, 500, 1000, 2000, 3000, 4000, 5000, 9000)
        val raw = ByteBuffer.allocate(w * h * 2).order(ByteOrder.LITTLE_ENDIAN).also { b -> mm.forEach { b.putShort(it) } }.array()
        val other = deflateRaw(ByteArray(100) { it.toByte() })
        val packed = deflateRaw(raw)
        val f = File.createTempFile("depth", ".zlib.bin")
        try {
            f.writeBytes(other + packed)   // second chunk, as SessionWriter.Blob appends them
            val got = Depth.load(f, DepthRef(0.0, other.size.toLong(), packed.size, w, h))!!
            assertArrayEquals(mm, got)
            assertNull(Depth.load(f, DepthRef(0.0, other.size.toLong(), packed.size, w + 1, h)))   // wrong size
            assertNull(Depth.load(f, DepthRef(0.0, f.length(), 10, w, h)))                        // out of range
        } finally { f.delete() }
        val c = Depth.colorize(mm)
        assertEquals(0, c[0])                                     // no depth -> transparent
        assertEquals(0xff, c[1] ushr 24)                          // opaque
        assertEquals(c[6], c[7])                                  // clamped beyond 5 m
        assertTrue((c[1] shr 16 and 0xff) > 150 && (c[1] and 0xff) < 60)   // near is red (turbo)
        assertTrue((c[6] and 0xff) > 2 * (c[6] shr 16 and 0xff))         // far is blue (turbo)
        assertEquals(DepthViz.color(9f, 5f), c[7])
        assertEquals(3000, Depth.medianMm(mm))
    }

    // ---------------------------------------------------------------- PLY

    @Test fun plyBinaryPointsAsWrittenByMapBuilder() {
        val n = 3
        val header = "ply\nformat binary_little_endian 1.0\nelement vertex $n\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
        val b = ByteBuffer.allocate(n * 12).order(ByteOrder.LITTLE_ENDIAN)
        floatArrayOf(1f, 2f, 3f, 4f, 5f, 6f, -1f, -2f, -3f).forEach { b.putFloat(it) }
        val p = Ply.parse(header.toByteArray() + b.array())!!
        assertEquals(3, p.vertexCount)
        assertArrayEquals(floatArrayOf(1f, 2f, 3f, 4f, 5f, 6f, -1f, -2f, -3f), p.xyz, 0f)
        assertNull(p.rgb)
        assertEquals(0, p.tris.size)
    }

    @Test fun plyBinaryMeshWithNormalsFacesAndClass() {
        // iOS mesh.ply layout: 6 floats per vertex, faces = uchar count + 3 int + uchar class
        val header = "ply\nformat binary_little_endian 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\n" +
            "property float nx\nproperty float ny\nproperty float nz\nelement face 2\nproperty list uchar int vertex_indices\nproperty uchar classification\nend_header\n"
        val b = ByteBuffer.allocate(4 * 24 + 2 * 14).order(ByteOrder.LITTLE_ENDIAN)
        for (k in 0 until 4) { b.putFloat(k.toFloat()); b.putFloat(0f); b.putFloat(0f); b.putFloat(0f); b.putFloat(1f); b.putFloat(0f) }
        b.put(3); b.putInt(0); b.putInt(1); b.putInt(2); b.put(1)
        b.put(3); b.putInt(0); b.putInt(2); b.putInt(9); b.put(2)   // index 9 out of range -> dropped
        val p = Ply.parse(header.toByteArray() + b.array())!!
        assertEquals(4, p.vertexCount)
        assertEquals(3f, p.xyz[9], 0f)
        assertArrayEquals(intArrayOf(0, 1, 2), p.tris)
    }

    @Test fun plyAsciiWithColoursAndQuad() {
        val s = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n" +
            "element face 1\nproperty list uchar int vertex_indices\nend_header\n" +
            "0 0 0 255 0 0\n1 0 0 0 255 0\n1 1 0 0 0 255\n0 1 0 10 20 30\n4 0 1 2 3\n"
        val p = Ply.parse(s.toByteArray())!!
        assertArrayEquals(intArrayOf(0xff0000, 0x00ff00, 0x0000ff, 0x0a141e), p.rgb)
        assertArrayEquals(intArrayOf(0, 1, 2, 0, 2, 3), p.tris)   // quad -> fan
        assertNull(Ply.parse("not a ply".toByteArray()))
        assertNull(Ply.parse("ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nend_header\n1\n".toByteArray()))
    }

    // ---------------------------------------------------------------- charts

    @Test fun decimateKeepsSpikesAndSkipsNaN() {
        val n = 10_000
        val xs = DoubleArray(n) { it / 100.0 }
        val ys = DoubleArray(n) { if (it == 5_000) 100.0 else if (it == 7_000) Double.NaN else 0.0 }
        val out = decimateMinMax(xs, ys, 0.0, 100.0, 50)
        assertTrue(out.size <= 50 * 4)
        var max = 0f; var k = 1; while (k < out.size) { max = maxOf(max, out[k]); assertFalse(out[k].isNaN()); k += 2 }
        assertEquals(100f, max, 0f)
        // x stays sorted
        k = 2; while (k < out.size) { assertTrue(out[k] >= out[k - 2]); k += 2 }
        // small input passes through
        assertArrayEquals(floatArrayOf(0f, 1f, 1f, 2f), decimateMinMax(doubleArrayOf(0.0, 1.0), doubleArrayOf(1.0, 2.0), 0.0, 1.0, 10), 0f)
        assertEquals(0, decimateMinMax(DoubleArray(0), DoubleArray(0), 0.0, 1.0, 10).size)
    }

    @Test fun cameraTableFromFrames() {
        val f = Frames.parse(sequenceOf(frameLine(0, 1.0), frameLine(1, 2.0)))
        val t = SensorPlots.cameraTable(f.frames)!!
        assertArrayEquals(doubleArrayOf(10.0, 10.0), t.col("exp_ms"), 1e-9)
        assertArrayEquals(doubleArrayOf(2.0, 2.0), t.col("iso_100"), 1e-9)
        assertNull(SensorPlots.cameraTable(Frames.parse(sequenceOf("""{"t":1.0}""")).frames))
    }

    // ---------------------------------------------------------------- session list

    @Test fun sessionListNewestFirstWithSummary() {
        val root = Files.createTempDirectory("sessions").toFile()
        try {
            val a = File(root, "20260101_100000_arcore_rgbd").apply { mkdirs() }
            File(a, "session.json").writeText("""{"mode":"arcore_rgbd","start_uptime":100.0,"end_uptime":165.5,"counts":{"frames":1800}}""")
            File(a, "video.mp4").writeBytes(ByteArray(1000))
            File(a, "DONE").writeText("ok\n")
            val b = File(root, "20260102_090000_multicam").apply { mkdirs() }
            File(b, "session.json").writeText("""{"mode":"multicam","start_uptime":1.0,"end_uptime":11.0,"counts":{"frames":0,"cam_samples":600}}""")
            File(b, "cams").mkdirs(); File(b, "cams/wide.mp4").writeBytes(ByteArray(10))
            val c = File(root, "20251231_235959_arcore_rgbd").apply { mkdirs() }   // crashed: no session.json
            File(c, "frames.jsonl").writeText("{}\n{}\n\n{}\n")
            File(root, "camera_inventory.json").writeText("{}")                   // files are not sessions
            val d = File(root, "20251231_120000_arcore_rgbd").apply { mkdirs() }   // recorder never started
            File(d, "session.json").writeText("""{"mode":"arcore_rgbd","start_uptime":5.0,"end_uptime":5.6,"aborted":true,"abort_reason":"ARCore session failed: FatalException"}""")
            File(d, "DONE").writeText("ok\n")

            val l = SessionScan.list(root)
            assertEquals(listOf(b.name, a.name, c.name, d.name), l.map { it.name })
            assertEquals("ARCore session failed: FatalException", l[3].aborted); assertTrue(l[3].complete)
            assertNull(l[0].aborted); assertNull(l[2].aborted)
            assertEquals("multicam", l[0].mode); assertEquals(600, l[0].frames); assertEquals(10.0, l[0].durationS!!, 1e-9)
            assertEquals("B: Camera2 multi-cam", l[0].modeLabel)
            assertEquals(1800, l[1].frames); assertEquals(65.5, l[1].durationS!!, 1e-9)
            assertTrue(l[1].bytes >= 1000)
            assertTrue(l[1].complete)
            assertEquals("arcore_rgbd", l[2].mode); assertEquals(3, l[2].frames); assertNull(l[2].durationS); assertFalse(l[2].complete)
            assertEquals(listOf("DONE", "session.json", "video.mp4"), SessionScan.files(a).map { it.first })
            assertEquals(listOf("cams/wide.mp4", "session.json"), SessionScan.files(b).map { it.first })
        } finally { root.deleteRecursively() }
    }

    @Test fun formatting() {
        assertEquals("512 B", fmtBytes(512))
        assertEquals("1.5 KB", fmtBytes(1536))
        assertEquals("2.0 MB", fmtBytes(2L shl 20))
        assertEquals("1.50 GB", fmtBytes(3L shl 29))
        assertEquals("1:05", fmtDuration(65.9))
        assertEquals("1:01:01", fmtDuration(3661.0))
        assertEquals("?", fmtDuration(null))
        assertNotNull(modeLabel("sensors"))
    }
}
