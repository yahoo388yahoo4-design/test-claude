package com.real2sim.capture

import org.json.JSONArray
import org.json.JSONObject
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/** JVM tests of the mode B hardware budget, the NaN-safe JSON helpers and the crash report text. */
class CaptureSafetyTest {
    private val lenses = listOf("wide_2", "ultrawide_3", "tele_4")

    @Test fun ladderStepsDownLikeIos() {
        val l = CameraBudget.ladder(lenses, 1920, 30)
        assertEquals(5, l.size)
        assertEquals(CameraBudget.Step(lenses, 1920, 30, emptyList()), l[0])
        assertEquals(CameraBudget.Step(lenses, 1280, 30, listOf("reduced to 1280 px")), l[1])
        assertEquals(CameraBudget.Step(lenses, 1280, 24, listOf("reduced to 1280 px", "reduced to 24 fps")), l[2])
        assertEquals(listOf("wide_2", "ultrawide_3"), l[3].lenses)
        assertEquals(listOf("reduced to 1280 px", "reduced to 24 fps", "dropped tele_4"), l[3].actions)
        assertEquals(CameraBudget.Step(listOf("wide_2"), 1280, 24,
            listOf("reduced to 1280 px", "reduced to 24 fps", "dropped tele_4", "dropped ultrawide_3")), l[4])
    }

    @Test fun ladderSkipsStepsAlreadyWithinBudget() {
        val l = CameraBudget.ladder(listOf("wide_0"), 1280, 24)
        assertEquals(1, l.size)
        assertTrue(l[0].actions.isEmpty())
        val l2 = CameraBudget.ladder(listOf("wide_0", "tele_2"), 1024, 30)
        assertEquals(listOf(emptyList(), listOf("reduced to 24 fps"), listOf("reduced to 24 fps", "dropped tele_2")), l2.map { it.actions })
        assertEquals(1024, l2.last().maxWidth)
    }

    @Test fun ladderNeverDropsTheMainLensAndOnlyGetsCheaper() {
        val l = CameraBudget.ladder(lenses, 4000, 60)
        assertTrue(l.all { it.lenses.first() == "wide_2" })
        for (i in 1 until l.size) {
            val a = l[i - 1]; val b = l[i]
            assertTrue(b.maxWidth <= a.maxWidth && b.fps <= a.fps && b.lenses.size <= a.lenses.size)
            assertTrue(b != a)
        }
    }

    @Test fun streamAndPreviewSizes() {
        val sizes = listOf(3840 to 2160, 1920 to 1440, 1920 to 1080, 1280 to 960, 1280 to 720, 640 to 480)
        assertEquals(1920 to 1440, CameraBudget.pickStreamSize(sizes, 1920))
        assertEquals(1280 to 960, CameraBudget.pickStreamSize(sizes, 1280))
        assertEquals(1280 to 720, CameraBudget.pickStreamSize(listOf(1920 to 1080, 1280 to 720), 1280))
        assertNull(CameraBudget.pickStreamSize(sizes, 320))
        // preview: same aspect as the stream, small
        assertEquals(640 to 480, CameraBudget.pickPreviewSize(sizes, 1280, 960))
        assertEquals(1024 to 768, CameraBudget.pickPreviewSize(sizes + (1024 to 768), 1920, 1440))
        assertEquals(1280 to 720, CameraBudget.pickPreviewSize(sizes, 1920, 1080, maxWidth = 1280))
        assertEquals(640 to 480, CameraBudget.pickPreviewSize(listOf(1920 to 1080, 640 to 480), 1600, 1000))
        assertNull(CameraBudget.pickPreviewSize(emptyList(), 1280, 960))
    }

    @Test fun fpsRange() {
        val r = listOf(15 to 15, 7 to 30, 24 to 24, 30 to 30, 15 to 24)
        assertEquals(24 to 24, CameraBudget.pickFpsRange(r, 24))
        assertEquals(30 to 30, CameraBudget.pickFpsRange(r, 30))
        assertEquals(15 to 24, CameraBudget.pickFpsRange(listOf(7 to 30, 15 to 24, 10 to 24), 24))
        assertEquals(15 to 15, CameraBudget.pickFpsRange(listOf(15 to 15, 30 to 30), 24))
        assertNull(CameraBudget.pickFpsRange(listOf(30 to 30, 30 to 60), 24))
    }

    @Test fun jsonSafeReplacesNonFiniteNumbers() {
        assertEquals(JSONObject.NULL, JsonSafe.num(Double.NaN))
        assertEquals(JSONObject.NULL, JsonSafe.num(Float.POSITIVE_INFINITY))
        assertEquals(1.5, JsonSafe.num(1.5f))
        val c = JsonSafe.clean(mapOf("a" to Double.NaN, "b" to listOf(1.0, Double.NEGATIVE_INFINITY, Float.NaN, "x"),
            "c" to mapOf("d" to 2, "e" to Double.POSITIVE_INFINITY), "f" to null)) as JSONObject
        val parsed = JSONObject(c.toString(2))
        assertTrue(parsed.isNull("a"))
        val b = parsed.getJSONArray("b")
        assertEquals(1.0, b.getDouble(0), 0.0)
        assertTrue(b.isNull(1)); assertTrue(b.isNull(2))
        assertEquals("x", b.getString(3))
        assertEquals(2, parsed.getJSONObject("c").getInt("d"))
        assertTrue(parsed.getJSONObject("c").isNull("e"))
        assertTrue(parsed.isNull("f"))
    }

    @Test fun jsonSafeCleansNestedOrgJsonAndNeverThrows() {
        val o = JSONObject().put("ok", 3).put("arr", JSONArray().put(1.25)).put("nested", JSONObject().put("s", "t"))
        assertEquals(JSONObject(o.toString()).toString(), JSONObject(JsonSafe.stringify(o, 2)).toString())
        val meta = JSONObject()
        JsonSafe.put(meta, "nan", Double.NaN)            // org.json's own put would throw here
        JsonSafe.put(meta, "list", listOf(Float.NaN, 2f))
        JsonSafe.put(meta, "fine", 7)
        val parsed = JSONObject(JsonSafe.stringify(meta, 2))
        assertTrue(parsed.isNull("nan"))
        assertTrue(parsed.getJSONArray("list").isNull(0))
        assertEquals(2.0, parsed.getJSONArray("list").getDouble(1), 0.0)
        assertEquals(7, parsed.getInt("fine"))
        val copy = JsonSafe.clean(o) as JSONObject
        assertFalse(copy === o)
        assertEquals(1.25, copy.getJSONArray("arr").getDouble(0), 0.0)
    }

    @Test fun crashReportHasEverythingNeeded() {
        val e = IllegalStateException("boom", RuntimeException("root cause"))
        val t = CrashLog.report(e, "camera2", 1_700_000_000_123L, "Xiaomi 2304FPN6DG", "Android 14 (API 34)", "0.1.0 (1)", "multicam")
        assertTrue(t.contains("time: 2023-11-14T22:13:20.123Z (unix 1700000000)"))
        assertTrue(t.contains("device: Xiaomi 2304FPN6DG"))
        assertTrue(t.contains("os: Android 14 (API 34)"))
        assertTrue(t.contains("app_version: 0.1.0 (1)"))
        assertTrue(t.contains("mode: multicam"))
        assertTrue(t.contains("thread: camera2"))
        assertTrue(t.contains("java.lang.IllegalStateException: boom"))
        assertTrue(t.contains("Caused by: java.lang.RuntimeException: root cause"))
        assertTrue(t.contains("at com.real2sim.capture.CaptureSafetyTest"))
    }
}
