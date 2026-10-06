package com.real2sim.capture

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

/** JVM tests of the pts.csv / results merge-join and the ToF second-device selection (mode B). */
class Camera2JoinTest {
    private fun run(pts: List<Pair<Long, Int>>, results: List<Pair<Long, String>>, tol: Long = PtsJoin.DEFAULT_TOL_NS): List<Triple<Int, Long, String?>> {
        val out = mutableListOf<Triple<Int, Long, String?>>()
        PtsJoin.join(pts.asSequence(), results.asSequence(), tol) { i, t, r -> out += Triple(i, t, r) }
        return out
    }

    private val ms = 1_000_000L

    @Test fun joinMatchesExactAndNearestWithinTolerance() {
        val results = listOf(100 * ms to "a", 133 * ms to "b", 166 * ms to "c", 200 * ms to "d")
        val pts = listOf(100 * ms to 0, 134 * ms to 1, 165 * ms to 2, 201 * ms to 3)
        assertEquals(listOf("a", "b", "c", "d"), run(pts, results).map { it.third })
        assertEquals(listOf(0, 1, 2, 3), run(pts, results).map { it.first })
    }

    @Test fun joinResultsArrivingBeforeOrAfterThePtsRow() {
        // result slightly before the sample, and slightly after it: both within 2 ms are taken
        val results = listOf(99 * ms to "before", 201 * ms to "after")
        val pts = listOf(100 * ms to 0, 200 * ms to 1)
        assertEquals(listOf("before", "after"), run(pts, results).map { it.third })
        // the nearer of the two neighbours wins
        val r2 = listOf(100 * ms to "x", 103 * ms to "y")
        assertEquals("y", run(listOf(102 * ms to 0), r2).single().third)
        assertEquals("x", run(listOf(101 * ms to 0), r2).single().third)
    }

    @Test fun joinLeavesSamplesWithoutResultNullButStillEmitsThem() {
        val results = listOf(100 * ms to "a", 300 * ms to "c")
        val pts = listOf(100 * ms to 0, 200 * ms to 1, 300 * ms to 2, 400 * ms to 3)
        val out = run(pts, results)
        assertEquals(4, out.size)
        assertEquals(listOf("a", null, "c", null), out.map { it.third })
        assertEquals(listOf(100 * ms, 200 * ms, 300 * ms, 400 * ms), out.map { it.second })
        // exactly at the tolerance is not a match (strict, as before)
        assertNull(run(listOf(102 * ms to 0), listOf(100 * ms to "a")).single().third)
        assertEquals("a", run(listOf(102 * ms - 1 to 0), listOf(100 * ms to "a")).single().third)
    }

    @Test fun joinHandlesEmptyInputsAndResultsAheadOrBehind() {
        assertEquals(emptyList<Triple<Int, Long, String?>>(), run(emptyList(), listOf(1L to "a")))
        assertEquals(listOf<String?>(null, null), run(listOf(1L to 0, 2L to 1), emptyList()).map { it.third })
        // all results long before the first sample / long after the last one
        assertEquals(listOf<String?>(null), run(listOf(500 * ms to 0), listOf(1 * ms to "a", 2 * ms to "b")).map { it.third })
        assertEquals(listOf<String?>(null), run(listOf(1 * ms to 0), listOf(500 * ms to "a", 600 * ms to "b")).map { it.third })
        // one result may serve two samples that both fall within tolerance (as the old nearest search did)
        assertEquals(listOf("a", "a"), run(listOf(99 * ms to 0, 101 * ms to 1), listOf(100 * ms to "a")).map { it.third })
    }

    @Test fun joinIsSinglePassOverResults() {
        var pulled = 0
        val results = sequence { for (k in 0 until 1000) { pulled++; yield(k * 33 * ms to "r$k") } }
        val pts = (0 until 1000).map { it * 33 * ms to it }.asSequence()
        var n = 0
        PtsJoin.join(pts, results) { _, _, r -> n++; assertEquals(true, r != null) }
        assertEquals(1000, n)
        assertEquals(1000, pulled)
    }

    @Test fun resultTimestampFromLine() {
        assertEquals(123_456_789_012L, PtsJoin.resultTimestampNs("""{"t":123.456789012,"exposure_ns":20000000,"iso":100}"""))
        assertEquals(5_000_000_000L, PtsJoin.resultTimestampNs("""{"exposure_ns":1,"t":5.0}"""))
        assertEquals(1_500_000_000L, PtsJoin.resultTimestampNs("""{"t": 1.5E0}"""))
        assertNull(PtsJoin.resultTimestampNs("""{"exposure_ns":1}"""))
        assertNull(PtsJoin.resultTimestampNs(""))
    }

    @Test fun depthCameraMustBeConcurrentlyOpenable() {
        val logical = "0"; val physical = listOf("2", "3")
        val listed = setOf("0", "1", "4")
        val sets = listOf(setOf("0", "1"), setOf("1", "4"))
        // the logical camera and its members are never a second device
        assertNull(CameraInfo.pickDepthCamera(listOf("0", "2", "3"), logical, physical, listed, sets, true))
        // a listed id in a concurrent set with the logical camera: yes
        assertEquals("1", CameraInfo.pickDepthCamera(listOf("0", "1"), logical, physical, listed, sets, true))
        // a listed id with no such set: no
        assertNull(CameraInfo.pickDepthCamera(listOf("4"), logical, physical, listed, sets, true))
        // hidden ids (not in cameraIdList) cannot be checked: the attempt is allowed
        assertEquals("7", CameraInfo.pickDepthCamera(listOf("4", "7"), logical, physical, listed, sets, true))
        // without concurrent info (API < 30) only hidden ids are tried
        assertNull(CameraInfo.pickDepthCamera(listOf("1", "4"), logical, physical, listed, emptyList(), false))
        assertEquals("7", CameraInfo.pickDepthCamera(listOf("1", "7"), logical, physical, listed, emptyList(), false))
        assertNull(CameraInfo.pickDepthCamera(emptyList(), logical, physical, listed, sets, true))
    }
}
