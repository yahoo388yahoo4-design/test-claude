package com.real2sim.capture

import kotlin.math.abs

/**
 * Streaming join of a video's pts.csv rows to the per-frame CaptureResult lines of the same camera
 * (Camera2Recorder, mode B). Both inputs are in increasing sensor-timestamp order, so a two-cursor
 * merge finds, for every video sample, the nearest result within [tolNs] in one pass and O(1) memory.
 */
object PtsJoin {
    const val DEFAULT_TOL_NS = 2_000_000L

    /**
     * [pts]: (t_ns, frame index) per video sample; [results]: (t_ns, line) per capture result, both in
     * increasing t. [out] is called once per pts row, in order, with the nearest result line within
     * [tolNs] or null when there is none (results may be missing: dropped writes, encoder lag).
     */
    fun join(
        pts: Sequence<Pair<Long, Int>>,
        results: Sequence<Pair<Long, String>>,
        tolNs: Long = DEFAULT_TOL_NS,
        out: (i: Int, tNs: Long, result: String?) -> Unit,
    ) {
        val it = results.iterator()
        var prev: Pair<Long, String>? = null
        var cur: Pair<Long, String>? = if (it.hasNext()) it.next() else null
        for ((tNs, i) in pts) {
            while (cur != null && cur.first < tNs) { prev = cur; cur = if (it.hasNext()) it.next() else null }
            val p = prev; val c = cur
            val best = when {
                p == null -> c
                c == null -> p
                abs(c.first - tNs) < abs(p.first - tNs) -> c
                else -> p
            }
            out(i, tNs, best?.takeIf { abs(it.first - tNs) < tolNs }?.second)
        }
    }

    private val tKey = Regex("\"t\":\\s*(-?[0-9][0-9.eE+-]*)")

    /** Sensor timestamp (ns) of one results line, from its `"t": <seconds>` field; null if absent. */
    fun resultTimestampNs(line: String): Long? {
        val m = tKey.find(line) ?: return null
        val t = m.groupValues[1].toDoubleOrNull() ?: return null
        if (t.isNaN() || t.isInfinite()) return null
        return Math.round(t * 1e9)
    }
}
