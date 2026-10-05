package com.real2sim.capture

/**
 * Hardware budget for mode B, the Android counterpart of the iOS multi-cam `fitBudget`.
 *
 * A HAL accepts only some combinations of streams, sizes and frame rates. Instead of failing (or
 * silently recording one lens), the recorder walks [ladder] until a configuration is accepted:
 * the requested one, then smaller streams (<= [REDUCED_WIDTH] px wide), then [REDUCED_FPS] fps, then
 * the lowest-priority lens is dropped, one at a time, down to the main lens alone. Every step carries
 * the cumulative list of what was changed; the accepted step's list is written to session.json as
 * `budget_actions`.
 *
 * Pure Kotlin (no Android types) so it is unit tested on the JVM.
 */
object CameraBudget {
    const val REDUCED_WIDTH = 1280
    const val REDUCED_FPS = 24

    data class Step(val lenses: List<String>, val maxWidth: Int, val fps: Int, val actions: List<String>)

    /** [lenses] in priority order (main lens first, never dropped). */
    fun ladder(lenses: List<String>, maxWidth: Int, fps: Int): List<Step> {
        require(lenses.isNotEmpty()) { "no lenses" }
        val steps = mutableListOf<Step>()
        val actions = mutableListOf<String>()
        var w = maxWidth
        var f = fps
        var ls = lenses
        steps += Step(ls, w, f, actions.toList())
        if (w > REDUCED_WIDTH) {
            w = REDUCED_WIDTH
            actions += "reduced to $REDUCED_WIDTH px"
            steps += Step(ls, w, f, actions.toList())
        }
        if (f > REDUCED_FPS) {
            f = REDUCED_FPS
            actions += "reduced to $REDUCED_FPS fps"
            steps += Step(ls, w, f, actions.toList())
        }
        while (ls.size > 1) {
            val dropped = ls.last()
            ls = ls.dropLast(1)
            actions += "dropped $dropped"
            steps += Step(ls, w, f, actions.toList())
        }
        return steps
    }

    /** Largest (w, h) no wider than [maxWidth], preferring 4:3; null if nothing fits. */
    fun pickStreamSize(sizes: List<Pair<Int, Int>>, maxWidth: Int): Pair<Int, Int>? {
        val ok = sizes.filter { it.first <= maxWidth }
        return ok.filter { it.first * 3 == it.second * 4 }.maxByOrNull { it.first * it.second }
            ?: ok.maxByOrNull { it.first * it.second }
    }

    /**
     * Preview size for the extra preview stream: same aspect as the recorded stream, no wider than
     * [maxWidth] (a preview does not need full resolution and should cost little bandwidth). Falls back
     * to the closest aspect, then to the smallest size.
     */
    fun pickPreviewSize(sizes: List<Pair<Int, Int>>, streamW: Int, streamH: Int, maxWidth: Int = 1024): Pair<Int, Int>? {
        if (sizes.isEmpty()) return null
        val small = sizes.filter { it.first <= maxWidth }
        val same = small.filter { it.first.toLong() * streamH == it.second.toLong() * streamW }
        if (same.isNotEmpty()) return same.maxByOrNull { it.first * it.second }
        val aspect = streamW.toDouble() / streamH
        if (small.isNotEmpty()) return small.minWithOrNull(compareBy<Pair<Int, Int>> { kotlin.math.abs(it.first.toDouble() / it.second - aspect) }
            .thenByDescending { it.first * it.second })
        return sizes.minByOrNull { it.first * it.second }
    }

    /**
     * AE target fps range for [fps] among the camera's ranges (lower, upper): a fixed range (fps, fps)
     * if offered, else the range whose upper bound is fps with the highest lower bound, else the
     * fastest range not above fps. Null if every range is faster (leave the template default).
     */
    fun pickFpsRange(ranges: List<Pair<Int, Int>>, fps: Int): Pair<Int, Int>? {
        ranges.firstOrNull { it.first == fps && it.second == fps }?.let { return it }
        ranges.filter { it.second == fps }.maxByOrNull { it.first }?.let { return it }
        return ranges.filter { it.second < fps }.maxWithOrNull(compareBy<Pair<Int, Int>> { it.second }.thenBy { it.first })
    }
}
