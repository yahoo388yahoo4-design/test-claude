package com.real2sim.capture

/** Pure helpers for the closed-loop move task (unit-tested in NavMathTest). */
object NavMath {
    /** Signed distance travelled from (sx, sz) to (x, z) along heading h. Both points must be the same tracked point (the phone pose). */
    fun travelledAlong(x: Float, z: Float, sx: Float, sz: Float, h: Float): Float =
        (x - sx) * kotlin.math.cos(h) + (z - sz) * kotlin.math.sin(h)

    /** Remaining distance of a `dist` move started at (sx, sz), measured at the phone pose (x, z). */
    fun moveRemaining(dist: Float, x: Float, z: Float, sx: Float, sz: Float, h: Float): Float =
        kotlin.math.abs(dist) - kotlin.math.abs(travelledAlong(x, z, sx, sz, h))
}
