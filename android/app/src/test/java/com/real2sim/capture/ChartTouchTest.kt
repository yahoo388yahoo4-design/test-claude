package com.real2sim.capture

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

/** ChartView touch decision: vertical drags are left to the ScrollView, horizontal drags and taps seek. */
class ChartTouchTest {
    private val slop = 16f

    @Test fun tapSeeksOnUpWithoutClaiming() {
        val g = ChartGesture(slop)
        g.down(100f, 50f)
        assertFalse(g.move(104f, 53f))       // jitter below slop: not claimed
        assertFalse(g.claimed)
        assertTrue(g.up())                   // released without a drag -> tap -> seek once
        assertFalse(g.claimed)
    }

    @Test fun horizontalDragClaimsOnceThenSeeks() {
        val g = ChartGesture(slop)
        g.down(100f, 50f)
        assertFalse(g.move(110f, 52f))       // under slop
        assertTrue(g.move(130f, 55f))        // |dx|=30 > slop and > |dy|=5: claim now
        assertTrue(g.claimed)
        assertFalse(g.move(200f, 90f))       // already claimed: no second disallow-intercept, keeps seeking
        assertTrue(g.claimed)
        assertFalse(g.up())                  // drag end is not a tap
        assertFalse(g.claimed)
    }

    @Test fun verticalDragNeverClaims() {
        val g = ChartGesture(slop)
        g.down(100f, 50f)
        assertFalse(g.move(105f, 90f))       // |dy|=40 > slop but |dx|=5: parent (ScrollView) should take it
        assertFalse(g.move(120f, 200f))      // |dx|=20 > slop but |dy| larger: still not ours
        assertFalse(g.claimed)
        g.cancel()                           // ScrollView intercepted
        assertFalse(g.claimed)
        assertTrue(g.up())                   // a fresh release with no claim is a tap again
    }

    @Test fun diagonalDragGoesToTheLargerAxis() {
        val g = ChartGesture(slop)
        g.down(0f, 0f)
        assertFalse(g.move(20f, 20f))        // equal: not strictly more horizontal -> parent
        assertTrue(g.move(41f, 40f))
        assertTrue(g.claimed)
    }

    @Test fun cancelResetsAndNextDownStartsFresh() {
        val g = ChartGesture(slop)
        g.down(0f, 0f)
        assertTrue(g.move(50f, 0f))
        g.cancel()
        assertFalse(g.claimed)
        g.down(300f, 0f)
        assertFalse(g.move(305f, 0f))        // distance is measured from the new DOWN, not the old one
        assertFalse(g.claimed)
    }
}
