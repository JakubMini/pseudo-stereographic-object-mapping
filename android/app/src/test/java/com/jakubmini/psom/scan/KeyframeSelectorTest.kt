package com.jakubmini.psom.scan

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

class KeyframeSelectorTest {
    private fun at(x: Double, yawDeg: Double = 0.0): Pose {
        val h = Math.toRadians(yawDeg) / 2
        return Pose(x, 0.0, 0.0, 0.0, sin(h), 0.0, cos(h))
    }

    @Test fun firstTrackingFrameIsKept() {
        val s = KeyframeSelector()
        assertEquals(Verdict.ACCEPTED, s.offer(0, true, at(0.0), 100.0))
    }

    @Test fun lostTrackingIsRejectedAndCounted() {
        val s = KeyframeSelector()
        assertEquals(Verdict.TRACKING, s.offer(0, false, at(0.0), 100.0))
        assertEquals(1, s.rejected["tracking"])
    }

    @Test fun needsMovementOrTurnBetweenKeyframes() {
        val s = KeyframeSelector()
        s.offer(0, true, at(0.0), 100.0)
        assertEquals(Verdict.STILL, s.offer(1_000_000_000, true, at(0.01), 100.0))
        assertEquals(Verdict.ACCEPTED, s.offer(2_000_000_000, true, at(0.03), 100.0))
        assertEquals(Verdict.ACCEPTED, s.offer(3_000_000_000, true, at(0.03, 6.0), 100.0))
    }

    @Test fun rateIsCapped() {
        val s = KeyframeSelector(maxRateHz = 3.0)
        s.offer(0, true, at(0.0), 100.0)
        assertEquals(Verdict.RATE, s.offer(100_000_000, true, at(0.1), 100.0))
    }

    @Test fun blurryFramesAreRejectedAgainstRecentSharpness() {
        val s = KeyframeSelector()
        var t = 0L
        for (k in 0 until 6) { s.offer(t, true, at(0.05 * k), 100.0); t += 1_000_000_000 }
        assertEquals(Verdict.BLUR, s.offer(t, true, at(0.3), 20.0))
        assertEquals(1, s.rejected["blur"])
    }

    @Test fun sharpnessSeesEdgesNotFlatGrey() {
        val w = 64
        val flat = ByteArray(w * w) { 128.toByte() }
        val checker = ByteArray(w * w) { i -> if (((i % w) / 4 + (i / w) / 4) % 2 == 0) 0 else 255.toByte() }
        assertEquals(0.0, KeyframeSelector.sharpness(flat, w, w, w, 1), 1e-9)
        assertTrue(KeyframeSelector.sharpness(checker, w, w, w, 1) > 1000)
    }

    @Test fun aPoseJumpIsRejectedThenANewStretchStarts() {
        val s = KeyframeSelector()
        s.offer(0, true, at(0.0), 100.0)
        assertEquals(Verdict.JUMP, s.offer(500_000_000, true, at(2.0), 100.0))  // 4 m/s: lost track
        assertEquals(1, s.rejected["jump"])
        assertEquals(Verdict.JUMP, s.offer(1_000_000_000, true, at(2.05), 100.0))
        // still far from the old pose a second later: accept it as the start of a new stretch
        assertEquals(Verdict.ACCEPTED, s.offer(1_600_000_000, true, at(2.1), 100.0))
        assertEquals(-1L, s.jumpingSince)
    }

    @Test fun normalWalkingIsNotAJump() {
        val s = KeyframeSelector()
        s.offer(0, true, at(0.0), 100.0)
        assertEquals(Verdict.ACCEPTED, s.offer(500_000_000, true, at(0.1), 100.0))  // 0.2 m/s
    }
}
