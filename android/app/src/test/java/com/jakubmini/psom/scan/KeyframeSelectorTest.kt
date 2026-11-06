package com.jakubmini.psom.scan

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

class KeyframeSelectorTest {
    /** Direction from the object to the camera after walking [deg] degrees around it. */
    private fun around(deg: Double): DoubleArray {
        val a = Math.toRadians(deg)
        return doubleArrayOf(cos(a), 0.5, sin(a))
    }

    private val s100 = 100.0
    private val second = 1_000_000_000L

    @Test fun firstPhotoIsKept() {
        assertEquals(Verdict.ACCEPTED, KeyframeSelector().offer(0, around(0.0), 0.1, s100))
    }

    @Test fun needsATurnOrAPauseBetweenPhotos() {
        val s = KeyframeSelector()
        s.offer(0, around(0.0), 0.1, s100)
        assertEquals(Verdict.STILL, s.offer(second / 2, around(1.0), 0.1, s100))
        assertEquals(Verdict.ACCEPTED, s.offer(second / 2, around(6.0), 0.1, s100))
        // standing still for a while still adds one now and then
        assertEquals(Verdict.ACCEPTED, s.offer(3 * second, around(6.0), 0.1, s100))
    }

    @Test fun rateIsCapped() {
        val s = KeyframeSelector()
        s.offer(0, around(0.0), 0.1, s100)
        assertEquals(Verdict.RATE, s.offer(second / 10, around(30.0), 0.1, s100))
    }

    @Test fun turningFastIsRejectedAndCounted() {
        val s = KeyframeSelector()
        s.offer(0, around(0.0), 0.1, s100)
        assertEquals(Verdict.FAST, s.offer(second, around(20.0), 2.5, s100))
        assertEquals(1, s.rejected["motion"])
    }

    @Test fun blurryPhotosAreRejectedAgainstRecentSharpness() {
        val s = KeyframeSelector()
        var t = 0L
        for (k in 0 until 6) { s.offer(t, around(10.0 * k), 0.1, s100); t += second }
        assertEquals(Verdict.BLUR, s.offer(t, around(70.0), 0.1, 20.0))
        assertEquals(1, s.rejected["blur"])
    }

    @Test fun stopsAtTheLimit() {
        val s = KeyframeSelector(maxFrames = 3)
        for (k in 0 until 3) s.offer(k * second, around(10.0 * k), 0.1, s100)
        assertEquals(Verdict.FULL, s.offer(10 * second, around(90.0), 0.1, s100))
        assertEquals(3, s.accepted)
    }

    @Test fun sharpnessSeesEdgesNotFlatGrey() {
        val w = 64
        val flat = ByteArray(w * w) { 128.toByte() }
        val checker = ByteArray(w * w) { i -> if (((i % w) / 4 + (i / w) / 4) % 2 == 0) 0 else 255.toByte() }
        assertEquals(0.0, KeyframeSelector.sharpness(flat, w, w, w, 1), 1e-9)
        assertTrue(KeyframeSelector.sharpness(checker, w, w, w, 1) > 1000)
    }

    @Test fun keepsTheSharpestFrameOfItsWindow() {
        val w = SharpestInWindow<String>(windowNs = 300)
        var made = 0
        w.offer(0, 50.0) { made++; "a" }
        w.offer(100, 80.0) { made++; "b" }
        w.offer(200, 60.0) { made++; "c" }  // not sharper: never copied
        assertEquals(null, w.tick(250))
        assertEquals("b", w.tick(300))
        assertEquals(2, made)
        assertEquals(null, w.flush())
    }
}
