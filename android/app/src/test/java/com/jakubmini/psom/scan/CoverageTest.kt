package com.jakubmini.psom.scan

import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.cos
import kotlin.math.sin

class CoverageTest {
    private val c = doubleArrayOf(0.0, 0.0, 0.0)

    @Test fun fullRingAtOneHeightCoversOneBand() {
        val cov = Coverage()
        for (k in 0 until 36) {
            val a = Math.toRadians(k * 10.0)
            cov.add(doubleArrayOf(cos(a), 0.3, sin(a)), c)
        }
        assertEquals(12, cov.covered())
    }

    @Test fun viewsFromBelowTheTableDontCount() {
        val cov = Coverage()
        cov.add(doubleArrayOf(1.0, -0.5, 0.0), c)
        assertEquals(0, cov.covered())
    }

    @Test fun topViewLandsInTheTopBand() {
        val cov = Coverage()
        cov.add(doubleArrayOf(0.01, 1.0, 0.0), c)
        assertEquals(true, cov.bins[2 * 12])
    }
}
