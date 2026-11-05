package com.jakubmini.psom.scan

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
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

    @Test fun axesConvergeOnTheObject() {
        val ax = AxesConvergence()
        val target = doubleArrayOf(0.1, 0.05, -0.3)
        for (k in 0 until 8) {
            val a = Math.toRadians(k * 45.0)
            val cam = doubleArrayOf(target[0] + 0.3 * cos(a), target[1] + 0.2, target[2] + 0.3 * sin(a))
            ax.add(cam, DoubleArray(3) { target[it] - cam[it] })
        }
        assertArrayEquals(target, ax.center(), 1e-9)
    }

    @Test fun parallelAxesGiveNoCentre() {
        val ax = AxesConvergence()
        for (k in 0 until 5) ax.add(doubleArrayOf(k * 0.01, 0.0, 0.0), doubleArrayOf(0.0, 0.0, -1.0))
        assertNull(ax.center())
    }
}
