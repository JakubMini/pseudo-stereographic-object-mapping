package com.jakubmini.psom.scan

import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.sqrt

/**
 * Which directions around the object the camera has seen it from: 12 azimuth x 3 elevation bins
 * (0-30, 30-60, 60-90 degrees), world +y up. The engine's report uses the same bins.
 */
class Coverage(val azBins: Int = 12, val elBins: Int = 3) {
    val bins = BooleanArray(azBins * elBins)

    fun add(cam: DoubleArray, center: DoubleArray) {
        val dx = cam[0] - center[0]
        val dy = cam[1] - center[1]
        val dz = cam[2] - center[2]
        val len = sqrt(dx * dx + dy * dy + dz * dz)
        if (len < 1e-6) return
        val el = Math.toDegrees(asin(dy / len))
        if (el < 0) return  // below the table
        var az = Math.toDegrees(atan2(dz, dx))
        if (az < 0) az += 360
        val e = minOf(elBins - 1, (el / (90.0 / elBins)).toInt())
        val a = minOf(azBins - 1, (az / (360.0 / azBins)).toInt())
        bins[e * azBins + a] = true
    }

    fun covered() = bins.count { it }
    fun fraction() = covered().toDouble() / bins.size
    fun clear() = bins.fill(false)
}

/**
 * Least-squares point closest to every camera's optical axis: the object centre when there is no
 * hit test. Null while the axes are still nearly parallel.
 */
class AxesConvergence {
    private val a = DoubleArray(9)
    private val b = DoubleArray(3)
    private var n = 0

    /** [c] camera centre, [d] viewing direction (any length). */
    fun add(c: DoubleArray, d: DoubleArray) {
        val l = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
        val u = doubleArrayOf(d[0] / l, d[1] / l, d[2] / l)
        for (i in 0..2) for (j in 0..2) {
            val m = (if (i == j) 1.0 else 0.0) - u[i] * u[j]
            a[i * 3 + j] += m
            b[i] += m * c[j]
        }
        n++
    }

    fun center(): DoubleArray? {
        if (n < 3) return null
        val det = det3(a)
        if (det < 1e-2 * n * n * n) return null  // the axes barely cross yet: not enough movement around
        val out = DoubleArray(3)
        for (k in 0..2) {
            val m = a.copyOf()
            for (i in 0..2) m[i * 3 + k] = b[i]
            out[k] = det3(m) / det
        }
        return out
    }

    private fun det3(m: DoubleArray) =
        m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6])
}
