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
