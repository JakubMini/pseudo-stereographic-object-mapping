package com.jakubmini.psom.scan

import kotlin.math.acos
import kotlin.math.min
import kotlin.math.sqrt

enum class Verdict(val key: String) { ACCEPTED("accepted"), STILL("still"), BLUR("blur"), RATE("rate"), FAST("motion"), FULL("limit") }

/**
 * Keeps a photo when the view has turned enough since the last one (or a while has passed), the
 * phone isn't turning so fast that it blurs, the image is sharp compared with the recent ones, and
 * the rate stays under [maxRateHz]. Without tracking there are no positions, only directions.
 */
class KeyframeSelector(
    private val minTurnDeg: Double = 4.0,
    private val maxWaitNs: Long = 1_500_000_000L,
    private val sharpFactor: Double = 0.6,
    private val maxRateHz: Double = 3.0,
    private val maxTurnRate: Double = MAX_TURN_RATE,
    val maxFrames: Int = 120,
) {
    private var lastDir: DoubleArray? = null
    private var lastT = Long.MIN_VALUE
    private val recent = ArrayDeque<Double>()
    val rejected = mutableMapOf<String, Int>()
    var accepted = 0
        private set

    /** Cheap checks first, so the caller only copies and measures photos that could be kept. */
    fun wouldConsider(tNs: Long, dir: DoubleArray?, turnRate: Double): Verdict {
        if (accepted >= maxFrames) return Verdict.FULL
        if (turnRate > maxTurnRate) return Verdict.FAST
        if (accepted == 0) return Verdict.ACCEPTED
        if (tNs - lastT < 1e9 / maxRateHz) return Verdict.RATE
        val l = lastDir
        if (l != null && dir != null && angleDeg(l, dir) < minTurnDeg && tNs - lastT < maxWaitNs) return Verdict.STILL
        return Verdict.ACCEPTED
    }

    fun offer(tNs: Long, dir: DoubleArray?, turnRate: Double, sharpness: Double): Verdict {
        var v = wouldConsider(tNs, dir, turnRate)
        if (v == Verdict.ACCEPTED && recent.size >= 5 && sharpness < median() * sharpFactor) v = Verdict.BLUR
        if (v == Verdict.ACCEPTED || v == Verdict.BLUR) {
            recent.addLast(sharpness)
            if (recent.size > 30) recent.removeFirst()
        }
        if (v == Verdict.ACCEPTED) {
            lastDir = dir
            lastT = tNs
            accepted++
        } else if (v != Verdict.STILL && v != Verdict.RATE) {
            rejected[v.key] = (rejected[v.key] ?: 0) + 1
        }
        return v
    }

    private fun median(): Double = recent.sorted()[recent.size / 2]

    companion object {
        /** rad/s. At 1/30 s exposure and a ~1400 px focal length, 0.7 rad/s already smears ~30 px. */
        const val MAX_TURN_RATE = 0.7

        fun angleDeg(a: DoubleArray, b: DoubleArray): Double {
            val na = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
            val nb = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2])
            val c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)
            return Math.toDegrees(acos(min(1.0, maxOf(-1.0, c))))
        }

        /** Variance of the 4-neighbour Laplacian of a luma plane, sampled every [step] pixels. */
        fun sharpness(y: ByteArray, width: Int, height: Int, rowStride: Int, step: Int = 4): Double {
            var sum = 0.0
            var sum2 = 0.0
            var n = 0
            var r = step
            while (r < height - step) {
                var c = step
                while (c < width - step) {
                    fun at(rr: Int, cc: Int) = (y[rr * rowStride + cc].toInt() and 0xFF).toDouble()
                    val l = at(r - step, c) + at(r + step, c) + at(r, c - step) + at(r, c + step) - 4 * at(r, c)
                    sum += l
                    sum2 += l * l
                    n++
                    c += step
                }
                r += step
            }
            if (n == 0) return 0.0
            val mean = sum / n
            return sum2 / n - mean * mean
        }
    }
}

/**
 * Once a photo is due, the next few frames are all fair candidates: keeps the sharpest of those
 * that arrive within [windowNs] of the first, and hands it over when a later frame closes the window
 * (or on [flush]). A hand-held phone pauses for a moment now and then; this catches that moment.
 */
class SharpestInWindow<T>(private val windowNs: Long = 300_000_000L) {
    private var startNs = 0L
    private var best: T? = null
    private var bestSharpness = 0.0

    /** Call for every frame: returns the held frame when [tNs] closes its window. */
    fun tick(tNs: Long): T? = if (best != null && tNs - startNs >= windowNs) flush() else null

    /** A candidate; [make] (the costly copy) only runs when it beats the one held. */
    fun offer(tNs: Long, sharpness: Double, make: () -> T) {
        if (best == null) startNs = tNs
        else if (sharpness <= bestSharpness) return
        best = make()
        bestSharpness = sharpness
    }

    fun flush(): T? = best.also { best = null }
}
