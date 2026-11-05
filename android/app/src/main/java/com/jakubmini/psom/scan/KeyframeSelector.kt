package com.jakubmini.psom.scan

import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.min
import kotlin.math.sqrt

/** Camera pose: translation in metres and a unit quaternion (x, y, z, w). */
data class Pose(val tx: Double, val ty: Double, val tz: Double, val qx: Double, val qy: Double, val qz: Double, val qw: Double)

enum class Verdict(val key: String) { ACCEPTED("accepted"), TRACKING("tracking"), STILL("motion"), BLUR("blur"), RATE("rate"), JUMP("jump") }

/**
 * Keeps a frame when tracking is good, the camera moved enough since the last keyframe, the image
 * is sharp compared with the recent ones, and the rate stays under [maxRateHz].
 */
class KeyframeSelector(
    private val minTranslation: Double = 0.02,
    private val minRotationDeg: Double = 5.0,
    private val sharpFactor: Double = 0.6,
    private val maxRateHz: Double = 3.0,
    /** Faster than a hand moves: ARCore has lost track and the pose jumped. */
    private val maxSpeed: Double = 0.5,
) {
    private var last: Pose? = null
    private var lastT = Long.MIN_VALUE
    private val recent = ArrayDeque<Double>()
    val rejected = mutableMapOf<String, Int>()
    var accepted = 0
        private set
    /** When the current run of jumps began, or -1. */
    var jumpingSince = -1L
        private set

    /** Cheap checks first, so the caller only computes sharpness for frames that could be kept. */
    fun wouldConsider(tNs: Long, tracking: Boolean, pose: Pose): Verdict {
        if (!tracking) return Verdict.TRACKING
        val l = last
        if (l != null) {
            if (moved(l, pose) < minTranslation && turned(l, pose) < minRotationDeg) return Verdict.STILL
            if ((tNs - lastT) < 1e9 / maxRateHz) return Verdict.RATE
            val speed = moved(l, pose) / ((tNs - lastT) / 1e9)
            // a second of jumps: ARCore has settled somewhere new, so start a new steady stretch there
            if (speed > maxSpeed && (jumpingSince < 0 || tNs - jumpingSince < 1_000_000_000L)) return Verdict.JUMP
        }
        return Verdict.ACCEPTED
    }

    fun offer(tNs: Long, tracking: Boolean, pose: Pose, sharpness: Double): Verdict {
        var v = wouldConsider(tNs, tracking, pose)
        if (v == Verdict.ACCEPTED && recent.size >= 5 && sharpness < median() * sharpFactor) v = Verdict.BLUR
        if (v == Verdict.ACCEPTED || v == Verdict.BLUR) {
            recent.addLast(sharpness)
            if (recent.size > 30) recent.removeFirst()
        }
        if (v == Verdict.JUMP) {
            if (jumpingSince < 0) jumpingSince = tNs
        } else if (v == Verdict.ACCEPTED) {
            jumpingSince = -1
        }
        if (v == Verdict.ACCEPTED) {
            last = pose
            lastT = tNs
            accepted++
        } else if (v != Verdict.STILL) {
            rejected[v.key] = (rejected[v.key] ?: 0) + 1
        }
        return v
    }

    private fun median(): Double = recent.sorted()[recent.size / 2]

    companion object {
        fun moved(a: Pose, b: Pose): Double {
            val dx = a.tx - b.tx
            val dy = a.ty - b.ty
            val dz = a.tz - b.tz
            return sqrt(dx * dx + dy * dy + dz * dz)
        }

        fun turned(a: Pose, b: Pose): Double {
            val dot = abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw)
            return Math.toDegrees(2 * acos(min(1.0, dot)))
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
