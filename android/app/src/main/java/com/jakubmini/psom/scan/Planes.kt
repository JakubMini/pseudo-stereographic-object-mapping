package com.jakubmini.psom.scan

/**
 * A camera image's planes, bulk-copied out of the Image so it can be closed at once and the
 * frame processed off the camera thread. Strides are kept as the camera gave them.
 */
class YuvPlanes(
    val width: Int,
    val height: Int,
    val y: ByteArray, val yRow: Int,
    val u: ByteArray, val uRow: Int, val uPix: Int,
    val v: ByteArray, val vRow: Int, val vPix: Int,
) {
    /** NV21: the luma rows, then interleaved V/U at quarter resolution. */
    fun nv21(): ByteArray {
        val out = ByteArray(width * height * 3 / 2)
        var o = 0
        for (r in 0 until height) {
            System.arraycopy(y, r * yRow, out, o, width)
            o += width
        }
        for (r in 0 until height / 2) for (c in 0 until width / 2) {
            out[o++] = v[r * vRow + c * vPix]
            out[o++] = u[r * uRow + c * uPix]
        }
        return out
    }
}
