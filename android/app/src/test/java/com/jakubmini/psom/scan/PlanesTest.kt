package com.jakubmini.psom.scan

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

class PlanesTest {
    @Test fun nv21HonoursRowPaddingAndInterleavedChroma() {
        // 4x2 image, luma rows padded to 6 bytes; chroma interleaved (pixel stride 2), row stride 4
        val y = byteArrayOf(1, 2, 3, 4, 0, 0, 5, 6, 7, 8, 0, 0)
        val u = byteArrayOf(10, 0, 11, 0)
        val v = byteArrayOf(20, 0, 21, 0)
        val p = YuvPlanes(4, 2, y, 6, u, 4, 2, v, 4, 2)
        assertArrayEquals(byteArrayOf(1, 2, 3, 4, 5, 6, 7, 8, 20, 10, 21, 11), p.nv21())
    }

    @Test fun depthIsLittleEndianAndSkipsRowPadding() {
        // 2x2 samples, rows padded to 6 bytes: 1000 mm, 2, 513, 65535 (as unsigned)
        val b = byteArrayOf(0xE8.toByte(), 0x03, 2, 0, 9, 9, 1, 2, 0xFF.toByte(), 0xFF.toByte(), 9, 9)
        val s = DepthPlane(2, 2, b, 6, 2).shorts()
        assertEquals(1000, s[0].toInt())
        assertEquals(2, s[1].toInt())
        assertEquals(513, s[2].toInt())
        assertEquals(65535, s[3].toInt() and 0xFFFF)
    }

    @Test fun confidenceSkipsRowPadding() {
        val b = byteArrayOf(1, 2, 0, 3, 4, 0)
        assertArrayEquals(byteArrayOf(1, 2, 3, 4), DepthPlane(2, 2, b, 3, 1).bytes())
    }
}
