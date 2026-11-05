package com.jakubmini.psom.scan

import org.junit.Assert.assertArrayEquals
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
}
