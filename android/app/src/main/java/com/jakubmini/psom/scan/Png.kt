package com.jakubmini.psom.scan

import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.util.zip.CRC32
import java.util.zip.DeflaterOutputStream

/** Greyscale PNG encoder: 8-bit (confidence) or 16-bit (depth in mm), which Android's Bitmap can't write. */
object Png {
    fun gray8(width: Int, height: Int, data: ByteArray): ByteArray {
        require(data.size == width * height)
        return encode(width, height, 8) { row, out -> out.write(data, row * width, width) }
    }

    fun gray16(width: Int, height: Int, data: ShortArray): ByteArray {
        require(data.size == width * height)
        val line = ByteArray(width * 2)
        return encode(width, height, 16) { row, out ->
            for (x in 0 until width) {
                val v = data[row * width + x].toInt()
                line[2 * x] = (v shr 8).toByte()  // PNG is big-endian
                line[2 * x + 1] = v.toByte()
            }
            out.write(line)
        }
    }

    private fun encode(w: Int, h: Int, depth: Int, row: (Int, java.io.OutputStream) -> Unit): ByteArray {
        val raw = ByteArrayOutputStream()
        DeflaterOutputStream(raw).use { z ->
            for (r in 0 until h) {
                z.write(0)  // filter: none
                row(r, z)
            }
        }
        val out = ByteArrayOutputStream()
        out.write(byteArrayOf(0x89.toByte(), 'P'.code.toByte(), 'N'.code.toByte(), 'G'.code.toByte(), 13, 10, 26, 10))
        chunk(out, "IHDR", ByteBuffer.allocate(13).putInt(w).putInt(h).put(depth.toByte()).put(0).put(0).put(0).put(0).array())
        chunk(out, "IDAT", raw.toByteArray())
        chunk(out, "IEND", ByteArray(0))
        return out.toByteArray()
    }

    private fun chunk(out: ByteArrayOutputStream, type: String, data: ByteArray) {
        val t = type.toByteArray(Charsets.US_ASCII)
        out.write(ByteBuffer.allocate(4).putInt(data.size).array())
        out.write(t)
        out.write(data)
        val crc = CRC32()
        crc.update(t)
        crc.update(data)
        out.write(ByteBuffer.allocate(4).putInt(crc.value.toInt()).array())
    }
}
