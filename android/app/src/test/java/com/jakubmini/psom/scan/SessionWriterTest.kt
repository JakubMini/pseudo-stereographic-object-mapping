package com.jakubmini.psom.scan

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.nio.ByteBuffer
import java.io.File
import java.util.zip.Inflater

class SessionWriterTest {
    @get:Rule val tmp = TemporaryFolder()

    private fun meta() = SessionMeta("2026-11-05T10:00:00Z", "Google", "Pixel", "1.57", 90, 0, 640, 480, true, 160, 120, 16_666_666, 0, null)

    private fun frame(t: Long, depth: Boolean) = Keyframe(
        tNs = t,
        pose = doubleArrayOf(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        k = doubleArrayOf(500.0, 500.0, 320.0, 240.0),
        sharpness = 50.0,
        jpeg = byteArrayOf(1, 2, 3),
        depthMm = if (depth) ShortArray(160 * 120) { 1234 } else null,
        confidence = if (depth) ByteArray(160 * 120) { 255.toByte() } else null,
        depthWidth = 160, depthHeight = 120, depthTNs = t,
        points = floatArrayOf(0.1f, 0.2f, 0.3f, 0.9f), pointIds = intArrayOf(7),
    )

    @Test fun writesTheContractAndMarksTheProject() {
        val p = File(tmp.root, "p1")
        val w = SessionWriter(p)
        assertEquals("capturing", Projects.info(p).status)
        w.meta(meta())
        w.event(1, "capture_start")
        w.keyframe(frame(10, true))
        w.keyframe(frame(20, false))
        w.finish(30, mapOf("blur" to 2))

        val s = File(p, "session")
        val meta = Json.parseToJsonElement(File(s, "session.json").readText()).jsonObject
        assertEquals("arcore_gl_world_from_camera", meta["conventions"]!!.jsonObject["pose"]!!.jsonPrimitive.content)
        val lines = File(s, "frames.jsonl").readLines()
        assertEquals(2, lines.size)
        val l0 = Json.parseToJsonElement(lines[0]).jsonObject
        assertEquals("true", l0["depth"]!!.jsonPrimitive.content)
        assertEquals("false", Json.parseToJsonElement(lines[1]).jsonObject["depth"]!!.jsonPrimitive.content)
        assertTrue(File(s, "frames/000001.jpg").exists())
        assertTrue(File(s, "events.jsonl").readText().contains("capture_end"))
        assertEquals(3, File(s, "points.csv").readLines().size)  // header + one point per keyframe
        assertEquals("captured", Projects.info(p).status)
        assertEquals(2, Projects.info(p).keyframes)
    }

    /** Minimal reader for what Png writes: IHDR fields and the inflated scanlines. */
    private fun decode(png: ByteArray): Triple<Int, Int, ByteArray> {
        val b = ByteBuffer.wrap(png)
        b.position(8)
        var w = 0
        var h = 0
        val idat = java.io.ByteArrayOutputStream()
        while (b.remaining() > 0) {
            val len = b.int
            val type = ByteArray(4).also { b.get(it) }.toString(Charsets.US_ASCII)
            val data = ByteArray(len).also { b.get(it) }
            b.int  // crc
            if (type == "IHDR") ByteBuffer.wrap(data).let { w = it.int; h = it.int }
            if (type == "IDAT") idat.write(data)
        }
        val inf = Inflater().apply { setInput(idat.toByteArray()) }
        val out = ByteArray(1 shl 20)
        val n = inf.inflate(out)
        return Triple(w, h, out.copyOf(n))
    }

    @Test fun depthPngRoundTrips() {
        val data = ShortArray(160 * 120) { (it % 4000).toShort() }
        val png = Png.gray16(160, 120, data)
        assertEquals(0x89.toByte(), png[0])
        val (w, h, rows) = decode(png)
        assertEquals(160, w)
        assertEquals(120, h)
        assertEquals(120 * (1 + 160 * 2), rows.size)
        fun at(i: Int): Int {
            val o = (i / 160) * (1 + 320) + 1 + (i % 160) * 2
            return ((rows[o].toInt() and 0xFF) shl 8) or (rows[o + 1].toInt() and 0xFF)
        }
        assertEquals(1234, at(1234))
        assertEquals(3999, at(3999))
        val (_, _, c) = decode(Png.gray8(4, 2, byteArrayOf(0, 64, -128, -1, 1, 2, 3, 4)))
        assertEquals(255, c[4].toInt() and 0xFF)
    }

    @Test fun anAbandonedCaptureIsRecoveredOnLaunch() {
        val root = File(tmp.root, "projects")
        val p = File(root, "p2")
        val w = SessionWriter(p)
        w.meta(meta())
        w.keyframe(frame(10, false))
        // the app dies here: no finish()
        Projects.recover(root, active = null)
        assertEquals("captured", Projects.info(p).status)
    }

    @Test fun scaleAndCropEditsKeepTheRest() {
        val p = File(tmp.root, "p3")
        SessionWriter(p).finish(1, emptyMap())
        Projects.setScale(p, doubleArrayOf(0.0, 0.0, 0.0), doubleArrayOf(0.1, 0.0, 0.0), 110.0, 1.1)
        assertEquals(1.1, Projects.scaleFactor(p), 1e-12)
        assertEquals("captured", Projects.info(p).status)
    }

    @Test fun lateDepthCarriesItsPoseAndScansZip() {
        val p = File(tmp.root, "p4")
        val w = SessionWriter(p)
        w.meta(meta())
        val late = DoubleArray(16) { if (it % 5 == 0) 1.0 else 0.0 }.also { it[3] = 0.5 }
        w.keyframe(Keyframe(10, frame(10, true).pose, doubleArrayOf(500.0, 500.0, 320.0, 240.0), 50.0, byteArrayOf(1),
            ShortArray(160 * 120), ByteArray(160 * 120), 160, 120, 5, depthPose = late))
        w.finish(20, emptyMap())
        val line = Json.parseToJsonElement(File(p, "session/frames.jsonl").readLines()[0]).jsonObject
        assertEquals("0.5", line["depth_pose"]!!.jsonArray[3].jsonPrimitive.content)

        val zip = Projects.zipScan(p, File(tmp.root, "out/p4.zip"))
        val names = java.util.zip.ZipFile(zip).use { z -> z.entries().toList().map { it.name } }
        assertTrue(names.contains("p4/session/frames.jsonl"))
        assertTrue(names.contains("p4/session/depth/000000.png"))
        assertTrue(names.contains("p4/project.json"))
    }
}
