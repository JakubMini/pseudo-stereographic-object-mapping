package com.jakubmini.psom.scan

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.rules.TemporaryFolder
import java.io.File

class SessionWriterTest {
    @get:Rule val tmp = TemporaryFolder()

    private fun meta() = SessionMeta("2026-11-05T10:00:00Z", "samsung", "SM-A566B", 90, 1600, 1200)

    private fun frame(t: Long) = Keyframe(
        tNs = t,
        k = doubleArrayOf(1300.0, 1300.0, 799.5, 599.5),
        sharpness = 50.0,
        jpeg = byteArrayOf(1, 2, 3),
        gravity = floatArrayOf(0.1f, 9.7f, 1.2f),
    )

    @Test fun writesAPhotoSessionAndMarksTheProject() {
        val p = File(tmp.root, "p1")
        val w = SessionWriter(p)
        assertEquals("capturing", Projects.info(p).status)
        w.meta(meta())
        w.event(1, "capture_start")
        w.keyframe(frame(10))
        w.keyframe(frame(20).let { Keyframe(it.tNs, it.k, it.sharpness, it.jpeg, null) })
        w.imu(15, 'g', 0.1f, 0.2f, 0.3f)
        w.finish(30, mapOf("blur" to 2))

        val s = File(p, "session")
        val meta = Json.parseToJsonElement(File(s, "session.json").readText()).jsonObject
        assertEquals("none", meta["conventions"]!!.jsonObject["pose"]!!.jsonPrimitive.content)
        assertEquals("guess", meta["camera"]!!.jsonObject["intrinsics"]!!.jsonPrimitive.content)
        val lines = File(s, "frames.jsonl").readLines()
        assertEquals(2, lines.size)
        val l0 = Json.parseToJsonElement(lines[0]).jsonObject
        assertEquals(null, l0["pose"])
        assertEquals("9.7", l0["gravity"]!!.jsonArray[1].jsonPrimitive.content)
        assertEquals(null, Json.parseToJsonElement(lines[1]).jsonObject["gravity"])
        assertTrue(File(s, "frames/000001.jpg").exists())
        assertTrue(File(s, "events.jsonl").readText().contains("capture_end"))
        assertEquals(2, File(s, "imu.csv").readLines().size)
        assertEquals("captured", Projects.info(p).status)
        assertEquals(2, Projects.info(p).keyframes)
    }

    @Test fun anAbandonedCaptureIsRecoveredOnLaunch() {
        val root = File(tmp.root, "projects")
        val p = File(root, "p2")
        val w = SessionWriter(p)
        w.meta(meta())
        w.keyframe(frame(10))
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

    @Test fun scaleIsSetOnlyOnceMeasured() {
        val p = File(tmp.root, "p5")
        SessionWriter(p).finish(1, emptyMap())
        assertFalse(Projects.scaleSet(p))
        Projects.setScale(p, doubleArrayOf(0.0, 0.0, 0.0), doubleArrayOf(0.1, 0.0, 0.0), 110.0, 1.1)
        assertTrue(Projects.scaleSet(p))
    }

    @Test fun scansZip() {
        val p = File(tmp.root, "p4")
        val w = SessionWriter(p)
        w.meta(meta())
        w.keyframe(frame(10))
        w.finish(20, emptyMap())
        val zip = Projects.zipScan(p, File(tmp.root, "out/p4.zip"))
        val names = java.util.zip.ZipFile(zip).use { z -> z.entries().toList().map { it.name } }
        assertTrue(names.contains("p4/session/frames.jsonl"))
        assertTrue(names.contains("p4/session/frames/000000.jpg"))
        assertTrue(names.contains("p4/project.json"))
    }
}
