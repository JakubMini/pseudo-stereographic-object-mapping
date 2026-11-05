package com.jakubmini.psom.scan

import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.add
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put
import kotlinx.serialization.json.putJsonObject
import java.io.BufferedWriter
import java.io.File
import java.io.FileOutputStream
import java.io.OutputStreamWriter

/** What session.json records about the device and the camera (docs/session-format.md). */
data class SessionMeta(
    val createdUtc: String,
    val manufacturer: String,
    val model: String,
    val sensorOrientationDeg: Int,
    val imageWidth: Int,
    val imageHeight: Int,
)

class Keyframe(
    val tNs: Long,
    /** fx, fy, cx, cy in pixels of the stored image: a starting guess the engine refines. */
    val k: DoubleArray,
    val sharpness: Double,
    val jpeg: ByteArray,
    /** Up in device axes from the gravity sensor, m/s², or null if there was no reading yet. */
    val gravity: FloatArray? = null,
)

/**
 * Writes session/ as the scan happens: photos only, no poses (the engine works them out from the
 * images). A keyframe's photo is written before its frames.jsonl line, and every line is flushed,
 * so a process killed mid-scan leaves a valid prefix. Not thread-safe: call it from one thread.
 */
class SessionWriter(val project: File) {
    private val dir = File(project, "session")
    private var frames: BufferedWriter
    private var events: BufferedWriter
    private var imu: BufferedWriter
    private var next = 0
    val count get() = next

    init {
        File(dir, "frames").mkdirs()
        frames = open("frames.jsonl")
        events = open("events.jsonl")
        imu = open("imu.csv").apply { write("t_ns,sensor,x,y,z\n") }
        Projects.writeProject(project, Projects.newProject("capturing"))
    }

    private fun open(name: String) = BufferedWriter(OutputStreamWriter(FileOutputStream(File(dir, name), true), Charsets.UTF_8))

    fun meta(m: SessionMeta) {
        val j = buildJsonObject {
            put("format", "psom-session/1")
            put("created_utc", m.createdUtc)
            putJsonObject("device") { put("manufacturer", m.manufacturer); put("model", m.model) }
            putJsonObject("conventions") {
                put("pose", "none"); put("world_up", "+y")
                put("depth_units", "mm"); put("image_orientation", "sensor")
            }
            putJsonObject("camera") {
                put("sensor_orientation_deg", m.sensorOrientationDeg); put("display_rotation_deg", 0); put("intrinsics", "guess")
            }
            putJsonObject("image") { put("width", m.imageWidth); put("height", m.imageHeight) }
            putJsonObject("depth") { put("supported", false) }
        }
        atomicWrite(File(dir, "session.json"), j.toString())
    }

    fun event(tNs: Long, name: String, extra: JsonObject? = null) {
        val j = buildJsonObject {
            put("t_ns", tNs)
            put("event", name)
            extra?.forEach { (k, v) -> put(k, v) }
        }
        events.write(j.toString())
        events.write("\n")
        events.flush()
    }

    fun keyframe(f: Keyframe): Int {
        val i = next
        File(dir, "frames/%06d.jpg".format(i)).writeBytes(f.jpeg)
        val line = buildJsonObject {
            put("i", i)
            put("t_ns", f.tNs)
            put("K", buildJsonArray { f.k.forEach { add(it) } })
            put("sharpness", f.sharpness)
            put("depth", false)
            f.gravity?.let { g -> put("gravity", buildJsonArray { g.forEach { add(it) } }) }
        }
        frames.write(line.toString())
        frames.write("\n")
        frames.flush()
        next++
        return i
    }

    fun imu(tNs: Long, sensor: Char, x: Float, y: Float, z: Float) {
        imu.write("$tNs,$sensor,$x,$y,$z\n")
    }

    /** Ends the capture: closes the files and marks the project captured. */
    fun finish(tNs: Long, rejected: Map<String, Int>) {
        event(tNs, "capture_end", buildJsonObject { put("rejected", JsonObject(rejected.mapValues { JsonPrimitive(it.value) })) })
        for (w in listOf(frames, events, imu)) w.close()
        Projects.update(project) { put("status", JsonPrimitive("captured")) }
    }

    companion object {
        fun atomicWrite(f: File, text: String) {
            val tmp = File(f.path + ".tmp")
            tmp.writeText(text)
            if (!tmp.renameTo(f)) {
                f.delete()
                check(tmp.renameTo(f)) { "could not write ${f.path}" }
            }
        }
    }
}
