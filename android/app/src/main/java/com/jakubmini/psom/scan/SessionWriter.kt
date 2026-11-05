package com.jakubmini.psom.scan

import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.add
import kotlinx.serialization.json.buildJsonArray
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put
import kotlinx.serialization.json.putJsonArray
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
    val arcore: String,
    val sensorOrientationDeg: Int,
    val displayRotationDeg: Int,
    val imageWidth: Int,
    val imageHeight: Int,
    val depthSupported: Boolean,
    val depthWidth: Int,
    val depthHeight: Int,
    val maxSkewNs: Long,
    val sensorMinusArcoreNs: Long,
    val target: DoubleArray?,
)

class Keyframe(
    val tNs: Long,
    /** T_world_cam, row-major, ARCore's GL camera convention. */
    val pose: DoubleArray,
    /** fx, fy, cx, cy in pixels of the stored image. */
    val k: DoubleArray,
    val sharpness: Double,
    val jpeg: ByteArray,
    val depthMm: ShortArray? = null,
    val confidence: ByteArray? = null,
    val depthWidth: Int = 0,
    val depthHeight: Int = 0,
    val depthTNs: Long = 0,
    /** x, y, z, confidence per point, world metres. */
    val points: FloatArray = FloatArray(0),
    val pointIds: IntArray = IntArray(0),
    /** Pose of the camera frame the depth belongs to (row-major, GL), when it is older than this one. */
    val depthPose: DoubleArray? = null,
)

/**
 * Writes session/ as the scan happens. A keyframe's files are written before its frames.jsonl
 * line, and every line is flushed, so a process killed mid-scan leaves a valid prefix.
 * Not thread-safe: call it from one thread.
 */
class SessionWriter(val project: File) {
    private val dir = File(project, "session")
    private var frames: BufferedWriter
    private var events: BufferedWriter
    private var points: BufferedWriter
    private var imu: BufferedWriter
    private var next = 0
    val count get() = next

    init {
        for (sub in listOf("frames", "depth", "conf")) File(dir, sub).mkdirs()
        frames = open("frames.jsonl")
        events = open("events.jsonl")
        points = open("points.csv").apply { write("t_ns,id,x,y,z,confidence\n") }
        imu = open("imu.csv").apply { write("t_ns,sensor,x,y,z\n") }
        Projects.writeProject(project, Projects.newProject("capturing"))
    }

    private fun open(name: String) = BufferedWriter(OutputStreamWriter(FileOutputStream(File(dir, name), true), Charsets.UTF_8))

    fun meta(m: SessionMeta) {
        val j = buildJsonObject {
            put("format", "psom-session/1")
            put("created_utc", m.createdUtc)
            putJsonObject("device") { put("manufacturer", m.manufacturer); put("model", m.model); put("arcore", m.arcore) }
            putJsonObject("conventions") {
                put("pose", "arcore_gl_world_from_camera"); put("world_up", "+y")
                put("depth_units", "mm"); put("image_orientation", "sensor")
            }
            putJsonObject("camera") { put("sensor_orientation_deg", m.sensorOrientationDeg); put("display_rotation_deg", m.displayRotationDeg) }
            putJsonObject("image") { put("width", m.imageWidth); put("height", m.imageHeight) }
            putJsonObject("depth") {
                put("supported", m.depthSupported); put("width", m.depthWidth); put("height", m.depthHeight); put("max_skew_ns", m.maxSkewNs)
            }
            putJsonObject("clocks") { put("sensor_minus_arcore_ns", m.sensorMinusArcoreNs) }
            if (m.target != null) putJsonArray("target") { m.target.forEach { add(it) } } else put("target", null as String?)
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
        val stem = "%06d".format(i)
        File(dir, "frames/$stem.jpg").writeBytes(f.jpeg)
        val withDepth = f.depthMm != null && f.confidence != null
        if (withDepth) {
            File(dir, "depth/$stem.png").writeBytes(Png.gray16(f.depthWidth, f.depthHeight, f.depthMm!!))
            File(dir, "conf/$stem.png").writeBytes(Png.gray8(f.depthWidth, f.depthHeight, f.confidence!!))
        }
        for (p in f.pointIds.indices) {
            points.write("${f.tNs},${f.pointIds[p]},${f.points[4 * p]},${f.points[4 * p + 1]},${f.points[4 * p + 2]},${f.points[4 * p + 3]}\n")
        }
        points.flush()
        val line = buildJsonObject {
            put("i", i)
            put("t_ns", f.tNs)
            put("pose", buildJsonArray { f.pose.forEach { add(it) } })
            put("K", buildJsonArray { f.k.forEach { add(it) } })
            put("tracking", "tracking")
            put("sharpness", f.sharpness)
            put("depth", withDepth)
            if (withDepth) put("depth_t_ns", f.depthTNs)
            if (withDepth && f.depthPose != null) put("depth_pose", buildJsonArray { f.depthPose.forEach { add(it) } })
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
        for (w in listOf(frames, events, points, imu)) w.close()
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
