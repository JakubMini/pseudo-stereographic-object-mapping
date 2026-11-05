package com.jakubmini.psom.scan

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonElement
import kotlinx.serialization.json.JsonNull
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.put
import kotlinx.serialization.json.putJsonObject
import java.io.File

data class ProjectInfo(
    val dir: File,
    val status: String,
    val errorStage: String?,
    val error: String?,
    val keyframes: Int,
) {
    val id: String get() = dir.name
    val thumbnail: File get() = File(dir, "session/frames/000000.jpg")
}

/** project.json and the projects folder. The session itself is never edited here. */
object Projects {
    private val json = Json { prettyPrint = true }

    fun newProject(status: String) = buildJsonObject {
        put("format", "psom-project/1")
        put("status", status)
        put("last_error", JsonNull)
        put("crop", JsonNull)
        putJsonObject("scale") { put("factor", 1.0); put("a", JsonNull); put("b", JsonNull); put("known_mm", JsonNull) }
        put("runs", JsonArray(emptyList()))
    }

    fun read(project: File): JsonObject = try {
        Json.parseToJsonElement(File(project, "project.json").readText()).jsonObject
    } catch (e: Exception) {
        newProject("captured")
    }

    fun writeProject(project: File, j: JsonObject) = SessionWriter.atomicWrite(File(project, "project.json"), json.encodeToString(JsonObject.serializer(), j))

    /** Read, change some top-level fields, write back; everything else is kept. */
    fun update(project: File, change: MutableMap<String, JsonElement>.() -> Unit) {
        val m = read(project).toMutableMap()
        m.change()
        writeProject(project, JsonObject(m))
    }

    fun list(root: File): List<ProjectInfo> =
        (root.listFiles { f -> f.isDirectory && File(f, "session").isDirectory } ?: emptyArray())
            .sortedByDescending { it.name }
            .map { info(it) }

    fun info(dir: File): ProjectInfo {
        val p = read(dir)
        val err = p["last_error"] as? JsonObject
        val frames = File(dir, "session/frames.jsonl").takeIf { it.exists() }?.useLines { s -> s.count { it.isNotBlank() } } ?: 0
        return ProjectInfo(
            dir,
            (p["status"] as? JsonPrimitive)?.content ?: "captured",
            (err?.get("stage") as? JsonPrimitive)?.content,
            (err?.get("error") as? JsonPrimitive)?.content,
            frames,
        )
    }

    /** A project left "capturing" by a scan that never finished keeps its frames and becomes "captured". */
    fun recover(root: File, active: File?) {
        for (p in list(root))
            if (p.status == "capturing" && p.dir != active) update(p.dir) { put("status", JsonPrimitive("captured")) }
    }

    fun scaleFactor(project: File): Double =
        (read(project)["scale"] as? JsonObject)?.get("factor")?.jsonPrimitive?.doubleOrNull ?: 1.0

    fun setScale(project: File, a: DoubleArray, b: DoubleArray, knownMm: Double, factor: Double) = update(project) {
        put("scale", buildJsonObject {
            put("factor", factor)
            put("a", JsonArray(a.map { JsonPrimitive(it) }))
            put("b", JsonArray(b.map { JsonPrimitive(it) }))
            put("known_mm", knownMm)
        })
    }

    fun setCrop(project: File, crop: JsonObject?) = update(project) { put("crop", crop ?: JsonNull) }

    fun deleteProject(project: File) = project.deleteRecursively()

    /** The scan's raw data (session, project.json, report) as one zip, to send for a closer look. */
    fun zipScan(project: File, out: File): File {
        out.parentFile?.mkdirs()
        java.util.zip.ZipOutputStream(out.outputStream().buffered()).use { z ->
            val files = File(project, "session").walkTopDown().filter { it.isFile }.toList() +
                listOf(File(project, "project.json"), File(project, "recon/report.json")).filter { it.exists() }
            for (f in files) {
                z.putNextEntry(java.util.zip.ZipEntry(project.name + "/" + f.relativeTo(project).invariantSeparatorsPath))
                f.inputStream().use { it.copyTo(z) }
                z.closeEntry()
            }
        }
        return out
    }
}
