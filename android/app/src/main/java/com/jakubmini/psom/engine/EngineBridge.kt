package com.jakubmini.psom.engine

import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.boolean

/** What every engine call returns. ok is only true once the engine has checked its output. */
data class EngineResult(val ok: Boolean, val stage: String, val error: String, val warnings: List<String>) {
    companion object {
        fun parse(s: String): EngineResult = try {
            val j = Json.parseToJsonElement(s).jsonObject
            EngineResult(
                j["ok"]!!.jsonPrimitive.boolean,
                j["stage"]?.jsonPrimitive?.content ?: "",
                j["error"]?.jsonPrimitive?.content ?: "",
                (j["warnings"] as? JsonArray)?.map { it.jsonPrimitive.content } ?: emptyList(),
            )
        } catch (e: Exception) {
            EngineResult(false, "engine", "unreadable engine result: $s", emptyList())
        }
    }
}

/** The C++ engine (engine/), built into libpsom.so. Calls block: run them off the main thread. */
object EngineBridge {
    init {
        System.loadLibrary("psom")
    }

    fun interface ProgressListener {
        fun onProgress(stage: String, fraction: Double)
    }

    private external fun reconstruct(projectDir: String, memMb: Int, listener: ProgressListener?): String
    private external fun export(projectDir: String, listener: ProgressListener?): String

    fun runReconstruct(projectDir: String, memMb: Int, listener: ProgressListener?) = EngineResult.parse(reconstruct(projectDir, memMb, listener))
    fun runExport(projectDir: String, listener: ProgressListener?) = EngineResult.parse(export(projectDir, listener))
}
