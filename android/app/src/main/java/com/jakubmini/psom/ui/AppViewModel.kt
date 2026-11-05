package com.jakubmini.psom.ui

import android.app.ActivityManager
import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.jakubmini.psom.engine.EngineBridge
import com.jakubmini.psom.engine.EngineResult
import com.jakubmini.psom.scan.Phase
import com.jakubmini.psom.scan.ProjectInfo
import com.jakubmini.psom.scan.Projects
import com.jakubmini.psom.scan.ScanState
import com.jakubmini.psom.scan.ScanStateMachine
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.jsonObject
import java.io.File

sealed interface Screen {
    data object Projects : Screen
    data object Scan : Screen
    data class Project(val dir: File) : Screen
}

data class Progress(val stage: String = "", val fraction: Double = 0.0)

class AppViewModel(app: Application) : AndroidViewModel(app) {
    val root = File(app.filesDir, "projects").apply { mkdirs() }
    val screen = MutableStateFlow<Screen>(Screen.Projects)
    val projects = MutableStateFlow<List<ProjectInfo>>(emptyList())
    val state = MutableStateFlow(ScanState(Phase.READY))
    val progress = MutableStateFlow(Progress())
    val lastResult = MutableStateFlow<EngineResult?>(null)
    /** Bumped whenever recon/ changes, so the review screen reloads the mesh and report. */
    val reconVersion = MutableStateFlow(0)
    private var machine = ScanStateMachine()
    private var busy = false

    /** The TSDF budget: a quarter of the phone's memory, at most 1.5 GB. */
    private val memMb: Int = run {
        val mi = ActivityManager.MemoryInfo()
        (app.getSystemService(ActivityManager::class.java)).getMemoryInfo(mi)
        (mi.totalMem / 4 / (1 shl 20)).toInt().coerceIn(256, 1536)
    }

    init {
        Projects.recover(root, active = null)
        refresh()
    }

    fun refresh() {
        projects.value = Projects.list(root)
    }

    fun openProjects() {
        refresh()
        machine.reset()
        state.value = machine.state
        screen.value = Screen.Projects
    }

    fun openScan() {
        machine.reset()
        state.value = machine.state
        screen.value = Screen.Scan
    }

    fun pressed() = machine.press().also { state.value = machine.state }

    fun released() = machine.release().also { state.value = machine.state }

    /** A scan just finished writing: reconstruct it. */
    fun captured(dir: File) {
        screen.value = Screen.Project(dir)
        reconstruct(dir)
    }

    fun open(p: ProjectInfo) {
        machine = ScanStateMachine(ScanStateMachine.resume(p.status, p.errorStage, p.error))
        state.value = machine.state
        lastResult.value = null
        screen.value = Screen.Project(p.dir)
        if (machine.state.phase == Phase.PROCESSING) reconstruct(p.dir)
    }

    fun reconstruct(dir: File) {
        if (busy) return
        if (machine.state.phase != Phase.PROCESSING) machine.reprocess()
        state.value = machine.state
        busy = true
        viewModelScope.launch {
            val r = withContext(Dispatchers.Default) {
                EngineBridge.runReconstruct(dir.path, memMb) { s, f -> progress.value = Progress(s, f) }
            }
            busy = false
            lastResult.value = r
            machine.processed(r.ok, r.stage, r.error)
            state.value = machine.state
            reconVersion.value++
            refresh()
        }
    }

    fun export(dir: File) {
        if (busy) return
        busy = true
        progress.value = Progress("export", 0.0)
        viewModelScope.launch {
            val r = withContext(Dispatchers.Default) {
                EngineBridge.runExport(dir.path) { s, f -> progress.value = Progress(s, f) }
            }
            busy = false
            lastResult.value = r
            machine.exported(r.ok, r.stage, r.error)
            state.value = machine.state
            reconVersion.value++
            refresh()
        }
    }

    fun setCrop(dir: File, crop: JsonObject?) {
        Projects.setCrop(dir, crop)
        reconstruct(dir)
    }

    fun setScale(dir: File, a: DoubleArray, b: DoubleArray, knownMm: Double) {
        val d = Math.sqrt((0..2).sumOf { (a[it] - b[it]) * (a[it] - b[it]) })
        if (d <= 0 || knownMm <= 0) return
        // the mesh is in ARCore metres; factor maps them onto the known distance
        Projects.setScale(dir, a, b, knownMm, knownMm / (d * 1000))
        reconVersion.value++
    }

    fun delete(p: ProjectInfo) {
        Projects.deleteProject(p.dir)
        refresh()
    }

    fun report(dir: File): JsonObject? = try {
        Json.parseToJsonElement(File(dir, "recon/report.json").readText()).jsonObject
    } catch (e: Exception) {
        null
    }
}
