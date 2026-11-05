package com.jakubmini.psom.ui

import android.content.Intent
import android.graphics.BitmapFactory
import android.net.Uri
import android.opengl.GLSurfaceView
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.ExtendedFloatingActionButton
import androidx.compose.material3.FilterChip
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.PointerEventPass
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.core.content.FileProvider
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import com.jakubmini.psom.gl.ColorMode
import com.jakubmini.psom.gl.Crop
import com.jakubmini.psom.gl.MeshData
import com.jakubmini.psom.gl.MeshView
import com.jakubmini.psom.gl.Tool
import com.jakubmini.psom.scan.ArCapture
import com.jakubmini.psom.scan.Phase
import com.jakubmini.psom.scan.ProjectInfo
import com.jakubmini.psom.scan.Projects
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.boolean
import kotlinx.serialization.json.double
import kotlinx.serialization.json.jsonPrimitive
import java.io.File
import kotlin.math.PI
import kotlin.math.sqrt

@Composable
fun App(vm: AppViewModel, capture: ArCapture) {
    val screen by vm.screen.collectAsState()
    MaterialTheme {
        when (val s = screen) {
            Screen.Projects -> ProjectsScreen(vm)
            Screen.Scan -> ScanScreen(vm, capture)
            is Screen.Project -> ProjectScreen(vm, s.dir)
        }
    }
}

// ---------- projects ----------

@Composable
fun ProjectsScreen(vm: AppViewModel) {
    val list by vm.projects.collectAsState()
    var deleting by remember { mutableStateOf<ProjectInfo?>(null) }
    Scaffold(floatingActionButton = { ExtendedFloatingActionButton(onClick = vm::openScan) { Text("New scan") } }) { pad ->
        Column(Modifier.padding(pad).padding(16.dp)) {
            Text("Scans", style = MaterialTheme.typography.headlineMedium)
            Spacer(Modifier.height(8.dp))
            if (list.isEmpty()) Text("No scans yet. Put an object on a table and tap New scan.")
            LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                items(list, key = { it.id }) { p ->
                    Card(Modifier.fillMaxWidth().clickable { vm.open(p) }) {
                        Row(Modifier.padding(8.dp), verticalAlignment = Alignment.CenterVertically) {
                            Thumb(p.thumbnail)
                            Column(Modifier.weight(1f).padding(horizontal = 12.dp)) {
                                Text(p.id, fontWeight = FontWeight.SemiBold)
                                Text("${statusLabel(p.status)} · ${p.keyframes} keyframes", style = MaterialTheme.typography.bodySmall)
                                p.error?.let { Text("Last attempt failed (${p.errorStage}): $it", color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall) }
                            }
                            TextButton(onClick = { deleting = p }) { Text("Delete") }
                        }
                    }
                }
            }
        }
    }
    deleting?.let { p ->
        AlertDialog(
            onDismissRequest = { deleting = null },
            title = { Text("Delete ${p.id}?") },
            text = { Text("The scan, its mesh and its STEP file are removed from the phone. This can't be undone.") },
            confirmButton = { TextButton(onClick = { vm.delete(p); deleting = null }) { Text("Delete") } },
            dismissButton = { TextButton(onClick = { deleting = null }) { Text("Keep") } },
        )
    }
}

private fun statusLabel(s: String) = when (s) {
    "capturing" -> "Scan interrupted"
    "captured" -> "Not reconstructed"
    "reconstructed" -> "Reconstructed"
    "exported" -> "STEP exported"
    else -> s
}

@Composable
private fun Thumb(f: File) {
    val bmp = remember(f) {
        if (f.exists()) BitmapFactory.decodeFile(f.path, BitmapFactory.Options().apply { inSampleSize = 8 })?.asImageBitmap() else null
    }
    Box(Modifier.size(64.dp).clip(MaterialTheme.shapes.small).background(Color.DarkGray)) {
        if (bmp != null) Image(bmp, contentDescription = null, contentScale = ContentScale.Crop, modifier = Modifier.fillMaxSize())
    }
}

// ---------- scan ----------

@Composable
fun ScanScreen(vm: AppViewModel, capture: ArCapture) {
    val ui by capture.ui.collectAsState()
    val ctx = LocalContext.current
    val owner = LocalLifecycleOwner.current
    var error by remember { mutableStateOf<String?>(null) }
    val glView = remember {
        GLSurfaceView(ctx).apply {
            preserveEGLContextOnPause = true
            setEGLContextClientVersion(3)
            setEGLConfigChooser(8, 8, 8, 8, 16, 0)
            setRenderer(capture)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
    }
    DisposableEffect(owner) {
        capture.onFinished = { dir -> vm.captured(dir) }
        val obs = LifecycleEventObserver { _, e ->
            when (e) {
                Lifecycle.Event.ON_RESUME -> { error = capture.resume(); glView.onResume() }
                Lifecycle.Event.ON_PAUSE -> { glView.onPause(); capture.pause() }
                else -> {}
            }
        }
        owner.lifecycle.addObserver(obs)
        if (owner.lifecycle.currentState.isAtLeast(Lifecycle.State.RESUMED)) { error = capture.resume(); glView.onResume() }
        onDispose {
            owner.lifecycle.removeObserver(obs)
            glView.onPause()
            capture.pause()
        }
    }
    Box(Modifier.fillMaxSize()) {
        AndroidView({ glView }, Modifier.fillMaxSize())
        Column(Modifier.fillMaxWidth().padding(16.dp)) {
            TextButton(onClick = vm::openProjects) { Text("‹ Scans", color = Color.White) }
            (error ?: ui.error)?.let { Banner(it, Color(0xFFB00020)) }
            if (!ui.tracking) Banner(trackingHint(ui.reason), Color(0xCC8A5A00))
            if (ui.scanning) Text("${ui.keyframes} keyframes" + if (ui.depth) " · depth on" else "", color = Color.White)
            else if (!ui.depth && ui.tracking) Text("This phone has no raw depth: the shape comes from the images alone. Keep the object well lit and textured.", color = Color.White, style = MaterialTheme.typography.bodySmall)
        }
        Column(Modifier.align(Alignment.BottomCenter).padding(bottom = 40.dp), horizontalAlignment = Alignment.CenterHorizontally) {
            CoverageRing(ui.coverage, Modifier.size(150.dp))
            Spacer(Modifier.height(16.dp))
            HoldButton(
                scanning = ui.scanning,
                onPress = { if (vm.pressed()) capture.startScan() },
                onRelease = { if (vm.released()) capture.stopScan() },
            )
            Text(if (ui.scanning) "Release to finish" else "Hold to scan", color = Color.White)
        }
    }
}

private fun trackingHint(reason: String?) = when (reason) {
    "INSUFFICIENT_LIGHT" -> "Tracking lost: too dark"
    "EXCESSIVE_MOTION" -> "Tracking lost: moving too fast"
    "INSUFFICIENT_FEATURES" -> "Tracking lost: point at something with more detail"
    "CAMERA_UNAVAILABLE" -> "Camera unavailable"
    else -> "Looking for the surroundings… move the phone slowly"
}

@Composable
private fun Banner(text: String, color: Color) {
    Text(text, color = Color.White, modifier = Modifier.fillMaxWidth().background(color, MaterialTheme.shapes.small).padding(8.dp))
    Spacer(Modifier.height(6.dp))
}

/**
 * Press starts, lifting the finger ends, even if it slid off the button. Events keep arriving
 * here after the pointer leaves the bounds, and cancellation also ends the scan.
 */
@Composable
fun HoldButton(scanning: Boolean, onPress: () -> Unit, onRelease: () -> Unit) {
    Box(
        Modifier.size(88.dp).clip(CircleShape).background(if (scanning) Color(0xFFE53935) else Color.White)
            .semantics { contentDescription = if (scanning) "Scanning. Release to finish" else "Hold to scan" }
            .pointerInput(Unit) {
                awaitEachGesture {
                    awaitFirstDown(requireUnconsumed = false)
                    onPress()
                    try {
                        do {
                            val e = awaitPointerEvent(PointerEventPass.Initial)
                        } while (e.changes.any { it.pressed })
                    } finally {
                        onRelease()
                    }
                }
            },
    )
}

/** 12 azimuth sectors in 3 rings (low, middle, top); filled = seen from there. */
@Composable
fun CoverageRing(bins: List<Boolean>, modifier: Modifier) {
    Canvas(modifier) {
        val c = Offset(size.width / 2, size.height / 2)
        val r = size.minDimension / 2
        for (e in 0 until 3) {
            val radius = r * (1f - e * 0.28f) - 8f
            for (a in 0 until 12) {
                val seen = bins.getOrElse(e * 12 + a) { false }
                drawArc(
                    color = if (seen) Color(0xFF4CAF50) else Color(0x66FFFFFF),
                    startAngle = a * 30f + 2f, sweepAngle = 26f, useCenter = false,
                    topLeft = Offset(c.x - radius, c.y - radius), size = Size(radius * 2, radius * 2),
                    style = Stroke(width = 14f),
                )
            }
        }
    }
}

// ---------- processing, review, export ----------

@Composable
fun ProjectScreen(vm: AppViewModel, dir: File) {
    val state by vm.state.collectAsState()
    val progress by vm.progress.collectAsState()
    val result by vm.lastResult.collectAsState()
    val version by vm.reconVersion.collectAsState()
    Column(Modifier.fillMaxSize().padding(16.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            TextButton(onClick = vm::openProjects) { Text("‹ Scans") }
            Text(dir.name, style = MaterialTheme.typography.titleMedium)
        }
        when (state.phase) {
            Phase.PROCESSING -> {
                Text("Reconstructing: ${progress.stage}")
                LinearProgressIndicator(progress = { progress.fraction.toFloat() }, modifier = Modifier.fillMaxWidth())
                Text("The scan is saved. If this fails you can retry without scanning again.", style = MaterialTheme.typography.bodySmall)
            }
            Phase.FAILED -> {
                Text("Reconstruction failed at ${state.stage}", color = MaterialTheme.colorScheme.error, fontWeight = FontWeight.SemiBold)
                Text(state.error ?: "")
                result?.warnings?.forEach { Text("• $it", style = MaterialTheme.typography.bodySmall) }
                Spacer(Modifier.height(8.dp))
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(onClick = { vm.reconstruct(dir) }) { Text("Retry") }
                    if (File(dir, "recon/mesh.ply").exists()) OutlinedButton(onClick = { vm.setCrop(dir, null) }) { Text("Retry with automatic crop") }
                }
            }
            else -> Review(vm, dir, version, state.phase == Phase.EXPORTED, state.error)
        }
    }
}

@Composable
private fun Review(vm: AppViewModel, dir: File, version: Int, exported: Boolean, exportError: String?) {
    val ctx = LocalContext.current
    val report = remember(version) { vm.report(dir) }
    val mesh = remember(version) { runCatching { MeshData.load(File(dir, "recon/mesh.ply")) }.getOrNull() }
    var tool by remember { mutableStateOf(Tool.ORBIT) }
    var color by remember { mutableStateOf(ColorMode.SHADED) }
    val autoCrop = remember(version) { Crop.fromJson(Projects.read(dir)["crop"] as? JsonObject) }
    var crop by remember(version) { mutableStateOf(autoCrop) }
    var cropEdited by remember(version) { mutableStateOf(false) }
    val picks = remember(version) { mutableStateListOf<DoubleArray>() }
    var askDistance by remember { mutableStateOf(false) }
    val factor = remember(version) { Projects.scaleFactor(dir) }
    val view = remember { MeshView(ctx) }

    LaunchedEffect(mesh) { mesh?.let { view.setMesh(it, color) } }
    LaunchedEffect(tool, crop) { view.tool = tool; view.setCrop(if (tool == Tool.CROP) crop else null) }
    LaunchedEffect(color) { view.setColorMode(color) }
    LaunchedEffect(picks.size) { view.setMarks(picks.toList()) }
    view.onCropChanged = { c -> crop = c; cropEdited = true }
    view.onPick = { p -> if (picks.size >= 2) picks.clear(); picks.add(p); if (picks.size == 2) askDistance = true }

    val saveLauncher = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("application/step")) { uri: Uri? ->
        if (uri != null) ctx.contentResolver.openOutputStream(uri)?.use { out -> File(dir, "recon/model.step").inputStream().use { it.copyTo(out) } }
    }

    if (mesh == null) {
        Text("No mesh yet.")
        Button(onClick = { vm.reconstruct(dir) }) { Text("Reconstruct") }
        return
    }
    Column {
        Dimensions(report, factor)
        Box(Modifier.fillMaxWidth().height(340.dp)) { AndroidView({ view }, Modifier.fillMaxSize()) }
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            FilterChip(tool == Tool.ORBIT, { tool = Tool.ORBIT }, { Text("View") })
            FilterChip(tool == Tool.CROP, { tool = Tool.CROP }, { Text("Crop") })
            FilterChip(tool == Tool.MEASURE, { tool = Tool.MEASURE; picks.clear() }, { Text("Set scale") })
        }
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            for ((m, label) in listOf(ColorMode.SHADED to "Shape", ColorMode.WEIGHT to "Observations", ColorMode.DIVERSITY to "View angles", ColorMode.UNCERTAINTY to "Uncertainty"))
                FilterChip(color == m, { color = m }, { Text(label, style = MaterialTheme.typography.labelSmall) })
        }
        if (color != ColorMode.SHADED) Text("Green is stronger, red weaker. These are separate signals, none of them a measured accuracy.", style = MaterialTheme.typography.bodySmall)
        when (tool) {
            Tool.CROP -> Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Text("Drag the orange handles to move each side.", style = MaterialTheme.typography.bodySmall, modifier = Modifier.weight(1f))
                Button(enabled = cropEdited, onClick = { crop?.let { vm.setCrop(dir, it.toJson()) } }) { Text("Apply") }
                OutlinedButton(onClick = { vm.setCrop(dir, null) }) { Text("Auto") }
            }
            Tool.MEASURE -> Text("Tap two points a known distance apart (${picks.size}/2).", style = MaterialTheme.typography.bodySmall)
            Tool.ORBIT -> {}
        }
        Column(Modifier.weight(1f).verticalScroll(rememberScrollState())) {
            QualityNotes(report)
            exportError?.let { Text("Export failed: $it", color = MaterialTheme.colorScheme.error) }
            Spacer(Modifier.height(8.dp))
            ExportPanel(vm, dir, report, exported, onSave = { saveLauncher.launch("${dir.name}.step") })
        }
    }
    if (askDistance && picks.size == 2) {
        val a = picks[0]
        val b = picks[1]
        val scanned = sqrt((0..2).sumOf { (a[it] - b[it]) * (a[it] - b[it]) }) * 1000 * factor
        var text by remember { mutableStateOf("") }
        AlertDialog(
            onDismissRequest = { askDistance = false },
            title = { Text("Set the scale") },
            text = {
                Column {
                    Text("Between the two points the scan measures %.1f mm. What is the real distance?".format(scanned))
                    OutlinedTextField(text, { text = it }, label = { Text("mm") }, singleLine = true)
                }
            },
            confirmButton = {
                TextButton(onClick = {
                    text.toDoubleOrNull()?.let { vm.setScale(dir, a, b, it) }
                    askDistance = false
                    tool = Tool.ORBIT
                }) { Text("Set") }
            },
            dismissButton = { TextButton(onClick = { askDistance = false }) { Text("Cancel") } },
        )
    }
}

@Composable
private fun Dimensions(report: JsonObject?, factor: Double) {
    val d = report?.get("dimensions_mm") as? JsonObject ?: return
    val atRun = d["scale_factor"]?.jsonPrimitive?.double ?: 1.0
    fun v(k: String) = (d[k]?.jsonPrimitive?.double ?: 0.0) / atRun * factor
    Text("%.1f × %.1f × %.1f mm".format(v("length"), v("width"), v("height")), style = MaterialTheme.typography.titleLarge)
    Text(if (factor == 1.0) "Scale from ARCore, not calibrated." else "Scale calibrated (×%.4f).".format(factor), style = MaterialTheme.typography.bodySmall)
}

@Composable
private fun QualityNotes(report: JsonObject?) {
    report ?: return
    val topo = report["topology"] as? JsonObject
    val closed = topo?.get("closed_valid")?.jsonPrimitive?.boolean == true
    Text(
        if (closed) "Closed surface: exports as a solid." else "Open surface: parts were not seen, so it exports as an open shell. Nothing is invented to fill them.",
        color = if (closed) Color(0xFF2E7D32) else Color(0xFFB26A00),
    )
    (report["warnings"] as? JsonArray)?.forEach { Text("• ${it.jsonPrimitive.content}", style = MaterialTheme.typography.bodySmall) }
}

@Composable
private fun ExportPanel(vm: AppViewModel, dir: File, report: JsonObject?, exported: Boolean, onSave: () -> Unit) {
    val ctx = LocalContext.current
    val step = File(dir, "recon/model.step")
    Text(
        "The STEP is faceted: every triangle becomes a flat face. It is valid CAD data with the scanned shape, not an editable parametric model.",
        style = MaterialTheme.typography.bodySmall,
    )
    Spacer(Modifier.height(6.dp))
    Button(onClick = { vm.export(dir) }) { Text(if (exported) "Export again" else "Export STEP") }
    val exp = report?.get("export") as? JsonObject
    if (exported && step.exists() && exp != null) {
        val solid = exp["solid"]?.jsonPrimitive?.boolean == true
        Text(if (solid) "Exported a solid." else "Exported an OPEN SHELL.", fontWeight = FontWeight.SemiBold)
        (exp["warnings"] as? JsonArray)?.forEach { Text("• ${it.jsonPrimitive.content}", style = MaterialTheme.typography.bodySmall) }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = {
                val uri = FileProvider.getUriForFile(ctx, ctx.packageName + ".files", step)
                val send = Intent(Intent.ACTION_SEND).setType("application/step").putExtra(Intent.EXTRA_STREAM, uri)
                    .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                ctx.startActivity(Intent.createChooser(send, "Share STEP"))
            }) { Text("Share") }
            OutlinedButton(onClick = onSave) { Text("Save to…") }
        }
    }
}
