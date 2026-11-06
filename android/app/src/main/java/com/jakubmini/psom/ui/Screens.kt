package com.jakubmini.psom.ui

import android.content.Intent
import android.graphics.BitmapFactory
import android.graphics.Matrix
import android.net.Uri
import android.text.format.DateUtils
import androidx.activity.compose.BackHandler
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.ExperimentalFoundationApi
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.combinedClickable
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.navigationBarsPadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.itemsIndexed
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.BasicTextField
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.DrawScope
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.PointerEventPass
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalLifecycleOwner
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.role
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
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
import androidx.camera.view.PreviewView
import com.jakubmini.psom.scan.CameraCapture
import com.jakubmini.psom.scan.Phase
import com.jakubmini.psom.scan.ProjectInfo
import com.jakubmini.psom.scan.Projects
import kotlinx.coroutines.launch
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.boolean
import kotlinx.serialization.json.double
import kotlinx.serialization.json.int
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import java.io.File
import java.text.SimpleDateFormat
import java.util.Locale
import kotlin.math.sqrt

private val Warn = Color(0xFF9A4A00)  // warning text on cream (mustard itself is too light for text)
private val Paper = Color(0xFFFFFBF3) // raised surfaces: the sheet, the selected segment

@Composable
fun App(vm: AppViewModel, capture: CameraCapture) {
    val screen by vm.screen.collectAsState()
    PsomTheme {
        Box(Modifier.fillMaxSize().background(Palette.Cream)) {
            when (val s = screen) {
                Screen.Projects -> ProjectsScreen(vm)
                Screen.Scan -> ScanScreen(vm, capture)
                is Screen.Project -> ProjectScreen(vm, s.dir)
            }
        }
    }
}

// ---------- pieces ----------

@Composable
fun PrimaryButton(text: String, modifier: Modifier = Modifier, enabled: Boolean = true, icon: (@Composable () -> Unit)? = null, onClick: () -> Unit) {
    Row(
        modifier.height(56.dp).clip(RoundedCornerShape(28.dp))
            .background(if (enabled) Palette.Ink else Palette.Ink.copy(alpha = 0.3f))
            .clickable(enabled = enabled, role = Role.Button, onClick = onClick)
            .padding(horizontal = 28.dp),
        horizontalArrangement = Arrangement.spacedBy(10.dp, Alignment.CenterHorizontally),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        icon?.invoke()
        Text(text, style = Type.BodyStrong, color = Palette.Cream)
    }
}

@Composable
fun SecondaryButton(text: String, modifier: Modifier = Modifier, onClick: () -> Unit) {
    Box(
        modifier.height(56.dp).clip(RoundedCornerShape(28.dp)).background(Palette.Wash)
            .clickable(role = Role.Button, onClick = onClick),
        contentAlignment = Alignment.Center,
    ) { Text(text, style = Type.BodyStrong, color = Palette.Ink) }
}

@Composable
fun QuietButton(text: String, onClick: () -> Unit) {
    Box(
        Modifier.height(44.dp).clip(RoundedCornerShape(22.dp)).clickable(role = Role.Button, onClick = onClick).padding(horizontal = 16.dp),
        contentAlignment = Alignment.Center,
    ) { Text(text, style = Type.Body, color = Palette.Ink) }
}

@Composable
private fun BackLink(onClick: () -> Unit) {
    Row(
        Modifier.height(44.dp).clip(RoundedCornerShape(22.dp)).clickable(role = Role.Button, onClick = onClick).padding(horizontal = 8.dp),
        verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(6.dp),
    ) {
        Canvas(Modifier.size(10.dp, 16.dp)) {
            val w = size.width
            val h = size.height
            drawLine(Palette.Ink, Offset(w * 0.85f, h * 0.1f), Offset(w * 0.2f, h * 0.5f), 1.8.dp.toPx(), StrokeCap.Round)
            drawLine(Palette.Ink, Offset(w * 0.2f, h * 0.5f), Offset(w * 0.85f, h * 0.9f), 1.8.dp.toPx(), StrokeCap.Round)
        }
        Text("Scans", style = Type.Body, color = Palette.Ink)
    }
}

@Composable
private fun Hairline(start: Dp = 0.dp) = Box(Modifier.padding(start = start).fillMaxWidth().height(1.dp).background(Palette.Line))

@Composable
private fun Dot(filled: Boolean, color: Color) {
    Canvas(Modifier.size(10.dp)) {
        if (filled) drawCircle(color) else drawCircle(color, radius = size.minDimension / 2 - 1.dp.toPx(), style = Stroke(1.6.dp.toPx()))
    }
}

@Composable
private fun Segmented(options: List<String>, selected: Int, onSelect: (Int) -> Unit, modifier: Modifier = Modifier) {
    Row(
        modifier.height(40.dp).clip(RoundedCornerShape(20.dp)).background(Palette.Wash).padding(4.dp),
        horizontalArrangement = Arrangement.spacedBy(4.dp),
    ) {
        options.forEachIndexed { i, label ->
            val on = i == selected
            Box(
                Modifier.weight(1f).fillMaxSize().clip(RoundedCornerShape(16.dp))
                    .background(if (on) Paper else Color.Transparent)
                    .clickable(role = Role.Tab) { onSelect(i) },
                contentAlignment = Alignment.Center,
            ) { Text(label, style = if (on) Type.SmallStrong else Type.Small, color = if (on) Palette.Ink else Palette.Ink2) }
        }
    }
}

/** The 36 viewing directions as three rings of arcs: outside = low views, inside = from above. */
private fun DrawScope.coverageArcs(bins: List<Boolean>, unlit: Color, litAlpha: Float = 1f) {
    val c = center
    val unit = size.minDimension / 140f
    val radii = floatArrayOf(62f, 53f, 44f)
    for (ring in 0..2) {
        val r = radii[ring] * unit
        for (i in 0 until 12) {
            val lit = bins.getOrElse(ring * 12 + i) { false }
            drawArc(
                brush = if (lit) Palette.Glow else SolidColor(unlit),
                startAngle = -90f + i * 30f + 4f, sweepAngle = 22f, useCenter = false,
                topLeft = Offset(c.x - r, c.y - r), size = Size(2 * r, 2 * r),
                alpha = if (lit) litAlpha else 1f,
                style = Stroke(5 * unit, cap = StrokeCap.Round),
            )
        }
    }
}

// ---------- scans ----------

private data class Summary(val dims: String?, val solid: Boolean?, val exported: Boolean)

private fun summary(dir: File): Summary = try {
    val r = Json.parseToJsonElement(File(dir, "recon/report.json").readText()).jsonObject
    val d = r["dimensions_mm"] as JsonObject
    val f = Projects.scaleFactor(dir) / d["scale_factor"]!!.jsonPrimitive.double
    val dims = if (!scaleKnown(r, dir)) "Size not set" else "%.1f × %.1f × %.1f mm".format(
        d["length"]!!.jsonPrimitive.double * f, d["width"]!!.jsonPrimitive.double * f, d["height"]!!.jsonPrimitive.double * f,
    )
    Summary(dims, (r["topology"] as JsonObject)["closed_valid"]!!.jsonPrimitive.boolean, File(dir, "recon/model.step").exists())
} catch (e: Exception) {
    Summary(null, null, false)
}

/** A photo scan's size is a guess until the user calibrates it. */
private fun scaleKnown(report: JsonObject?, dir: File): Boolean =
    (report?.get("dimensions_mm") as? JsonObject)?.get("scale_known")?.jsonPrimitive?.boolean != false || Projects.scaleSet(dir)

private fun dateTitle(id: String): String {
    val t = try { SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).parse(id)?.time } catch (e: Exception) { null } ?: return id
    val hm = SimpleDateFormat("HH:mm", Locale.getDefault()).format(t)
    return when {
        DateUtils.isToday(t) -> "Today, $hm"
        DateUtils.isToday(t + DateUtils.DAY_IN_MILLIS) -> "Yesterday, $hm"
        else -> SimpleDateFormat("EEE d MMM, HH:mm", Locale.getDefault()).format(t)
    }
}

@OptIn(ExperimentalFoundationApi::class)
@Composable
fun ProjectsScreen(vm: AppViewModel) {
    val list by vm.projects.collectAsState()
    var deleting by remember { mutableStateOf<ProjectInfo?>(null) }
    Box(Modifier.fillMaxSize().background(Palette.Cream)) {
        LazyColumn(Modifier.fillMaxSize(), contentPadding = PaddingValues(top = 72.dp, bottom = 168.dp)) {
            item {
                Text("Scans", style = Type.Title, color = Palette.Ink, modifier = Modifier.padding(horizontal = 24.dp))
                Spacer(Modifier.height(24.dp))
            }
            if (list.isEmpty()) item {
                Text("No scans yet", style = Type.Body, color = Palette.Ink2, modifier = Modifier.padding(horizontal = 24.dp))
            }
            itemsIndexed(list, key = { _, p -> p.id }) { i, p ->
                if (i > 0) Hairline(start = 104.dp)
                val s = remember(p.dir, p.status) { summary(p.dir) }
                Row(
                    Modifier.fillMaxWidth().combinedClickable(onClick = { vm.open(p) }, onLongClick = { deleting = p })
                        .padding(horizontal = 24.dp, vertical = 12.dp),
                    verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(16.dp),
                ) {
                    Thumb(p)
                    Column(Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                        Text(dateTitle(p.id), style = Type.Body, color = Palette.Ink)
                        Text(s.dims ?: "${p.keyframes} photos", style = if (s.dims != null) Type.SmallMono else Type.Small, color = Palette.Ink2)
                        val (filled, color, label) = when {
                            p.error != null && p.status == "captured" -> Triple(false, Palette.Mustard, "Needs another try")
                            s.solid == true -> Triple(true, Palette.Sea, if (s.exported) "Solid · exported" else "Solid")
                            s.solid == false -> Triple(false, Palette.Mustard, if (s.exported) "Open shell · exported" else "Open shell")
                            else -> Triple(false, Palette.Ink2, "Not built")
                        }
                        Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                            Dot(filled, color)
                            Text(label, style = Type.Small, color = Palette.Ink2)
                        }
                    }
                }
            }
        }
        Box(Modifier.align(Alignment.BottomCenter).fillMaxWidth().height(168.dp).background(Brush.verticalGradient(listOf(Palette.Cream.copy(alpha = 0f), Palette.Cream, Palette.Cream))))
        PrimaryButton("New scan", Modifier.align(Alignment.BottomCenter).navigationBarsPadding().padding(bottom = 32.dp), icon = { ViewfinderIcon(Palette.Cream) }, onClick = vm::openScan)
    }
    deleting?.let { p ->
        AlertDialog(
            onDismissRequest = { deleting = null },
            title = { Text("Delete this scan?", style = Type.BodyStrong) },
            text = { Text("${dateTitle(p.id)}. This can't be undone.", style = Type.Body) },
            confirmButton = { TextButton(onClick = { vm.delete(p); deleting = null }) { Text("Delete", color = Warn) } },
            dismissButton = { TextButton(onClick = { deleting = null }) { Text("Keep") } },
        )
    }
}

@Composable
private fun ViewfinderIcon(color: Color) {
    Canvas(Modifier.size(20.dp)) {
        val s = size.width
        val w = 1.8.dp.toPx()
        val k = s * 0.3f
        for ((x, y) in listOf(0f to 0f, s to 0f, s to s, 0f to s)) {
            val dx = if (x == 0f) k else -k
            val dy = if (y == 0f) k else -k
            drawLine(color, Offset(x, y), Offset(x + dx, y), w, StrokeCap.Round)
            drawLine(color, Offset(x, y), Offset(x, y + dy), w, StrokeCap.Round)
        }
        drawCircle(color, s * 0.15f)
    }
}

@Composable
private fun Thumb(p: ProjectInfo) {
    val bmp = remember(p.thumbnail) {
        if (!p.thumbnail.exists()) null else runCatching {
            val b = BitmapFactory.decodeFile(p.thumbnail.path, BitmapFactory.Options().apply { inSampleSize = 8 })
            // frames are stored in sensor orientation; turn the thumbnail upright
            val deg = runCatching {
                Json.parseToJsonElement(File(p.dir, "session/session.json").readText()).jsonObject["camera"]!!
                    .jsonObject["sensor_orientation_deg"]!!.jsonPrimitive.int
            }.getOrDefault(0)
            android.graphics.Bitmap.createBitmap(b, 0, 0, b.width, b.height, Matrix().apply { postRotate(deg.toFloat()) }, true).asImageBitmap()
        }.getOrNull()
    }
    Box(Modifier.size(64.dp).clip(RoundedCornerShape(16.dp)).background(Palette.Wash)) {
        if (bmp != null) Image(bmp, contentDescription = null, contentScale = ContentScale.Crop, modifier = Modifier.fillMaxSize())
    }
}

// ---------- scanner ----------

@Composable
fun ScanScreen(vm: AppViewModel, capture: CameraCapture) {
    val ui by capture.ui.collectAsState()
    val ctx = LocalContext.current
    val owner = LocalLifecycleOwner.current
    val preview = remember { PreviewView(ctx) }
    DisposableEffect(owner) {
        capture.onFinished = { dir -> vm.captured(dir) }
        // bound again on every resume: the first try may come before the camera permission
        val obs = LifecycleEventObserver { _, e ->
            if (e == Lifecycle.Event.ON_RESUME) capture.bind(owner, preview)
            if (e == Lifecycle.Event.ON_PAUSE) capture.pause()
        }
        owner.lifecycle.addObserver(obs)
        onDispose {
            owner.lifecycle.removeObserver(obs)
            capture.unbind()
        }
    }
    BackHandler { vm.openProjects() }

    val bins = ui.coverage
    val low = bins.subList(0, 12).count { it }
    val mid = bins.subList(12, 24).count { it }
    val top = bins.subList(24, 36).count { it }
    val hint = when {
        !ui.scanning -> "Hold and walk around it"
        ui.fast -> "Slow down"
        low < 9 -> "Keep going round"
        mid < 6 -> "Now a little higher"
        top < 3 -> "Now a few from above"
        else -> "Lift to finish"
    }

    Box(Modifier.fillMaxSize().background(Palette.Night)) {
        AndroidView({ preview }, Modifier.fillMaxSize())
        Box(Modifier.fillMaxSize().background(Brush.verticalGradient(0f to Color(0x66000000), 0.25f to Color.Transparent, 0.7f to Color.Transparent, 1f to Color(0x99000000))))

        Box(Modifier.fillMaxWidth().statusBarsPadding().padding(16.dp)) {
            if (!ui.scanning) {
                Box(
                    Modifier.size(44.dp).clip(CircleShape).background(Color.White.copy(alpha = 0.14f))
                        .clickable(role = Role.Button, onClick = vm::openProjects).semantics { contentDescription = "Close scanner" },
                    contentAlignment = Alignment.Center,
                ) {
                    Canvas(Modifier.size(14.dp)) {
                        drawLine(Color.White, Offset.Zero, Offset(size.width, size.height), 1.8.dp.toPx(), StrokeCap.Round)
                        drawLine(Color.White, Offset(size.width, 0f), Offset(0f, size.height), 1.8.dp.toPx(), StrokeCap.Round)
                    }
                }
            } else {
                Pill(Modifier.align(Alignment.Center), Color.White.copy(alpha = 0.14f)) {
                    Canvas(Modifier.size(8.dp)) { drawCircle(Palette.Terracotta) }
                    val sec = ui.elapsedMs / 1000
                    Text("${ui.keyframes} photos · %d:%02d".format(sec / 60, sec % 60), style = Type.SmallMono, color = Color.White)
                }
            }
        }
        ui.error?.let {
            Text(it, style = Type.Body, color = Palette.Ink, modifier = Modifier.align(Alignment.Center).padding(24.dp)
                .clip(RoundedCornerShape(16.dp)).background(Palette.Cream).padding(16.dp))
        }

        Column(
            Modifier.align(Alignment.BottomCenter).navigationBarsPadding().padding(bottom = 32.dp),
            horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(24.dp),
        ) {
            Text(hint, style = Type.Body, color = Color.White.copy(alpha = 0.9f), textAlign = TextAlign.Center)
            Shutter(
                bins = bins, scanning = ui.scanning, lost = ui.fast,
                onPress = { if (vm.pressed()) capture.startScan() },
                onRelease = { if (vm.released()) capture.stopScan() },
            )
        }
    }
}

@Composable
private fun Pill(modifier: Modifier, color: Color, content: @Composable () -> Unit) {
    Row(
        modifier.height(36.dp).clip(RoundedCornerShape(18.dp)).background(color).padding(horizontal = 16.dp),
        verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp),
    ) { content() }
}

/**
 * Press starts, lifting the finger ends, even if it slid off the shutter: events keep coming here
 * after the pointer leaves the bounds, and a cancelled gesture also ends the scan.
 */
@Composable
fun Shutter(bins: List<Boolean>, scanning: Boolean, lost: Boolean, onPress: () -> Unit, onRelease: () -> Unit) {
    Box(
        Modifier.size(140.dp)
            .semantics { contentDescription = if (scanning) "Scanning. Lift to finish" else "Hold to scan"; role = Role.Button }
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
    ) {
        if (scanning && !lost) Box(Modifier.align(Alignment.Center).size(120.dp).clip(CircleShape).background(Brush.radialGradient(listOf(Palette.Terracotta.copy(alpha = 0.35f), Color.Transparent))))
        Canvas(Modifier.fillMaxSize()) {
            coverageArcs(bins, Color.White.copy(alpha = 0.18f), if (lost) 0.45f else 1f)
            val u = size.minDimension / 140f
            if (!scanning) {
                drawCircle(Color.White, 33 * u)
            } else {
                drawCircle(Color.White.copy(alpha = if (lost) 0.5f else 0.9f), 33 * u, style = Stroke(2 * u))
                drawRoundRect(
                    brush = if (lost) SolidColor(Color.White.copy(alpha = 0.35f)) else Palette.Glow,
                    topLeft = Offset(center.x - 15 * u, center.y - 15 * u), size = Size(30 * u, 30 * u), cornerRadius = CornerRadius(9 * u),
                )
            }
        }
    }
}

// ---------- a project ----------

/** Zips the scan's raw data off the main thread and opens the share sheet with it. */
private fun shareScan(ctx: android.content.Context, scope: kotlinx.coroutines.CoroutineScope, dir: File) {
    scope.launch {
        val zip = kotlinx.coroutines.withContext(kotlinx.coroutines.Dispatchers.IO) {
            Projects.zipScan(dir, File(ctx.cacheDir, "exports/scan-${dir.name}.zip"))
        }
        val uri = FileProvider.getUriForFile(ctx, ctx.packageName + ".files", zip)
        val send = Intent(Intent.ACTION_SEND).setType("application/zip").putExtra(Intent.EXTRA_STREAM, uri)
            .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        ctx.startActivity(Intent.createChooser(send, null))
    }
}

@Composable
fun ProjectScreen(vm: AppViewModel, dir: File) {
    val state by vm.state.collectAsState()
    BackHandler { vm.openProjects() }
    when (state.phase) {
        Phase.PROCESSING -> BuildingScreen(vm, dir)
        Phase.FAILED -> FailedScreen(vm, dir, state.stage, state.error)
        else -> ReviewScreen(vm, dir, state.phase == Phase.EXPORTED, state.error)
    }
}

@Composable
private fun BuildingScreen(vm: AppViewModel, dir: File) {
    val progress by vm.progress.collectAsState()
    val frames = remember(dir) { Projects.info(dir).keyframes }
    val steps = listOf(
        "Placing $frames photos" to setOf("", "load", "poses"),
        "Depth" to setOf("depth"),
        "Finding the object" to setOf("isolate"),
        "Fusing the surfaces" to setOf("fusion"),
        "Filling the gaps" to setOf("guess"),
        "Closing and checking" to setOf("mesh", "repair", "report", "done"),
    )
    val current = steps.indexOfFirst { progress.stage in it.second }.coerceAtLeast(0)
    val spin = rememberInfiniteTransition(label = "build")
    val head by spin.animateFloat(0f, 36f, infiniteRepeatable(tween(2400, easing = LinearEasing), RepeatMode.Restart), label = "head")

    Column(Modifier.fillMaxSize().background(Palette.Cream).statusBarsPadding()) {
        Box(Modifier.padding(16.dp)) { BackLink(vm::openProjects) }
        Box(Modifier.fillMaxWidth().height(280.dp), contentAlignment = Alignment.Center) {
            Canvas(Modifier.size(140.dp)) {
                val lit = List(36) { i -> ((head.toInt() - i) % 36 + 36) % 36 < 9 }
                coverageArcs(lit, Palette.Line)
            }
        }
        Column(Modifier.padding(horizontal = 24.dp)) {
            Text("Building", style = Type.Title, color = Palette.Ink)
            Spacer(Modifier.height(32.dp))
            steps.forEachIndexed { i, (label, _) ->
                val done = i < current
                val now = i == current
                Row(Modifier.padding(vertical = 8.dp), verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                    Canvas(Modifier.size(20.dp)) {
                        when {
                            done -> {
                                drawCircle(Palette.Ink)
                                val w = 1.8.dp.toPx()
                                drawLine(Palette.Cream, Offset(size.width * 0.3f, size.height * 0.52f), Offset(size.width * 0.44f, size.height * 0.66f), w, StrokeCap.Round)
                                drawLine(Palette.Cream, Offset(size.width * 0.44f, size.height * 0.66f), Offset(size.width * 0.72f, size.height * 0.36f), w, StrokeCap.Round)
                            }
                            now -> {
                                val w = 3.dp.toPx()
                                drawCircle(Palette.Line, size.minDimension / 2 - w / 2, style = Stroke(w))
                                drawArc(
                                    Palette.Glow, -90f, 360f * progress.fraction.toFloat().coerceIn(0.08f, 1f), false,
                                    topLeft = Offset(w / 2, w / 2), size = Size(size.width - w, size.height - w),
                                    style = Stroke(w, cap = StrokeCap.Round),
                                )
                            }
                            else -> drawCircle(Palette.Line, size.minDimension / 2 - 1.dp.toPx(), style = Stroke(1.6.dp.toPx()))
                        }
                    }
                    Text(label, style = if (now) Type.BodyStrong else Type.Body, color = if (i > current) Palette.Ink2 else Palette.Ink, modifier = Modifier.weight(1f))
                    if (now && progress.fraction > 0) Text("${(progress.fraction * 100).toInt()}%", style = Type.SmallMono, color = Palette.Ink2)
                }
            }
        }
    }
}

private fun stageName(s: String?) = when (s) {
    "load" -> "Reading the scan"
    "poses" -> "Placing the photos"
    "depth" -> "Depth"
    "isolate" -> "Finding the object"
    "fusion" -> "Fusing"
    "guess" -> "Filling the gaps"
    "mesh", "repair" -> "Meshing"
    "export" -> "Export"
    else -> s ?: "Unknown"
}

@Composable
private fun FailedScreen(vm: AppViewModel, dir: File, stage: String?, error: String?) {
    val ctx = LocalContext.current
    val scope = rememberCoroutineScope()
    val result by vm.lastResult.collectAsState()
    val notes = result?.warnings.orEmpty()
    var showNotes by remember { mutableStateOf(false) }
    val frames = remember(dir) { Projects.info(dir).keyframes }
    val userCrop = remember(dir) { (Projects.read(dir)["crop"] as? JsonObject)?.get("source")?.jsonPrimitive?.content == "user" }
    Column(Modifier.fillMaxSize().background(Palette.Cream).statusBarsPadding().navigationBarsPadding()) {
        Box(Modifier.padding(16.dp)) { BackLink(vm::openProjects) }
        Column(Modifier.weight(1f).padding(horizontal = 24.dp).verticalScroll(rememberScrollState())) {
            Spacer(Modifier.height(64.dp))
            Canvas(Modifier.size(56.dp)) {
                val w = 2.4.dp.toPx()
                drawCircle(Palette.Mustard, size.minDimension / 2 - w, style = Stroke(w))
                drawLine(Palette.Ink, Offset(center.x, size.height * 0.3f), Offset(center.x, size.height * 0.56f), 2.8.dp.toPx(), StrokeCap.Round)
                drawCircle(Palette.Ink, 1.6.dp.toPx(), Offset(center.x, size.height * 0.69f))
            }
            Spacer(Modifier.height(24.dp))
            Text("Couldn't build this one", style = Type.Title, color = Palette.Ink)
            Spacer(Modifier.height(16.dp))
            Text((error ?: "").replaceFirstChar { it.uppercase() }, style = Type.Body, color = Palette.Ink)
            Spacer(Modifier.height(32.dp))
            Hairline()
            FactRow("Stopped at", stageName(stage))
            Hairline()
            FactRow("Your scan", "Saved, $frames photos")
            Hairline()
            if (notes.isNotEmpty()) {
                Text(
                    if (showNotes) "Hide notes" else "${notes.size} note${if (notes.size > 1) "s" else ""}", style = Type.SmallStrong, color = Palette.Ink,
                    modifier = Modifier.clickable(role = Role.Button) { showNotes = !showNotes }.padding(vertical = 12.dp),
                )
                if (showNotes) notes.forEach { Text(it, style = Type.Small, color = Palette.Ink2, modifier = Modifier.padding(bottom = 6.dp)) }
            }
        }
        Column(Modifier.fillMaxWidth().padding(24.dp), horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(8.dp)) {
            PrimaryButton("Try again", Modifier.fillMaxWidth()) { vm.reconstruct(dir) }
            if (userCrop) QuietButton("Back to automatic crop") { vm.setCrop(dir, null) }
            Row {
                QuietButton("Scan again", vm::openScan)
                QuietButton("Share scan files") { shareScan(ctx, scope, dir) }
            }
        }
    }
}

@Composable
private fun FactRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth().padding(vertical = 12.dp), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, style = Type.Body, color = Palette.Ink2)
        Text(value, style = Type.Body, color = Palette.Ink)
    }
}

@Composable
private fun ReviewScreen(vm: AppViewModel, dir: File, exported: Boolean, error: String?) {
    val ctx = LocalContext.current
    val version by vm.reconVersion.collectAsState()
    val sheet by vm.sheet.collectAsState()
    val exporting by vm.exporting.collectAsState()
    val report = remember(version) { vm.report(dir) }
    val mesh = remember(version) { runCatching { MeshData.load(File(dir, "recon/mesh.ply")) }.getOrNull() }
    var tool by remember { mutableStateOf(Tool.ORBIT) }
    var color by remember { mutableStateOf(ColorMode.SHADED) }
    var crop by remember(version) { mutableStateOf(Crop.fromJson(Projects.read(dir)["crop"] as? JsonObject)) }
    var cropEdited by remember(version) { mutableStateOf(false) }
    val picks = remember(version) { mutableStateListOf<DoubleArray>() }
    val factor = remember(version) { Projects.scaleFactor(dir) }
    val scaleKnown = remember(version) { scaleKnown(report, dir) }
    var menu by remember { mutableStateOf(false) }
    var confirmDelete by remember { mutableStateOf(false) }
    val scope = rememberCoroutineScope()
    val view = remember { MeshView(ctx) }

    LaunchedEffect(mesh) { mesh?.let { view.setMesh(it, color) } }
    LaunchedEffect(tool, crop) { view.tool = tool; view.setCrop(if (tool == Tool.CROP) crop else null) }
    LaunchedEffect(color) { view.setColorMode(color) }
    LaunchedEffect(picks.size, tool) { view.setMarks(if (tool == Tool.MEASURE) picks.toList() else emptyList()) }
    view.onCropChanged = { c -> crop = c; cropEdited = true }
    view.onPick = { p -> if (picks.size >= 2) picks.clear(); picks.add(p) }

    if (mesh == null) {
        Column(Modifier.fillMaxSize().statusBarsPadding().padding(24.dp)) {
            BackLink(vm::openProjects)
            Spacer(Modifier.height(24.dp))
            PrimaryButton("Build", Modifier.fillMaxWidth()) { vm.reconstruct(dir) }
        }
        return
    }

    Column(Modifier.fillMaxSize().background(Palette.Cream)) {
        Box(Modifier.fillMaxWidth().height(468.dp)) {
            AndroidView({ view }, Modifier.fillMaxSize())
            Row(Modifier.fillMaxWidth().statusBarsPadding().padding(16.dp), horizontalArrangement = Arrangement.SpaceBetween) {
                BackLink(vm::openProjects)
                Box {
                    Box(
                        Modifier.size(44.dp).clip(CircleShape).clickable(role = Role.Button) { menu = true }.semantics { contentDescription = "More" },
                        contentAlignment = Alignment.Center,
                    ) {
                        Canvas(Modifier.size(20.dp, 4.dp)) { for (k in 0..2) drawCircle(Palette.Ink, 2.dp.toPx(), Offset(size.width * (0.1f + 0.4f * k), size.height / 2)) }
                    }
                    DropdownMenu(menu, { menu = false }, containerColor = Paper) {
                        DropdownMenuItem({ Text("Rebuild", style = Type.Body) }, { menu = false; vm.reconstruct(dir) })
                        DropdownMenuItem({ Text("Share scan files", style = Type.Body) }, { menu = false; shareScan(ctx, scope, dir) })
                        DropdownMenuItem({ Text("Delete scan", style = Type.Body, color = Warn) }, { menu = false; confirmDelete = true })
                    }
                }
            }
            Segmented(
                listOf("Inspect", "Crop", "Scale"), tool.ordinal,
                { tool = Tool.entries[it]; if (tool == Tool.MEASURE) picks.clear() },
                Modifier.align(Alignment.BottomCenter).padding(bottom = 24.dp).width(264.dp),
            )
        }
        Column(Modifier.weight(1f).fillMaxWidth().padding(horizontal = 24.dp).navigationBarsPadding()) {
            when (tool) {
                Tool.ORBIT -> {
                    InspectPanel(report, factor, scaleKnown, color, { color = it }, { tool = Tool.MEASURE; picks.clear() })
                    Spacer(Modifier.weight(1f))
                    error?.let { Text(it, style = Type.Small, color = Warn, modifier = Modifier.padding(bottom = 8.dp)) }
                    val step = File(dir, "recon/model.step")
                    PrimaryButton(
                        when { exporting -> "Exporting…"; !scaleKnown -> "Set the scale"; exported && step.exists() -> "Share STEP"; else -> "Export STEP" },
                        Modifier.fillMaxWidth().padding(bottom = 24.dp), enabled = !exporting,
                    ) {
                        when {
                            !scaleKnown -> { tool = Tool.MEASURE; picks.clear() }
                            exported && step.exists() -> vm.openSheet()
                            else -> vm.export(dir)
                        }
                    }
                }
                Tool.CROP -> CropPanel(cropEdited, onApply = { crop?.let { vm.setCrop(dir, it.toJson()) } }, onAuto = { vm.setCrop(dir, null) })
                Tool.MEASURE -> ScalePanel(picks, factor, onSet = { a, b, mm -> vm.setScale(dir, a, b, mm); tool = Tool.ORBIT }, onAgain = { picks.clear() })
            }
        }
    }

    if (sheet) ExportSheet(vm, dir, report)
    if (confirmDelete) AlertDialog(
        onDismissRequest = { confirmDelete = false },
        title = { Text("Delete this scan?", style = Type.BodyStrong) },
        text = { Text("This can't be undone.", style = Type.Body) },
        confirmButton = { TextButton(onClick = { confirmDelete = false; vm.delete(dir) }) { Text("Delete", color = Warn) } },
        dismissButton = { TextButton(onClick = { confirmDelete = false }) { Text("Keep") } },
    )
}

@Composable
private fun ColumnScope.InspectPanel(report: JsonObject?, factor: Double, scaleKnown: Boolean, color: ColorMode, onColor: (ColorMode) -> Unit, onCalibrate: () -> Unit) {
    val d = report?.get("dimensions_mm") as? JsonObject
    val atRun = d?.get("scale_factor")?.jsonPrimitive?.double ?: 1.0
    fun v(k: String) = (d?.get(k)?.jsonPrimitive?.double ?: 0.0) / atRun * factor
    val scaled = scaleKnown
    val closed = (report?.get("topology") as? JsonObject)?.get("closed_valid")?.jsonPrimitive?.boolean == true
    val guessed = (report?.get("guess") as? JsonObject)?.get("fraction")?.jsonPrimitive?.double ?: 0.0
    val notes = (report?.get("warnings") as? JsonArray)?.map { it.jsonPrimitive.content } ?: emptyList()
    var showNotes by remember { mutableStateOf(false) }

    Spacer(Modifier.height(24.dp))
    Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
        for ((label, key) in listOf("Length" to "length", "Width" to "width", "Height" to "height"))
            Column(Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Text(label, style = Type.Small, color = Palette.Ink2)
                Text(if (scaled) "%.1f".format(v(key)) else "–", style = Type.Number, color = if (scaled) Palette.Ink else Palette.Ink2)
            }
    }
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text(if (scaled) "mm · calibrated" else "Size not set", style = Type.Small, color = Palette.Ink2)
        Spacer(Modifier.width(8.dp))
        Text(if (scaled) "Recalibrate" else "Set the scale", style = Type.SmallStrong, color = Palette.Ink, modifier = Modifier.clickable(role = Role.Button, onClick = onCalibrate).padding(vertical = 12.dp))
    }
    Hairline()
    Row(Modifier.padding(top = 16.dp), verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        Dot(closed, if (closed && guessed < 0.01) Palette.Sea else Palette.Mustard)
        Text(
            when {
                !closed -> "Open shell: parts weren't seen"
                guessed >= 0.01 -> "Solid · %.0f %% guessed, in yellow".format(guessed * 100)
                else -> "Solid"
            },
            style = Type.Body, color = Palette.Ink, modifier = Modifier.weight(1f),
        )
        if (notes.isNotEmpty())
            Text(
                if (showNotes) "Hide notes" else "${notes.size} note${if (notes.size > 1) "s" else ""}", style = Type.SmallStrong, color = Palette.Ink,
                modifier = Modifier.clickable(role = Role.Button) { showNotes = !showNotes }.padding(vertical = 8.dp),
            )
    }
    if (showNotes) Column(Modifier.padding(top = 8.dp), verticalArrangement = Arrangement.spacedBy(6.dp)) {
        notes.forEach { Text(it, style = Type.Small, color = Palette.Ink2) }
    }
    Row(Modifier.padding(top = 16.dp), horizontalArrangement = Arrangement.spacedBy(16.dp)) {
        for ((m, label) in listOf(ColorMode.SHADED to "Shape", ColorMode.WEIGHT to "Times seen", ColorMode.DIVERSITY to "View angles", ColorMode.UNCERTAINTY to "Uncertainty")) {
            val on = m == color
            Column(Modifier.clickable(role = Role.Tab) { onColor(m) }.padding(vertical = 4.dp), horizontalAlignment = Alignment.CenterHorizontally) {
                Text(label, style = if (on) Type.SmallStrong else Type.Small, color = if (on) Palette.Ink else Palette.Ink2)
                Box(Modifier.padding(top = 4.dp).width(24.dp).height(2.dp).background(if (on) Palette.Ink else Color.Transparent))
            }
        }
    }
}

@Composable
private fun ColumnScope.CropPanel(edited: Boolean, onApply: () -> Unit, onAuto: () -> Unit) {
    Spacer(Modifier.height(24.dp))
    Text("Keep only the object", style = Type.BodyStrong, color = Palette.Ink)
    Spacer(Modifier.weight(1f))
    Column(Modifier.fillMaxWidth().padding(bottom = 24.dp), horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(8.dp)) {
        PrimaryButton("Rebuild with this crop", Modifier.fillMaxWidth(), enabled = edited, onClick = onApply)
        QuietButton("Back to automatic", onAuto)
    }
}

@Composable
private fun ColumnScope.ScalePanel(picks: List<DoubleArray>, factor: Double, onSet: (DoubleArray, DoubleArray, Double) -> Unit, onAgain: () -> Unit) {
    var text by remember(picks.size) { mutableStateOf("") }
    Spacer(Modifier.height(24.dp))
    if (picks.size < 2) {
        Text("Tap two points you can measure", style = Type.BodyStrong, color = Palette.Ink)
        Text("${picks.size} of 2", style = Type.Small, color = Palette.Ink2, modifier = Modifier.padding(top = 4.dp))
        return
    }
    val a = picks[0]
    val b = picks[1]
    val metres = sqrt((0..2).sumOf { (a[it] - b[it]) * (a[it] - b[it]) })
    val mm = text.replace(',', '.').toDoubleOrNull()?.takeIf { it > 0 }
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text("As scanned  ", style = Type.Small, color = Palette.Ink2)
        Text("%.1f mm".format(metres * 1000 * factor), style = Type.SmallMono, color = Palette.Ink)
    }
    Spacer(Modifier.height(16.dp))
    Text("Real distance", style = Type.Small, color = Palette.Ink2)
    Row(Modifier.fillMaxWidth().padding(top = 4.dp), verticalAlignment = Alignment.Bottom) {
        BasicTextField(
            text, { text = it }, singleLine = true, textStyle = Type.Number.copy(color = Palette.Ink),
            cursorBrush = SolidColor(Palette.Terracotta),
            keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Decimal),
            modifier = Modifier.weight(1f).semantics { contentDescription = "Real distance in millimetres" },
        )
        Text("mm", style = Type.Body, color = Palette.Ink2, modifier = Modifier.padding(bottom = 6.dp))
    }
    Box(Modifier.fillMaxWidth().height(2.dp).background(Palette.Ink))
    if (mm != null) Text("×%.3f".format(mm / (metres * 1000)), style = Type.SmallMono, color = Palette.Ink2, modifier = Modifier.padding(top = 8.dp))
    Spacer(Modifier.weight(1f))
    Column(Modifier.fillMaxWidth().padding(bottom = 24.dp), horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(8.dp)) {
        PrimaryButton("Set scale", Modifier.fillMaxWidth(), enabled = mm != null) { mm?.let { onSet(a, b, it) } }
        QuietButton("Pick again", onAgain)
    }
}

@Composable
private fun ExportSheet(vm: AppViewModel, dir: File, report: JsonObject?) {
    val ctx = LocalContext.current
    val step = File(dir, "recon/model.step")
    val exp = report?.get("export") as? JsonObject
    val solid = exp?.get("solid")?.jsonPrimitive?.boolean == true
    val faces = exp?.get("faces")?.jsonPrimitive?.int ?: 0
    val save = rememberLauncherForActivityResult(ActivityResultContracts.CreateDocument("application/step")) { uri: Uri? ->
        if (uri != null) ctx.contentResolver.openOutputStream(uri)?.use { out -> step.inputStream().use { it.copyTo(out) } }
    }
    BackHandler { vm.closeSheet() }
    Box(Modifier.fillMaxSize()) {
        Box(
            Modifier.fillMaxSize().background(Palette.Ink.copy(alpha = 0.32f))
                .clickable(interactionSource = remember { MutableInteractionSource() }, indication = null, onClick = vm::closeSheet),
        )
        Column(
            Modifier.align(Alignment.BottomCenter).fillMaxWidth().clip(RoundedCornerShape(topStart = 32.dp, topEnd = 32.dp))
                .background(Paper).navigationBarsPadding().padding(start = 24.dp, end = 24.dp, top = 12.dp, bottom = 32.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            Box(Modifier.width(36.dp).height(4.dp).clip(RoundedCornerShape(2.dp)).background(Palette.Line))
            Spacer(Modifier.height(32.dp))
            Box(Modifier.size(120.dp), contentAlignment = Alignment.Center) {
                Box(Modifier.size(110.dp).clip(CircleShape).background(Brush.radialGradient(listOf(Palette.Terracotta.copy(alpha = 0.25f), Color.Transparent))))
                Canvas(Modifier.fillMaxSize()) {
                    coverageArcs(List(36) { true }, Palette.Line)
                    val u = size.minDimension / 140f
                    val w = 5 * u
                    drawLine(Palette.Ink, Offset(54 * u, 71 * u), Offset(65 * u, 82 * u), w, StrokeCap.Round)
                    drawLine(Palette.Ink, Offset(65 * u, 82 * u), Offset(87 * u, 58 * u), w, StrokeCap.Round)
                }
            }
            Spacer(Modifier.height(24.dp))
            Text("STEP ready", style = Type.Title, color = Palette.Ink)
            Spacer(Modifier.height(8.dp))
            Text(
                "%s · %,d faces · %.1f MB".format(if (solid) "Solid" else "Open shell", faces, step.length() / 1_048_576.0),
                style = Type.SmallMono, color = Palette.Ink2,
            )
            Text("Faceted, not parametric", style = Type.Small, color = Palette.Ink2, modifier = Modifier.padding(top = 4.dp))
            Spacer(Modifier.height(32.dp))
            PrimaryButton("Share", Modifier.fillMaxWidth()) {
                val uri = FileProvider.getUriForFile(ctx, ctx.packageName + ".files", step)
                val send = Intent(Intent.ACTION_SEND).setType("application/step").putExtra(Intent.EXTRA_STREAM, uri)
                    .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                ctx.startActivity(Intent.createChooser(send, null))
            }
            Spacer(Modifier.height(8.dp))
            SecondaryButton("Save to Files", Modifier.fillMaxWidth()) { save.launch("${dir.name}.step") }
        }
    }
}
