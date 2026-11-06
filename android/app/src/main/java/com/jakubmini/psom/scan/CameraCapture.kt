package com.jakubmini.psom.scan

import android.content.Context
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CaptureRequest
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.util.Range
import android.util.Size
import androidx.annotation.OptIn
import androidx.camera.camera2.interop.Camera2CameraInfo
import androidx.camera.camera2.interop.Camera2Interop
import androidx.camera.camera2.interop.ExperimentalCamera2Interop
import androidx.camera.core.Camera
import androidx.camera.core.CameraInfo
import androidx.camera.core.CameraSelector
import androidx.camera.core.ImageAnalysis
import androidx.camera.core.ImageProxy
import androidx.camera.core.Preview
import androidx.camera.core.resolutionselector.AspectRatioStrategy
import androidx.camera.core.resolutionselector.ResolutionSelector
import androidx.camera.core.resolutionselector.ResolutionStrategy
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.camera.view.PreviewView
import androidx.core.content.ContextCompat
import androidx.lifecycle.LifecycleOwner
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.put
import java.io.ByteArrayOutputStream
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.concurrent.Executors
import kotlin.math.abs

/**
 * The camera and the scan. The preview is its own stream, so it never waits for anything here.
 * Photos arrive on one background thread, which also does all the writing; start and stop are
 * requests picked up at the next photo.
 */
class CameraCapture(private val context: Context, private val root: File) {
    data class Ui(
        val scanning: Boolean = false,
        val keyframes: Int = 0,
        val elapsedMs: Long = 0,
        /** Turning fast enough to blur. */
        val fast: Boolean = false,
        val coverage: List<Boolean> = List(36) { false },
        val error: String? = null,
        val hasTorch: Boolean = false,
        val torch: Boolean = false,
    )

    val ui = MutableStateFlow(Ui())
    /** Called on the main thread once a finished scan's files are all written. */
    var onFinished: ((File) -> Unit)? = null

    private val exec = Executors.newSingleThreadExecutor()
    private val main = Handler(Looper.getMainLooper())
    private val motion = Motion(context)
    private var provider: ProcessCameraProvider? = null
    private var camera: Camera? = null

    @Volatile private var startRequested = false
    @Volatile private var stopRequested = false
    // below: touched only on [exec]
    private var scanning = false
    private var writer: SessionWriter? = null
    private var project: File? = null
    private var selector = KeyframeSelector()
    private val coverage = Coverage()
    private var k = DoubleArray(4)
    private var startT = 0L
    private var lastT = 0L
    private var frameCount = 0
    private class Candidate(val t: Long, val dir: DoubleArray?, val rate: Double, val sharpness: Double, val planes: YuvPlanes, val gravity: FloatArray?)
    private var window = SharpestInWindow<Candidate>()

    @OptIn(ExperimentalCamera2Interop::class)
    fun bind(owner: LifecycleOwner, view: PreviewView) {
        if (ContextCompat.checkSelfPermission(context, android.Manifest.permission.CAMERA) != android.content.pm.PackageManager.PERMISSION_GRANTED) return
        motion.start()
        val future = ProcessCameraProvider.getInstance(context)
        future.addListener({
            try {
                val p = future.get()
                provider = p
                // 4:3 is the sensor's own shape, so the photos keep its whole field of view
                val fourThree = AspectRatioStrategy.RATIO_4_3_FALLBACK_AUTO_STRATEGY
                val fps = steadyFps(CameraSelector.DEFAULT_BACK_CAMERA.filter(p.availableCameraInfos).firstOrNull())
                val previewBuilder = Preview.Builder().setResolutionSelector(ResolutionSelector.Builder().setAspectRatioStrategy(fourThree).build())
                val analysisBuilder = ImageAnalysis.Builder()
                    .setResolutionSelector(
                        ResolutionSelector.Builder().setAspectRatioStrategy(fourThree)
                            .setResolutionStrategy(ResolutionStrategy(Size(1600, 1200), ResolutionStrategy.FALLBACK_RULE_CLOSEST_HIGHER_THEN_LOWER))
                            .build(),
                    )
                    .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
                if (fps != null) {
                    // a frame rate that can't drop keeps each exposure short: indoors, auto exposure
                    // otherwise slows to 1/15 s and every photo taken on the move smears
                    Camera2Interop.Extender(previewBuilder).setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fps)
                    Camera2Interop.Extender(analysisBuilder).setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, fps)
                }
                val preview = previewBuilder.build()
                preview.setSurfaceProvider(view.surfaceProvider)
                val analysis = analysisBuilder.build()
                analysis.setAnalyzer(exec, ::analyze)
                p.unbindAll()
                val cam = p.bindToLifecycle(owner, CameraSelector.DEFAULT_BACK_CAMERA, preview, analysis)
                camera = cam
                val hasTorch = cam.cameraInfo.hasFlashUnit()
                if (hasTorch && ui.value.torch) cam.cameraControl.enableTorch(true)  // back from the background
                ui.value = ui.value.copy(error = null, hasTorch = hasTorch, torch = hasTorch && ui.value.torch)
            } catch (e: Exception) {
                Log.e(TAG, "camera failed to start", e)
                ui.value = ui.value.copy(error = "The camera could not start: ${e.message ?: e.javaClass.simpleName}")
            }
        }, ContextCompat.getMainExecutor(context))
    }

    /** Ends a scan in progress with what it has (the app is going to the background). */
    fun pause() = exec.execute { if (scanning) finish(lastT) }

    fun unbind() {
        pause()
        provider?.unbindAll()
        camera = null
        motion.stop()
    }

    /** The torch: steady light means shorter exposures, so less blur. Glossy things show its glare. */
    fun setTorch(on: Boolean) {
        val cam = camera ?: return
        if (!ui.value.hasTorch) return
        cam.cameraControl.enableTorch(on)
        ui.value = ui.value.copy(torch = on)
    }

    fun startScan() {
        stopRequested = false
        startRequested = true
    }

    fun stopScan() {
        startRequested = false
        stopRequested = true
    }

    private fun analyze(image: ImageProxy) {
        image.use {
            val t = image.imageInfo.timestamp
            lastT = t
            if (startRequested && !scanning) {
                startRequested = false
                begin(t, image)
            }
            if (stopRequested && scanning) {
                stopRequested = false
                finish(t)
            }
            val rate = motion.turnRate
            if (scanning) try {
                keep(t, image, rate)
            } catch (e: Exception) {
                Log.e(TAG, "photo failed", e)
            }
            val fast = scanning && rate > KeyframeSelector.MAX_TURN_RATE
            if (++frameCount % 5 == 0 || ui.value.scanning != scanning || ui.value.fast != fast)
                ui.value = ui.value.copy(
                    scanning = scanning, keyframes = selector.accepted, fast = fast, coverage = coverage.bins.toList(),
                    elapsedMs = if (scanning) (t - startT) / 1_000_000 else 0,
                )
        }
    }

    private fun keep(t: Long, image: ImageProxy, rate: Double) {
        val w = writer ?: return
        window.tick(t)?.let { commit(w, it) }
        val dir = motion.towardCamera()
        if (selector.wouldConsider(t, dir, rate) != Verdict.ACCEPTED) {
            selector.offer(t, dir, rate, 0.0)  // counts the rejection
            return
        }
        val (yp, up, vp) = image.planes
        val y = copy(yp.buffer)
        val sharp = KeyframeSelector.sharpness(y, image.width, image.height, yp.rowStride)
        window.offer(t, sharp) {
            val planes = YuvPlanes(
                image.width, image.height,
                y, yp.rowStride,
                copy(up.buffer), up.rowStride, up.pixelStride,
                copy(vp.buffer), vp.rowStride, vp.pixelStride,
            )
            Candidate(t, dir, rate, sharp, planes, motion.gravity?.copyOf())
        }
    }

    /** The sharpest frame of a window becomes a keyframe, if it is sharp enough next to the recent ones. */
    private fun commit(w: SessionWriter, c: Candidate) {
        if (selector.offer(c.t, c.dir, c.rate, c.sharpness) != Verdict.ACCEPTED) return
        val p = c.planes
        val out = ByteArrayOutputStream()
        YuvImage(p.nv21(), ImageFormat.NV21, p.width, p.height, null).compressToJpeg(Rect(0, 0, p.width, p.height), 92, out)
        w.keyframe(Keyframe(c.t, k, c.sharpness, out.toByteArray(), c.gravity))
        if (c.dir != null) coverage.add(c.dir, ORIGIN)
    }

    private fun begin(t: Long, image: ImageProxy) {
        val dir = File(root, SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date()))
        project = dir
        selector = KeyframeSelector()
        window = SharpestInWindow()
        coverage.clear()
        k = intrinsics(image.width, image.height)
        startT = t
        val w = SessionWriter(dir)
        writer = w
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US).apply { timeZone = TimeZone.getTimeZone("UTC") }
        w.meta(SessionMeta(iso.format(Date()), Build.MANUFACTURER, Build.MODEL, sensorOrientation(image), image.width, image.height))
        w.event(t, "capture_start", buildJsonObject { put("torch", ui.value.torch) })
        motion.sink = { ts, kind, x, y, z -> exec.execute { writer?.imu(ts, kind, x, y, z) } }
        scanning = true
    }

    private fun finish(t: Long) {
        scanning = false
        motion.sink = null
        val w = writer ?: return
        val dir = project ?: return
        writer = null
        try {
            window.flush()?.let { commit(w, it) }
            w.finish(t, selector.rejected.toMap())
        } catch (e: Exception) {
            Log.e(TAG, "finishing the session failed", e)
        }
        main.post { onFinished?.invoke(dir) }
    }

    /**
     * fx, fy, cx, cy for a w x h photo. Only a starting point: the engine refines the focal length
     * from the photos. The factory calibration when the phone has one, else the lens's spec sheet.
     */
    @OptIn(ExperimentalCamera2Interop::class)
    private fun intrinsics(w: Int, h: Int): DoubleArray {
        val info = camera?.cameraInfo?.let { Camera2CameraInfo.from(it) }
        val cal = info?.getCameraCharacteristic(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)
        val pre = info?.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE)
        val lens = info?.getCameraCharacteristic(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull()
        val phys = info?.getCameraCharacteristic(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
        val f = when {
            // the calibration is for the full active array; it only carries over at the same shape
            cal != null && pre != null && cal[0] > 0 && abs(pre.width().toDouble() / pre.height() - w.toDouble() / h) < 0.01 ->
                cal[0].toDouble() * w / pre.width()
            lens != null && phys != null && phys.width > 0 -> lens.toDouble() / phys.width * w
            else -> 0.8 * w
        }
        // shortcut: the principal point is taken as the image centre and lens distortion is ignored;
        // fine for phone main cameras, upgrade if a lens with strong distortion shows bent edges.
        return doubleArrayOf(f, f, (w - 1) / 2.0, (h - 1) / 2.0)
    }

    /** The frame-rate range with the highest floor (30 fps on most phones), or null if none reaches 24. */
    @OptIn(ExperimentalCamera2Interop::class)
    private fun steadyFps(info: CameraInfo?): Range<Int>? {
        val ranges = info?.let { Camera2CameraInfo.from(it).getCameraCharacteristic(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES) }
        return ranges?.filter { it.lower >= 24 }?.maxWithOrNull(compareBy<Range<Int>> { it.lower }.thenByDescending { it.upper })
    }

    @OptIn(ExperimentalCamera2Interop::class)
    private fun sensorOrientation(image: ImageProxy): Int =
        camera?.cameraInfo?.let { Camera2CameraInfo.from(it).getCameraCharacteristic(CameraCharacteristics.SENSOR_ORIENTATION) }
            ?: image.imageInfo.rotationDegrees

    companion object {
        private const val TAG = "psom"
        private val ORIGIN = DoubleArray(3)

        /** One bulk copy of a plane's bytes (a memcpy, not a per-pixel loop). */
        fun copy(b: java.nio.ByteBuffer): ByteArray {
            b.rewind()
            return ByteArray(b.remaining()).also { b.get(it) }
        }
    }
}
