package com.jakubmini.psom.scan

import android.app.Activity
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.media.Image
import android.opengl.GLES30
import android.opengl.GLSurfaceView
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import com.google.ar.core.CameraConfigFilter
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Point
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import com.jakubmini.psom.gl.BackgroundRenderer
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
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/**
 * The ARCore session and the scan. Runs on the GL thread; files are written on one writer thread,
 * so the camera never waits for the disk. Start and stop are requests picked up at the next frame.
 */
class ArCapture(private val activity: Activity, private val root: File) : GLSurfaceView.Renderer {
    data class Ui(
        val tracking: Boolean = false,
        val reason: String? = null,
        val scanning: Boolean = false,
        val keyframes: Int = 0,
        val coverage: List<Boolean> = List(36) { false },
        val depth: Boolean = false,
        val error: String? = null,
    )

    val ui = MutableStateFlow(Ui())
    /** Called on the main thread once a finished scan's files are all written. */
    var onFinished: ((File) -> Unit)? = null

    private var session: Session? = null
    private val background = BackgroundRenderer()
    private val writerThread = Executors.newSingleThreadExecutor()
    private val main = Handler(Looper.getMainLooper())
    private val imu = ImuRecorder(activity)
    private var viewW = 1
    private var viewH = 1
    private var geometryDirty = true
    private var textureSet = false

    @Volatile private var startRequested = false
    @Volatile private var stopRequested = false
    private var scanning = false
    private var writer: SessionWriter? = null
    private var project: File? = null
    private var selector = KeyframeSelector()
    private val coverage = Coverage()
    private var axes = AxesConvergence()
    private var target: DoubleArray? = null
    private val cameraCentres = ArrayList<DoubleArray>()
    private var wasTracking = true
    private var depthMode = false
    private var metaWritten = false
    private var frameCount = 0
    private var lastT = 0L

    /** Opens the camera. Returns an error to show, or null. */
    fun resume(): String? {
        try {
            val s = session ?: Session(activity).also { s ->
                // the largest CPU image: more pixels for multi-view stereo
                val configs = s.getSupportedCameraConfigs(CameraConfigFilter(s))
                configs.maxByOrNull { it.imageSize.width * it.imageSize.height }?.let { s.cameraConfig = it }
                val c = Config(s)
                depthMode = s.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY)
                if (depthMode) c.depthMode = Config.DepthMode.RAW_DEPTH_ONLY
                c.focusMode = Config.FocusMode.AUTO
                c.updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                s.configure(c)
                session = s
            }
            s.resume()
            geometryDirty = true
            ui.value = ui.value.copy(depth = depthMode, error = null)
            return null
        } catch (e: Exception) {
            Log.e(TAG, "ARCore failed to start", e)
            return "The camera could not start: ${e.message ?: e.javaClass.simpleName}"
        }
    }

    /** Call after the GL view has paused. A scan in progress ends here, with what it has. */
    fun pause() {
        if (scanning) finishScan(lastT)
        session?.pause()
    }

    fun close() {
        session?.close()
        session = null
        writerThread.shutdown()
    }

    fun startScan() {
        stopRequested = false
        startRequested = true
    }

    fun stopScan() {
        startRequested = false
        stopRequested = true
    }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES30.glClearColor(0f, 0f, 0f, 1f)
        background.create()
        textureSet = false
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        GLES30.glViewport(0, 0, width, height)
        viewW = width
        viewH = height
        geometryDirty = true
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT or GLES30.GL_DEPTH_BUFFER_BIT)
        val s = session ?: return
        if (!textureSet) {
            s.setCameraTextureName(background.textureId)
            textureSet = true
        }
        if (geometryDirty) {
            s.setDisplayGeometry(displayRotation(), viewW, viewH)
            geometryDirty = false
        }
        val frame = try {
            s.update()
        } catch (e: Exception) {
            ui.value = ui.value.copy(error = "Camera error: ${e.message}")
            return
        }
        background.draw(frame)
        val cam = frame.camera
        val tracking = cam.trackingState == TrackingState.TRACKING
        val reason = if (tracking) null else cam.trackingFailureReason.name
        lastT = frame.timestamp

        if (startRequested && !scanning) {
            startRequested = false
            startCapture(frame)
        }
        if (stopRequested && scanning) {
            stopRequested = false
            finishScan(frame.timestamp)
        }
        if (scanning) scanFrame(frame, tracking, reason)

        if (++frameCount % 5 == 0 || ui.value.tracking != tracking)
            ui.value = ui.value.copy(
                tracking = tracking, reason = reason, scanning = scanning,
                keyframes = selector.accepted, coverage = coverage.bins.toList(),
            )
    }

    private fun startCapture(frame: Frame) {
        val id = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
        val dir = File(root, id)
        project = dir
        selector = KeyframeSelector()
        coverage.clear()
        axes = AxesConvergence()
        cameraCentres.clear()
        metaWritten = false
        wasTracking = true
        // An optional hint: whatever is under the screen centre when the scan starts.
        target = frame.hitTest(viewW / 2f, viewH / 2f).firstOrNull { h ->
            val t = h.trackable
            (t is Plane && t.isPoseInPolygon(h.hitPose)) || t is Point
        }?.hitPose?.let { doubleArrayOf(it.tx().toDouble(), it.ty().toDouble(), it.tz().toDouble()) }
        val t = frame.timestamp
        val clockOffset = SystemClock.elapsedRealtimeNanos() - t
        val w = SessionWriter(dir)
        writer = w
        val meta = meta(frame, clockOffset, 0, 0)
        writerThread.execute {
            w.meta(meta)
            w.event(t, "capture_start")
        }
        imu.start { ts, kind, x, y, z -> writerThread.execute { w.imu(ts, kind, x, y, z) } }
        scanning = true
    }

    private fun scanFrame(frame: Frame, tracking: Boolean, reason: String?) {
        val w = writer ?: return
        val t = frame.timestamp
        if (tracking != wasTracking) {
            val extra = if (!tracking) buildJsonObject { put("reason", reason) } else null
            writerThread.execute { w.event(t, if (tracking) "tracking_resumed" else "tracking_lost", extra) }
            wasTracking = tracking
        }
        val cam = frame.camera
        val p = cam.pose
        val pose = Pose(p.tx().toDouble(), p.ty().toDouble(), p.tz().toDouble(), p.qx().toDouble(), p.qy().toDouble(), p.qz().toDouble(), p.qw().toDouble())
        if (selector.wouldConsider(t, tracking, pose) != Verdict.ACCEPTED) {
            selector.offer(t, tracking, pose, 0.0)  // counts tracking and rate rejections
            return
        }
        val image = try {
            frame.acquireCameraImage()
        } catch (e: NotYetAvailableException) {
            return
        }
        image.use { img ->
            val y = img.planes[0]
            val luma = ByteArray(y.buffer.remaining()).also { y.buffer.get(it) }
            val sharp = KeyframeSelector.sharpness(luma, img.width, img.height, y.rowStride)
            if (selector.offer(t, tracking, pose, sharp) != Verdict.ACCEPTED) return
            val nv21 = nv21(img)
            val intr = cam.imageIntrinsics
            val dims = intr.imageDimensions
            if (dims[0] != img.width || dims[1] != img.height) {
                Log.w(TAG, "intrinsics are for ${dims[0]}x${dims[1]}, image is ${img.width}x${img.height}; frame skipped")
                return
            }
            val k = doubleArrayOf(intr.focalLength[0].toDouble(), intr.focalLength[1].toDouble(), intr.principalPoint[0].toDouble(), intr.principalPoint[1].toDouble())
            val m = FloatArray(16)
            p.toMatrix(m, 0)  // column-major
            val rowMajor = DoubleArray(16) { m[(it % 4) * 4 + it / 4].toDouble() }

            var depth: ShortArray? = null
            var conf: ByteArray? = null
            var dw = 0
            var dh = 0
            var dt = 0L
            if (depthMode) {
                try {
                    frame.acquireRawDepthImage16Bits().use { d ->
                        frame.acquireRawDepthConfidenceImage().use { c ->
                            dw = d.width
                            dh = d.height
                            dt = d.timestamp
                            depth = shorts(d)
                            conf = bytes(c)
                        }
                    }
                } catch (e: NotYetAvailableException) {
                    depth = null
                }
            }
            val (pts, ids) = frame.acquirePointCloud().use { pc ->
                val f = FloatArray(pc.points.remaining()).also { pc.points.get(it) }
                val i = IntArray(pc.ids.remaining()).also { pc.ids.get(it) }
                f to i
            }
            val width = img.width
            val height = img.height
            if (!metaWritten && (!depthMode || depth != null)) {
                val meta = meta(frame, SystemClock.elapsedRealtimeNanos() - t, dw, dh)
                writerThread.execute { w.meta(meta) }
                metaWritten = true
            }
            val d = depth
            val c = conf
            val sized = metaWritten  // read here, on the GL thread
            writerThread.execute {
                val out = ByteArrayOutputStream()
                YuvImage(nv21, ImageFormat.NV21, width, height, null).compressToJpeg(Rect(0, 0, width, height), 92, out)
                // depth before metadata could carry its size is dropped rather than mislabelled
                val keep = d != null && c != null && sized
                w.keyframe(Keyframe(t, rowMajor, k, sharp, out.toByteArray(), if (keep) d else null, if (keep) c else null, dw, dh, dt, pts, ids))
            }

            // coverage around the hit-test point, or where the camera axes cross
            val centre = doubleArrayOf(p.tx().toDouble(), p.ty().toDouble(), p.tz().toDouble())
            val z = p.zAxis
            axes.add(centre, doubleArrayOf(-z[0].toDouble(), -z[1].toDouble(), -z[2].toDouble()))
            cameraCentres.add(centre)
            val c0 = target ?: axes.center()
            if (c0 != null) {
                coverage.clear()
                for (cc in cameraCentres) coverage.add(cc, c0)
            }
        }
    }

    private fun finishScan(t: Long) {
        scanning = false
        imu.stop()
        val w = writer ?: return
        val dir = project ?: return
        val rejected = selector.rejected.toMap()
        writer = null
        writerThread.execute {
            try {
                w.finish(t, rejected)
            } catch (e: Exception) {
                Log.e(TAG, "finishing the session failed", e)
            }
            main.post { onFinished?.invoke(dir) }
        }
    }

    private fun meta(frame: Frame, clockOffset: Long, dw: Int, dh: Int): SessionMeta {
        val dims = frame.camera.imageIntrinsics.imageDimensions
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US).apply { timeZone = TimeZone.getTimeZone("UTC") }
        val arcore = try {
            activity.packageManager.getPackageInfo("com.google.ar.core", 0).versionName ?: "unknown"
        } catch (e: Exception) {
            "unknown"
        }
        return SessionMeta(
            iso.format(Date()), Build.MANUFACTURER, Build.MODEL, arcore,
            sensorOrientation(), displayRotation() * 90, dims[0], dims[1],
            depthMode && dw > 0, dw, dh, 16_666_666, clockOffset, target,
        )
    }

    private fun sensorOrientation(): Int = try {
        val id = session?.cameraConfig?.cameraId ?: return 0
        val cm = activity.getSystemService(CameraManager::class.java)
        cm.getCameraCharacteristics(id).get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 0
    } catch (e: Exception) {
        0
    }

    @Suppress("DEPRECATION")
    private fun displayRotation(): Int = when (activity.windowManager.defaultDisplay.rotation) {
        Surface.ROTATION_90 -> 1
        Surface.ROTATION_180 -> 2
        Surface.ROTATION_270 -> 3
        else -> 0
    }

    companion object {
        private const val TAG = "psom"

        /** YUV_420_888 to NV21 (Y, then interleaved V/U), honouring row and pixel strides. */
        fun nv21(img: Image): ByteArray {
            val w = img.width
            val h = img.height
            val out = ByteArray(w * h * 3 / 2)
            val y = img.planes[0]
            var o = 0
            for (r in 0 until h) {
                y.buffer.position(r * y.rowStride)
                y.buffer.get(out, o, w)
                o += w
            }
            val u = img.planes[1]
            val v = img.planes[2]
            for (r in 0 until h / 2) for (c in 0 until w / 2) {
                out[o++] = v.buffer.get(r * v.rowStride + c * v.pixelStride)
                out[o++] = u.buffer.get(r * u.rowStride + c * u.pixelStride)
            }
            return out
        }

        fun shorts(img: Image): ShortArray {
            val p = img.planes[0]
            val buf = p.buffer.order(java.nio.ByteOrder.nativeOrder())
            val out = ShortArray(img.width * img.height)
            for (r in 0 until img.height) for (c in 0 until img.width)
                out[r * img.width + c] = buf.getShort(r * p.rowStride + c * p.pixelStride)
            return out
        }

        fun bytes(img: Image): ByteArray {
            val p = img.planes[0]
            val out = ByteArray(img.width * img.height)
            for (r in 0 until img.height) for (c in 0 until img.width)
                out[r * img.width + c] = p.buffer.get(r * p.rowStride + c * p.pixelStride)
            return out
        }
    }
}
