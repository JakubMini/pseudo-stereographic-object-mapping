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
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Point
import com.google.ar.core.Session
import com.google.ar.core.TrackingFailureReason
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
        val elapsedMs: Long = 0,
        /** ARCore's pose jumped: it lost track of where the phone is. */
        val jumping: Boolean = false,
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
    private var scanStartT = 0L
    private val inFlight = java.util.concurrent.atomic.AtomicBoolean(false)
    private val recentPoses = ArrayDeque<Pair<Long, DoubleArray>>()
    private var notTrackingSince = -1L
    private var lastReason = "NONE"

    /** Opens the camera. Returns an error to show, or null. */
    fun resume(): String? {
        try {
            val s = session ?: Session(activity).also { s ->
                // ARCore's stock settings except autofocus (fixed focus left close objects soft).
                // On a Galaxy A56 our earlier extras (1080p CPU stream, raw depth, non-blocking
                // updates, autofocus) went with tracking pausing 6 times a second and the pose
                // running away; this build keeps only autofocus to tell which one did it.
                // shortcut: raw depth is off everywhere; turn it back on per device once verified on one
                // with a real depth sensor (the engine already uses it, with its own pose).
                val c = Config(s)
                c.focusMode = Config.FocusMode.AUTO
                depthMode = false
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
        // ARCore can report a one-frame pause while still tracking fine: only a pause lasting
        // 150 ms counts as lost, for the banner and the log. Keyframes still need `tracking`.
        if (tracking) notTrackingSince = -1L else if (notTrackingSince < 0) notTrackingSince = frame.timestamp
        val lost = !tracking && frame.timestamp - notTrackingSince >= 150_000_000L
        if (!tracking && cam.trackingFailureReason != TrackingFailureReason.NONE) lastReason = cam.trackingFailureReason.name
        val reason = if (lost) lastReason else null
        lastT = frame.timestamp
        if (scanning) {
            // remembered so late raw depth can be paired with the frame it was measured from
            val m = FloatArray(16)
            cam.pose.toMatrix(m, 0)
            recentPoses.addLast(frame.timestamp to DoubleArray(16) { m[(it % 4) * 4 + it / 4].toDouble() })
            if (recentPoses.size > 90) recentPoses.removeFirst()
        }

        if (startRequested && !scanning) {
            startRequested = false
            startCapture(frame)
        }
        if (stopRequested && scanning) {
            stopRequested = false
            finishScan(frame.timestamp)
        }
        if (scanning) scanFrame(frame, tracking, lost, reason)

        val jumping = scanning && selector.jumpingSince >= 0
        if (++frameCount % 5 == 0 || ui.value.tracking != !lost || ui.value.jumping != jumping)
            ui.value = ui.value.copy(
                tracking = !lost, reason = reason, scanning = scanning, jumping = jumping,
                keyframes = selector.accepted, coverage = synchronized(coverage) { coverage.bins.toList() },
                elapsedMs = if (scanning) (frame.timestamp - scanStartT) / 1_000_000 else 0,
            )
    }

    private fun startCapture(frame: Frame) {
        val id = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
        val dir = File(root, id)
        project = dir
        selector = KeyframeSelector()
        inFlight.set(false)
        recentPoses.clear()
        synchronized(coverage) {
            coverage.clear()
            axes = AxesConvergence()
            cameraCentres.clear()
        }
        metaWritten = false
        wasTracking = true
        // An optional hint: whatever is under the screen centre when the scan starts.
        target = frame.hitTest(viewW / 2f, viewH / 2f).firstOrNull { h ->
            val t = h.trackable
            (t is Plane && t.isPoseInPolygon(h.hitPose)) || t is Point
        }?.hitPose?.let { doubleArrayOf(it.tx().toDouble(), it.ty().toDouble(), it.tz().toDouble()) }
        val t = frame.timestamp
        scanStartT = t
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

    private fun scanFrame(frame: Frame, tracking: Boolean, lost: Boolean, reason: String?) {
        val w = writer ?: return
        val t = frame.timestamp
        if (!lost != wasTracking) {
            val extra = if (lost) buildJsonObject { put("reason", reason ?: "NONE") } else null
            writerThread.execute { w.event(t, if (lost) "tracking_lost" else "tracking_resumed", extra) }
            wasTracking = !lost
        }
        val cam = frame.camera
        val p = cam.pose
        val pose = Pose(p.tx().toDouble(), p.ty().toDouble(), p.tz().toDouble(), p.qx().toDouble(), p.qy().toDouble(), p.qz().toDouble(), p.qw().toDouble())
        // One keyframe in flight at a time; until it is written, frames just render.
        if (inFlight.get()) return
        val verdict = synchronized(selector) { selector.wouldConsider(t, tracking, pose) }
        if (verdict != Verdict.ACCEPTED) {
            synchronized(selector) { selector.offer(t, tracking, pose, 0.0) }  // counts tracking and rate rejections
            return
        }
        val intr = cam.imageIntrinsics
        val dims = intr.imageDimensions
        // On the camera thread only bulk copies happen (a few ms); everything slow runs on the
        // writer thread, so the preview never stops.
        val planes = try {
            frame.acquireCameraImage().use { img ->
                if (dims[0] != img.width || dims[1] != img.height) {
                    Log.w(TAG, "intrinsics are for ${dims[0]}x${dims[1]}, image is ${img.width}x${img.height}; frame skipped")
                    return
                }
                val (yp, up, vp) = img.planes
                YuvPlanes(
                    img.width, img.height,
                    copy(yp.buffer), yp.rowStride,
                    copy(up.buffer), up.rowStride, up.pixelStride,
                    copy(vp.buffer), vp.rowStride, vp.pixelStride,
                )
            }
        } catch (e: NotYetAvailableException) {
            return
        }
        var depth: DepthPlane? = null
        var conf: DepthPlane? = null
        var dt = 0L
        if (depthMode) {
            try {
                frame.acquireRawDepthImage16Bits().use { d ->
                    frame.acquireRawDepthConfidenceImage().use { c ->
                        dt = d.timestamp
                        depth = DepthPlane(d.width, d.height, copy(d.planes[0].buffer), d.planes[0].rowStride, d.planes[0].pixelStride)
                        conf = DepthPlane(c.width, c.height, copy(c.planes[0].buffer), c.planes[0].rowStride, c.planes[0].pixelStride)
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
        val k = doubleArrayOf(intr.focalLength[0].toDouble(), intr.focalLength[1].toDouble(), intr.principalPoint[0].toDouble(), intr.principalPoint[1].toDouble())
        val m = FloatArray(16)
        p.toMatrix(m, 0)  // column-major
        val rowMajor = DoubleArray(16) { m[(it % 4) * 4 + it / 4].toDouble() }
        val centre = doubleArrayOf(p.tx().toDouble(), p.ty().toDouble(), p.tz().toDouble())
        val z = p.zAxis
        val forward = doubleArrayOf(-z[0].toDouble(), -z[1].toDouble(), -z[2].toDouble())
        val depthPose = if (depth != null) recentPoses.lastOrNull { kotlin.math.abs(it.first - dt) < 1_000_000L }?.second else null
        if (depthPose == null) {  // too old to pair with a pose: dropped rather than misplaced
            depth = null
            conf = null
        }
        val d = depth
        val c = conf
        if (!metaWritten && (!depthMode || d != null)) {
            val meta = meta(frame, SystemClock.elapsedRealtimeNanos() - t, d?.width ?: 0, d?.height ?: 0)
            writerThread.execute { w.meta(meta) }
            metaWritten = true
        }
        val sized = metaWritten  // read here, on the camera thread
        val sel = selector
        inFlight.set(true)
        writerThread.execute {
            try {
                val sharp = KeyframeSelector.sharpness(planes.y, planes.width, planes.height, planes.yRow)
                if (synchronized(sel) { sel.offer(t, true, pose, sharp) } != Verdict.ACCEPTED) return@execute
                val out = ByteArrayOutputStream()
                YuvImage(planes.nv21(), ImageFormat.NV21, planes.width, planes.height, null)
                    .compressToJpeg(Rect(0, 0, planes.width, planes.height), 92, out)
                // depth from before the metadata could carry its size is dropped rather than mislabelled
                val keep = d != null && c != null && sized
                w.keyframe(
                    Keyframe(t, rowMajor, k, sharp, out.toByteArray(), if (keep) d!!.shorts() else null, if (keep) c!!.bytes() else null,
                        d?.width ?: 0, d?.height ?: 0, dt, pts, ids, if (keep) depthPose else null),
                )
                // coverage around the hit-test point, or where the camera axes cross
                synchronized(coverage) {
                    axes.add(centre, forward)
                    cameraCentres.add(centre)
                    val c0 = target ?: axes.center()
                    if (c0 != null) {
                        coverage.clear()
                        for (cc in cameraCentres) coverage.add(cc, c0)
                    }
                }
            } catch (e: Exception) {
                Log.e(TAG, "keyframe failed", e)
            } finally {
                inFlight.set(false)
            }
        }
    }

    private fun finishScan(t: Long) {
        scanning = false
        imu.stop()
        val w = writer ?: return
        val dir = project ?: return
        val rejected = synchronized(selector) { selector.rejected.toMap() }
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

        /** One bulk copy of a plane's bytes (a memcpy, not a per-pixel loop). */
        fun copy(b: java.nio.ByteBuffer): ByteArray {
            b.rewind()
            return ByteArray(b.remaining()).also { b.get(it) }
        }
    }
}
