package com.jakubmini.psom.gl

import android.annotation.SuppressLint
import android.content.Context
import android.opengl.GLES30
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import kotlinx.serialization.json.JsonArray
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.JsonPrimitive
import kotlinx.serialization.json.buildJsonObject
import kotlinx.serialization.json.double
import kotlinx.serialization.json.jsonArray
import kotlinx.serialization.json.jsonPrimitive
import kotlinx.serialization.json.put
import java.io.DataInputStream
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import java.nio.IntBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/** The engine's mesh.ply: world metres, three confidence channels per vertex. */
class MeshData(val pos: FloatArray, val weight: FloatArray, val diversity: FloatArray, val sigma: FloatArray, val tris: IntArray) {
    val vertices get() = pos.size / 3

    fun bounds(): Pair<FloatArray, FloatArray> {
        val lo = FloatArray(3) { Float.MAX_VALUE }
        val hi = FloatArray(3) { -Float.MAX_VALUE }
        for (i in 0 until vertices) for (k in 0..2) {
            lo[k] = min(lo[k], pos[3 * i + k])
            hi[k] = max(hi[k], pos[3 * i + k])
        }
        return lo to hi
    }

    /** Nearest triangle hit along a ray (Möller–Trumbore), or null. */
    fun pick(o: DoubleArray, d: DoubleArray): DoubleArray? {
        var best = Double.MAX_VALUE
        for (t in 0 until tris.size / 3) {
            val a = tris[3 * t] * 3
            val b = tris[3 * t + 1] * 3
            val c = tris[3 * t + 2] * 3
            val e1 = DoubleArray(3) { (pos[b + it] - pos[a + it]).toDouble() }
            val e2 = DoubleArray(3) { (pos[c + it] - pos[a + it]).toDouble() }
            val p = cross(d, e2)
            val det = dot(e1, p)
            if (abs(det) < 1e-12) continue
            val s = DoubleArray(3) { o[it] - pos[a + it] }
            val u = dot(s, p) / det
            if (u < 0 || u > 1) continue
            val q = cross(s, e1)
            val v = dot(d, q) / det
            if (v < 0 || u + v > 1) continue
            val dist = dot(e2, q) / det
            if (dist in 0.0..best) best = dist
        }
        return if (best == Double.MAX_VALUE) null else DoubleArray(3) { o[it] + d[it] * best }
    }

    companion object {
        fun load(f: File): MeshData {
            DataInputStream(f.inputStream().buffered()).use { inp ->
                var nv = 0
                var nf = 0
                while (true) {
                    val line = buildString {
                        while (true) {
                            val ch = inp.read()
                            if (ch < 0 || ch == '\n'.code) break
                            append(ch.toChar())
                        }
                    }
                    if (line.startsWith("element vertex ")) nv = line.substring(15).trim().toInt()
                    if (line.startsWith("element face ")) nf = line.substring(13).trim().toInt()
                    if (line == "end_header") break
                }
                val vb = ByteBuffer.wrap(ByteArray(nv * 24).also { inp.readFully(it) }).order(ByteOrder.LITTLE_ENDIAN)
                val pos = FloatArray(nv * 3)
                val w = FloatArray(nv)
                val dv = FloatArray(nv)
                val sg = FloatArray(nv)
                for (i in 0 until nv) {
                    for (k in 0..2) pos[3 * i + k] = vb.float
                    w[i] = vb.float
                    dv[i] = vb.float
                    sg[i] = vb.float
                }
                val fb = ByteBuffer.wrap(ByteArray(nf * 13).also { inp.readFully(it) }).order(ByteOrder.LITTLE_ENDIAN)
                val tris = IntArray(nf * 3)
                for (i in 0 until nf) {
                    fb.get()
                    for (k in 0..2) tris[3 * i + k] = fb.int
                }
                return MeshData(pos, w, dv, sg, tris)
            }
        }

        fun cross(a: DoubleArray, b: DoubleArray) = doubleArrayOf(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
        fun dot(a: DoubleArray, b: DoubleArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }
}

/** The oriented crop box, as project.json stores it. */
class Crop(val center: DoubleArray, val axes: Array<DoubleArray>, val half: DoubleArray) {
    fun copy() = Crop(center.copyOf(), Array(3) { axes[it].copyOf() }, half.copyOf())
    fun faceCenter(k: Int, s: Int) = DoubleArray(3) { center[it] + axes[k][it] * half[k] * s }

    fun corners(): List<DoubleArray> = (0 until 8).map { i ->
        val sx = if (i and 1 != 0) 1 else -1
        val sy = if (i and 2 != 0) 1 else -1
        val sz = if (i and 4 != 0) 1 else -1
        DoubleArray(3) { center[it] + axes[0][it] * half[0] * sx + axes[1][it] * half[1] * sy + axes[2][it] * half[2] * sz }
    }

    /** Moves one face along its normal by [d] metres; the opposite face stays put. */
    fun moveFace(k: Int, s: Int, d: Double) {
        val h = max(0.003, half[k] + d / 2)
        val applied = (h - half[k]) * 2
        half[k] = h
        for (i in 0..2) center[i] += axes[k][i] * s * applied / 2
    }

    fun toJson(): JsonObject = buildJsonObject {
        put("source", "user")
        put("center", JsonArray(center.map { JsonPrimitive(it) }))
        put("axes", JsonArray(axes.map { a -> JsonArray(a.map { JsonPrimitive(it) }) }))
        put("half", JsonArray(half.map { JsonPrimitive(it) }))
    }

    companion object {
        fun fromJson(j: JsonObject?): Crop? {
            j ?: return null
            fun v(e: kotlinx.serialization.json.JsonElement) = e.jsonArray.map { it.jsonPrimitive.double }.toDoubleArray()
            return try {
                Crop(v(j["center"]!!), j["axes"]!!.jsonArray.map { v(it) }.toTypedArray(), v(j["half"]!!))
            } catch (e: Exception) {
                null
            }
        }
    }
}

enum class ColorMode { SHADED, WEIGHT, DIVERSITY, UNCERTAINTY }

private val TERRACOTTA = floatArrayOf(0.851f, 0.482f, 0.290f, 1f)
private val WHITE = floatArrayOf(1f, 0.984f, 0.953f, 1f)
enum class Tool { ORBIT, CROP, MEASURE }

/** Orbit/zoom viewer for the reconstructed mesh, with the crop box and picked points drawn on top. */
@SuppressLint("ViewConstructor")
class MeshView(context: Context) : GLSurfaceView(context) {
    var onPick: ((DoubleArray) -> Unit)? = null
    var onCropChanged: ((Crop) -> Unit)? = null
    var tool = Tool.ORBIT

    private val r = Renderer()
    @Volatile private var vp = FloatArray(16)
    private var mesh: MeshData? = null
    private var crop: Crop? = null
    private var activeHandle: Pair<Int, Int>? = null

    init {
        setEGLContextClientVersion(3)
        setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        setRenderer(r)
        renderMode = RENDERMODE_WHEN_DIRTY
    }

    fun setMesh(m: MeshData, mode: ColorMode) {
        mesh = m
        queueEvent { r.upload(m, mode) }
        requestRender()
    }

    fun setColorMode(mode: ColorMode) {
        val m = mesh ?: return
        queueEvent { r.upload(m, mode) }
        requestRender()
    }

    fun setCrop(c: Crop?) {
        crop = c?.copy()
        val snapshot = crop?.copy()
        queueEvent { r.crop = snapshot }
        requestRender()
    }

    fun setMarks(points: List<DoubleArray>) {
        queueEvent { r.marks = points.toList() }
        requestRender()
    }

    private fun project(p: DoubleArray): FloatArray? {
        val v = floatArrayOf(p[0].toFloat(), p[1].toFloat(), p[2].toFloat(), 1f)
        val o = FloatArray(4)
        Matrix.multiplyMV(o, 0, vp, 0, v, 0)
        if (o[3] <= 0) return null
        return floatArrayOf((o[0] / o[3] + 1) / 2 * width, (1 - o[1] / o[3]) / 2 * height)
    }

    private fun ray(x: Float, y: Float): Pair<DoubleArray, DoubleArray> {
        val inv = FloatArray(16)
        Matrix.invertM(inv, 0, vp, 0)
        fun un(z: Float): DoubleArray {
            val o = FloatArray(4)
            Matrix.multiplyMV(o, 0, inv, 0, floatArrayOf(x / width * 2 - 1, 1 - y / height * 2, z, 1f), 0)
            return DoubleArray(3) { (o[it] / o[3]).toDouble() }
        }
        val a = un(-1f)
        val b = un(1f)
        val d = DoubleArray(3) { b[it] - a[it] }
        val l = sqrt(MeshData.dot(d, d))
        return a to DoubleArray(3) { d[it] / l }
    }

    private val scale = ScaleGestureDetector(context, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(d: ScaleGestureDetector): Boolean {
            val f = d.scaleFactor
            queueEvent { r.distance /= f }
            requestRender()
            return true
        }
    })

    private val gestures = GestureDetector(context, object : GestureDetector.SimpleOnGestureListener() {
        override fun onDown(e: MotionEvent): Boolean {
            activeHandle = null
            val c = crop
            if (tool == Tool.CROP && c != null) {
                var best = 80f
                for (k in 0..2) for (s in intArrayOf(-1, 1)) {
                    val sp = project(c.faceCenter(k, s)) ?: continue
                    val dd = hypot(sp[0] - e.x, sp[1] - e.y)
                    if (dd < best) best = dd.also { activeHandle = k to s }
                }
            }
            return true
        }

        override fun onScroll(e1: MotionEvent?, e2: MotionEvent, dx: Float, dy: Float): Boolean {
            val h = activeHandle
            val c = crop
            if (h != null && c != null) {
                // screen direction and length of 1 cm along the face normal
                val a = project(c.faceCenter(h.first, h.second)) ?: return true
                val tip = c.faceCenter(h.first, h.second).let { p -> DoubleArray(3) { p[it] + c.axes[h.first][it] * h.second * 0.01 } }
                val b = project(tip) ?: return true
                val vx = b[0] - a[0]
                val vy = b[1] - a[1]
                val len2 = vx * vx + vy * vy
                if (len2 < 1f) return true
                val along = (-dx * vx - dy * vy) / len2 * 0.01  // metres
                c.moveFace(h.first, h.second, along.toDouble())
                val snapshot = c.copy()
                queueEvent { r.crop = snapshot }
                onCropChanged?.invoke(c.copy())
            } else {
                queueEvent {
                    r.yaw += dx * 0.3f
                    r.pitch = (r.pitch + dy * 0.3f).coerceIn(-85f, 85f)
                }
            }
            requestRender()
            return true
        }

        override fun onSingleTapUp(e: MotionEvent): Boolean {
            if (tool != Tool.MEASURE) return false
            val m = mesh ?: return false
            val (o, d) = ray(e.x, e.y)
            m.pick(o, d)?.let { onPick?.invoke(it) }
            return true
        }
    })

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(e: MotionEvent): Boolean {
        scale.onTouchEvent(e)
        if (!scale.isInProgress) gestures.onTouchEvent(e)
        return true
    }

    private inner class Renderer : GLSurfaceView.Renderer {
        var yaw = 30f
        var pitch = 25f
        var distance = 0.4f
        var crop: Crop? = null
        var marks: List<DoubleArray> = emptyList()
        private var target = floatArrayOf(0f, 0f, 0f)
        private var meshProg = 0
        private var lineProg = 0
        private var bgProg = 0
        private var viewW = 1f
        private var viewH = 1f
        private var pos: FloatBuffer? = null
        private var nrm: FloatBuffer? = null
        private var col: FloatBuffer? = null
        private var idx: IntBuffer? = null
        private var count = 0
        private val proj = FloatArray(16)
        private val view = FloatArray(16)
        private var aspect = 1f

        override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
            GLES30.glClearColor(0.969f, 0.933f, 0.867f, 1f)
            GLES30.glEnable(GLES30.GL_DEPTH_TEST)
            // a soft studio light behind the model: warm white in the middle, cream at the edges
            bgProg = Shaders.program(
                """
                #version 300 es
                in vec2 a_pos; void main() { gl_Position = vec4(a_pos, 0.0, 1.0); }
                """,
                """
                #version 300 es
                precision mediump float; uniform vec2 u_size; out vec4 o;
                void main() {
                  vec2 p = (gl_FragCoord.xy / u_size - 0.5) * vec2(u_size.x / u_size.y, 1.0);
                  float t = smoothstep(0.05, 0.5, length(p));
                  o = vec4(mix(vec3(1.0, 0.988, 0.965), vec3(0.969, 0.933, 0.867), t), 1.0);  // to the panel's cream
                }
                """,
            )
            meshProg = Shaders.program(
                """
                #version 300 es
                in vec3 a_pos; in vec3 a_nrm; in vec3 a_col; uniform mat4 u_vp; uniform vec3 u_eye;
                out vec3 v_col; out float v_light;
                void main() {
                  gl_Position = u_vp * vec4(a_pos, 1.0);
                  vec3 l = normalize(u_eye - a_pos);
                  v_light = 0.42 + 0.58 * abs(dot(normalize(a_nrm), l));
                  v_col = a_col;
                }
                """,
                """
                #version 300 es
                precision mediump float; in vec3 v_col; in float v_light; out vec4 o;
                void main() { o = vec4(v_col * v_light, 1.0); }
                """,
            )
            lineProg = Shaders.program(
                """
                #version 300 es
                in vec3 a_pos; uniform mat4 u_vp; uniform float u_size;
                void main() { gl_Position = u_vp * vec4(a_pos, 1.0); gl_PointSize = u_size; }
                """,
                """
                #version 300 es
                precision mediump float; uniform vec4 u_col; uniform float u_round; out vec4 o;
                void main() {
                  if (u_round > 0.5 && length(gl_PointCoord - vec2(0.5)) > 0.5) discard;  // round points
                  o = u_col;
                }
                """,
            )
            mesh?.let { upload(it, ColorMode.SHADED) }
        }

        fun upload(m: MeshData, mode: ColorMode) {
            val (lo, hi) = m.bounds()
            target = FloatArray(3) { (lo[it] + hi[it]) / 2 }
            val radius = sqrt((0..2).sumOf { ((hi[it] - lo[it]) / 2).toDouble().let { d -> d * d } }).toFloat()
            if (count == 0) distance = max(0.05f, radius * 3.6f)
            val n = FloatArray(m.pos.size)
            for (t in 0 until m.tris.size / 3) {
                val a = m.tris[3 * t] * 3
                val b = m.tris[3 * t + 1] * 3
                val c = m.tris[3 * t + 2] * 3
                val e1 = DoubleArray(3) { (m.pos[b + it] - m.pos[a + it]).toDouble() }
                val e2 = DoubleArray(3) { (m.pos[c + it] - m.pos[a + it]).toDouble() }
                val fn = MeshData.cross(e1, e2)
                for (v in intArrayOf(a, b, c)) for (k in 0..2) n[v + k] += fn[k].toFloat()
            }
            val colors = FloatArray(m.pos.size)
            val ch: FloatArray? = when (mode) {
                ColorMode.SHADED -> null
                ColorMode.WEIGHT -> m.weight
                ColorMode.DIVERSITY -> m.diversity
                ColorMode.UNCERTAINTY -> m.sigma
            }
            // guessed by the engine, not seen: weight 0 (a seen vertex has at least 0.25 after decimation)
            fun guessed(i: Int) = m.weight[i] < 0.25f
            if (ch == null) {
                for (i in 0 until m.vertices) { colors[3 * i] = 0.98f; colors[3 * i + 1] = 0.95f; colors[3 * i + 2] = 0.89f }
            } else {
                val sorted = (0 until m.vertices).filter { !guessed(it) }.map { ch[it] }.sorted().ifEmpty { listOf(0f) }
                val p5 = sorted[sorted.size / 20]
                val p95 = sorted[sorted.size * 19 / 20]
                for (i in 0 until m.vertices) {
                    var t = if (p95 > p5) (ch[i] - p5) / (p95 - p5) else 0.5f
                    t = t.coerceIn(0f, 1f)
                    if (mode == ColorMode.UNCERTAINTY) t = 1 - t  // low sigma is good
                    // terracotta = weak, sea green = strong (they differ in lightness too)
                    colors[3 * i] = 0.851f + (0.525f - 0.851f) * t
                    colors[3 * i + 1] = 0.482f + (0.773f - 0.482f) * t
                    colors[3 * i + 2] = 0.290f + (0.706f - 0.290f) * t
                }
            }
            for (i in 0 until m.vertices) if (guessed(i)) { colors[3 * i] = 0.839f; colors[3 * i + 1] = 0.698f; colors[3 * i + 2] = 0.243f }  // mustard
            pos = fb(m.pos)
            nrm = fb(n)
            col = fb(colors)
            idx = ByteBuffer.allocateDirect(m.tris.size * 4).order(ByteOrder.nativeOrder()).asIntBuffer().apply { put(m.tris); position(0) }
            count = m.tris.size
        }

        override fun onSurfaceChanged(gl: GL10?, w: Int, h: Int) {
            GLES30.glViewport(0, 0, w, h)
            aspect = w.toFloat() / max(1, h)
            viewW = w.toFloat()
            viewH = h.toFloat()
        }

        override fun onDrawFrame(gl: GL10?) {
            GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT or GLES30.GL_DEPTH_BUFFER_BIT)
            GLES30.glDisable(GLES30.GL_DEPTH_TEST)
            GLES30.glUseProgram(bgProg)
            GLES30.glUniform2f(GLES30.glGetUniformLocation(bgProg, "u_size"), viewW, viewH)
            val quad = fb(floatArrayOf(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f))
            val bl = GLES30.glGetAttribLocation(bgProg, "a_pos")
            GLES30.glVertexAttribPointer(bl, 2, GLES30.GL_FLOAT, false, 0, quad)
            GLES30.glEnableVertexAttribArray(bl)
            GLES30.glDrawArrays(GLES30.GL_TRIANGLE_STRIP, 0, 4)
            GLES30.glDisableVertexAttribArray(bl)
            GLES30.glEnable(GLES30.GL_DEPTH_TEST)
            val yr = Math.toRadians(yaw.toDouble())
            val pr = Math.toRadians(pitch.toDouble())
            val eye = floatArrayOf(
                target[0] + distance * (cos(pr) * cos(yr)).toFloat(),
                target[1] + distance * sin(pr).toFloat(),
                target[2] + distance * (cos(pr) * sin(yr)).toFloat(),
            )
            Matrix.perspectiveM(proj, 0, 45f, aspect, distance * 0.05f, distance * 10f)
            Matrix.setLookAtM(view, 0, eye[0], eye[1], eye[2], target[0], target[1], target[2], 0f, 1f, 0f)
            val m = FloatArray(16)
            Matrix.multiplyMM(m, 0, proj, 0, view, 0)
            vp = m
            if (count > 0) {
                GLES30.glUseProgram(meshProg)
                GLES30.glUniformMatrix4fv(GLES30.glGetUniformLocation(meshProg, "u_vp"), 1, false, m, 0)
                GLES30.glUniform3fv(GLES30.glGetUniformLocation(meshProg, "u_eye"), 1, eye, 0)
                attr(meshProg, "a_pos", pos!!)
                attr(meshProg, "a_nrm", nrm!!)
                attr(meshProg, "a_col", col!!)
                idx!!.position(0)
                GLES30.glDrawElements(GLES30.GL_TRIANGLES, count, GLES30.GL_UNSIGNED_INT, idx)
            }
            crop?.let { c ->
                val k = c.corners()
                val edges = intArrayOf(0, 1, 2, 3, 4, 5, 6, 7, 0, 2, 1, 3, 4, 6, 5, 7, 0, 4, 1, 5, 2, 6, 3, 7)
                lines(m, edges.flatMap { k[it].map(Double::toFloat) }.toFloatArray(), GLES30.GL_LINES, TERRACOTTA, 1f)
                val handles = (0..2).flatMap { a -> listOf(-1, 1).flatMap { s -> c.faceCenter(a, s).map(Double::toFloat) } }.toFloatArray()
                lines(m, handles, GLES30.GL_POINTS, TERRACOTTA, 40f)
                lines(m, handles, GLES30.GL_POINTS, WHITE, 24f)
            }
            if (marks.isNotEmpty()) {
                val pts = marks.flatMap { p -> p.map(Double::toFloat) }.toFloatArray()
                if (marks.size == 2) lines(m, pts, GLES30.GL_LINES, TERRACOTTA, 1f)
                lines(m, pts, GLES30.GL_POINTS, TERRACOTTA, 36f)
                lines(m, pts, GLES30.GL_POINTS, WHITE, 18f)
            }
        }

        private fun lines(m: FloatArray, pts: FloatArray, mode: Int, color: FloatArray, size: Float) {
            GLES30.glDisable(GLES30.GL_DEPTH_TEST)
            GLES30.glUseProgram(lineProg)
            GLES30.glUniformMatrix4fv(GLES30.glGetUniformLocation(lineProg, "u_vp"), 1, false, m, 0)
            GLES30.glUniform4fv(GLES30.glGetUniformLocation(lineProg, "u_col"), 1, color, 0)
            GLES30.glUniform1f(GLES30.glGetUniformLocation(lineProg, "u_size"), size)
            GLES30.glUniform1f(GLES30.glGetUniformLocation(lineProg, "u_round"), if (mode == GLES30.GL_POINTS) 1f else 0f)
            attr(lineProg, "a_pos", fb(pts))
            GLES30.glDrawArrays(mode, 0, pts.size / 3)
            GLES30.glEnable(GLES30.GL_DEPTH_TEST)
        }

        private fun attr(prog: Int, name: String, b: FloatBuffer) {
            val loc = GLES30.glGetAttribLocation(prog, name)
            if (loc < 0) return
            b.position(0)
            GLES30.glVertexAttribPointer(loc, 3, GLES30.GL_FLOAT, false, 0, b)
            GLES30.glEnableVertexAttribArray(loc)
        }

        private fun fb(a: FloatArray): FloatBuffer =
            ByteBuffer.allocateDirect(a.size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer().apply { put(a); position(0) }
    }
}
