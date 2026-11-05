package com.jakubmini.psom.gl

import android.opengl.GLES11Ext
import android.opengl.GLES30
import com.google.ar.core.Coordinates2d
import com.google.ar.core.Frame
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/** Draws ARCore's camera image behind everything. */
class BackgroundRenderer {
    var textureId = -1
        private set
    private var program = 0
    private val quad = floats(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f)
    private val uv = floats(0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f)

    fun create() {
        val t = IntArray(1)
        GLES30.glGenTextures(1, t, 0)
        textureId = t[0]
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_CLAMP_TO_EDGE)
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_CLAMP_TO_EDGE)
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_LINEAR)
        GLES30.glTexParameteri(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_LINEAR)
        program = Shaders.program(
            """
            attribute vec4 a_pos; attribute vec2 a_uv; varying vec2 v_uv;
            void main() { gl_Position = a_pos; v_uv = a_uv; }
            """,
            """
            #extension GL_OES_EGL_image_external : require
            precision mediump float; varying vec2 v_uv; uniform samplerExternalOES u_tex;
            void main() { gl_FragColor = texture2D(u_tex, v_uv); }
            """,
        )
    }

    fun draw(frame: Frame) {
        if (frame.hasDisplayGeometryChanged())
            frame.transformCoordinates2d(Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES, quad, Coordinates2d.TEXTURE_NORMALIZED, uv)
        if (frame.timestamp == 0L) return
        quad.position(0)
        uv.position(0)
        GLES30.glDisable(GLES30.GL_DEPTH_TEST)
        GLES30.glDepthMask(false)
        GLES30.glUseProgram(program)
        GLES30.glActiveTexture(GLES30.GL_TEXTURE0)
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId)
        val pos = GLES30.glGetAttribLocation(program, "a_pos")
        val tex = GLES30.glGetAttribLocation(program, "a_uv")
        GLES30.glVertexAttribPointer(pos, 2, GLES30.GL_FLOAT, false, 0, quad)
        GLES30.glVertexAttribPointer(tex, 2, GLES30.GL_FLOAT, false, 0, uv)
        GLES30.glEnableVertexAttribArray(pos)
        GLES30.glEnableVertexAttribArray(tex)
        GLES30.glDrawArrays(GLES30.GL_TRIANGLE_STRIP, 0, 4)
        GLES30.glDisableVertexAttribArray(pos)
        GLES30.glDisableVertexAttribArray(tex)
        GLES30.glDepthMask(true)
    }

    private fun floats(vararg v: Float): FloatBuffer =
        ByteBuffer.allocateDirect(v.size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer().apply { put(v); position(0) }
}

object Shaders {
    fun program(vs: String, fs: String): Int {
        fun compile(type: Int, src: String): Int {
            val s = GLES30.glCreateShader(type)
            GLES30.glShaderSource(s, src.trimIndent())
            GLES30.glCompileShader(s)
            val ok = IntArray(1)
            GLES30.glGetShaderiv(s, GLES30.GL_COMPILE_STATUS, ok, 0)
            check(ok[0] != 0) { "shader: " + GLES30.glGetShaderInfoLog(s) }
            return s
        }
        val p = GLES30.glCreateProgram()
        GLES30.glAttachShader(p, compile(GLES30.GL_VERTEX_SHADER, vs))
        GLES30.glAttachShader(p, compile(GLES30.GL_FRAGMENT_SHADER, fs))
        GLES30.glLinkProgram(p)
        return p
    }
}
