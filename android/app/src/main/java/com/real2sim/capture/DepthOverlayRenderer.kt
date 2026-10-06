package com.real2sim.capture

import android.opengl.GLES20
import com.google.ar.core.Coordinates2d
import com.google.ar.core.Frame
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/**
 * Live depth view: the newest *filtered* depth map (DepthFusion.liveDepth, turbo colours, rejected pixels
 * transparent) blended over the camera image, registered to it through ARCore's texture-normalised
 * coordinates (ARCore depth covers the camera texture's field of view, as in the hello_ar_depth sample).
 */
class DepthOverlayRenderer(private val source: () -> DepthFusion.LiveDepth?) {
    private var program = 0
    private var aPos = 0; private var aUv = 0; private var uTex = 0; private var uAlpha = 0
    private var tex = 0
    private var shown = -1
    private var texW = 0; private var texH = 0
    private var uvReady = false
    private var rgba = ByteArray(0)
    private val ndc: FloatBuffer = buf(floatArrayOf(-1f, -1f, 1f, -1f, -1f, 1f, 1f, 1f))
    private val uv: FloatBuffer = buf(FloatArray(8))
    @Volatile var alpha = 0.6f

    fun create() {
        program = GlUtil.link(
            """
            attribute vec4 a_Pos; attribute vec2 a_Uv; varying vec2 v_Uv;
            void main() { gl_Position = a_Pos; v_Uv = a_Uv; }
            """.trimIndent(),
            """
            precision mediump float; varying vec2 v_Uv; uniform sampler2D u_Tex; uniform float u_Alpha;
            void main() { vec4 c = texture2D(u_Tex, v_Uv); gl_FragColor = vec4(c.rgb, c.a * u_Alpha); }
            """.trimIndent())
        aPos = GLES20.glGetAttribLocation(program, "a_Pos")
        aUv = GLES20.glGetAttribLocation(program, "a_Uv")
        uTex = GLES20.glGetUniformLocation(program, "u_Tex")
        uAlpha = GLES20.glGetUniformLocation(program, "u_Alpha")
        val t = IntArray(1); GLES20.glGenTextures(1, t, 0); tex = t[0]
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tex)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_S, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_WRAP_T, GLES20.GL_CLAMP_TO_EDGE)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0)
        shown = -1; texW = 0; texH = 0; uvReady = false
    }

    fun draw(frame: Frame) {
        if (frame.hasDisplayGeometryChanged() || !uvReady) {
            ndc.position(0); uv.position(0)
            frame.transformCoordinates2d(Coordinates2d.OPENGL_NORMALIZED_DEVICE_COORDINATES, ndc, Coordinates2d.TEXTURE_NORMALIZED, uv)
            uvReady = true
        }
        val d = source() ?: return
        GLES20.glActiveTexture(GLES20.GL_TEXTURE0)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, tex)
        if (d.version != shown) {
            rgba = DepthViz.toRgba(d.argb, rgba.takeIf { it.size == d.argb.size * 4 })
            val b = ByteBuffer.allocateDirect(rgba.size).order(ByteOrder.nativeOrder()).put(rgba).also { it.position(0) }
            GLES20.glPixelStorei(GLES20.GL_UNPACK_ALIGNMENT, 1)
            if (d.w != texW || d.h != texH) {
                GLES20.glTexImage2D(GLES20.GL_TEXTURE_2D, 0, GLES20.GL_RGBA, d.w, d.h, 0, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, b)
                texW = d.w; texH = d.h
            } else GLES20.glTexSubImage2D(GLES20.GL_TEXTURE_2D, 0, 0, 0, d.w, d.h, GLES20.GL_RGBA, GLES20.GL_UNSIGNED_BYTE, b)
            shown = d.version
        }
        GLES20.glDisable(GLES20.GL_DEPTH_TEST)
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)
        GLES20.glUseProgram(program)
        GLES20.glUniform1i(uTex, 0)
        GLES20.glUniform1f(uAlpha, alpha)
        ndc.position(0); uv.position(0)
        GLES20.glVertexAttribPointer(aPos, 2, GLES20.GL_FLOAT, false, 0, ndc)
        GLES20.glVertexAttribPointer(aUv, 2, GLES20.GL_FLOAT, false, 0, uv)
        GLES20.glEnableVertexAttribArray(aPos); GLES20.glEnableVertexAttribArray(aUv)
        GLES20.glDrawArrays(GLES20.GL_TRIANGLE_STRIP, 0, 4)
        GLES20.glDisableVertexAttribArray(aPos); GLES20.glDisableVertexAttribArray(aUv)
        GLES20.glDisable(GLES20.GL_BLEND)
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, 0)
    }

    private fun buf(a: FloatArray): FloatBuffer =
        ByteBuffer.allocateDirect(a.size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer().apply { put(a); position(0) }
}

object GlUtil {
    fun link(vs: String, fs: String): Int {
        fun compile(type: Int, src: String): Int {
            val s = GLES20.glCreateShader(type)
            GLES20.glShaderSource(s, src); GLES20.glCompileShader(s)
            val ok = IntArray(1); GLES20.glGetShaderiv(s, GLES20.GL_COMPILE_STATUS, ok, 0)
            check(ok[0] != 0) { "shader: " + GLES20.glGetShaderInfoLog(s) }
            return s
        }
        val p = GLES20.glCreateProgram()
        GLES20.glAttachShader(p, compile(GLES20.GL_VERTEX_SHADER, vs))
        GLES20.glAttachShader(p, compile(GLES20.GL_FRAGMENT_SHADER, fs))
        GLES20.glLinkProgram(p)
        return p
    }
}
