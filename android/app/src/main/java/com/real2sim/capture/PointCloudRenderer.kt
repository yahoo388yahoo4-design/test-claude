package com.real2sim.capture

import android.opengl.GLES20
import android.opengl.Matrix
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer

/**
 * Draws the accumulated map point cloud over the camera image, coloured by height above the floor
 * (blue near the floor -> green -> red at 2 m), plus optional line strips (planned path) on the floor.
 * Re-uploads the cloud only when the map changed and at most every [uploadMs].
 */
class PointCloudRenderer(private val map: MapBuilder, private val uploadMs: Long = 400) {
    private var program = 0
    private var aPos = 0
    private var uMvp = 0
    private var uFloor = 0
    private var uColor = 0
    private var uMode = 0
    private var uSize = 0
    private var vbo = 0
    private var count = 0
    private var uploadedVersion = -1
    private var lastUpload = 0L
    private val scratch = FloatArray(map.maxPoints * 3)
    private var fb: FloatBuffer = ByteBuffer.allocateDirect(map.maxPoints * 12).order(ByteOrder.nativeOrder()).asFloatBuffer()
    private val mvp = FloatArray(16)
    @Volatile var visible = true

    fun create() {
        program = link(
            """
            uniform mat4 u_Mvp; uniform float u_Floor; uniform float u_Size; attribute vec3 a_Pos; varying float v_H;
            void main() { gl_Position = u_Mvp * vec4(a_Pos, 1.0); gl_PointSize = u_Size; v_H = a_Pos.y - u_Floor; }
            """.trimIndent(),
            """
            precision mediump float; varying float v_H; uniform vec4 u_Color; uniform int u_Mode;
            void main() {
              if (u_Mode == 1) { gl_FragColor = u_Color; return; }
              float h = clamp(v_H / 2.0, 0.0, 1.0);
              vec3 c = h < 0.5 ? mix(vec3(0.1, 0.4, 1.0), vec3(0.1, 1.0, 0.4), h * 2.0) : mix(vec3(0.1, 1.0, 0.4), vec3(1.0, 0.25, 0.2), (h - 0.5) * 2.0);
              gl_FragColor = vec4(c, 0.85);
            }
            """.trimIndent())
        aPos = GLES20.glGetAttribLocation(program, "a_Pos")
        uMvp = GLES20.glGetUniformLocation(program, "u_Mvp")
        uFloor = GLES20.glGetUniformLocation(program, "u_Floor")
        uColor = GLES20.glGetUniformLocation(program, "u_Color")
        uMode = GLES20.glGetUniformLocation(program, "u_Mode")
        uSize = GLES20.glGetUniformLocation(program, "u_Size")
        val b = IntArray(1); GLES20.glGenBuffers(1, b, 0); vbo = b[0]
    }

    fun draw(view: FloatArray, proj: FloatArray, path: List<FloatArray>?, goal: FloatArray?) {
        Matrix.multiplyMM(mvp, 0, proj, 0, view, 0)
        val now = android.os.SystemClock.elapsedRealtime()
        if (visible && map.version != uploadedVersion && now - lastUpload > uploadMs) {
            count = map.copyPoints(scratch)
            fb.position(0); fb.put(scratch, 0, count * 3); fb.position(0)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
            GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, count * 12, fb, GLES20.GL_DYNAMIC_DRAW)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
            uploadedVersion = map.version; lastUpload = now
        }
        val floor = if (map.floorY.isNaN()) -1.4f else map.floorY
        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
        GLES20.glUniform1f(uFloor, floor)
        GLES20.glEnable(GLES20.GL_BLEND)
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA)
        if (visible && count > 0) {
            GLES20.glUniform1i(uMode, 0); GLES20.glUniform1f(uSize, 5f)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
            GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, 0)
            GLES20.glEnableVertexAttribArray(aPos)
            GLES20.glDrawArrays(GLES20.GL_POINTS, 0, count)
            GLES20.glDisableVertexAttribArray(aPos)
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        }
        // planned path as a thick line just above the floor, goal as a big point
        val y = floor + 0.02f
        if (path != null && path.size >= 2) {
            val pts = FloatArray(path.size * 3)
            path.forEachIndexed { k, p -> pts[k * 3] = p[0]; pts[k * 3 + 1] = y; pts[k * 3 + 2] = p[1] }
            lines(pts, path.size, floatArrayOf(0.2f, 0.9f, 1f, 0.95f), GLES20.GL_LINE_STRIP, 1f)
            lines(pts, path.size, floatArrayOf(0.2f, 0.9f, 1f, 0.95f), GLES20.GL_POINTS, 14f)
        }
        if (goal != null) lines(floatArrayOf(goal[0], y, goal[1], goal[0], y + 0.6f, goal[1]), 2, floatArrayOf(1f, 0.8f, 0.1f, 1f), GLES20.GL_LINES, 1f)
        if (goal != null) lines(floatArrayOf(goal[0], y + 0.6f, goal[1]), 1, floatArrayOf(1f, 0.8f, 0.1f, 1f), GLES20.GL_POINTS, 30f)
        GLES20.glDisable(GLES20.GL_BLEND)
    }

    private fun lines(pts: FloatArray, n: Int, color: FloatArray, mode: Int, size: Float) {
        val b = ByteBuffer.allocateDirect(pts.size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer().apply { put(pts); position(0) }
        GLES20.glUniform1i(uMode, 1)
        GLES20.glUniform4fv(uColor, 1, color, 0)
        GLES20.glUniform1f(uSize, size)
        GLES20.glLineWidth(8f)
        GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, b)
        GLES20.glEnableVertexAttribArray(aPos)
        GLES20.glDrawArrays(mode, 0, n)
        GLES20.glDisableVertexAttribArray(aPos)
    }

    private fun link(vs: String, fs: String): Int {
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
