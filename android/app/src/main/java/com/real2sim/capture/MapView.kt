package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.view.MotionEvent
import android.view.View
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/**
 * Top-down map: occupancy grid (unknown dark, free light, occupied red), trajectory, robot arrow,
 * virtual-lidar rays, planned path and goal. North-up in ARCore world (+x right, +z down). Tap to pick
 * a navigation goal ([onTap] gets world x, z).
 */
class MapView(ctx: Context) : View(ctx) {
    var onTap: ((Float, Float) -> Unit)? = null

    @Volatile private var bmp: Bitmap? = null
    @Volatile private var rect: FloatArray? = null       // world x0, z0, x1, z1 of bmp
    @Volatile private var savedBmp: Bitmap? = null
    @Volatile private var savedRect: FloatArray? = null
    @Volatile private var pose: FloatArray? = null        // x, z, heading
    @Volatile private var traj: FloatArray = FloatArray(0) // x,z pairs
    @Volatile var path: List<FloatArray>? = null
    @Volatile var goal: FloatArray? = null
    @Volatile private var scan: FloatArray? = null
    @Volatile var title = ""

    private val pBmp = Paint(Paint.FILTER_BITMAP_FLAG)
    private val pSaved = Paint(Paint.FILTER_BITMAP_FLAG).apply { alpha = 110 }
    private val pTraj = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(255, 210, 60); strokeWidth = 3f; style = Paint.Style.STROKE }
    private val pPath = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(40, 220, 255); strokeWidth = 6f; style = Paint.Style.STROKE }
    private val pGoal = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(255, 200, 20) }
    private val pRobot = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(60, 255, 120) }
    private val pRay = Paint(Paint.ANTI_ALIAS_FLAG).apply { strokeWidth = 2f }
    private val pBg = Paint().apply { color = Color.argb(170, 0, 0, 0) }
    private val pText = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 26f }
    private val pGrid = Paint().apply { color = Color.argb(60, 255, 255, 255); strokeWidth = 1f }

    // world -> screen
    private var s = 50f; private var ox = 0f; private var oz = 0f

    fun setMap(b: Bitmap?, r: FloatArray?) { bmp = b; rect = r; postInvalidate() }
    fun setSaved(b: Bitmap?, r: FloatArray?) { savedBmp = b; savedRect = r; postInvalidate() }
    fun setPose(x: Float, z: Float, heading: Float, t: List<FloatArray>) {
        pose = floatArrayOf(x, z, heading)
        val a = FloatArray(min(t.size, 4000) * 2)
        val off = max(0, t.size - a.size / 2)
        for (k in 0 until a.size / 2) { val p = t[off + k]; a[2 * k] = p[0]; a[2 * k + 1] = p[2] }
        traj = a
        postInvalidate()
    }
    fun setScan(bins: FloatArray?) { scan = bins?.copyOf(); postInvalidate() }

    private fun fit() {
        var x0 = Float.POSITIVE_INFINITY; var z0 = Float.POSITIVE_INFINITY; var x1 = Float.NEGATIVE_INFINITY; var z1 = Float.NEGATIVE_INFINITY
        fun add(x: Float, z: Float) { x0 = min(x0, x); x1 = max(x1, x); z0 = min(z0, z); z1 = max(z1, z) }
        rect?.let { add(it[0], it[1]); add(it[2], it[3]) }
        savedRect?.let { add(it[0], it[1]); add(it[2], it[3]) }
        pose?.let { add(it[0] - 1.5f, it[1] - 1.5f); add(it[0] + 1.5f, it[1] + 1.5f) }
        goal?.let { add(it[0], it[1]) }
        if (!x0.isFinite()) { x0 = -2f; z0 = -2f; x1 = 2f; z1 = 2f }
        val w = width.toFloat(); val h = height.toFloat()
        s = min(w / (x1 - x0), h / (z1 - z0)) * 0.95f
        ox = w / 2 - (x0 + x1) / 2 * s; oz = h / 2 - (z0 + z1) / 2 * s
    }
    private fun sx(x: Float) = ox + x * s
    private fun sz(z: Float) = oz + z * s

    override fun onDraw(c: Canvas) {
        c.drawRect(0f, 0f, width.toFloat(), height.toFloat(), pBg)
        fit()
        // 1 m grid
        val gx0 = ((-ox) / s).toInt() - 1; val gx1 = ((width - ox) / s).toInt() + 1
        for (gx in gx0..gx1) c.drawLine(sx(gx.toFloat()), 0f, sx(gx.toFloat()), height.toFloat(), pGrid)
        val gz0 = ((-oz) / s).toInt() - 1; val gz1 = ((height - oz) / s).toInt() + 1
        for (gz in gz0..gz1) c.drawLine(0f, sz(gz.toFloat()), width.toFloat(), sz(gz.toFloat()), pGrid)
        savedBmp?.let { b -> savedRect?.let { r -> c.drawBitmap(b, null, RectF(sx(r[0]), sz(r[1]), sx(r[2]), sz(r[3])), pSaved) } }
        bmp?.let { b -> rect?.let { r -> c.drawBitmap(b, null, RectF(sx(r[0]), sz(r[1]), sx(r[2]), sz(r[3])), pBmp) } }
        val t = traj
        if (t.size >= 4) {
            val p = Path(); p.moveTo(sx(t[0]), sz(t[1]))
            var k = 2; while (k < t.size) { p.lineTo(sx(t[k]), sz(t[k + 1])); k += 2 }
            c.drawPath(p, pTraj)
        }
        path?.let { pa -> if (pa.size >= 2) {
            val p = Path(); p.moveTo(sx(pa[0][0]), sz(pa[0][1])); pa.drop(1).forEach { p.lineTo(sx(it[0]), sz(it[1])) }
            c.drawPath(p, pPath)
        } }
        goal?.let { g -> c.drawCircle(sx(g[0]), sz(g[1]), 14f, pGoal); c.drawText("goal", sx(g[0]) + 16, sz(g[1]) - 8, pText) }
        pose?.let { ps ->
            val x = sx(ps[0]); val z = sz(ps[1]); val hd = ps[2]
            scan?.let { sc ->
                for (b in sc.indices) {
                    val r = sc[b]; if (!r.isFinite()) continue
                    val a = hd - Math.toRadians((if (b > sc.size / 2) b - sc.size else b) * 360.0 / sc.size).toFloat()
                    pRay.color = when { r < 0.3f -> Color.RED; r < 0.8f -> Color.rgb(255, 160, 0); else -> Color.rgb(80, 200, 255) }
                    pRay.alpha = 150
                    c.drawLine(x, z, x + cos(a) * r * s, z + sin(a) * r * s, pRay)
                }
            }
            val path = Path()
            val L = 26f
            path.moveTo(x + cos(hd) * L, z + sin(hd) * L)
            path.lineTo(x + cos(hd + 2.5f) * L * 0.7f, z + sin(hd + 2.5f) * L * 0.7f)
            path.lineTo(x + cos(hd - 2.5f) * L * 0.7f, z + sin(hd - 2.5f) * L * 0.7f)
            path.close()
            c.drawPath(path, pRobot)
        }
        if (title.isNotEmpty()) c.drawText(title, 10f, 30f, pText)
        c.drawText("1 m grid", 10f, height - 10f, pText)
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (e.action == MotionEvent.ACTION_UP && onTap != null) {
            onTap?.invoke((e.x - ox) / s, (e.y - oz) / s)
            performClick()
        }
        return onTap != null
    }

    override fun performClick(): Boolean { super.performClick(); return true }
}
