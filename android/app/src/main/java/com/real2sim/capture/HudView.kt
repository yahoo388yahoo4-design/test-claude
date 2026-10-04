package com.real2sim.capture

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.view.View
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

/**
 * Head-up display drawn over the camera view:
 *  - stats block (tracking, depth, map size, speeds, distances, robot link),
 *  - speed dial (phone speed from ARCore poses; robot odometry speed as a second needle),
 *  - "virtual lidar" polar plot of the nearest obstacle per 5 degrees around the robot,
 *  - front / left / right nearest-obstacle bars with colour coding,
 *  - guidance arrow + instruction text, and the planned path projected into the camera image.
 */
class HudView(ctx: Context) : View(ctx) {
    class State {
        var lines: List<String> = emptyList()
        var speed = 0f                 // m/s
        var robotSpeed = Float.NaN     // m/s
        var vMax = 0.5f
        var scan: FloatArray? = null   // per bin, CCW from heading, m
        var front = Float.POSITIVE_INFINITY
        var left = Float.POSITIVE_INFINITY
        var right = Float.POSITIVE_INFINITY
        var bearing = Float.NaN        // rad to the next path point, CCW positive
        var instruction = ""
        var banner = ""                // big warning (blocked, tracking lost, arrived)
        var bannerColor = Color.RED
        var pathPx: FloatArray? = null // projected path, x,y pairs (screen px); NaN breaks the line
        var goalPx: FloatArray? = null
        var nav = false
    }

    @Volatile var state = State()
        set(v) { field = v; postInvalidate() }

    private val dp = resources.displayMetrics.density
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 13 * dp; typeface = android.graphics.Typeface.MONOSPACE }
    private val big = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 22 * dp; isFakeBoldText = true; textAlign = Paint.Align.CENTER }
    private val bg = Paint().apply { color = Color.argb(140, 0, 0, 0) }
    private val stroke = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 2 * dp; color = Color.WHITE }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG)
    private val pathPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 6 * dp; color = Color.argb(200, 40, 220, 255); strokeCap = Paint.Cap.ROUND }

    private fun distColor(d: Float) = when {
        d < 0.3f -> Color.rgb(255, 50, 40)
        d < 0.8f -> Color.rgb(255, 170, 0)
        d < 1.5f -> Color.rgb(250, 240, 80)
        else -> Color.rgb(80, 230, 120)
    }

    private fun fmt(d: Float) = if (!d.isFinite()) "  --  " else if (d < 1f) "%3.0f cm".format(d * 100) else "%4.2f m".format(d)

    override fun onDraw(c: Canvas) {
        val st = state
        val w = width.toFloat(); val h = height.toFloat()

        // projected path + goal in the camera image
        st.pathPx?.let { p ->
            val path = Path(); var pen = false
            var k = 0
            while (k + 1 < p.size) {
                val x = p[k]; val y = p[k + 1]
                if (x.isNaN()) pen = false else if (!pen) { path.moveTo(x, y); pen = true } else path.lineTo(x, y)
                k += 2
            }
            c.drawPath(path, pathPaint)
        }
        st.goalPx?.let { g -> if (!g[0].isNaN()) { fill.color = Color.rgb(255, 200, 20); c.drawCircle(g[0], g[1], 14 * dp, fill); c.drawText("GOAL", g[0], g[1] - 20 * dp, big) } }

        // stats block, top-left (below the controls when not navigating is handled by layout margins)
        if (st.lines.isNotEmpty()) {
            val lh = text.textSize * 1.25f
            val bw = st.lines.maxOf { text.measureText(it) } + 16 * dp
            c.drawRoundRect(RectF(8 * dp, 8 * dp, 8 * dp + bw, 8 * dp + lh * st.lines.size + 10 * dp), 8 * dp, 8 * dp, bg)
            st.lines.forEachIndexed { i, s -> c.drawText(s, 16 * dp, 8 * dp + lh * (i + 1), text) }
        }

        // virtual lidar polar plot, bottom-left
        val R = min(w, h) * 0.17f
        val cx = 12 * dp + R; val cy = h - 12 * dp - R
        c.drawCircle(cx, cy, R + 6 * dp, bg)
        stroke.color = Color.argb(90, 255, 255, 255); stroke.strokeWidth = 1 * dp
        val rangeM = 3f
        for (ring in 1..3) c.drawCircle(cx, cy, R * ring / rangeM, stroke)
        text.textSize = 10 * dp
        c.drawText("1m", cx + 2, cy - R / rangeM, text); c.drawText("2m", cx + 2, cy - 2 * R / rangeM, text)
        text.textSize = 13 * dp
        st.scan?.let { sc ->
            val n = sc.size
            for (b in 0 until n) {
                val d = sc[b]; if (!d.isFinite()) continue
                val bearing = Math.toRadians((if (b > n / 2) b - n else b) * 360.0 / n)
                val r = min(d, rangeM) / rangeM * R
                // forward = up on screen, CCW (left) bearing = to the left
                val x = cx - sin(bearing).toFloat() * r; val y = cy - cos(bearing).toFloat() * r
                fill.color = distColor(d)
                c.drawCircle(x, y, 3.5f * dp, fill)
            }
        }
        fill.color = Color.rgb(60, 255, 120)
        val tri = Path().apply { moveTo(cx, cy - 10 * dp); lineTo(cx - 7 * dp, cy + 7 * dp); lineTo(cx + 7 * dp, cy + 7 * dp); close() }
        c.drawPath(tri, fill)
        c.drawText("LIDAR (depth+map)", cx - R, cy - R - 10 * dp, text)

        // obstacle bars: left / front / right, next to the polar plot
        val bx = cx + R + 20 * dp; val bh = R * 1.6f; val bwid = 22 * dp
        listOf("L" to st.left, "F" to st.front, "R" to st.right).forEachIndexed { i, (lab, d) ->
            val x = bx + i * (bwid + 14 * dp)
            c.drawRect(x, cy + R - bh, x + bwid, cy + R, bg)
            val frac = if (d.isFinite()) (1f - min(d, 2f) / 2f) else 0f
            fill.color = distColor(d)
            c.drawRect(x, cy + R - bh * frac, x + bwid, cy + R, fill)
            c.drawText(lab, x + 6 * dp, cy + R - bh - 6 * dp, text)
        }
        text.textSize = 12 * dp
        c.drawText("front ${fmt(st.front)}", bx, cy - R - 30 * dp, text)
        text.textSize = 13 * dp

        // speed dial, bottom-right
        val dr = R * 0.9f
        val dx = w - 12 * dp - dr; val dy = h - 12 * dp - dr * 0.3f
        c.drawArc(RectF(dx - dr, dy - dr, dx + dr, dy + dr), 180f, 180f, true, bg)
        stroke.color = Color.WHITE; stroke.strokeWidth = 2 * dp
        c.drawArc(RectF(dx - dr, dy - dr, dx + dr, dy + dr), 180f, 180f, false, stroke)
        fun needle(v: Float, col: Int) {
            val f = (v / st.vMax).coerceIn(0f, 1f)
            val a = Math.PI * (1 - f)
            stroke.color = col; stroke.strokeWidth = 4 * dp
            c.drawLine(dx, dy, dx + (cos(a) * dr * 0.9f).toFloat(), dy - (sin(a) * dr * 0.9f).toFloat(), stroke)
        }
        needle(st.speed, Color.rgb(60, 255, 120))
        if (!st.robotSpeed.isNaN()) needle(abs(st.robotSpeed), Color.rgb(255, 120, 255))
        c.drawText("%.0f cm/s".format(st.speed * 100), dx - dr * 0.6f, dy - 6 * dp, text)
        c.drawText("speed (max %.0f)".format(st.vMax * 100), dx - dr, dy - dr - 6 * dp, text)

        // guidance arrow + instruction, top-centre-ish
        if (st.nav && !st.bearing.isNaN()) {
            val ax = w / 2; val ay = h * 0.42f; val L = 50 * dp
            c.save(); c.rotate(-Math.toDegrees(st.bearing.toDouble()).toFloat(), ax, ay)
            fill.color = Color.argb(220, 40, 220, 255)
            val arrow = Path().apply {
                moveTo(ax, ay - L); lineTo(ax - L * 0.6f, ay); lineTo(ax - L * 0.22f, ay); lineTo(ax - L * 0.22f, ay + L)
                lineTo(ax + L * 0.22f, ay + L); lineTo(ax + L * 0.22f, ay); lineTo(ax + L * 0.6f, ay); close()
            }
            c.drawPath(arrow, fill)
            c.restore()
        }
        if (st.instruction.isNotEmpty()) {
            c.drawRoundRect(RectF(w / 2 - big.measureText(st.instruction) / 2 - 12 * dp, h * 0.42f + 60 * dp, w / 2 + big.measureText(st.instruction) / 2 + 12 * dp, h * 0.42f + 96 * dp), 8 * dp, 8 * dp, bg)
            c.drawText(st.instruction, w / 2, h * 0.42f + 88 * dp, big)
        }
        if (st.banner.isNotEmpty()) {
            fill.color = st.bannerColor; fill.alpha = 200
            c.drawRect(0f, h * 0.30f, w, h * 0.30f + 44 * dp, fill)
            c.drawText(st.banner, w / 2, h * 0.30f + 31 * dp, big)
        }
    }
}
