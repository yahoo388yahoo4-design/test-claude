package com.real2sim.capture

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.DashPathEffect
import android.graphics.Paint
import android.graphics.Path
import android.graphics.RectF
import android.view.Gravity
import android.widget.LinearLayout
import android.widget.TextView
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

/**
 * Navigation-screen widgets drawn like the iOS NavModeView ones (RadarView, SpeedGauge, DistanceRow).
 * They only display what NavActivity already computes (HudView.State and a few of its fields).
 */

/** iOS `gapColor`: red inside the stop distance, orange inside slow, yellow a bit further, else green. */
fun gapColor(d: Float, stop: Float, slow: Float): Int = when {
    !d.isFinite() -> Ui.GREEN
    d <= stop -> Ui.RED
    d <= slow -> Ui.ORANGE
    d <= slow + 0.6f -> Ui.YELLOW
    else -> Ui.GREEN
}

fun fmtDist(m: Float): String = when {
    !m.isFinite() -> "–"
    m < 1f -> "%.0f cm".format(m * 100)
    else -> "%.2f m".format(m)
}

/** Polar plot of the 72-beam virtual lidar (nearest obstacle per 5°, CCW from the heading), 3 m range. */
class RadarView(ctx: Context) : android.view.View(ctx) {
    var scan: FloatArray? = null
        set(v) { field = v; invalidate() }
    var robotRadius = 0.2f
    var stop = 0.25f
    var slow = 0.6f

    private val d = Ui.dp(ctx, 1f)
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; color = Color.argb(40, 255, 255, 255); strokeWidth = d }
    private val wedge = Paint(Paint.ANTI_ALIAS_FLAG)
    private val dot = Paint(Paint.ANTI_ALIAS_FLAG)
    private val white = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; color = Color.WHITE; strokeWidth = 1.5f * d }
    private val dash = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; color = Ui.RED; strokeWidth = d; pathEffect = DashPathEffect(floatArrayOf(3 * d, 3 * d), 0f) }
    private val txt = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 9 * d; fontFeatureSettings = Ui.TNUM }
    private val path = Path()

    override fun onDraw(c: Canvas) {
        val cx = width / 2f; val cy = height / 2f
        val R = min(width, height) / 2f - 4 * d
        val maxR = 3f
        val k = R / maxR
        for (r in floatArrayOf(0.5f, 1f, 2f, 3f)) c.drawCircle(cx, cy, r * k, ring)
        fun px(bDeg: Double, r: Float) = cx - (sin(Math.toRadians(bDeg)) * r * k).toFloat()
        fun py(bDeg: Double, r: Float) = cy - (cos(Math.toRadians(bDeg)) * r * k).toFloat()
        var nearest = Float.POSITIVE_INFINITY; var nearestB = 0.0
        scan?.let { sc ->
            val n = sc.size
            for (b in 0 until n) {
                val dist = sc[b]
                val bearing = (if (b > n / 2) b - n else b) * 360.0 / n
                val gap = dist - robotRadius
                if (dist.isFinite() && dist < nearest) { nearest = dist; nearestB = bearing }
                // wedge per beam, translucent, like the iOS sector wedges
                val r = if (dist.isFinite()) min(maxR, dist) else maxR
                path.reset(); path.moveTo(cx, cy)
                path.lineTo(px(bearing - 2.5, r), py(bearing - 2.5, r)); path.lineTo(px(bearing + 2.5, r), py(bearing + 2.5, r)); path.close()
                wedge.color = gapColor(gap, stop, slow); wedge.alpha = 70
                c.drawPath(path, wedge)
                if (dist.isFinite() && dist < maxR) {
                    dot.color = gapColor(gap, stop, slow)
                    c.drawCircle(px(bearing, dist), py(bearing, dist), 2 * d, dot)
                }
            }
        }
        val rr = robotRadius * k
        c.drawCircle(cx, cy, rr, white)
        c.drawLine(cx, cy, cx, cy - rr - 6 * d, white)
        if (nearest.isFinite() && nearest < maxR) c.drawLine(cx, cy, px(nearestB, nearest), py(nearestB, nearest), dash)
        c.drawText("radar 3 m", 4 * d, 11 * d, txt)
        val s = "nearest ${fmtDist(nearest - robotRadius)} @ %+.0f°".format(nearestB)
        c.drawText(s, cx - txt.measureText(s) / 2, height - 4 * d, txt)
    }
}

/** Speed dial: open-bottom arc filled to the measured speed, a white tick at the commanded speed. */
class SpeedGauge(ctx: Context) : android.view.View(ctx) {
    var speed = 0f; var cmd = 0f; var maxSpeed = 0.25f; var robot = Float.NaN
    private val d = Ui.dp(ctx, 1f)
    private val track = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 8 * d; strokeCap = Paint.Cap.ROUND; color = Color.argb(50, 255, 255, 255) }
    private val arc = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 8 * d; strokeCap = Paint.Cap.ROUND }
    private val tick = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; strokeWidth = 3 * d; strokeCap = Paint.Cap.ROUND }
    private val big = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 16 * d; typeface = Ui.BOLD; textAlign = Paint.Align.CENTER; fontFeatureSettings = Ui.TNUM }
    private val small = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 9 * d; textAlign = Paint.Align.CENTER; fontFeatureSettings = Ui.TNUM }
    private val r = RectF()

    fun set(speed: Float, cmd: Float, maxSpeed: Float, robot: Float) {
        this.speed = speed; this.cmd = cmd; this.maxSpeed = maxSpeed; this.robot = robot; invalidate()
    }

    override fun onDraw(c: Canvas) {
        val cx = width / 2f; val cy = height / 2f
        val rad = min(width, height) / 2f - 6 * d
        r.set(cx - rad, cy - rad, cx + rad, cy + rad)
        // 252° sweep starting at 144° (bottom-left), opening at the bottom
        c.drawArc(r, 144f, 252f, false, track)
        val frac = (abs(speed) / maxOf(0.05f, maxSpeed)).coerceIn(0f, 1f)
        arc.color = if (speed < 0) Ui.ORANGE else Ui.CYAN
        if (frac > 0.003f) c.drawArc(r, 144f, 252f * frac, false, arc)
        val cf = (abs(cmd) / maxOf(0.05f, maxSpeed)).coerceIn(0f, 1f)
        val a = Math.toRadians(144.0 + 252.0 * cf)
        c.drawLine(cx + (cos(a) * (rad - 7 * d)).toFloat(), cy + (sin(a) * (rad - 7 * d)).toFloat(),
            cx + (cos(a) * (rad + 5 * d)).toFloat(), cy + (sin(a) * (rad + 5 * d)).toFloat(), tick)
        c.drawText("%.2f".format(speed), cx, cy + 2 * d, big)
        c.drawText("m/s", cx, cy + 13 * d, small)
        small.color = Ui.SECONDARY
        c.drawText(if (robot.isFinite()) "robot %.2f".format(robot) else "cmd %.2f".format(cmd), cx, cy + 24 * d, small)
        small.color = Color.WHITE
    }
}

/** Front / left / right / nearest chips (iOS DistanceRow). */
class DistanceRow(ctx: Context) : LinearLayout(ctx) {
    private val chips = listOf("front", "left", "right", "nearest").map { name ->
        val v = Ui.label(ctx, "–", 12f, Color.WHITE, Ui.BOLD, digits = true).apply { gravity = Gravity.CENTER }
        val n = Ui.label(ctx, name, 9f, Color.WHITE).apply { gravity = Gravity.CENTER }
        val box = LinearLayout(ctx).apply {
            orientation = VERTICAL; gravity = Gravity.CENTER
            setPadding(0, Ui.dpi(ctx, 3), 0, Ui.dpi(ctx, 3))
            addView(v); addView(n)
        }
        Triple(box, v, n)
    }

    init {
        orientation = HORIZONTAL
        chips.forEachIndexed { i, (box, _, _) ->
            addView(box, LayoutParams(0, -2, 1f).apply { if (i > 0) marginStart = Ui.dpi(ctx, 6) })
        }
    }

    fun set(front: Float, left: Float, right: Float, nearest: Float, stop: Float, slow: Float) {
        listOf(front, left, right, nearest).forEachIndexed { i, dd ->
            val (box, v, _) = chips[i]
            (v as TextView).text = fmtDist(dd)
            val col = gapColor(dd, stop, slow)
            box.background = Ui.rounded(Color.argb(90, Color.red(col), Color.green(col), Color.blue(col)), Ui.dp(context, 6f))
        }
    }
}
