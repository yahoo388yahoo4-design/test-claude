package com.real2sim.capture

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.view.View
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.floor

/**
 * Live coverage for the Camera2 multi-cam mode (no poses on device): which viewing directions the main
 * camera has already recorded, as a yaw x pitch panorama (10 x 10 degree cells) filled by dwell time,
 * from the game rotation vector. Also lists per-lens frame counts passed in via [status].
 */
class CoverageView(ctx: Context) : View(ctx), SensorEventListener {
    private val sm = ctx.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val cols = 36; private val rows = 18
    private val dwell = FloatArray(cols * rows)
    private var lastT = 0L
    @Volatile private var curYaw = 0f
    @Volatile private var curPitch = 0f
    @Volatile var recording = false
    @Volatile var status = ""
    private val r = FloatArray(9)
    private val cell = Paint()
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 13 * resources.displayMetrics.density }
    private val cursor = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.rgb(60, 255, 120); style = Paint.Style.STROKE; strokeWidth = 4f }

    fun start() {
        dwell.fill(0f); lastT = 0L
        sm.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)?.let { sm.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME) }
    }

    fun stop() = sm.unregisterListener(this)

    override fun onSensorChanged(e: SensorEvent) {
        SensorManager.getRotationMatrixFromVector(r, e.values)
        // back camera looks along device -Z; world direction = R * (0, 0, -1)
        val dx = -r[2]; val dy = -r[5]; val dz = -r[8]
        curYaw = ((Math.toDegrees(atan2(dx, dy).toDouble()) + 360) % 360).toFloat()
        curPitch = Math.toDegrees(asin(dz.coerceIn(-1f, 1f)).toDouble()).toFloat()
        if (recording && lastT != 0L) {
            val dt = (e.timestamp - lastT) / 1e9f
            val c = floor(curYaw / 10f).toInt().coerceIn(0, cols - 1)
            val rr = floor((90f - curPitch) / 10f).toInt().coerceIn(0, rows - 1)
            dwell[rr * cols + c] += dt
        }
        lastT = e.timestamp
        postInvalidate()
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onDraw(c: Canvas) {
        val w = width.toFloat(); val h = height.toFloat() * 0.7f
        val cw = w / cols; val ch = h / rows
        var covered = 0
        for (rr in 0 until rows) for (cc in 0 until cols) {
            val d = dwell[rr * cols + cc]
            if (d > 0.3f) covered++
            val a = (d / 2f).coerceIn(0f, 1f)
            cell.color = if (d <= 0f) Color.argb(150, 30, 30, 35) else Color.argb(200, (255 * (1 - a)).toInt(), (80 + 175 * a).toInt(), 90)
            c.drawRect(cc * cw + 1, rr * ch + 1, (cc + 1) * cw - 1, (rr + 1) * ch - 1, cell)
        }
        val x = curYaw / 360f * w; val y = (90f - curPitch) / 180f * h
        c.drawCircle(x, y, 14f, cursor)
        // horizon line
        cursor.alpha = 90; c.drawLine(0f, h / 2, w, h / 2, cursor); cursor.alpha = 255
        c.drawText("Coverage (yaw x pitch, 10 deg cells): ${covered * 100 / (cols * rows)}% of sphere, " +
            "${dwell.withIndex().count { (k, d) -> d > 0.3f && k / cols in 6..11 } * 100 / (cols * 6)}% around the horizon", 8f, h + 22 * resources.displayMetrics.density, text)
        status.lines().forEachIndexed { i, s -> c.drawText(s, 8f, h + (44 + 18 * i) * resources.displayMetrics.density, text) }
    }
}
