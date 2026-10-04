package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.view.MotionEvent
import android.view.View
import kotlin.math.hypot
import kotlin.math.min

/** Thumb joystick for manual driving: up = forward, left = turn left. Reports (fwd, left) in -1..1, (0, 0) on release. */
class JoystickView(ctx: Context) : View(ctx) {
    var onChange: ((Float, Float) -> Unit)? = null
    private var kx = 0f; private var ky = 0f
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 4f; color = Color.argb(200, 255, 255, 255) }
    private val base = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.argb(90, 0, 0, 0) }
    private val knob = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.argb(220, 60, 160, 255) }

    override fun onDraw(c: Canvas) {
        val r = min(width, height) / 2f - 4f; val cx = width / 2f; val cy = height / 2f
        c.drawCircle(cx, cy, r, base); c.drawCircle(cx, cy, r, ring)
        c.drawLine(cx - r, cy, cx + r, cy, ring); c.drawLine(cx, cy - r, cx, cy + r, ring)
        c.drawCircle(cx + kx * r, cy + ky * r, r * 0.3f, knob)
    }

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(e: MotionEvent): Boolean {
        val r = min(width, height) / 2f - 4f
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                var dx = (e.x - width / 2f) / r; var dy = (e.y - height / 2f) / r
                val n = hypot(dx, dy); if (n > 1f) { dx /= n; dy /= n }
                kx = dx; ky = dy
            }
            else -> { kx = 0f; ky = 0f }
        }
        invalidate()
        onChange?.invoke(-ky, -kx)
        return true
    }
}
