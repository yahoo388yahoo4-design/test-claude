package com.real2sim.capture

import android.app.Activity
import android.content.Context
import android.graphics.BitmapFactory
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.opengl.Matrix
import android.util.Log
import android.view.Gravity
import android.view.MotionEvent
import android.view.View
import android.widget.Button
import android.widget.FrameLayout
import android.widget.HorizontalScrollView
import android.widget.LinearLayout
import android.widget.TextView
import com.google.ar.core.AugmentedImage
import com.google.ar.core.AugmentedImageDatabase
import com.google.ar.core.Config
import com.google.ar.core.DepthPoint
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Point
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import org.json.JSONObject
import java.io.File
import java.util.concurrent.ConcurrentLinkedQueue

/**
 * Marks the ESP32 sensing nodes in the scan while recording (mode A):
 *  - automatically: the printed node tags (sensing/esp32/tags, bundled in assets/esp_tags) are ARCore
 *    augmented images; every tracked sighting of tag N updates node N (median of the sightings);
 *  - by tapping the ESP on screen: ARCore hit test (depth point, plane or feature point) at the tap;
 *    when nothing is hit, the tap's ray is kept and two rays from different angles are intersected.
 * Results go to <session>/esp_nodes.json in the session's world frame. All ARCore calls run on the GL thread.
 */
class EspLive(private val activity: Activity) {
    var store: EspNodeStore? = null
        private set
    @Volatile var selected = 1
    @Volatile var overlay: EspLiveOverlay? = null
    private val taps = ConcurrentLinkedQueue<FloatArray>()     // x, y (view px), node id
    private val sightings = HashMap<Int, ArrayList<DoubleArray>>()
    private val rays = HashMap<Int, ArrayList<Pair<DoubleArray, DoubleArray>>>()
    private var lastSave = 0L
    private var tagCount = 0

    /** Adds the tag images to the ARCore config (before session.configure). Returns a status line. */
    fun configure(session: Session, cfg: Config): String {
        return try {
            val db = AugmentedImageDatabase(session)
            val width = try {
                JSONObject(activity.assets.open("esp_tags/esp_tags.json").bufferedReader().readText()).getDouble("tag_size_m").toFloat()
            } catch (_: Exception) { EspMath.TAG_SIZE_M }
            for (i in 1..EspMath.NODE_COUNT) {
                val bmp = activity.assets.open("esp_tags/esp_$i.png").use { BitmapFactory.decodeStream(it) } ?: continue
                db.addImage("esp_$i", bmp, width)
                tagCount++
            }
            cfg.augmentedImageDatabase = db
            "ESP tags: $tagCount"
        } catch (e: Exception) {
            Log.w("EspLive", "tag database", e)
            "ESP tags unavailable: ${e.javaClass.simpleName}"
        }
    }

    fun begin(dir: File) {
        store = EspNodeStore.forSession(dir)
        synchronized(sightings) { sightings.clear() }
        rays.clear()
        pushOverlay(emptyList(), null)
    }

    fun finish() {
        store?.let { if (it.dirty) it.save() }
    }

    fun tap(x: Float, y: Float) { taps.add(floatArrayOf(x, y, selected.toFloat())) }

    /** GL thread, every frame while recording; [view]/[proj] are this frame's camera matrices (column-major). */
    fun onFrame(frame: Frame, view: FloatArray, proj: FloatArray, viewW: Int, viewH: Int, frameIdx: Int) {
        val st = store ?: return
        var msg: String? = null
        // 1) tag sightings
        for (img in frame.getUpdatedTrackables(AugmentedImage::class.java)) {
            if (img.trackingState != TrackingState.TRACKING || img.trackingMethod != AugmentedImage.TrackingMethod.FULL_TRACKING) continue
            val id = img.name.removePrefix("esp_").toIntOrNull() ?: continue
            val p = img.centerPose
            val list = synchronized(sightings) { sightings.getOrPut(id) { ArrayList() } }
            list.add(doubleArrayOf(p.tx().toDouble(), p.ty().toDouble(), p.tz().toDouble()))
            if (list.size > 60) list.removeAt(0)
            val n = p.yAxis   // the image's normal
            val node = EspNode(id, EspMath.median(list), doubleArrayOf(n[0].toDouble(), n[1].toDouble(), n[2].toDouble()),
                "auto_tag_live", frameIdx, frame.timestamp / 1e9, minOf(1.0, list.size / 10.0), list.size)
            if (st.put(node) && list.size == 1) msg = "ESP $id found from its tag"
        }
        // 2) taps
        while (true) {
            val t = taps.poll() ?: break
            val id = t[2].toInt()
            msg = placeTap(frame, view, proj, viewW, viewH, t[0], t[1], id, frameIdx, st)
        }
        val now = System.currentTimeMillis()
        if (st.dirty && now - lastSave > 1000) { st.save(); lastSave = now }
        // 3) where every placed node is on screen
        val vp = FloatArray(16)
        Matrix.multiplyMM(vp, 0, proj, 0, view, 0)
        val marks = st.all().mapNotNull { n ->
            val c = FloatArray(4)
            Matrix.multiplyMV(c, 0, vp, 0, floatArrayOf(n.pos[0].toFloat(), n.pos[1].toFloat(), n.pos[2].toFloat(), 1f), 0)
            if (c[3] <= 0f) null else EspLiveOverlay.Mark(n.id, (c[0] / c[3] + 1f) / 2f * viewW, (1f - c[1] / c[3]) / 2f * viewH,
                n.method, c[3])
        }
        pushOverlay(marks, msg)
    }

    private fun placeTap(frame: Frame, view: FloatArray, proj: FloatArray, w: Int, h: Int, x: Float, y: Float, id: Int,
                         frameIdx: Int, st: EspNodeStore): String {
        if (frame.camera.trackingState != TrackingState.TRACKING) return "hold still: tracking is not ready"
        val hit = frame.hitTest(x, y).firstOrNull { hr ->
            when (val tr = hr.trackable) {
                is DepthPoint -> true
                is Plane -> tr.isPoseInPolygon(hr.hitPose)
                is Point -> true
                else -> false
            }
        }
        if (hit != null) {
            val p = hit.hitPose
            val n = if (hit.trackable is Point && (hit.trackable as Point).orientationMode != Point.OrientationMode.ESTIMATED_SURFACE_NORMAL) null
                else p.yAxis.let { doubleArrayOf(it[0].toDouble(), it[1].toDouble(), it[2].toDouble()) }
            st.put(EspNode(id, doubleArrayOf(p.tx().toDouble(), p.ty().toDouble(), p.tz().toDouble()), n, "tap_live",
                frameIdx, frame.timestamp / 1e9, 0.8, 1), force = true)
            rays.remove(id)
            if (id == selected && id < EspMath.NODE_COUNT) selected = id + 1
            return String.format(java.util.Locale.US, "ESP %d placed (%.2f m away)", id, hit.distance)
        }
        // No surface under the tap: keep its ray; two taps from different angles are intersected.
        val r = screenRay(view, proj, w, h, x, y)
        val list = rays.getOrPut(id) { ArrayList() }
        list.add(r)
        val tri = EspMath.triangulate(list)
        if (tri != null && tri.third >= 5 && tri.second < 0.1) {
            st.put(EspNode(id, tri.first, null, "triangulated", frameIdx, frame.timestamp / 1e9,
                kotlin.math.exp(-tri.second / 0.03), list.size), force = true)
            return String.format(java.util.Locale.US, "ESP %d placed from %d taps (%.0f° apart)", id, list.size, tri.third)
        }
        return "ESP $id: no surface there yet. Step to the side and tap it again"
    }

    private fun screenRay(view: FloatArray, proj: FloatArray, w: Int, h: Int, x: Float, y: Float): Pair<DoubleArray, DoubleArray> {
        val vp = FloatArray(16); val inv = FloatArray(16)
        Matrix.multiplyMM(vp, 0, proj, 0, view, 0)
        Matrix.invertM(inv, 0, vp, 0)
        val nx = 2f * x / w - 1f; val ny = 1f - 2f * y / h
        val a = FloatArray(4); val b = FloatArray(4)
        Matrix.multiplyMV(a, 0, inv, 0, floatArrayOf(nx, ny, -1f, 1f), 0)
        Matrix.multiplyMV(b, 0, inv, 0, floatArrayOf(nx, ny, 1f, 1f), 0)
        val o = DoubleArray(3) { (a[it] / a[3]).toDouble() }
        val d = DoubleArray(3) { (b[it] / b[3]).toDouble() - o[it] }
        val n = EspMath.norm(d)
        return Pair(o, DoubleArray(3) { d[it] / n })
    }

    private fun pushOverlay(marks: List<EspLiveOverlay.Mark>, msg: String?) {
        val ov = overlay ?: return
        val placed = store?.all()?.associate { it.id to it.method } ?: emptyMap()
        ov.post { ov.update(marks, placed, selected, msg) }
    }
}

/** Live overlay over the AR preview: node buttons, markers on placed nodes, tap-to-place when "Mark ESP" is on. */
class EspLiveOverlay(context: Context, private val live: EspLive) : FrameLayout(context) {
    class Mark(val id: Int, val x: Float, val y: Float, val method: String, val dist: Float)

    private var marks: List<Mark> = emptyList()
    private var placed: Map<Int, String> = emptyMap()
    private var marking = false
    private val bar = LinearLayout(context).apply { orientation = LinearLayout.HORIZONTAL; setBackgroundColor(0x99000000.toInt()) }
    private val buttons = HashMap<Int, Button>()
    private val toggle: Button
    private val info = TextView(context).apply { setTextColor(Color.WHITE); textSize = 12f; setShadowLayer(3f, 0f, 0f, Color.BLACK) }
    private val dot = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 5f }
    private val txt = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 34f; setShadowLayer(4f, 0f, 0f, Color.BLACK) }

    init {
        setWillNotDraw(false)
        val d = resources.displayMetrics.density
        toggle = Button(context).apply {
            text = "Mark ESP"; isAllCaps = false; textSize = 12f
            setOnClickListener { marking = !marking; refreshButtons() }
        }
        bar.addView(toggle)
        for (i in 1..EspMath.NODE_COUNT) {
            val b = Button(context).apply {
                text = "$i"; textSize = 12f; minWidth = 0; minimumWidth = (40 * d).toInt()
                setOnClickListener { live.selected = i; marking = true; refreshButtons() }
            }
            buttons[i] = b
            bar.addView(b)
        }
        addView(HorizontalScrollView(context).apply { addView(bar) },
            LayoutParams(LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT, Gravity.BOTTOM or Gravity.START).apply { bottomMargin = (120 * d).toInt() })
        addView(info, LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT, Gravity.BOTTOM).apply {
            bottomMargin = (172 * d).toInt(); leftMargin = (8 * d).toInt() })
        refreshButtons()
    }

    fun update(m: List<Mark>, p: Map<Int, String>, sel: Int, msg: String?) {
        marks = m; placed = p
        if (msg != null) info.text = msg
        refreshButtons()
        invalidate()
    }

    private fun refreshButtons() {
        toggle.text = if (marking) "Tap ESP ${live.selected}" else "Mark ESP"
        for ((i, b) in buttons) {
            val m = placed[i]
            b.setBackgroundColor(when {
                i == live.selected && marking -> Color.rgb(230, 120, 0)
                m == null -> Color.DKGRAY
                EspMath.rank(m) >= 4 -> Color.rgb(20, 140, 60)
                else -> Color.rgb(30, 100, 180)
            })
            b.setTextColor(Color.WHITE)
        }
        if (!marking && info.text.isNullOrEmpty()) info.text = "Film each ESP's tag for a second, or tap Mark ESP and tap the board."
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        if (!marking) return false            // let the preview's own gestures through
        if (e.action == MotionEvent.ACTION_UP) live.tap(e.x, e.y)
        return true
    }

    override fun onDraw(c: Canvas) {
        super.onDraw(c)
        for (m in marks) {
            if (m.x < -50 || m.y < -50 || m.x > width + 50 || m.y > height + 50) continue
            dot.color = if (EspMath.rank(m.method) >= 4) Color.rgb(40, 220, 90) else Color.rgb(60, 170, 255)
            val r = (40f / m.dist.coerceAtLeast(0.3f)).coerceIn(14f, 70f)
            c.drawCircle(m.x, m.y, r, dot)
            c.drawText("ESP ${m.id}", m.x + r + 6, m.y + 12, txt)
        }
    }
}
