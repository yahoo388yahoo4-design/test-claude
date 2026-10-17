package com.real2sim.capture

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Matrix
import android.graphics.Paint
import android.graphics.Typeface
import android.media.MediaMetadataRetriever
import android.os.Bundle
import android.view.Gravity
import android.view.MotionEvent
import android.view.View
import android.view.ViewConfiguration
import android.view.ViewGroup
import android.widget.Button
import android.widget.HorizontalScrollView
import android.widget.LinearLayout
import android.widget.SeekBar
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import java.io.File
import java.util.Locale
import java.util.concurrent.Executors
import kotlin.math.abs

/**
 * Tap the ESP32 board in a recorded video frame to put it into the scan (sensing/esp32/README.md).
 * With a depth map near that frame one tap is enough; without depth, tap the board in two frames seen
 * from different angles and the two rays are intersected. Writes <session>/esp_nodes.json.
 * Placed nodes are drawn back into every frame, so you can scrub and check them.
 */
class EspMarkActivity : AppCompatActivity() {
    companion object { const val EXTRA_DIR = "dir" }

    private lateinit var dir: File
    private lateinit var store: EspNodeStore
    private lateinit var data: FrameData
    private lateinit var image: FrameTapView
    private lateinit var status: TextView
    private lateinit var seek: SeekBar
    private val io = Executors.newSingleThreadExecutor()
    private var retriever: MediaMetadataRetriever? = null
    private var selected = 1
    private var current = -1
    private val nodeButtons = HashMap<Int, Button>()
    private val rays = HashMap<Int, ArrayList<Pair<DoubleArray, DoubleArray>>>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        dir = File(intent.getStringExtra(EXTRA_DIR) ?: run { finish(); return })
        store = EspNodeStore.forSession(dir)
        val d = resources.displayMetrics.density
        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setBackgroundColor(Color.BLACK) }
        root.addView(TextView(this).apply {
            text = "ESP nodes · ${dir.name}"; setTextColor(Color.WHITE); textSize = 15f; typeface = Typeface.MONOSPACE
            setPadding((12 * d).toInt(), (10 * d).toInt(), (12 * d).toInt(), (4 * d).toInt())
        })
        val bar = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        for (i in 1..EspMath.NODE_COUNT) {
            val b = Button(this).apply {
                text = "ESP $i"; isAllCaps = false; textSize = 12f
                setOnClickListener { selected = i; refresh() }
            }
            nodeButtons[i] = b; bar.addView(b)
        }
        bar.addView(Button(this).apply {
            text = "Clear"; isAllCaps = false; textSize = 12f
            setOnClickListener { store.remove(selected); rays.remove(selected); store.save(); refresh(); showFrame(current, force = true) }
        })
        root.addView(HorizontalScrollView(this).apply { addView(bar) })
        image = FrameTapView(this) { u, v -> onTap(u, v) }
        root.addView(image, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))
        seek = SeekBar(this)
        root.addView(seek)
        val nav = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER }
        for ((label, step) in listOf("−30" to -30, "−1" to -1, "+1" to 1, "+30" to 30)) nav.addView(Button(this).apply {
            text = label; isAllCaps = false; setOnClickListener { seek.progress = (seek.progress + step).coerceIn(0, seek.max); showFrame(seek.progress) }
        })
        nav.addView(Button(this).apply { text = "Zoom"; isAllCaps = false; setOnClickListener { image.cycleZoom() } })
        root.addView(nav)
        status = TextView(this).apply { setTextColor(Color.LTGRAY); textSize = 12f; setPadding((12 * d).toInt(), (4 * d).toInt(), (12 * d).toInt(), (12 * d).toInt()) }
        root.addView(status)
        setContentView(root)
        status.text = "Loading…"
        io.execute {
            val fd = Frames.read(File(dir, "frames.jsonl"))
            val video = listOf("video.mp4", "video.mov").map { File(dir, it) }.firstOrNull { it.isFile }
            val mr = video?.let { v -> try { MediaMetadataRetriever().apply { setDataSource(v.absolutePath) } } catch (_: Exception) { null } }
            runOnUiThread {
                if (isDestroyed) return@runOnUiThread
                data = fd; retriever = mr
                if (fd.frames.none { it.hasPose } || mr == null) {
                    status.text = "This session has no video with camera poses (record in mode A to mark ESPs)."
                    return@runOnUiThread
                }
                seek.max = fd.frames.size - 1
                seek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                    override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) { if (fromUser) showFrame(p) }
                    override fun onStartTrackingTouch(s: SeekBar) {}
                    override fun onStopTrackingTouch(s: SeekBar) { showFrame(s.progress, force = true) }
                })
                // start where the selected node was last placed, else in the middle
                val start = store.get(selected)?.frameIndex?.coerceIn(0, seek.max) ?: (fd.frames.size / 2)
                seek.progress = start
                refresh()
                showFrame(start, force = true)
            }
        }
    }

    override fun onDestroy() {
        io.shutdownNow()
        retriever?.release()
        super.onDestroy()
    }

    private var loading = false
    private var queued = -1

    private fun showFrame(k: Int, force: Boolean = false) {
        if (!::data.isInitialized || k !in data.frames.indices) return
        if (k == current && !force) return
        if (loading) { queued = k; return }
        loading = true
        val f = data.frames[k]
        io.execute {
            val bmp = try { retriever?.getFrameAtIndex(f.i) } catch (_: Exception) { null }
            // The video is stored in the sensor's landscape orientation; undo any rotation the decoder applied.
            val img = bmp?.let { b -> if (b.width == f.h && b.height == f.w && f.w != f.h)
                Bitmap.createBitmap(b, 0, 0, b.width, b.height, Matrix().apply { postRotate(-90f) }, true) else b }
            runOnUiThread {
                loading = false
                if (isDestroyed) return@runOnUiThread
                current = k
                image.setFrame(img, f.w, f.h)
                drawMarks()
                status.text = statusLine()
                if (queued >= 0 && queued != k) { val q = queued; queued = -1; showFrame(q) } else queued = -1
            }
        }
    }

    private fun drawMarks() {
        val f = data.frames.getOrNull(current) ?: return
        val K = f.K; val T = f.T
        image.marks = if (K == null || T == null) emptyList() else store.all().mapNotNull { n ->
            EspMath.project(K, T, n.pos)?.let { p -> FrameTapView.Mark(n.id, p[0].toFloat(), p[1].toFloat(), EspMath.rank(n.method) >= 4) }
        }
        image.rayMarks = rays[selected]?.size ?: 0
        image.invalidate()
    }

    private fun statusLine(): String {
        val f = data.frames.getOrNull(current) ?: return ""
        val sb = StringBuilder(String.format(Locale.US, "frame %d / %d  t=%.2f s%s\n", current, data.frames.size - 1, f.t - data.frames[0].t,
            if (f.hasPose) "" else "  (no pose in this frame)"))
        sb.append("Tap ESP $selected in the picture. ")
        store.get(selected)?.let { n -> sb.append(String.format(Locale.US, "Placed (%s): %.2f, %.2f, %.2f m. ", n.method, n.pos[0], n.pos[1], n.pos[2])) }
        val placed = store.all().map { it.id }
        sb.append("Placed: ${if (placed.isEmpty()) "none" else placed.joinToString(", ")}")
        return sb.toString()
    }

    private fun refresh() {
        val placed = store.all().associateBy { it.id }
        for ((i, b) in nodeButtons) {
            b.setBackgroundColor(when {
                i == selected -> Color.rgb(230, 120, 0)
                placed[i] == null -> Color.DKGRAY
                EspMath.rank(placed[i]!!.method) >= 4 -> Color.rgb(20, 140, 60)
                else -> Color.rgb(30, 100, 180)
            })
            b.setTextColor(Color.WHITE)
        }
        if (::data.isInitialized) { status.text = statusLine(); drawMarks() }
    }

    /** (u, v) in native image pixels. */
    private fun onTap(u: Double, v: Double) {
        val f = data.frames.getOrNull(current) ?: return
        val K = f.K ?: return
        val T = f.T ?: run { status.text = "No camera pose in this frame: pick another one."; return }
        val id = selected
        io.execute {
            val dz = depthNear(f)?.let { (mm, ref) -> EspMath.depthAt(mm, ref.w, ref.h, f.w, f.h, u, v, 1) }
            val msg: String
            if (dz != null) {
                store.put(EspNode(id, EspMath.unproject(K, T, u, v, dz), null, "tap_frame", current, f.t, 0.8, 1), force = true)
                rays.remove(id)
                msg = String.format(Locale.US, "ESP %d placed %.2f m from the camera.", id, dz)
            } else {
                val list = rays.getOrPut(id) { ArrayList() }
                list.add(EspMath.ray(K, T, u, v))
                val tri = EspMath.triangulate(list)
                msg = if (tri != null && tri.third >= 5 && tri.second < 0.1) {
                    store.put(EspNode(id, tri.first, null, "triangulated", current, f.t, kotlin.math.exp(-tri.second / 0.03), list.size), force = true)
                    String.format(Locale.US, "ESP %d placed from %d views (%.0f° apart, %.1f cm residual).", id, list.size, tri.third, tri.second * 100)
                } else "No depth here: scrub to a frame that sees ESP $id from another angle and tap it again."
            }
            store.save()
            runOnUiThread { refresh(); status.text = msg + "\n" + statusLine() }
        }
    }

    /** Depth map within 50 ms of the frame (raw, else smoothed). */
    private fun depthNear(f: FrameRec): Pair<ShortArray, DepthRef>? {
        for ((refs, times, blob) in listOf(Triple(data.depth, data.depthTimes, "depth.zlib.bin"), Triple(data.smooth, data.smoothTimes, "depth_smooth.zlib.bin"))) {
            val j = nearestIndex(times, f.t)
            if (j < 0 || abs(times[j] - f.t) > 0.05) continue
            val ref = refs[j]
            val mm = Depth.load(File(dir, blob), ref) ?: continue
            return Pair(mm, ref)
        }
        return null
    }
}

/** Shows one frame (fit, optional zoom + pan) and reports taps in native image pixels. */
class FrameTapView(context: Context, private val onTap: (Double, Double) -> Unit) : View(context) {
    class Mark(val id: Int, val u: Float, val v: Float, val manual: Boolean)

    var marks: List<Mark> = emptyList()
    var rayMarks = 0
    private var bmp: Bitmap? = null
    private var iw = 1; private var ih = 1
    private var zoom = 1f
    private var cx = 0.5f; private var cy = 0.5f        // view centre in normalised image coords
    private val m = Matrix(); private val inv = Matrix()
    private val ring = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 4f }
    private val txt = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 32f; setShadowLayer(4f, 0f, 0f, Color.BLACK) }
    private val slop = ViewConfiguration.get(context).scaledTouchSlop
    private var downX = 0f; private var downY = 0f; private var lastX = 0f; private var lastY = 0f; private var moved = false

    fun setFrame(b: Bitmap?, w: Int, h: Int) { bmp = b; iw = w; ih = h; invalidate() }

    fun cycleZoom() { zoom = when (zoom) { 1f -> 2f; 2f -> 4f; else -> 1f }; if (zoom == 1f) { cx = 0.5f; cy = 0.5f }; invalidate() }

    private fun layoutMatrix() {
        val base = minOf(width.toFloat() / iw, height.toFloat() / ih)
        val s = base * zoom
        m.reset()
        m.postTranslate(-cx * iw, -cy * ih)
        m.postScale(s, s)
        m.postTranslate(width / 2f, height / 2f)
        m.invert(inv)
    }

    override fun onDraw(c: Canvas) {
        super.onDraw(c)
        val b = bmp ?: return
        layoutMatrix()
        val bm = Matrix(m).apply { preScale(iw.toFloat() / b.width, ih.toFloat() / b.height) }
        c.drawBitmap(b, bm, null)
        val p = FloatArray(2)
        for (k in marks) {
            p[0] = k.u; p[1] = k.v; m.mapPoints(p)
            ring.color = if (k.manual) Color.rgb(40, 220, 90) else Color.rgb(60, 170, 255)
            c.drawCircle(p[0], p[1], 26f, ring)
            c.drawText("ESP ${k.id}", p[0] + 32, p[1] + 10, txt)
        }
        if (rayMarks > 0) c.drawText("views so far: $rayMarks", 16f, height - 20f, txt)
    }

    override fun onTouchEvent(e: MotionEvent): Boolean {
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> { downX = e.x; downY = e.y; lastX = e.x; lastY = e.y; moved = false }
            MotionEvent.ACTION_MOVE -> {
                if (abs(e.x - downX) > slop || abs(e.y - downY) > slop) moved = true
                if (moved && zoom > 1f) {
                    val s = minOf(width.toFloat() / iw, height.toFloat() / ih) * zoom
                    cx = (cx - (e.x - lastX) / s / iw).coerceIn(0f, 1f); cy = (cy - (e.y - lastY) / s / ih).coerceIn(0f, 1f)
                    invalidate()
                }
                lastX = e.x; lastY = e.y
            }
            MotionEvent.ACTION_UP -> if (!moved && bmp != null) {
                layoutMatrix()
                val p = floatArrayOf(e.x, e.y); inv.mapPoints(p)
                if (p[0] in 0f..iw.toFloat() && p[1] in 0f..ih.toFloat()) onTap(p[0].toDouble(), p[1].toDouble())
            }
        }
        return true
    }
}
