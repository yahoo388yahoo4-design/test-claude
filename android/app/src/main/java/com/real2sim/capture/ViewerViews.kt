package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.LinearGradient
import android.graphics.Matrix
import android.graphics.Paint
import android.graphics.Path
import android.graphics.Shader
import android.graphics.SurfaceTexture
import android.media.MediaPlayer
import android.media.PlaybackParams
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Handler
import android.os.SystemClock
import android.util.Log
import android.view.MotionEvent
import android.view.ScaleGestureDetector
import android.view.Surface
import android.view.TextureView
import android.view.ViewConfiguration
import android.view.View
import android.widget.FrameLayout
import androidx.core.graphics.withClip
import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Locale
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.tan

/** FrameLayout that keeps a width/height [aspect] (height follows width). */
@SuppressLint("ViewConstructor")
class AspectFrame(ctx: Context, aspect: Float) : FrameLayout(ctx) {
    var aspect: Float = aspect
        set(v) { if (v != field) { field = v; requestLayout() } }

    /** Optional height cap (px) used when the parent does not limit the height (e.g. inside a ScrollView). */
    var maxHeightPx = 0

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        var h = (w / aspect).toInt()
        var maxH = if (MeasureSpec.getMode(heightMeasureSpec) != MeasureSpec.UNSPECIFIED) MeasureSpec.getSize(heightMeasureSpec) else 0
        if (maxHeightPx > 0 && (maxH == 0 || maxHeightPx < maxH)) maxH = maxHeightPx
        var ww = w
        if (maxH in 1 until h) { h = maxH; ww = (h * aspect).toInt() }
        super.onMeasure(MeasureSpec.makeMeasureSpec(ww, MeasureSpec.EXACTLY), MeasureSpec.makeMeasureSpec(h, MeasureSpec.EXACTLY))
    }
}

/**
 * Matrix mapping content of [cw]x[ch] (stretched to the view by default for TextureView, pass cw = vw,
 * ch = vh) onto a [vw]x[vh] view rotated by [deg] (multiple of 90). The view's aspect should already be
 * the rotated content's aspect.
 */
fun rotateFit(cw: Float, ch: Float, vw: Float, vh: Float, deg: Int): Matrix {
    val m = Matrix()
    val cx = vw / 2; val cy = vh / 2
    m.setScale(vw / cw, vh / ch)
    if (deg % 180 != 0) m.postScale(vh / vw, vw / vh, cx, cy)
    if (deg % 360 != 0) m.postRotate(deg.toFloat(), cx, cy)
    return m
}

/**
 * Plays several videos in sync on a session clock. Each track starts [Track.offset] seconds after time 0
 * (time 0 = the earliest first video frame). With no video it is a virtual clock over [duration], so
 * sensor charts can still be scrubbed and played. The first playing track drives the clock; the others
 * are re-seeked when they drift more than 0.25 s.
 */
class Playback(private val ui: Handler) {
    class Track(val name: String, val file: File, val offset: Double) {
        var player: MediaPlayer? = null
        var prepared = false
        var durationS = 0.0
        var videoW = 0; var videoH = 0
        var surface: Surface? = null
    }

    val tracks = mutableListOf<Track>()
    var duration = 0.0
    var time = 0.0; private set
    var playing = false; private set
    var speed = 1f; private set
    private var baseWall = 0L
    private var baseTime = 0.0
    private val listeners = mutableListOf<(Double) -> Unit>()
    var onState: (() -> Unit)? = null

    fun listen(l: (Double) -> Unit) { listeners.add(l) }

    /** Re-sends the current time to every listener (after a tab becomes visible). */
    fun poke() = notifyTime()

    private fun notifyTime() { for (l in listeners) l(time) }

    /** Binds [track] to a TextureView: the player is created when the surface exists and released with it. */
    fun attach(track: Track, tv: TextureView, onSize: (Int, Int) -> Unit = { _, _ -> }) {
        tv.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) = open(track, Surface(st), onSize)
            override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, w: Int, h: Int) {}
            override fun onSurfaceTextureDestroyed(st: SurfaceTexture): Boolean { close(track); return true }
            override fun onSurfaceTextureUpdated(st: SurfaceTexture) {}
        }
        tv.surfaceTexture?.let { open(track, Surface(it), onSize) }
    }

    private fun open(track: Track, s: Surface, onSize: (Int, Int) -> Unit) {
        close(track)
        track.surface = s
        val mp = MediaPlayer()
        track.player = mp
        try {
            mp.setDataSource(track.file.absolutePath)
            mp.setSurface(s)
            mp.setVolume(0f, 0f)
            mp.setOnVideoSizeChangedListener { _, w, h -> if (w > 0 && h > 0) { track.videoW = w; track.videoH = h; onSize(w, h) } }
            mp.setOnPreparedListener { p ->
                track.prepared = true
                track.durationS = p.duration / 1000.0
                val end = track.offset + track.durationS
                if (end > duration) { duration = end; onState?.invoke() }
                seekTrack(track, time)
                if (playing) sync(SystemClock.elapsedRealtime())
            }
            mp.setOnErrorListener { _, what, extra -> Log.w(TAG, "${track.name}: error $what/$extra"); true }
            mp.prepareAsync()
        } catch (e: Exception) {
            Log.w(TAG, "open ${track.file}", e)
        }
    }

    private fun close(track: Track) {
        track.player?.let { try { it.release() } catch (_: Exception) {} }
        track.player = null; track.prepared = false
        track.surface?.release(); track.surface = null
    }

    fun release() {
        pause()
        tracks.forEach { close(it) }
        listeners.clear()
    }

    private fun seekTrack(t: Track, sessionT: Double) {
        val p = t.player ?: return
        if (!t.prepared) return
        val local = (sessionT - t.offset).coerceIn(0.0, max(0.0, t.durationS - 0.001))
        try { p.seekTo((local * 1000).toLong(), MediaPlayer.SEEK_CLOSEST) } catch (_: Exception) {}
    }

    fun seek(t: Double) {
        time = t.coerceIn(0.0, max(duration, 0.0))
        baseTime = time; baseWall = SystemClock.elapsedRealtime()
        for (tr in tracks) {
            val p = tr.player ?: continue
            if (!tr.prepared) continue
            val local = time - tr.offset
            if (playing && (local < 0 || local >= tr.durationS) && p.isPlaying) p.pause()
            seekTrack(tr, time)
        }
        notifyTime()
    }

    fun toggle() = if (playing) pause() else play()

    fun play() {
        if (duration <= 0) return
        if (time >= duration - 0.05) seek(0.0)
        playing = true
        baseTime = time; baseWall = SystemClock.elapsedRealtime()
        sync(baseWall)
        ui.removeCallbacks(tick); ui.post(tick)
        onState?.invoke()
    }

    fun pause() {
        if (!playing) return
        playing = false
        ui.removeCallbacks(tick)
        for (tr in tracks) tr.player?.let { p -> try { if (tr.prepared && p.isPlaying) p.pause() } catch (_: Exception) {} }
        onState?.invoke()
        notifyTime()
    }

    fun setSpeed(s: Float) {
        val now = SystemClock.elapsedRealtime()
        baseTime = clock(now); baseWall = now
        speed = s
        if (playing) for (tr in tracks) tr.player?.let { p -> try { if (tr.prepared && p.isPlaying) p.playbackParams = params() } catch (_: Exception) {} }
        onState?.invoke()
    }

    private fun params() = PlaybackParams().setSpeed(speed).setPitch(1f)

    private fun clock(now: Long) = baseTime + (now - baseWall) / 1000.0 * speed

    /** Advance the clock (following the first running video) and keep every track at it. */
    private fun sync(now: Long) {
        var t = clock(now)
        val master = tracks.firstOrNull { it.prepared && it.player?.isPlaying == true }
        if (master != null) {
            val mt = master.player!!.currentPosition / 1000.0 + master.offset
            if (abs(mt - t) < 0.5) { t = mt; baseTime = mt; baseWall = now }
        }
        time = t.coerceIn(0.0, duration)
        for (tr in tracks) {
            val p = tr.player ?: continue
            if (!tr.prepared) continue
            val local = time - tr.offset
            try {
                if (local < 0 || local >= tr.durationS - 0.02) { if (p.isPlaying) p.pause(); continue }
                if (!p.isPlaying) {
                    p.seekTo((local * 1000).toLong(), MediaPlayer.SEEK_CLOSEST)
                    p.start(); p.playbackParams = params()
                } else if (tr !== master && abs(p.currentPosition / 1000.0 - local) > 0.25) {
                    p.seekTo((local * 1000).toLong(), MediaPlayer.SEEK_CLOSEST)
                }
            } catch (e: Exception) { Log.w(TAG, "sync ${tr.name}", e) }
        }
    }

    private val tick = object : Runnable {
        override fun run() {
            if (!playing) return
            sync(SystemClock.elapsedRealtime())
            notifyTime()
            if (time >= duration) { pause(); return }
            ui.postDelayed(this, 33)
        }
    }

    companion object { const val TAG = "Playback" }
}

/** Horizontal colour bar of the depth palette with a label. */
class DepthLegend(ctx: Context) : View(ctx) {
    var label = "depth 0 → 5 m"
    private val bar = Paint()
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.LTGRAY; textSize = 28f }
    private val colors = IntArray(9) { k -> Depth.colorize(shortArrayOf((k * 5000 / 8).coerceAtLeast(1).toShort()))[0] }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        val bw = w - text.measureText(label) - 16
        if (bw > 10) bar.shader = LinearGradient(0f, 0f, bw, 0f, colors, null, Shader.TileMode.CLAMP)
    }

    override fun onDraw(c: Canvas) {
        val tw = text.measureText(label) + 16
        val w = width - tw
        if (w > 10) c.drawRoundRect(0f, height / 2f - 8, w, height / 2f + 8, 8f, 8f, bar)
        c.drawText(label, width - tw + 12, height / 2f + 10, text)
    }
}

/**
 * Line chart of one or more series against session time (s), with a cursor at the playback time.
 * Tapping or dragging on the chart calls [onSeek].
 */
class ChartView(ctx: Context) : View(ctx) {
    class Series(val name: String, val xy: FloatArray)

    var title = ""
    var series: List<Series> = emptyList()
        set(v) { field = v; computeRange(); invalidate() }
    var x0 = 0f; var x1 = 1f
    var cursor = Float.NaN
        set(v) { if (v != field) { field = v; invalidate() } }
    var onSeek: ((Double) -> Unit)? = null

    private var y0 = 0f; private var y1 = 1f
    private val palette = intArrayOf(Color.rgb(255, 99, 99), Color.rgb(99, 220, 120), Color.rgb(90, 160, 255), Color.rgb(255, 200, 60), Color.rgb(200, 120, 255), Color.rgb(80, 220, 230))
    private val pLine = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 2.5f }
    private val pAxis = Paint().apply { color = Color.argb(90, 255, 255, 255); strokeWidth = 1f }
    private val pCursor = Paint().apply { color = Color.argb(200, 255, 255, 255); strokeWidth = 3f }
    private val pText = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE; textSize = 28f }
    private val pSmall = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.LTGRAY; textSize = 24f }
    private val path = Path()
    private val cursorText = StringBuilder()

    private fun computeRange() {
        var lo = Float.POSITIVE_INFINITY; var hi = Float.NEGATIVE_INFINITY
        for (s in series) { var k = 1; while (k < s.xy.size) { val v = s.xy[k]; if (v < lo) lo = v; if (v > hi) hi = v; k += 2 } }
        if (!lo.isFinite()) { lo = 0f; hi = 1f }
        if (hi - lo < 1e-6f) { lo -= 0.5f; hi += 0.5f }
        val m = (hi - lo) * 0.06f
        y0 = lo - m; y1 = hi + m
    }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        setMeasuredDimension(w, (resources.displayMetrics.density * 170).toInt())
    }

    private val left get() = 8f
    private val top get() = 64f
    private val bottom get() = height - 30f

    override fun onDraw(c: Canvas) {
        c.drawColor(Color.rgb(24, 26, 30))
        c.drawText(title, 8f, 28f, pText)
        // legend
        var lx = 8f
        series.forEachIndexed { k, s ->
            pSmall.color = palette[k % palette.size]
            c.drawText(s.name, lx, 56f, pSmall); lx += pSmall.measureText(s.name) + 24
        }
        pSmall.color = Color.LTGRAY
        val w = width - 16f
        val h = bottom - top
        if (w <= 0 || h <= 0) return
        fun sx(x: Float) = left + (x - x0) / (x1 - x0) * w
        fun sy(y: Float) = bottom - (y - y0) / (y1 - y0) * h
        c.drawLine(left, bottom, left + w, bottom, pAxis); c.drawLine(left, top, left + w, top, pAxis)
        if (y0 < 0 && y1 > 0) c.drawLine(left, sy(0f), left + w, sy(0f), pAxis)
        c.drawText(fmt(y1), left + 4, top + 24, pSmall)
        c.drawText(fmt(y0), left + 4, bottom - 6, pSmall)
        c.drawText(String.format(Locale.US, "%.0f s", x0), left, height - 4f, pSmall)
        val endLabel = String.format(Locale.US, "%.0f s", x1)
        c.drawText(endLabel, left + w - pSmall.measureText(endLabel), height - 4f, pSmall)
        c.withClip(left, top - 2, left + w, bottom + 2) {
            series.forEachIndexed { k, s ->
                if (s.xy.size < 4) return@forEachIndexed
                pLine.color = palette[k % palette.size]
                path.reset(); path.moveTo(sx(s.xy[0]), sy(s.xy[1]))
                var i = 2; while (i < s.xy.size) { path.lineTo(sx(s.xy[i]), sy(s.xy[i + 1])); i += 2 }
                drawPath(path, pLine)
            }
        }
        if (!cursor.isNaN() && cursor in x0..x1) {
            val x = sx(cursor)
            c.drawLine(x, top, x, bottom, pCursor)
            // values at the cursor
            val sb = cursorText; sb.setLength(0)
            series.forEach { s -> valueAt(s.xy, cursor)?.let { v -> if (sb.isNotEmpty()) sb.append("  "); sb.append(s.name).append(' ').append(fmt(v)) } }
            val txt = sb.toString()
            val tw = pSmall.measureText(txt)
            c.drawText(txt, max(left, min(x + 8, left + w - tw)), bottom - 6, pSmall)
        }
    }

    private fun valueAt(xy: FloatArray, x: Float): Float? {
        if (xy.size < 2) return null
        var lo = 0; var hi = xy.size / 2 - 1
        while (lo < hi) { val mid = (lo + hi) / 2; if (xy[mid * 2] < x) lo = mid + 1 else hi = mid }
        return xy[lo * 2 + 1]
    }

    private fun fmt(v: Float) = when {
        abs(v) >= 1000 -> String.format(Locale.US, "%.0f", v)
        abs(v) >= 10 -> String.format(Locale.US, "%.1f", v)
        else -> String.format(Locale.US, "%.3f", v)
    }

    private val gesture = ChartGesture(ViewConfiguration.get(ctx).scaledTouchSlop.toFloat())

    /**
     * Intercept is not disallowed on DOWN, so the enclosing ScrollView claims a mostly vertical drag itself (we then
     * get ACTION_CANCEL); the chart only takes over for a mostly horizontal drag, and a plain tap seeks on UP.
     */
    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(e: MotionEvent): Boolean {
        val cb = onSeek ?: return false
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> gesture.down(e.x, e.y)
            MotionEvent.ACTION_MOVE -> {
                if (gesture.move(e.x, e.y)) parent?.requestDisallowInterceptTouchEvent(true)
                if (gesture.claimed) seekTo(e.x, cb)
            }
            MotionEvent.ACTION_UP -> {
                if (gesture.up()) seekTo(e.x, cb)
                performClick(); parent?.requestDisallowInterceptTouchEvent(false)
            }
            MotionEvent.ACTION_CANCEL -> { gesture.cancel(); parent?.requestDisallowInterceptTouchEvent(false) }
        }
        return true
    }

    private fun seekTo(px: Float, cb: (Double) -> Unit) {
        val w = width - 16f
        if (w <= 0) return
        cb((x0 + (px - left) / w * (x1 - x0)).toDouble().coerceIn(x0.toDouble(), x1.toDouble()))
    }

    override fun performClick(): Boolean { super.performClick(); return true }
}

/**
 * Touch decision for [ChartView], kept free of View so it is unit-testable: a drag is claimed (seeks) once it
 * moves more than [slop] and mostly horizontally; otherwise the parent may scroll, and a release without a
 * claimed drag is a tap.
 */
class ChartGesture(private val slop: Float) {
    private var downX = 0f; private var downY = 0f
    var claimed = false
        private set

    fun down(x: Float, y: Float) { downX = x; downY = y; claimed = false }

    /** True on the move that newly claims the gesture (the caller then disallows parent intercept). */
    fun move(x: Float, y: Float): Boolean {
        if (claimed) return false
        val dx = abs(x - downX); val dy = abs(y - downY)
        if (dx > slop && dx > dy) { claimed = true; return true }
        return false
    }

    /** True when the gesture ended as a tap (no claimed drag), i.e. seek once at the release point. */
    fun up(): Boolean { val tap = !claimed; claimed = false; return tap }

    fun cancel() { claimed = false }
}

/**
 * 3D view of a session: point cloud / mesh (colour by height, or the PLY's colours), the camera trajectory
 * (yellow), the camera at the playback time (red) and a 1 m floor grid. Drag to orbit, pinch to zoom,
 * two fingers to pan, double-tap to reset.
 */
@SuppressLint("ViewConstructor")
class SceneView(ctx: Context) : GLSurfaceView(ctx), GLSurfaceView.Renderer {
    private var program = 0
    private var aPos = 0; private var aCol = 0; private var aNrm = 0
    private var uMvp = 0; private var uFlat = 0; private var uUseFlat = 0; private var uLit = 0; private var uSize = 0

    @Volatile private var pending: Scene? = null
    private var vboPts = 0; private var nPts = 0
    private var vboTri = 0; private var nTri = 0
    private var traj: java.nio.FloatBuffer? = null; private var nTraj = 0
    private var grid: java.nio.FloatBuffer? = null; private var nGrid = 0
    @Volatile private var marker: FloatArray? = null

    // orbit camera
    @Volatile private var cx = 0f; @Volatile private var cy = 0f; @Volatile private var cz = 0f
    @Volatile private var dist = 5f; @Volatile private var yaw = 0.6f; @Volatile private var pitch = 0.7f
    private var home = floatArrayOf(0f, 0f, 0f, 5f)
    private val proj = FloatArray(16); private val view = FloatArray(16); private val mvp = FloatArray(16)

    /** Geometry: [points] xyz, [pointRgb] per-point 0xRRGGBB (or null = height colour), [tris] mesh, [traj] xyz path. */
    class Scene(val points: FloatArray, val pointRgb: IntArray?, val tris: IntArray, val traj: FloatArray)

    init {
        setEGLContextClientVersion(2)
        setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        setRenderer(this)
        renderMode = RENDERMODE_WHEN_DIRTY
    }

    fun setScene(s: Scene) {
        // bounds -> orbit centre and distance
        var x0 = Float.POSITIVE_INFINITY; var y0 = Float.POSITIVE_INFINITY; var z0 = Float.POSITIVE_INFINITY
        var x1 = Float.NEGATIVE_INFINITY; var y1 = Float.NEGATIVE_INFINITY; var z1 = Float.NEGATIVE_INFINITY
        fun add(a: FloatArray) { var k = 0; while (k + 2 < a.size) { val x = a[k]; val y = a[k + 1]; val z = a[k + 2]
            if (x.isFinite() && y.isFinite() && z.isFinite()) { x0 = min(x0, x); x1 = max(x1, x); y0 = min(y0, y); y1 = max(y1, y); z0 = min(z0, z); z1 = max(z1, z) }; k += 3 } }
        add(s.points); add(s.traj)
        if (!x0.isFinite()) { x0 = -1f; x1 = 1f; y0 = -1f; y1 = 1f; z0 = -1f; z1 = 1f }
        val size = max(max(x1 - x0, z1 - z0), 1f)
        home = floatArrayOf((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, size * 1.2f)
        resetView()
        pending = s
        requestRender()
    }

    fun resetView() { cx = home[0]; cy = home[1]; cz = home[2]; dist = home[3]; yaw = 0.6f; pitch = 0.7f; requestRender() }

    fun setMarker(p: FloatArray?) { marker = p; requestRender() }

    /** Height colours (default) <-> the PLY's own colours (camera RGB of the TSDF mesh), when it has any. */
    @Volatile var fileColors = false
        private set

    fun toggleColors() { fileColors = !fileColors; lastScene?.let { pending = it }; requestRender() }

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        program = link(
            """
            uniform mat4 u_Mvp; uniform float u_Size; attribute vec3 a_Pos; attribute vec3 a_Col; attribute vec3 a_Nrm;
            uniform float u_Lit; varying vec3 v_Col;
            void main() {
              gl_Position = u_Mvp * vec4(a_Pos, 1.0); gl_PointSize = u_Size;
              vec3 n = normalize(a_Nrm + vec3(1e-6));
              float key = abs(dot(n, normalize(vec3(0.3, 1.0, 0.5)))); float fill = abs(dot(n, normalize(vec3(-0.6, 0.2, -0.7))));
              float l = mix(1.0, 0.28 + 0.58 * key + 0.22 * fill, u_Lit);
              v_Col = a_Col * l;
            }
            """.trimIndent(),
            """
            precision mediump float; varying vec3 v_Col; uniform vec4 u_Flat; uniform float u_UseFlat;
            void main() { gl_FragColor = mix(vec4(v_Col, 1.0), u_Flat, u_UseFlat); }
            """.trimIndent())
        aPos = GLES20.glGetAttribLocation(program, "a_Pos")
        aCol = GLES20.glGetAttribLocation(program, "a_Col")
        aNrm = GLES20.glGetAttribLocation(program, "a_Nrm")
        uMvp = GLES20.glGetUniformLocation(program, "u_Mvp")
        uFlat = GLES20.glGetUniformLocation(program, "u_Flat")
        uUseFlat = GLES20.glGetUniformLocation(program, "u_UseFlat")
        uLit = GLES20.glGetUniformLocation(program, "u_Lit")
        uSize = GLES20.glGetUniformLocation(program, "u_Size")
        val b = IntArray(2); GLES20.glGenBuffers(2, b, 0); vboPts = b[0]; vboTri = b[1]
        nPts = 0; nTri = 0
        if (pending == null) lastScene?.let { pending = it }
        GLES20.glClearColor(0.05f, 0.05f, 0.06f, 1f)
    }

    private var lastScene: Scene? = null

    override fun onSurfaceChanged(gl: GL10?, w: Int, h: Int) {
        GLES20.glViewport(0, 0, w, h)
        android.opengl.Matrix.perspectiveM(proj, 0, 60f, w.toFloat() / max(h, 1), 0.02f, 1000f)
    }

    private fun upload(s: Scene) {
        lastScene = s
        // floor = 2nd percentile of point heights (or lowest trajectory point - 1.4 m)
        val ys = ArrayList<Float>()
        var k = 1; while (k < s.points.size) { ys.add(s.points[k]); k += 3 * max(1, s.points.size / 3 / 20000) }
        ys.sort()
        val floor = if (ys.isNotEmpty()) ys[(ys.size * 0.02).toInt()] else {
            var m = Float.POSITIVE_INFINITY; var j = 1; while (j < s.traj.size) { m = min(m, s.traj[j]); j += 3 }
            if (m.isFinite()) m - 1.4f else 0f
        }
        fun heightColor(y: Float, out: FloatArray, o: Int) {
            val h = ((y - floor) / 2f).coerceIn(0f, 1f)
            if (h < 0.5f) { val t = h * 2; out[o] = 0.1f; out[o + 1] = 0.4f + 0.6f * t; out[o + 2] = 1f - 0.6f * t }
            else { val t = (h - 0.5f) * 2; out[o] = 0.1f + 0.9f * t; out[o + 1] = 1f - 0.75f * t; out[o + 2] = 0.4f - 0.2f * t }
        }
        val nv = s.points.size / 3
        val hasMesh = s.tris.isNotEmpty()
        // points (only when there is no mesh): pos, col, nrm(0)
        if (!hasMesh && nv > 0) {
            val a = FloatArray(nv * 9)
            for (i in 0 until nv) {
                a[i * 9] = s.points[i * 3]; a[i * 9 + 1] = s.points[i * 3 + 1]; a[i * 9 + 2] = s.points[i * 3 + 2]
                val rgb = s.pointRgb?.takeIf { fileColors }
                if (rgb != null) { a[i * 9 + 3] = (rgb[i] shr 16 and 255) / 255f; a[i * 9 + 4] = (rgb[i] shr 8 and 255) / 255f; a[i * 9 + 5] = (rgb[i] and 255) / 255f }
                else heightColor(s.points[i * 3 + 1], a, i * 9 + 3)
            }
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboPts)
            GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, a.size * 4, fb(a), GLES20.GL_STATIC_DRAW)
            nPts = nv
        } else nPts = 0
        if (hasMesh) {
            // per-vertex normals, then unrolled triangles (GLES2 has no 32-bit indices without an extension)
            val nrm = FloatArray(nv * 3)
            var t = 0
            while (t + 2 < s.tris.size) {
                val ia = s.tris[t] * 3; val ib = s.tris[t + 1] * 3; val ic = s.tris[t + 2] * 3
                val ux = s.points[ib] - s.points[ia]; val uy = s.points[ib + 1] - s.points[ia + 1]; val uz = s.points[ib + 2] - s.points[ia + 2]
                val vx = s.points[ic] - s.points[ia]; val vy = s.points[ic + 1] - s.points[ia + 1]; val vz = s.points[ic + 2] - s.points[ia + 2]
                val nx = uy * vz - uz * vy; val ny = uz * vx - ux * vz; val nz = ux * vy - uy * vx
                for (i in intArrayOf(ia, ib, ic)) { nrm[i] += nx; nrm[i + 1] += ny; nrm[i + 2] += nz }
                t += 3
            }
            val maxTris = 400_000
            val stepT = max(1, s.tris.size / 3 / maxTris)
            val nT = s.tris.size / 3 / stepT
            val a = FloatArray(nT * 3 * 9)
            var o = 0
            for (q in 0 until nT) for (c in 0 until 3) {
                val vi = s.tris[(q * stepT) * 3 + c]
                a[o] = s.points[vi * 3]; a[o + 1] = s.points[vi * 3 + 1]; a[o + 2] = s.points[vi * 3 + 2]
                val rgb = s.pointRgb?.takeIf { fileColors }
                if (rgb != null) { a[o + 3] = (rgb[vi] shr 16 and 255) / 255f; a[o + 4] = (rgb[vi] shr 8 and 255) / 255f; a[o + 5] = (rgb[vi] and 255) / 255f }
                else heightColor(s.points[vi * 3 + 1], a, o + 3)
                a[o + 6] = nrm[vi * 3]; a[o + 7] = nrm[vi * 3 + 1]; a[o + 8] = nrm[vi * 3 + 2]
                o += 9
            }
            GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vboTri)
            GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, a.size * 4, fb(a), GLES20.GL_STATIC_DRAW)
            nTri = nT * 3
        } else nTri = 0
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        traj = if (s.traj.size >= 6) fb(s.traj) else null; nTraj = s.traj.size / 3
        // 1 m floor grid over the bounds
        val r = (home[3] / 1.2f / 2 + 1).toInt().coerceAtMost(60)
        val gx = Math.round(home[0]); val gz = Math.round(home[2])
        val g = ArrayList<Float>()
        for (i in -r..r) {
            g.addAll(listOf((gx + i).toFloat(), floor, (gz - r).toFloat(), (gx + i).toFloat(), floor, (gz + r).toFloat()))
            g.addAll(listOf((gx - r).toFloat(), floor, (gz + i).toFloat(), (gx + r).toFloat(), floor, (gz + i).toFloat()))
        }
        grid = fb(g.toFloatArray()); nGrid = g.size / 3
    }

    override fun onDrawFrame(gl: GL10?) {
        pending?.let { pending = null; upload(it) }
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        val ex = cx + dist * cos(pitch) * sin(yaw); val ey = cy + dist * sin(pitch); val ez = cz + dist * cos(pitch) * cos(yaw)
        android.opengl.Matrix.setLookAtM(view, 0, ex, ey, ez, cx, cy, cz, 0f, 1f, 0f)
        android.opengl.Matrix.multiplyMM(mvp, 0, proj, 0, view, 0)
        GLES20.glUseProgram(program)
        GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)
        GLES20.glDisableVertexAttribArray(aCol); GLES20.glDisableVertexAttribArray(aNrm)
        GLES20.glVertexAttrib3f(aNrm, 0f, 1f, 0f); GLES20.glVertexAttrib3f(aCol, 1f, 1f, 1f)
        grid?.let { flat(it, nGrid, GLES20.GL_LINES, floatArrayOf(0.35f, 0.35f, 0.4f, 1f), 1f) }
        if (nTri > 0) drawVbo(vboTri, nTri, GLES20.GL_TRIANGLES, lit = 1f, size = 1f)
        if (nPts > 0) drawVbo(vboPts, nPts, GLES20.GL_POINTS, lit = 0f, size = 4f)
        traj?.let { flat(it, nTraj, GLES20.GL_LINE_STRIP, floatArrayOf(1f, 0.82f, 0.2f, 1f), 1f) }
        marker?.let { m ->
            GLES20.glDisable(GLES20.GL_DEPTH_TEST)
            flat(fb(m), 1, GLES20.GL_POINTS, floatArrayOf(1f, 0.15f, 0.15f, 1f), 26f)
        }
    }

    private fun drawVbo(vbo: Int, n: Int, mode: Int, lit: Float, size: Float) {
        GLES20.glUniform1f(uUseFlat, 0f); GLES20.glUniform1f(uLit, lit); GLES20.glUniform1f(uSize, size)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo)
        GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 36, 0)
        GLES20.glVertexAttribPointer(aCol, 3, GLES20.GL_FLOAT, false, 36, 12)
        GLES20.glVertexAttribPointer(aNrm, 3, GLES20.GL_FLOAT, false, 36, 24)
        GLES20.glEnableVertexAttribArray(aPos); GLES20.glEnableVertexAttribArray(aCol); GLES20.glEnableVertexAttribArray(aNrm)
        GLES20.glDrawArrays(mode, 0, n)
        GLES20.glDisableVertexAttribArray(aPos); GLES20.glDisableVertexAttribArray(aCol); GLES20.glDisableVertexAttribArray(aNrm)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
    }

    private fun flat(b: java.nio.FloatBuffer, n: Int, mode: Int, color: FloatArray, size: Float) {
        GLES20.glUniform1f(uUseFlat, 1f); GLES20.glUniform1f(uLit, 0f); GLES20.glUniform1f(uSize, size)
        GLES20.glUniform4fv(uFlat, 1, color, 0)
        GLES20.glLineWidth(4f)
        b.position(0)
        GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, b)
        GLES20.glEnableVertexAttribArray(aPos)
        GLES20.glDrawArrays(mode, 0, n)
        GLES20.glDisableVertexAttribArray(aPos)
    }

    private fun fb(a: FloatArray): java.nio.FloatBuffer =
        ByteBuffer.allocateDirect(a.size * 4).order(ByteOrder.nativeOrder()).asFloatBuffer().apply { put(a); position(0) }

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

    // ---- touch: orbit / zoom / pan
    private val scale = ScaleGestureDetector(ctx, object : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(d: ScaleGestureDetector): Boolean { dist = (dist / d.scaleFactor).coerceIn(0.2f, 500f); requestRender(); return true }
    })
    private var lastX = 0f; private var lastY = 0f; private var lastN = 0; private var lastDown = 0L

    @SuppressLint("ClickableViewAccessibility")
    override fun onTouchEvent(e: MotionEvent): Boolean {
        scale.onTouchEvent(e)
        var sx = 0f; var sy = 0f
        for (i in 0 until e.pointerCount) { sx += e.getX(i); sy += e.getY(i) }
        sx /= e.pointerCount; sy /= e.pointerCount
        when (e.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                parent?.requestDisallowInterceptTouchEvent(true)
                val now = SystemClock.uptimeMillis()
                if (now - lastDown < 300) { resetView(); performClick() }
                lastDown = now
            }
            MotionEvent.ACTION_MOVE -> if (e.pointerCount == lastN) {
                val dx = sx - lastX; val dy = sy - lastY
                if (e.pointerCount == 1) {
                    yaw -= dx * 0.008f; pitch = (pitch + dy * 0.008f).coerceIn(-1.5f, 1.5f)
                } else {
                    // pan in the view plane
                    val k = dist * 2 * tan(Math.toRadians(30.0)).toFloat() / max(height, 1)
                    val rx = cos(yaw); val rz = -sin(yaw)
                    val ux = -sin(pitch) * sin(yaw); val uy = cos(pitch); val uz = -sin(pitch) * cos(yaw)
                    cx += (-dx * rx + dy * ux) * k; cy += (dy * uy) * k; cz += (-dx * rz + dy * uz) * k
                }
                requestRender()
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> parent?.requestDisallowInterceptTouchEvent(false)
        }
        lastX = sx; lastY = sy; lastN = if (e.actionMasked == MotionEvent.ACTION_POINTER_UP) e.pointerCount - 1 else e.pointerCount
        if (e.actionMasked == MotionEvent.ACTION_POINTER_UP) {
            // recompute the centroid without the lifted pointer so the next move does not jump
            var ax = 0f; var ay = 0f; var n = 0
            for (i in 0 until e.pointerCount) if (i != e.actionIndex) { ax += e.getX(i); ay += e.getY(i); n++ }
            if (n > 0) { lastX = ax / n; lastY = ay / n }
        }
        return true
    }

    override fun performClick(): Boolean { super.performClick(); return true }
}
