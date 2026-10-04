package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Color
import android.graphics.Typeface
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.Gravity
import android.view.TextureView
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.CheckBox
import android.widget.FrameLayout
import android.widget.HorizontalScrollView
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.SeekBar
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.edit
import java.io.File
import java.util.Locale
import java.util.concurrent.Executors
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * In-app playback of one recorded session, the Android counterpart of ios/R2SCapture/SessionViewer.swift.
 *
 *  - Video: mode A plays video.mp4 with the ARCore depth map (raw or smoothed) alpha-blended on top,
 *    matched by timestamp, plus the frame's time, pose, intrinsics and exposure. Mode B plays every lens
 *    video side by side in sync, with the ToF depth as an extra tile, RAW / ToF counts and per-lens info.
 *  - 3D: extras/map/points.ply (or a mesh.ply) + the camera trajectory, with the camera at the playback
 *    time. Drag to orbit, pinch to zoom, two fingers to pan, double-tap to reset.
 *  - Map: the top-down occupancy map of the live map (extras/map) with the trajectory and current pose.
 *  - Sensors: IMU, magnetometer, barometer, GNSS, thermal / battery, light, proximity, ... with a cursor at
 *    the playback time (tap a chart to seek).
 *  - Info: session.json and the file list.
 * The transport bar (play / pause, scrubber, speed) drives every tab; sessions without video get a virtual
 * clock over the recording so the sensor charts can still be played.
 */
@SuppressLint("SetTextI18n")
class SessionViewerActivity : AppCompatActivity() {
    private val ui = Handler(Looper.getMainLooper())
    private val io = Executors.newSingleThreadExecutor()
    private val depthExec = Executors.newSingleThreadExecutor()
    private lateinit var dir: File
    private lateinit var content: FrameLayout
    private lateinit var tabBar: LinearLayout
    private lateinit var transport: LinearLayout
    private val playback = Playback(ui)
    private var data: ViewerData? = null
    private val tabs = LinkedHashMap<String, View?>()
    private val tabButtons = HashMap<String, Button>()
    private var current = ""
    private var scene: SceneView? = null
    private var rotation = 0

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        dir = File(intent.getStringExtra(EXTRA_DIR) ?: run { finish(); return })
        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setBackgroundColor(Color.BLACK) }
        root.addView(TextView(this).apply {
            text = dir.name; setTextColor(Color.WHITE); textSize = 15f; typeface = Typeface.MONOSPACE
            setPadding(dp(12), dp(10), dp(12), dp(4))
        })
        tabBar = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        root.addView(HorizontalScrollView(this).apply { addView(tabBar) })
        content = FrameLayout(this)
        root.addView(content, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))
        transport = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL; setPadding(dp(8), dp(4), dp(8), dp(8)) }
        root.addView(transport)
        setContentView(root)
        content.addView(text("Loading…"))
        io.execute {
            val d = try { ViewerData.load(dir) } catch (e: Exception) { null }
            runOnUiThread { if (!isDestroyed) { if (d == null) { content.removeAllViews(); content.addView(text("Could not read ${dir.name}")) } else build(d) } }
        }
    }

    override fun onPause() {
        playback.pause()
        scene?.onPause()
        super.onPause()
    }

    override fun onResume() {
        super.onResume()
        scene?.onResume()
    }

    override fun onDestroy() {
        playback.release()
        io.shutdownNow(); depthExec.shutdownNow()
        super.onDestroy()
    }

    // ------------------------------------------------------------------ layout

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    private fun text(s: String, size: Float = 13f, color: Int = Color.LTGRAY, mono: Boolean = false) = TextView(this).apply {
        text = s; textSize = size; setTextColor(color); setPadding(dp(12), dp(6), dp(12), dp(6))
        if (mono) typeface = Typeface.MONOSPACE
    }

    private fun button(label: String, onClick: () -> Unit) = Button(this).apply {
        text = label; isAllCaps = false; minWidth = 0; minimumWidth = 0; setPadding(dp(10), 0, dp(10), 0)
        setOnClickListener { onClick() }
    }

    private fun build(d: ViewerData) {
        data = d
        rotation = getSharedPreferences("viewer", Context.MODE_PRIVATE).getInt("rot_${d.mode}", d.defaultRotation)
        for (v in d.videos) playback.tracks.add(Playback.Track(v.name, v.file, v.firstT - d.t0))
        playback.duration = d.duration
        for (name in listOf("Video", "3D", "Map", "Sensors", "Info")) {
            tabs[name] = null
            val b = button(name) { show(name) }
            tabButtons[name] = b
            tabBar.addView(b)
        }
        buildTransport()
        show("Video")
    }

    private fun show(name: String) {
        val d = data ?: return
        current = name
        for ((n, b) in tabButtons) b.alpha = if (n == name) 1f else 0.55f
        if (tabs[name] == null) {
            val v = when (name) {
                "Video" -> videoTab(d)
                "3D" -> sceneTab(d)
                "Map" -> mapTab(d)
                "Sensors" -> sensorsTab(d)
                else -> infoTab(d)
            }
            tabs[name] = v
            content.addView(v, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))
        }
        // keep the video tab laid out (its TextureViews hold the players) but hidden
        for ((n, v) in tabs) v?.visibility = if (n == name) View.VISIBLE else View.INVISIBLE
        (0 until content.childCount).map { content.getChildAt(it) }.filter { it !in tabs.values }.forEach { content.removeView(it) }
        if (name == "3D") scene?.requestRender()
        playback.poke()
    }

    private fun buildTransport() {
        val play = button("Play") { playback.toggle() }
        val seek = SeekBar(this).apply { max = 1000 }
        val label = TextView(this).apply { setTextColor(Color.WHITE); textSize = 12f; typeface = Typeface.MONOSPACE; setPadding(dp(6), 0, dp(6), 0) }
        val speeds = floatArrayOf(0.25f, 0.5f, 1f, 2f, 4f)
        val speed = button("1x") {}
        speed.setOnClickListener {
            val k = (speeds.indexOfFirst { it == playback.speed } + 1) % speeds.size
            playback.setSpeed(speeds[k])
        }
        transport.addView(play)
        transport.addView(seek, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        transport.addView(label)
        transport.addView(speed)
        var scrubbing = false
        seek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) { if (fromUser) playback.seek(p / 1000.0 * playback.duration) }
            override fun onStartTrackingTouch(s: SeekBar) { scrubbing = true; playback.pause() }
            override fun onStopTrackingTouch(s: SeekBar) { scrubbing = false }
        })
        fun refresh() {
            play.text = if (playback.playing) "Pause" else "Play"
            speed.text = (if (playback.speed < 1) String.format(Locale.US, "%.2gx", playback.speed) else String.format(Locale.US, "%.0fx", playback.speed))
            label.text = String.format(Locale.US, "%.1f / %.1f s", playback.time, playback.duration)
            if (!scrubbing && playback.duration > 0) seek.progress = (playback.time / playback.duration * 1000).toInt()
            transport.visibility = if (playback.duration > 0) View.VISIBLE else View.GONE
        }
        playback.onState = { refresh() }
        playback.listen { refresh() }
        refresh()
    }

    // ------------------------------------------------------------------ video tab

    private fun videoTab(d: ViewerData): View {
        val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        val scroll = ScrollView(this).apply { addView(col) }
        when {
            d.videos.isEmpty() -> col.addView(text(when (d.mode) {
                "sensors" -> "Sensors-only session: no video. The Sensors tab plays the sensor logs on the session clock."
                else -> "No video in this session."
            }))
            d.mode == "multicam" -> multicamVideo(d, col)
            else -> rgbdVideo(d, col)
        }
        return scroll
    }

    private fun rotateButton(onChange: () -> Unit) = button("Rotate ${rotation}°") {}.also { b ->
        b.setOnClickListener {
            rotation = (rotation + 90) % 360
            b.text = "Rotate ${rotation}°"
            getSharedPreferences("viewer", Context.MODE_PRIVATE).edit { putInt("rot_${data?.mode}", rotation) }
            onChange()
        }
    }

    /** Video + depth overlay tile; returns (frame, apply-rotation). */
    private class Tile(val frame: AspectFrame, val texture: TextureView?, val overlay: ImageView?, val w: Int, val h: Int) {
        var bmpW = 0; var bmpH = 0
        fun apply(rot: Int) {
            frame.aspect = if (rot % 180 == 0) w.toFloat() / h else h.toFloat() / w
            val vw = frame.width.toFloat(); val vh = frame.height.toFloat()
            if (vw <= 0 || vh <= 0) return
            texture?.setTransform(rotateFit(vw, vh, vw, vh, rot))
            if (overlay != null && bmpW > 0) overlay.imageMatrix = rotateFit(bmpW.toFloat(), bmpH.toFloat(), vw, vh, rot)
        }
    }

    private fun tile(w: Int, h: Int, video: Boolean, overlay: Boolean): Tile {
        val f = AspectFrame(this, w.toFloat() / max(h, 1)).apply { setBackgroundColor(Color.rgb(20, 20, 20)) }
        val tv = if (video) TextureView(this).also { f.addView(it, FrameLayout.LayoutParams(-1, -1)) } else null
        val iv = if (overlay) ImageView(this).apply { scaleType = ImageView.ScaleType.MATRIX }.also { f.addView(it, FrameLayout.LayoutParams(-1, -1)) } else null
        val t = Tile(f, tv, iv, max(w, 1), max(h, 1))
        f.addOnLayoutChangeListener { _, _, _, _, _, _, _, _, _ -> t.apply(rotation) }
        return t
    }

    private fun setDepth(t: Tile, bmp: Bitmap) {
        t.bmpW = bmp.width; t.bmpH = bmp.height
        t.overlay?.setImageBitmap(bmp)
        t.apply(rotation)
    }

    private fun rgbdVideo(d: ViewerData, col: LinearLayout) {
        val v = d.videos[0]
        val tile = tile(v.w, v.h, video = true, overlay = true)
        playback.attach(playback.tracks[0], tile.texture!!)
        tile.frame.maxHeightPx = (resources.displayMetrics.heightPixels * 0.6).toInt()
        col.addView(tile.frame, LinearLayout.LayoutParams(-1, -2).apply { gravity = Gravity.CENTER_HORIZONTAL })
        tile.frame.post { tile.apply(rotation) }

        val f = d.frames
        var useSmooth = false
        var showDepth = f.depth.isNotEmpty() || f.smooth.isNotEmpty()
        if (f.depth.isEmpty() && f.smooth.isNotEmpty()) useSmooth = true
        val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        row.addView(rotateButton { tile.apply(rotation) })
        val info = text("", 12f, mono = true)
        var shown = -2; var busy = false
        fun updateDepth(force: Boolean = false) {
            val refs = if (useSmooth) f.smooth else f.depth
            val ts = if (useSmooth) f.smoothTimes else f.depthTimes
            val blob = if (useSmooth) d.smoothBlob else d.depthBlob
            if (!showDepth || refs.isEmpty() || blob == null) { tile.overlay?.visibility = View.INVISIBLE; return }
            tile.overlay?.visibility = View.VISIBLE
            val st = d.sessionTime(0, playback.time)
            val k = nearestIndex(ts, st)
            if (k < 0 || (k == shown && !force) || busy) return
            busy = true
            val ref = refs[k]
            depthExec.execute {
                val mm = Depth.load(blob, ref)
                val bmp = mm?.let { Bitmap.createBitmap(Depth.colorize(it), ref.w, ref.h, Bitmap.Config.ARGB_8888) }
                val med = mm?.let { Depth.medianMm(it) } ?: 0
                runOnUiThread {
                    busy = false
                    shown = k
                    if (bmp != null) setDepth(tile, bmp)
                    depthLine = String.format(Locale.US, "%s depth %d / %d  %dx%d  dt %+.3f s  median %.2f m",
                        if (useSmooth) "smoothed" else "raw", k + 1, refs.size, ref.w, ref.h, ref.t - st, med / 1000.0)
                    updateInfo(d, info)
                }
            }
        }
        if (f.depth.isNotEmpty() || f.smooth.isNotEmpty()) {
            row.addView(CheckBox(this).apply {
                text = "Depth overlay"; setTextColor(Color.WHITE); isChecked = showDepth
                setOnCheckedChangeListener { _, c -> showDepth = c; updateDepth(true) }
            })
            if (f.depth.isNotEmpty() && f.smooth.isNotEmpty()) row.addView(button("raw") {}.also { b ->
                b.setOnClickListener { useSmooth = !useSmooth; b.text = if (useSmooth) "smoothed" else "raw"; shown = -2; updateDepth(true) }
            })
        }
        col.addView(row)
        if (f.depth.isNotEmpty() || f.smooth.isNotEmpty()) {
            val op = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
            op.addView(text("Opacity", 12f))
            op.addView(SeekBar(this).apply {
                max = 100; progress = 50
                setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                    override fun onProgressChanged(s: SeekBar, p: Int, fromUser: Boolean) { tile.overlay?.alpha = p / 100f }
                    override fun onStartTrackingTouch(s: SeekBar) {}
                    override fun onStopTrackingTouch(s: SeekBar) {}
                })
            }, LinearLayout.LayoutParams(0, -2, 1f))
            col.addView(op)
            tile.overlay?.alpha = 0.5f
            col.addView(DepthLegend(this), LinearLayout.LayoutParams(-1, dp(28)).apply { setMargins(dp(12), 0, dp(12), 0) })
        } else col.addView(text("No depth maps in this session."))
        col.addView(info)
        playback.listen { updateDepth(); updateInfo(d, info) }
        updateDepth(); updateInfo(d, info)
    }

    private var depthLine = ""

    private fun updateInfo(d: ViewerData, tv: TextView) {
        if (tabs["Video"]?.visibility == View.INVISIBLE) return
        val st = d.sessionTime(0, playback.time)
        val f = d.frames
        val sb = StringBuilder()
        sb.append(String.format(Locale.US, "t %.3f s  (boottime %.3f)\n", st - d.t0, st))
        val k = nearestIndex(f.times, st)
        if (k >= 0) {
            val r = f.frames[k]
            sb.append(String.format(Locale.US, "frame %d / %d  %dx%d  track %s\n", r.i + 1, f.frames.size, r.w, r.h, r.track ?: "?"))
            if (r.hasPose) sb.append(String.format(Locale.US, "position x %.3f  y %.3f  z %.3f m  yaw %.1f°\n", r.x(), r.y(), r.z(), Math.toDegrees(r.heading())))
            r.K?.takeIf { it.size >= 4 }?.let { K -> sb.append(String.format(Locale.US, "K fx %.1f fy %.1f cx %.1f cy %.1f\n", K[0], K[1], K[2], K[3])) }
            val parts = mutableListOf<String>()
            if (!r.exp.isNaN()) parts.add(String.format(Locale.US, "exposure %.2f ms", r.exp * 1000))
            if (!r.iso.isNaN()) parts.add(String.format(Locale.US, "ISO %.0f", r.iso))
            if (!r.amb.isNaN()) parts.add(String.format(Locale.US, "ambient %.2f", r.amb))
            if (parts.isNotEmpty()) sb.append(parts.joinToString("  ")).append('\n')
        }
        if (depthLine.isNotEmpty()) sb.append(depthLine)
        tv.text = sb.toString().trimEnd()
    }

    private fun multicamVideo(d: ViewerData, col: LinearLayout) {
        val tiles = ArrayList<Tile>()
        val cells = ArrayList<View>()
        d.videos.forEachIndexed { k, v ->
            val t = tile(v.w, v.h, video = true, overlay = false)
            playback.attach(playback.tracks[k], t.texture!!)
            tiles.add(t)
            cells.add(LinearLayout(this).apply {
                orientation = LinearLayout.VERTICAL
                addView(t.frame, LinearLayout.LayoutParams(-1, -2))
                addView(TextView(this@SessionViewerActivity).apply { text = v.name; setTextColor(Color.WHITE); textSize = 11f; gravity = Gravity.CENTER })
            })
        }
        var tofTile: Tile? = null
        val tof = d.tof
        if (tof != null && tof.depth.isNotEmpty() && d.tofBlob != null) {
            val r0 = tof.depth[0]
            val t = tile(r0.w, r0.h, video = false, overlay = true)
            t.overlay?.alpha = 1f
            tofTile = t; tiles.add(t)
            cells.add(LinearLayout(this).apply {
                orientation = LinearLayout.VERTICAL
                addView(t.frame, LinearLayout.LayoutParams(-1, -2))
                addView(TextView(this@SessionViewerActivity).apply { text = "tof_depth"; setTextColor(Color.WHITE); textSize = 11f; gravity = Gravity.CENTER })
            })
        }
        // two columns
        var k = 0
        while (k < cells.size) {
            val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
            for (c in 0 until 2) {
                val lp = LinearLayout.LayoutParams(0, -2, 1f).apply { setMargins(dp(3), dp(3), dp(3), dp(3)) }
                if (k + c < cells.size) row.addView(cells[k + c], lp) else row.addView(View(this), lp)
            }
            col.addView(row)
            k += 2
        }
        val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        row.addView(rotateButton { tiles.forEach { it.apply(rotation) } })
        col.addView(row)
        tiles.forEach { t -> t.frame.post { t.apply(rotation) } }
        if (tofTile != null) col.addView(DepthLegend(this), LinearLayout.LayoutParams(-1, dp(28)).apply { setMargins(dp(12), 0, dp(12), 0) })
        val sb = StringBuilder()
        sb.append("RAW DNG: ${d.rawFiles.size} file(s)")
        if (d.rawFiles.isNotEmpty()) sb.append(" (" + d.rawFiles.map { it.name.substringBeforeLast('_') }.distinct().joinToString() + ")")
        sb.append("\nToF depth: ${tof?.depth?.size ?: 0} frame(s)")
        d.meta?.obj("android")?.str("tof_error")?.let { sb.append(" ($it)") }
        d.meta?.obj("android")?.str("raw_error")?.let { sb.append("\nRAW: $it") }
        col.addView(text(sb.toString(), 12f))
        val info = text("", 12f, mono = true)
        col.addView(info)
        var shown = -2; var busy = false
        fun update() {
            if (tabs["Video"]?.visibility == View.INVISIBLE) return
            val st = d.t0 + playback.time
            val s = StringBuilder(String.format(Locale.US, "t %.3f s  (boottime %.3f)\n", playback.time, st))
            d.videos.forEachIndexed { i, v ->
                s.append("\n").append(v.name).append("  ").append("${v.w}x${v.h}").append("  ").append(v.codec)
                    .append("  ").append(v.samples).append(" samples")
                v.physicalId?.let { s.append("  phys ").append(it) }
                val fr = v.frames
                val j = nearestIndex(fr.times, d.sessionTime(i, playback.time))
                if (j >= 0) {
                    val r = fr.frames[j]
                    s.append(String.format(Locale.US, "\n  sample %d", r.i))
                    r.K?.takeIf { it.size >= 4 }?.let { K -> s.append(String.format(Locale.US, "  K %.0f %.0f %.0f %.0f", K[0], K[1], K[2], K[3])) }
                    if (!r.exp.isNaN()) s.append(String.format(Locale.US, "  exp %.2f ms", r.exp * 1000))
                    if (!r.iso.isNaN()) s.append(String.format(Locale.US, "  ISO %.0f", r.iso))
                }
            }
            val raw = d.rawTimes
            if (raw.isNotEmpty()) {
                val j = nearestIndex(raw, st)
                s.append(String.format(Locale.US, "\n\nnearest RAW: %s (dt %+.2f s)", d.rawFiles[j].name, raw[j] - st))
            }
            info.text = s.toString()
            val tt = tofTile
            if (tt != null && tof != null && !busy) {
                val j = nearestIndex(tof.depthTimes, st)
                if (j >= 0 && j != shown) {
                    busy = true
                    val ref = tof.depth[j]
                    depthExec.execute {
                        val bmp = Depth.load(d.tofBlob!!, ref)?.let { Bitmap.createBitmap(Depth.colorize(it), ref.w, ref.h, Bitmap.Config.ARGB_8888) }
                        runOnUiThread { busy = false; shown = j; if (bmp != null) setDepth(tt, bmp) }
                    }
                }
            }
        }
        playback.listen { update() }
        update()
    }

    // ------------------------------------------------------------------ 3D tab

    private fun sceneTab(d: ViewerData): View {
        val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        val poses = d.frames.poses
        if (d.plyFile == null && poses.isEmpty() && d.trajectory.isEmpty()) {
            col.addView(text(if (d.mode == "multicam")
                "Multi-cam recordings have no camera poses or point cloud. Run tools/recover_poses.py on a computer to get them."
            else "No point cloud, mesh or trajectory in this session."))
            return col
        }
        val sv = SceneView(this)
        scene = sv
        col.addView(sv, LinearLayout.LayoutParams(-1, 0, 1f))
        val legend = text("yellow: trajectory   red: camera   points coloured by height (blue floor → red 2 m)", 11f)
        col.addView(legend)
        val row = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL }
        row.addView(button("Reset view") { sv.resetView() })
        row.addView(text("Drag to orbit, pinch to zoom, two fingers to pan.", 11f))
        col.addView(row)
        io.execute {
            val ply = d.plyFile?.let { Ply.read(it) }
            val traj = d.trajectory
            runOnUiThread {
                sv.setScene(SceneView.Scene(ply?.xyz ?: FloatArray(0), ply?.rgb, ply?.tris ?: IntArray(0), traj))
                val what = when {
                    ply == null && d.plyFile != null -> "could not read ${d.plyFile.name}"
                    ply == null -> "no point cloud"
                    ply.tris.isNotEmpty() -> "${d.plyFile!!.name}: ${ply.vertexCount} vertices, ${ply.tris.size / 3} triangles"
                    else -> "${d.plyFile!!.relativeTo(d.dir).path}: ${ply.vertexCount} points"
                }
                legend.text = "$what · ${traj.size / 3} trajectory points\nyellow: trajectory   red: camera   " +
                    (if (ply?.rgb != null) "PLY colours" else "colour by height (blue floor → red 2 m)")
            }
        }
        val update = {
            val p = d.poseAt(d.sessionTime(0, playback.time))
            sv.setMarker(p?.let { floatArrayOf(it.x().toFloat(), it.y().toFloat(), it.z().toFloat()) })
        }
        playback.listen { if (current == "3D") update() }
        update()
        return col
    }

    // ------------------------------------------------------------------ map tab

    private fun mapTab(d: ViewerData): View {
        val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        if (d.mapDir == null && d.frames.poses.isEmpty() && d.trajectory.isEmpty()) {
            col.addView(text(if (d.mode == "multicam") "Multi-cam recordings have no on-device poses, so there is no map."
            else "No top-down map (extras/map) or trajectory in this session."))
            return col
        }
        val mv = MapView(this).apply { title = "occupancy · trajectory · camera" }
        col.addView(mv, LinearLayout.LayoutParams(-1, 0, 1f))
        val info = text("", 12f)
        col.addView(info)
        // trajectory for drawing (MapView keeps at most 4000 points)
        val tr = d.trajectory
        val n = tr.size / 3
        val step = max(1, (n + 3999) / 4000)
        val trail = ArrayList<FloatArray>()
        var i = 0; while (i < n) { trail.add(floatArrayOf(tr[i * 3], tr[i * 3 + 1], tr[i * 3 + 2])); i += step }
        io.execute {
            var line = ""
            val bm = d.mapDir?.let { md ->
                try {
                    val m = MapBuilder.load(md)
                    MiniJson.obj(File(md, "map.json").readText())?.let { j ->
                        line = String.format(Locale.US, "map %.0f m² known · %.1f m travelled · %.0f points · %.2f m cells",
                            j.num("known_area_m2") ?: 0.0, j.num("travelled_m") ?: 0.0, j.num("points") ?: 0.0, j.num("res") ?: 0.0)
                    }
                    m.bitmap()
                } catch (e: Exception) { line = "could not read the map: $e"; null }
            }
            val png = if (bm == null) d.mapDir?.let { File(it, "occupancy.png") }?.takeIf { it.isFile }?.let { BitmapFactory.decodeFile(it.path) } else null
            runOnUiThread {
                if (bm != null) mv.setMap(bm.first, bm.second)
                if (png != null) {
                    mv.visibility = View.GONE
                    col.addView(ImageView(this).apply { setImageBitmap(png); scaleType = ImageView.ScaleType.FIT_CENTER }, 0, LinearLayout.LayoutParams(-1, 0, 1f))
                    line = "occupancy.png (no grid metadata)"
                }
                info.text = (if (line.isNotEmpty()) "$line\n" else "") + "red: obstacle · light: free · yellow: trajectory · green arrow: camera"
                update(d, mv, trail)
            }
        }
        playback.listen { if (current == "Map") update(d, mv, trail) }
        return col
    }

    private fun update(d: ViewerData, mv: MapView, trail: List<FloatArray>) {
        val p = d.poseAt(d.sessionTime(0, playback.time))
        if (p != null) mv.setPose(p.x().toFloat(), p.z().toFloat(), p.heading().toFloat(), trail)
        else trail.lastOrNull()?.let { mv.setPose(it[0], it[2], 0f, trail) }
    }

    // ------------------------------------------------------------------ sensors tab

    private fun sensorsTab(d: ViewerData): View {
        val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(6), dp(6), dp(6), dp(6)) }
        val scroll = ScrollView(this).apply { addView(col) }
        val loading = text("Reading sensor logs…")
        col.addView(loading)
        val charts = ArrayList<ChartView>()
        io.execute {
            val plots = SensorCharts.build(d)
            runOnUiThread {
                col.removeView(loading)
                if (plots.charts.isEmpty()) col.addView(text("No sensor logs in this session."))
                for (c in plots.charts) {
                    val cv = ChartView(this).apply {
                        title = c.title; x0 = plots.x0; x1 = plots.x1; series = c.series
                        onSeek = { t -> playback.pause(); playback.seek(t) }
                    }
                    charts.add(cv)
                    col.addView(cv, LinearLayout.LayoutParams(-1, -2).apply { setMargins(0, dp(4), 0, dp(4)) })
                }
                plots.footer.takeIf { it.isNotEmpty() }?.let { col.addView(text(it, 12f, mono = true)) }
                val cur = playback.time.toFloat()
                charts.forEach { it.cursor = cur }
            }
        }
        playback.listen { t -> if (current == "Sensors") charts.forEach { it.cursor = t.toFloat() } }
        return scroll
    }

    // ------------------------------------------------------------------ info tab

    private fun infoTab(d: ViewerData): View {
        val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        val scroll = ScrollView(this).apply { addView(col) }
        val s = d.meta?.num("start_uptime"); val e = d.meta?.num("end_uptime")
        col.addView(text("${modeLabel(d.mode)} · ${fmtDuration(if (s != null && e != null) e - s else null)} · " +
            "${fmtBytes(d.files.sumOf { it.second })} · ${d.files.size} files" +
            (if (d.meta == null) " · incomplete (no session.json)" else "") + "\n${d.dir.absolutePath}", 12f, Color.WHITE))
        col.addView(text("session.json", 14f, Color.WHITE))
        col.addView(text(d.meta?.let { MiniJson.pretty(it) } ?: "(no session.json)", 10f, mono = true).apply { setTextIsSelectable(true) })
        col.addView(text("Files", 14f, Color.WHITE))
        val w = d.files.maxOfOrNull { it.first.length }?.coerceAtMost(48) ?: 10
        col.addView(text(d.files.joinToString("\n") { (p, s) -> p.padEnd(w) + "  " + fmtBytes(s) }, 10f, mono = true).apply { setTextIsSelectable(true) })
        return scroll
    }

    companion object { const val EXTRA_DIR = "dir" }
}

/** Everything the viewer needs from a session folder, read once in the background. */
class ViewerData(
    val dir: File,
    val meta: Map<String, Any?>?,
    val mode: String,
    val videos: List<Video>,
    val t0: Double,
    val duration: Double,
    val frames: FrameData,
    val depthBlob: File?,
    val smoothBlob: File?,
    val tof: FrameData?,
    val tofBlob: File?,
    val rawFiles: List<File>,
    val rawTimes: DoubleArray,
    val plyFile: File?,
    val mapDir: File?,
    /** xyz of the camera path (frames.jsonl poses, else extras/map/trajectory.csv). */
    val trajectory: FloatArray,
    val files: List<Pair<String, Long>>,
    val defaultRotation: Int,
) {
    class Video(val name: String, val file: File, val firstT: Double, val w: Int, val h: Int, val pts: PtsTable?, val frames: FrameData,
                val codec: String, val samples: Int, val physicalId: String?)

    /** Session (boottime) time shown by video [k] at playback time [time] (exact sample time when pts.csv exists). */
    fun sessionTime(k: Int, time: Double): Double {
        val v = videos.getOrNull(k) ?: return t0 + time
        val local = time - (v.firstT - t0)
        return if (local >= 0) v.pts?.sessionTime(local) ?: (t0 + time) else t0 + time
    }

    fun poseAt(st: Double): FrameRec? {
        val p = frames.poses
        if (p.isEmpty()) return null
        return p[floorIndex(frames.poseTimes, st)]
    }

    companion object {
        fun load(dir: File): ViewerData {
            val meta = File(dir, "session.json").takeIf { it.isFile }?.let { MiniJson.obj(it.readText()) }
            val mode = meta?.str("mode") ?: dir.name.split('_').drop(2).joinToString("_").ifEmpty { "?" }
            val frames = Frames.read(File(dir, "frames.jsonl"))
            val videos = ArrayList<Video>()
            var rot = 0
            if (mode == "multicam") {
                val cams = File(dir, "cams")
                val streams = (meta?.obj("android")?.obj("multicam")?.get("streams") as? List<*>)?.mapNotNull {
                    @Suppress("UNCHECKED_CAST") (it as? Map<String, Any?>)
                } ?: emptyList()
                val names = cams.listFiles { f -> f.name.endsWith(".mp4") }?.map { it.name.removeSuffix(".mp4") }?.sorted() ?: emptyList()
                val ordered = streams.mapNotNull { it.str("name") }.filter { it in names } + names.filter { n -> streams.none { it.str("name") == n } }
                for (n in ordered) {
                    val st = streams.firstOrNull { it.str("name") == n }
                    val pts = PtsTable.read(File(cams, "$n.mp4.pts.csv"))
                    val fr = Frames.read(File(cams, "$n.jsonl"))
                    val first = pts?.firstT ?: fr.frames.firstOrNull()?.t ?: continue
                    val w = st?.num("w")?.toInt() ?: fr.frames.firstOrNull()?.w ?: 4
                    val h = st?.num("h")?.toInt() ?: fr.frames.firstOrNull()?.h ?: 3
                    videos.add(Video(n, File(cams, "$n.mp4"), first, w, h, pts, fr,
                        (st?.str("codec") ?: "").substringAfter("video/"), pts?.size ?: fr.frames.size, st?.get("physical_id") as? String))
                }
                rot = sensorOrientation(File(dir, "extras/camera_inventory.json"),
                    meta?.obj("android")?.obj("multicam")?.str("logical_id"))
            } else {
                val vf = File(dir, "video.mp4")
                if (vf.isFile) {
                    val pts = PtsTable.read(File(dir, "video.mp4.pts.csv"))
                    val vm = meta?.obj("video")
                    val first = pts?.firstT ?: frames.frames.firstOrNull()?.t
                    if (first != null) videos.add(Video("video", vf, first,
                        vm?.num("width")?.toInt() ?: frames.frames.firstOrNull()?.w ?: 4,
                        vm?.num("height")?.toInt() ?: frames.frames.firstOrNull()?.h ?: 3,
                        pts, frames, vm?.str("codec") ?: "", pts?.size ?: frames.frames.size, null))
                }
                val chars = File(dir, "extras").listFiles { f -> f.name.startsWith("camera_characteristics_arcore_") }?.firstOrNull()
                if (chars != null) rot = (MiniJson.obj(chars.readText())?.num("android.sensor.orientation") ?: 0.0).toInt()
            }
            val start = meta?.num("start_uptime"); val end = meta?.num("end_uptime")
            val t0 = videos.minOfOrNull { it.firstT } ?: start ?: frames.frames.firstOrNull()?.t ?: 0.0
            val vidEnd = videos.maxOfOrNull { v -> v.firstT + (v.pts?.durationS ?: ((v.frames.frames.lastOrNull()?.t ?: v.firstT) - v.firstT)) }
            val duration = max(0.0, (vidEnd ?: end ?: t0) - t0)

            val tofF = File(dir, "cams/tof_depth.jsonl")
            val tof = if (tofF.isFile) Frames.read(tofF, "w" to "h") else null
            val raws = File(dir, "cams/raw").listFiles { f -> f.name.endsWith(".dng") }?.sortedBy { it.name } ?: emptyList()
            val rawPairs = raws.mapNotNull { f -> f.name.removeSuffix(".dng").substringAfterLast('_').toDoubleOrNull()?.let { f to it } }.sortedBy { it.second }

            val mapDir = File(dir, "extras/map").takeIf { File(it, "map.json").isFile && File(it, "occupancy.bin").isFile }
                ?: File(dir, "extras/map").takeIf { File(it, "occupancy.png").isFile }
            val ply = listOf("mesh.ply", "extras/map/points.ply").map { File(dir, it) }.firstOrNull { it.isFile && it.length() > 0 }
            val traj: FloatArray = if (frames.poses.isNotEmpty()) {
                FloatArray(frames.poses.size * 3).also { a -> frames.poses.forEachIndexed { k, p -> a[k * 3] = p.x().toFloat(); a[k * 3 + 1] = p.y().toFloat(); a[k * 3 + 2] = p.z().toFloat() } }
            } else {
                CsvTable.read(File(dir, "extras/map/trajectory.csv"))?.let { t ->
                    val x = t.col("x"); val y = t.col("y"); val z = t.col("z")
                    if (x == null || y == null || z == null) null
                    else FloatArray(t.rows * 3).also { a -> for (k in 0 until t.rows) { a[k * 3] = x[k].toFloat(); a[k * 3 + 1] = y[k].toFloat(); a[k * 3 + 2] = z[k].toFloat() } }
                } ?: FloatArray(0)
            }
            return ViewerData(dir, meta, mode, videos, t0, duration, frames,
                File(dir, "depth.zlib.bin").takeIf { it.isFile }, File(dir, "depth_smooth.zlib.bin").takeIf { it.isFile },
                tof, File(dir, "cams/tof_depth.zlib.bin").takeIf { it.isFile },
                rawPairs.map { it.first }, rawPairs.map { it.second }.toDoubleArray(),
                ply, mapDir, traj, SessionScan.files(dir), normRot(rot))
        }

        private fun normRot(r: Int) = ((r % 360 + 360) % 360) / 90 * 90

        private fun sensorOrientation(inv: File, id: String?): Int {
            if (!inv.isFile || id == null) return 0
            val cams = MiniJson.obj(inv.readText())?.obj("cameras") ?: return 0
            return (cams.obj(id)?.num("android.sensor.orientation") ?: 0.0).toInt()
        }
    }
}

/** Builds the Sensors tab's charts (background thread). */
object SensorCharts {
    class Chart(val title: String, val series: List<ChartView.Series>)
    class Result(val charts: List<Chart>, val x0: Float, val x1: Float, val footer: String)

    fun build(d: ViewerData): Result {
        val tables = HashMap<String, CsvTable?>()
        fun table(file: String): CsvTable? = tables.getOrPut(file) {
            when (file) {
                "frames" -> SensorPlots.cameraTable(d.frames.frames.ifEmpty { d.videos.firstOrNull()?.frames?.frames ?: emptyList() })
                else -> CsvTable.read(File(d.dir, file))
            }
        }
        val defs = SensorPlots.defs.filter { p -> table(p.file)?.let { t -> t.rows > 0 && p.columns.any { t.has(it) } } == true }
        val raw = RawSensors.read(File(d.dir, "extras/sensors_raw.csv"), SensorPlots.rawCovered)
        // x range: the session clock, widened to the sensor data
        var lo = 0.0; var hi = max(d.duration, 1e-3)
        fun span(t: DoubleArray) { t.firstOrNull { !it.isNaN() }?.let { lo = min(lo, it - d.t0) }; t.lastOrNull { !it.isNaN() }?.let { hi = max(hi, it - d.t0) } }
        defs.forEach { p -> table(p.file)?.col("t")?.let { span(it) } }
        raw.values.forEach { t -> t.col("t")?.let { span(it) } }
        if (d.videos.isNotEmpty()) { lo = max(lo, -5.0); hi = min(hi, d.duration + 5) }   // keep the video's window readable
        fun series(t: CsvTable, cols: List<String>, names: List<String> = cols): List<ChartView.Series> {
            val ts = t.col("t") ?: return emptyList()
            val rel = DoubleArray(ts.size) { ts[it] - d.t0 }
            return cols.mapIndexedNotNull { k, c -> t.col(c)?.let { v -> ChartView.Series(names[k], decimateMinMax(rel, v, lo, hi, 400)) } }
                .filter { it.xy.size >= 4 }
        }
        val charts = ArrayList<Chart>()
        for (p in defs) {
            val s = series(table(p.file)!!, p.columns.filter { table(p.file)!!.has(it) })
            if (s.isNotEmpty()) charts.add(Chart(if (p.unit.isEmpty()) p.title else "${p.title} (${p.unit})", s))
        }
        for ((name, t) in raw) {
            val cols = t.columns.filter { it != "t" }
            val s = series(t, cols)
            if (s.isNotEmpty()) charts.add(Chart("$name (${SensorPlots.rawUnits[name] ?: "raw"}) · sensors_raw", s))
        }
        val footer = StringBuilder()
        table("location.csv")?.let { loc ->
            val lat = loc.col("lat"); val lon = loc.col("lon")
            if (lat != null && lon != null) {
                val k = (lat.indices).lastOrNull { !lat[it].isNaN() && !lon[it].isNaN() }
                if (k != null) footer.append(String.format(Locale.US, "Last GPS fix: %.6f, %.6f (%d fixes)", lat[k], lon[k], loc.rows))
            }
        }
        if (d.mode == "sensors" || d.videos.isEmpty()) {
            if (footer.isNotEmpty()) footer.append('\n')
            footer.append("Time 0 = session start; tap a chart to move the cursor.")
        } else {
            if (footer.isNotEmpty()) footer.append('\n')
            footer.append("Time 0 = first video frame; tap a chart to seek the video.")
        }
        return Result(charts, lo.toFloat(), hi.toFloat(), footer.toString())
    }
}
