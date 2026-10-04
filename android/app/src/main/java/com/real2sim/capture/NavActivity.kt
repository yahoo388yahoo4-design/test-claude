package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.Color
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import android.os.Bundle
import android.os.SystemClock
import android.text.InputType
import android.util.Log
import android.view.Gravity
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.view.WindowManager
import android.widget.Button
import android.widget.CheckBox
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.HorizontalScrollView
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Plane
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import org.json.JSONArray
import org.json.JSONObject
import java.io.BufferedWriter
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.Executors
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min

/**
 * Navigation mode: the phone is the robot's sensor head.
 *
 *  - SLAM: ARCore 6-DoF tracking + depth (depth-from-motion on phones without a depth sensor) build a
 *    live occupancy map and point cloud (MapBuilder). A map saved by a mode A capture or by this screen
 *    can be loaded and aligned to the live one (correlative scan matching), or merged assuming the same
 *    start pose.
 *  - Goal: tap the top-down map, or tap the floor in the camera view.
 *  - Path: A* on the inflated grid, replanned twice a second; drawn in 3D over the camera image and on
 *    the map.
 *  - Obstacles: a "virtual lidar" (nearest obstacle per 5 degrees) from the newest depth map plus the
 *    map; front / left / right distances, colour-coded, with a parking-sensor beep.
 *  - Guidance: arrow + text + speech ("turn left 30 degrees", "go straight 1.5 meters").
 *  - Robot: optional WebSocket link (robot/PROTOCOL.md). Auto-drive streams velocity commands from pure
 *    pursuit, slowing down near obstacles and stopping inside the stop distance or when tracking is lost.
 *    Manual "move X cm at V cm/s" and "turn A deg" commands, with the phone measuring the actual motion.
 */
class NavActivity : AppCompatActivity(), GLSurfaceView.Renderer {
    private lateinit var gl: GLSurfaceView
    private lateinit var hud: HudView
    private lateinit var mapView: MapView
    private lateinit var statusView: TextView
    private var session: Session? = null
    private val bg = BackgroundRenderer()
    private var map = MapBuilder()
    private lateinit var cloud: PointCloudRenderer
    private var saved: MapBuilder? = null
    private val link = RobotLink { msg -> status(msg) }
    private lateinit var guide: Guidance
    private val exec = Executors.newSingleThreadExecutor()
    private val taps = ConcurrentLinkedQueue<FloatArray>()

    // parameters (cm in the UI, metres here)
    @Volatile private var mountHeight = 0.30f
    @Volatile private var robotRadius = 0.20f
    @Volatile private var robotHeight = 0.50f
    @Volatile private var vMax = 0.25f
    @Volatile private var wMax = Math.toRadians(60.0).toFloat()
    @Volatile private var autoDrive = false

    // state
    @Volatile private var goal: FloatArray? = null
    @Volatile private var path: List<FloatArray>? = null
    @Volatile private var planning = false
    private var lastPlanMs = 0L
    private var lastVelMs = 0L
    private var lastHudMs = 0L
    private var lastMapUiMs = 0L
    private var lastDepthTs = -1L
    private var frames = 0
    private var depthMaps = 0
    private var speed = 0f
    private var lastPose: FloatArray? = null      // x, z, heading, t
    private var arrivedSaid = false
    private var trackingWasOk = false
    private var viewW = 1; private var viewH = 1
    private val viewM = FloatArray(16); private val projM = FloatArray(16); private val vp = FloatArray(16)
    private val scan = FloatArray(72)
    private var manual: FloatArray? = null        // start x, z, heading, target (cm or deg), kind (0 move, 1 turn)
    private var log: BufferedWriter? = null
    private var lastLogMs = 0L

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        guide = Guidance(this)
        cloud = PointCloudRenderer(map)
        val root = FrameLayout(this).apply { setBackgroundColor(Color.BLACK) }
        gl = GLSurfaceView(this).apply {
            preserveEGLContextOnPause = true
            setEGLContextClientVersion(2)
            setEGLConfigChooser(8, 8, 8, 8, 16, 0)
            setRenderer(this@NavActivity)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
        root.addView(gl, FrameLayout.LayoutParams(-1, -1))
        hud = HudView(this)
        root.addView(hud, FrameLayout.LayoutParams(-1, -1))
        mapView = MapView(this).apply { title = "tap to set goal" }
        val side = (resources.displayMetrics.widthPixels * 0.46f).toInt()
        root.addView(mapView, FrameLayout.LayoutParams(side, side).apply { gravity = Gravity.TOP or Gravity.END; topMargin = dp(8); rightMargin = dp(8) })
        mapView.onTap = { x, z -> setGoal(x, z, "map") }
        root.addView(controls(), FrameLayout.LayoutParams(-1, -2).apply { gravity = Gravity.BOTTOM; bottomMargin = 0 })
        setContentView(root)
        gl.setOnTouchListener { v, e ->
            if (e.action == MotionEvent.ACTION_UP) { taps.add(floatArrayOf(e.x, e.y)); v.performClick() }
            true
        }
        openLog()
    }

    // ------------------------------------------------------------------ UI
    @SuppressLint("SetTextI18n")
    private fun controls(): View {
        fun btn(t: String, color: Int? = null, f: () -> Unit) = Button(this).apply {
            text = t; isAllCaps = false; setOnClickListener { f() }
            color?.let { setBackgroundColor(it); setTextColor(Color.WHITE) }
        }
        fun num(hint: String, v: String, w: Int = 70) = EditText(this).apply {
            this.hint = hint; setText(v); inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_DECIMAL or InputType.TYPE_NUMBER_FLAG_SIGNED
            setTextColor(Color.WHITE); setHintTextColor(Color.GRAY); width = dp(w)
        }
        fun lab(t: String) = TextView(this).apply { text = t; setTextColor(Color.LTGRAY); textSize = 11f }
        fun row(vararg v: View) = HorizontalScrollView(this).apply {
            addView(LinearLayout(this@NavActivity).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL; v.forEach { addView(it) } })
        }
        val prefs = getSharedPreferences("nav", Context.MODE_PRIVATE)
        val url = EditText(this).apply {
            hint = "robot ip[:8766]"; setText(prefs.getString("robot", "")); setTextColor(Color.WHITE); setHintTextColor(Color.GRAY)
            width = dp(170); inputType = InputType.TYPE_TEXT_VARIATION_URI
        }
        val connect = btn("Connect") {
            prefs.edit().putString("robot", url.text.toString()).apply()
            if (link.connected) link.close() else if (url.text.isNotBlank()) link.connect(url.text.toString())
        }
        val estop = btn("STOP", Color.rgb(200, 30, 30)) {
            autoDrive = false; autoBox?.isChecked = false
            link.estop(); guide.say("Stopped", force = true)
        }
        val reset = btn("Reset e-stop") { link.reset() }
        val auto = CheckBox(this).apply {
            text = "Auto-drive"; setTextColor(Color.WHITE)
            setOnCheckedChangeListener { _, c -> autoDrive = c; if (!c) link.stop(); guide.say(if (c) "Auto drive on" else "Auto drive off", force = true) }
        }
        autoBox = auto
        val voice = CheckBox(this).apply { text = "Voice"; isChecked = true; setTextColor(Color.WHITE); setOnCheckedChangeListener { _, c -> guide.voice = c } }
        val beeps = CheckBox(this).apply { text = "Beeps"; isChecked = true; setTextColor(Color.WHITE); setOnCheckedChangeListener { _, c -> guide.beeps = c } }
        val points = CheckBox(this).apply { text = "3D points"; isChecked = true; setTextColor(Color.WHITE); setOnCheckedChangeListener { _, c -> cloud.visible = c } }

        val dist = num("cm", "50"); val spd = num("cm/s", "15"); val ang = num("deg", "90"); val tspd = num("deg/s", "45")
        val move = btn("Move") {
            val d = dist.text.toString().toFloatOrNull() ?: return@btn; val v = spd.text.toString().toFloatOrNull() ?: return@btn
            lastPose?.let { manual = floatArrayOf(it[0], it[1], it[2], d, 0f) }
            if (link.connected) link.move(d, v) else status("robot not connected (measuring phone motion only)")
            guide.say("Moving ${d.toInt()} centimeters", force = true)
        }
        val turn = btn("Turn") {
            val a = ang.text.toString().toFloatOrNull() ?: return@btn; val w = tspd.text.toString().toFloatOrNull() ?: return@btn
            lastPose?.let { manual = floatArrayOf(it[0], it[1], it[2], a, 1f) }
            if (link.connected) link.turn(a, w) else status("robot not connected (measuring phone motion only)")
            guide.say("Turning ${if (a > 0) "left" else "right"} ${abs(a).toInt()} degrees", force = true)
        }
        val mount = num("mount cm", "${(mountHeight * 100).toInt()}"); val rad = num("radius cm", "${(robotRadius * 100).toInt()}")
        val rh = num("height cm", "${(robotHeight * 100).toInt()}"); val vm = num("max cm/s", "${(vMax * 100).toInt()}")
        val apply = btn("Apply") {
            mount.text.toString().toFloatOrNull()?.let { mountHeight = it / 100 }
            rad.text.toString().toFloatOrNull()?.let { robotRadius = it / 100 }
            rh.text.toString().toFloatOrNull()?.let { robotHeight = it / 100; map.maxObstacleHeight = robotHeight + 0.1f }
            vm.text.toString().toFloatOrNull()?.let { vMax = it / 100 }
            path = null; lastPlanMs = 0
            status("params: mount ${(mountHeight * 100).toInt()} cm, radius ${(robotRadius * 100).toInt()} cm, height ${(robotHeight * 100).toInt()} cm, vmax ${(vMax * 100).toInt()} cm/s")
        }
        map.maxObstacleHeight = robotHeight + 0.1f

        statusView = TextView(this).apply { setTextColor(Color.rgb(0, 255, 120)); textSize = 11f; typeface = android.graphics.Typeface.MONOSPACE }
        val panel = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL; setBackgroundColor(Color.argb(170, 0, 0, 0)); setPadding(dp(6), dp(4), dp(6), dp(4))
            addView(row(url, connect, estop, reset))
            addView(row(btn("Load map") { loadMap() }, btn("Align") { align() }, btn("Same start") { mergeSameStart() },
                btn("Save map") { saveMap() }, btn("Clear goal") { goal = null; path = null; mapView.goal = null; mapView.path = null; link.stop() },
                btn("New map") { resetMap() }))
            addView(row(auto, voice, beeps, points))
            addView(row(lab("move"), dist, lab("at"), spd, move, lab("  turn"), ang, lab("at"), tspd, turn))
            addView(row(lab("mount"), mount, lab("radius"), rad, lab("height"), rh, lab("vmax"), vm, apply))
            addView(statusView)
        }
        return ScrollView(this).apply { addView(panel); isFillViewport = false; layoutParams = ViewGroup.LayoutParams(-1, -2) }
    }
    private var autoBox: CheckBox? = null

    private fun status(s: String) = runOnUiThread { statusView.text = s }

    private fun setGoal(x: Float, z: Float, src: String) {
        goal = floatArrayOf(x, z); path = null; lastPlanMs = 0; arrivedSaid = false
        mapView.goal = goal
        status("goal (%.2f, %.2f) from %s".format(x, z, src))
        guide.say("New goal set", force = true)
    }

    // ------------------------------------------------------------------ maps
    private fun mapsDir() = File(SessionWriter.sessionsRoot(this).parentFile, "maps")

    private fun loadMap() = exec.execute {
        val dirs = mapsDir().listFiles()?.filter { File(it, "map.json").exists() }?.sortedByDescending { it.lastModified() }
        val d = dirs?.firstOrNull() ?: run { status("no saved maps in ${mapsDir()} (record a mode A session first)"); return@execute }
        try {
            val m = MapBuilder.load(d)
            saved = m
            val b = m.bitmap()
            runOnUiThread { mapView.setSaved(b?.first, b?.second) }
            status("loaded ${d.name}: %.1f m2 known. Press Align (after a short look around) or Same start.".format(m.knownAreaM2()))
        } catch (e: Exception) { status("load failed: $e") }
    }

    private fun align() = exec.execute {
        val s = saved ?: run { status("load a map first"); return@execute }
        status("aligning live map to saved map...")
        val a = MapBuilder.align(map, s) { status(it) }
        if (a == null) { status("not enough live map yet: look around (walls, furniture) then Align again"); return@execute }
        if (a.score < 0.45f) { status("alignment weak (score %.2f): look around more, or use Same start".format(a.score)); return@execute }
        map.mergeFrom(s, a)
        runOnUiThread { mapView.setSaved(null, null) }
        status("aligned: rot %.0f deg, shift (%.2f, %.2f) m, score %.2f; saved map merged".format(Math.toDegrees(a.theta.toDouble()), a.tx, a.tz, a.score))
        guide.say("Map aligned", force = true)
    }

    private fun mergeSameStart() = exec.execute {
        val s = saved ?: run { status("load a map first"); return@execute }
        map.mergeFrom(s, MapBuilder.Align(0f, 0f, 0f, 1f))
        runOnUiThread { mapView.setSaved(null, null) }
        status("merged saved map assuming this session started at the same spot and heading")
    }

    private fun saveMap() = exec.execute {
        val d = File(mapsDir(), "nav_" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()))
        map.save(d, JSONObject().put("source", "navigation mode"))
        status("saved map to $d")
    }

    private fun resetMap() {
        goal = null; path = null
        gl.queueEvent {
            map = MapBuilder().also { it.maxObstacleHeight = robotHeight + 0.1f }
            cloud = PointCloudRenderer(map).also { it.create() }
        }
        status("new empty map")
    }

    // ------------------------------------------------------------------ lifecycle
    override fun onResume() {
        super.onResume()
        if (session == null) {
            try {
                session = Session(this).also { s ->
                    val cfg = Config(s).apply {
                        updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                        focusMode = Config.FocusMode.AUTO
                        planeFindingMode = Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
                        depthMode = if (s.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) Config.DepthMode.AUTOMATIC else Config.DepthMode.DISABLED
                    }
                    s.configure(cfg)
                    if (cfg.depthMode == Config.DepthMode.DISABLED) status("this phone has no ARCore Depth API: obstacles come from feature points only")
                }
            } catch (e: Exception) { status("ARCore unavailable: $e"); return }
        }
        try { session?.resume() } catch (e: Exception) { status("camera: $e"); return }
        gl.onResume()
    }

    override fun onPause() {
        link.stop()
        autoDrive = false
        gl.onPause()
        session?.pause()
        super.onPause()
    }

    override fun onDestroy() {
        link.close()
        guide.shutdown()
        session?.close(); session = null
        log?.close()
        super.onDestroy()
    }

    private fun openLog() {
        try {
            val d = File(SessionWriter.sessionsRoot(this).parentFile, "nav/" + SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()))
            d.mkdirs()
            log = File(d, "nav.jsonl").bufferedWriter()
        } catch (e: Exception) { Log.w(TAG, "log", e) }
    }

    // ------------------------------------------------------------------ GL
    override fun onSurfaceCreated(g: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0f, 0f, 0f, 1f)
        bg.create(); cloud.create()
        session?.setCameraTextureName(bg.textureId)
    }

    override fun onSurfaceChanged(g: GL10?, width: Int, height: Int) {
        viewW = width; viewH = height
        GLES20.glViewport(0, 0, width, height)
        @Suppress("DEPRECATION")
        session?.setDisplayGeometry(windowManager.defaultDisplay.rotation, width, height)
    }

    override fun onDrawFrame(g: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        val s = session ?: return
        s.setCameraTextureName(bg.textureId)
        val frame = try { s.update() } catch (e: Exception) { return }
        bg.draw(frame)
        try { step(s, frame) } catch (e: Exception) { Log.e(TAG, "nav step", e) }
    }

    private fun step(s: Session, frame: Frame) {
        val cam = frame.camera
        val now = SystemClock.elapsedRealtime()
        val tracking = cam.trackingState == TrackingState.TRACKING
        frames++
        // AR taps -> goal on a detected plane
        while (true) {
            val t = taps.poll() ?: break
            if (!tracking) continue
            val hit = frame.hitTest(t[0], t[1]).firstOrNull { (it.trackable as? Plane)?.let { p -> p.isPoseInPolygon(it.hitPose) && p.type == Plane.Type.HORIZONTAL_UPWARD_FACING } == true }
            if (hit != null) runOnUiThread { setGoal(hit.hitPose.tx(), hit.hitPose.tz(), "camera") }
            else status("tap the floor where it is detected, or tap the map")
        }
        if (!tracking) {
            if (trackingWasOk) {
                trackingWasOk = false
                if (link.connected) link.stop()
                guide.say("Tracking lost, stopping", force = true)
            }
            if (now - lastHudMs > 200) {
                lastHudMs = now
                hud.state = HudView.State().apply {
                    nav = true; banner = "TRACKING ${cam.trackingFailureReason.name}"; bannerColor = Color.rgb(200, 60, 30)
                    lines = listOf("ARCore: ${cam.trackingState.name.lowercase()} (${cam.trackingFailureReason.name.lowercase()})", "move the phone slowly, point it at textured surfaces")
                }
            }
            return
        }
        trackingWasOk = true
        val m = FloatArray(16); cam.pose.toMatrix(m, 0)
        val x = m[12]; val z = m[14]
        val heading = atan2(-m[10], -m[8])
        map.addPose(x, m[13], z, frame.timestamp)
        if (frames % 30 == 0) updateFloor(s, m[13])

        // depth -> map + virtual lidar
        try {
            frame.acquireDepthImage16Bits().use { d ->
                if (d.timestamp != lastDepthTs) {
                    lastDepthTs = d.timestamp
                    val k = cam.imageIntrinsics
                    val sx = d.width.toFloat() / k.imageDimensions[0]; val sy = d.height.toFloat() / k.imageDimensions[1]
                    val p = d.planes[0]
                    map.integrateDepth(p.buffer, d.width, d.height, p.rowStride, k.focalLength[0] * sx, k.focalLength[1] * sy,
                        k.principalPoint[0] * sx, k.principalPoint[1] * sy, m, heading, d.timestamp)
                    depthMaps++
                }
            }
        } catch (_: NotYetAvailableException) {} catch (_: IllegalStateException) {}

        // speed from poses
        val tS = frame.timestamp / 1e9f
        lastPose?.let { lp ->
            val dt = tS - lp[3]
            if (dt in 0.005f..0.5f) speed = 0.8f * speed + 0.2f * (hypot(x - lp[0], z - lp[1]) / dt)
        }
        lastPose = floatArrayOf(x, z, heading, tS)

        map.scan(x, z, heading, frame.timestamp, scan)
        fun minIn(fromDeg: Int, toDeg: Int): Float {
            var r = Float.POSITIVE_INFINITY
            for (deg in fromDeg..toDeg step 5) r = min(r, scan[((deg / 5) % 72 + 72) % 72])
            return r
        }
        val front = minIn(-25, 25); val left = minIn(30, 120); val right = minIn(-120, -30)
        // obstacles closer than the robot body radius are the robot itself / noise: report distance to the body edge
        val frontClear = front - robotRadius * 0.5f
        guide.obstacle(frontClear)

        // planning (background)
        val gl0 = goal
        if (gl0 != null && !planning && now - lastPlanMs > 500) {
            planning = true; lastPlanMs = now
            val grid = synchronized(map) { Planner.Grid(map.occCopy(), map.size, map.res, map.originX, map.originZ) }
            exec.execute {
                try {
                    val p = Planner.plan(grid, x, z, gl0[0], gl0[1], robotRadius)
                    if (goal === gl0) {
                        path = p
                        runOnUiThread { mapView.path = p }
                        if (p == null) { status("no path to goal (blocked or outside map)"); guide.say("No path to goal") }
                    }
                } finally { planning = false }
            }
        }

        // following + robot commands
        var cmd: Planner.Cmd? = null
        val p = path
        if (p != null && p.size >= 2) {
            cmd = Planner.follow(p, x, z, heading, frontClear, vMax, wMax, stopDist = 0.15f, slowDist = 0.6f)
            if (cmd.arrived) {
                if (!arrivedSaid) { arrivedSaid = true; guide.say("Goal reached", force = true); if (link.connected) link.stop() }
            } else {
                arrivedSaid = false
                if (cmd.blocked) guide.say("Obstacle ahead, ${guide.meters(frontClear.coerceAtLeast(0f))}")
                else guide.say(guide.instruction(cmd.bearing, cmd.remaining))
                if (autoDrive && link.connected && now - lastVelMs > 100) {
                    lastVelMs = now
                    link.vel(cmd.v * 100, Math.toDegrees(cmd.w.toDouble()).toFloat())
                }
            }
        }

        // manual command progress measured by the phone
        val man = manual
        val manualText = if (man == null) "" else if (man[4] == 0f) {
            val moved = hypot(x - man[0], z - man[1]) * 100
            "move: phone measured %.1f / %.0f cm".format(moved, abs(man[3]))
        } else {
            var dh = Math.toDegrees((man[2] - heading).toDouble())     // CCW positive
            while (dh > 180) dh -= 360.0; while (dh < -180) dh += 360.0
            "turn: phone measured %.1f / %.0f deg".format(dh, man[3])
        }

        // draw 3D overlay
        cam.getViewMatrix(viewM, 0); cam.getProjectionMatrix(projM, 0, 0.05f, 50f)
        cloud.draw(viewM, projM, p, goal)

        // HUD at ~10 Hz
        if (now - lastHudMs > 100) {
            lastHudMs = now
            Matrix.multiplyMM(vp, 0, projM, 0, viewM, 0)
            val floor = if (map.floorY.isNaN()) m[13] - mountHeight else map.floorY
            val odom = link.odom
            val st = HudView.State().apply {
                nav = true
                speed = this@NavActivity.speed; vMax = this@NavActivity.vMax
                robotSpeed = if (odom != null && now - link.odomRxMs < 1000) odom[3] / 100f else Float.NaN
                this.scan = this@NavActivity.scan.copyOf()
                this.front = frontClear; this.left = left; this.right = right
                if (cmd != null && !cmd.arrived) { bearing = cmd.bearing; instruction = if (cmd.blocked) "Obstacle ${fmtM(frontClear)}" else guide.instruction(cmd.bearing, cmd.remaining) }
                if (cmd?.arrived == true) { banner = "GOAL REACHED"; bannerColor = Color.rgb(30, 160, 60) }
                else if (cmd?.blocked == true) { banner = "BLOCKED  ${fmtM(frontClear)}"; bannerColor = Color.rgb(210, 40, 30) }
                else if (link.estopLatched) { banner = "E-STOP (press Reset e-stop)"; bannerColor = Color.rgb(210, 40, 30) }
                pathPx = p?.let { project(it, floor) }
                goalPx = goal?.let { g -> project(listOf(g), floor + 0.3f) }
                lines = listOfNotNull(
                    "track OK  depth ${depthMaps}  pts ${map.pointCount}  map %.1f m2".format(map.knownAreaM2()),
                    "pos (%.2f, %.2f) m  hdg %.0f  walked %.1f m".format(x, z, Math.toDegrees(-heading.toDouble()), map.travelled),
                    "speed %.0f cm/s  front %s  L %s  R %s".format(speed * 100, fmtM(frontClear), fmtM(left), fmtM(right)),
                    goal?.let { gg -> "goal %.2f m away, path %s".format(hypot(gg[0] - x, gg[1] - z), p?.let { "%.2f m".format(Planner.length(it)) } ?: "planning...") },
                    if (cmd != null && !cmd.arrived) "cmd v %.0f cm/s  w %.0f deg/s  %s".format(cmd.v * 100, Math.toDegrees(cmd.w.toDouble()), if (autoDrive) "AUTO" else "guide only") else null,
                    "robot: ${link.state.name.lowercase()}" + (if (link.connected) "  rtt %.0f ms".format(link.rttMs) + (if (!link.batteryV.isNaN()) "  bat %.1f V".format(link.batteryV) else "") else ""),
                    odom?.let { o -> "odom (%.0f, %.0f) cm  %.0f deg  v %.0f cm/s".format(o[0], o[1], o[2], o[3]) },
                    manualText.ifEmpty { null },
                )
            }
            hud.state = st
        }
        if (now - lastMapUiMs > 600) {
            lastMapUiMs = now
            val t = synchronized(map) { ArrayList(map.traj) }
            val sc = scan.copyOf()
            exec.execute {
                val b = map.bitmap()
                runOnUiThread { mapView.setMap(b?.first, b?.second); mapView.setPose(x, z, heading, t); mapView.setScan(sc) }
            }
        }
        if (now - lastLogMs > 200) {
            lastLogMs = now
            log?.let { w ->
                val o = JSONObject().put("t", tS.toDouble()).put("x", x.toDouble()).put("z", z.toDouble()).put("heading", heading.toDouble())
                    .put("speed", speed.toDouble()).put("front", if (frontClear.isFinite()) frontClear.toDouble() else -1.0)
                    .put("goal", goal?.let { JSONArray(listOf(it[0].toDouble(), it[1].toDouble())) } ?: JSONObject.NULL)
                    .put("cmd", cmd?.let { JSONArray(listOf(it.v.toDouble(), it.w.toDouble())) } ?: JSONObject.NULL)
                    .put("auto", autoDrive).put("robot", link.state.name)
                exec.execute { try { w.write(o.toString()); w.write("\n") } catch (_: Exception) {} }
            }
        }
    }

    private fun fmtM(d: Float) = if (!d.isFinite()) "--" else if (d < 1f) "%.0f cm".format(d.coerceAtLeast(0f) * 100) else "%.2f m".format(d)

    /** World floor points -> screen pixels (NaN pair = gap: point behind the camera). */
    private fun project(pts: List<FloatArray>, y: Float): FloatArray {
        // densify so the line follows the floor in perspective
        val dense = ArrayList<FloatArray>()
        for (k in pts.indices) {
            if (k > 0) {
                val a = pts[k - 1]; val b = pts[k]
                val n = (MapBuilder.dist(a, b) / 0.1f).toInt().coerceIn(1, 100)
                for (i in 1 until n) dense.add(floatArrayOf(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
            }
            dense.add(pts[k])
        }
        val out = FloatArray(dense.size * 2)
        val v = FloatArray(4); val r = FloatArray(4)
        dense.forEachIndexed { i, p ->
            v[0] = p[0]; v[1] = y; v[2] = p[1]; v[3] = 1f
            Matrix.multiplyMV(r, 0, vp, 0, v, 0)
            if (r[3] <= 0.01f) { out[2 * i] = Float.NaN; out[2 * i + 1] = Float.NaN }
            else { out[2 * i] = (r[0] / r[3] * 0.5f + 0.5f) * viewW; out[2 * i + 1] = (1 - (r[1] / r[3] * 0.5f + 0.5f)) * viewH }
        }
        return out
    }

    private fun updateFloor(s: Session, camY: Float) {
        // Floor = lowest large upward plane that is roughly at the mount height below the phone; else mount height.
        var best = Float.NaN
        for (p in s.getAllTrackables(Plane::class.java)) {
            if (p.trackingState != TrackingState.TRACKING || p.type != Plane.Type.HORIZONTAL_UPWARD_FACING || p.subsumedBy != null) continue
            if (p.extentX * p.extentZ < 0.25f) continue
            val y = p.centerPose.ty()
            if (abs((camY - y) - mountHeight) > 0.6f) continue
            if (best.isNaN() || y < best) best = y
        }
        map.floorY = if (!best.isNaN()) best else camY - mountHeight
    }

    companion object { const val TAG = "NavActivity" }
}
