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
import android.view.WindowManager
import android.widget.FrameLayout
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import com.real2sim.capture.robot.UsbRobot
import com.real2sim.capture.robot.UsbRobotKind
import com.real2sim.capture.voice.NavActions
import com.real2sim.capture.voice.VoiceAssistant
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
 *  - Robot: Wi-Fi WebSocket or BLE UART link, same protocol as the iPhone app (robot/PROTOCOL.md). Guide /
 *    Auto / Manual (joystick) drive modes. Auto streams velocity commands from pure
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
    /** Depth filter + occupancy + live TSDF mesh, off the GL thread. */
    private var fusion = DepthFusion(map)
    private lateinit var cloud: PointCloudRenderer
    private var saved: MapBuilder? = null
    private lateinit var link: RobotLink
    private lateinit var guide: Guidance
    private val exec = Executors.newSingleThreadExecutor()
    private val taps = ConcurrentLinkedQueue<FloatArray>()

    // parameters (cm in the UI, metres here)
    @Volatile private var mountHeight = 0.30f
    @Volatile private var robotRadius = 0.20f
    @Volatile private var robotHeight = 0.50f
    @Volatile private var vMax = 0.25f
    @Volatile private var wMax = Math.toRadians(60.0).toFloat()
    @Volatile private var mountForward = 0f          // camera ahead of the robot's turning centre (m)
    @Volatile private var stopDist = 0.25f           // forward gap from the robot's edge
    @Volatile private var slowDist = 0.60f
    @Volatile private var goalTol = 0.15f
    @Volatile private var closedLoopMoves = true     // move/turn: phone measures with ARCore and streams vel
    @Volatile private var haptics = true
    enum class Drive { GUIDE, AUTO, MANUAL }
    @Volatile private var drive = Drive.GUIDE

    /** What the robot is doing besides following the path. */
    private sealed class Task {
        object Idle : Task()
        /** Auto mode is driving the robot along the path (set by GO or a new goal in Auto; cleared by stop / e-stop / mode change / arrival). */
        object Follow : Task()
        class Joy(val v: Float, val w: Float, val at: Long) : Task()
        class Move(val sx: Float, val sz: Float, val h: Float, val dist: Float, val speed: Float, val began: Long) : Task()
        class Turn(var lastH: Float, var turned: Float, val target: Float, val rate: Float, val began: Long) : Task()
        class Backup(val until: Long) : Task()
    }
    @Volatile private var task: Task = Task.Idle
    private var stopSent = true
    private var lastCmd = floatArrayOf(0f, 0f)
    private var lastWarnMs = 0L
    private var blockedSince = 0L

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
    private var vibrator: android.os.Vibrator? = null
    private var log: BufferedWriter? = null
    private var lastLogMs = 0L

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    override fun onCreate(savedInstanceState: Bundle?) {
        Ui.edgeToEdge(this)
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        guide = Guidance(this)
        link = RobotLink(this) { msg -> status(msg) }
        link.onMessage = { m -> if (m.optString("type") == "estop") { task = Task.Idle; sendStop(); guide.say("Robot emergency stop", force = true); buzz(300) } }
        vibrator = getSystemService(android.os.Vibrator::class.java)
        cloud = PointCloudRenderer(map) { fusion.liveMesh }
        val root = FrameLayout(this).apply { setBackgroundColor(Color.BLACK) }
        gl = GLSurfaceView(this).apply {
            preserveEGLContextOnPause = true
            setEGLContextClientVersion(2)
            setEGLConfigChooser(8, 8, 8, 8, 16, 0)
            setRenderer(this@NavActivity)
            renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        }
        root.addView(gl, FrameLayout.LayoutParams(-1, -1))
        hud = HudView(this).apply { compact = true; onState = { st -> showHud(st) } }
        root.addView(hud, FrameLayout.LayoutParams(-1, -1))
        mapView = MapView(this).apply { title = "tap = goal" }
        mapView.onTap = { x, z -> setGoal(x, z, "map") }
        loadParams()
        voice = VoiceAssistant(this, voiceActions) { status(it) }
        root.addView(overlay(), FrameLayout.LayoutParams(-1, -1))
        setContentView(root)
        gl.setOnTouchListener { v, e ->
            if (e.action == MotionEvent.ACTION_UP) { taps.add(floatArrayOf(e.x, e.y)); v.performClick() }
            true
        }
        openLog()
    }

    // ------------------------------------------------------------------ UI
    // Laid out like the iOS NavModeView: top bar (close · tracking / map / link · settings), instruction
    // banner, gauges, then at the bottom the voice button and one material panel with radar + minimap,
    // distance chips, the Guide / Auto / Manual picker, manual drive controls and the action row
    // (go-to menu · GO · STOP · save map). Everything else is in the settings sheet.

    private val prefs by lazy { getSharedPreferences("nav", Context.MODE_PRIVATE) }
    private lateinit var trackDot: View
    private lateinit var trackText: TextView
    private lateinit var mapText: TextView
    private lateinit var linkDot: View
    private lateinit var linkText: TextView
    private lateinit var arrow: ImageView
    private lateinit var instrText: TextView
    private lateinit var instrSub: TextView
    private lateinit var gauge: SpeedGauge
    private lateinit var statsText: TextView
    private lateinit var radar: RadarView
    private lateinit var distances: DistanceRow
    private lateinit var modeControl: SegmentedControl
    private lateinit var manualBox: View
    private lateinit var actionRow: LinearLayout
    private lateinit var goBtn: View
    private lateinit var manualToggle: ImageView
    private var showManual = true
    private val hideToast = Runnable { statusView.visibility = View.GONE }

    private fun loadParams() {
        fun pf(k: String, d: Float) = prefs.getFloat(k, d)
        mountHeight = pf("mount", mountHeight); mountForward = pf("mountFwd", mountForward); robotRadius = pf("radius", robotRadius)
        robotHeight = pf("height", robotHeight); vMax = pf("vmax", vMax); wMax = pf("wmax", wMax)
        stopDist = pf("stop", stopDist); slowDist = pf("slow", slowDist); goalTol = pf("goalTol", goalTol)
        closedLoopMoves = prefs.getBoolean("closedLoop", true); haptics = prefs.getBoolean("haptics", true)
        map.maxObstacleHeight = robotHeight + 0.1f
    }

    private fun card(v: View, radius: Float = 12f, padDp: Int = 8): View = v.apply {
        background = Ui.material(this@NavActivity, radius)
        setPadding(dp(padDp), dp(padDp), dp(padDp), dp(padDp))
    }

    private fun dot(color: Int) = View(this).apply { background = Ui.oval(color); layoutParams = LinearLayout.LayoutParams(dp(8), dp(8)) }

    @SuppressLint("SetTextI18n")
    private fun overlay(): View {
        val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(10), dp(4), dp(10), dp(6)) }
        Ui.applyInsets(col, top = true, bottom = true)

        // top bar
        trackDot = dot(Ui.ORANGE); linkDot = dot(Ui.GRAY)
        trackText = Ui.label(this, "Starting ARCore…", 11f, Color.WHITE, Ui.SEMIBOLD).apply { maxLines = 1 }
        mapText = Ui.label(this, "", 11f, Ui.SECONDARY, digits = true).apply { maxLines = 1 }
        linkText = Ui.label(this, link.statusLine(), 11f, Color.WHITE, digits = true).apply { maxLines = 1 }
        val info = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            addView(hstack(this@NavActivity, 6, trackDot, trackText))
            addView(mapText.apply { setPadding(0, dp(2), 0, dp(2)) })
            addView(hstack(this@NavActivity, 6, linkDot, linkText))
        }
        val close = Ui.iconButton(this, R.drawable.ic_close, "Close", 36, bg = Color.argb(70, 255, 255, 255)).apply { setOnClickListener { finish() } }
        val gear = Ui.iconButton(this, R.drawable.ic_gear, "Navigation settings", 36, bg = Color.argb(70, 255, 255, 255)).apply { setOnClickListener { showSettings() } }
        val top = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            addView(close)
            addView(info, LinearLayout.LayoutParams(0, -2, 1f).apply { marginStart = dp(8); marginEnd = dp(8) })
            addView(gear)
        }
        col.addView(card(top))

        // instruction banner
        arrow = ImageView(this).apply { setImageResource(R.drawable.ic_mappin); imageTintList = android.content.res.ColorStateList.valueOf(Ui.CYAN) }
        instrText = Ui.label(this, "Tap the map or the floor to set a goal", 17f, Color.WHITE, Ui.SEMIBOLD).apply { maxLines = 2 }
        instrSub = Ui.label(this, "Guide", 12f, Ui.SECONDARY, digits = true).apply { maxLines = 1; setPadding(0, dp(2), 0, 0) }
        val banner = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            addView(arrow, LinearLayout.LayoutParams(dp(40), dp(40)).apply { marginEnd = dp(12) })
            addView(LinearLayout(this@NavActivity).apply { orientation = LinearLayout.VERTICAL; addView(instrText); addView(instrSub) }, LinearLayout.LayoutParams(0, -2, 1f))
        }
        col.addView(card(banner), LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(6) })

        // gauges (left)
        gauge = SpeedGauge(this)
        statsText = Ui.label(this, "", 9f, Color.WHITE, digits = true).apply { setLineSpacing(0f, 1.1f) }
        val gauges = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            addView(card(FrameLayout(this@NavActivity).apply { addView(gauge, FrameLayout.LayoutParams(dp(84), dp(84))) }, 12f, 4),
                LinearLayout.LayoutParams(-2, -2))
            addView(card(statsText, 8f, 5), LinearLayout.LayoutParams(dp(236), -2).apply { topMargin = dp(6) })
        }
        col.addView(gauges, LinearLayout.LayoutParams(-2, -2).apply { topMargin = dp(6) })
        col.addView(View(this), LinearLayout.LayoutParams(-1, 0, 1f))

        // toast (status messages; tap to dismiss)
        statusView = Ui.label(this, "", 12f, Color.WHITE).apply {
            background = Ui.material(this@NavActivity, 14f)
            setPadding(dp(12), dp(6), dp(12), dp(6)); gravity = Gravity.CENTER; maxLines = 3
            visibility = View.GONE
            setOnClickListener { visibility = View.GONE }
        }
        col.addView(statusView, LinearLayout.LayoutParams(-2, -2).apply { gravity = Gravity.CENTER_HORIZONTAL; bottomMargin = dp(6) })

        // voice (iOS VoiceOverlay: small gear above the round mic, right-aligned)
        voice?.let { v ->
            val vcol = LinearLayout(this).apply {
                orientation = LinearLayout.VERTICAL; gravity = Gravity.CENTER_HORIZONTAL
                addView(Ui.iconButton(this@NavActivity, R.drawable.ic_gear, "Voice settings", 30, bg = Color.argb(115, 0, 0, 0)).apply { setOnClickListener { v.showSettings() } })
                addView(v.micButton(), LinearLayout.LayoutParams(dp(58), dp(58)).apply { topMargin = dp(6) })
            }
            col.addView(vcol, LinearLayout.LayoutParams(-2, -2).apply { gravity = Gravity.END; bottomMargin = dp(6) })
        }

        // bottom panel
        val panel = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        radar = RadarView(this).apply { background = Ui.rounded(Color.argb(115, 0, 0, 0), Ui.dp(this@NavActivity, 10f)) }
        mapView.background = Ui.rounded(Color.argb(115, 0, 0, 0), Ui.dp(this, 10f)); mapView.clipToOutline = true
        panel.addView(LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            addView(radar, LinearLayout.LayoutParams(0, dp(150), 1f))
            addView(mapView, LinearLayout.LayoutParams(0, dp(150), 1f).apply { marginStart = dp(8) })
        })
        distances = DistanceRow(this)
        panel.addView(distances, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(8) })
        modeControl = SegmentedControl(this).apply {
            setItems(Drive.values().map { it.name.lowercase().replaceFirstChar { c -> c.uppercase() } }, drive.ordinal)
            onSelect = { i -> setDrive(Drive.values()[i]) }
        }
        panel.addView(modeControl, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(8) })
        manualBox = manualControls()
        panel.addView(manualBox, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(8) })
        actionRow = actionRow()
        panel.addView(actionRow, LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(8) })
        col.addView(card(panel, 16f, 10))
        refreshDriveUi()
        return col
    }

    private fun actionRow(): LinearLayout {
        val goTo = Ui.iconButton(this, R.drawable.ic_mappin, "Go to", 40, bg = Color.TRANSPARENT).apply { setOnClickListener { goToMenu() } }
        goBtn = Ui.filledButton(this, "GO", Ui.GREEN) { go() }.apply { layoutParams = LinearLayout.LayoutParams(dp(56), dp(44)) }
        manualToggle = ImageView(this).apply {
            setImageResource(R.drawable.ic_chevron_down); contentDescription = "Show or hide manual controls"
            setPadding(dp(8), dp(8), dp(8), dp(8))
            setOnClickListener { showManual = !showManual; refreshDriveUi() }
            layoutParams = LinearLayout.LayoutParams(dp(40), dp(40))
        }
        val stop = Ui.filledButton(this, "STOP", Ui.RED) { stopAll("Stopped") }.apply {
            typeface = android.graphics.Typeface.create("sans-serif-black", android.graphics.Typeface.NORMAL)
            layoutParams = LinearLayout.LayoutParams(0, dp(44), 1f)
        }
        val save = Ui.iconButton(this, R.drawable.ic_save, "Save map", 40, bg = Color.TRANSPARENT).apply { setOnClickListener { saveMap() } }
        return hstack(this, 10, goTo, goBtn, manualToggle, stop, save)
    }

    /** iOS ManualControls: joystick on the left; move / turn steppers with speed sliders on the right. */
    @SuppressLint("SetTextI18n")
    private fun manualControls(): View {
        val joy = JoystickView(this)
        joy.onChange = { f, l -> task = if (f == 0f && l == 0f) Task.Idle else Task.Joy(f * vMax, l * wMax, SystemClock.elapsedRealtime()) }
        var moveCm = 50f; var speedCms = 15f; var turnDeg = 90f; var turnDps = 45f
        val moveLabel = Ui.label(this, "", 12f, Color.WHITE, digits = true)
        val turnLabel = Ui.label(this, "", 12f, Color.WHITE, digits = true)
        fun labels() {
            moveLabel.text = "${moveCm.toInt()} cm @ ${speedCms.toInt()} cm/s"
            turnLabel.text = "${turnDeg.toInt()}° @ ${turnDps.toInt()}°/s"
        }
        labels()
        fun stepper(get: () -> Float, set: (Float) -> Unit, lo: Float, hi: Float, step: Float): View {
            val box = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; background = Ui.rounded(Ui.BG_FILL, Ui.dp(this@NavActivity, 7f)) }
            fun b(t: String, d: Float) = Ui.label(this, t, 17f, Color.WHITE).apply {
                gravity = Gravity.CENTER; contentDescription = if (d < 0) "less" else "more"
                setOnClickListener { set((get() + d).coerceIn(lo, hi)); labels() }
            }
            box.addView(b("−", -step), LinearLayout.LayoutParams(dp(34), dp(28)))
            box.addView(b("+", step), LinearLayout.LayoutParams(dp(34), dp(28)))
            return box
        }
        fun slider(get: () -> Float, set: (Float) -> Unit, lo: Float, hi: Float, step: Float) = android.widget.SeekBar(this).apply {
            max = ((hi - lo) / step).toInt(); progress = ((get() - lo) / step).toInt()
            progressTintList = android.content.res.ColorStateList.valueOf(Ui.BLUE)
            thumbTintList = android.content.res.ColorStateList.valueOf(Color.WHITE)
            setOnSeekBarChangeListener(object : android.widget.SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(sb: android.widget.SeekBar, p: Int, fromUser: Boolean) { if (fromUser) { set(lo + p * step); labels() } }
                override fun onStartTrackingTouch(sb: android.widget.SeekBar) {}
                override fun onStopTrackingTouch(sb: android.widget.SeekBar) {}
            })
        }
        fun doMove(sign: Float) {
            val d = sign * moveCm; val v = speedCms
            val lp = lastPose
            lp?.let { manual = floatArrayOf(it[0], it[1], it[2], d, 0f) }
            if (!closedLoopMoves) { if (!link.move(d, v)) status("robot not connected (measuring phone motion only)") }
            else if (lp != null) task = Task.Move(lp[0], lp[1], lp[2], d / 100, v / 100, SystemClock.elapsedRealtime())
            guide.say("Moving ${if (d < 0) "back " else ""}${abs(d).toInt()} centimeters", force = true)
        }
        fun doTurn(sign: Float) {
            val a = sign * turnDeg; val w = turnDps
            val lp = lastPose
            lp?.let { manual = floatArrayOf(it[0], it[1], it[2], a, 1f) }
            if (!closedLoopMoves) { if (!link.turn(a, w)) status("robot not connected (measuring phone motion only)") }
            else if (lp != null) task = Task.Turn(lp[2], 0f, Math.toRadians(a.toDouble()).toFloat(), Math.toRadians(w.toDouble()).toFloat(), SystemClock.elapsedRealtime())
            guide.say("Turning ${if (a > 0) "left" else "right"} ${abs(a).toInt()} degrees", force = true)
        }
        fun pair(l: String, lf: () -> Unit, r: String, rf: () -> Unit) = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            addView(Ui.borderedButton(this@NavActivity, l) { lf() }, LinearLayout.LayoutParams(0, dp(34), 1f))
            addView(Ui.borderedButton(this@NavActivity, r) { rf() }, LinearLayout.LayoutParams(0, dp(34), 1f).apply { marginStart = dp(8) })
        }
        fun header(l: TextView, st: View) = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            addView(l, LinearLayout.LayoutParams(0, -2, 1f)); addView(st)
        }
        val right = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            addView(header(moveLabel, stepper({ moveCm }, { moveCm = it }, 5f, 500f, 5f)))
            addView(slider({ speedCms }, { speedCms = it }, 5f, 100f, 5f), LinearLayout.LayoutParams(-1, dp(24)))
            addView(pair("▲ Fwd", { doMove(1f) }, "▼ Back", { doMove(-1f) }))
            addView(header(turnLabel, stepper({ turnDeg }, { turnDeg = it }, 5f, 360f, 5f)), LinearLayout.LayoutParams(-1, -2).apply { topMargin = dp(6) })
            addView(slider({ turnDps }, { turnDps = it }, 10f, 180f, 5f), LinearLayout.LayoutParams(-1, dp(24)))
            addView(pair("⟲ Left", { doTurn(1f) }, "Right ⟳", { doTurn(-1f) }))
        }
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            addView(joy, LinearLayout.LayoutParams(dp(110), dp(110)))
            addView(right, LinearLayout.LayoutParams(0, -2, 1f).apply { marginStart = dp(10) })
        }
    }

    private fun setDrive(d: Drive) {
        drive = d
        task = Task.Idle; sendStop()
        guide.say("${drive.name.lowercase()} mode", force = true)
        refreshDriveUi()
    }

    private fun refreshDriveUi() {
        if (modeControl.selected != drive.ordinal) modeControl.select(drive.ordinal)
        manualBox.visibility = if (drive == Drive.MANUAL && showManual) View.VISIBLE else View.GONE
        goBtn.visibility = if (drive == Drive.AUTO) View.VISIBLE else View.GONE
        manualToggle.visibility = if (drive == Drive.MANUAL) View.VISIBLE else View.GONE
        manualToggle.setImageResource(if (showManual) R.drawable.ic_chevron_down else R.drawable.ic_gamepad)
    }

    /** iOS GO: (re)start following the current goal (the planned path is kept). */
    private fun go() {
        val g = goal ?: run { status("Pick a goal first: tap the map or the floor"); guide.say("Pick a goal first", force = true); return }
        task = Task.Follow; blockedSince = 0L; lastPlanMs = 0; arrivedSaid = false
        status("GO: following the path to (%.2f, %.2f)".format(g[0], g[1]))
        guide.say("Go", force = true)
    }

    /** iOS "Go to" menu (no RoomPlan objects on Android: a few handy goals instead). */
    private fun goToMenu() {
        val items = arrayOf("1 m ahead", "Back to the start point", "Clear goal")
        Ui.alert(this).setTitle("Go to").setItems(items) { _, i ->
            when (i) {
                0 -> voiceActions.goToRelative(1.0, 0.0, false)
                1 -> setGoal(0f, 0f, "start point")
                else -> clearGoal()
            }
        }.show()
    }

    private fun clearGoal() { goal = null; path = null; mapView.goal = null; mapView.path = null; if (task === Task.Follow) task = Task.Idle; sendStop() }

    /** Feeds the native HUD cards from the state NavActivity computes for HudView (UI thread). */
    @SuppressLint("SetTextI18n")
    private fun showHud(st: HudView.State) {
        val lost = st.banner.startsWith("TRACKING")
        trackDot.background = Ui.oval(if (lost) Ui.ORANGE else Ui.GREEN)
        trackText.text = if (lost) st.lines.firstOrNull() ?: st.banner else "Tracking OK"
        mapText.text = if (lost) st.lines.getOrNull(1) ?: "" else st.lines.firstOrNull()?.removePrefix("track OK")?.trim() ?: ""
        val connected = link.connected
        val stTxt = link.statusLine()
        linkDot.background = Ui.oval(when {
            connected -> Ui.GREEN
            stTxt.contains("connecting", true) -> Ui.YELLOW
            stTxt.contains("fail", true) || stTxt.contains("error", true) -> Ui.RED
            else -> Ui.GRAY
        })
        linkText.text = stTxt
        val g = goal; val p = lastPose
        if (!st.bearing.isNaN() && g != null) {
            arrow.setImageResource(R.drawable.ic_nav)
            arrow.rotation = -Math.toDegrees(st.bearing.toDouble()).toFloat()
        } else { arrow.setImageResource(R.drawable.ic_mappin); arrow.rotation = 0f }
        arrow.imageTintList = android.content.res.ColorStateList.valueOf(if (st.banner.startsWith("BLOCKED")) Ui.RED else Ui.CYAN)
        instrText.text = when {
            st.instruction.isNotEmpty() -> st.instruction
            st.banner.isNotEmpty() -> st.banner.lowercase().replaceFirstChar { it.uppercase() }
            g == null -> "Tap the map or the floor to set a goal"
            else -> "Planning…"
        }
        val sub = StringBuilder(drive.name.lowercase().replaceFirstChar { it.uppercase() })
        if (g != null && p != null) sub.append("   goal ").append(fmtDist(hypot(g[0] - p[0], g[1] - p[1])))
        if (!st.bearing.isNaN()) sub.append("   %+.0f°".format(Math.toDegrees(st.bearing.toDouble())))
        instrSub.text = sub
        gauge.set(st.speed, lastCmd[0], vMax, st.robotSpeed)
        statsText.text = st.lines.drop(1).joinToString("\n")
        radar.robotRadius = robotRadius; radar.stop = stopDist; radar.slow = slowDist
        radar.scan = st.scan
        val nearest = st.scan?.filter { it.isFinite() }?.minOrNull()?.minus(robotRadius) ?: Float.POSITIVE_INFINITY
        distances.set(st.front, st.left - robotRadius, st.right - robotRadius, nearest, stopDist, slowDist)
    }

    /** Settings sheet, sectioned like the iOS NavSettingsView. Parameters are applied when it closes. */
    @SuppressLint("SetTextI18n")
    private fun showSettings() {
        val kinds = listOf("None (guide only)", "Wi-Fi (WebSocket)", "Bluetooth LE (UART)", "USB: Neato", "USB: OpenBot")
        var kind = prefs.getInt("linkKind", 1)
        var url = prefs.getString("wsUrl", "ws://192.168.4.1:8777/robot") ?: ""
        var ble = prefs.getString("bleName", "R2S-Robot") ?: ""
        val vals = linkedMapOf(
            "mount" to "${(mountHeight * 100).toInt()}", "fwd" to "${(mountForward * 100).toInt()}",
            "radius" to "${(robotRadius * 100).toInt()}", "height" to "${(robotHeight * 100).toInt()}",
            "vmax" to "${(vMax * 100).toInt()}", "wmax" to "${Math.toDegrees(wMax.toDouble()).toInt()}",
            "slow" to "${(slowDist * 100).toInt()}", "stop" to "${(stopDist * 100).toInt()}", "goal" to "${(goalTol * 100).toInt()}")
        val start = HashMap(vals)
        val sheet = Sheet(this, "Navigation")
        sheet.form {
            section("Map (from mode A or a saved nav map)",
                "Load brings back the newest map in ${mapsDir().name}/. Align matches it to the live map after a short look around; Same start assumes you started where it started.") {
                button("Load map") { loadMap() }
                button("Align to live map") { align() }
                button("Merge (same start)") { mergeSameStart() }
                button("Save map") { saveMap() }
                button("Clear goal") { clearGoal() }
                button("New map", Ui.RED) { resetMap() }
            }
            section("Robot link", "Closed-loop: the phone measures the motion and streams velocities. Off: \"move\" and \"turn\" go to the robot as is (it needs encoders).") {
                choice("Link", kinds, kind) { kind = it }
                field("ws://192.168.4.1:8777/robot", url, InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI) { url = it }
                field("BLE name prefix", ble) { ble = it }
                button("Connect") {
                    prefs.edit().putInt("linkKind", kind).putString("wsUrl", url).putString("bleName", ble).apply()
                    connect(kind, url, ble)
                }
                button("Disconnect") { link.disconnect() }
                button("E-stop latch", Ui.RED) { stopAll("Emergency stop"); link.estop(true) }
                button("Release E-stop") { link.estop(false) }
                toggle("Moves / turns closed-loop with ARCore", closedLoopMoves) { c -> closedLoopMoves = c; prefs.edit().putBoolean("closedLoop", c).apply() }
            }
            section("Robot and mount") {
                number("Camera height (until floor found)", vals["mount"]!!, "cm") { vals["mount"] = it }
                number("Camera ahead of turning centre", vals["fwd"]!!, "cm") { vals["fwd"] = it }
                number("Robot radius", vals["radius"]!!, "cm") { vals["radius"] = it }
                number("Obstacle height up to", vals["height"]!!, "cm") { vals["height"] = it }
            }
            section("Motion and safety") {
                number("Max speed", vals["vmax"]!!, "cm/s") { vals["vmax"] = it }
                number("Max turn rate", vals["wmax"]!!, "°/s") { vals["wmax"] = it }
                number("Slow down below", vals["slow"]!!, "cm") { vals["slow"] = it }
                number("Stop below", vals["stop"]!!, "cm") { vals["stop"] = it }
                number("Goal tolerance", vals["goal"]!!, "cm") { vals["goal"] = it }
            }
            section("Guidance and display") {
                toggle("Voice instructions", guide.voice) { guide.voice = it }
                toggle("Proximity beeps", guide.beeps) { guide.beeps = it }
                toggle("Haptics", haptics) { c -> haptics = c; prefs.edit().putBoolean("haptics", c).apply() }
                toggle("Show 3D points", cloud.visible) { cloud.visible = it }
                button("Voice assistant settings…") { voice?.showSettings() }
            }
        }
        sheet.onDismiss = { if (vals != start) applyParams(vals) }
        sheet.show()
    }

    private fun connect(kind: Int, url: String, ble: String) {
        if (kind >= 3) {   // Neato / OpenBot on the phone's USB-OTG port (robot/UsbRobot.kt)
            val rk = if (kind == 3) UsbRobotKind.NEATO else UsbRobotKind.OPENBOT
            link.connectCustom("USB ${rk.title}") { incoming ->
                // onDead: the serial port died (cable pulled): drop the link so the HUD / auto mode stop treating it as connected
                UsbRobot(this, rk, incoming, { status(it) }, onDead = { runOnUiThread { if (link.kind == RobotLink.Kind.CUSTOM) link.disconnect() } }).also { it.start() }
            }
            return
        }
        val k = RobotLink.Kind.values()[kind]
        if (k == RobotLink.Kind.BLE && !hasBlePermission()) { requestBlePermission(); return }
        link.connect(k, url, ble)
    }

    private fun applyParams(f: Map<String, String>) {
        fun v(k: String) = f[k]?.toFloatOrNull()
        v("mount")?.let { mountHeight = it / 100 }; v("fwd")?.let { mountForward = it / 100 }
        v("radius")?.let { robotRadius = it / 100 }; v("height")?.let { robotHeight = it / 100; map.maxObstacleHeight = robotHeight + 0.1f }
        v("vmax")?.let { vMax = it / 100 }; v("wmax")?.let { wMax = Math.toRadians(it.toDouble()).toFloat() }
        v("stop")?.let { stopDist = it / 100 }; v("slow")?.let { slowDist = maxOf(it / 100, stopDist + 0.05f) }; v("goal")?.let { goalTol = it / 100 }
        prefs.edit().putFloat("mount", mountHeight).putFloat("mountFwd", mountForward).putFloat("radius", robotRadius).putFloat("height", robotHeight)
            .putFloat("vmax", vMax).putFloat("wmax", wMax).putFloat("stop", stopDist).putFloat("slow", slowDist).putFloat("goalTol", goalTol).apply()
        path = null; lastPlanMs = 0
        status("params: mount ${(mountHeight * 100).toInt()} cm (fwd ${(mountForward * 100).toInt()}), radius ${(robotRadius * 100).toInt()} cm, height ${(robotHeight * 100).toInt()} cm, " +
            "vmax ${(vMax * 100).toInt()} cm/s, slow/stop ${(slowDist * 100).toInt()}/${(stopDist * 100).toInt()} cm")
    }

    private var voice: VoiceAssistant? = null

    /** Voice control (voice/VoiceAssistant.kt) drives the same commands as the buttons. */
    private val voiceActions = object : NavActions {
        override fun robotConnected() = link.connected
        override fun stopAll() = stopAll("Stopped")
        override fun move(meters: Double, speedCms: Double) {
            val v = minOf(speedCms.toFloat(), vMax * 100); val d = (meters * 100).toFloat()
            val lp = lastPose
            lp?.let { manual = floatArrayOf(it[0], it[1], it[2], d, 0f) }
            if (!closedLoopMoves) link.move(d, v)
            else if (lp != null) task = Task.Move(lp[0], lp[1], lp[2], d / 100, v / 100, SystemClock.elapsedRealtime())
        }
        override fun turn(degrees: Double, speedDps: Double) {
            val a = degrees.toFloat(); val w = minOf(speedDps.toFloat(), Math.toDegrees(wMax.toDouble()).toFloat())
            val lp = lastPose
            lp?.let { manual = floatArrayOf(it[0], it[1], it[2], a, 1f) }
            if (!closedLoopMoves) link.turn(a, w)
            else if (lp != null) task = Task.Turn(lp[2], 0f, Math.toRadians(a.toDouble()).toFloat(), Math.toRadians(w.toDouble()).toFloat(), SystemClock.elapsedRealtime())
        }
        override fun pose() = lastPose?.let { doubleArrayOf(it[0].toDouble(), it[1].toDouble(), it[2].toDouble()) }
        override fun goTo(x: Double, z: Double, label: String, drive: Boolean) {
            // switch mode first: setDrive resets the task, and setGoal in Auto starts following
            runOnUiThread {
                if (drive && this@NavActivity.drive != Drive.AUTO) setDrive(Drive.AUTO)
                setGoal(x.toFloat(), z.toFloat(), "voice: $label")
            }
        }
        override fun goToRelative(forward: Double, left: Double, drive: Boolean) {
            val p = lastPose ?: return
            val h = p[2].toDouble()   // heading grows clockwise: left of forward (cos h, sin h) is (sin h, -cos h)
            goTo(p[0] + forward * kotlin.math.cos(h) + left * kotlin.math.sin(h), p[1] + forward * kotlin.math.sin(h) - left * kotlin.math.cos(h), "point", drive)
        }
        override fun setMode(mode: String) {
            val d = Drive.values().firstOrNull { it.name.equals(mode, true) } ?: return
            runOnUiThread { if (drive != d) setDrive(d) }
        }
        override fun mode() = drive.name.lowercase()
        override fun clearGoal() = this@NavActivity.clearGoal()
        override fun busy() = task is Task.Move || task is Task.Turn || task === Task.Follow || link.motionPending
        override fun statusJSON(): JSONObject {
            val o = JSONObject().put("mode", mode()).put("robot_connected", link.connected).put("robot", link.robotName)
            lastPose?.let { o.put("pose", JSONObject().put("x", it[0].toDouble()).put("z", it[1].toDouble()).put("heading_deg", Math.toDegrees(it[2].toDouble()))) }
            goal?.let { g -> lastPose?.let { p -> o.put("goal_distance_m", kotlin.math.hypot((g[0] - p[0]).toDouble(), (g[1] - p[1]).toDouble())) } }
            if (!link.batteryV.isNaN()) o.put("battery_v", link.batteryV.toDouble())
            o.put("busy", busy())
            return o
        }
        override fun spokenStatus(): String {
            val parts = ArrayList<String>()
            parts += if (lastPose != null) "I'm tracking" else "I'm not tracking yet"
            parts += "${mode()} mode"
            val g = goal; val p = lastPose
            if (g != null && p != null) parts += "goal %.1f meters away".format(kotlin.math.hypot(g[0] - p[0], g[1] - p[1]))
            parts += if (link.connected) "robot connected" else "robot not connected"
            if (!link.batteryV.isNaN()) parts += "battery %.1f volts".format(link.batteryV)
            return parts.joinToString(", ") + "."
        }
    }

    private fun stopAll(say: String) {
        task = Task.Idle
        if (drive == Drive.AUTO) { goal = null; path = null; mapView.goal = null; mapView.path = null }
        link.stop(); stopSent = true
        guide.say(say, force = true); buzz(120)
    }

    private fun sendStop() { link.stop(); stopSent = true }

    private fun buzz(ms: Long) {
        if (!haptics) return
        try { vibrator?.vibrate(android.os.VibrationEffect.createOneShot(ms, android.os.VibrationEffect.DEFAULT_AMPLITUDE)) } catch (_: Exception) {}
    }

    private fun blePerms() = if (android.os.Build.VERSION.SDK_INT >= 31)
        arrayOf(android.Manifest.permission.BLUETOOTH_SCAN, android.Manifest.permission.BLUETOOTH_CONNECT)
    else arrayOf(android.Manifest.permission.ACCESS_FINE_LOCATION)

    private fun hasBlePermission() = blePerms().all { checkSelfPermission(it) == android.content.pm.PackageManager.PERMISSION_GRANTED }

    private fun requestBlePermission() {
        status("allow Bluetooth access, then press Connect again")
        requestPermissions(blePerms(), 7)
    }

    private fun status(s: String) = runOnUiThread {
        if (!::statusView.isInitialized) return@runOnUiThread
        statusView.text = s; statusView.visibility = View.VISIBLE
        statusView.removeCallbacks(hideToast); statusView.postDelayed(hideToast, 8000)
    }

    private fun setGoal(x: Float, z: Float, src: String) {
        goal = floatArrayOf(x, z); path = null; lastPlanMs = 0; arrivedSaid = false
        if (drive == Drive.AUTO) { task = Task.Follow; blockedSince = 0L }
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
        clearGoal()
        gl.queueEvent {
            map = MapBuilder().also { it.maxObstacleHeight = robotHeight + 0.1f }
            fusion.close()
            fusion = DepthFusion(map)
            cloud = PointCloudRenderer(map) { fusion.liveMesh }.also { it.create() }
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
        task = Task.Idle
        gl.onPause()
        session?.pause()
        super.onPause()
    }

    override fun onDestroy() {
        voice?.shutdown()
        link.disconnect()
        guide.shutdown()
        session?.close(); session = null
        fusion.close()
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
                if (task !== Task.Follow) task = Task.Idle    // following resumes when tracking returns; nothing is streamed meanwhile
                sendStop(); buzz(300)
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

        // depth -> DepthFusion worker: filter -> map + virtual lidar (+ live mesh). Dense smoothed depth keeps
        // textureless walls as obstacles.
        try {
            frame.acquireDepthImage16Bits().use { d ->
                if (d.timestamp != lastDepthTs) {
                    lastDepthTs = d.timestamp
                    val k = cam.textureIntrinsics     // ARCore depth covers the camera texture's FOV; DepthFusion scales to the depth size
                    val p = d.planes[0]
                    val mm = DepthFilter.shortsLE(SessionWriter.packPlane(p.buffer, d.width, d.height, p.rowStride, 2, p.pixelStride.coerceAtLeast(2)), d.width * d.height)
                    fusion.submit(DepthFusion.Frame(null, null, 0, 0, mm, d.width, d.height,
                        floatArrayOf(k.focalLength[0], k.focalLength[1], k.principalPoint[0], k.principalPoint[1]),
                        k.imageDimensions[0], k.imageDimensions[1], m, d.timestamp))
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

        // robot turning centre (the phone may sit ahead of or behind it)
        val rx = x - mountForward * kotlin.math.cos(heading); val rz = z - mountForward * kotlin.math.sin(heading)
        map.scan(x, z, heading, frame.timestamp, scan)
        fun minIn(fromDeg: Int, toDeg: Int): Float {
            var r = Float.POSITIVE_INFINITY
            for (deg in fromDeg..toDeg step 5) r = min(r, scan[((deg / 5) % 72 + 72) % 72])
            return r
        }
        val front = minIn(-25, 25); val left = minIn(30, 120); val right = minIn(-120, -30); val rear = minIn(155, 205)
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
                    val p = Planner.plan(grid, rx, rz, gl0[0], gl0[1], robotRadius)
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
        var out = floatArrayOf(0f, 0f)               // v m/s, w rad/s sent to the robot
        var state = ""
        val p = path
        if (p != null && p.size >= 2) {
            cmd = Planner.follow(p, rx, rz, heading, frontClear, vMax, wMax, stopDist = stopDist, slowDist = slowDist, goalTol = goalTol)
            if (cmd.arrived) {
                if (!arrivedSaid) { arrivedSaid = true; guide.say("Goal reached", force = true); buzz(200) }
                if (task === Task.Follow) task = Task.Idle
                blockedSince = 0L
            } else {
                arrivedSaid = false
                if (cmd.blocked) guide.say("Obstacle ahead, ${guide.meters(frontClear.coerceAtLeast(0f))}")
                else guide.say(guide.instruction(cmd.bearing, cmd.remaining))
                if (drive == Drive.AUTO && task === Task.Follow) {   // only after GO / a new goal in Auto; e-stop, STOP and mode changes end it
                    out = floatArrayOf(cmd.v, cmd.w)
                    // blocked for 3 s: back up a little if the way behind is clear, then replan
                    if (cmd.blocked) {
                        if (blockedSince == 0L) blockedSince = now
                        else if (now - blockedSince > 3000 && rear > 0.35f) { task = Task.Backup(now + 1200); blockedSince = 0L; lastPlanMs = 0; guide.say("Backing up", force = true) }
                        state = "Blocked"
                    } else blockedSince = 0L
                }
            }
        }
        fun safety(v: Float, w: Float): FloatArray {
            if (v <= 0f) return floatArrayOf(v, w)
            val k = if (frontClear <= stopDist) 0f else if (frontClear >= slowDist) 1f else (frontClear - stopDist) / (slowDist - stopDist)
            return floatArrayOf(v * k.coerceIn(0f, 1f), w)
        }
        fun wrap(a: Float): Float { var r = a; while (r > Math.PI) r -= (2 * Math.PI).toFloat(); while (r < -Math.PI) r += (2 * Math.PI).toFloat(); return r }
        when (val t = task) {
            is Task.Idle, is Task.Follow -> {}
            is Task.Joy -> if (now - t.at > 500 && t.v == 0f && t.w == 0f) task = Task.Idle else out = safety(t.v, t.w)
            is Task.Backup -> if (now < t.until && rear > 0.1f) out = floatArrayOf(-0.08f, 0f)
                else task = if (goal != null && drive == Drive.AUTO) Task.Follow else Task.Idle
            is Task.Move -> {
                // progress from the phone pose: the same point the task's start (sx, sz) was taken from
                val travelled = NavMath.travelledAlong(x, z, t.sx, t.sz, t.h)
                val remaining = NavMath.moveRemaining(t.dist, x, z, t.sx, t.sz, t.h)
                if (remaining <= 0.01f || now - t.began > (abs(t.dist) / maxOf(0.02f, t.speed) * 3 + 3) * 1000) {
                    task = Task.Idle; guide.say("Done", force = true)
                } else {
                    val v = (if (t.dist < 0) -1f else 1f) * minOf(t.speed, maxOf(0.05f, 1.5f * remaining))
                    val w = (2f * wrap(heading - t.h)).coerceIn(-0.5f, 0.5f)     // hold the start heading (heading grows clockwise)
                    out = safety(v, w)
                    if (t.dist > 0 && out[0] == 0f) state = "Blocked"
                }
            }
            is Task.Turn -> {
                t.turned += wrap(t.lastH - heading); t.lastH = heading                  // CCW positive
                val remaining = abs(t.target) - abs(t.turned)
                if (remaining <= Math.toRadians(1.5).toFloat() || now - t.began > (abs(t.target) / maxOf(0.1f, t.rate) * 3 + 3) * 1000) {
                    task = Task.Idle; guide.say("Done", force = true)
                } else out = floatArrayOf(0f, (if (t.target < 0) -1f else 1f) * minOf(t.rate, maxOf(Math.toRadians(12.0).toFloat(), 2f * remaining)))
            }
        }
        if (drive == Drive.GUIDE && task !is Task.Move && task !is Task.Turn) out = floatArrayOf(0f, 0f)
        lastCmd = out
        if (link.connected) {
            if (out[0] != 0f || out[1] != 0f) {
                if (now - lastVelMs >= 100) { lastVelMs = now; link.vel(out[0], out[1]); stopSent = false }
            } else if (!stopSent) sendStop()
        }
        if (frontClear < stopDist + 0.05f && (out[0] > 0f || speed > 0.05f) && now - lastWarnMs > 3000) { lastWarnMs = now; buzz(150) }

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
            val st = HudView.State().apply {
                nav = true
                speed = this@NavActivity.speed; vMax = this@NavActivity.vMax
                robotSpeed = if (now - link.statusRxMs < 1000) link.robotV else Float.NaN
                this.scan = this@NavActivity.scan.copyOf()
                this.front = frontClear; this.left = left; this.right = right
                if (cmd != null && !cmd.arrived) { bearing = cmd.bearing; instruction = if (cmd.blocked) "Obstacle ${fmtM(frontClear)}" else guide.instruction(cmd.bearing, cmd.remaining) }
                if (cmd?.arrived == true) { banner = "GOAL REACHED"; bannerColor = Color.rgb(30, 160, 60) }
                else if (cmd?.blocked == true) { banner = "BLOCKED  ${fmtM(frontClear)}"; bannerColor = Color.rgb(210, 40, 30) }
                else if (link.robotEstop) { banner = "E-STOP (press Release)"; bannerColor = Color.rgb(210, 40, 30) }
                else if (state.isNotEmpty()) { banner = state.uppercase(); bannerColor = Color.rgb(210, 40, 30) }
                pathPx = p?.let { project(it, floor) }
                goalPx = goal?.let { g -> project(listOf(g), floor + 0.3f) }
                lines = listOfNotNull(
                    "track OK  depth ${depthMaps}  pts ${map.pointCount}  map %.1f m2".format(map.knownAreaM2()),
                    "pos (%.2f, %.2f) m  hdg %.0f  walked %.1f m".format(x, z, Math.toDegrees(-heading.toDouble()), map.travelled),
                    "speed %.0f cm/s  front %s  L %s  R %s".format(speed * 100, fmtM(frontClear), fmtM(left), fmtM(right)),
                    goal?.let { gg -> "goal %.2f m away, path %s".format(hypot(gg[0] - x, gg[1] - z), p?.let { "%.2f m".format(Planner.length(it)) } ?: "planning...") },
                    "${drive.name}  sent v %.0f cm/s  w %.0f deg/s  rear %s".format(lastCmd[0] * 100, Math.toDegrees(lastCmd[1].toDouble()), fmtM(rear)),
                    link.statusLine(),
                    if (link.robotV.isFinite() && now - link.statusRxMs < 1000) "robot v %.0f cm/s  w %.0f deg/s".format(link.robotV * 100, Math.toDegrees(link.robotW.toDouble())) else null,
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
                    .put("sent", JSONArray(listOf(lastCmd[0].toDouble(), lastCmd[1].toDouble())))
                    .put("drive", drive.name).put("robot", link.stateText)
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
