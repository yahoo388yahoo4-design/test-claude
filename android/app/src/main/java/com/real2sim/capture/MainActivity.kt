package com.real2sim.capture

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.camera2.CameraManager
import android.os.Build
import android.os.Bundle
import android.os.PowerManager
import android.text.InputType
import android.view.View
import android.view.ViewGroup
import android.widget.FrameLayout
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.ar.core.ArCoreApk
import java.io.File
import kotlin.concurrent.thread

class MainActivity : AppCompatActivity() {
    private lateinit var statusView: TextView
    private lateinit var statsView: TextView
    private lateinit var modeControl: SegmentedControl
    private lateinit var modeTitle: TextView
    private lateinit var btnRecord: RecordButton
    private lateinit var recTimer: TextView
    private lateinit var previewHost: FrameLayout
    private lateinit var topBar: View
    private lateinit var bottomBar: View
    private lateinit var settings: CaptureSettings
    private var writer: SessionWriter? = null
    private var sensors: SensorRecorder? = null
    private var ar: ArRecorder? = null
    private var cam2: Camera2Recorder? = null
    private var lastSession: File? = null
    private var wake: PowerManager.WakeLock? = null
    private var arInstallRequested = false
    private var mapView: MapView? = null
    private var coverage: CoverageView? = null
    private var recStartMs = 0L
    private lateinit var idle: IdlePreview
    private val ui = android.os.Handler(android.os.Looper.getMainLooper())
    private val bgExec = java.util.concurrent.Executors.newSingleThreadExecutor()
    @Volatile private var liveBusy = false

    override fun onCreate(savedInstanceState: Bundle?) {
        Ui.edgeToEdge(this)
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        settings = CaptureSettings(this)
        statusView = findViewById(R.id.status)
        statsView = findViewById(R.id.stats)
        modeControl = findViewById(R.id.modeControl)
        modeTitle = findViewById(R.id.modeTitle)
        btnRecord = findViewById(R.id.btnRecord)
        recTimer = findViewById(R.id.recTimer)
        previewHost = findViewById(R.id.previewHost)
        topBar = findViewById(R.id.topBar)
        bottomBar = findViewById(R.id.bottomBar)
        Ui.applyInsets(topBar, top = true, bottom = false)
        Ui.applyInsets(bottomBar, top = false, bottom = true)
        idle = IdlePreview(this, previewHost) { status(it) }
        modeControl.setItems(CaptureMode.values().map { it.shortTitle }, settings.mode.ordinal)
        modeControl.onSelect = { i -> settings.mode = CaptureMode.values()[i]; modeChanged() }
        modeChanged()

        btnRecord.setOnClickListener { if (stopping) return@setOnClickListener; if (writer == null) startRecording() else stopRecording() }
        findViewById<View>(R.id.btnSettings).setOnClickListener {
            if (writer != null) { status("stop recording first"); return@setOnClickListener }
            showSettings()
        }
        findViewById<View>(R.id.btnNav).setOnClickListener {
            if (writer != null) { status("stop recording first"); return@setOnClickListener }
            startActivity(android.content.Intent(this, NavActivity::class.java))
        }
        findViewById<View>(R.id.btnSessions).setOnClickListener {
            if (writer != null) { status("stop recording first"); return@setOnClickListener }
            startActivity(android.content.Intent(this, SessionsActivity::class.java))
        }
        requestPerms()
        if (Build.VERSION.SDK_INT >= 30 && !SessionWriter.allFilesAccess()) {
            // Optional: lets sessions go to /sdcard/Real2SimCapture where adb pull always works.
            try {
                startActivity(android.content.Intent(android.provider.Settings.ACTION_MANAGE_APP_ALL_FILES_ACCESS_PERMISSION,
                    android.net.Uri.parse("package:$packageName")))
            } catch (_: Exception) {}
        }
    }

    /** Live camera while idle (like iOS); the recorders get the camera only after it is released. */
    @Volatile private var stopping = false

    private fun startPreview() {
        if (writer != null || stopping || isFinishing) return
        idle.start(settings.mode)
        findViewById<View>(R.id.idleHint).visibility = if (idle.running) View.GONE else View.VISIBLE
    }

    private fun stopPreview() {
        if (::idle.isInitialized) idle.stop()
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (writer == null && !notResumed()) startPreview()
    }

    private fun notResumed() = !lifecycle.currentState.isAtLeast(androidx.lifecycle.Lifecycle.State.RESUMED)

    private fun modeChanged() {
        if (::idle.isInitialized && idle.running && writer == null) startPreview()
        modeTitle.text = settings.mode.title
        findViewById<TextView>(R.id.idleText).text = if (settings.mode == CaptureMode.SENSORS)
            "Sensors only: IMU, GNSS, barometer, … (no camera)" else getString(R.string.idle_hint)
    }

    /** Settings sheet, grouped like the iOS SettingsView form; every change is saved at once. */
    private fun showSettings() {
        val s = settings
        val root = SessionWriter.sessionsRoot(this)
        Sheet(this, "Settings").form {
            section("Mode") {
                inlinePicker(CaptureMode.values().map { it.title }, s.mode.ordinal) { i ->
                    s.mode = CaptureMode.values()[i]; modeControl.select(i); modeChanged()
                }
            }
            section("Capture", "IMU (accelerometer, gyro, magnetometer at the fastest rate), barometer, GNSS, thermal and battery are always recorded.") {
                toggle("Audio track", s.recordAudio) { s.recordAudio = it }
                toggle("Full-res stills", s.hiResStills, "Mode A: shared camera, a JPEG every 0.5 s") { s.hiResStills = it }
            }
            section("Depth and poses (mode A)", "ARCore depth (ToF where the phone has one, depth-from-motion otherwise), raw and smoothed, plus poses, intrinsics and planes are always saved.") {
                toggle("ARCore session recording (.mp4)", s.arcoreMp4, "Replayable ARCore dataset alongside our own files") { s.arcoreMp4 = it }
            }
            section("Camera") {
                toggle("Lock AE / AF / AWB", s.lock, "Modes A and B") { s.lock = it }
                toggle("RAW DNG", s.rawDng, "Mode B: one DNG per second from the main lens") { s.rawDng = it }
                toggle("OIS off", s.oisOff, "Mode B: stabilisation off for stable intrinsics") { s.oisOff = it }
                button("Dump camera inventory") { dumpCameras() }
            }
            section("Location", if (BuildConfig.HAS_ARCORE_API_KEY) "GPS is always recorded." else
                "GPS is always recorded. Geospatial needs an app built with -PARCORE_API_KEY=…") {
                toggle("ARCore Geospatial (Earth pose)", s.geospatial, "Mode A, outdoors with Street View coverage") { s.geospatial = it }
            }
            section("Upload receiver (tools/receiver.py)",
                "Or copy sessions with adb: adb pull ${root.absolutePath.replace("/storage/emulated/0", "/sdcard")}/") {
                field("http://192.168.1.20:8765#token", s.uploadUrl, InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI) { s.uploadUrl = it }
                button("Upload last session") { uploadLast() }
            }
            section(null) {
                value("Sessions", root.name)
                note(root.absolutePath)
            }
        }.show()
    }

    private fun uploadLast() {
        val base = settings.uploadUrl
        val dir = lastSession ?: latestSession()
        if (dir == null || base.isBlank()) { status("nothing to upload / no URL"); return }
        thread(name = "upload") {
            try { Uploader.upload(dir, base) { status(it) } } catch (e: Exception) { status("upload failed: $e") }
        }
    }

    private fun showRoot() {
        val root = SessionWriter.sessionsRoot(this)
        status("Ready · sessions in ${root.absolutePath.replace("/storage/emulated/0", "/sdcard")}")
    }

    override fun onResume() {
        super.onResume()
        if (writer == null) showRoot()
        // Ask Play Services for AR to install/update once; Camera2 mode works without it.
        try {
            if (ArCoreApk.getInstance().requestInstall(this, !arInstallRequested) == ArCoreApk.InstallStatus.INSTALL_REQUESTED) arInstallRequested = true
        } catch (e: Exception) { status("ARCore unavailable: ${e.javaClass.simpleName} (mode B still works)") }
        if (writer == null) startPreview()
    }

    override fun onPause() {
        if (writer != null) stopRecording()
        stopPreview()
        super.onPause()
    }

    private fun requestPerms() {
        val perms = mutableListOf(Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.RECORD_AUDIO)
        val missing = perms.filter { ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) ActivityCompat.requestPermissions(this, missing.toTypedArray(), 1)
    }

    private fun status(s: String) = runOnUiThread { statusView.text = s }

    private fun stats(lines: List<String>) = runOnUiThread {
        statsView.text = lines.joinToString("\n")
        statsView.visibility = if (lines.isEmpty()) View.GONE else View.VISIBLE
    }

    private fun startRecording() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) { requestPerms(); return }
        val s = settings
        val m = s.mode
        stopPreview()      // the recorder needs the camera (ARCore session / CameraDevice) to itself
        val w = SessionWriter(this, m.id)
        writer = w
        sensors = SensorRecorder(this, w).also { it.start(recordAudio = s.recordAudio) }
        try {
            when (m) {
                CaptureMode.RGBD -> {
                    val r = ArRecorder(this, w, ArOptions(hiResStills = s.hiResStills, arcoreMp4 = s.arcoreMp4,
                        lockFocus = s.lock, geospatial = s.geospatial), ::status)
                    r.create()?.let { err -> status(err); abort(); return }
                    previewHost.addView(r.view, ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))
                    r.start()
                    ar = r
                    addLiveViews(r)
                }
                CaptureMode.MULTICAM -> {
                    val r = Camera2Recorder(this, w, Camera2Options(rawDng = s.rawDng, lock = s.lock, oisOff = s.oisOff), ::status)
                    // live preview behind the coverage overlay (its surface must exist before the session is configured)
                    previewHost.addView(r.previewView, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT, android.view.Gravity.CENTER))
                    val err = try { r.start() } catch (e: Exception) { "camera start failed: $e" }
                    if (err != null) { previewHost.removeView(r.previewView); status(err); abort(); return }
                    cam2 = r
                    val cv = CoverageView(this)
                    previewHost.addView(cv, FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT).apply {
                        topMargin = topBar.bottom + dp(8); bottomMargin = bottomBar.height })
                    cv.recording = true; cv.start()
                    coverage = cv
                    ui.postDelayed(liveTick, 500)
                }
                CaptureMode.SENSORS -> {}
            }
        } catch (e: Exception) {
            status("start failed: $e"); abort(); return
        }
        @Suppress("DEPRECATION")
        wake = (getSystemService(Context.POWER_SERVICE) as PowerManager).newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "capture:rec").apply { acquire(3 * 3600 * 1000L) }
        setRecordingUi(true)
        status("Recording · ${m.title}")
    }

    /** Record button morphs to a square, timer capsule on, the other controls off (as on iOS). */
    private fun setRecordingUi(on: Boolean) {
        btnRecord.recording = on
        modeControl.isEnabled = !on
        findViewById<View>(R.id.idleHint).visibility = if (on && settings.mode != CaptureMode.SENSORS) View.GONE else View.VISIBLE
        for (id in intArrayOf(R.id.btnSessions, R.id.btnSettings, R.id.btnNav)) findViewById<View>(id).apply { isEnabled = !on; alpha = if (on) 0.4f else 1f }
        recTimer.visibility = if (on) View.VISIBLE else View.GONE
        ui.removeCallbacks(timerTick)
        if (on) { recStartMs = android.os.SystemClock.elapsedRealtime(); timerTick.run() } else stats(emptyList())
    }

    private val timerTick = object : Runnable {
        override fun run() {
            recTimer.text = "\u25CF  " + Ui.fmtClock(android.os.SystemClock.elapsedRealtime() - recStartMs)
            if (writer != null) ui.postDelayed(this, 500)
        }
    }

    private fun abort() {
        sensors?.stop(); sensors = null
        writer?.finish { put("aborted", true) }; writer = null
        if (!notResumed()) startPreview()
    }

    private fun stopRecording() {
        val w = writer ?: return
        btnRecord.isEnabled = false
        status("stopping…")
        ui.removeCallbacks(liveTick)
        mapView?.let { previewHost.removeView(it) }; mapView = null
        coverage?.let { it.stop(); previewHost.removeView(it) }; coverage = null
        // Stopping writes the final mesh / map and can take seconds on a big room: do it off the UI thread.
        val r = ar; val c = cam2; val sn = sensors
        ar = null; cam2 = null; sensors = null; writer = null; stopping = true
        status("stopping… building the 3D mesh")
        Thread {
            try { r?.stop() } catch (e: Exception) { android.util.Log.e("MainActivity", "ar stop", e) }
            try { c?.stop() } catch (e: Exception) { android.util.Log.e("MainActivity", "cam2 stop", e) }
            val sum = sn?.summary() ?: ""
            try { sn?.stop() } catch (_: Exception) {}
            try { w.finish() } catch (e: Exception) { android.util.Log.e("MainActivity", "finish", e) }
            ui.post {
                stopping = false
                r?.let { previewHost.removeView(it.view) }
                c?.let { previewHost.removeView(it.previewView) }
                lastSession = w.dir
                wake?.let { if (it.isHeld) it.release() }; wake = null
                btnRecord.isEnabled = true
                setRecordingUi(false)
                if (!notResumed()) startPreview()
                status("Saved ${w.dir.name}\n$sum")
            }
        }.apply { name = "stop-recording" }.start()
    }

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    /** Mode A live view: 3D points are drawn by ArRecorder; here the top-down map, and the stats in the status card. */
    private fun addLiveViews(r: ArRecorder) {
        val mv = MapView(this).apply {
            title = "captured so far (long-press: mesh / depth / off)"
            background = Ui.material(this@MainActivity, 12f); clipToOutline = true
        }
        val side = (resources.displayMetrics.widthPixels * 0.42f).toInt()
        previewHost.addView(mv, FrameLayout.LayoutParams(side, side).apply {
            gravity = android.view.Gravity.TOP or android.view.Gravity.END; topMargin = topBar.bottom + dp(8); rightMargin = dp(16) })
        mv.setOnLongClickListener { r.cycleLiveView(); true }
        mapView = mv
        ui.postDelayed(liveTick, 500)
    }

    private val liveTick = object : Runnable {
        override fun run() {
            cam2?.let { c -> coverage?.status = c.liveStatus() }
            val r = ar; val mv = mapView
            if (r != null && mv != null && !liveBusy) {
                liveBusy = true
                bgExec.execute {
                    try {
                        val bm = r.map.bitmap()
                        val t = synchronized(r.map) { ArrayList(r.map.traj) }
                        val lines = r.liveStats()
                        runOnUiThread {
                            if (mapView !== mv) return@runOnUiThread
                            mv.setMap(bm?.first, bm?.second)
                            t.lastOrNull()?.let { p -> mv.setPose(p[0], p[2], r.heading(), t) }
                            stats(lines)
                        }
                    } finally { liveBusy = false }
                }
            }
            if (ar != null || cam2 != null) ui.postDelayed(this, 700)
        }
    }

    private fun latestSession(): File? =
        SessionWriter.sessionsRoot(this).listFiles()?.filter { File(it, "session.json").exists() }?.maxByOrNull { it.name }

    private fun dumpCameras() {
        thread(name = "camdump") {
            try {
                val cm = getSystemService(Context.CAMERA_SERVICE) as CameraManager
                val inv = CameraInfo.inventory(cm)
                val f = File(SessionWriter.sessionsRoot(this).parentFile, "camera_inventory_${Build.MODEL.replace(' ', '_')}.json")
                f.writeText(inv.toString(1))
                val cams = inv.getJSONObject("cameras")
                val lines = cams.keys().asSequence().map { id ->
                    val c = cams.getJSONObject(id)
                    val caps = c.optJSONArray("_capabilities")?.let { a -> (0 until a.length()).map { a.getString(it) } } ?: emptyList()
                    val facing = c.opt("android.lens.facing")
                    val fl = c.optJSONArray("android.lens.info.availableFocalLengths")?.optDouble(0)
                    "id=$id facing=$facing f=${fl}mm phys=${c.optJSONArray("_physical_ids")} depth16=${c.optBoolean("_has_depth16")} " +
                        "raw=${c.optBoolean("_has_raw_sensor")} ${if (c.optBoolean("_not_in_id_list")) "HIDDEN " else ""}" +
                        caps.filter { it in setOf("LOGICAL_MULTI_CAMERA", "DEPTH_OUTPUT", "RAW", "MANUAL_SENSOR") }.joinToString(",")
                }.joinToString("\n")
                status("wrote ${f.name}")
                runOnUiThread { Ui.showText(this, "Cameras", "${f.absolutePath}\nconcurrent=${inv.optJSONArray("concurrent_camera_ids")}\n\n$lines") }
            } catch (e: Exception) { status("camera dump failed: $e") }
        }
    }
}
