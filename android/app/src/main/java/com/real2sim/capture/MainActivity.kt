package com.real2sim.capture

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.hardware.camera2.CameraManager
import android.os.Build
import android.os.Bundle
import android.os.PowerManager
import android.view.ViewGroup
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.CheckBox
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.Spinner
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.google.ar.core.ArCoreApk
import java.io.File
import kotlin.concurrent.thread

class MainActivity : AppCompatActivity() {
    private lateinit var statusView: TextView
    private lateinit var modeSpinner: Spinner
    private lateinit var btnRecord: Button
    private lateinit var previewHost: FrameLayout
    private var writer: SessionWriter? = null
    private var sensors: SensorRecorder? = null
    private var ar: ArRecorder? = null
    private var cam2: Camera2Recorder? = null
    private var lastSession: File? = null
    private var wake: PowerManager.WakeLock? = null
    private var arInstallRequested = false

    private val modes = listOf("A: ARCore RGB-D + poses", "B: Camera2 multi-cam + RAW (no poses)", "Sensors only (IMU/GNSS)")

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        statusView = findViewById(R.id.status)
        modeSpinner = findViewById(R.id.modeSpinner)
        btnRecord = findViewById(R.id.btnRecord)
        previewHost = findViewById(R.id.previewHost)
        modeSpinner.adapter = ArrayAdapter(this, android.R.layout.simple_spinner_dropdown_item, modes)
        val prefs = getSharedPreferences("capture", Context.MODE_PRIVATE)
        val url = findViewById<EditText>(R.id.uploadUrl)
        url.setText(prefs.getString("upload_url", ""))

        btnRecord.setOnClickListener { if (writer == null) startRecording() else stopRecording() }
        findViewById<Button>(R.id.btnCams).setOnClickListener { dumpCameras() }
        findViewById<Button>(R.id.btnUpload).setOnClickListener {
            val base = url.text.toString()
            prefs.edit().putString("upload_url", base).apply()
            val dir = lastSession ?: latestSession()
            if (dir == null || base.isBlank()) { status("nothing to upload / no URL"); return@setOnClickListener }
            thread(name = "upload") {
                try { Uploader.upload(dir, base) { status(it) } } catch (e: Exception) { status("upload failed: $e") }
            }
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

    private fun showRoot() {
        val root = SessionWriter.sessionsRoot(this)
        status("sessions: ${root.absolutePath}\nadb pull ${root.absolutePath.replace("/storage/emulated/0", "/sdcard")}/")
    }

    override fun onResume() {
        super.onResume()
        if (writer == null) showRoot()
        // Ask Play Services for AR to install/update once; Camera2 mode works without it.
        try {
            if (ArCoreApk.getInstance().requestInstall(this, !arInstallRequested) == ArCoreApk.InstallStatus.INSTALL_REQUESTED) arInstallRequested = true
        } catch (e: Exception) { status("ARCore unavailable: ${e.javaClass.simpleName} (mode B still works)") }
    }

    override fun onPause() {
        if (writer != null) stopRecording()
        super.onPause()
    }

    private fun requestPerms() {
        val perms = mutableListOf(Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.RECORD_AUDIO)
        val missing = perms.filter { ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) ActivityCompat.requestPermissions(this, missing.toTypedArray(), 1)
    }

    private fun status(s: String) = runOnUiThread { statusView.text = s }

    private fun cb(id: Int) = findViewById<CheckBox>(id).isChecked

    private fun startRecording() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) { requestPerms(); return }
        val modeIdx = modeSpinner.selectedItemPosition
        val mode = when (modeIdx) { 0 -> "arcore_rgbd"; 1 -> "multicam"; else -> "sensors" }
        val w = SessionWriter(this, mode)
        writer = w
        sensors = SensorRecorder(this, w).also { it.start(recordAudio = cb(R.id.cbAudio)) }
        try {
            when (modeIdx) {
                0 -> {
                    val r = ArRecorder(this, w, ArOptions(hiResStills = cb(R.id.cbHiRes), arcoreMp4 = cb(R.id.cbArRec),
                        lockFocus = cb(R.id.cbLock), geospatial = cb(R.id.cbGeo)), ::status)
                    r.create()?.let { err -> status(err); abort(); return }
                    previewHost.addView(r.view, ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT))
                    r.start()
                    ar = r
                }
                1 -> {
                    val r = Camera2Recorder(this, w, Camera2Options(rawDng = cb(R.id.cbRaw), lock = cb(R.id.cbLock), oisOff = cb(R.id.cbOisOff)), ::status)
                    r.start()?.let { err -> status(err); abort(); return }
                    cam2 = r
                }
            }
        } catch (e: Exception) {
            status("start failed: $e"); abort(); return
        }
        @Suppress("DEPRECATION")
        wake = (getSystemService(Context.POWER_SERVICE) as PowerManager).newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "capture:rec").apply { acquire(3 * 3600 * 1000L) }
        btnRecord.text = "Stop"
        modeSpinner.isEnabled = false
    }

    private fun abort() {
        sensors?.stop(); sensors = null
        writer?.finish { put("aborted", true) }; writer = null
    }

    private fun stopRecording() {
        val w = writer ?: return
        btnRecord.isEnabled = false
        status("stopping…")
        ar?.let { it.stop(); previewHost.removeView(it.view) }; ar = null
        cam2?.stop(); cam2 = null
        val sum = sensors?.summary() ?: ""
        sensors?.stop(); sensors = null
        w.finish()
        writer = null
        lastSession = w.dir
        wake?.let { if (it.isHeld) it.release() }; wake = null
        btnRecord.text = "Record"; btnRecord.isEnabled = true; modeSpinner.isEnabled = true
        status("saved ${w.dir.name}\n$sum")
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
                status("${f.name}\nconcurrent=${inv.optJSONArray("concurrent_camera_ids")}\n$lines")
            } catch (e: Exception) { status("camera dump failed: $e") }
        }
    }
}
