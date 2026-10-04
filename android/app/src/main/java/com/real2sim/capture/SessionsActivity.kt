package com.real2sim.capture

import android.annotation.SuppressLint
import android.app.AlertDialog
import android.content.Context
import android.content.Intent
import android.graphics.Color
import android.graphics.Typeface
import android.os.Bundle
import android.text.InputType
import android.view.Gravity
import android.view.ViewGroup
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.edit
import java.io.File
import java.util.concurrent.Executors

/**
 * Recorded sessions, newest first (name, mode, duration, size, frame count), the Android counterpart of
 * the iOS SessionsView. Per session: Open (SessionViewerActivity), Upload (Uploader -> tools/receiver.py,
 * same receiver URL as the main screen) and Delete (after a confirmation).
 */
@SuppressLint("SetTextI18n")
class SessionsActivity : AppCompatActivity() {
    private val exec = Executors.newSingleThreadExecutor()
    private lateinit var list: LinearLayout
    private lateinit var status: TextView
    private lateinit var url: EditText
    @Volatile private var uploading = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setBackgroundColor(Color.BLACK); setPadding(dp(8), dp(8), dp(8), dp(8)) }
        root.addView(TextView(this).apply { text = "Sessions"; setTextColor(Color.WHITE); textSize = 20f })
        root.addView(TextView(this).apply {
            text = SessionWriter.sessionsRoot(this@SessionsActivity).absolutePath
            setTextColor(Color.GRAY); textSize = 11f; typeface = Typeface.MONOSPACE
        })
        url = EditText(this).apply {
            hint = "upload receiver, e.g. http://192.168.1.10:8765#token"
            inputType = InputType.TYPE_TEXT_VARIATION_URI or InputType.TYPE_CLASS_TEXT
            setTextColor(Color.WHITE); setHintTextColor(Color.GRAY); textSize = 13f
            setText(prefs().getString("upload_url", ""))
        }
        root.addView(url)
        status = TextView(this).apply { setTextColor(Color.rgb(0, 230, 118)); textSize = 11f; typeface = Typeface.MONOSPACE }
        root.addView(status)
        list = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        root.addView(ScrollView(this).apply { addView(list) }, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))
        setContentView(root)
    }

    override fun onResume() {
        super.onResume()
        refresh()
    }

    override fun onDestroy() {
        exec.shutdown()
        super.onDestroy()
    }

    private fun prefs() = getSharedPreferences("capture", Context.MODE_PRIVATE)

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    private fun setStatus(s: String) = runOnUiThread { status.text = s }

    private fun refresh() {
        val root = SessionWriter.sessionsRoot(this)
        exec.execute {
            val sessions = try { SessionScan.list(root) } catch (e: Exception) { setStatus("could not list sessions: $e"); emptyList() }
            runOnUiThread { if (!isDestroyed) show(sessions) }
        }
    }

    private fun show(sessions: List<SessionSummary>) {
        list.removeAllViews()
        if (sessions.isEmpty()) {
            list.addView(TextView(this).apply { text = "No sessions yet."; setTextColor(Color.LTGRAY); setPadding(0, dp(16), 0, 0) })
            return
        }
        for (s in sessions) list.addView(row(s))
    }

    private fun row(s: SessionSummary): LinearLayout {
        val box = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.rgb(28, 30, 34))
            setPadding(dp(10), dp(8), dp(10), dp(4))
        }
        box.addView(TextView(this).apply { text = s.name; setTextColor(Color.WHITE); textSize = 14f; typeface = Typeface.MONOSPACE })
        val parts = mutableListOf(s.modeLabel, fmtDuration(s.durationS), fmtBytes(s.bytes))
        s.frames?.let { parts.add("$it frames") }
        if (!s.complete) parts.add("incomplete")
        box.addView(TextView(this).apply { text = parts.joinToString(" · "); setTextColor(Color.LTGRAY); textSize = 12f })
        val buttons = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL; gravity = Gravity.END }
        fun btn(label: String, onClick: () -> Unit) = Button(this).apply { text = label; isAllCaps = false; setOnClickListener { onClick() } }
        buttons.addView(btn("Open") { open(s) })
        buttons.addView(btn("Upload") { upload(s) })
        buttons.addView(btn("Delete") { confirmDelete(s) })
        box.addView(buttons)
        box.setOnClickListener { open(s) }
        return box.also {
            it.layoutParams = LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT).apply { setMargins(0, dp(4), 0, dp(4)) }
        }
    }

    private fun open(s: SessionSummary) {
        startActivity(Intent(this, SessionViewerActivity::class.java).putExtra(SessionViewerActivity.EXTRA_DIR, s.dir.absolutePath))
    }

    private fun upload(s: SessionSummary) {
        val base = url.text.toString().trim()
        if (base.isEmpty()) { setStatus("enter the receiver URL (tools/receiver.py) first"); return }
        if (uploading) { setStatus("an upload is already running"); return }
        prefs().edit { putString("upload_url", base) }
        uploading = true
        Thread({
            try { Uploader.upload(s.dir, base) { setStatus("${s.name}: $it") } } catch (e: Exception) { setStatus("upload failed: $e") }
            finally { uploading = false }
        }, "upload").start()
    }

    private fun confirmDelete(s: SessionSummary) {
        AlertDialog.Builder(this)
            .setTitle("Delete session?")
            .setMessage("${s.name}\n${s.modeLabel} · ${fmtBytes(s.bytes)}\n\nThis removes the folder from the phone and cannot be undone.")
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Delete") { _, _ ->
                exec.execute {
                    val ok = deleteDir(s.dir)
                    setStatus(if (ok) "deleted ${s.name}" else "could not delete everything in ${s.name}")
                    runOnUiThread { refresh() }
                }
            }
            .show()
    }

    private fun deleteDir(d: File): Boolean = d.deleteRecursively()
}
