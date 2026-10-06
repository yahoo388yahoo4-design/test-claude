package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.graphics.Color
import android.os.Bundle
import android.text.InputType
import android.view.Gravity
import android.view.ViewGroup
import android.view.View
import android.widget.FrameLayout
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.edit
import java.io.File
import java.util.concurrent.Executors

/**
 * Recorded sessions, newest first, laid out like the iOS SessionsView: large title + Done, an
 * inset-grouped list (name, then mode · duration · size · frames), swipe left for Upload / Delete,
 * long-press for the same actions as a menu, tap to open the viewer (SessionViewerActivity). Upload
 * goes to the receiver URL from the main Settings sheet (tools/receiver.py).
 */
@SuppressLint("SetTextI18n")
class SessionsActivity : AppCompatActivity() {
    private val exec = Executors.newSingleThreadExecutor()
    private lateinit var list: LinearLayout
    private lateinit var status: TextView
    private lateinit var receiver: TextView

    override fun onCreate(savedInstanceState: Bundle?) {
        Ui.edgeToEdge(this)
        super.onCreate(savedInstanceState)
        val root = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setBackgroundColor(Color.BLACK) }
        root.addView(Ui.navBar(this, "", null, Ui.barButton(this, "Done", bold = true) { finish() }))
        val col = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL; setPadding(dp(16), 0, dp(16), dp(24)) }
        col.addView(Ui.label(this, "Sessions", 34f, Color.WHITE, Ui.BOLD).apply { setPadding(dp(4), dp(2), 0, dp(10)) })
        receiver = Ui.label(this, "", 13f, Ui.BLUE).apply {
            setPadding(dp(4), 0, dp(4), dp(6))
            setOnClickListener { editReceiver() }
        }
        col.addView(receiver)
        status = Ui.label(this, "", 12f, Ui.SECONDARY, digits = true).apply { setPadding(dp(4), 0, dp(4), dp(6)); visibility = View.GONE }
        col.addView(status)
        list = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            background = Ui.rounded(Ui.BG_SHEET, Ui.dp(this@SessionsActivity, 10f))
            clipToOutline = true
        }
        col.addView(list, LinearLayout.LayoutParams(-1, -2))
        col.addView(Ui.label(this, "Swipe a session left to upload or delete it, or long-press it.\n" +
            SessionWriter.sessionsRoot(this).absolutePath, 12f, Ui.SECONDARY).apply { setPadding(dp(16), dp(8), dp(16), 0) })
        root.addView(ScrollView(this).apply { addView(col) }, LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, 0, 1f))
        Ui.applyInsets(root, top = true, bottom = true)
        setContentView(root)
    }

    override fun onResume() {
        super.onResume()
        showReceiver()
        // Upload state lives in Uploader so it survives recreation (rotation, Done + reopen): rebind and restore.
        Uploader.listener = { msg -> runOnUiThread { if (!isDestroyed) setStatus(msg) } }
        Uploader.lastStatus.takeIf { it.isNotEmpty() }?.let { setStatus(it) }
        refresh()
    }

    override fun onPause() {
        Uploader.listener = null
        super.onPause()
    }

    override fun onDestroy() {
        exec.shutdown()
        super.onDestroy()
    }

    private fun prefs() = getSharedPreferences("capture", Context.MODE_PRIVATE)

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    private fun setStatus(s: String) = runOnUiThread { status.text = s; status.visibility = if (s.isEmpty()) View.GONE else View.VISIBLE }

    private fun showReceiver() {
        val u = prefs().getString("upload_url", "").orEmpty()
        receiver.text = if (u.isBlank()) "Set upload receiver…" else "Receiver: ${u.substringBefore('#')}"
    }

    private fun editReceiver() {
        val e = EditText(this).apply {
            hint = "http://192.168.1.20:8765#token"
            inputType = InputType.TYPE_TEXT_VARIATION_URI or InputType.TYPE_CLASS_TEXT
            setSingleLine(); setText(prefs().getString("upload_url", ""))
        }
        val box = FrameLayout(this).apply { setPadding(dp(20), dp(8), dp(20), 0); addView(e) }
        Ui.alert(this).setTitle("Upload receiver").setMessage("tools/receiver.py on your computer").setView(box)
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Save") { _, _ -> prefs().edit { putString("upload_url", e.text.toString().trim()) }; showReceiver() }
            .show()
    }

    private fun refresh() {
        if (isDestroyed || exec.isShutdown) return
        val root = SessionWriter.sessionsRoot(this)
        exec.execute {
            val sessions = try { SessionScan.list(root) } catch (e: Exception) { setStatus("could not list sessions: $e"); emptyList() }
            runOnUiThread { if (!isDestroyed) show(sessions) }
        }
    }

    private fun show(sessions: List<SessionSummary>) {
        list.removeAllViews()
        if (sessions.isEmpty()) {
            list.addView(Ui.label(this, "No sessions yet. Record one on the main screen.", 15f, Ui.SECONDARY).apply {
                gravity = Gravity.CENTER; setPadding(dp(16), dp(28), dp(16), dp(28))
            })
            return
        }
        sessions.forEachIndexed { i, s ->
            if (i > 0) list.addView(View(this).apply { setBackgroundColor(Ui.SEPARATOR) },
                LinearLayout.LayoutParams(-1, maxOf(1, dp(1) / 2)).apply { marginStart = dp(60) })
            list.addView(row(s))
        }
    }

    private fun modeBadge(mode: String): Pair<String, Int> = when (mode) {
        "arcore_rgbd" -> "A" to Ui.BLUE
        "multicam" -> "B" to Ui.PURPLE
        "sensors" -> "C" to Ui.GRAY
        else -> "?" to Ui.GRAY
    }

    private fun row(s: SessionSummary): View {
        val content = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            setBackgroundColor(Ui.BG_SHEET)
            setPadding(dp(16), dp(10), dp(10), dp(10))
            minimumHeight = dp(60)
            isClickable = true; isLongClickable = true
            background = Ui.pressable(android.graphics.drawable.ColorDrawable(Ui.BG_SHEET), android.graphics.drawable.ColorDrawable(Color.WHITE))
        }
        val (letter, color) = modeBadge(s.mode)
        content.addView(Ui.label(this, letter, 15f, Color.WHITE, Ui.BOLD).apply {
            gravity = Gravity.CENTER; background = Ui.rounded(color, Ui.dp(this@SessionsActivity, 7f))
        }, LinearLayout.LayoutParams(dp(30), dp(30)).apply { marginEnd = dp(14) })
        val text = LinearLayout(this).apply { orientation = LinearLayout.VERTICAL }
        text.addView(Ui.label(this, s.name, 15f, Color.WHITE, mono = true).apply { maxLines = 1; ellipsize = android.text.TextUtils.TruncateAt.MIDDLE })
        val parts = mutableListOf(modeLabel(s.mode), fmtDuration(s.durationS), fmtBytes(s.bytes))
        s.frames?.let { parts.add("$it frames") }
        text.addView(Ui.label(this, parts.joinToString(" · "), 13f, Ui.SECONDARY, digits = true).apply { setPadding(0, dp(3), 0, 0) })
        if (!s.complete) text.addView(Ui.label(this, "incomplete (no session.json)", 12f, Ui.ORANGE).apply { setPadding(0, dp(2), 0, 0) })
        content.addView(text, LinearLayout.LayoutParams(0, -2, 1f))
        content.addView(Ui.icon(this, R.drawable.ic_chevron_right, Ui.TERTIARY, 16))
        content.setOnClickListener { open(s) }
        content.setOnLongClickListener { menu(s); true }
        return SwipeRow(this, content, listOf(
            Triple("Upload", Ui.BLUE) { upload(s) },
            Triple("Delete", Ui.RED) { confirmDelete(s) },
        ))
    }

    /** Long-press menu (iOS context menu). */
    private fun menu(s: SessionSummary) {
        Ui.alert(this).setTitle(s.name).setItems(arrayOf("Open", "Upload to receiver", "Delete")) { _, i ->
            when (i) { 0 -> open(s); 1 -> upload(s); else -> confirmDelete(s) }
        }.show()
    }

    private fun open(s: SessionSummary) {
        startActivity(Intent(this, SessionViewerActivity::class.java).putExtra(SessionViewerActivity.EXTRA_DIR, s.dir.absolutePath))
    }

    private fun upload(s: SessionSummary) {
        val base = prefs().getString("upload_url", "").orEmpty().trim()
        if (base.isEmpty()) { setStatus("set the receiver URL (tools/receiver.py) first"); editReceiver(); return }
        if (!Uploader.start(s.dir, base)) setStatus("an upload is already running")
    }

    private fun confirmDelete(s: SessionSummary) {
        Ui.alert(this)
            .setTitle("Delete session?")
            .setMessage("${s.name}\n${s.modeLabel} · ${fmtBytes(s.bytes)}\n\nThis removes the folder from the phone and cannot be undone.")
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Delete") { _, _ ->
                exec.execute {
                    val ok = deleteDir(s.dir)
                    setStatus(if (ok) "deleted ${s.name}" else "could not delete everything in ${s.name}")
                    runOnUiThread { if (!isDestroyed) refresh() }
                }
            }
            .show()
    }

    private fun deleteDir(d: File): Boolean = d.deleteRecursively()
}
