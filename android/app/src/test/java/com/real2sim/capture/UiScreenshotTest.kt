package com.real2sim.capture

import android.app.Activity
import android.app.Dialog
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.view.View
import android.widget.TextView
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.Robolectric
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import org.robolectric.annotation.GraphicsMode
import org.robolectric.shadows.ShadowDialog
import org.robolectric.shadows.ShadowLooper
import java.io.File

/**
 * Renders the redesigned screens off-device (Robolectric native graphics) to app/build/screenshots (PNG),
 * so the layout can be checked without a phone. The camera / GL views are black here; everything drawn
 * on top of them (cards, buttons, sheets) is real. Also checks the main controls exist and work.
 */
@RunWith(RobolectricTestRunner::class)
@GraphicsMode(GraphicsMode.Mode.NATIVE)
@Config(sdk = [35], qualifiers = "w392dp-h850dp-port-xxhdpi")
class UiScreenshotTest {
    private val out = File("build/screenshots").apply { mkdirs() }

    @org.junit.Before
    fun storage() {
        // SessionWriter.allFilesAccess() asks Environment about the primary external volume
        org.robolectric.shadows.ShadowEnvironment.setExternalStorageDirectory(java.nio.file.Files.createTempDirectory("ext"))
        org.robolectric.shadows.ShadowEnvironment.addExternalDir("ext0")
    }

    private fun settle() = ShadowLooper.idleMainLooper(600, java.util.concurrent.TimeUnit.MILLISECONDS)

    private fun render(a: Activity, name: String, dialog: Dialog? = null) {
        settle()
        val root = a.window.decorView
        val w = root.width.takeIf { it > 0 } ?: 1078
        val h = root.height.takeIf { it > 0 } ?: 2338
        val bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
        val c = Canvas(bmp)
        c.drawColor(Color.BLACK)
        root.draw(c)
        dialog?.window?.decorView?.let { d ->
            c.drawColor(Color.argb(115, 0, 0, 0))
            if (d.width == 0) {
                d.measure(View.MeasureSpec.makeMeasureSpec(w, View.MeasureSpec.EXACTLY), View.MeasureSpec.makeMeasureSpec(h, View.MeasureSpec.EXACTLY))
                d.layout(0, 0, w, h)
            }
            d.draw(c)
        }
        File(out, "$name.png").outputStream().use { bmp.compress(Bitmap.CompressFormat.PNG, 100, it) }
    }

    private fun fakeSession(root: File, name: String, mode: String, secs: Double) {
        val d = File(root, name).apply { mkdirs() }
        File(d, "session.json").writeText("""{"mode":"$mode","start_time":1000.0,"end_time":${1000.0 + secs},"frames":${(secs * 30).toInt()}}""")
        File(d, "video.mp4").writeBytes(ByteArray(300_000))
    }

    @Test
    fun mainScreen() {
        val c = Robolectric.buildActivity(MainActivity::class.java).setup()
        val a = c.get()
        render(a, "1_main")
        assertNotNull(a.findViewById<SegmentedControl>(R.id.modeControl))
        // the look while recording (no camera here: just the controls' state)
        a.findViewById<RecordButton>(R.id.btnRecord).recording = true
        a.findViewById<TextView>(R.id.recTimer).apply { visibility = View.VISIBLE; text = "●  00:42" }
        a.findViewById<TextView>(R.id.stats).apply { visibility = View.VISIBLE; text = "frames 1260  depth 410  tracked 1251\nmap 18.4 m2  points 48211  walked 12.3 m\nfloor -1.32 m" }
        ShadowLooper.idleMainLooper(400, java.util.concurrent.TimeUnit.MILLISECONDS)
        render(a, "2_main_recording")
        a.findViewById<RecordButton>(R.id.btnRecord).recording = false
        a.findViewById<View>(R.id.btnSettings).performClick()
        val sheet = ShadowDialog.getLatestDialog()
        assertNotNull(sheet)
        render(a, "3_main_settings", sheet)
        sheet.dismiss()
    }

    @Test
    fun sessionsScreen() {
        val app = org.robolectric.RuntimeEnvironment.getApplication()
        val root = SessionWriter.sessionsRoot(app)
        fakeSession(root, "20261005_101500_arcore_rgbd", "arcore_rgbd", 83.0)
        fakeSession(root, "20261005_102210_multicam", "multicam", 41.5)
        fakeSession(root, "20261004_181003_sensors", "sensors", 600.0)
        val a = Robolectric.buildActivity(SessionsActivity::class.java).setup().get()
        // the list is filled from a background thread
        repeat(20) { Thread.sleep(50); settle() }
        render(a, "4_sessions")
        val v = Robolectric.buildActivity(SessionViewerActivity::class.java,
            android.content.Intent(app, SessionViewerActivity::class.java).putExtra(SessionViewerActivity.EXTRA_DIR, File(root, "20261004_181003_sensors").absolutePath)).setup().get()
        repeat(20) { Thread.sleep(50); settle() }
        render(v, "5_viewer")
        assertTrue(File(out, "4_sessions.png").length() > 0)
    }

    @Test
    fun navScreen() {
        // not resumed: onResume opens an ARCore session (native code, not available on the JVM)
        val a = Robolectric.buildActivity(NavActivity::class.java).create().start().postCreate(null).visible().get()
        // a fake HUD state so the cards show something
        val hud = (a.window.decorView as android.view.ViewGroup).findHud()
        hud?.state = HudView.State().apply {
            nav = true; speed = 0.18f; vMax = 0.25f
            scan = FloatArray(72) { k -> if (k in 60..71 || k in 0..10) 0.9f + k % 5 * 0.05f else if (k in 15..25) 1.6f else 2.6f }
            front = 0.85f; left = 1.6f; right = 0.9f; bearing = 0.4f; instruction = "Turn left 25°, then 1.5 m"
            lines = listOf("track OK  depth 210  pts 15400  map 9.8 m2", "pos (0.42, -1.10) m  hdg 23  walked 3.4 m",
                "speed 18 cm/s  front 85 cm  L 1.60 m  R 90 cm", "GUIDE  sent v 0 cm/s  w 0 deg/s  rear --", "robot: off")
        }
        render(a, "6_nav")
        // open the settings sheet through the gear button
        val gear = (a.window.decorView as android.view.ViewGroup).findByDesc("Navigation settings")
        assertNotNull(gear)
        gear!!.performClick()
        render(a, "7_nav_settings", ShadowDialog.getLatestDialog())
    }

    private fun android.view.ViewGroup.findHud(): HudView? {
        for (i in 0 until childCount) {
            val c = getChildAt(i)
            if (c is HudView) return c
            if (c is android.view.ViewGroup) c.findHud()?.let { return it }
        }
        return null
    }

    private fun android.view.ViewGroup.findByDesc(d: String): View? {
        for (i in 0 until childCount) {
            val c = getChildAt(i)
            if (c.contentDescription == d) return c
            if (c is android.view.ViewGroup) c.findByDesc(d)?.let { return it }
        }
        return null
    }
}
