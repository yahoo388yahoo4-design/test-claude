package com.real2sim.capture

import android.content.Context
import android.os.Build
import java.io.File
import java.io.PrintWriter
import java.io.StringWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone

/**
 * Writes uncaught exceptions to crash_logs/crash_<epoch>.txt next to the sessions folder (so they come
 * off the phone over USB with the sessions), then hands the exception to the previous handler so the
 * app still crashes normally. Counterpart of the iOS CrashLog (Documents/crash_logs).
 */
object CrashLog {
    /** What the app is doing, included in the report; set by SessionWriter. */
    @Volatile var currentMode: String = "idle"

    @Volatile private var installed = false

    fun install(context: Context) {
        if (installed) return
        installed = true
        val app = context.applicationContext
        val previous = Thread.getDefaultUncaughtExceptionHandler()
        Thread.setDefaultUncaughtExceptionHandler { thread, e ->
            try {
                val now = System.currentTimeMillis()
                val text = report(e, thread.name, now, "${Build.MANUFACTURER} ${Build.MODEL}",
                    "Android ${Build.VERSION.RELEASE} (API ${Build.VERSION.SDK_INT})",
                    "${BuildConfig.VERSION_NAME} (${BuildConfig.VERSION_CODE})", currentMode)
                val name = "crash_${now / 1000}.txt"
                val written = dirs(app).any { d ->
                    try { (d.isDirectory || d.mkdirs()) && File(d, name).also { it.writeText(text) }.isFile } catch (_: Throwable) { false }
                }
                if (!written) android.util.Log.e("CrashLog", "could not write $name")
            } catch (_: Throwable) {
                // never let the crash logger itself hide the original crash
            }
            previous?.uncaughtException(thread, e)
        }
    }

    /** crash_logs/ beside the sessions root, then app-private storage as a last resort. */
    fun dirs(context: Context): List<File> {
        val out = mutableListOf<File>()
        try { SessionWriter.sessionsRoot(context).parentFile?.let { out += File(it, "crash_logs") } } catch (_: Throwable) {}
        try { context.getExternalFilesDir(null)?.let { out += File(it, "crash_logs") } } catch (_: Throwable) {}
        out += File(context.filesDir, "crash_logs")
        return out.distinct()
    }

    /** The report text. Pure (no Android calls) so it is unit tested. */
    fun report(e: Throwable, thread: String, timeMs: Long, device: String, os: String, appVersion: String, mode: String): String {
        val iso = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US).apply { timeZone = TimeZone.getTimeZone("UTC") }.format(Date(timeMs))
        val sw = StringWriter()
        e.printStackTrace(PrintWriter(sw))
        return buildString {
            append("time: ").append(iso).append(" (unix ").append(timeMs / 1000).append(")\n")
            append("device: ").append(device).append('\n')
            append("os: ").append(os).append('\n')
            append("app_version: ").append(appVersion).append('\n')
            append("mode: ").append(mode).append('\n')
            append("thread: ").append(thread).append('\n')
            append('\n').append(sw.toString())
        }
    }
}
