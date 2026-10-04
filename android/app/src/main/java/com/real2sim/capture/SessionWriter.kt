package com.real2sim.capture

import android.content.Context
import android.os.Build
import android.os.SystemClock
import org.json.JSONArray
import org.json.JSONObject
import java.io.BufferedWriter
import java.io.ByteArrayOutputStream
import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.zip.Deflater
import java.util.zip.DeflaterOutputStream

/**
 * One capture session on disk in the shared r2s-capture v1 raw format (capture/FORMAT.md, the same
 * format the iOS app writes), plus Android-only data under extras/ (see capture/android/README.md).
 *
 * Every `t` is seconds on CLOCK_BOOTTIME (SystemClock.elapsedRealtimeNanos / 1e9), the clock
 * Camera2 sensor timestamps use when SENSOR_INFO_TIMESTAMP_SOURCE == REALTIME and the clock of
 * SensorEvent.timestamp. clock.csv pairs it with unix time once a second.
 */
class SessionWriter(context: Context, val mode: String) {
    val id: String = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date()) + "_" + mode
    val dir: File = File(sessionsRoot(context), id)
    val meta = JSONObject()

    /** Single background thread for all disk writes, so callers never block the camera thread. */
    private val io = Executors.newSingleThreadExecutor { r -> Thread(r, "session-io").apply { priority = Thread.NORM_PRIORITY - 1 } }
    private val frames: BufferedWriter
    private val open = mutableMapOf<String, BufferedWriter>()
    @Volatile var droppedWrites = 0
    @Volatile private var pending = 0

    init {
        File(dir, "extras").mkdirs()
        frames = File(dir, "frames.jsonl").bufferedWriter()
        meta.put("format", "r2s-capture")
        meta.put("version", 1)
        meta.put("platform", "android")
        meta.put("mode", mode)
        meta.put("device", "${Build.MANUFACTURER} ${Build.MODEL}")
        meta.put("os", "Android ${Build.VERSION.RELEASE} (API ${Build.VERSION.SDK_INT})")
        meta.put("app_version", "${BuildConfig.VERSION_NAME} (${BuildConfig.VERSION_CODE})")
        meta.put("android", JSONObject().apply {
            put("manufacturer", Build.MANUFACTURER)
            put("model", Build.MODEL)
            put("device", Build.DEVICE)
            put("product", Build.PRODUCT)
            put("hardware", Build.HARDWARE)
            put("soc_model", if (Build.VERSION.SDK_INT >= 31) Build.SOC_MODEL else "")
            put("sdk_int", Build.VERSION.SDK_INT)
            put("release", Build.VERSION.RELEASE)
            put("fingerprint", Build.FINGERPRINT)
        })
        meta.put("clock", "t = SystemClock.elapsedRealtimeNanos()/1e9 (CLOCK_BOOTTIME), the 'uptime' clock of FORMAT.md")
        val c = clockPair()
        meta.put("start_unix", c.first); meta.put("start_uptime", c.second)
    }

    /** (unix seconds, boottime seconds) read back-to-back. */
    fun clockPair(): Pair<Double, Double> {
        val t0 = SystemClock.elapsedRealtimeNanos()
        val wall = System.currentTimeMillis()
        val t1 = SystemClock.elapsedRealtimeNanos()
        return Pair(wall / 1000.0, (t0 + t1) / 2 / 1e9)
    }

    /** Append-only blob of raw-deflate chunks; returns [offset, length] for frames.jsonl. */
    inner class Blob(rel: String) {
        private val out = FileOutputStream(file(rel))
        private var off = 0L
        /** Compresses on the caller's thread (small maps), writes synchronously; returns the byte range. */
        fun append(raw: ByteArray): JSONArray {
            val d = Deflater(1, true)
            d.setInput(raw); d.finish()
            val bos = java.io.ByteArrayOutputStream(raw.size / 2 + 64)
            val tmp = ByteArray(65536)
            while (!d.finished()) { val n = d.deflate(tmp); bos.write(tmp, 0, n) }
            d.end()
            val b = bos.toByteArray()
            synchronized(this) {
                out.write(b)
                val r = JSONArray(listOf(off, b.size)); off += b.size
                return r
            }
        }
        fun close() = synchronized(this) { out.close() }
    }

    fun file(rel: String): File = File(dir, rel).also { it.parentFile?.mkdirs() }

    /** Queue a write; drops (and counts) work if the IO queue is badly behind, instead of OOMing. */
    fun submit(maxPending: Int = 256, job: () -> Unit) {
        if (pending > maxPending) { droppedWrites++; return }
        synchronized(this) { pending++ }
        io.execute {
            try { job() } catch (e: Exception) { android.util.Log.e("SessionWriter", "write failed", e) }
            finally { synchronized(this) { pending-- } }
        }
    }

    fun frame(obj: JSONObject) = submit(4096) { frames.write(obj.toString()); frames.write("\n") }

    /** Append a line to a CSV (created with header on first use). Thread-safe through the IO thread. */
    fun csv(rel: String, header: String, line: String) = submit(65536) {
        val w = open.getOrPut(rel) { file(rel).bufferedWriter().also { if (header.isNotEmpty()) { it.write(header); it.write("\n") } } }
        w.write(line); w.write("\n")
    }

    fun text(rel: String, s: String) = submit { file(rel).writeText(s) }

    fun bytes(rel: String, b: ByteArray) = submit { file(rel).writeBytes(b) }

    /** raw-deflate a buffer into its own file (point clouds etc.). */
    fun deflateFile(rel: String, data: ByteArray) = submit {
        DeflaterOutputStream(FileOutputStream(file(rel)), Deflater(1, true)).use { it.write(data) }
    }

    fun finish(extra: JSONObject.() -> Unit = {}) {
        val c = clockPair()
        meta.put("end_unix", c.first); meta.put("end_uptime", c.second)
        meta.extra()
        io.execute {
            frames.close()
            open.values.forEach { it.close() }
            meta.put("dropped_writes", droppedWrites)
            File(dir, "session.json").writeText(meta.toString(2))
            File(dir, "DONE").writeText("ok\n")
        }
        io.shutdown()
        io.awaitTermination(60, TimeUnit.SECONDS)
    }

    companion object {
        fun allFilesAccess() = Build.VERSION.SDK_INT >= 30 && android.os.Environment.isExternalStorageManager()

        /** /sdcard/Real2SimCapture/sessions with "All files access", else app-specific external storage. */
        fun sessionsRoot(context: Context): File {
            if (allFilesAccess()) {
                val d = File(android.os.Environment.getExternalStorageDirectory(), "Real2SimCapture/sessions")
                if (d.isDirectory || d.mkdirs()) return d
            }
            return File(context.getExternalFilesDir(null), "sessions")
        }

        fun mat4RowMajorFromColumnMajor(m: FloatArray): JSONArray {
            val a = JSONArray()
            for (r in 0 until 4) for (c in 0 until 4) a.put(m[c * 4 + r].toDouble())
            return a
        }

        /** Copy a plane (with row/pixel stride) into a tightly packed little-endian array. */
        fun packPlane(buf: ByteBuffer, width: Int, height: Int, rowStride: Int, bytesPerPixel: Int, pixelStride: Int = bytesPerPixel): ByteArray {
            val out = ByteArray(width * height * bytesPerPixel)
            val b = buf.duplicate().order(ByteOrder.LITTLE_ENDIAN)
            var o = 0
            for (y in 0 until height) {
                val rowStart = y * rowStride
                if (pixelStride == bytesPerPixel) {
                    b.position(rowStart)
                    b.get(out, o, width * bytesPerPixel)
                    o += width * bytesPerPixel
                } else {
                    for (x in 0 until width) {
                        for (k in 0 until bytesPerPixel) out[o++] = b.get(rowStart + x * pixelStride + k)
                    }
                }
            }
            return out
        }
    }
}
