package com.real2sim.capture

import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder
import java.util.concurrent.atomic.AtomicReference

/**
 * Same protocol as the iOS app (ios/R2SCapture/Uploader.swift) and tools/receiver.py:
 *   PUT <base>/upload/<session>/<relative path>   (header X-R2S-Token if the URL has a #token)
 * Files already complete on the receiver (HEAD 200 with the same Content-Length) are skipped, so an
 * interrupted upload can simply be restarted. A final ".complete" marker closes the session.
 */
object Uploader {
    private val runningRef = AtomicReference<File?>(null)

    /** Session folder being uploaded right now, or null. Lives here so it survives SessionsActivity recreation. */
    val running: File? get() = runningRef.get()

    /** Last progress line, so a recreated SessionsActivity can show the state of an upload already in flight. */
    @Volatile var lastStatus: String = ""

    /** Progress sink bound by the foreground SessionsActivity (cleared in onPause); called on the upload thread. */
    @Volatile var listener: ((String) -> Unit)? = null

    /**
     * Starts uploading [dir] on a background thread; false (and nothing started) when another upload is running.
     * [work] is the transfer itself (defaults to [upload]); injectable for tests.
     */
    fun start(dir: File, base: String, work: (File, String, (String) -> Unit) -> Unit = ::upload): Boolean {
        if (!runningRef.compareAndSet(null, dir)) return false
        Thread({
            try { work(dir, base) { report("${dir.name}: $it") } } catch (e: Exception) { report("upload failed: $e") }
            finally { runningRef.set(null) }
        }, "upload").start()
        return true
    }

    private fun report(s: String) { lastStatus = s; listener?.invoke(s) }

    fun upload(dir: File, baseIn: String, progress: (String) -> Unit) {
        var base = baseIn.trim()
        var token: String? = null
        base.indexOf('#').takeIf { it >= 0 }?.let { token = base.substring(it + 1); base = base.substring(0, it) }
        base = base.trimEnd('/')
        val files = dir.walkTopDown().filter { it.isFile }.map { it.relativeTo(dir).invariantSeparatorsPath to it }
            .sortedBy { it.second.length() }.toList()
        var done = 0
        for ((rel, f) in files) {
            val enc = rel.split('/').joinToString("/") { URLEncoder.encode(it, "UTF-8").replace("+", "%20") }
            val url = URL("$base/upload/${dir.name}/$enc")
            val head = (url.openConnection() as HttpURLConnection).apply {
                requestMethod = "HEAD"; connectTimeout = 10_000; readTimeout = 30_000
                token?.let { setRequestProperty("X-R2S-Token", it) }
            }
            val skip = try { head.responseCode == 200 && head.contentLengthLong == f.length() } catch (_: Exception) { false } finally { head.disconnect() }
            if (skip) { done++; progress("skip $rel ($done/${files.size})"); continue }
            progress("upload $rel ${f.length() / 1024} KiB (${done + 1}/${files.size})")
            val c = (url.openConnection() as HttpURLConnection).apply {
                requestMethod = "PUT"; doOutput = true; connectTimeout = 10_000; readTimeout = 600_000
                setRequestProperty("Content-Type", "application/octet-stream")
                setFixedLengthStreamingMode(f.length())
                token?.let { setRequestProperty("X-R2S-Token", it) }
            }
            try {
                c.outputStream.use { out -> f.inputStream().use { it.copyTo(out, 1 shl 20) } }
                if (c.responseCode !in 200..299) throw java.io.IOException("PUT $rel -> HTTP ${c.responseCode}")
            } finally { c.disconnect() }
            done++
        }
        val c = (URL("$base/upload/${dir.name}/.complete").openConnection() as HttpURLConnection).apply {
            requestMethod = "PUT"; doOutput = true; token?.let { setRequestProperty("X-R2S-Token", it) }
        }
        try { c.outputStream.use { it.write("${files.size}\n".toByteArray()) }; c.responseCode } catch (_: Exception) {} finally { c.disconnect() }
        progress("uploaded ${files.size} files")
    }
}
