package com.real2sim.capture

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.util.concurrent.CopyOnWriteArrayList
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/** Upload state lives in [Uploader] (not the Activity) so it survives recreation and keeps the single-upload guard. */
class UploadStateTest {
    @Test fun singleUploadGuardAndProgressSurviveListenerRebind() {
        val dir = File("/tmp/20260301_120000_arcore_rgbd")
        val block = CountDownLatch(1)
        val started = CountDownLatch(1)
        val finished = CountDownLatch(1)
        val seen = CopyOnWriteArrayList<String>()
        Uploader.lastStatus = ""
        Uploader.listener = { seen += it }
        try {
            val ok = Uploader.start(dir, "http://x") { _, _, progress ->
                progress("upload a (1/2)")
                started.countDown()
                block.await(5, TimeUnit.SECONDS)
                progress("uploaded 2 files")
            }
            assertTrue(ok)
            assertTrue(started.await(5, TimeUnit.SECONDS))
            assertEquals(dir, Uploader.running)
            assertFalse(Uploader.start(dir, "http://x") { _, _, _ -> error("must not run") })   // second tap refused
            assertEquals("20260301_120000_arcore_rgbd: upload a (1/2)", Uploader.lastStatus)   // what a recreated Activity restores
            assertEquals(listOf(Uploader.lastStatus), seen)

            // "Rotation": the old Activity unbinds, the new one binds and keeps receiving progress.
            Uploader.listener = null
            val seen2 = CopyOnWriteArrayList<String>()
            Uploader.listener = { seen2 += it; if (it.endsWith("uploaded 2 files")) finished.countDown() }
            block.countDown()
            assertTrue(finished.await(5, TimeUnit.SECONDS))
            assertEquals(listOf("20260301_120000_arcore_rgbd: uploaded 2 files"), seen2)
            assertEquals(1, seen.size)
            waitUntil { Uploader.running == null }
            assertNull(Uploader.running)
            assertTrue(Uploader.start(dir, "http://x") { _, _, _ -> })                          // guard released
            waitUntil { Uploader.running == null }
        } finally {
            Uploader.listener = null
            block.countDown()
        }
    }

    @Test fun failureIsReportedAndReleasesTheGuard() {
        val dir = File("/tmp/s")
        val done = CountDownLatch(1)
        var msg = ""
        Uploader.listener = { msg = it; done.countDown() }
        try {
            assertTrue(Uploader.start(dir, "http://x") { _, _, _ -> throw java.io.IOException("PUT a -> HTTP 500") })
            assertTrue(done.await(5, TimeUnit.SECONDS))
            assertTrue(msg, msg.startsWith("upload failed: java.io.IOException: PUT a -> HTTP 500"))
            assertEquals(msg, Uploader.lastStatus)
            waitUntil { Uploader.running == null }
        } finally { Uploader.listener = null }
    }

    private fun waitUntil(cond: () -> Boolean) {
        val end = System.nanoTime() + 5_000_000_000L
        while (!cond() && System.nanoTime() < end) Thread.sleep(5)
        assertTrue(cond())
    }
}
