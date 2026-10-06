package com.real2sim.capture

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import java.nio.ByteBuffer

/** Pure maths factored out of ArRecorder: depth intrinsics and the depth -> CPU-image pixel map. */
class ArRecorderMathTest {
    /** Principal point at the pixel-centre convention (ARCore: (w-1)/2 for a centred lens). */
    private fun centredK(f: Float, w: Int, h: Int) = floatArrayOf(f, f, (w - 1) / 2f, (h - 1) / 2f)

    @Test fun scaleKeepsTheFieldOfView() {
        val k = DepthIntrinsics.scale(floatArrayOf(1500f, 1500f, 959.5f, 539.5f), 1920, 1080, 160, 90)
        assertArrayEquals(floatArrayOf(125f, 125f, 959.5f / 12, 539.5f / 12), k, 1e-4f)
        // the ray through the image corner is the same before and after scaling
        assertEquals((0 - 959.5f) / 1500f, (0 - k[2]) / k[0], 1e-6f)
        assertEquals((1079 - 539.5f) / 1500f, (89.9166f - k[3]) / k[1], 1e-3f)
    }

    @Test fun affineRoundTripsRays() {
        val dK = floatArrayOf(125f, 125f, 79.958f, 44.958f)
        val imgK = floatArrayOf(500f, 500f, 319.5f, 239.5f)
        val a = DepthIntrinsics.affine(dK, imgK)
        for ((u, v) in listOf(0f to 0f, 159f to 89f, 80f to 45f)) {
            val ui = a[0] * u + a[1]; val vi = a[2] * v + a[3]
            assertEquals((u - dK[2]) / dK[0], (ui - imgK[2]) / imgK[0], 1e-4f)
            assertEquals((v - dK[3]) / dK[1], (vi - imgK[3]) / imgK[1], 1e-4f)
        }
    }

    /** Texture and CPU image with the same aspect: the mapping is the plain resample YuvSampler did before. */
    @Test fun sameAspectMatchesPlainResample() {
        val iw = 640; val ih = 480; val tw = 1920; val th = 1440; val dw = 160; val dh = 120
        val imgK = centredK(500f, iw, ih)
        val texK = centredK(1500f, tw, th)
        val dK = DepthIntrinsics.scale(texK, tw, th, dw, dh)
        val a = DepthIntrinsics.affine(dK, imgK)
        // a pure scale by iw/dw, no crop: every depth pixel lands where the plain resample (YuvSampler.sample:
        // sx = ((i + 0.5) * srcW / outW).toInt()) put it, up to the half depth pixel by which plain K scaling
        // (cx * dw / w, as ARCore's sample, DepthFusion and FORMAT.md do) shifts the pixel-centre grid
        assertEquals(iw.toFloat() / dw, a[0], 1e-4f); assertEquals(ih.toFloat() / dh, a[2], 1e-4f)
        val half = iw / dw / 2
        for (j in 0 until dh) for (i in 0 until dw) {
            val p = DepthIntrinsics.sourcePixel(a, i, j, iw, ih)
            assertEquals("u of ($i,$j)", ((i + 0.5f) * iw / dw).toInt().toFloat(), p[0].toFloat(), half.toFloat())
            assertEquals("v of ($i,$j)", ((j + 0.5f) * ih / dh).toInt().toFloat(), p[1].toFloat(), half.toFloat())
        }
        assertEquals(0, DepthIntrinsics.sourcePixel(a, 0, 0, iw, ih)[0]); assertEquals(0, DepthIntrinsics.sourcePixel(a, 0, 0, iw, ih)[1])
        assertEquals(iw - 4, DepthIntrinsics.sourcePixel(a, dw - 1, dh - 1, iw, ih)[0])
        assertEquals(ih - 4, DepthIntrinsics.sourcePixel(a, dw - 1, dh - 1, iw, ih)[1])
        // the YUV sampler itself: with the map of the plain resample it agrees pixel for pixel with YuvSampler
        val plain = floatArrayOf(iw.toFloat() / dw, iw / dw / 2f - 0.5f, ih.toFloat() / dh, ih / dh / 2f - 0.5f)
        val y = ByteBuffer.wrap(ByteArray(iw * ih) { ((it % iw) * 255 / iw).toByte() })
        val uv = ByteBuffer.wrap(ByteArray((iw / 2) * (ih / 2) * 2) { 128.toByte() })
        val old = YuvSampler.sample(y, iw, 1, uv, uv, iw, 2, iw, ih, dw, dh)
        assertArrayEquals(old, DepthIntrinsics.sampleYuv(y, iw, 1, uv, uv, iw, 2, iw, ih, dw, dh, plain))
        // and through the intrinsics map the gradient differs by at most one level (the half pixel above)
        val new = DepthIntrinsics.sampleYuv(y, iw, 1, uv, uv, iw, 2, iw, ih, dw, dh, a)
        for (i in old.indices) assertEquals("pixel $i", (old[i] and 255).toFloat(), (new[i] and 255).toFloat(), 2f)
    }

    /** 16:9 texture next to a 4:3 CPU image (same horizontal FOV): the depth map is a vertical crop of the image. */
    @Test fun wideTextureCropsTheImageVertically() {
        val iw = 640; val ih = 480; val tw = 1920; val th = 1080; val dw = 160; val dh = 90
        val imgK = centredK(500f, iw, ih)
        val texK = centredK(1500f, tw, th)        // same horizontal FOV as the image, 3/4 of its vertical FOV
        val dK = DepthIntrinsics.scale(texK, tw, th, dw, dh)
        val a = DepthIntrinsics.affine(dK, imgK)
        // columns: full width, 4 image pixels per depth pixel
        assertEquals(4f, a[0], 1e-4f); assertEquals(4f, a[2], 1e-4f)
        assertEquals(0, DepthIntrinsics.sourcePixel(a, 0, 0, iw, ih)[0])
        assertEquals(iw - 4, DepthIntrinsics.sourcePixel(a, dw - 1, 0, iw, ih)[0])
        // rows: depth row 0 is image row 60 (the 16:9 crop of 480 rows is rows 60..419), the last is row 418
        assertEquals(60, DepthIntrinsics.sourcePixel(a, 0, 0, iw, ih)[1])
        assertEquals(ih - 64, DepthIntrinsics.sourcePixel(a, 0, dh - 1, iw, ih)[1])
        // sampled rows come from the crop, never from the top/bottom bands
        val y = ByteBuffer.wrap(ByteArray(iw * ih) { i -> val row = i / iw; (if (row < 60 || row >= 420) 16 else 235).toByte() })
        val uv = ByteBuffer.wrap(ByteArray((iw / 2) * (ih / 2) * 2) { 128.toByte() })
        val rgb = DepthIntrinsics.sampleYuv(y, iw, 1, uv, uv, iw, 2, iw, ih, dw, dh, a)
        for (c in rgb) assertTrue("bright: %06x".format(c), (c and 255) >= 250)
        // the plain resample (what the recorder did before) would have mixed the black bands in
        val old = YuvSampler.sample(y, iw, 1, uv, uv, iw, 2, iw, ih, dw, dh)
        assertTrue("old top-left is black: %06x".format(old[0]), (old[0] and 255) < 10)
    }
}
