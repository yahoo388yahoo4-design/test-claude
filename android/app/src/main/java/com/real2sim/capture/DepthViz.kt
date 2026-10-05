package com.real2sim.capture

import java.nio.ByteBuffer
import kotlin.math.max
import kotlin.math.min

/**
 * Depth visualisation shared by the live view and the session viewer: depth -> "turbo" colour map
 * (perceptually ordered rainbow; near = red, far = blue), invalid / filtered-out pixels fully transparent.
 */
object DepthViz {
    /** Turbo colour map (polynomial fit of Google's Turbo), t in [0, 1] -> 0xFFRRGGBB. */
    fun turbo(t0: Float): Int {
        val t = t0.coerceIn(0f, 1f)
        val r = 0.13572138f + t * (4.61539260f + t * (-42.66032258f + t * (132.13108234f + t * (-152.94239396f + t * 59.28637943f))))
        val g = 0.09140261f + t * (2.19418839f + t * (4.84296658f + t * (-14.18503333f + t * (4.27729857f + t * 2.82956604f))))
        val b = 0.10667330f + t * (12.64194608f + t * (-60.58204836f + t * (110.36276771f + t * (-89.90310912f + t * 27.34824973f))))
        fun c(v: Float) = (v.coerceIn(0f, 1f) * 255f + 0.5f).toInt()
        return (0xff shl 24) or (c(r) shl 16) or (c(g) shl 8) or c(b)
    }

    private val LUT = IntArray(256) { turbo(it / 255f) }

    /** Colour of a depth (m) for a [maxM] range: near red, far blue (the near-black ends of turbo are skipped). */
    fun color(m: Float, maxM: Float): Int = LUT[((0.08f + 0.88f * (1f - min(m / maxM, 1f))) * 255f).toInt().coerceIn(0, 255)]

    /** Metres (0 = invalid) -> ARGB pixels, invalid transparent. */
    fun colorizeMeters(d: FloatArray, maxM: Float = 5f, out: IntArray? = null): IntArray {
        val o = if (out != null && out.size == d.size) out else IntArray(d.size)
        for (i in d.indices) o[i] = if (d[i] > 0f) color(d[i], maxM) else 0
        return o
    }

    /**
     * Viewer path: DEPTH16 + optional confidence (0..255, or ARCore levels 0..2 when [confLevels]) through
     * a [DepthFilter] (no intrinsics: confidence, range, edge-preserving median, flying pixels).
     */
    fun filteredArgb(mm: ShortArray, w: Int, h: Int, conf: ByteArray?, confLevels: Boolean, maxM: Float = 5f,
                     cfg: DepthFilter.Config = VIEWER_FILTER): IntArray {
        val c = if (conf != null && confLevels) ByteArray(conf.size) { LEVEL_TO_255[(conf[it].toInt() and 0xff).coerceIn(0, 2)] } else conf
        val m = DepthFilter(cfg).filter(mm, c, w, h, 0f, 0f, 0f, 0f, null)
        return colorizeMeters(m, maxM)
    }

    /** ARCore confidence levels 0..2 (conf.zlib.bin) -> representative 0..255 values. */
    private val LEVEL_TO_255 = byteArrayOf(42, 127, 212.toByte())

    /** Viewer default: level >= 1 passes (0.45 * 255 = 115 < 127), wider range for display. */
    val VIEWER_FILTER = DepthFilter.Config(minConfidence = 0.45f, maxRangeM = 8f, temporal = false)

    /** ARGB pixels -> RGBA bytes (GL_RGBA upload). */
    fun toRgba(px: IntArray, out: ByteArray? = null): ByteArray {
        val o = if (out != null && out.size == px.size * 4) out else ByteArray(px.size * 4)
        for (i in px.indices) {
            val c = px[i]
            o[i * 4] = (c shr 16).toByte(); o[i * 4 + 1] = (c shr 8).toByte(); o[i * 4 + 2] = c.toByte(); o[i * 4 + 3] = (c ushr 24).toByte()
        }
        return o
    }
}

/** YUV_420_888 -> small RGB (nearest sample), for colouring the TSDF at depth resolution. */
object YuvSampler {
    /**
     * [y]/[u]/[v] plane buffers (absolute indexing from 0) with their row / pixel strides; [srcW]x[srcH] the
     * image size. Returns outW*outH 0xRRGGBB.
     */
    fun sample(y: ByteBuffer, yRow: Int, yPix: Int, u: ByteBuffer, v: ByteBuffer, uvRow: Int, uvPix: Int,
               srcW: Int, srcH: Int, outW: Int, outH: Int): IntArray {
        val out = IntArray(outW * outH)
        for (j in 0 until outH) {
            val sy = min(srcH - 1, ((j + 0.5f) * srcH / outH).toInt())
            for (i in 0 until outW) {
                val sx = min(srcW - 1, ((i + 0.5f) * srcW / outW).toInt())
                val yy = (y.get(sy * yRow + sx * yPix).toInt() and 255) - 16
                val ui = (sy / 2) * uvRow + (sx / 2) * uvPix
                val uu = (u.get(ui).toInt() and 255) - 128
                val vv = (v.get(ui).toInt() and 255) - 128
                val c = 1.164f * max(0, yy)
                val r = (c + 1.596f * vv).toInt().coerceIn(0, 255)
                val g = (c - 0.392f * uu - 0.813f * vv).toInt().coerceIn(0, 255)
                val b = (c + 2.017f * uu).toInt().coerceIn(0, 255)
                out[j * outW + i] = (r shl 16) or (g shl 8) or b
            }
        }
        return out
    }
}
