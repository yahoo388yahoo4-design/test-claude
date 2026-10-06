package com.real2sim.capture

import java.nio.ByteBuffer
import kotlin.math.max
import kotlin.math.min

/**
 * Pinhole intrinsics maths for the ARCore depth maps (pure Kotlin, no Android dependencies).
 *
 * ARCore depth covers the field of view of the camera *texture* (textureIntrinsics), not of the CPU image
 * (imageIntrinsics), and the two can have different aspects (e.g. a 16:9 texture next to a 4:3 CPU image).
 * The depth intrinsics are therefore the texture intrinsics scaled to the depth map size, and a depth pixel
 * maps to a CPU-image pixel through both: depth pixel -> normalised ray (depth K) -> image pixel (image K),
 * which is an affine map in pixel space.
 */
object DepthIntrinsics {
    /** [k] = fx, fy, cx, cy for an image of [fromW] x [fromH]; returns the same intrinsics at [toW] x [toH]. */
    fun scale(k: FloatArray, fromW: Int, fromH: Int, toW: Int, toH: Int): FloatArray {
        val sx = toW.toFloat() / fromW; val sy = toH.toFloat() / fromH
        return floatArrayOf(k[0] * sx, k[1] * sy, k[2] * sx, k[3] * sy)
    }

    /**
     * Pixel map from an image with intrinsics [from] to one with intrinsics [to] (both fx, fy, cx, cy, same
     * camera centre): u_to = a[0] * u_from + a[1], v_to = a[2] * v_from + a[3].
     */
    fun affine(from: FloatArray, to: FloatArray): FloatArray {
        val ax = to[0] / from[0]; val ay = to[1] / from[1]
        return floatArrayOf(ax, to[2] - ax * from[2], ay, to[3] - ay * from[3])
    }

    /**
     * YUV_420_888 -> outW*outH 0xRRGGBB, sampling the source pixel (nearest) that [map] (see [affine]) gives
     * for each output pixel; sources outside the image are clamped to its border. Same conventions as
     * [YuvSampler.sample].
     */
    fun sampleYuv(y: ByteBuffer, yRow: Int, yPix: Int, u: ByteBuffer, v: ByteBuffer, uvRow: Int, uvPix: Int,
                  srcW: Int, srcH: Int, outW: Int, outH: Int, map: FloatArray): IntArray {
        val out = IntArray(outW * outH)
        val sxs = IntArray(outW) { i -> (map[0] * i + map[1] + 0.5f).toInt().coerceIn(0, srcW - 1) }
        for (j in 0 until outH) {
            val sy = (map[2] * j + map[3] + 0.5f).toInt().coerceIn(0, srcH - 1)
            for (i in 0 until outW) {
                val sx = sxs[i]
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

    /** Source pixel (nearest) of output pixel ([i], [j]) under [map], clamped to [srcW] x [srcH]; for tests. */
    fun sourcePixel(map: FloatArray, i: Int, j: Int, srcW: Int, srcH: Int): IntArray = intArrayOf(
        min(srcW - 1, max(0, (map[0] * i + map[1] + 0.5f).toInt())),
        min(srcH - 1, max(0, (map[2] * j + map[3] + 0.5f).toInt())))
}
