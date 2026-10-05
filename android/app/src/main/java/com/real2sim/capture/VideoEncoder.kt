package com.real2sim.capture

import android.media.Image
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaCodecList
import android.media.MediaFormat
import android.media.MediaMuxer
import android.util.Log
import android.view.Surface
import java.io.BufferedWriter
import java.io.File

/**
 * HEVC (fallback AVC) encoder into an mp4. Two input paths:
 *  - surface mode: Camera2 renders straight into [inputSurface]; buffer pts = sensor timestamp.
 *  - YUV mode: [encodeYuv] copies an android.media.Image (YUV_420_888, e.g. ARCore's CPU image).
 * Every encoded frame's pts (us, CLOCK_BOOTTIME / 1000) is appended to <mp4>.pts.csv so the
 * converter can map frames.jsonl rows to video frames by timestamp, never by counting.
 */
class VideoEncoder(
    val file: File,
    val width: Int,
    val height: Int,
    fps: Int,
    bitrate: Int,
    surfaceInput: Boolean,
) {
    val mime: String = if (hasEncoder(MediaFormat.MIMETYPE_VIDEO_HEVC)) MediaFormat.MIMETYPE_VIDEO_HEVC else MediaFormat.MIMETYPE_VIDEO_AVC
    private val codec: MediaCodec = MediaCodec.createEncoderByType(mime)
    var inputSurface: Surface? = null
        private set
    private var muxer = MediaMuxer(file.absolutePath, MediaMuxer.OutputFormat.MUXER_OUTPUT_MPEG_4)
    private var track = -1
    private var muxStarted = false
    private val info = MediaCodec.BufferInfo()
    private val pts: BufferedWriter = File(file.path + ".pts.csv").bufferedWriter().apply { write("frame,pts_us,t_ns\n") }
    var encoded = 0
        private set
    private val lock = Object()
    @Volatile private var stopped = false
    private var lastPtsUs = Long.MIN_VALUE

    init {
        val fmt = MediaFormat.createVideoFormat(mime, width, height).apply {
            setInteger(MediaFormat.KEY_BIT_RATE, bitrate)
            setInteger(MediaFormat.KEY_FRAME_RATE, fps)
            setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
            setInteger(MediaFormat.KEY_COLOR_FORMAT,
                if (surfaceInput) MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface
                else MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible)
            // Constant quality-ish: we care about reconstruction detail, not bandwidth.
            setInteger(MediaFormat.KEY_BITRATE_MODE, MediaCodecInfo.EncoderCapabilities.BITRATE_MODE_VBR)
        }
        codec.configure(fmt, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        if (surfaceInput) inputSurface = codec.createInputSurface()
        codec.start()
    }

    /** Copy one YUV_420_888 image into the encoder. Blocks briefly if the encoder is busy. */
    fun encodeYuv(img: Image, tNs: Long): Boolean {
        synchronized(lock) {
            if (stopped || tNs / 1000 <= lastPtsUs) return false   // the muxer needs strictly increasing pts
            val idx = codec.dequeueInputBuffer(10_000)
            if (idx < 0) { drain(false); return false }
            val dst = codec.getInputImage(idx)
            if (dst == null) { codec.queueInputBuffer(idx, 0, 0, tNs / 1000, 0); return false }
            copyYuv(img, dst)
            codec.queueInputBuffer(idx, 0, width * height * 3 / 2, tNs / 1000, 0)
            lastPtsUs = tNs / 1000
            drain(false)
            return true
        }
    }

    /** Call regularly in surface mode (e.g. from capture callbacks) to move output to the muxer. */
    fun poll() = synchronized(lock) { if (!stopped) drain(false) }

    private fun drain(eos: Boolean) {
        val deadline = System.nanoTime() + 3_000_000_000L
        while (true) {
            val i = codec.dequeueOutputBuffer(info, if (eos) 10_000 else 0)
            when {
                i == MediaCodec.INFO_TRY_AGAIN_LATER -> if (!eos || System.nanoTime() > deadline) return
                i == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED -> {
                    track = muxer.addTrack(codec.outputFormat)
                    muxer.start(); muxStarted = true
                }
                i >= 0 -> {
                    val buf = codec.getOutputBuffer(i)!!
                    val isConfig = info.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0
                    if (!isConfig && info.size > 0 && muxStarted) {
                        muxer.writeSampleData(track, buf, info)
                        pts.write("$encoded,${info.presentationTimeUs},${info.presentationTimeUs * 1000}\n")
                        encoded++
                    }
                    codec.releaseOutputBuffer(i, false)
                    if (info.flags and MediaCodec.BUFFER_FLAG_END_OF_STREAM != 0) return
                }
            }
        }
    }

    fun stop() {
        synchronized(lock) {
            if (stopped) return
            try {
                if (inputSurface != null) codec.signalEndOfInputStream()
                else {
                    val idx = codec.dequeueInputBuffer(50_000)
                    if (idx >= 0) codec.queueInputBuffer(idx, 0, 0, 0, MediaCodec.BUFFER_FLAG_END_OF_STREAM)
                }
                drain(true)
            } catch (e: Exception) { Log.w("VideoEncoder", "drain at stop", e) }
            stopped = true
            try { codec.stop() } catch (_: Exception) {}
            codec.release()
            if (muxStarted) try { muxer.stop() } catch (e: Exception) { Log.w("VideoEncoder", "muxer stop", e) }
            muxer.release()
            inputSurface?.release()
            pts.close()
        }
    }

    /** Release without finishing the file and delete it and its pts.csv (a configuration attempt that was not used). */
    fun discard() {
        synchronized(lock) {
            if (stopped) return
            stopped = true
            try { codec.stop() } catch (_: Exception) {}
            try { codec.release() } catch (_: Exception) {}
            try { muxer.release() } catch (_: Exception) {}
            inputSurface?.release()
            try { pts.close() } catch (_: Exception) {}
            file.delete(); File(file.path + ".pts.csv").delete()
        }
    }

    companion object {
        fun hasEncoder(mime: String): Boolean =
            MediaCodecList(MediaCodecList.REGULAR_CODECS).codecInfos.any { it.isEncoder && it.supportedTypes.any { t -> t.equals(mime, true) } }

        /** Generic YUV_420_888 -> YUV_420_888 copy honouring both images' row/pixel strides. */
        fun copyYuv(src: Image, dst: Image) {
            for (p in 0 until 3) {
                val sp = src.planes[p]; val dp = dst.planes[p]
                val w = if (p == 0) src.width else src.width / 2
                val h = if (p == 0) src.height else src.height / 2
                val sb = sp.buffer; val db = dp.buffer
                val sps = sp.pixelStride; val dps = dp.pixelStride
                if (sps == 1 && dps == 1) {
                    val row = ByteArray(w)
                    for (y in 0 until h) {
                        sb.position(y * sp.rowStride); sb.get(row, 0, w)
                        db.position(y * dp.rowStride); db.put(row, 0, w)
                    }
                } else {
                    for (y in 0 until h) {
                        val so = y * sp.rowStride; val dOff = y * dp.rowStride
                        for (x in 0 until w) db.put(dOff + x * dps, sb.get(so + x * sps))
                    }
                }
            }
        }
    }
}
