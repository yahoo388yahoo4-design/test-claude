package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.DngCreator
import android.hardware.camera2.TotalCaptureResult
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.media.ImageReader
import android.media.MediaCodec
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import android.view.Surface
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.Executor

data class Camera2Options(
    val rawDng: Boolean,
    val rawPeriodMs: Long = 1000,
    val lock: Boolean,
    val oisOff: Boolean,
    val maxWidth: Int = 1920,
)

/**
 * Mode B ("multicam" in FORMAT.md): Camera2 only, no ARCore, so no poses on device; poses are
 * recovered offline (SfM) with IMU / ToF for metric scale.
 *
 *  - Opens the rear LOGICAL multi-camera and streams every physical camera it will accept at once
 *    (OutputConfiguration.setPhysicalCameraId), one HEVC video per camera -> cams/<name>.mp4.
 *    Combinations are probed with isSessionConfigurationSupported, greedily adding cameras.
 *  - Per frame and per physical camera: the full set of reconstruction-relevant CaptureResult keys
 *    (exposure, ISO, focus distance, LENS_INTRINSIC_CALIBRATION, LENS_DISTORTION, LENS_POSE_*,
 *    OIS samples, rolling-shutter skew, ...) -> cams/<name>.jsonl, joined to video samples by
 *    sensor timestamp at stop.
 *  - RAW_SENSOR DNG every rawPeriodMs on the first stream's camera -> cams/raw/<name>_<t>.dng.
 *  - DEPTH16 (ToF) from any camera advertising DEPTH_OUTPUT, opened as a second device if the HAL
 *    allows -> cams/tof_depth.zlib.bin + cams/tof_depth.jsonl.
 *  - cams/calibration.json: intrinsics, distortion, extrinsics (LENS_POSE_*) of every camera.
 */
class Camera2Recorder(
    private val ctx: Context,
    private val s: SessionWriter,
    private val opt: Camera2Options,
    private val status: (String) -> Unit,
) {
    private val cm = ctx.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private val thread = HandlerThread("camera2").apply { start() }
    private val handler = Handler(thread.looper)
    private val executor = Executor { handler.post(it) }

    private class Stream(val name: String, val physicalId: String?, val camId: String, val size: Size, val chars: CameraCharacteristics) {
        lateinit var encoder: VideoEncoder
        val results = ConcurrentHashMap<Long, JSONObject>()
    }

    private val streams = mutableListOf<Stream>()
    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var rawReader: ImageReader? = null
    private var rawStream: Stream? = null
    private val pendingRaw = ConcurrentHashMap<Long, TotalCaptureResult>()
    private var rawCount = 0
    private var frames = 0L
    @Volatile private var running = false

    // ToF
    private var depthDevice: CameraDevice? = null
    private var depthReader: ImageReader? = null
    private var depthBlob: SessionWriter.Blob? = null
    private var depthCount = 0

    private fun lensName(c: CameraCharacteristics, fallback: String): String {
        val f = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: return fallback
        val sensorW = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)?.width ?: return fallback
        // 35 mm-equivalent focal length -> ultrawide / wide / tele names
        val eq = f * 36f / sensorW
        return when {
            eq < 18 -> "ultrawide"
            eq < 40 -> "wide"
            eq < 95 -> "tele"
            else -> "tele2"
        } + "_" + fallback
    }

    private fun pickSize(c: CameraCharacteristics): Size? {
        val map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP) ?: return null
        val sizes = map.getOutputSizes(MediaCodec::class.java) ?: return null
        val ok = sizes.filter { it.width <= opt.maxWidth }
        return ok.filter { it.width * 3 == it.height * 4 }.maxByOrNull { it.width * it.height }
            ?: ok.maxByOrNull { it.width * it.height }
    }

    private fun capabilities(c: CameraCharacteristics) = c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: intArrayOf()

    @SuppressLint("MissingPermission")
    fun start(): String? {
        // Full inventory first (useful even if recording fails).
        val inv = CameraInfo.inventory(cm)
        s.text("extras/camera_inventory.json", inv.toString(1))

        val back = cm.cameraIdList.filter { cm.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK }
        if (back.isEmpty()) return "no back camera"
        val logical = back.firstOrNull { CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA in capabilities(cm.getCameraCharacteristics(it)) }
            ?: back.first()
        val lc = cm.getCameraCharacteristics(logical)
        val physical = lc.physicalCameraIds.toList()
        s.meta.getJSONObject("android").put("multicam", JSONObject().apply {
            put("logical_id", logical); put("physical_ids", JSONArray(physical))
            put("back_ids", JSONArray(back))
            if (Build.VERSION.SDK_INT >= 30) put("concurrent_sets", JSONArray(cm.concurrentCameraIds.map { JSONArray(it.toList()) }))
        })

        val candidates = if (physical.isEmpty()) listOf(Stream(lensName(lc, logical), null, logical, pickSize(lc) ?: Size(1920, 1080), lc))
        else physical.mapNotNull { pid ->
            val pc = cm.getCameraCharacteristics(pid)
            val sz = pickSize(pc) ?: pickSize(lc) ?: return@mapNotNull null
            Stream(lensName(pc, pid), pid, logical, sz, pc)
        }.sortedBy { if (it.name.startsWith("wide")) 0 else 1 }   // main camera first

        // ToF / DEPTH16 camera (may be hidden from cameraIdList; inventory probed ids 0..15)
        val depthId = inv.getJSONObject("cameras").keys().asSequence().firstOrNull { id ->
            inv.getJSONObject("cameras").getJSONObject(id).optBoolean("_has_depth16")
        }
        s.meta.getJSONObject("android").getJSONObject("multicam").put("depth16_camera", depthId ?: JSONObject.NULL)

        val cb = object : CameraDevice.StateCallback() {
            override fun onOpened(d: CameraDevice) { device = d; configure(d, candidates) }
            override fun onDisconnected(d: CameraDevice) { d.close() }
            override fun onError(d: CameraDevice, e: Int) { status("camera error $e"); d.close() }
        }
        cm.openCamera(logical, executor, cb)
        if (depthId != null) openDepth(depthId)
        writeCalibration(candidates, lc)
        return null
    }

    private fun outputs(sel: List<Stream>, withRaw: Boolean): List<OutputConfiguration> {
        val outs = sel.map { st ->
            OutputConfiguration(st.encoder.inputSurface!!).apply { st.physicalId?.let { setPhysicalCameraId(it) } }
        }.toMutableList()
        if (withRaw) rawReader?.let { r -> outs += OutputConfiguration(r.surface).apply { rawStream?.physicalId?.let { setPhysicalCameraId(it) } } }
        return outs
    }

    private fun supported(d: CameraDevice, outs: List<OutputConfiguration>): Boolean = try {
        d.isSessionConfigurationSupported(SessionConfiguration(SessionConfiguration.SESSION_REGULAR, outs, executor,
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(p0: CameraCaptureSession) {}
                override fun onConfigureFailed(p0: CameraCaptureSession) {}
            }))
    } catch (e: Exception) { Log.w(TAG, "isSessionConfigurationSupported", e); outs.size == 1 }

    private fun configure(d: CameraDevice, candidates: List<Stream>) {
        // Encoders for all candidates; drop those the HAL will not stream together.
        for (st in candidates) {
            st.encoder = VideoEncoder(s.file("cams/${st.name}.mp4"), st.size.width, st.size.height, 30,
                (st.size.width * st.size.height * 30 * 0.25).toInt().coerceIn(8_000_000, 80_000_000), surfaceInput = true)
        }
        val chosen = mutableListOf<Stream>()
        for (st in candidates) {
            val trial = chosen + st
            if (supported(d, outputs(trial, false))) chosen += st
        }
        if (chosen.isEmpty()) chosen += candidates.first()
        for (st in candidates - chosen.toSet()) { st.encoder.stop(); File(st.encoder.file.path).delete(); File(st.encoder.file.path + ".pts.csv").delete() }
        streams += chosen

        if (opt.rawDng) {
            val st = chosen.first()
            val map = st.chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            val rawSizes = map?.getOutputSizes(ImageFormat.RAW_SENSOR)
            if (CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_RAW in capabilities(st.chars) && !rawSizes.isNullOrEmpty()) {
                val rs = rawSizes.maxByOrNull { it.width * it.height }!!
                rawReader = ImageReader.newInstance(rs.width, rs.height, ImageFormat.RAW_SENSOR, 3)
                rawStream = st
                if (!supported(d, outputs(chosen, true))) {
                    rawReader?.close(); rawReader = null; rawStream = null
                    s.meta.getJSONObject("android").put("raw_error", "RAW stream not supported together with ${chosen.size} video streams")
                } else rawReader!!.setOnImageAvailableListener({ r -> onRaw(r) }, handler)
            } else s.meta.getJSONObject("android").put("raw_error", "camera ${st.name} has no RAW capability")
        }

        val cfg = SessionConfiguration(SessionConfiguration.SESSION_REGULAR, outputs(chosen, rawReader != null), executor,
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(cs: CameraCaptureSession) { session = cs; startRepeating(d, cs, locked = false) }
                override fun onConfigureFailed(cs: CameraCaptureSession) { status("capture session configure failed") }
            })
        d.createCaptureSession(cfg)
        s.meta.getJSONObject("android").getJSONObject("multicam").put("streams", JSONArray(chosen.map { st ->
            JSONObject().apply { put("name", st.name); put("physical_id", st.physicalId ?: JSONObject.NULL); put("w", st.size.width); put("h", st.size.height)
                put("file", "cams/${st.name}.mp4"); put("codec", st.encoder.mime) }
        }))
        status("multicam: ${chosen.joinToString { it.name + " " + it.size }}" + (if (rawReader != null) " +RAW" else ""))
    }

    private fun baseRequest(d: CameraDevice, locked: Boolean, last: TotalCaptureResult?): CaptureRequest.Builder {
        val rb = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
        streams.forEach { rb.addTarget(it.encoder.inputSurface!!) }
        rb.set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE, CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF) // EIS warps geometry
        if (opt.oisOff) rb.set(CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE, CaptureRequest.LENS_OPTICAL_STABILIZATION_MODE_OFF)
        rb.set(CaptureRequest.STATISTICS_OIS_DATA_MODE, CaptureRequest.STATISTICS_OIS_DATA_MODE_ON)
        rb.set(CaptureRequest.DISTORTION_CORRECTION_MODE, CaptureRequest.DISTORTION_CORRECTION_MODE_OFF) // keep raw optics; LENS_DISTORTION recorded
        if (locked && last != null) {
            rb.set(CaptureRequest.CONTROL_AE_LOCK, true)
            rb.set(CaptureRequest.CONTROL_AWB_LOCK, true)
            rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
            last.get(CaptureResult.LENS_FOCUS_DISTANCE)?.let { rb.set(CaptureRequest.LENS_FOCUS_DISTANCE, it) }
        } else rb.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
        return rb
    }

    private var lastResult: TotalCaptureResult? = null

    private val captureCb = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(cs: CameraCaptureSession, req: CaptureRequest, res: TotalCaptureResult) {
            lastResult = res
            frames++
            val phys: Map<String, CaptureResult> = if (Build.VERSION.SDK_INT >= 31) res.physicalCameraTotalResults else @Suppress("DEPRECATION") res.physicalCameraResults
            for (st in streams) {
                val r: CaptureResult = st.physicalId?.let { phys[it] } ?: res
                val t = r.get(CaptureResult.SENSOR_TIMESTAMP) ?: res.get(CaptureResult.SENSOR_TIMESTAMP) ?: continue
                val o = JSONObject().put("t", t / 1e9)
                CameraInfo.resultFields(r, o)
                st.results[t] = o
                st.encoder.poll()
            }
            if (req.tag == RAW_TAG) res.get(CaptureResult.SENSOR_TIMESTAMP)?.let { t -> pendingRaw[t] = res }
            if (frames % 30 == 0L) status("multicam ${streams.size} cams f=$frames raw=$rawCount tof=$depthCount drop=${s.droppedWrites}")
        }
    }

    private fun startRepeating(d: CameraDevice, cs: CameraCaptureSession, locked: Boolean) {
        running = true
        cs.setRepeatingRequest(baseRequest(d, locked, lastResult).build(), captureCb, handler)
        if (opt.lock && !locked) handler.postDelayed({ if (running) startRepeating(d, cs, locked = true) }, 1500)
        if (rawReader != null && !locked) handler.postDelayed(object : Runnable {
            override fun run() {
                if (!running) return
                try {
                    val rb = baseRequest(d, opt.lock, lastResult)
                    rb.addTarget(rawReader!!.surface)
                    rb.setTag(RAW_TAG)
                    cs.capture(rb.build(), captureCb, handler)
                } catch (e: Exception) { Log.w(TAG, "raw capture", e) }
                handler.postDelayed(this, opt.rawPeriodMs)
            }
        }, opt.rawPeriodMs)
    }

    private fun onRaw(r: ImageReader) {
        val img = r.acquireNextImage() ?: return
        val res = pendingRaw.remove(img.timestamp)
        val st = rawStream
        if (res == null || st == null) { img.close(); return }
        val rel = "cams/raw/${st.name}_%.6f.dng".format(java.util.Locale.US, img.timestamp / 1e9)
        try {
            s.file(rel).outputStream().use { out -> DngCreator(st.chars, res).use { it.writeImage(out, img) } }
            rawCount++
        } catch (e: Exception) { Log.w(TAG, "dng", e) } finally { img.close() }
    }

    // ------------------------------------------------------------------ ToF DEPTH16
    @SuppressLint("MissingPermission")
    private fun openDepth(id: String) {
        val c = cm.getCameraCharacteristics(id)
        val sizes = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)?.getOutputSizes(ImageFormat.DEPTH16) ?: return
        val sz = sizes.maxByOrNull { it.width * it.height } ?: return
        val reader = ImageReader.newInstance(sz.width, sz.height, ImageFormat.DEPTH16, 4)
        depthReader = reader
        depthBlob = s.Blob("cams/tof_depth.zlib.bin")
        val confBlob = s.Blob("cams/tof_conf.zlib.bin")
        val jsonl = "cams/tof_depth.jsonl"
        val k = c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)
        reader.setOnImageAvailableListener({ r ->
            val img = r.acquireNextImage() ?: return@setOnImageAvailableListener
            try {
                val p = img.planes[0]
                val raw = SessionWriter.packPlane(p.buffer, img.width, img.height, p.rowStride, 2, p.pixelStride.coerceAtLeast(2))
                // DEPTH16: low 13 bits = range mm, high 3 bits = confidence (0 = 100%, 1..7 = 0%..86%)
                val mm = ByteArray(raw.size); val conf = ByteArray(raw.size / 2)
                for (i in 0 until raw.size / 2) {
                    val v = (raw[2 * i].toInt() and 0xff) or ((raw[2 * i + 1].toInt() and 0xff) shl 8)
                    val d = v and 0x1fff; val cc = (v shr 13) and 7
                    mm[2 * i] = (d and 0xff).toByte(); mm[2 * i + 1] = (d shr 8).toByte()
                    conf[i] = (if (cc == 0) 2 else if (cc >= 5) 1 else 0).toByte()
                }
                val dr = depthBlob!!.append(mm); val cr = confBlob.append(conf)
                val o = JSONObject().apply {
                    put("i", depthCount); put("t", img.timestamp / 1e9); put("w", img.width); put("h", img.height)
                    put("d", dr); put("c", cr); put("K", if (k != null) JSONArray(k.take(4).map { it.toDouble() }) else JSONObject.NULL)
                }
                s.csv(jsonl, "", o.toString())
                depthCount++
            } finally { img.close() }
        }, handler)
        cm.openCamera(id, executor, object : CameraDevice.StateCallback() {
            override fun onOpened(d: CameraDevice) {
                depthDevice = d
                d.createCaptureSession(SessionConfiguration(SessionConfiguration.SESSION_REGULAR, listOf(OutputConfiguration(reader.surface)), executor,
                    object : CameraCaptureSession.StateCallback() {
                        override fun onConfigured(cs: CameraCaptureSession) {
                            val rb = d.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW); rb.addTarget(reader.surface)
                            cs.setRepeatingRequest(rb.build(), null, handler)
                        }
                        override fun onConfigureFailed(cs: CameraCaptureSession) { s.meta.getJSONObject("android").put("tof_error", "configure failed") }
                    }))
            }
            override fun onDisconnected(d: CameraDevice) { d.close() }
            override fun onError(d: CameraDevice, e: Int) { s.meta.getJSONObject("android").put("tof_error", "open error $e (concurrent use refused?)"); d.close() }
        })
        s.meta.getJSONObject("android").put("tof", JSONObject().apply {
            put("camera_id", id); put("w", sz.width); put("h", sz.height)
            put("files", "cams/tof_depth.zlib.bin (uint16 mm) + cams/tof_conf.zlib.bin (0..2) + cams/tof_depth.jsonl")
        })
    }

    // ------------------------------------------------------------------ calibration.json
    private fun writeCalibration(cands: List<Stream>, lc: CameraCharacteristics) {
        val o = JSONObject()
        for (st in cands) {
            val c = st.chars
            val aa = c.get(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE)
            o.put(st.name, JSONObject().apply {
                put("camera_id", st.physicalId ?: st.camId)
                put("stream_wh", JSONArray(listOf(st.size.width, st.size.height)))
                put("intrinsics_active_array", CameraInfo.toJson(c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)))
                put("K_stream", kForStream(c, st.size) ?: JSONObject.NULL)
                put("K_source", if ((c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)?.get(0) ?: 0f) > 0f) "LENS_INTRINSIC_CALIBRATION" else "focal_length/physical_size (nominal)")
                put("distortion_k1k2k3p1p2", CameraInfo.toJson(c.get(CameraCharacteristics.LENS_DISTORTION)))
                put("pose_translation", CameraInfo.toJson(c.get(CameraCharacteristics.LENS_POSE_TRANSLATION)))
                put("pose_rotation_xyzw", CameraInfo.toJson(c.get(CameraCharacteristics.LENS_POSE_ROTATION)))
                put("pose_reference", CameraInfo.toJson(c.get(CameraCharacteristics.LENS_POSE_REFERENCE)))
                put("pre_correction_active_array", CameraInfo.toJson(aa))
                put("active_array", CameraInfo.toJson(c.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)))
                put("pixel_array", CameraInfo.toJson(c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE)))
                put("physical_size_mm", CameraInfo.toJson(c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)))
                put("focal_lengths_mm", CameraInfo.toJson(c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)))
                put("timestamp_source", if (c.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME) "REALTIME" else "UNKNOWN")
                put("sync_type", CameraInfo.toJson(lc.get(CameraCharacteristics.LOGICAL_MULTI_CAMERA_SENSOR_SYNC_TYPE)))
            })
        }
        o.put("_notes", "intrinsics_active_array = [fx, fy, cx, cy, s] in pre-correction active-array pixels (Camera2). K_stream rescales to the " +
            "stream assuming the stream covers the full active array (true for 4:3 streams; 16:9 streams crop vertically). " +
            "pose_* = lens pose relative to pose_reference (0 primary camera, 1 gyroscope, 2 undefined, 3 automotive), Android sensor axes.")
        s.text("cams/calibration.json", o.toString(1))
    }

    private fun kForStream(c: CameraCharacteristics, size: Size): JSONArray? {
        val aa = c.get(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE) ?: return null
        val k = c.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)?.takeIf { it[0] > 0f } ?: run {
            // No factory calibration (common): pinhole from focal length and physical sensor size.
            val f = c.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: return null
            val phys = c.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE) ?: return null
            val pa = c.get(CameraCharacteristics.SENSOR_INFO_PIXEL_ARRAY_SIZE) ?: return null
            val fpx = f / phys.width * pa.width
            floatArrayOf(fpx, fpx, aa.width() / 2f, aa.height() / 2f, 0f)
        }
        val sx = size.width.toDouble() / aa.width()
        // crop to stream aspect, centred
        val cropH = aa.width() * size.height.toDouble() / size.width
        val oy = (aa.height() - cropH) / 2
        return JSONArray(listOf(k[0] * sx, k[1] * sx, (k[2] - 0) * sx, (k[3] - oy) * sx))
    }

    /** Per-lens progress for the live coverage view. */
    fun liveStatus(): String =
        streams.joinToString("\n") { st -> "${st.name} ${st.size.width}x${st.size.height}: ${st.encoder.encoded} frames" } +
            "\nRAW DNG $rawCount   ToF depth $depthCount   capture results $frames"

    // ------------------------------------------------------------------ stop
    fun stop() {
        running = false
        val done = java.util.concurrent.CountDownLatch(1)
        handler.post {
            try { session?.stopRepeating(); session?.abortCaptures() } catch (_: Exception) {}
            done.countDown()
        }
        done.await(2, java.util.concurrent.TimeUnit.SECONDS)
        Thread.sleep(200)
        for (st in streams) st.encoder.stop()
        try { session?.close() } catch (_: Exception) {}
        device?.close(); depthDevice?.close()
        rawReader?.close(); depthReader?.close()
        depthBlob?.close()
        // Join per-frame results to video samples -> cams/<name>.jsonl (FORMAT.md mode C).
        var nTotal = 0
        for (st in streams) {
            val pts = File(st.encoder.file.path + ".pts.csv")
            val K = kForStream(st.chars, st.size)
            val sb = StringBuilder()
            val keys = st.results.keys.sorted().toLongArray()
            pts.readLines().drop(1).forEach { line ->
                val parts = line.split(","); val i = parts[0].toInt(); val tNs = parts[2].toLong()
                // pts are us -> match the result with the nearest sensor timestamp
                var idx = java.util.Arrays.binarySearch(keys, tNs).let { if (it < 0) -it - 1 else it }
                if (idx > 0 && (idx >= keys.size || kotlin.math.abs(keys[idx - 1] - tNs) < kotlin.math.abs(keys[idx] - tNs))) idx--
                val r = if (keys.isNotEmpty() && kotlin.math.abs(keys[idx.coerceIn(0, keys.size - 1)] - tNs) < 2_000_000) st.results[keys[idx.coerceIn(0, keys.size - 1)]] else null
                val o = JSONObject(r?.toString() ?: "{}")
                o.put("i", i); o.put("t", r?.optDouble("t") ?: (tNs / 1e9)); o.put("w", st.size.width); o.put("h", st.size.height)
                o.put("K", K ?: JSONObject.NULL)
                o.optLong("exposure_ns", -1).takeIf { it > 0 }?.let { o.put("exp", it / 1e9) }
                o.opt("iso")?.let { o.put("iso", it) }
                sb.append(o.toString()).append('\n')
                nTotal++
            }
            s.text("cams/${st.name}.jsonl", sb.toString())
        }
        s.meta.put("counts", JSONObject().apply {
            put("frames", 0); put("capture_results", frames); put("cam_samples", nTotal); put("raw_dng", rawCount); put("tof_depth", depthCount)
            for (st in streams) put("cam_${st.name}", st.encoder.encoded)
        })
        s.meta.put("settings", JSONObject().apply {
            put("raw_dng", opt.rawDng); put("raw_period_ms", opt.rawPeriodMs); put("lock", opt.lock); put("ois_off", opt.oisOff)
            put("eis", "off"); put("distortion_correction", "off")
        })
        s.meta.put("notes", "Camera2 multicam: no on-device poses (frames.jsonl empty). Recover with SfM (tools/recover_poses.py); metric scale from ToF depth or IMU.")
        thread.quitSafely()
    }

    companion object { const val TAG = "Camera2Recorder"; const val RAW_TAG = "raw" }
}
