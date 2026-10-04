package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.graphics.ImageFormat
import android.graphics.SurfaceTexture
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
import android.os.Looper
import android.util.Log
import android.util.Range
import android.util.Size
import android.view.Surface
import android.view.TextureView
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
    val fps: Int = 30,
)

/** TextureView that keeps the content aspect ratio inside its parent (centred via layout gravity). */
class PreviewView(ctx: Context) : TextureView(ctx) {
    private var cw = 0
    private var ch = 0

    /** Size of the content as displayed (portrait: stream height x stream width). */
    fun setContentSize(w: Int, h: Int) { cw = w; ch = h; requestLayout() }

    override fun onMeasure(widthMeasureSpec: Int, heightMeasureSpec: Int) {
        val w = MeasureSpec.getSize(widthMeasureSpec)
        val h = MeasureSpec.getSize(heightMeasureSpec)
        when {
            cw == 0 || ch == 0 || w == 0 || h == 0 -> setMeasuredDimension(w, h)
            w.toLong() * ch <= h.toLong() * cw -> setMeasuredDimension(w, (w.toLong() * ch / cw).toInt())
            else -> setMeasuredDimension((h.toLong() * cw / ch).toInt(), h)
        }
    }
}

/**
 * Mode B ("multicam" in FORMAT.md): Camera2 only, no ARCore, so no poses on device; poses are
 * recovered offline (SfM) with IMU / ToF for metric scale.
 *
 *  - Opens the rear LOGICAL multi-camera and streams every physical camera it will accept at once
 *    (OutputConfiguration.setPhysicalCameraId), one HEVC video per camera -> cams/<name>.mp4.
 *  - Hardware budget (CameraBudget.ladder, like the iOS app): if the HAL refuses all lenses at the
 *    requested size / fps, it steps down to 1280 px, then 24 fps, then drops lenses one at a time,
 *    checking each step with isSessionConfigurationSupported and, when that is inconclusive or wrong,
 *    with the real createCaptureSession. What it changed goes to session.json (budget_actions).
 *  - Live preview, always on: an extra preview stream on the main lens ("preview_surface"); if no
 *    configuration accepts that extra stream, the preview shares the recorded main stream through
 *    surface sharing ("recorded_stream"), which costs no extra camera stream. session.json says
 *    which one was used (preview_kind).
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
    private val ui = Handler(Looper.getMainLooper())

    private class Stream(val name: String, val physicalId: String?, val camId: String, val chars: CameraCharacteristics) {
        var size: Size = Size(0, 0)
        var fps = 30
        var encoder: VideoEncoder? = null
        val enc: VideoEncoder get() = encoder!!
        val results = ConcurrentHashMap<Long, JSONObject>()
    }

    private data class Attempt(val step: CameraBudget.Step, val preview: String)

    private var candidates: List<Stream> = emptyList()
    private val streams = mutableListOf<Stream>()
    private var logicalChars: CameraCharacteristics? = null
    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    private var rawReader: ImageReader? = null
    private var rawStream: Stream? = null
    private var rawAllowed = opt.rawDng
    private val pendingRaw = ConcurrentHashMap<Long, TotalCaptureResult>()
    private var rawCount = 0
    private var frames = 0L
    @Volatile private var running = false
    @Volatile private var stopped = false

    // budget + preview
    private var attempts: List<Attempt> = emptyList()
    private var attemptIdx = 0
    private var aeRange: Range<Int>? = null
    @Volatile private var budgetActions: List<String> = emptyList()
    @Volatile var previewKind = PREVIEW_NONE
        private set
    private var previewSize: Size? = null
    private var previewSurface: Surface? = null
    @Volatile private var previewTexture: SurfaceTexture? = null

    /** Live preview; the activity adds it to its layout before [start]. */
    val previewView = PreviewView(ctx).apply {
        surfaceTextureListener = object : TextureView.SurfaceTextureListener {
            override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) { previewTexture = st }
            override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, w: Int, h: Int) {}
            override fun onSurfaceTextureDestroyed(st: SurfaceTexture): Boolean { previewTexture = null; return true }
            override fun onSurfaceTextureUpdated(st: SurfaceTexture) {}
        }
    }

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

    private fun videoSizes(c: CameraCharacteristics): List<Pair<Int, Int>> =
        c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)?.getOutputSizes(MediaCodec::class.java)?.map { it.width to it.height } ?: emptyList()

    /** Largest 4:3 (else any) video size no wider than maxWidth; the smallest size if none is that small. */
    private fun pickSize(c: CameraCharacteristics, maxWidth: Int): Size? {
        val sizes = videoSizes(c).ifEmpty { logicalChars?.let { videoSizes(it) } ?: emptyList() }
        val p = CameraBudget.pickStreamSize(sizes, maxWidth) ?: sizes.minByOrNull { it.first * it.second } ?: return null
        return Size(p.first, p.second)
    }

    private fun capabilities(c: CameraCharacteristics) = c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: intArrayOf()

    private fun multicamMeta(): JSONObject = s.meta.getJSONObject("android").getJSONObject("multicam")

    @SuppressLint("MissingPermission")
    fun start(): String? {
        // Full inventory first (useful even if recording fails).
        val inv = CameraInfo.inventory(cm)
        s.text("extras/camera_inventory.json", JsonSafe.stringify(inv, 1))

        val back = cm.cameraIdList.filter { cm.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK }
        if (back.isEmpty()) return "no back camera"
        val logical = back.firstOrNull { CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA in capabilities(cm.getCameraCharacteristics(it)) }
            ?: back.first()
        val lc = cm.getCameraCharacteristics(logical)
        logicalChars = lc
        val physical = lc.physicalCameraIds.toList()
        s.meta.getJSONObject("android").put("multicam", JSONObject().apply {
            put("logical_id", logical); put("physical_ids", JSONArray(physical))
            put("back_ids", JSONArray(back))
            if (Build.VERSION.SDK_INT >= 30) put("concurrent_sets", JSONArray(cm.concurrentCameraIds.map { JSONArray(it.toList()) }))
        })

        candidates = if (physical.isEmpty()) listOf(Stream(lensName(lc, logical), null, logical, lc))
        else physical.mapNotNull { pid ->
            val pc = cm.getCameraCharacteristics(pid)
            if (videoSizes(pc).isEmpty() && videoSizes(lc).isEmpty()) return@mapNotNull null
            Stream(lensName(pc, pid), pid, logical, pc)
        }.sortedBy { if (it.name.startsWith("wide")) 0 else 1 }   // main camera first
        if (candidates.isEmpty()) return "no camera with video output sizes"
        for (st in candidates) st.size = pickSize(st.chars, opt.maxWidth) ?: Size(1920, 1080)

        // Budget ladder; at every step the extra preview stream first, then the preview from the
        // recorded main stream; only if nothing at all works, the main lens without a preview.
        val ladder = CameraBudget.ladder(candidates.map { it.name }, opt.maxWidth, opt.fps)
        attempts = ladder.flatMap { listOf(Attempt(it, PREVIEW_SURFACE), Attempt(it, PREVIEW_SHARED)) } + Attempt(ladder.last(), PREVIEW_NONE)
        multicamMeta().put("budget_requested", JSONObject().apply {
            put("lenses", JSONArray(candidates.map { it.name })); put("max_width", opt.maxWidth); put("fps", opt.fps)
        })

        // ToF / DEPTH16 camera (may be hidden from cameraIdList; inventory probed ids 0..15)
        val depthId = inv.getJSONObject("cameras").keys().asSequence().firstOrNull { id ->
            inv.getJSONObject("cameras").getJSONObject(id).optBoolean("_has_depth16")
        }
        multicamMeta().put("depth16_camera", depthId ?: JSONObject.NULL)

        val cb = object : CameraDevice.StateCallback() {
            override fun onOpened(d: CameraDevice) { device = d; configureWhenPreviewReady(d, 0) }
            override fun onDisconnected(d: CameraDevice) { d.close() }
            override fun onError(d: CameraDevice, e: Int) { status("camera error $e"); d.close() }
        }
        cm.openCamera(logical, executor, cb)
        if (depthId != null) openDepth(depthId)
        return null
    }

    // ------------------------------------------------------------------ configuration (budget + preview)

    /** The TextureView gets its surface after the first layout pass; wait for it briefly (camera thread). */
    private fun configureWhenPreviewReady(d: CameraDevice, waitedMs: Int) {
        if (stopped) return
        if (previewTexture == null && waitedMs < 2000) {
            handler.postDelayed({ configureWhenPreviewReady(d, waitedMs + 50) }, 50)
            return
        }
        tryNext(d)
    }

    private fun tryNext(d: CameraDevice) {
        while (!stopped && attemptIdx < attempts.size) {
            val a = attempts[attemptIdx++]
            val inFlight = try { tryAttempt(d, a) } catch (e: Exception) { Log.w(TAG, "attempt $a", e); false }
            if (inFlight) return
        }
        if (stopped) return
        multicamMeta().put("configure_error", "no stream configuration was accepted (${attempts.size} tried)")
        writeCalibration()
        status("multicam: the camera accepted no configuration, even the main lens alone at ${CameraBudget.REDUCED_WIDTH} px / ${CameraBudget.REDUCED_FPS} fps")
    }

    /** Sets up encoders for [a]; returns true if a capture session is being created for it. */
    private fun tryAttempt(d: CameraDevice, a: Attempt): Boolean {
        val sel = a.step.lenses.map { n -> candidates.first { it.name == n } }
        for (st in sel) {
            val sz = pickSize(st.chars, a.step.maxWidth) ?: return false
            st.encoder?.let { if (it.width != sz.width || it.height != sz.height) { it.discard(); st.encoder = null } }
            if (st.encoder == null) {
                // Creating the encoders is part of the budget too: codecs have instance / size limits.
                st.encoder = try {
                    VideoEncoder(s.file("cams/${st.name}.mp4"), sz.width, sz.height, a.step.fps,
                        (sz.width.toLong() * sz.height * a.step.fps / 4).coerceIn(8_000_000, 80_000_000).toInt(), surfaceInput = true)
                } catch (e: Exception) { Log.w(TAG, "encoder ${st.name} $sz", e); return false }
            }
            st.size = sz; st.fps = a.step.fps
        }
        val main = sel.first()
        val pv: Surface?
        val psz: Size?
        if (a.preview == PREVIEW_NONE) { pv = null; psz = null } else {
            val tex = previewTexture ?: return false
            psz = if (a.preview == PREVIEW_SURFACE) previewSizeFor(main) ?: return false else main.size
            tex.setDefaultBufferSize(psz.width, psz.height)
            pv = previewSurface?.takeIf { it.isValid } ?: Surface(tex).also { previewSurface = it }
        }
        aeRange = logicalChars?.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            ?.map { it.lower to it.upper }?.let { CameraBudget.pickFpsRange(it, a.step.fps) }?.let { Range(it.first, it.second) }
        val params = try { d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD).apply { aeRange?.let { set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) } }.build() } catch (_: Exception) { null }

        val outs = outputs(sel, a.preview, pv, withRaw = false)
        if (supported(d, outs, params) == false) return false
        var raw = false
        if (rawAllowed && ensureRawReader(main)) raw = supported(d, outputs(sel, a.preview, pv, withRaw = true), params) != false

        val cfg = SessionConfiguration(SessionConfiguration.SESSION_REGULAR, outputs(sel, a.preview, pv, raw), executor,
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(cs: CameraCaptureSession) {
                    if (stopped) { cs.close(); return }
                    commit(d, cs, a, sel, psz, raw)
                }
                override fun onConfigureFailed(cs: CameraCaptureSession) {
                    Log.w(TAG, "configure failed: $a raw=$raw")
                    if (raw) {   // retry the same step without RAW before stepping down
                        rawAllowed = false
                        multicamMeta().put("raw_note", "configure failed with the RAW stream; recorded without RAW")
                        attemptIdx--
                    }
                    tryNext(d)
                }
            })
        params?.let { cfg.sessionParameters = it }
        d.createCaptureSession(cfg)
        return true
    }

    private fun previewSizeFor(st: Stream): Size? {
        val sizes = st.chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)?.getOutputSizes(SurfaceTexture::class.java)
            ?.map { it.width to it.height } ?: return null
        return CameraBudget.pickPreviewSize(sizes, st.size.width, st.size.height)?.let { Size(it.first, it.second) }
    }

    private fun ensureRawReader(st: Stream): Boolean {
        if (rawReader != null) return true
        val map = st.chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
        val rawSizes = map?.getOutputSizes(ImageFormat.RAW_SENSOR)
        if (CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_RAW !in capabilities(st.chars) || rawSizes.isNullOrEmpty()) {
            rawAllowed = false
            s.meta.getJSONObject("android").put("raw_error", "camera ${st.name} has no RAW capability")
            return false
        }
        val rs = rawSizes.maxByOrNull { it.width * it.height }!!
        rawReader = ImageReader.newInstance(rs.width, rs.height, ImageFormat.RAW_SENSOR, 3).also { it.setOnImageAvailableListener({ r -> onRaw(r) }, handler) }
        rawStream = st
        return true
    }

    private fun outputs(sel: List<Stream>, preview: String, pv: Surface?, withRaw: Boolean): List<OutputConfiguration> {
        val outs = sel.mapIndexed { i, st ->
            OutputConfiguration(st.enc.inputSurface!!).apply {
                st.physicalId?.let { setPhysicalCameraId(it) }
                if (i == 0 && preview == PREVIEW_SHARED && pv != null) { enableSurfaceSharing(); addSurface(pv) }
            }
        }.toMutableList()
        if (preview == PREVIEW_SURFACE && pv != null) outs += OutputConfiguration(pv).apply { sel.first().physicalId?.let { setPhysicalCameraId(it) } }
        if (withRaw) rawReader?.let { r -> outs += OutputConfiguration(r.surface).apply { rawStream?.physicalId?.let { setPhysicalCameraId(it) } } }
        return outs
    }

    /** true / false, or null when the HAL cannot say (then the real createCaptureSession decides). */
    private fun supported(d: CameraDevice, outs: List<OutputConfiguration>, params: CaptureRequest?): Boolean? = try {
        d.isSessionConfigurationSupported(SessionConfiguration(SessionConfiguration.SESSION_REGULAR, outs, executor,
            object : CameraCaptureSession.StateCallback() {
                override fun onConfigured(p0: CameraCaptureSession) {}
                override fun onConfigureFailed(p0: CameraCaptureSession) {}
            }).also { c -> params?.let { c.sessionParameters = it } })
    } catch (e: Exception) { Log.w(TAG, "isSessionConfigurationSupported", e); null }

    /** A configuration was accepted: record what the budget did and which preview is shown, start streaming. */
    private fun commit(d: CameraDevice, cs: CameraCaptureSession, a: Attempt, sel: List<Stream>, psz: Size?, raw: Boolean) {
        session = cs
        streams.clear(); streams += sel
        previewKind = a.preview
        previewSize = psz
        budgetActions = a.step.actions
        if (!raw && rawReader != null) {
            rawReader?.close(); rawReader = null; rawStream = null
            s.meta.getJSONObject("android").put("raw_error", "RAW stream not supported together with ${sel.size} video streams")
        }
        for (st in candidates) if (st !in sel) { st.encoder?.discard(); st.encoder = null }

        val streamsJson = JSONArray(sel.map { st ->
            JSONObject().apply { put("name", st.name); put("physical_id", st.physicalId ?: JSONObject.NULL); put("w", st.size.width); put("h", st.size.height)
                put("fps", st.fps); put("file", "cams/${st.name}.mp4"); put("codec", st.enc.mime) }
        })
        multicamMeta().apply {
            put("streams", streamsJson)
            put("budget_actions", JSONArray(budgetActions))
            put("budget_attempts", attemptIdx)
            put("preview_kind", previewKind)
            put("preview_size", psz?.let { JSONArray(listOf(it.width, it.height)) } ?: JSONObject.NULL)
            put("ae_target_fps_range", aeRange?.let { JSONArray(listOf(it.lower, it.upper)) } ?: JSONObject.NULL)
        }
        // Same shape as the iOS app's session.json "multicam" block.
        s.meta.put("multicam", JSONObject().apply {
            put("budget_actions", JSONArray(budgetActions))
            put("preview", previewKind)
            put("preview_kind", previewKind)
            put("streams", JSONArray(sel.map { st -> JSONObject().apply { put("name", st.name); put("width", st.size.width); put("height", st.size.height); put("fps", st.fps) } }))
        })
        writeCalibration()

        val sensorRot = sel.first().chars.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 90
        val shown = psz ?: sel.first().size
        ui.post { if (sensorRot % 180 != 0) previewView.setContentSize(shown.height, shown.width) else previewView.setContentSize(shown.width, shown.height) }
        status("multicam: ${sel.joinToString { "${it.name} ${it.size}@${it.fps}" }}" + (if (rawReader != null) " +RAW" else "") +
            "\npreview: $previewKind" + budgetText())
        try { startRepeating(d, cs, locked = false) } catch (e: Exception) {
            Log.e(TAG, "setRepeatingRequest", e)
            multicamMeta().put("configure_error", "repeating request failed: $e")
            status("multicam: camera refused the repeating request: $e")
        }
    }

    private fun budgetText(): String = if (budgetActions.isEmpty()) "" else "\nbudget: " + budgetActions.joinToString(", ")

    private fun baseRequest(d: CameraDevice, locked: Boolean, last: TotalCaptureResult?): CaptureRequest.Builder {
        val rb = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
        streams.forEach { rb.addTarget(it.enc.inputSurface!!) }
        if (previewKind != PREVIEW_NONE) previewSurface?.let { rb.addTarget(it) }
        aeRange?.let { rb.set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, it) }
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
            // Never let metadata or a muxer hiccup take the camera thread (and the recording) down.
            try {
                val phys: Map<String, CaptureResult> = if (Build.VERSION.SDK_INT >= 31) res.physicalCameraTotalResults else @Suppress("DEPRECATION") res.physicalCameraResults
                for (st in streams) {
                    val r: CaptureResult = st.physicalId?.let { phys[it] } ?: res
                    val t = r.get(CaptureResult.SENSOR_TIMESTAMP) ?: res.get(CaptureResult.SENSOR_TIMESTAMP) ?: continue
                    val o = JSONObject().put("t", t / 1e9)
                    CameraInfo.resultFields(r, o)
                    st.results[t] = o
                    try { st.enc.poll() } catch (e: Exception) { Log.w(TAG, "encoder ${st.name}", e) }
                }
                if (req.tag == RAW_TAG) res.get(CaptureResult.SENSOR_TIMESTAMP)?.let { t -> pendingRaw[t] = res }
            } catch (e: Exception) { Log.w(TAG, "capture result", e) }
            if (frames % 30 == 0L) status("multicam ${streams.size} cams f=$frames raw=$rawCount tof=$depthCount drop=${s.droppedWrites}\npreview: $previewKind" + budgetText())
        }
    }

    private fun startRepeating(d: CameraDevice, cs: CameraCaptureSession, locked: Boolean) {
        running = true
        cs.setRepeatingRequest(baseRequest(d, locked, lastResult).build(), captureCb, handler)
        if (opt.lock && !locked) handler.postDelayed({ if (running) try { startRepeating(d, cs, locked = true) } catch (e: Exception) { Log.w(TAG, "lock", e) } }, 1500)
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
                s.csv(jsonl, "", JsonSafe.stringify(o))
                depthCount++
            } catch (e: Exception) { Log.w(TAG, "tof frame", e) } finally { img.close() }
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
    private fun writeCalibration() {
        val lc = logicalChars ?: return
        val o = JSONObject()
        for (st in candidates) {
            val c = st.chars
            val aa = c.get(CameraCharacteristics.SENSOR_INFO_PRE_CORRECTION_ACTIVE_ARRAY_SIZE)
            o.put(st.name, JSONObject().apply {
                put("camera_id", st.physicalId ?: st.camId)
                put("stream_wh", JSONArray(listOf(st.size.width, st.size.height)))
                put("streamed", st in streams)
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
        s.text("cams/calibration.json", JsonSafe.stringify(o, 1))
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

    /** Per-lens progress for the live coverage view. */
    fun liveStatus(): String =
        streams.joinToString("\n") { st -> "${st.name} ${st.size.width}x${st.size.height}@${st.fps}: ${st.encoder?.encoded ?: 0} frames" } +
            "\nRAW DNG $rawCount   ToF depth $depthCount   capture results $frames" +
            "\npreview: $previewKind" + (if (budgetActions.isEmpty()) "" else "   budget: " + budgetActions.joinToString(", "))

    // ------------------------------------------------------------------ stop
    fun stop() {
        running = false
        stopped = true
        val done = java.util.concurrent.CountDownLatch(1)
        handler.post {
            try { session?.stopRepeating(); session?.abortCaptures() } catch (_: Exception) {}
            // Encoders made for configuration attempts that were never used.
            for (st in candidates) if (st !in streams) { st.encoder?.discard(); st.encoder = null }
            done.countDown()
        }
        done.await(2, java.util.concurrent.TimeUnit.SECONDS)
        Thread.sleep(200)
        for (st in streams) try { st.enc.stop() } catch (e: Exception) { Log.w(TAG, "encoder stop ${st.name}", e) }
        try { session?.close() } catch (_: Exception) {}
        device?.close(); depthDevice?.close()
        rawReader?.close(); depthReader?.close()
        try { depthBlob?.close() } catch (_: Exception) {}
        previewSurface?.release(); previewSurface = null
        // Join per-frame results to video samples -> cams/<name>.jsonl (FORMAT.md mode C).
        var nTotal = 0
        for (st in streams) {
            try {
                val pts = File(st.enc.file.path + ".pts.csv")
                val K = kForStream(st.chars, st.size)
                val sb = StringBuilder()
                val keys = st.results.keys.sorted().toLongArray()
                pts.readLines().drop(1).forEach { line ->
                    val parts = line.split(","); val i = parts[0].toInt(); val tNs = parts[2].toLong()
                    // pts are us -> match the result with the nearest sensor timestamp
                    var idx = java.util.Arrays.binarySearch(keys, tNs).let { if (it < 0) -it - 1 else it }
                    if (idx > 0 && (idx >= keys.size || kotlin.math.abs(keys[idx - 1] - tNs) < kotlin.math.abs(keys[idx] - tNs))) idx--
                    val r = if (keys.isNotEmpty() && kotlin.math.abs(keys[idx.coerceIn(0, keys.size - 1)] - tNs) < 2_000_000) st.results[keys[idx.coerceIn(0, keys.size - 1)]] else null
                    val o = JSONObject(r?.let { JsonSafe.stringify(it) } ?: "{}")
                    o.put("i", i); o.put("t", r?.optDouble("t") ?: (tNs / 1e9)); o.put("w", st.size.width); o.put("h", st.size.height)
                    o.put("K", K ?: JSONObject.NULL)
                    o.optLong("exposure_ns", -1).takeIf { it > 0 }?.let { o.put("exp", it / 1e9) }
                    o.opt("iso")?.let { o.put("iso", it) }
                    sb.append(JsonSafe.stringify(o)).append('\n')
                    nTotal++
                }
                s.text("cams/${st.name}.jsonl", sb.toString())
            } catch (e: Exception) { Log.w(TAG, "jsonl ${st.name}", e) }
        }
        s.meta.put("counts", JSONObject().apply {
            put("frames", 0); put("capture_results", frames); put("cam_samples", nTotal); put("raw_dng", rawCount); put("tof_depth", depthCount)
            for (st in streams) put("cam_${st.name}", st.encoder?.encoded ?: 0)
        })
        s.meta.put("settings", JSONObject().apply {
            put("raw_dng", opt.rawDng); put("raw_period_ms", opt.rawPeriodMs); put("lock", opt.lock); put("ois_off", opt.oisOff)
            put("eis", "off"); put("distortion_correction", "off"); put("requested_max_width", opt.maxWidth); put("requested_fps", opt.fps)
        })
        s.meta.put("notes", "Camera2 multicam: no on-device poses (frames.jsonl empty). Recover with SfM (tools/recover_poses.py); metric scale from ToF depth or IMU.")
        thread.quitSafely()
    }

    companion object {
        const val TAG = "Camera2Recorder"
        const val RAW_TAG = "raw"
        /** Extra preview stream on the main lens. */
        const val PREVIEW_SURFACE = "preview_surface"
        /** Preview fed by the recorded main stream (surface sharing), no extra camera stream. */
        const val PREVIEW_SHARED = "recorded_stream"
        const val PREVIEW_NONE = "none"
    }
}
