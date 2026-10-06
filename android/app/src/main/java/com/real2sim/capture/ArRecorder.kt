package com.real2sim.capture

import android.annotation.SuppressLint
import android.app.Activity
import android.content.Context
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.media.Image
import android.media.ImageReader
import android.net.Uri
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Handler
import android.os.HandlerThread
import android.os.SystemClock
import android.util.Log
import android.util.Size
import android.view.Surface
import android.view.WindowManager
import com.google.ar.core.CameraConfig
import com.google.ar.core.CameraConfigFilter
import com.google.ar.core.Config
import com.google.ar.core.Earth
import com.google.ar.core.Frame
import com.google.ar.core.ImageMetadata
import com.google.ar.core.LightEstimate
import com.google.ar.core.Plane
import com.google.ar.core.RecordingConfig
import com.google.ar.core.Session
import com.google.ar.core.SharedCamera
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.File
import java.util.EnumSet
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

data class ArOptions(
    val hiResStills: Boolean,      // SharedCamera + extra full-res YUV stream, JPEG at stillPeriodMs
    val stillPeriodMs: Long = 500,
    val arcoreMp4: Boolean,        // ARCore Recording API (replayable dataset) alongside our own files
    val lockFocus: Boolean,
    val geospatial: Boolean,
)

/**
 * Mode A: ARCore. Per frame: sensor-oriented + display-oriented camera pose, image and texture
 * intrinsics, tracking state, light estimate (HDR), Camera2 metadata ARCore forwards, CPU image
 * into an HEVC video, smoothed + raw depth + raw confidence when new, point cloud and planes
 * periodically, Earth/Geospatial pose when available. Optional: SharedCamera full-res stills
 * captured in the SAME request as an ARCore frame (identical sensor timestamp => exact pose),
 * and an ARCore Recording API mp4.
 */
class ArRecorder(
    private val activity: Activity,
    private val s: SessionWriter,
    private val opt: ArOptions,
    private val status: (String) -> Unit,
) : GLSurfaceView.Renderer {
    val view = GLSurfaceView(activity)
    private lateinit var session: Session
    private val bg = BackgroundRenderer()
    private var encoder: VideoEncoder? = null
    private var frameIdx = 0          // every ARCore frame (extras/arcore_frames.jsonl)
    private var videoIdx = 0          // frames that made it into video.mp4 (frames.jsonl "i")
    private lateinit var depthBlob: SessionWriter.Blob
    private lateinit var confBlob: SessionWriter.Blob
    private lateinit var conf255Blob: SessionWriter.Blob
    private lateinit var sdepthBlob: SessionWriter.Blob
    /** frames.jsonl lines held back a few frames so a late depth map can still be attached. */
    private val pending = ArrayDeque<Pair<Long, JSONObject>>()
    /** Pose + CPU-image intrinsics of one ARCore frame (compact; the JSON is built when a still is written). */
    private class PoseRec(val m: FloatArray /* column-major camera-to-world */, val K: FloatArray /* fx, fy, cx, cy */, val w: Int, val h: Int, val track: String)
    /**
     * Hi-res stills only: t_ns -> pose of the last ~2 s of frames, to pose the stills (same sensor timestamp).
     * Written, read and evicted on the GL thread in [record]; [writeStillJsons] flushes the rest at stop.
     */
    private val poseByT = HashMap<Long, PoseRec>()
    private val poseOrder = ArrayDeque<Long>()
    @Volatile private var stillW = 0
    @Volatile private var stillH = 0
    private var depthDims: Pair<Int, Int>? = null
    private var lastDepthTs = -1L
    private var lastRawDepthTs = -1L
    private var lastPcTs = -1L
    private var depthCount = 0
    private var rawDepthCount = 0
    private var stillCount = 0
    private var trackedFrames = 0
    @Volatile private var recording = false
    private var viewW = 1
    private var viewH = 1

    // Live view: map of what has been captured (3D points over the camera image + top-down map).
    val map = MapBuilder()
    /** Filter -> occupancy + TSDF fusion of the depth maps, off the GL thread. */
    val fusion = DepthFusion(map)
    private val cloud = PointCloudRenderer(map) { fusion.liveMesh }
    private val depthOverlay = DepthOverlayRenderer { fusion.liveDepth }
    /** Live view: 0 = mesh, 1 = mesh + filtered depth, 2 = filtered depth, 3 = camera only. */
    @Volatile var liveView = 0
    fun cycleLiveView() { liveView = (liveView + 1) % 4 }
    private var latestRgb: IntArray? = null
    private var latestRgbT = 0L
    private var latestRgbW = 0
    private var latestRgbH = 0
    private var lastRawSubmitT = 0L
    private val viewM = FloatArray(16)
    private val projM = FloatArray(16)
    private var lastK = floatArrayOf(0f, 0f, 0f, 0f, 0f, 0f)   // fx, fy, cx, cy, w, h of the CPU image
    /** fx, fy, cx, cy, w, h of the camera texture: the field of view ARCore depth maps cover. */
    private var lastKTex = floatArrayOf(0f, 0f, 0f, 0f, 0f, 0f)
    private var lastPoseM = FloatArray(16)
    private var lastHeading = 0f
    @Volatile var showCloud = true
        set(v) { field = v; liveView = if (v) 0 else 3 }

    // Shared camera (hi-res stills)
    private var shared: SharedCamera? = null
    private var camThread: HandlerThread? = null
    private var camHandler: Handler? = null
    private var camera: CameraDevice? = null
    private var captureSession: CameraCaptureSession? = null
    private var stillReader: ImageReader? = null
    private var stillTicker: Runnable? = null
    private val stillIndex = java.util.concurrent.ConcurrentLinkedQueue<Pair<Long, String>>()
    @Volatile private var arActive = false

    fun create(): String? {
        val features = if (opt.hiResStills) EnumSet.of(Session.Feature.SHARED_CAMERA) else EnumSet.noneOf(Session.Feature::class.java)
        session = try { Session(activity, features) } catch (e: Exception) { return "ARCore session failed: $e" }
        // Anything failing after this point (configure, encoder, blobs) must not leak the Session.
        return try { setup(); null } catch (e: Exception) { release(); "ARCore setup failed: $e" }
    }

    private fun setup() {
        // ---- choose the camera config: largest CPU image, 30 fps, prefer a hardware depth sensor
        val filter = CameraConfigFilter(session).setFacingDirection(CameraConfig.FacingDirection.BACK)
        val all = session.getSupportedCameraConfigs(filter)
        val configs = JSONArray()
        for (c in all) configs.put(JSONObject().apply {
            put("camera_id", c.cameraId); put("image_size", JSONArray(listOf(c.imageSize.width, c.imageSize.height)))
            put("texture_size", JSONArray(listOf(c.textureSize.width, c.textureSize.height)))
            put("fps_range", JSONArray(listOf(c.fpsRange.lower, c.fpsRange.upper)))
            put("depth_sensor_usage", c.depthSensorUsage.name); put("stereo_camera_usage", c.stereoCameraUsage.name)
        })
        s.text("extras/arcore_camera_configs.json", configs.toString(2))
        s.meta.getJSONObject("android").put("arcore_config_count", all.size)
        val best = all.sortedWith(compareByDescending<CameraConfig> { it.depthSensorUsage == CameraConfig.DepthSensorUsage.REQUIRE_AND_USE }
            .thenByDescending { it.imageSize.width * 3 == it.imageSize.height * 4 }   // 4:3 = full sensor FOV, matches 256x192
            .thenByDescending { it.imageSize.width * it.imageSize.height }
            .thenByDescending { it.textureSize.width * it.textureSize.height }
            .thenByDescending { it.fpsRange.upper }).firstOrNull()
        if (best != null) session.cameraConfig = best

        val cfg = Config(session).apply {
            updateMode = Config.UpdateMode.BLOCKING
            focusMode = if (opt.lockFocus) Config.FocusMode.FIXED else Config.FocusMode.AUTO
            planeFindingMode = Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
            lightEstimationMode = Config.LightEstimationMode.ENVIRONMENTAL_HDR
            depthMode = when {
                session.isDepthModeSupported(Config.DepthMode.AUTOMATIC) -> Config.DepthMode.AUTOMATIC
                session.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY) -> Config.DepthMode.RAW_DEPTH_ONLY
                else -> Config.DepthMode.DISABLED
            }
            if (opt.geospatial && BuildConfig.HAS_ARCORE_API_KEY && session.isGeospatialModeSupported(Config.GeospatialMode.ENABLED))
                geospatialMode = Config.GeospatialMode.ENABLED
        }
        try { session.configure(cfg) } catch (e: Exception) {
            cfg.geospatialMode = Config.GeospatialMode.DISABLED
            session.configure(cfg)
            s.meta.getJSONObject("android").put("geospatial_error", e.toString())
        }

        val cc = session.cameraConfig
        val cm = activity.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        val chars = cm.getCameraCharacteristics(cc.cameraId)
        s.meta.getJSONObject("android").put("arcore", JSONObject().apply {
            put("sdk", "com.google.ar:core:1.56.0")
            put("camera_id", cc.cameraId)
            put("image_size", JSONArray(listOf(cc.imageSize.width, cc.imageSize.height)))
            put("texture_size", JSONArray(listOf(cc.textureSize.width, cc.textureSize.height)))
            put("fps_range", JSONArray(listOf(cc.fpsRange.lower, cc.fpsRange.upper)))
            put("depth_sensor_usage", cc.depthSensorUsage.name)
            put("depth_mode", cfg.depthMode.name)
            put("focus_mode", cfg.focusMode.name)
            put("light_estimation", cfg.lightEstimationMode.name)
            put("geospatial", cfg.geospatialMode.name)
            put("shared_camera", opt.hiResStills)
            put("timestamp_source", if (chars.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME) "REALTIME" else "UNKNOWN")
        })
        s.text("extras/camera_characteristics_arcore_${cc.cameraId}.json", CameraInfo.characteristics(chars).toString(1))
        val br = bitrateFor(cc.imageSize.width, cc.imageSize.height)
        encoder = VideoEncoder(s.file("video.mp4"), cc.imageSize.width, cc.imageSize.height, cc.fpsRange.upper.coerceAtLeast(30), br, surfaceInput = false)
        s.meta.put("video", JSONObject().apply {
            put("file", "video.mp4"); put("width", cc.imageSize.width); put("height", cc.imageSize.height)
            put("fps", cc.fpsRange.upper); put("codec", if (encoder!!.mime.endsWith("hevc")) "hevc" else "h264"); put("bitrate", br)
            put("source", "ARCore CPU image (YUV_420_888), sensor orientation"); put("pts_file", "video.mp4.pts.csv")
        })
        depthBlob = s.Blob("depth.zlib.bin"); confBlob = s.Blob("conf.zlib.bin")
        sdepthBlob = s.Blob("depth_smooth.zlib.bin"); conf255Blob = s.Blob("extras/conf255.zlib.bin")

        if (opt.arcoreMp4) {
            try {
                val f = s.file("extras/arcore_recording.mp4")
                session.startRecording(RecordingConfig(session).setMp4DatasetUri(Uri.fromFile(f)).setAutoStopOnPause(true))
                s.meta.getJSONObject("android").put("arcore_recording", "extras/arcore_recording.mp4")
            } catch (e: Exception) { s.meta.getJSONObject("android").put("arcore_recording_error", e.toString()) }
        }

        view.preserveEGLContextOnPause = true
        view.setEGLContextClientVersion(3)
        view.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
        view.setRenderer(this)
        view.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
    }

    private fun bitrateFor(w: Int, h: Int) = (w * h * 30 * 0.25).toInt().coerceIn(8_000_000, 120_000_000)

    fun start() {
        // SharedCamera: the camera is opened from onSurfaceCreated, once ARCore has its texture.
        if (!opt.hiResStills) session.resume()      // may throw (CameraNotAvailableException): the caller releases
        recording = true
        view.onResume()
    }

    /**
     * Tears down a recorder whose [create] or [start] failed (or that never started): nothing is written, the
     * GL thread is stopped, the encoder and blob files are discarded and the Session is closed. Safe after a
     * partial [create].
     */
    fun release() {
        recording = false
        try { view.onPause() } catch (_: Exception) {}           // stop the GL thread before touching the session
        stillTicker?.let { camHandler?.removeCallbacks(it) }
        try { captureSession?.close() } catch (_: Exception) {}
        try { camera?.close() } catch (_: Exception) {}
        try { stillReader?.close() } catch (_: Exception) {}
        camThread?.quitSafely()
        encoder?.discard(); encoder = null
        for (b in listOfNotNull(
            if (::depthBlob.isInitialized) depthBlob else null, if (::confBlob.isInitialized) confBlob else null,
            if (::sdepthBlob.isInitialized) sdepthBlob else null, if (::conf255Blob.isInitialized) conf255Blob else null)) {
            try { b.close() } catch (_: Exception) {}
        }
        try { fusion.close() } catch (_: Exception) {}
        if (::session.isInitialized) {
            try { if (opt.arcoreMp4) session.stopRecording() } catch (_: Exception) {}
            try { session.pause() } catch (_: Exception) {}
            try { session.close() } catch (_: Exception) {}
        }
    }

    fun stop() {
        recording = false
        view.queueEvent { }
        view.onPause()
        stillTicker?.let { camHandler?.removeCallbacks(it) }
        try { if (opt.arcoreMp4) session.stopRecording() } catch (_: Exception) {}
        session.pause()
        try { captureSession?.close() } catch (_: Exception) {}
        camera?.close()
        stillReader?.close()
        camThread?.quitSafely()
        encoder?.stop()
        while (pending.isNotEmpty()) s.frame(pending.removeFirst().second)
        listOf(depthBlob, confBlob, sdepthBlob, conf255Blob).forEach { it.close() }
        writeStillJsons()
        depthDims?.let { (w, h) ->
            s.meta.put("depth", JSONObject().apply {
                put("width", w); put("height", h); put("unit", "mm"); put("dtype", "uint16"); put("compression", "raw-deflate")
                put("source", "ARCore raw depth (acquireRawDepthImage16Bits); depth_smooth = acquireDepthImage16Bits")
                put("confidence", "conf.zlib.bin = ARCore raw confidence quantised 0..84->0, 85..169->1, 170..255->2; full 0..255 in extras/conf255.zlib.bin (range 'c255' in extras/arcore_frames.jsonl)")
                put("dK_note", "ARCore depth covers the field of view of the camera texture (K_tex in extras/arcore_frames.jsonl), " +
                    "not necessarily of the CPU image: each frames.jsonl line with depth carries dK = [fx, fy, cx, cy] of the " +
                    "depth map at dw x dh (texture intrinsics scaled). Prefer dK over K * dw / w; map depth to RGB pixels through dK and K.")
            })
        }
        s.meta.put("counts", JSONObject().apply {
            put("frames", videoIdx); put("arcore_frames", frameIdx); put("repeated_updates_skipped", repeatedUpdates); put("tracked_frames", trackedFrames); put("depth", rawDepthCount)
            put("depth_smooth", depthCount); put("hires", stillCount); put("video_samples", encoder?.encoded ?: 0)
        })
        s.meta.put("settings", JSONObject().apply {
            put("hires_stills", opt.hiResStills); put("still_period_ms", opt.stillPeriodMs); put("arcore_mp4", opt.arcoreMp4)
            put("lock", opt.lockFocus); put("geospatial", opt.geospatial)
        })
        dumpPlanesFinal()
        // final TSDF mesh + cleaned points (extras/recon/), settings + stats in session.json
        var clean: FloatArray? = null
        try {
            val r = fusion.finish(map.floorY)
            if (r != null) {
                fusion.writeOutputs(r, s.file("extras/recon/recon.json").parentFile!!)
                clean = r.points
            }
            s.meta.put("depth_processing", fusion.describe(r).apply {
                put("outputs_dir", "extras/recon/")
                put("note", "depth filtered (confidence, range, edge-preserving median, flying pixels, grazing angle, temporal) " +
                    "before occupancy and TSDF fusion; extras/map/points.ply = cleaned TSDF surface points")
            })
        } catch (e: Exception) { Log.w(TAG, "recon", e) }
        try {
            val extra = JSONObject().put("session", s.id).put("source", "mode A capture")
            map.save(s.file("extras/map/map.json").parentFile!!, extra, clean)
            map.save(File(SessionWriter.sessionsRoot(activity).parentFile, "maps/${s.id}"), extra, clean)
        } catch (e: Exception) { Log.w(TAG, "map save", e) }
        session.close()
    }

    // ------------------------------------------------------------------ GL renderer
    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0f, 0f, 0f, 1f)
        bg.create()
        cloud.create()
        depthOverlay.create()
        session.setCameraTextureName(bg.textureId)
        if (opt.hiResStills && shared == null) activity.runOnUiThread { try { openSharedCamera() } catch (e: Exception) { status("shared camera failed: $e") } }
    }

    override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
        viewW = width; viewH = height
        GLES20.glViewport(0, 0, width, height)
        @Suppress("DEPRECATION")
        val rot = (activity.getSystemService(Context.WINDOW_SERVICE) as WindowManager).defaultDisplay.rotation
        session.setDisplayGeometry(rot, width, height)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        if (!recording || (opt.hiResStills && !arActive)) return
        val frame = try { session.update() } catch (e: Exception) { Log.w(TAG, "update", e); return }
        bg.draw(frame)
        try { record(frame) } catch (e: Exception) { Log.e(TAG, "record", e) }
        if (frame.camera.trackingState == TrackingState.TRACKING) {
            frame.camera.getViewMatrix(viewM, 0)
            frame.camera.getProjectionMatrix(projM, 0, 0.05f, 50f)
            val lv = liveView
            if (lv == 1 || lv == 2) depthOverlay.draw(frame)
            cloud.visible = lv <= 1
            cloud.draw(viewM, projM, null, null)
        }
    }

    private var lastFrameTs = -1L
    private var repeatedUpdates = 0

    private fun record(frame: Frame) {
        // BLOCKING update can hand back the same camera frame again (and 0 before the first one):
        // log each camera frame once.
        if (frame.timestamp == 0L || frame.timestamp == lastFrameTs) { repeatedUpdates++; return }
        lastFrameTs = frame.timestamp
        val cam = frame.camera
        val o = JSONObject()          // rich per-frame record -> extras/arcore_frames.jsonl
        o.put("n", frameIdx)
        o.put("t_arcore_ns", frame.timestamp)
        o.put("t_update", SystemClock.elapsedRealtimeNanos() / 1e9)

        // t = ARCore frame timestamp: the time the pose refers to and the timestamp depth maps carry.
        // On devices it is the camera sensor timestamp (CLOCK_BOOTTIME with REALTIME timestamp source);
        // the CPU image's own timestamp is logged as t_image for checking.
        val tNs = frame.timestamp
        var inVideo = false
        try {
            frame.acquireCameraImage().use { img ->
                o.put("t_image", img.timestamp / 1e9)
                inVideo = encoder?.encodeYuv(img, tNs) == true
                // small RGB at depth resolution for colouring the TSDF (~3 times a second is plenty): the depth map
                // covers the texture FOV, so each depth pixel is mapped to its CPU-image pixel through both
                // intrinsics (an affine crop/resample, computed once per frame) instead of resampling the whole image
                if (frameIdx % 10 == 0) try {
                    val (dw, dh) = depthDims ?: Pair(160, 120)
                    val y = img.planes[0]; val u = img.planes[1]; val v = img.planes[2]
                    val kt = lastKTex; val ki = lastK
                    latestRgb = if (kt[4] > 0f && ki[4] > 0f) {
                        val dK = DepthIntrinsics.scale(kt, kt[4].toInt(), kt[5].toInt(), dw, dh)
                        val imgK = DepthIntrinsics.scale(ki, ki[4].toInt(), ki[5].toInt(), img.width, img.height)
                        DepthIntrinsics.sampleYuv(y.buffer, y.rowStride, y.pixelStride, u.buffer, v.buffer, u.rowStride, u.pixelStride,
                            img.width, img.height, dw, dh, DepthIntrinsics.affine(dK, imgK))
                    } else YuvSampler.sample(y.buffer, y.rowStride, y.pixelStride, u.buffer, v.buffer, u.rowStride, u.pixelStride,
                        img.width, img.height, dw, dh)
                    latestRgbT = tNs; latestRgbW = dw; latestRgbH = dh
                } catch (_: Exception) {}
            }
        } catch (_: NotYetAvailableException) {
        } catch (e: Exception) { Log.w(TAG, "cpu image", e) }
        o.put("t", tNs / 1e9)

        val track = when (cam.trackingState) {
            TrackingState.TRACKING -> { trackedFrames++; "normal" }
            TrackingState.PAUSED -> "limited:" + cam.trackingFailureReason.name.lowercase()
            else -> "not_available"
        }
        o.put("track", track)
        val m = FloatArray(16)
        cam.pose.toMatrix(m, 0)
        val T = SessionWriter.mat4RowMajorFromColumnMajor(m)
        o.put("T", T)
        cam.displayOrientedPose.toMatrix(m, 0)
        o.put("T_display", SessionWriter.mat4RowMajorFromColumnMajor(m))
        val k = cam.imageIntrinsics
        val K = JSONArray(listOf(k.focalLength[0].toDouble(), k.focalLength[1].toDouble(), k.principalPoint[0].toDouble(), k.principalPoint[1].toDouble()))
        val kw = k.imageDimensions[0]; val kh = k.imageDimensions[1]
        o.put("K", K); o.put("w", kw); o.put("h", kh)
        lastK = floatArrayOf(k.focalLength[0], k.focalLength[1], k.principalPoint[0], k.principalPoint[1], kw.toFloat(), kh.toFloat())
        if (cam.trackingState == TrackingState.TRACKING) {
            cam.pose.toMatrix(lastPoseM, 0)
            lastHeading = kotlin.math.atan2(-lastPoseM[10], -lastPoseM[8])
            map.addPose(lastPoseM[12], lastPoseM[13], lastPoseM[14], tNs)
            if (frameIdx % 30 == 0) updateFloor()
        }
        cam.textureIntrinsics.let { t ->
            o.put("K_tex", JSONArray(listOf(t.focalLength[0].toDouble(), t.focalLength[1].toDouble(),
                t.principalPoint[0].toDouble(), t.principalPoint[1].toDouble(), t.imageDimensions[0], t.imageDimensions[1])))
            lastKTex = floatArrayOf(t.focalLength[0], t.focalLength[1], t.principalPoint[0], t.principalPoint[1],
                t.imageDimensions[0].toFloat(), t.imageDimensions[1].toFloat())
        }
        val proj = FloatArray(16)
        cam.getProjectionMatrix(proj, 0, 0.01f, 100f)
        o.put("P_display", SessionWriter.mat4RowMajorFromColumnMajor(proj))

        metadata(frame, o)
        light(frame.lightEstimate, o)
        if (frameIdx % 10 == 0) pointCloud(frame, o)
        if (frameIdx % 90 == 0) dumpPlanes()
        if (opt.geospatial) geo(o)
        if (opt.hiResStills) {
            poseByT[tNs] = PoseRec(FloatArray(16).also { cam.pose.toMatrix(it, 0) }, floatArrayOf(k.focalLength[0], k.focalLength[1], k.principalPoint[0], k.principalPoint[1]), kw, kh, track)
            poseOrder.addLast(tNs)
            // Stills older than 1 s have certainly had their ARCore frame: pose them now, then forget frames older than 2 s.
            while (true) {
                val st = stillIndex.peek() ?: break
                if (tNs - st.first < 1_000_000_000L) break
                stillIndex.poll()
                writeStillJson(st.first, st.second)
            }
            while (poseOrder.isNotEmpty() && tNs - poseOrder.first() > 2_000_000_000L) poseByT.remove(poseOrder.removeFirst())
        }

        if (inVideo) {
            // FORMAT.md frames.jsonl line (one per video sample, in video order)
            val line = JSONObject().apply {
                put("i", videoIdx); put("t", tNs / 1e9); put("w", kw); put("h", kh); put("K", K); put("T", T); put("track", track)
                o.optLong("exposure_ns", -1).takeIf { it > 0 }?.let { put("exp", it / 1e9) }
                if (o.has("iso")) put("iso", o.getInt("iso"))
                o.optJSONObject("light")?.let { l -> put("amb", l.getJSONArray("main_intensity").let { (it.getDouble(0) + it.getDouble(1) + it.getDouble(2)) / 3 }) }
                put("d", JSONObject.NULL); put("c", JSONObject.NULL)
            }
            o.put("i", videoIdx)
            pending.addLast(Pair(tNs, line))
            videoIdx++
        }
        depth(frame, o)
        while (pending.size > 6) s.frame(pending.removeFirst().second)

        s.csv("extras/arcore_frames.jsonl", "", o.toString())
        frameIdx++
        if (frameIdx % 15 == 0) status("AR $track f=$videoIdx/$frameIdx depth=$rawDepthCount smooth=$depthCount hires=$stillCount drop=${s.droppedWrites}")
    }

    /** The held-back frames.jsonl line whose timestamp matches a depth map (ARCore depth can lag a frame or two). */
    private fun lineFor(tNs: Long): JSONObject? = pending.lastOrNull { kotlin.math.abs(it.first - tNs) < 2_000_000 }?.second

    private fun metadata(frame: Frame, o: JSONObject) {
        val md = try { frame.imageMetadata } catch (_: Exception) { return }
        fun long(name: String, key: Int) = try { o.put(name, md.getLong(key)) } catch (_: Exception) { null }
        fun float(name: String, key: Int) = try { o.put(name, md.getFloat(key).toDouble()) } catch (_: Exception) { null }
        fun int(name: String, key: Int) = try { o.put(name, md.getInt(key)) } catch (_: Exception) { null }
        long("sensor_timestamp_ns", ImageMetadata.SENSOR_TIMESTAMP)
        long("exposure_ns", ImageMetadata.SENSOR_EXPOSURE_TIME)
        int("iso", ImageMetadata.SENSOR_SENSITIVITY)
        long("frame_duration_ns", ImageMetadata.SENSOR_FRAME_DURATION)
        long("rolling_shutter_skew_ns", ImageMetadata.SENSOR_ROLLING_SHUTTER_SKEW)
        float("focus_distance_diopters", ImageMetadata.LENS_FOCUS_DISTANCE)
        float("focal_length_mm", ImageMetadata.LENS_FOCAL_LENGTH)
        float("aperture", ImageMetadata.LENS_APERTURE)
        int("ae_state", ImageMetadata.CONTROL_AE_STATE)
        int("af_state", ImageMetadata.CONTROL_AF_STATE)
        int("awb_state", ImageMetadata.CONTROL_AWB_STATE)
    }

    private fun light(le: LightEstimate, o: JSONObject) {
        if (le.state != LightEstimate.State.VALID) return
        o.put("light", JSONObject().apply {
            put("main_dir", JSONArray(le.environmentalHdrMainLightDirection.map { it.toDouble() }))
            put("main_intensity", JSONArray(le.environmentalHdrMainLightIntensity.map { it.toDouble() }))
            put("sh", JSONArray(le.environmentalHdrAmbientSphericalHarmonics.map { it.toDouble() }))
        })
    }

    private fun depth(frame: Frame, o: JSONObject) {
        var rawSub: Triple<ShortArray, Int, Int>? = null
        var confSub: ByteArray? = null
        var smSub: Triple<ShortArray, Int, Int>? = null
        // Raw depth + per-pixel confidence (0..255) -> depth.zlib.bin / conf.zlib.bin (0..2) / extras/conf255.zlib.bin
        try {
            frame.acquireRawDepthImage16Bits().use { d ->
                if (d.timestamp != lastRawDepthTs) {
                    lastRawDepthTs = d.timestamp
                    val rawBytes = plane(d, 2)
                    val dr = depthBlob.append(rawBytes)
                    val rd = JSONObject().apply { put("t", d.timestamp / 1e9); put("d", dr); put("dw", d.width); put("dh", d.height) }
                    var cr: JSONArray? = null
                    try {
                        frame.acquireRawDepthConfidenceImage().use { c ->
                            val c255 = plane(c, 1)
                            confSub = c255
                            rd.put("c255", conf255Blob.append(c255))
                            val q = ByteArray(c255.size) { i -> val v = c255[i].toInt() and 0xff; (if (v < 85) 0 else if (v < 170) 1 else 2).toByte() }
                            cr = confBlob.append(q)
                            rd.put("c", cr)
                        }
                    } catch (_: Exception) {}
                    depthK(d.width, d.height)?.let { rd.put("dK", it) }
                    o.put("raw_depth", rd)
                    depthDims = Pair(d.width, d.height)
                    lineFor(d.timestamp)?.apply {
                        put("d", dr); put("c", cr ?: JSONObject.NULL); put("dw", d.width); put("dh", d.height)
                        depthK(d.width, d.height)?.let { put("dK", it) }
                    }
                    rawDepthCount++
                    rawSub = Triple(DepthFilter.shortsLE(rawBytes, d.width * d.height), d.width, d.height)
                }
            }
        } catch (_: NotYetAvailableException) {} catch (_: IllegalStateException) {}
        // Smoothed depth -> depth_smooth.zlib.bin ("sd"); no confidence for it.
        try {
            frame.acquireDepthImage16Bits().use { d ->
                if (d.timestamp != lastDepthTs) {
                    lastDepthTs = d.timestamp
                    val smBytes = plane(d, 2)
                    val r = sdepthBlob.append(smBytes)
                    smSub = Triple(DepthFilter.shortsLE(smBytes, d.width * d.height), d.width, d.height)
                    o.put("smooth_depth", JSONObject().apply { put("t", d.timestamp / 1e9); put("sd", r); put("w", d.width); put("h", d.height) })
                    lineFor(d.timestamp)?.apply {
                        put("sd", r)
                        if (!has("dw")) { put("dw", d.width); put("dh", d.height); depthK(d.width, d.height)?.let { put("dK", it) } }
                    }
                    depthCount++
                }
            }
        } catch (_: NotYetAvailableException) {} catch (_: IllegalStateException) {}
        // -> DepthFusion: raw depth + confidence for the TSDF, dense smoothed depth for the occupancy grid;
        // smoothed alone (TSDF too) when raw depth has not come for a second.
        if (frame.camera.trackingState != TrackingState.TRACKING) return
        val sm = smSub ?: pendingSmooth
        val raw = rawSub
        if (raw != null) {
            pendingSmooth = null
            submitDepth(frame, raw.first, confSub, raw.second, raw.third, sm?.first, sm?.second ?: 0, sm?.third ?: 0)
            lastRawSubmitT = frame.timestamp
        } else if (smSub != null) {
            if (frame.timestamp - lastRawSubmitT > 1_000_000_000L) { pendingSmooth = null; submitDepth(frame, null, null, 0, 0, sm!!.first, sm.second, sm.third) }
            else pendingSmooth = smSub
        }
    }

    /** Newest smoothed depth, attached to the next raw-depth submission (occupancy uses the dense map). */
    private var pendingSmooth: Triple<ShortArray, Int, Int>? = null

    /** Intrinsics of a [dw] x [dh] depth map (texture intrinsics scaled), as the frames.jsonl "dK"; null before the first frame. */
    private fun depthK(dw: Int, dh: Int): JSONArray? {
        val kt = lastKTex
        if (kt[4] <= 0f) return null
        val k = DepthIntrinsics.scale(kt, kt[4].toInt(), kt[5].toInt(), dw, dh)
        return JSONArray(listOf(k[0].toDouble(), k[1].toDouble(), k[2].toDouble(), k[3].toDouble()))
    }

    /**
     * Depth map(s) + pose of this frame -> DepthFusion (filter, occupancy, TSDF) on its worker thread. The
     * texture intrinsics go with the texture size: DepthFusion scales them to each depth map itself.
     */
    private fun submitDepth(frame: Frame, raw: ShortArray?, conf: ByteArray?, rw: Int, rh: Int, smooth: ShortArray?, sw: Int, sh: Int) {
        val k = lastKTex
        if (k[4] <= 0f) return
        val m = FloatArray(16)
        frame.camera.pose.toMatrix(m, 0)
        val rgb = latestRgb?.takeIf { kotlin.math.abs(frame.timestamp - latestRgbT) < 400_000_000L }
        fusion.submit(DepthFusion.Frame(raw, conf, rw, rh, smooth, sw, sh,
            floatArrayOf(k[0], k[1], k[2], k[3]), k[4].toInt(), k[5].toInt(), m, frame.timestamp,
            rgb, if (rgb != null) latestRgbW else 0, if (rgb != null) latestRgbH else 0))
    }

    /** Floor height = lowest large upward-facing plane; else 1.4 m below the first tracked camera position. */
    private fun updateFloor() {
        var best = Float.NaN
        for (p in session.getAllTrackables(Plane::class.java)) {
            if (p.trackingState != TrackingState.TRACKING || p.type != Plane.Type.HORIZONTAL_UPWARD_FACING || p.subsumedBy != null) continue
            if (p.extentX * p.extentZ < 0.3f) continue
            val y = p.centerPose.ty()
            if (best.isNaN() || y < best) best = y
        }
        if (!best.isNaN()) map.floorY = best
        else if (map.floorY.isNaN() && map.traj.isNotEmpty()) map.floorY = map.traj[0][1] - 1.4f
    }

    fun heading() = lastHeading

    /** Snapshot for the UI thread: stats lines. */
    fun liveStats(): List<String> = listOf(
        "frames $videoIdx  depth $depthCount  tracked $trackedFrames",
        "map %.1f m2  walked %.1f m".format(map.knownAreaM2(), map.travelled),
        "mesh %d tris  blocks %d  fused %d  %.1f ms/frame".format(fusion.liveMesh?.triangles ?: 0, fusion.tsdf.blockCount, fusion.fused, fusion.integrateMs + fusion.filterMs),
        "floor %s".format(if (map.floorY.isNaN()) "?" else "%.2f m".format(map.floorY)),
    )

    private fun plane(img: Image, bpp: Int): ByteArray {
        val p = img.planes[0]
        return SessionWriter.packPlane(p.buffer, img.width, img.height, p.rowStride, bpp, p.pixelStride.coerceAtLeast(bpp))
    }

    private fun pointCloud(frame: Frame, o: JSONObject) {
        frame.acquirePointCloud().use { pc ->
            if (pc.timestamp == lastPcTs) return
            lastPcTs = pc.timestamp
            val pts = pc.points; val ids = pc.ids
            val n = pts.remaining() / 4
            if (n == 0) return
            val buf = java.nio.ByteBuffer.allocate(n * 20).order(java.nio.ByteOrder.LITTLE_ENDIAN)
            for (i in 0 until n) {
                buf.putFloat(pts.get(i * 4)); buf.putFloat(pts.get(i * 4 + 1)); buf.putFloat(pts.get(i * 4 + 2)); buf.putFloat(pts.get(i * 4 + 3))
                buf.putInt(ids.get(i))
            }
            val rel = "extras/pointcloud/%06d.bin".format(frameIdx)
            s.deflateFile(rel, buf.array())
            o.put("pointcloud", JSONObject().apply { put("file", rel); put("n", n); put("layout", "raw-deflate; per point x,y,z,confidence float32 + id int32 LE; ARCore world") })
        }
    }

    private fun planesJson(): JSONArray {
        val arr = JSONArray()
        for (p in session.getAllTrackables(Plane::class.java)) {
            if (p.trackingState == TrackingState.STOPPED || p.subsumedBy != null) continue
            val m = FloatArray(16); p.centerPose.toMatrix(m, 0)
            val poly = p.polygon; val boundary = JSONArray()
            // ARCore polygon is (x, z) pairs in the plane's local frame; FORMAT.md wants [x, y, z].
            while (poly.remaining() >= 2) { val x = poly.get().toDouble(); val z = poly.get().toDouble(); boundary.put(JSONArray(listOf(x, 0.0, z))) }
            arr.put(JSONObject().apply {
                put("id", System.identityHashCode(p).toString())
                put("alignment", if (p.type == Plane.Type.VERTICAL) "vertical" else "horizontal")
                put("classification", p.type.name.lowercase())
                put("T", SessionWriter.mat4RowMajorFromColumnMajor(m))
                put("extent", JSONArray(listOf(p.extentX.toDouble(), p.extentZ.toDouble())))
                put("boundary", boundary)
            })
        }
        return arr
    }

    private fun dumpPlanes() {
        val line = JSONObject().apply { put("t", SystemClock.elapsedRealtimeNanos() / 1e9); put("planes", planesJson()) }
        s.csv("extras/planes_timeline.jsonl", "", line.toString())
    }

    private fun dumpPlanesFinal() = s.text("planes.json", planesJson().toString(1))

    /** hires/<still>.json: the still's pose + intrinsics from the ARCore frame with the same timestamp (T null if none). */
    private fun writeStillJson(t: Long, rel: String) {
        val o = JSONObject().put("t", t / 1e9).put("file", rel)
        poseByT[t]?.let { p ->
            val sw = stillW; val sh = stillH
            val sc = if (p.w > 0 && sw > 0) sw.toDouble() / p.w else 1.0
            o.put("w", sw).put("h", sh).put("T", SessionWriter.mat4RowMajorFromColumnMajor(p.m)).put("track", p.track)
            o.put("K", JSONArray((0 until 4).map { p.K[it].toDouble() * sc }))
            o.put("K_note", "ARCore CPU-image K scaled by width ratio; valid when the still has the same aspect/crop")
        } ?: o.put("T", JSONObject.NULL)
        s.text(rel.removeSuffix(".jpg") + ".json", o.toString(1))
    }

    /** At stop (GL thread already paused): the stills of the last second that [record] has not posed yet. */
    private fun writeStillJsons() {
        while (true) { val st = stillIndex.poll() ?: break; writeStillJson(st.first, st.second) }
    }

    private fun geo(o: JSONObject) {
        val earth = session.earth ?: return
        o.put("earth_state", earth.earthState.name)
        if (earth.trackingState != TrackingState.TRACKING) return
        val g = earth.cameraGeospatialPose
        o.put("geo", JSONObject().apply {
            put("lat", g.latitude); put("lon", g.longitude); put("alt", g.altitude)
            put("eus_quat", JSONArray(g.eastUpSouthQuaternion.map { it.toDouble() }))
            put("h_acc", g.horizontalAccuracy); put("v_acc", g.verticalAccuracy); put("yaw_acc", g.orientationYawAccuracy)
        })
    }

    // ------------------------------------------------------------------ SharedCamera hi-res stills
    @SuppressLint("MissingPermission")
    private fun openSharedCamera() {
        val sc = session.sharedCamera
        shared = sc
        camThread = HandlerThread("ar-camera").apply { start() }
        camHandler = Handler(camThread!!.looper)
        val camId = session.cameraConfig.cameraId
        val cm = activity.getSystemService(Context.CAMERA_SERVICE) as CameraManager
        val chars = cm.getCameraCharacteristics(camId)
        val map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)!!
        // Largest 4:3 YUV size the camera offers in default (binned) mode, capped at 4096 px wide.
        val sizes = map.getOutputSizes(ImageFormat.YUV_420_888).sortedByDescending { it.width * it.height }
        val size: Size = sizes.firstOrNull { it.width <= 4096 && it.width * 3 == it.height * 4 } ?: sizes.first()
        val reader = ImageReader.newInstance(size.width, size.height, ImageFormat.YUV_420_888, 3)
        stillReader = reader
        reader.setOnImageAvailableListener({ r ->
            val img = r.acquireLatestImage() ?: return@setOnImageAvailableListener
            val ts = img.timestamp
            val nv21 = yuvToNv21(img)
            val w = img.width; val h = img.height
            img.close()
            stillCount++
            val rel = "hires/hires_%.6f.jpg".format(java.util.Locale.US, ts / 1e9)
            stillIndex.add(Pair(ts, rel))
            s.submit {
                val bos = ByteArrayOutputStream()
                YuvImage(nv21, ImageFormat.NV21, w, h, null).compressToJpeg(Rect(0, 0, w, h), 95, bos)
                s.file(rel).writeBytes(bos.toByteArray())
            }

        }, camHandler)
        sc.setAppSurfaces(camId, listOf(reader.surface))
        stillW = size.width; stillH = size.height
        s.meta.getJSONObject("android").put("stills", JSONObject().apply {
            put("source", "SharedCamera YUV ${size.width}x${size.height}, same capture request as an ARCore frame -> hires/hires_<t>.jpg + .json")
            put("period_ms", opt.stillPeriodMs)
            put("width", size.width); put("height", size.height)
            put("intrinsics_note", "scale ARCore image intrinsics by still/cpu size (same sensor crop when aspect matches); LENS_INTRINSIC_CALIBRATION in extras is the reference")
        })

        val devCb = object : CameraDevice.StateCallback() {
            override fun onOpened(d: CameraDevice) { camera = d; createSession(d, reader) }
            override fun onDisconnected(d: CameraDevice) { d.close() }
            override fun onError(d: CameraDevice, e: Int) { status("camera error $e"); d.close() }
        }
        cm.openCamera(camId, sc.createARDeviceStateCallback(devCb, camHandler), camHandler)
    }

    private fun createSession(d: CameraDevice, reader: ImageReader) {
        val sc = shared!!
        val arSurfaces: List<Surface> = sc.arCoreSurfaces
        val all = arSurfaces + reader.surface
        val cb = object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(cs: CameraCaptureSession) {
                captureSession = cs
                val rb = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
                arSurfaces.forEach { rb.addTarget(it) }
                applyLocks(rb)
                cs.setRepeatingRequest(rb.build(), null, camHandler)
                // Periodic still: one request that feeds ARCore AND the hi-res reader => same timestamp.
                val still = d.createCaptureRequest(CameraDevice.TEMPLATE_RECORD)
                all.forEach { still.addTarget(it) }
                still.setTag(STILL_TAG)
                applyLocks(still)
                val req = still.build()
                val tick = object : Runnable {
                    override fun run() {
                        if (!recording) return
                        try { cs.capture(req, null, camHandler) } catch (e: Exception) { Log.w(TAG, "still", e) }
                        camHandler?.postDelayed(this, opt.stillPeriodMs)
                    }
                }
                stillTicker = tick
                camHandler?.postDelayed(tick, 1000)
            }
            override fun onActive(cs: CameraCaptureSession) {
                if (!arActive) {
                    session.resume()
                    arActive = true
                    sc.setCaptureCallback(object : CameraCaptureSession.CaptureCallback() {
                        override fun onCaptureCompleted(c: CameraCaptureSession, r: CaptureRequest, res: TotalCaptureResult) {
                            if (r.tag != STILL_TAG) return
                            val o = JSONObject().put("t", (res.get(CaptureResult.SENSOR_TIMESTAMP) ?: 0L) / 1e9)
                            CameraInfo.resultFields(res, o)
                            s.csv("extras/hires_capture_results.jsonl", "", o.toString())
                        }
                    }, camHandler)
                }
            }
            override fun onConfigureFailed(cs: CameraCaptureSession) { status("shared camera: configure failed (stream combo unsupported?)") }
        }
        d.createCaptureSession(all, sc.createARSessionStateCallback(cb, camHandler), camHandler)
    }

    private fun applyLocks(rb: CaptureRequest.Builder) {
        if (opt.lockFocus) {
            rb.set(CaptureRequest.CONTROL_AE_LOCK, true)
            rb.set(CaptureRequest.CONTROL_AWB_LOCK, true)
        }
        rb.set(CaptureRequest.STATISTICS_OIS_DATA_MODE, CaptureRequest.STATISTICS_OIS_DATA_MODE_ON)
    }

    companion object {
        const val TAG = "ArRecorder"
        const val STILL_TAG = "still"

        fun yuvToNv21(img: Image): ByteArray {
            val w = img.width; val h = img.height
            val out = ByteArray(w * h * 3 / 2)
            val y = img.planes[0]
            var o = 0
            for (r in 0 until h) { y.buffer.position(r * y.rowStride); y.buffer.get(out, o, w); o += w }
            val u = img.planes[1]; val v = img.planes[2]
            for (r in 0 until h / 2) for (c in 0 until w / 2) {
                out[o++] = v.buffer.get(r * v.rowStride + c * v.pixelStride)
                out[o++] = u.buffer.get(r * u.rowStride + c * u.pixelStride)
            }
            return out
        }
    }
}
