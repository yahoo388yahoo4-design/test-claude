package com.real2sim.capture

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.SurfaceTexture
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.params.OutputConfiguration
import android.hardware.camera2.params.SessionConfiguration
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Handler
import android.os.HandlerThread
import android.util.Log
import android.util.Size
import android.view.Gravity
import android.view.Surface
import android.view.TextureView
import android.widget.FrameLayout
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.TimeUnit
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10

/**
 * Live camera on the main screen while not recording, like the iPhone app (which always shows the
 * camera). Nothing is written. Mode A runs a preview-only ARCore session (GLSurfaceView +
 * BackgroundRenderer); modes B / sensors, or phones without ARCore, show a plain Camera2 preview of
 * the main rear camera. [stop] fully releases the camera (ARCore session closed / CameraDevice closed)
 * before a recorder opens it; MainActivity restarts the preview after recording.
 */
class IdlePreview(private val act: Activity, private val host: FrameLayout, private val log: (String) -> Unit) {
    private var ar: ArPreview? = null
    private var cam: CameraPreview? = null
    var kind = ""
        private set

    val running get() = ar != null || cam != null

    /** (Re)starts the preview for [mode]; no-op without the camera permission. UI thread. */
    fun start(mode: CaptureMode) {
        stop()
        if (act.checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) return
        if (mode == CaptureMode.RGBD && arcoreReady()) {
            val p = ArPreview(act)
            if (p.open()) {
                host.addView(p.view, 0, FrameLayout.LayoutParams(-1, -1))
                p.resume()
                ar = p; kind = "arcore"
                return
            }
            p.close()
        }
        val c = CameraPreview(act, log)
        host.addView(c.view, 0, FrameLayout.LayoutParams(-1, -1, Gravity.CENTER))
        c.open()
        cam = c; kind = "camera2"
    }

    /** Releases the camera and removes the preview view. Blocks briefly until the camera is closed. UI thread. */
    fun stop() {
        ar?.let { it.close(); host.removeView(it.view) }; ar = null
        cam?.let { it.close(); host.removeView(it.view) }; cam = null
        kind = ""
    }

    private fun arcoreReady(): Boolean = try {
        ArCoreApk.getInstance().checkAvailability(act) == ArCoreApk.Availability.SUPPORTED_INSTALLED
    } catch (e: Throwable) { false }

    // ------------------------------------------------------------------ ARCore (mode A)

    private class ArPreview(private val act: Activity) : GLSurfaceView.Renderer {
        val view = GLSurfaceView(act)
        @Volatile private var session: Session? = null
        private val bg = BackgroundRenderer()

        fun open(): Boolean {
            session = try {
                Session(act).also { s ->
                    s.configure(Config(s).apply {
                        updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
                        focusMode = Config.FocusMode.AUTO
                        planeFindingMode = Config.PlaneFindingMode.DISABLED
                        depthMode = Config.DepthMode.DISABLED
                    })
                }
            } catch (e: Throwable) { Log.w(TAG, "ARCore preview", e); return false }   // incl. missing native lib
            view.preserveEGLContextOnPause = true
            view.setEGLContextClientVersion(2)
            view.setEGLConfigChooser(8, 8, 8, 8, 16, 0)
            view.setRenderer(this)
            view.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
            return true
        }

        fun resume() {
            try { session?.resume(); view.onResume() } catch (e: Exception) { Log.w(TAG, "ARCore preview resume", e) }
        }

        fun close() {
            val s = session ?: return
            try { view.onPause() } catch (_: Exception) {}   // waits for the GL thread to stop drawing
            session = null
            try { s.pause() } catch (_: Exception) {}
            try { s.close() } catch (_: Exception) {}
        }

        override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
            GLES20.glClearColor(0f, 0f, 0f, 1f)
            bg.create()
            session?.setCameraTextureName(bg.textureId)
        }

        override fun onSurfaceChanged(gl: GL10?, width: Int, height: Int) {
            GLES20.glViewport(0, 0, width, height)
            @Suppress("DEPRECATION")
            session?.setDisplayGeometry(act.windowManager.defaultDisplay.rotation, width, height)
        }

        override fun onDrawFrame(gl: GL10?) {
            GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
            val s = session ?: return
            try {
                s.setCameraTextureName(bg.textureId)
                bg.draw(s.update())
            } catch (_: Exception) {}
        }
    }

    // ------------------------------------------------------------------ Camera2 (modes B / sensors)

    /** TextureView that fills its parent, cropping the content to keep its aspect (iOS resizeAspectFill). */
    private class CoverView(ctx: Context) : TextureView(ctx) {
        var cw = 0; var ch = 0
        override fun onMeasure(wSpec: Int, hSpec: Int) {
            val w = MeasureSpec.getSize(wSpec); val h = MeasureSpec.getSize(hSpec)
            if (cw == 0 || ch == 0 || w == 0 || h == 0) { setMeasuredDimension(w, h); return }
            if (w.toLong() * ch >= h.toLong() * cw) setMeasuredDimension(w, (w.toLong() * ch / cw).toInt())
            else setMeasuredDimension((h.toLong() * cw / ch).toInt(), h)
        }
    }

    private class CameraPreview(private val act: Activity, private val log: (String) -> Unit) {
        val view = CoverView(act)
        private val thread = HandlerThread("idle-preview").apply { start() }
        private val handler = Handler(thread.looper)
        private val exec = Executor { handler.post(it) }
        @Volatile private var device: CameraDevice? = null
        @Volatile private var capture: CameraCaptureSession? = null
        @Volatile private var closed = false
        @Volatile private var opening = false
        private var closedLatch = CountDownLatch(0)
        private var size: Size? = null

        @SuppressLint("MissingPermission")
        fun open() {
            val cm = act.getSystemService(CameraManager::class.java) ?: return
            val id = try {
                cm.cameraIdList.firstOrNull { cm.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) == CameraCharacteristics.LENS_FACING_BACK }
            } catch (e: Exception) { null } ?: return
            val ch = cm.getCameraCharacteristics(id)
            val sizes = ch.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)?.getOutputSizes(SurfaceTexture::class.java) ?: return
            // 4:3 like the recordings, at most 1920 wide
            val s = sizes.filter { it.width <= 1920 && it.width * 3 == it.height * 4 }.maxByOrNull { it.width * it.height }
                ?: sizes.filter { it.width <= 1920 }.maxByOrNull { it.width * it.height } ?: sizes.first()
            size = s
            val rot = ch.get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 90
            if (rot % 180 != 0) { view.cw = s.height; view.ch = s.width } else { view.cw = s.width; view.ch = s.height }
            view.requestLayout()
            view.surfaceTextureListener = object : TextureView.SurfaceTextureListener {
                override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) { openCamera(cm, id, st) }
                override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, w: Int, h: Int) {}
                override fun onSurfaceTextureDestroyed(st: SurfaceTexture) = true
                override fun onSurfaceTextureUpdated(st: SurfaceTexture) {}
            }
            view.surfaceTexture?.let { openCamera(cm, id, it) }
        }

        @SuppressLint("MissingPermission")
        private fun openCamera(cm: CameraManager, id: String, st: SurfaceTexture) {
            if (closed || device != null) return
            val s = size ?: return
            st.setDefaultBufferSize(s.width, s.height)
            closedLatch = CountDownLatch(1)
            opening = true
            try {
                cm.openCamera(id, exec, object : CameraDevice.StateCallback() {
                    override fun onOpened(d: CameraDevice) {
                        opening = false
                        if (closed) { d.close(); return }
                        device = d
                        val surface = Surface(st)
                        val cfg = SessionConfiguration(SessionConfiguration.SESSION_REGULAR, listOf(OutputConfiguration(surface)), exec,
                            object : CameraCaptureSession.StateCallback() {
                                override fun onConfigured(cs: CameraCaptureSession) {
                                    if (closed) { cs.close(); return }
                                    capture = cs
                                    try {
                                        val rq = d.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                                            addTarget(surface)
                                            set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
                                        }.build()
                                        cs.setRepeatingRequest(rq, null, handler)
                                    } catch (e: Exception) { Log.w(TAG, "preview request", e) }
                                }
                                override fun onConfigureFailed(cs: CameraCaptureSession) { log("camera preview: configuration failed") }
                            })
                        try { d.createCaptureSession(cfg) } catch (e: Exception) { Log.w(TAG, "preview session", e) }
                    }
                    override fun onDisconnected(d: CameraDevice) { opening = false; d.close() }
                    override fun onError(d: CameraDevice, error: Int) { opening = false; d.close(); log("camera preview error $error") }
                    override fun onClosed(d: CameraDevice) { closedLatch.countDown() }
                })
            } catch (e: Exception) { opening = false; closedLatch.countDown(); log("camera preview: $e") }
        }

        fun close() {
            closed = true
            view.surfaceTextureListener = null
            val latch = closedLatch
            handler.post {
                try { capture?.close() } catch (_: Exception) {}
                capture = null
                val d = device
                device = null
                // still opening: onOpened sees `closed`, closes the device and onClosed releases the latch
                if (d != null) try { d.close() } catch (_: Exception) { latch.countDown() } else if (!opening) latch.countDown()
            }
            // the recorder opens the camera right after this: wait until the HAL has released it
            try { latch.await(1500, TimeUnit.MILLISECONDS) } catch (_: InterruptedException) {}
            thread.quitSafely()
        }
    }

    companion object { private const val TAG = "IdlePreview" }
}
