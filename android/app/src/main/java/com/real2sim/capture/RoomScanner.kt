package com.real2sim.capture

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Matrix
import android.os.Build
import android.util.Log
import com.google.ar.core.Plane
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.objectdetector.ObjectDetector
import org.json.JSONObject
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.cos
import kotlin.math.sin

/**
 * Room mapping during a mode A recording: an on-device object detector (EfficientDet-Lite2, COCO, MediaPipe
 * Tasks) runs ~2x a second on a 640 px copy of the camera image; each furniture / appliance box is lifted to 3D
 * with the ARCore depth map and the camera pose and fused into persistent objects (ObjectFusion). At the end
 * the ARCore planes give floor and walls, and [writeRoomMap] writes roommap/room_map.json (HOME.md) plus a mesh
 * crop of each object from the TSDF reconstruction (roommap/objects/<id>.ply).
 */
class RoomScanner(private val ctx: Context, private val sensorRotation: Int) {
    /** One frame for the detector: [rgb] 0xRRGGBB at [w] x [h] (sensor orientation) with intrinsics [k]. */
    class Input(
        val rgb: IntArray, val w: Int, val h: Int, val k: FloatArray, val pose: FloatArray,
        val depth: ShortArray?, val dw: Int, val dh: Int, val kDepth: FloatArray?, val floorY: Float,
    )

    /** A 2D detection for the live overlay, in the upright (portrait) image, normalised 0..1. */
    class Det2D(val label: String, val score: Float, val l: Float, val t: Float, val r: Float, val b: Float, val placed: Boolean)

    val fusion = ObjectFusion()
    @Volatile var lastDetections: List<Det2D> = emptyList(); private set
    @Volatile var error: String? = null; private set
    @Volatile var processed = 0; private set
    @Volatile var lastMs = 0L; private set
    private val busy = AtomicBoolean(false)
    private val exec = Executors.newSingleThreadExecutor { r -> Thread(r, "room-scan").apply { isDaemon = true; priority = Thread.NORM_PRIORITY - 1 } }
    private var detector: ObjectDetector? = null
    private var lastSubmit = 0L

    /** True when a frame is wanted now (not busy, >= [periodMs] since the last one). */
    fun wants(nowNs: Long, periodMs: Long = 450): Boolean = !busy.get() && error == null && nowNs - lastSubmit > periodMs * 1_000_000L

    fun submit(i: Input, nowNs: Long) {
        if (!busy.compareAndSet(false, true)) return
        lastSubmit = nowNs
        try {
            exec.execute {
                try { process(i) } catch (e: Throwable) { Log.w(TAG, "detect", e); error = e.toString() } finally { busy.set(false) }
            }
        } catch (_: java.util.concurrent.RejectedExecutionException) { busy.set(false) }
    }

    private fun detectorOrNull(): ObjectDetector? {
        detector?.let { return it }
        val opts = ObjectDetector.ObjectDetectorOptions.builder()
            .setBaseOptions(BaseOptions.builder().setModelAssetPath(MODEL).build())
            .setRunningMode(RunningMode.IMAGE)
            .setMaxResults(20)
            .setScoreThreshold(0.35f)
            .build()
        return ObjectDetector.createFromOptions(ctx, opts).also { detector = it }
    }

    private fun process(i: Input) {
        val t0 = System.currentTimeMillis()
        val det = detectorOrNull() ?: return
        val src = Bitmap.createBitmap(IntArray(i.rgb.size) { i.rgb[it] or (0xff shl 24) }, i.w, i.h, Bitmap.Config.ARGB_8888)
        val bmp = if (sensorRotation % 360 == 0) src else Bitmap.createBitmap(src, 0, 0, i.w, i.h, Matrix().apply { postRotate(sensorRotation.toFloat()) }, false)
        val res = det.detect(BitmapImageBuilder(bmp).build())
        val rw = bmp.width.toFloat(); val rh = bmp.height.toFloat()
        val out = ArrayList<Det2D>()
        for (d in res.detections()) {
            val c = d.categories().firstOrNull() ?: continue
            val cat = RoomCategories.FROM_COCO[c.categoryName()] ?: continue
            val bb = d.boundingBox()
            var placed = false
            if (cat in RoomCategories.MAPPED && i.depth != null && i.kDepth != null) {
                val box = toSensor(bb.left, bb.top, bb.right, bb.bottom, i.w, i.h)
                val front = DetectionLift.frontDepth(i.depth, i.dw, i.dh, box, i.k, i.kDepth)
                DetectionLift.lift(cat, c.score(), box, i.k, front, i.pose, i.floorY)?.let { fusion.add(it); placed = true }
            }
            out.add(Det2D(cat, c.score(), bb.left / rw, bb.top / rh, bb.right / rw, bb.bottom / rh, placed))
        }
        if (bmp !== src) bmp.recycle()
        src.recycle()
        lastDetections = out
        processed++
        lastMs = System.currentTimeMillis() - t0
    }

    /** Box in the rotated (upright) image -> box in the sensor-oriented image of [w] x [h]. */
    private fun toSensor(l: Float, t: Float, r: Float, b: Float, w: Int, h: Int): FloatArray {
        fun pt(x: Float, y: Float): FloatArray = when ((sensorRotation % 360 + 360) % 360) {
            90 -> floatArrayOf(y, h - x)
            180 -> floatArrayOf(w - x, h - y)
            270 -> floatArrayOf(w - y, x)
            else -> floatArrayOf(x, y)
        }
        val a = pt(l, t); val c = pt(r, b)
        return floatArrayOf(minOf(a[0], c[0]), minOf(a[1], c[1]), maxOf(a[0], c[0]), maxOf(a[1], c[1]))
    }

    fun liveLine(): String {
        error?.let { return "objects: detector failed ($it)" }
        val objs = fusion.confirmed().groupingBy { it.category }.eachCount().entries.joinToString(" ") { "${it.key}×${it.value}" }
        return "objects ${fusion.count} ($lastMs ms) $objs"
    }

    /** Waits for the frame in flight, then frees the detector. */
    fun close() {
        exec.shutdown()
        try { exec.awaitTermination(3, TimeUnit.SECONDS) } catch (_: InterruptedException) {}
        try { detector?.close() } catch (_: Exception) {}
        detector = null
    }

    companion object {
        private const val TAG = "RoomScanner"
        const val MODEL = "efficientdet_lite2.tflite"

        /** ARCore planes reduced to [PlaneInfo] (world coordinates); call before the session is closed. */
        fun planes(session: Session): List<PlaneInfo> {
            val out = ArrayList<PlaneInfo>()
            for (p in session.getAllTrackables(Plane::class.java)) {
                if (p.trackingState == TrackingState.STOPPED || p.subsumedBy != null) continue
                val pose = p.centerPose
                val poly = p.polygon
                val pts = ArrayList<FloatArray>()
                while (poly.remaining() >= 2) { val x = poly.get(); val z = poly.get(); pts.add(pose.transformPoint(floatArrayOf(x, 0f, z))) }
                out.add(PlaneInfo(p.type == Plane.Type.VERTICAL, p.type == Plane.Type.HORIZONTAL_UPWARD_FACING,
                    floatArrayOf(pose.tx(), pose.ty(), pose.tz()), pose.yAxis, pts))
            }
            return out
        }

        /**
         * roommap/room_map.json (+ roommap/objects/<id>.ply cut from [mesh] when there is one). Returns the JSON.
         */
        fun writeRoomMap(dir: File, name: String, session: String, planes: List<PlaneInfo>, map: MapBuilder,
                         objects: List<ObjectFusion.Track>, mesh: TriMesh?, colors: IntArray?): JSONObject {
            val traj = synchronized(map) { map.traj.map { it.copyOf() } }
            val plan0 = RoomGeometry.build(planes, map.floorY, traj)
            val plan = if (plan0.walls.size < 2) RoomGeometry.build(planes, map.floorY, traj, map.occupiedPoints(600)) else plan0
            val meshes = HashMap<Int, String>()
            if (mesh != null && mesh.triangleCount > 0) objects.forEachIndexed { idx, t ->
                try {
                    val sub = crop(mesh, t, plan.floorY)
                    if (sub != null && sub.second.triangleCount >= 30) {
                        val rel = "objects/${t.category}_$idx.ply"
                        val cols = IntArray(sub.second.vertexCount) { v -> colors?.getOrNull(sub.first[v]) ?: 0x9e9e9e }
                        MeshOps.writeMeshPly(File(dir, rel), sub.second, cols, "r2s room map object ${t.category}; ARCore world (y up, metres)")
                        meshes[idx] = rel
                    }
                } catch (e: Exception) { Log.w(TAG, "crop", e) }
            }
            val json = RoomMapJson.build(name.ifBlank { "Room" }, session, "${Build.MANUFACTURER} ${Build.MODEL}", plan, objects, meshes)
            dir.parentFile?.let { RoomMapJson.withEspNodes(json, it) }
            dir.mkdirs()
            File(dir, "room_map.json").writeText(json.toString(1))
            return json
        }

        /** Triangles of [m] inside the object's box (slightly enlarged, floor excluded); with the source vertex of each kept vertex. */
        private fun crop(m: TriMesh, t: ObjectFusion.Track, floorY: Float): Pair<IntArray, TriMesh>? {
            val ax = cos(t.yaw); val az = sin(t.yaw)
            val hu = t.width / 2 + 0.1f; val hw = t.depth / 2 + 0.15f
            val y0 = maxOf(t.cy - t.height / 2 - 0.05f, if (floorY.isNaN()) -1e9f else floorY + 0.03f); val y1 = t.cy + t.height / 2 + 0.1f
            val inside = BooleanArray(m.vertexCount) { v ->
                val dx = m.xyz[v * 3] - t.cx; val dz = m.xyz[v * 3 + 2] - t.cz; val y = m.xyz[v * 3 + 1]
                val u = dx * ax + dz * az; val w = -dx * az + dz * ax
                kotlin.math.abs(u) < hu && kotlin.math.abs(w) < hw && y in y0..y1
            }
            val keep = BooleanArray(m.triangleCount) { k -> inside[m.tris[k * 3]] && inside[m.tris[k * 3 + 1]] && inside[m.tris[k * 3 + 2]] }
            if (keep.none { it }) return null
            val sub = MeshOps.subset(m, keep)
            // source index of each kept vertex (same order MeshOps.subset assigns them)
            val src = IntArray(sub.vertexCount); val seen = IntArray(m.vertexCount) { -1 }; var n = 0
            for (k in 0 until m.triangleCount) if (keep[k]) for (c in 0 until 3) {
                val v = m.tris[k * 3 + c]; if (seen[v] < 0) { seen[v] = n; src[n++] = v }
            }
            return Pair(src, sub)
        }
    }
}
