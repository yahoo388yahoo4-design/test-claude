package com.real2sim.capture

import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.min

/**
 * Depth pipeline between ARCore and everything that uses depth, off the GL thread:
 *
 *   GL thread: copy the new depth map(s) + pose -> [submit] (latest frame wins, never blocks)
 *   worker:    DepthFilter -> MapBuilder occupancy / virtual lidar (every frame)
 *              -> TsdfVolume integration (at most every [tsdfIntervalMs])
 *              -> live mesh re-extraction of dirty blocks (at most every [liveMeshIntervalMs])
 *   stop:      [finish] -> final marching-cubes mesh, small components removed, structure/object
 *              segmentation, cleaned points -> [writeOutputs]
 *
 * Raw depth + confidence (ARCore acquireRawDepthImage16Bits / acquireRawDepthConfidenceImage) feeds the
 * TSDF when present; the dense smoothed depth (acquireDepthImage16Bits) feeds the occupancy grid (it
 * keeps textureless walls that raw depth drops), and either one stands in for the other when missing.
 */
class DepthFusion(
    @Volatile var map: MapBuilder?,
    val filterCfg: DepthFilter.Config = DepthFilter.Config(),
    val tsdfCfg: TsdfVolume.Config = TsdfVolume.Config(),
    val tsdfIntervalMs: Long = 100,
    val liveMeshIntervalMs: Long = 1200,
    val maxLiveTriangles: Int = 160_000,
    val minComponentTriangles: Int = 120,
) {
    /**
     * One depth sample. [raw] / [smooth] DEPTH16 (either may be null) with their sizes; [conf] raw confidence
     * 0..255. Intrinsics [k] = fx, fy, cx, cy of the CPU image of size [kw] x [kh] (scaled to each depth map
     * here). [pose] column-major camera-to-world. [rgb] optional 0xRRGGBB at [rgbW] x [rgbH].
     */
    class Frame(
        val raw: ShortArray?, val conf: ByteArray?, val rawW: Int, val rawH: Int,
        val smooth: ShortArray?, val smW: Int, val smH: Int,
        val k: FloatArray, val kw: Int, val kh: Int, val pose: FloatArray, val tNs: Long,
        val rgb: IntArray? = null, val rgbW: Int = 0, val rgbH: Int = 0,
    )

    /** Triangle soup for the live renderer: per vertex x, y, z, nx, ny, nz. */
    class LiveMesh(val soup: FloatArray, val triangles: Int, val version: Int)
    class LiveDepth(val argb: IntArray, val w: Int, val h: Int, val version: Int)

    class Result(
        val mesh: TriMesh, val rawTriangles: Int, val seg: MeshOps.Segmentation,
        val points: FloatArray, val pointNrm: FloatArray, val pointRgb: IntArray, val floorY: Float, val ms: Long,
    )

    private val exec = Executors.newSingleThreadExecutor { r -> Thread(r, "depth-fusion").apply { isDaemon = true; priority = Thread.NORM_PRIORITY - 1 } }
    private val slot = AtomicReference<Frame?>(null)
    private val scheduled = AtomicBoolean(false)
    private val filterRaw = DepthFilter(filterCfg)
    private val filterSmooth = DepthFilter(filterCfg.copy(minConfidence = 0f))
    val tsdf = TsdfVolume(tsdfCfg)

    @Volatile var liveMesh: LiveMesh? = null; private set
    @Volatile var liveDepth: LiveDepth? = null; private set
    @Volatile var closed = false; private set

    // stats (worker thread writes, others read approximately)
    @Volatile var submitted = 0; private set
    @Volatile var dropped = 0; private set
    @Volatile var processed = 0; private set
    @Volatile var fused = 0; private set
    @Volatile var filterMs = 0.0; private set
    @Volatile var integrateMs = 0.0; private set
    @Volatile var liveExtractMs = 0.0; private set
    @Volatile var usedRaw = 0; private set
    private var lastTsdf = 0L
    private var lastLive = 0L
    private var liveVersion = 0
    private val blockSoup = HashMap<Int, FloatArray>()

    fun submit(f: Frame) {
        if (closed) return
        submitted++
        if (slot.getAndSet(f) != null) dropped++
        if (scheduled.compareAndSet(false, true)) exec.execute(::drain)
    }

    private fun drain() {
        try {
            while (true) { val f = slot.getAndSet(null) ?: break; try { process(f) } catch (e: Exception) { lastError = e.toString() } }
        } finally {
            scheduled.set(false)
            if (slot.get() != null && !closed && scheduled.compareAndSet(false, true)) exec.execute(::drain)
        }
    }

    @Volatile var lastError: String? = null; private set

    private fun now() = System.nanoTime() / 1_000_000

    /** Processes one frame on the calling thread (the worker; tests call it directly). */
    fun process(f: Frame) {
        if (f.raw == null && f.smooth == null) return
        val t0 = System.nanoTime()
        fun scaled(w: Int, h: Int): FloatArray {
            val sx = w.toFloat() / f.kw; val sy = h.toFloat() / f.kh
            return floatArrayOf(f.k[0] * sx, f.k[1] * sy, f.k[2] * sx, f.k[3] * sy)
        }
        var rawM: FloatArray? = null; var kr: FloatArray? = null
        if (f.raw != null) {
            kr = scaled(f.rawW, f.rawH)
            rawM = filterRaw.filter(f.raw, f.conf, f.rawW, f.rawH, kr[0], kr[1], kr[2], kr[3], f.pose)
        }
        var smM: FloatArray? = null; var ks: FloatArray? = null
        if (f.smooth != null) {
            ks = scaled(f.smW, f.smH)
            smM = filterSmooth.filter(f.smooth, null, f.smW, f.smH, ks[0], ks[1], ks[2], ks[3], f.pose)
        }
        val t1 = System.nanoTime()
        filterMs = 0.9 * filterMs + 0.1 * (t1 - t0) / 1e6
        processed++
        // live depth view: what the TSDF gets
        val (vd, vw, vh) = if (rawM != null) Triple(rawM, f.rawW, f.rawH) else Triple(smM!!, f.smW, f.smH)
        liveDepth = LiveDepth(DepthViz.colorizeMeters(vd, 5f), vw, vh, processed)
        // occupancy + virtual lidar
        map?.let { m ->
            val (od, ow, oh, ok) = if (smM != null) Quad(smM, f.smW, f.smH, ks!!) else Quad(rawM!!, f.rawW, f.rawH, kr!!)
            m.integrateMeters(od, ow, oh, ok[0], ok[1], ok[2], ok[3], f.pose, atan2(-f.pose[10], -f.pose[8]), f.tNs)
        }
        // TSDF at a bounded rate
        val now = now()
        if (now - lastTsdf >= tsdfIntervalMs) {
            lastTsdf = now
            val (td, tw, th, tk) = if (rawM != null) Quad(rawM, f.rawW, f.rawH, kr!!) else Quad(smM!!, f.smW, f.smH, ks!!)
            if (rawM != null) usedRaw++
            val rgb = if (f.rgb != null && f.rgbW == tw && f.rgbH == th) f.rgb else null
            val t2 = System.nanoTime()
            tsdf.integrate(td, tw, th, tk[0], tk[1], tk[2], tk[3], f.pose, rgb)
            integrateMs = 0.9 * integrateMs + 0.1 * (System.nanoTime() - t2) / 1e6
            fused++
        }
        if (now - lastLive >= liveMeshIntervalMs && fused > 0) { lastLive = now; updateLive() }
    }

    private data class Quad(val d: FloatArray, val w: Int, val h: Int, val k: FloatArray)

    /** Re-extracts dirty blocks into per-block soups and publishes the concatenation. */
    fun updateLive() {
        val t0 = System.nanoTime()
        val dirty = tsdf.takeDirty(800)
        for (b in dirty) {
            val m = tsdf.extract(max(tsdfCfg.minExtractWeight, 3f), intArrayOf(b))
            if (m.triangleCount == 0) blockSoup.remove(b) else blockSoup[b] = soup(m)
        }
        var n = 0
        for (s in blockSoup.values) n += s.size
        n = min(n, maxLiveTriangles * 18)
        val all = FloatArray(n)
        var o = 0
        for (s in blockSoup.values) { if (o + s.size > n) break; System.arraycopy(s, 0, all, o, s.size); o += s.size }
        liveMesh = LiveMesh(if (o == n) all else all.copyOf(o), o / 18, ++liveVersion)
        liveExtractMs = (System.nanoTime() - t0) / 1e6
    }

    private fun soup(m: TriMesh): FloatArray {
        val a = FloatArray(m.triangleCount * 18)
        var o = 0
        for (t in m.tris) { System.arraycopy(m.xyz, t * 3, a, o, 3); System.arraycopy(m.nrm, t * 3, a, o + 3, 3); o += 6 }
        return a
    }

    /**
     * Waits for queued work, then extracts the final mesh (on the worker). Null if it did not finish within
     * [timeoutMs] or nothing was fused. Closes the worker.
     */
    fun finish(floorY: Float = Float.NaN, timeoutMs: Long = 30_000): Result? {
        if (closed) return null
        val fut = exec.submit<Result?> {
            slot.getAndSet(null)?.let { try { process(it) } catch (_: Exception) {} }
            if (fused == 0) null else buildResult(floorY)
        }
        return try { fut.get(timeoutMs, TimeUnit.MILLISECONDS) } catch (e: Exception) { lastError = e.toString(); null } finally { close() }
    }

    fun close() { closed = true; exec.shutdown() }

    /** Final mesh: marching cubes, small components removed, segmentation, cleaned points. */
    fun buildResult(floorY: Float = Float.NaN): Result {
        val t0 = System.nanoTime()
        val raw = tsdf.extract()
        val mesh = MeshOps.removeSmallComponents(raw, minComponentTriangles)
        val vox = tsdfCfg.voxel
        val seg = MeshOps.segment(mesh, tol = vox * 1.5f, minObjectTriangles = minComponentTriangles / 2)
        val floor = if (!floorY.isNaN()) floorY else lowPercentileY(mesh)
        // points: mesh vertices, one per voxel, statistical outliers removed
        val idx = MeshOps.voxelDownsample(mesh.xyz, vox)
        val sub = FloatArray(idx.size * 3) { mesh.xyz[idx[it / 3] * 3 + it % 3] }
        val keep = MeshOps.statisticalOutliers(sub, k = 8, radius = vox * 3, stdMul = 2f)
        val kept = idx.indices.filter { keep[it] }
        val pts = FloatArray(kept.size * 3); val nrm = FloatArray(kept.size * 3); val rgb = IntArray(kept.size)
        val vcol = vertexColors(mesh, floor)
        kept.forEachIndexed { j, i ->
            val v = idx[i]
            System.arraycopy(mesh.xyz, v * 3, pts, j * 3, 3); System.arraycopy(mesh.nrm, v * 3, nrm, j * 3, 3); rgb[j] = vcol[v]
        }
        return Result(mesh, raw.triangleCount, seg, pts, nrm, rgb, floor, (System.nanoTime() - t0) / 1_000_000)
    }

    /** Camera colour where the TSDF has one for most of the mesh, else height colour. */
    fun vertexColors(m: TriMesh, floor: Float): IntArray {
        val rgb = m.rgb
        val coloured = rgb?.count { it >= 0 } ?: 0
        val useCam = rgb != null && coloured >= m.vertexCount / 2
        return IntArray(m.vertexCount) { v ->
            if (useCam && rgb!![v] >= 0) rgb[v] else MeshOps.heightColor(m.xyz[v * 3 + 1], floor)
        }
    }

    /** Writes mesh.ply, points.ply, objects.json, recon.json into [dir]. */
    fun writeOutputs(r: Result, dir: File) {
        dir.mkdirs()
        val note = "r2s-capture TSDF fusion of filtered ARCore depth; ARCore world (y up, metres)"
        MeshOps.writeMeshPly(File(dir, "mesh.ply"), r.mesh, vertexColors(r.mesh, r.floorY), note)
        MeshOps.writePointsPly(File(dir, "points.ply"), r.points, r.pointNrm, r.pointRgb, note)
        File(dir, "objects.json").writeText(segJson(r).toString(1))
        File(dir, "recon.json").writeText(describe(r).toString(1))
    }

    fun segJson(r: Result): JSONObject = JSONObject().apply {
        put("frame", "ARCore world of the session (y up, metres)")
        put("planes", JSONArray().apply {
            r.seg.planes.forEach { p -> put(JSONObject().put("normal", JSONArray(listOf(p.nx, p.ny, p.nz).map { it.toDouble() })).put("d", p.d.toDouble()).put("vertices", p.inliers)) }
        })
        put("objects", JSONArray().apply {
            r.seg.objects.forEachIndexed { i, o ->
                put(JSONObject().put("id", i).put("triangles", o.triangles)
                    .put("min", JSONArray(o.min.map { it.toDouble() })).put("max", JSONArray(o.max.map { it.toDouble() })))
            }
        })
    }

    /** Settings + statistics (session.json "depth_processing" and recon.json). */
    fun describe(r: Result? = null): JSONObject = JSONObject().apply {
        put("filter", filterCfg.toJson())
        put("filter_stats_raw", filterRaw.total.toJson())
        put("filter_stats_smooth", filterSmooth.total.toJson())
        put("tsdf", tsdfCfg.toJson())
        put("tsdf_interval_ms", tsdfIntervalMs); put("min_component_triangles", minComponentTriangles)
        put("frames_submitted", submitted); put("frames_dropped_busy", dropped); put("frames_processed", processed)
        put("frames_fused", fused); put("frames_fused_from_raw_depth", usedRaw)
        put("blocks", tsdf.blockCount); put("volume_full", tsdf.full); put("volume_mb", tsdf.bytesUsed / 1e6)
        put("avg_filter_ms", filterMs); put("avg_integrate_ms", integrateMs)
        lastError?.let { put("error", it) }
        if (r != null) {
            put("mesh_vertices", r.mesh.vertexCount); put("mesh_triangles", r.mesh.triangleCount)
            put("mesh_triangles_before_component_filter", r.rawTriangles)
            put("points", r.points.size / 3); put("objects", r.seg.objects.size); put("planes", r.seg.planes.size)
            put("floor_y", r.floorY.toDouble()); put("extract_ms", r.ms)
            put("outputs", JSONArray(listOf("mesh.ply", "points.ply", "objects.json", "recon.json")))
        }
    }

    companion object {
        fun lowPercentileY(m: TriMesh): Float {
            if (m.vertexCount == 0) return 0f
            val ys = FloatArray(m.vertexCount) { m.xyz[it * 3 + 1] }
            ys.sort()
            return ys[(ys.size * 0.02f).toInt()]
        }
    }
}
