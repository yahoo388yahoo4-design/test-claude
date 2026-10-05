package com.real2sim.capture

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.io.File
import java.nio.ByteBuffer
import java.util.Random
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Synthetic scene: floor (y = 0) and a wall (z = -3) with a 0.6 x 0.5 x 0.6 m box on the floor at
 * (0, 0.25, -1.5). Depth maps are ray cast at ARCore depth resolution (160x120) with ARCore's camera
 * convention, then corrupted like depth-from-motion: relative noise, flying pixels smeared across the
 * silhouettes and random speckle with random confidence.
 */
class DepthFusionTest {
    companion object {
        const val W = 160; const val H = 120
        const val FX = 125f; const val FY = 125f; const val CX = 79.5f; const val CY = 59.5f
        val BOX_MIN = floatArrayOf(-0.3f, 0f, -1.8f)
        val BOX_MAX = floatArrayOf(0.3f, 0.5f, -1.2f)
        const val WALL_Z = -3f

        /** Column-major camera-to-world, camera -z looking from [eye] at [target], y up. */
        fun lookAt(eye: FloatArray, target: FloatArray): FloatArray {
            var fx = target[0] - eye[0]; var fy = target[1] - eye[1]; var fz = target[2] - eye[2]
            val fl = sqrt(fx * fx + fy * fy + fz * fz); fx /= fl; fy /= fl; fz /= fl
            // right = f x up
            var rx = -fz; val ry = 0f; var rz = fx
            val rl = sqrt(rx * rx + rz * rz); rx /= rl; rz /= rl
            // up' = r x f
            val ux = ry * fz - rz * fy; val uy = rz * fx - rx * fz; val uz = rx * fy - ry * fx
            return floatArrayOf(rx, ry, rz, 0f, ux, uy, uz, 0f, -fx, -fy, -fz, 0f, eye[0], eye[1], eye[2], 1f)
        }

        /** Distance along the camera -z axis to the scene for pixel (u, v), or 0. */
        fun castDepth(m: FloatArray, u: Float, v: Float): Float {
            val rx = (u - CX) / FX; val ry = -(v - CY) / FY
            val dx = m[0] * rx + m[4] * ry - m[8]; val dy = m[1] * rx + m[5] * ry - m[9]; val dz = m[2] * rx + m[6] * ry - m[10]
            val ox = m[12]; val oy = m[13]; val oz = m[14]
            var best = Float.MAX_VALUE
            if (dy < -1e-6f) { val t = -oy / dy; val x = ox + dx * t; if (t > 0 && abs(x) < 6f) best = min(best, t) }
            if (dz < -1e-6f) { val t = (WALL_Z - oz) / dz; val y = oy + dy * t; if (t > 0 && y > 0f && y < 3f) best = min(best, t) }
            // box: slabs
            var t0 = 0f; var t1 = Float.MAX_VALUE
            val o = floatArrayOf(ox, oy, oz); val d = floatArrayOf(dx, dy, dz)
            var hit = true
            for (k in 0 until 3) {
                if (abs(d[k]) < 1e-9f) { if (o[k] < BOX_MIN[k] || o[k] > BOX_MAX[k]) hit = false; continue }
                var a = (BOX_MIN[k] - o[k]) / d[k]; var b = (BOX_MAX[k] - o[k]) / d[k]
                if (a > b) { val s = a; a = b; b = s }
                t0 = max(t0, a); t1 = min(t1, b)
            }
            if (hit && t0 <= t1 && t0 > 0) best = min(best, t0)
            return if (best == Float.MAX_VALUE) 0f else best
        }

        fun trueDepth(m: FloatArray): FloatArray = FloatArray(W * H) { castDepth(m, (it % W).toFloat(), (it / W).toFloat()) }

        /** Distance of a world point to the nearest true surface. */
        fun surfaceDist(x: Float, y: Float, z: Float): Float {
            val qx = max(BOX_MIN[0] - x, x - BOX_MAX[0]); val qy = max(BOX_MIN[1] - y, y - BOX_MAX[1]); val qz = max(BOX_MIN[2] - z, z - BOX_MAX[2])
            val out = sqrt(max(qx, 0f).let { it * it } + max(qy, 0f).let { it * it } + max(qz, 0f).let { it * it })
            val box = abs(out + min(max(qx, max(qy, qz)), 0f))
            return min(min(abs(y), abs(z - WALL_Z)), box)
        }

        class Noisy(val mm: ShortArray, val conf: ByteArray, val flying: BooleanArray, val speckle: BooleanArray, val truth: FloatArray)

        /** Depth-from-motion-like corruption of the true depth. */
        fun corrupt(truth: FloatArray, rnd: Random, speckleFrac: Float = 0.004f): Noisy {
            val n = W * H
            val mm = ShortArray(n); val conf = ByteArray(n); val flying = BooleanArray(n); val speckle = BooleanArray(n)
            for (i in 0 until n) {
                val d = truth[i]
                if (d <= 0f) continue
                val u = i % W; val v = i / W
                // depth discontinuity next to this pixel -> smear between foreground and background
                var other = d
                for (dv in -1..1) for (du in -1..1) {
                    val uu = u + du; val vv = v + dv
                    if (uu < 0 || vv < 0 || uu >= W || vv >= H) continue
                    val e = truth[vv * W + uu]
                    if (e > 0f && abs(e - d) > 0.15f * d && abs(e - d) > abs(other - d)) other = e
                }
                var meas = d * (1f + 0.01f * rnd.nextGaussian().toFloat())
                var c = 150 + rnd.nextInt(106)
                if (other != d && rnd.nextFloat() < 0.7f) {
                    meas = d + (other - d) * (0.25f + 0.5f * rnd.nextFloat())
                    flying[i] = true
                    c = 60 + rnd.nextInt(196)
                }
                if (rnd.nextFloat() < speckleFrac) {
                    meas = 0.4f + 3.4f * rnd.nextFloat(); speckle[i] = true; c = rnd.nextInt(256)
                }
                mm[i] = (meas * 1000f).toInt().coerceIn(1, 65535).toShort()
                conf[i] = c.toByte()
            }
            return Noisy(mm, conf, flying, speckle, truth)
        }

        val TARGET = floatArrayOf(0f, 0.25f, -1.5f)

        /** Cameras on an arc in front of the box, 1.3 m high, 1.9 m from it. */
        fun poses(n: Int): List<FloatArray> = (0 until n).map { k ->
            val a = Math.toRadians(-65.0 + 130.0 * k / max(1, n - 1))
            lookAt(floatArrayOf((1.9 * kotlin.math.sin(a)).toFloat(), 1.3f, (-1.5 + 1.9 * kotlin.math.cos(a)).toFloat()), TARGET)
        }
    }

    @Test fun lookAtFollowsArcoreConvention() {
        val m = lookAt(floatArrayOf(0f, 1f, 0f), floatArrayOf(0f, 1f, -2f))
        // looking down -z: camera x = world x, camera y = world y, -camera z = world -z
        assertEquals(1f, m[0], 1e-5f); assertEquals(1f, m[5], 1e-5f); assertEquals(1f, m[10], 1e-5f)
        // centre pixel hits the wall at 3 m
        assertEquals(3f, castDepth(m, CX, CY), 1e-4f)
        // top rows look up (pixel rows top-down)
        val top = castDepth(m, CX, 0f); val bottom = castDepth(m, CX, H - 1f)
        assertTrue(bottom < top)   // the bottom of the image sees the floor, nearer
    }

    @Test fun filterRemovesFlyingPixelsAndSpeckle() {
        val rnd = Random(7)
        val m = poses(5)[2]
        val truth = trueDepth(m)
        val nz = corrupt(truth, rnd)
        val f = DepthFilter()
        val out = f.filter(nz.mm, nz.conf, W, H, FX, FY, CX, CY, m)
        var badIn = 0; var badOut = 0; var goodIn = 0; var goodKept = 0; var flyIn = 0; var flyOut = 0
        for (i in 0 until W * H) {
            val t = truth[i]; if (t <= 0f || t > 4f) continue
            val meas = (nz.mm[i].toInt() and 0xffff) / 1000f
            val bad = abs(meas - t) > 0.05f * t
            if (bad) { badIn++; if (out[i] > 0f) badOut++ } else { goodIn++; if (out[i] > 0f) goodKept++ }
            if (nz.flying[i] && abs(meas - t) > 0.05f * t) { flyIn++; if (out[i] > 0f && abs(out[i] - t) > 0.05f * t) flyOut++ }
        }
        println("filter: bad px $badIn -> $badOut, flying $flyIn -> $flyOut, good kept $goodKept / $goodIn; stats ${f.last.toJson()}")
        assertTrue("flying pixels in the test data", flyIn > 100)
        assertTrue("flying pixels removed: $flyOut of $flyIn left", flyOut <= flyIn * 0.08)
        assertTrue("bad pixels removed: $badOut of $badIn left", badOut <= badIn * 0.08)
        assertTrue("good pixels kept: $goodKept of $goodIn", goodKept >= goodIn * 0.8)
        // every surviving pixel within 5 % of the truth (or the median moved it a little)
        var far = 0; var kept = 0
        for (i in 0 until W * H) if (out[i] > 0f) { kept++; if (abs(out[i] - truth[i]) > 0.06f * truth[i]) far++ }
        assertTrue("outliers after filtering: $far / $kept", far <= kept * 0.01)
    }

    @Test fun filterDropsLowConfidenceAndRange() {
        val mm = ShortArray(W * H) { 2000 }
        val conf = ByteArray(W * H) { 200.toByte() }
        for (i in 0 until W * H step 7) conf[i] = 50
        mm[W * 60 + 80] = 100     // 0.1 m: too near
        mm[W * 30 + 31] = 6000    // 6 m: too far
        val f = DepthFilter(DepthFilter.Config(temporal = false))
        val out = f.filter(mm, conf, W, H, FX, FY, CX, CY, null)
        assertEquals(0f, out[0], 0f)                    // conf 50 < 128
        assertEquals(0f, out[W * 60 + 80], 0f)
        assertEquals(0f, out[W * 30 + 31], 0f)
        assertEquals(2f, out[W * 10 + 11], 1e-6f)
        assertTrue(f.last.lowConf > 0 && f.last.range == 2)
        // smoothed depth path (no confidence) keeps the low-confidence pixels
        val g = DepthFilter(DepthFilter.Config(temporal = false)).filter(mm, null, W, H, FX, FY, CX, CY, null)
        assertEquals(2f, g[0], 1e-6f)
    }

    @Test fun temporalCheckDropsPixelsThePreviousViewSawThrough() {
        val m = lookAt(floatArrayOf(0f, 1f, 0f), floatArrayOf(0f, 1f, -2f))
        val wall = ShortArray(W * H) { 3000 }
        val f = DepthFilter(DepthFilter.Config(maxGrazingDeg = 90f))
        f.filter(wall, null, W, H, FX, FY, CX, CY, m)
        // second frame: a floating 10x10 patch at 1.5 m that the first frame saw through
        val blob = wall.copyOf()
        for (v in 50 until 60) for (u in 70 until 80) blob[v * W + u] = 1500
        val out = f.filter(blob, null, W, H, FX, FY, CX, CY, m)
        assertEquals(0f, out[55 * W + 75], 0f)
        assertEquals(3f, out[20 * W + 20], 1e-3f)
        assertTrue(f.last.temporal >= 36)
    }

    @Test fun marchingCubesTableIsClosedAndOriented() {
        // every case: triangles only use cut edges, and every cut edge is used
        for (mask in 1 until 255) {
            val cut = HashSet<Int>()
            for (e in 0 until 12) {
                val a = TsdfVolume.EDGES[e * 2]; val b = TsdfVolume.EDGES[e * 2 + 1]
                if (((mask shr a) and 1) != ((mask shr b) and 1)) cut.add(e)
            }
            assertEquals("case $mask", cut, TsdfVolume.TRI[mask].filter { it < 12 }.toHashSet() + TsdfVolume.CENTROID_POLYS[mask].flatMap { it.toList() })
        }
        // random fields with a positive border -> closed, consistently oriented, manifold surfaces
        val rnd = Random(3)
        repeat(20) { rep ->
            val vol = TsdfVolume(TsdfVolume.Config(voxel = 0.1f, minExtractWeight = 1f))
            val n = 14
            for (z in -1..n) for (y in -1..n) for (x in -1..n) {
                val border = x <= 0 || y <= 0 || z <= 0 || x >= n - 1 || y >= n - 1 || z >= n - 1
                vol.setVoxel(x, y, z, if (border) 0.5f else rnd.nextFloat() - 0.5f + (if (rep % 2 == 0) 0.2f else -0.1f), 1f)
            }
            val m = vol.extract()
            assertTrue(m.triangleCount > 0)
            val directed = HashMap<Long, Int>()
            for (t in 0 until m.triangleCount) for (c in 0 until 3) {
                val a = m.tris[t * 3 + c].toLong(); val b = m.tris[t * 3 + (c + 1) % 3].toLong()
                val k = (a shl 32) or b
                directed[k] = (directed[k] ?: 0) + 1
            }
            for ((k, cnt) in directed) {
                assertEquals("directed edge used once", 1, cnt)
                val rev = ((k and 0xffffffffL) shl 32) or (k ushr 32)
                assertEquals("edge has an opposite twin (closed surface)", 1, directed[rev] ?: 0)
            }
        }
    }

    @Test fun marchingCubesSphereNormalsPointOutwards() {
        val vox = 0.05f
        val vol = TsdfVolume(TsdfVolume.Config(voxel = vox, minExtractWeight = 1f))
        val r = 0.4f
        for (z in -12..12) for (y in -12..12) for (x in -12..12) {
            val d = sqrt((x * x + y * y + z * z).toFloat()) * vox - r
            vol.setVoxel(x, y, z, (d / (4 * vox)).coerceIn(-1f, 1f), 5f)
        }
        val m = vol.extract()
        var maxErr = 0f; var inward = 0
        for (i in 0 until m.vertexCount) {
            val x = m.xyz[i * 3]; val y = m.xyz[i * 3 + 1]; val z = m.xyz[i * 3 + 2]
            val l = sqrt(x * x + y * y + z * z)
            maxErr = max(maxErr, abs(l - r))
            if (x * m.nrm[i * 3] + y * m.nrm[i * 3 + 1] + z * m.nrm[i * 3 + 2] < 0) inward++
        }
        assertTrue("sphere vertex error $maxErr", maxErr < vox * 0.3f)
        assertEquals(0, inward)
        assertEquals(1, MeshOps.triangleComponents(m).second)
    }

    private fun fuseScene(nViews: Int, rnd: Random, fusion: DepthFusion) {
        for (m in poses(nViews)) {
            val nz = corrupt(trueDepth(m), rnd)
            fusion.process(DepthFusion.Frame(nz.mm, nz.conf, W, H, null, 0, 0,
                floatArrayOf(FX, FY, CX, CY), W, H, m, 0L))
        }
    }

    @Test fun tsdfReconstructsBoxSeparatedAndClean() {
        val rnd = Random(11)
        val fusion = DepthFusion(MapBuilder(), tsdfIntervalMs = 0, liveMeshIntervalMs = Long.MAX_VALUE)
        fusion.map!!.floorY = 0f
        fuseScene(12, rnd, fusion)
        // a few frames of pure noise blobs (high confidence speckle) that no other view confirms
        val r = fusion.buildResult(0f)
        val m = r.mesh
        val vox = fusion.tsdfCfg.voxel
        println("mesh: ${m.vertexCount} vertices, ${m.triangleCount} triangles (${r.rawTriangles} before component filter), " +
            "${r.points.size / 3} points, planes ${r.seg.planes.size}, objects ${r.seg.objects.map { "${it.triangles} tris ${it.min.toList()}..${it.max.toList()}" }}, ${r.ms} ms")
        assertTrue(m.triangleCount > 5000)
        // vertices on the true surfaces
        var within = 0; var worst = 0f
        for (i in 0 until m.vertexCount) {
            val d = surfaceDist(m.xyz[i * 3], m.xyz[i * 3 + 1], m.xyz[i * 3 + 2])
            if (d <= vox) within++
            worst = max(worst, d)
        }
        println("vertices within 1 voxel: $within / ${m.vertexCount}, worst %.3f m".format(worst))
        assertTrue("vertices within one voxel: $within / ${m.vertexCount}", within >= m.vertexCount * 0.97)
        assertTrue("no floating blobs left (worst vertex %.3f m)".format(worst), worst < 0.10f)
        // manifold: no edge shared by more than two triangles, few open edges (scene border only)
        val und = HashMap<Long, Int>()
        for (t in 0 until m.triangleCount) for (c in 0 until 3) {
            val a = m.tris[t * 3 + c]; val b = m.tris[t * 3 + (c + 1) % 3]
            val k = (min(a, b).toLong() shl 32) or max(a, b).toLong()
            und[k] = (und[k] ?: 0) + 1
        }
        val open = und.values.count { it == 1 }; val nonManifold = und.values.count { it > 2 }
        println("edges ${und.size}, open $open, non-manifold $nonManifold")
        assertEquals(0, nonManifold)
        assertTrue("open edges $open of ${und.size}", open < und.size * 0.05)
        // box = its own object after removing the floor / wall planes
        assertTrue(r.seg.planes.size >= 2)
        val box = r.seg.objects.maxByOrNull { it.triangles }
        assertNotNull(box)
        for (k in 0 until 3) {
            if (k == 1) { assertEquals("box top", BOX_MAX[1], box!!.max[1], 2 * vox); continue }
            assertEquals("box min[$k]", BOX_MIN[k], box!!.min[k], 2 * vox)
            assertEquals("box max[$k]", BOX_MAX[k], box.max[k], 2 * vox)
        }
        // everything else is small next to the box (no wall/floor seams or blobs as "objects")
        val others = r.seg.objects.filter { it !== box }.sumOf { it.triangles }
        assertTrue("other objects: $others triangles", others < box!!.triangles * 0.1)
        // points: every cleaned point near a surface
        var badPts = 0
        for (i in 0 until r.points.size / 3) if (surfaceDist(r.points[i * 3], r.points[i * 3 + 1], r.points[i * 3 + 2]) > vox) badPts++
        assertTrue("points off-surface: $badPts", badPts <= r.points.size / 3 / 100)

        // outputs round trip through the viewer's PLY reader
        val dir = File(System.getProperty("java.io.tmpdir"), "r2s_recon_test_" + System.nanoTime())
        try {
            fusion.writeOutputs(r, dir)
            val ply = Ply.read(File(dir, "mesh.ply"))!!
            assertEquals(m.vertexCount, ply.vertexCount)
            assertEquals(m.triangleCount, ply.tris.size / 3)
            assertNotNull(ply.rgb)
            val pts = Ply.read(File(dir, "points.ply"))!!
            assertEquals(r.points.size / 3, pts.vertexCount)
            assertTrue(File(dir, "objects.json").readText().contains("\"objects\""))
            val desc = org.json.JSONObject(File(dir, "recon.json").readText())
            assertEquals(0.5, desc.getJSONObject("filter").getDouble("min_confidence"), 1e-6)
        } finally { dir.deleteRecursively() }

        // occupancy grid still built (from filtered depth): the box footprint is occupied
        val map = fusion.map!!
        println("occupancy: known %.1f m2, box top cell %d, in front %d".format(map.knownAreaM2(),
            map.occ[map.cellJ(-1.5f) * map.size + map.cellI(0f)], map.occ[map.cellJ(-0.6f) * map.size + map.cellI(0f)]))
        assertTrue(map.isOcc(map.cellI(0f), map.cellJ(-1.5f)))
        assertTrue(map.isFree(map.cellI(0f), map.cellJ(-0.5f)) || map.isUnknown(map.cellI(0f), map.cellJ(-0.5f)))

        renderPng(r, fusion)
    }

    @Test fun smallNoiseBlobsAreRemoved() {
        val vol = TsdfVolume(TsdfVolume.Config(voxel = 0.03f, minExtractWeight = 1f))
        // a big slab and a 2-voxel blob far away
        for (z in 0..30) for (x in 0..30) for (y in -3..3) vol.setVoxel(x, y, z, (y + 0.5f) / 4f, 3f)
        for (z in 60..63) for (y in 10..13) for (x in 60..63) vol.setVoxel(x, y, z, if (x in 61..62 && y in 11..12 && z in 61..62) -0.3f else 0.3f, 3f)
        val m = vol.extract()
        assertEquals(2, MeshOps.triangleComponents(m).second)
        val clean = MeshOps.removeSmallComponents(m, 120)
        assertEquals(1, MeshOps.triangleComponents(clean).second)
        for (i in 0 until clean.vertexCount) assertTrue(clean.xyz[i * 3] < 1.0f)
        // statistical outlier removal on points: isolated points go, a dense patch stays
        val pts = FloatList()
        for (i in 0 until 30) for (j in 0 until 30) pts.add3(i * 0.03f, 0f, j * 0.03f)
        pts.add3(3f, 2f, 1f); pts.add3(-2f, 1f, 0f)
        val keep = MeshOps.statisticalOutliers(pts.toArray(), 8, 0.09f, 2f)
        assertTrue(keep.take(900).count { it } >= 870)
        assertTrue(!keep[900] && !keep[901])
    }

    @Test fun integrationTiming() {
        val rnd = Random(5)
        val ps = poses(10)
        val frames = ps.map { corrupt(trueDepth(it), rnd) }
        val vol = TsdfVolume()
        val f = DepthFilter()
        // warm up the JIT
        repeat(3) { r -> ps.forEachIndexed { i, m -> vol.integrate(f.filter(frames[i].mm, frames[i].conf, W, H, FX, FY, CX, CY, m), W, H, FX, FY, CX, CY, m) }; if (r < 2) vol.clear() }
        // best of 5 rounds (each the mean over 10 views), so a busy build machine does not fail the check
        var filterMs = Double.MAX_VALUE; var integrateMs = Double.MAX_VALUE
        repeat(5) {
            var tf = 0L; var ti = 0L
            ps.forEachIndexed { i, m ->
                val t0 = System.nanoTime()
                val d = f.filter(frames[i].mm, frames[i].conf, W, H, FX, FY, CX, CY, m)
                val t1 = System.nanoTime()
                vol.integrate(d, W, H, FX, FY, CX, CY, m)
                ti += System.nanoTime() - t1; tf += t1 - t0
            }
            filterMs = min(filterMs, tf / 1e6 / ps.size); integrateMs = min(integrateMs, ti / 1e6 / ps.size)
        }
        val t0 = System.nanoTime()
        val mesh = vol.extract()
        val extractMs = (System.nanoTime() - t0) / 1e6
        println("timing per 160x120 frame: filter %.2f ms, TSDF integrate %.2f ms; full extraction %.1f ms (%d blocks, %d triangles)"
            .format(filterMs, integrateMs, extractMs, vol.blockCount, mesh.triangleCount))
        assertTrue("integrate $integrateMs ms", integrateMs < 30.0)
        assertTrue("filter + integrate ${filterMs + integrateMs} ms", filterMs + integrateMs < 30.0)
    }

    @Test fun turboDepthColoursAndYuv() {
        val near = DepthViz.color(0.2f, 5f); val far = DepthViz.color(5f, 5f)
        assertTrue((near shr 16 and 255) > (near and 255))      // near: red dominant
        assertTrue((far and 255) > (far shr 16 and 255))         // far: blue dominant
        val px = DepthViz.colorizeMeters(floatArrayOf(0f, 1f))
        assertEquals(0, px[0]); assertEquals(0xff, px[1] ushr 24)
        // grey YUV image (Y=128, U=V=128) -> grey
        val y = ByteBuffer.wrap(ByteArray(64 * 48) { 128.toByte() }); val uv = ByteBuffer.wrap(ByteArray(32 * 24 * 2) { 128.toByte() })
        val rgb = YuvSampler.sample(y, 64, 1, uv, uv, 64, 2, 64, 48, 16, 12)
        val c = rgb[5]
        assertTrue(abs((c shr 16 and 255) - (c and 255)) < 3 && abs((c shr 8 and 255) - (c and 255)) < 3)
    }

    // ------------------------------------------------------------------ visual check
    /**
     * Software rasteriser: left = the old pipeline (every raw depth pixel of all views as a point, as
     * MapBuilder dumped them), right = filtered TSDF mesh (lit, height colours, small components removed).
     * Written to build/recon_render.png, and to $R2S_RENDER_PNG when set.
     */
    private fun renderPng(r: DepthFusion.Result, fusion: DepthFusion) {
        val iw = 640; val ih = 480
        val img = Img(iw * 2, ih)
        val eye = floatArrayOf(2.3f, 2.0f, 0.6f); val tgt = floatArrayOf(0f, 0.3f, -1.8f)
        val cam = lookAt(eye, tgt)
        val f = 520f
        fun project(x: Float, y: Float, z: Float): FloatArray? {
            val dx = x - cam[12]; val dy = y - cam[13]; val dz = z - cam[14]
            val xc = cam[0] * dx + cam[1] * dy + cam[2] * dz
            val yc = cam[4] * dx + cam[5] * dy + cam[6] * dz
            val zc = -(cam[8] * dx + cam[9] * dy + cam[10] * dz)
            if (zc < 0.05f) return null
            return floatArrayOf(f * xc / zc + iw / 2f, ih / 2f - f * yc / zc, zc)
        }
        for (p in 0 until iw * 2 * ih) img.setRGB(p % (iw * 2), p / (iw * 2), 0x101114)
        // left: raw points
        val zb = FloatArray(iw * ih) { Float.MAX_VALUE }
        val rnd = Random(11)
        for (m in poses(12)) {
            val nz = corrupt(trueDepth(m), rnd)
            for (i in 0 until W * H step 2) {
                val d = (nz.mm[i].toInt() and 0xffff) / 1000f
                if (d <= 0f || d > 4f) continue
                val u = i % W; val v = i / W
                val xc = (u - CX) / FX * d; val yc = -(v - CY) / FY * d; val zc = -d
                val wx = m[0] * xc + m[4] * yc + m[8] * zc + m[12]; val wy = m[1] * xc + m[5] * yc + m[9] * zc + m[13]; val wz = m[2] * xc + m[6] * yc + m[10] * zc + m[14]
                val q = project(wx, wy, wz) ?: continue
                val px = q[0].toInt(); val py = q[1].toInt()
                if (px < 0 || py < 0 || px >= iw || py >= ih || q[2] > zb[py * iw + px]) continue
                zb[py * iw + px] = q[2]
                img.setRGB(px, py, MeshOps.heightColor(wy, 0f))
            }
        }
        // right: mesh
        zb.fill(Float.MAX_VALUE)
        val m = r.mesh
        val lx = 0.3f; val ly = 1f; val lz = 0.5f; val ll = sqrt(lx * lx + ly * ly + lz * lz)
        for (t in 0 until m.triangleCount) {
            val a = m.tris[t * 3]; val b = m.tris[t * 3 + 1]; val c = m.tris[t * 3 + 2]
            val pa = project(m.xyz[a * 3], m.xyz[a * 3 + 1], m.xyz[a * 3 + 2]) ?: continue
            val pb = project(m.xyz[b * 3], m.xyz[b * 3 + 1], m.xyz[b * 3 + 2]) ?: continue
            val pc = project(m.xyz[c * 3], m.xyz[c * 3 + 1], m.xyz[c * 3 + 2]) ?: continue
            val x0 = max(0, min(pa[0], min(pb[0], pc[0])).toInt()); val x1 = min(iw - 1, max(pa[0], max(pb[0], pc[0])).toInt() + 1)
            val y0 = max(0, min(pa[1], min(pb[1], pc[1])).toInt()); val y1 = min(ih - 1, max(pa[1], max(pb[1], pc[1])).toInt() + 1)
            val area = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pc[0] - pa[0]) * (pb[1] - pa[1])
            if (abs(area) < 1e-6f) continue
            for (py in y0..y1) for (px in x0..x1) {
                val sx = px + 0.5f; val sy = py + 0.5f
                val w0 = ((pb[0] - sx) * (pc[1] - sy) - (pc[0] - sx) * (pb[1] - sy)) / area
                val w1 = ((pc[0] - sx) * (pa[1] - sy) - (pa[0] - sx) * (pc[1] - sy)) / area
                val w2 = 1 - w0 - w1
                if (w0 < 0 || w1 < 0 || w2 < 0) continue
                val z = w0 * pa[2] + w1 * pb[2] + w2 * pc[2]
                if (z > zb[py * iw + px]) continue
                zb[py * iw + px] = z
                var nx = 0f; var ny = 0f; var nz = 0f; var hy = 0f
                for ((v, wv) in listOf(a to w0, b to w1, c to w2)) { nx += m.nrm[v * 3] * wv; ny += m.nrm[v * 3 + 1] * wv; nz += m.nrm[v * 3 + 2] * wv; hy += m.xyz[v * 3 + 1] * wv }
                val nl = sqrt(nx * nx + ny * ny + nz * nz).coerceAtLeast(1e-6f)
                val shade = 0.3f + 0.7f * abs(nx * lx + ny * ly + nz * lz) / (nl * ll)
                val hc = MeshOps.heightColor(hy, r.floorY)
                val col = (((hc shr 16 and 255) * shade).toInt() shl 16) or (((hc shr 8 and 255) * shade).toInt() shl 8) or ((hc and 255) * shade).toInt()
                img.setRGB(iw + px, py, col)
            }
        }
        for (py in 0 until ih) img.setRGB(iw, py, 0x606060)
        val outs = mutableListOf(File("build/recon_render.png"))
        System.getenv("R2S_RENDER_PNG")?.let { outs.add(File(it)) }
        for (o in outs) { o.absoluteFile.parentFile?.mkdirs(); o.writeBytes(img.png()) }
        println("render: ${outs.joinToString { it.absolutePath }} (${fusion.tsdf.blockCount} blocks)")
    }
}

/** Minimal RGB image + PNG encoder (no java.awt in Android unit tests). */
class Img(val w: Int, val h: Int) {
    val px = IntArray(w * h)
    fun setRGB(x: Int, y: Int, c: Int) { px[y * w + x] = c }
    fun png(): ByteArray {
        val raw = java.io.ByteArrayOutputStream()
        for (y in 0 until h) {
            raw.write(0)
            for (x in 0 until w) { val c = px[y * w + x]; raw.write(c shr 16 and 255); raw.write(c shr 8 and 255); raw.write(c and 255) }
        }
        val out = java.io.ByteArrayOutputStream()
        out.write(byteArrayOf(0x89.toByte(), 'P'.code.toByte(), 'N'.code.toByte(), 'G'.code.toByte(), 13, 10, 26, 10))
        fun chunk(type: String, data: ByteArray) {
            val d = java.io.DataOutputStream(out)
            d.writeInt(data.size)
            val td = type.toByteArray() + data
            d.write(td)
            val crc = java.util.zip.CRC32(); crc.update(td); d.writeInt(crc.value.toInt())
        }
        val ihdr = ByteBuffer.allocate(13).putInt(w).putInt(h).put(8).put(2).put(0).put(0).put(0).array()
        chunk("IHDR", ihdr)
        val z = java.io.ByteArrayOutputStream()
        java.util.zip.DeflaterOutputStream(z).use { it.write(raw.toByteArray()) }
        chunk("IDAT", z.toByteArray())
        chunk("IEND", ByteArray(0))
        return out.toByteArray()
    }
}
