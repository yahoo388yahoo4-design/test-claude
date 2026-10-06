package com.real2sim.capture

import org.json.JSONObject
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min

/**
 * Sparse truncated signed distance volume (voxel hashing): blocks of 8^3 voxels allocated around observed
 * surfaces, each voxel holding a truncated SDF (in units of the truncation distance, +1 = free space in
 * front of the surface, < 0 = behind it), a weight (running average, capped) and optionally an RGB colour.
 *
 * Integration (per filtered depth map + camera pose, ARCore OpenGL camera convention: +x right, +y up,
 * -z forward; depth image rows top-down):
 *  - blocks are allocated along each depth ray within +-truncation of the measured depth;
 *  - existing blocks between the camera and the surface are also visited (free-space carving), so blobs
 *    that later views see through are erased;
 *  - every voxel of every visited block is projected into the depth map: sdf = depth - voxel depth,
 *    skipped when sdf < -truncation, else tsdf = min(1, sdf / trunc) averaged with weight
 *    w = clamp((1.5 m / depth)^2, 0.25, 1) (depth-from-motion noise grows ~ depth^2), weights capped at
 *    [Config.maxWeight] so the map can still adapt.
 *
 * Extraction: marching cubes over voxels with weight >= [Config.minExtractWeight]. The case table is
 * generated (not hand-typed): on each cube face the iso-line segments are fixed by the face's own corner
 * signs (ambiguous faces always separate the inside corners), so neighbouring cubes agree and the mesh is
 * crack free; segments are chained into polygons, oriented towards free space and fan triangulated.
 * Vertices are shared through a hash of the grid edge they lie on (indexed mesh -> components, normals).
 *
 * Not thread safe: integrate and extract from one thread (DepthFusion's worker).
 */
class TsdfVolume(val cfg: Config = Config()) {
    data class Config(
        val voxel: Float = 0.03f,
        val truncVoxels: Float = 4f,
        val maxWeight: Float = 64f,
        val maxBlocks: Int = 8000,
        val minExtractWeight: Float = 2f,
        val allocStepPx: Int = 2,
        val carveStepPx: Int = 4,
        val color: Boolean = true,
    ) {
        val trunc get() = voxel * truncVoxels
        val blockSize get() = voxel * B
        fun toJson(): JSONObject = JSONObject().apply {
            put("voxel_m", voxel.toDouble()); put("block_voxels", B); put("truncation_voxels", truncVoxels.toDouble())
            put("max_weight", maxWeight.toDouble()); put("max_blocks", maxBlocks); put("min_extract_weight", minExtractWeight.toDouble())
            put("alloc_step_px", allocStepPx); put("carve_step_px", carveStepPx); put("color", color)
        }
    }

    private val index = LongIntMap(4096)
    private val coords = IntList(4096 * 3)
    private val tsdf = ArrayList<FloatArray>()
    private val weight = ArrayList<FloatArray>()
    private val rgb = ArrayList<ByteArray?>()
    /** Per-block colour weight (samples that contributed colour, capped at [MAX_COLOR_WEIGHT]); allocated with [rgb]. */
    private val cweight = ArrayList<FloatArray?>()
    private var stamp = IntArray(1024)
    private var dirty = BooleanArray(1024)
    private var frame = 0
    private val touched = IntList(1024)

    val blockCount get() = tsdf.size
    var full = false; private set
    var integratedFrames = 0; private set
    val bytesUsed: Long get() = tsdf.size.toLong() * BV * 8 + rgb.count { it != null } * BV * 7L   // rgb 3 B + colour weight 4 B

    fun blockIndex(bx: Int, by: Int, bz: Int): Int = index.get(MeshOps.key(bx, by, bz))

    private fun alloc(bx: Int, by: Int, bz: Int): Int {
        val k = MeshOps.key(bx, by, bz)
        val i = index.get(k)
        if (i >= 0) return i
        if (tsdf.size >= cfg.maxBlocks) { full = true; return -1 }
        val n = tsdf.size
        index.put(k, n)
        coords.add(bx); coords.add(by); coords.add(bz)
        tsdf.add(FloatArray(BV) { 1f }); weight.add(FloatArray(BV)); rgb.add(null); cweight.add(null)
        if (n >= stamp.size) { stamp = stamp.copyOf(stamp.size * 2); dirty = dirty.copyOf(dirty.size * 2) }
        stamp[n] = 0; dirty[n] = false
        return n
    }

    private fun touch(b: Int) { if (b >= 0 && stamp[b] != frame) { stamp[b] = frame; touched.add(b) } }

    /**
     * Integrates one filtered depth map ([depth] metres, 0 = invalid, w*h, rows top-down) seen from [pose]
     * (column-major camera-to-world). [rgb] optional w*h 0xRRGGBB aligned with the depth. Returns the
     * number of blocks updated.
     */
    fun integrate(depth: FloatArray, w: Int, h: Int, fx: Float, fy: Float, cx: Float, cy: Float, pose: FloatArray, rgb: IntArray? = null): Int {
        frame++
        touched.size = 0
        val m = pose
        val tx = m[12]; val ty = m[13]; val tz = m[14]
        val trunc = cfg.trunc
        val bs = cfg.blockSize
        val inv = 1f / bs
        // 1. allocate along the surface band, 2. visit existing blocks in front of the surface (carving)
        val step = max(1, cfg.allocStepPx)
        val carve = cfg.carveStepPx
        var v = 0
        while (v < h) {
            var u = 0
            while (u < w) {
                val d = depth[v * w + u]
                if (d > 0f) {
                    val rx = (u - cx) / fx; val ry = -(v - cy) / fy
                    val dx = m[0] * rx + m[4] * ry - m[8]; val dy = m[1] * rx + m[5] * ry - m[9]; val dz = m[2] * rx + m[6] * ry - m[10]
                    var s = d - trunc
                    while (s <= d + trunc + 1e-4f) {
                        touch(alloc(floor((tx + dx * s) * inv).toInt(), floor((ty + dy * s) * inv).toInt(), floor((tz + dz * s) * inv).toInt()))
                        s += bs * 0.5f
                    }
                    if (carve > 0 && u % carve == 0 && v % carve == 0) {
                        s = 0.15f
                        while (s < d - trunc) {
                            touch(blockIndex(floor((tx + dx * s) * inv).toInt(), floor((ty + dy * s) * inv).toInt(), floor((tz + dz * s) * inv).toInt()))
                            s += bs * 0.5f
                        }
                    }
                }
                u += step
            }
            v += step
        }
        // 3. project every voxel of the visited blocks
        val r00 = m[0]; val r10 = m[1]; val r20 = m[2]
        val r01 = m[4]; val r11 = m[5]; val r21 = m[6]
        val r02 = m[8]; val r12 = m[9]; val r22 = m[10]
        val ox = -(r00 * tx + r10 * ty + r20 * tz); val oy = -(r01 * tx + r11 * ty + r21 * tz); val oz = -(r02 * tx + r12 * ty + r22 * tz)
        val vox = cfg.voxel
        val maxW = cfg.maxWeight
        val useColor = rgb != null && cfg.color
        for (q in 0 until touched.size) {
            val b = touched.a[q]
            val T = tsdf[b]; val W = weight[b]
            var C = this.rgb[b]; var CW = cweight[b]
            val gx0 = coords.a[b * 3] * B; val gy0 = coords.a[b * 3 + 1] * B; val gz0 = coords.a[b * 3 + 2] * B
            var updated = false
            for (lz in 0 until B) {
                val pz = (gz0 + lz) * vox
                for (ly in 0 until B) {
                    val py = (gy0 + ly) * vox
                    for (lx in 0 until B) {
                        val px = (gx0 + lx) * vox
                        val zc = -(r02 * px + r12 * py + r22 * pz + oz)
                        if (zc < 0.05f) continue
                        val xc = r00 * px + r10 * py + r20 * pz + ox
                        val yc = r01 * px + r11 * py + r21 * pz + oy
                        val ui = (fx * xc / zc + cx + 0.5f).toInt(); val vi = (cy - fy * yc / zc + 0.5f).toInt()
                        if (ui < 0 || vi < 0 || ui >= w || vi >= h) continue
                        val pi = vi * w + ui
                        val dm = depth[pi]
                        if (dm <= 0f) continue
                        val sdf = dm - zc
                        if (sdf < -trunc) continue
                        val obs = min(1f, sdf / trunc)
                        val wo = (2.25f / (dm * dm)).coerceIn(0.25f, 1f)
                        val i = lx + B * (ly + B * lz)
                        val w0 = W[i]
                        val w1 = w0 + wo
                        T[i] = (T[i] * w0 + obs * wo) / w1
                        W[i] = min(w1, maxW)
                        if (useColor && abs(sdf) < trunc * 0.5f) {
                            if (C == null) { C = ByteArray(BV * 3); this.rgb[b] = C; CW = FloatArray(BV); cweight[b] = CW }
                            val c = rgb!![pi]
                            // colour has its own weight: the first sample is stored exactly (the geometry weight
                            // may already be high from frames that saw the voxel without colouring it)
                            val wc = CW!![i]
                            val k = i * 3
                            val f0 = wc / (wc + wo); val f1 = wo / (wc + wo)
                            C[k] = ((C[k].toInt() and 255) * f0 + (c shr 16 and 255) * f1 + 0.5f).toInt().toByte()
                            C[k + 1] = ((C[k + 1].toInt() and 255) * f0 + (c shr 8 and 255) * f1 + 0.5f).toInt().toByte()
                            C[k + 2] = ((C[k + 2].toInt() and 255) * f0 + (c and 255) * f1 + 0.5f).toInt().toByte()
                            CW[i] = min(wc + wo, MAX_COLOR_WEIGHT)
                        }
                        updated = true
                    }
                }
            }
            if (updated) markDirty(b)
        }
        integratedFrames++
        return touched.size
    }

    /** The block and the blocks whose cubes read its voxels (the -x/-y/-z neighbours) need re-extraction. */
    private fun markDirty(b: Int) {
        val bx = coords.a[b * 3]; val by = coords.a[b * 3 + 1]; val bz = coords.a[b * 3 + 2]
        for (d in 0 until 8) {
            val n = blockIndex(bx - (d and 1), by - (d shr 1 and 1), bz - (d shr 2 and 1))
            if (n >= 0) dirty[n] = true
        }
    }

    /** Blocks changed since the last call (clears the flags), at most [max] (the rest stay dirty). */
    fun takeDirty(max: Int = Int.MAX_VALUE): IntArray {
        val out = IntList()
        for (b in 0 until tsdf.size) if (dirty[b] && out.size < max) { dirty[b] = false; out.add(b) }
        return out.toArray()
    }

    /** Sets one voxel (allocating its block); for tests and synthetic volumes. */
    fun setVoxel(gx: Int, gy: Int, gz: Int, value: Float, w: Float) {
        val b = alloc(Math.floorDiv(gx, B), Math.floorDiv(gy, B), Math.floorDiv(gz, B))
        if (b < 0) return
        val i = Math.floorMod(gx, B) + B * (Math.floorMod(gy, B) + B * Math.floorMod(gz, B))
        tsdf[b][i] = value; weight[b][i] = w
        markDirty(b)
    }

    /** Voxel value / weight at grid coordinates (NaN / 0 where unallocated). For tests and debugging. */
    fun sample(gx: Int, gy: Int, gz: Int): Pair<Float, Float> {
        val b = blockIndex(Math.floorDiv(gx, B), Math.floorDiv(gy, B), Math.floorDiv(gz, B))
        if (b < 0) return Pair(Float.NaN, 0f)
        val i = Math.floorMod(gx, B) + B * (Math.floorMod(gy, B) + B * Math.floorMod(gz, B))
        return Pair(tsdf[b][i], weight[b][i])
    }

    /**
     * Marching cubes over [blocks] (default all) -> indexed mesh (vertex normals, per-vertex colour when the
     * volume has colour, else null). A cube belongs to the block of its minimum corner.
     */
    fun extract(minWeight: Float = cfg.minExtractWeight, blocks: IntArray? = null): TriMesh {
        val vmap = LongIntMap(4096)
        val xyz = FloatList(1 shl 14)
        val cols = IntList(1 shl 12)
        val tris = IntList(1 shl 14)
        val hasColor = rgb.any { it != null }
        val nbT = arrayOfNulls<FloatArray>(8); val nbW = arrayOfNulls<FloatArray>(8); val nbC = arrayOfNulls<ByteArray>(8)
        val cv = FloatArray(8); val cw = FloatArray(8)
        val cb = IntArray(8); val ci = IntArray(8)
        val vid = IntArray(16)
        val vox = cfg.voxel
        val list = blocks ?: IntArray(tsdf.size) { it }
        for (b in list) {
            val bx = coords.a[b * 3]; val by = coords.a[b * 3 + 1]; val bz = coords.a[b * 3 + 2]
            for (d in 0 until 8) {
                val n = if (d == 0) b else blockIndex(bx + (d and 1), by + (d shr 1 and 1), bz + (d shr 2 and 1))
                nbT[d] = if (n >= 0) tsdf[n] else null; nbW[d] = if (n >= 0) weight[n] else null; nbC[d] = if (n >= 0) rgb[n] else null
            }
            for (lz in 0 until B) for (ly in 0 until B) for (lx in 0 until B) {
                var mask = 0
                var ok = true
                var minAbs = 2f
                for (c in 0 until 8) {
                    val x = lx + (c and 1); val y = ly + (c shr 1 and 1); val z = lz + (c shr 2 and 1)
                    val nb = (x shr 3) or ((y shr 3) shl 1) or ((z shr 3) shl 2)
                    val Wn = nbW[nb]
                    if (Wn == null) { ok = false; break }
                    val i = (x and 7) + B * ((y and 7) + B * (z and 7))
                    val ww = Wn[i]
                    if (ww < minWeight) { ok = false; break }
                    val vv = nbT[nb]!![i]
                    cv[c] = vv; cw[c] = ww; cb[c] = nb; ci[c] = i
                    if (vv < 0f) mask = mask or (1 shl c)
                    minAbs = min(minAbs, abs(vv))
                }
                if (!ok || mask == 0 || mask == 255 || minAbs > 0.6f) continue
                val tri = TRI[mask]
                if (tri.isEmpty()) continue
                val gx = bx * B + lx; val gy = by * B + ly; val gz = bz * B + lz
                for (e in CUT[mask]) {
                    val a = EDGES[e * 2]; val c2 = EDGES[e * 2 + 1]
                    val axis = when (a xor c2) { 1 -> 0; 2 -> 1; else -> 2 }
                    val ax = gx + (a and 1); val ay = gy + (a shr 1 and 1); val az = gz + (a shr 2 and 1)
                    val key = MeshOps.key(ax, ay, az) or (axis.toLong() shl 60)
                    var id = vmap.get(key)
                    if (id < 0) {
                        val va = cv[a]; val vb = cv[c2]
                        val t = (va / (va - vb)).coerceIn(0f, 1f)
                        val px = (ax + if (axis == 0) t else 0f) * vox
                        val py = (ay + if (axis == 1) t else 0f) * vox
                        val pz = (az + if (axis == 2) t else 0f) * vox
                        id = xyz.size / 3
                        xyz.add3(px, py, pz)
                        if (hasColor) {
                            val ca = nbC[cb[a]]; val cc = nbC[cb[c2]]
                            cols.add(if (ca != null && cc != null) lerpRgb(ca, ci[a], cc, ci[c2], t) else (ca ?: cc)?.let { rgbAt(it, if (ca != null) ci[a] else ci[c2]) } ?: -1)
                        }
                        vmap.put(key, id)
                    }
                    vid[e] = id
                }
                CENTROID_POLYS[mask].forEachIndexed { k, poly ->
                    var sx = 0f; var sy = 0f; var sz = 0f
                    for (e in poly) { val v = vid[e] * 3; sx += xyz.a[v]; sy += xyz.a[v + 1]; sz += xyz.a[v + 2] }
                    vid[12 + k] = xyz.size / 3
                    xyz.add3(sx / poly.size, sy / poly.size, sz / poly.size)
                    if (hasColor) cols.add(cols.a[vid[poly[0]]])
                }
                for (e in tri) tris.add(vid[e])
            }
        }
        val pts = xyz.toArray()
        val mesh = TriMesh(pts, FloatArray(pts.size), if (hasColor) cols.toArray() else null, tris.toArray())
        mesh.computeNormals()
        return mesh
    }

    fun clear() {
        index.clear(); coords.size = 0; tsdf.clear(); weight.clear(); rgb.clear(); cweight.clear(); touched.size = 0; full = false; integratedFrames = 0
    }

    companion object {
        const val B = 8
        const val BV = B * B * B
        /** Cap of the per-voxel colour weight (running average over at most this many samples). */
        const val MAX_COLOR_WEIGHT = 8f

        private fun rgbAt(c: ByteArray, i: Int): Int {
            val r = c[i * 3].toInt() and 255; val g = c[i * 3 + 1].toInt() and 255; val b = c[i * 3 + 2].toInt() and 255
            return if (r == 0 && g == 0 && b == 0) -1 else (r shl 16) or (g shl 8) or b
        }

        private fun lerpRgb(ca: ByteArray, ia: Int, cb: ByteArray, ib: Int, t: Float): Int {
            val a = rgbAt(ca, ia); val b = rgbAt(cb, ib)
            if (a < 0) return b; if (b < 0) return a
            fun ch(s: Int) = ((a shr s and 255) * (1 - t) + (b shr s and 255) * t + 0.5f).toInt() shl s
            return ch(16) or ch(8) or ch(0)
        }

        /** Cube corner c = x + 2y + 4z. 12 edges as corner pairs (a < b, differing in one bit). */
        val EDGES: IntArray
        /** Faces as 4 corners in cyclic order. */
        private val FACES = arrayOf(intArrayOf(0, 2, 6, 4), intArrayOf(1, 3, 7, 5), intArrayOf(0, 1, 5, 4),
            intArrayOf(2, 3, 7, 6), intArrayOf(0, 1, 3, 2), intArrayOf(4, 5, 7, 6))
        private val FACE_NORMALS = arrayOf(floatArrayOf(-1f, 0f, 0f), floatArrayOf(1f, 0f, 0f), floatArrayOf(0f, -1f, 0f),
            floatArrayOf(0f, 1f, 0f), floatArrayOf(0f, 0f, -1f), floatArrayOf(0f, 0f, 1f))
        /** Per case: polygons triangulated around their centroid (TRI index 12 + k). */
        val CENTROID_POLYS: Array<Array<IntArray>>
        /** Per case: the cut edges. */
        val CUT: Array<IntArray>
        /** Per inside-corner mask: edge indices (12 + k = centroid of CENTROID_POLYS[mask][k]), 3 per triangle, wound counter-clockwise seen from outside (tsdf > 0). */
        val TRI: Array<IntArray>

        init {
            val e = ArrayList<Int>()
            for (a in 0 until 8) for (bit in intArrayOf(1, 2, 4)) if (a and bit == 0) { e.add(a); e.add(a or bit) }
            EDGES = e.toIntArray()
            val cp = Array(256) { mutableListOf<IntArray>() }
            TRI = Array(256) { buildCase(it, cp[it]) }
            CENTROID_POLYS = Array(256) { cp[it].toTypedArray() }
            CUT = Array(256) { m -> IntArray(12) { it }.filter { e -> ((m shr EDGES[e * 2]) and 1) != ((m shr EDGES[e * 2 + 1]) and 1) }.toIntArray() }
        }

        private fun edgeOf(a: Int, b: Int): Int {
            val lo = min(a, b); val hi = max(a, b)
            for (k in 0 until 12) if (EDGES[k * 2] == lo && EDGES[k * 2 + 1] == hi) return k
            error("no edge $a-$b")
        }

        private fun shareFace(a: Int, b: Int): Boolean = FACES.any { f ->
            fun on(e: Int) = EDGES[e * 2] in f && EDGES[e * 2 + 1] in f
            on(a) && on(b)
        }

        private fun buildCase(mask: Int, centroidPolys: MutableList<IntArray>): IntArray {
            fun inside(c: Int) = (mask shr c) and 1 == 1
            val adj = Array(12) { IntArray(2) { -1 } }
            val faceOf = HashMap<Int, Int>()      // (a * 12 + b) -> face of the segment a-b
            var face = 0
            fun link(a: Int, b: Int) {
                adj[a][if (adj[a][0] < 0) 0 else 1] = b
                adj[b][if (adj[b][0] < 0) 0 else 1] = a
                faceOf[a * 12 + b] = face; faceOf[b * 12 + a] = face
            }
            for ((fi, f) in FACES.withIndex()) {
                face = fi
                val cut = (0 until 4).filter { inside(f[it]) != inside(f[(it + 1) % 4]) }.map { edgeOf(f[it], f[(it + 1) % 4]) }
                when (cut.size) {
                    2 -> link(cut[0], cut[1])
                    4 -> if (inside(f[0])) {
                        link(edgeOf(f[3], f[0]), edgeOf(f[0], f[1])); link(edgeOf(f[1], f[2]), edgeOf(f[2], f[3]))
                    } else {
                        link(edgeOf(f[0], f[1]), edgeOf(f[1], f[2])); link(edgeOf(f[2], f[3]), edgeOf(f[3], f[0]))
                    }
                }
            }
            fun pos(c: Int) = floatArrayOf((c and 1).toFloat(), (c shr 1 and 1).toFloat(), (c shr 2 and 1).toFloat())
            val seen = BooleanArray(12)
            val out = ArrayList<Int>()
            for (start in 0 until 12) {
                if (adj[start][0] < 0 || seen[start]) continue
                val poly = ArrayList<Int>()
                var prev = -1; var cur = start
                do {
                    poly.add(cur); seen[cur] = true
                    val nx = if (adj[cur][0] != prev) adj[cur][0] else adj[cur][1]
                    prev = cur; cur = nx
                } while (cur != start && poly.size <= 12)
                // orientation, decided on the cube faces: walking a segment A -> B on a face with outward normal
                // nF, the surface (normal towards tsdf > 0) is counter-clockwise when nF x (B - A) points to
                // the outside end of edge A. Summed over the polygon's segments for robustness.
                fun mid(k: Int): FloatArray { val a = pos(EDGES[k * 2]); val b = pos(EDGES[k * 2 + 1]); return FloatArray(3) { (a[it] + b[it]) / 2 } }
                var score = 0f
                for (i in poly.indices) {
                    val a = poly[i]; val b = poly[(i + 1) % poly.size]
                    val fi = faceOf[a * 12 + b] ?: continue
                    val nF = FACE_NORMALS[fi]
                    val ma = mid(a); val mb = mid(b)
                    val dx = mb[0] - ma[0]; val dy = mb[1] - ma[1]; val dz = mb[2] - ma[2]
                    val cx = nF[1] * dz - nF[2] * dy; val cy = nF[2] * dx - nF[0] * dz; val cz = nF[0] * dy - nF[1] * dx
                    val e0 = EDGES[a * 2]; val e1 = EDGES[a * 2 + 1]
                    val (o, iIn) = if (inside(e0)) Pair(e1, e0) else Pair(e0, e1)
                    val po = pos(o); val pi = pos(iIn)
                    score += Math.signum(cx * (po[0] - pi[0]) + cy * (po[1] - pi[1]) + cz * (po[2] - pi[2]))
                }
                if (score < 0) poly.reverse()
                // fan triangulation from a vertex whose diagonals do not lie in a cube face (a diagonal in a face
                // would coincide with the neighbouring cube's and break the manifold); else fan around the
                // polygon's centroid (index 12 + k -> centroid of CENTROID_POLYS[mask][k])
                val k = poly.size
                val start = (0 until k).firstOrNull { s0 ->
                    (2 until k - 1).none { j -> shareFace(poly[s0], poly[(s0 + j) % k]) }
                }
                if (start != null) {
                    for (i in 1 until k - 1) { out.add(poly[start]); out.add(poly[(start + i) % k]); out.add(poly[(start + i + 1) % k]) }
                } else {
                    val c = 12 + centroidPolys.size
                    centroidPolys.add(poly.toIntArray())
                    for (i in 0 until k) { out.add(c); out.add(poly[i]); out.add(poly[(i + 1) % k]) }
                }
            }
            return out.toIntArray()
        }
    }
}
