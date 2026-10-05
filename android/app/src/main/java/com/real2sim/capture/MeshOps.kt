package com.real2sim.capture

import java.io.File
import java.io.FileOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/** Open-addressing Long -> Int map (no boxing). Keys must not be [EMPTY]. */
class LongIntMap(cap: Int = 1024) {
    private var keys = LongArray(pow2(cap * 2)) { EMPTY }
    private var vals = IntArray(keys.size)
    var size = 0; private set

    fun get(k: Long, missing: Int = -1): Int {
        val mask = keys.size - 1
        var i = mix(k) and mask
        while (true) {
            val kk = keys[i]
            if (kk == k) return vals[i]
            if (kk == EMPTY) return missing
            i = (i + 1) and mask
        }
    }

    fun put(k: Long, v: Int) {
        if ((size + 1) * 2 > keys.size) grow()
        val mask = keys.size - 1
        var i = mix(k) and mask
        while (true) {
            val kk = keys[i]
            if (kk == k) { vals[i] = v; return }
            if (kk == EMPTY) { keys[i] = k; vals[i] = v; size++; return }
            i = (i + 1) and mask
        }
    }

    fun clear() { keys.fill(EMPTY); size = 0 }

    private fun grow() {
        val ok = keys; val ov = vals
        keys = LongArray(ok.size * 2) { EMPTY }; vals = IntArray(keys.size); size = 0
        for (i in ok.indices) if (ok[i] != EMPTY) put(ok[i], ov[i])
    }

    companion object {
        const val EMPTY = Long.MIN_VALUE
        private fun pow2(n: Int): Int { var p = 16; while (p < n) p = p shl 1; return p }
        private fun mix(k: Long): Int { var h = k * -0x61c8864680b583ebL; h = h xor (h ushr 29); return (h xor (h ushr 32)).toInt() }
    }
}

class FloatList(cap: Int = 1024) {
    var a = FloatArray(cap); var size = 0
    fun add(v: Float) { if (size == a.size) a = a.copyOf(a.size * 2); a[size++] = v }
    fun add3(x: Float, y: Float, z: Float) { if (size + 3 > a.size) a = a.copyOf(max(a.size * 2, size + 3)); a[size] = x; a[size + 1] = y; a[size + 2] = z; size += 3 }
    fun toArray() = a.copyOf(size)
}

class IntList(cap: Int = 1024) {
    var a = IntArray(cap); var size = 0
    fun add(v: Int) { if (size == a.size) a = a.copyOf(a.size * 2); a[size++] = v }
    fun toArray() = a.copyOf(size)
}

/** Indexed triangle mesh: xyz / nrm per vertex (3 floats), rgb per vertex (0xRRGGBB, or null), tris (3 ints). */
class TriMesh(val xyz: FloatArray, val nrm: FloatArray, val rgb: IntArray?, val tris: IntArray) {
    val vertexCount get() = xyz.size / 3
    val triangleCount get() = tris.size / 3

    /** Area-weighted vertex normals from the triangles (in place). */
    fun computeNormals() {
        nrm.fill(0f)
        var t = 0
        while (t < tris.size) {
            val a = tris[t] * 3; val b = tris[t + 1] * 3; val c = tris[t + 2] * 3
            val ux = xyz[b] - xyz[a]; val uy = xyz[b + 1] - xyz[a + 1]; val uz = xyz[b + 2] - xyz[a + 2]
            val vx = xyz[c] - xyz[a]; val vy = xyz[c + 1] - xyz[a + 1]; val vz = xyz[c + 2] - xyz[a + 2]
            val nx = uy * vz - uz * vy; val ny = uz * vx - ux * vz; val nz = ux * vy - uy * vx
            for (i in intArrayOf(a, b, c)) { nrm[i] += nx; nrm[i + 1] += ny; nrm[i + 2] += nz }
            t += 3
        }
        for (i in 0 until vertexCount) {
            val l = sqrt(nrm[i * 3] * nrm[i * 3] + nrm[i * 3 + 1] * nrm[i * 3 + 1] + nrm[i * 3 + 2] * nrm[i * 3 + 2])
            if (l > 0f) { nrm[i * 3] /= l; nrm[i * 3 + 1] /= l; nrm[i * 3 + 2] /= l } else nrm[i * 3 + 1] = 1f
        }
    }

    fun bounds(): FloatArray {
        val b = floatArrayOf(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE)
        for (i in 0 until vertexCount) for (k in 0 until 3) { val v = xyz[i * 3 + k]; b[k] = min(b[k], v); b[k + 3] = max(b[k + 3], v) }
        return b
    }

    companion object { val EMPTY = TriMesh(FloatArray(0), FloatArray(0), null, IntArray(0)) }
}

object MeshOps {
    /** Component id per triangle (triangles sharing a vertex are connected) and the number of components. */
    fun triangleComponents(m: TriMesh): Pair<IntArray, Int> {
        val parent = IntArray(m.vertexCount) { it }
        fun find(x: Int): Int { var r = x; while (parent[r] != r) { parent[r] = parent[parent[r]]; r = parent[r] }; return r }
        fun union(a: Int, b: Int) { val ra = find(a); val rb = find(b); if (ra != rb) parent[ra] = rb }
        var t = 0
        while (t < m.tris.size) { union(m.tris[t], m.tris[t + 1]); union(m.tris[t], m.tris[t + 2]); t += 3 }
        val label = IntArray(m.vertexCount) { -1 }
        var n = 0
        val comp = IntArray(m.triangleCount)
        for (k in 0 until m.triangleCount) {
            val r = find(m.tris[k * 3])
            if (label[r] < 0) label[r] = n++
            comp[k] = label[r]
        }
        return Pair(comp, n)
    }

    /** Keeps the triangles selected by [keep] (per triangle) and the vertices they use. */
    fun subset(m: TriMesh, keep: BooleanArray): TriMesh {
        val remap = IntArray(m.vertexCount) { -1 }
        var nv = 0
        val tris = IntList(m.tris.size + 3)
        for (k in 0 until m.triangleCount) if (keep[k]) for (c in 0 until 3) {
            val v = m.tris[k * 3 + c]
            if (remap[v] < 0) remap[v] = nv++
            tris.add(remap[v])
        }
        val xyz = FloatArray(nv * 3); val nrm = FloatArray(nv * 3); val rgb = m.rgb?.let { IntArray(nv) }
        for (v in 0 until m.vertexCount) {
            val r = remap[v]; if (r < 0) continue
            System.arraycopy(m.xyz, v * 3, xyz, r * 3, 3); System.arraycopy(m.nrm, v * 3, nrm, r * 3, 3)
            if (rgb != null) rgb[r] = m.rgb!![v]
        }
        return TriMesh(xyz, nrm, rgb, tris.toArray())
    }

    /** Drops connected components with fewer than [minTriangles] triangles (floating noise blobs). */
    fun removeSmallComponents(m: TriMesh, minTriangles: Int): TriMesh {
        if (m.triangleCount == 0) return m
        val (comp, n) = triangleComponents(m)
        val count = IntArray(n)
        for (c in comp) count[c]++
        return subset(m, BooleanArray(m.triangleCount) { count[comp[it]] >= minTriangles })
    }

    class Plane(val nx: Float, val ny: Float, val nz: Float, val d: Float, val inliers: Int) {
        fun dist(x: Float, y: Float, z: Float) = nx * x + ny * y + nz * z + d
    }

    class Obj(val triangles: Int, val min: FloatArray, val max: FloatArray) {
        val center get() = FloatArray(3) { (min[it] + max[it]) / 2 }
        val size get() = FloatArray(3) { max[it] - min[it] }
    }

    class Segmentation(val planes: List<Plane>, val objects: List<Obj>, val objectTriangle: IntArray)

    /**
     * Structure / object split: up to [maxPlanes] dominant planes (floor, walls, table tops) are found by
     * RANSAC on the vertices (position + normal), triangles lying on a plane are "structure", and the
     * remaining triangles are grouped into connected components = objects (>= [minObjectTriangles]).
     * [objectTriangle] is the object index per triangle (-1 = structure or too small).
     */
    fun segment(m: TriMesh, tol: Float, maxPlanes: Int = 4, minPlaneFrac: Float = 0.08f, minObjectTriangles: Int = 40, seed: Long = 1): Segmentation {
        val nv = m.vertexCount
        val onPlane = BooleanArray(nv)
        val planes = ArrayList<Plane>()
        val rnd = java.util.Random(seed)
        repeat(maxPlanes) {
            val free = (0 until nv).filter { !onPlane[it] }
            if (free.size < 30) return@repeat
            var best: Plane? = null
            repeat(200) {
                val s = free[rnd.nextInt(free.size)]
                val nx = m.nrm[s * 3]; val ny = m.nrm[s * 3 + 1]; val nz = m.nrm[s * 3 + 2]
                val d = -(nx * m.xyz[s * 3] + ny * m.xyz[s * 3 + 1] + nz * m.xyz[s * 3 + 2])
                var cnt = 0
                var k = 0
                val stride = max(1, free.size / 4000)
                while (k < free.size) {
                    val v = free[k]
                    if (abs(nx * m.xyz[v * 3] + ny * m.xyz[v * 3 + 1] + nz * m.xyz[v * 3 + 2] + d) < tol &&
                        nx * m.nrm[v * 3] + ny * m.nrm[v * 3 + 1] + nz * m.nrm[v * 3 + 2] > 0.85f) cnt++
                    k += stride
                }
                cnt *= stride
                if (best == null || cnt > best!!.inliers) best = Plane(nx, ny, nz, d, cnt)
            }
            val p = best ?: return@repeat
            if (p.inliers < minPlaneFrac * nv) return@repeat
            // refine: least squares on inliers would be nicer; the RANSAC sample normal is good enough here
            var cnt = 0
            // membership by distance only: seam vertices (wall/floor corners, object footprints) join the plane
            for (v in free) if (abs(p.dist(m.xyz[v * 3], m.xyz[v * 3 + 1], m.xyz[v * 3 + 2])) < tol) { onPlane[v] = true; cnt++ }
            planes.add(Plane(p.nx, p.ny, p.nz, p.d, cnt))
        }
        val keep = BooleanArray(m.triangleCount) { t ->
            !(onPlane[m.tris[t * 3]] && onPlane[m.tris[t * 3 + 1]] && onPlane[m.tris[t * 3 + 2]])
        }
        // components of the non-structure triangles (connectivity through shared, non-plane-only vertices)
        val parent = IntArray(nv) { it }
        fun find(x: Int): Int { var r = x; while (parent[r] != r) { parent[r] = parent[parent[r]]; r = parent[r] }; return r }
        for (t in 0 until m.triangleCount) if (keep[t]) {
            // vertices on a plane do not connect objects (an object touching the floor shares floor vertices)
            val vs = intArrayOf(m.tris[t * 3], m.tris[t * 3 + 1], m.tris[t * 3 + 2]).filter { !onPlane[it] }
            for (k in 1 until vs.size) { val ra = find(vs[0]); val rb = find(vs[k]); if (ra != rb) parent[ra] = rb }
        }
        val rootOf = IntArray(m.triangleCount) { t ->
            if (!keep[t]) -1 else intArrayOf(m.tris[t * 3], m.tris[t * 3 + 1], m.tris[t * 3 + 2]).firstOrNull { !onPlane[it] }?.let { find(it) } ?: -1
        }
        val counts = HashMap<Int, Int>()
        for (r in rootOf) if (r >= 0) counts[r] = (counts[r] ?: 0) + 1
        val idOf = HashMap<Int, Int>()
        val objs = ArrayList<Obj>()
        val objTri = IntArray(m.triangleCount) { -1 }
        for (t in 0 until m.triangleCount) {
            val r = rootOf[t]; if (r < 0 || (counts[r] ?: 0) < minObjectTriangles) continue
            val id = idOf.getOrPut(r) { objs.add(Obj(0, floatArrayOf(Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE), floatArrayOf(-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE))); objs.size - 1 }
            objTri[t] = id
            val o = objs[id]
            for (c in 0 until 3) { val v = m.tris[t * 3 + c]; for (k in 0 until 3) { o.min[k] = min(o.min[k], m.xyz[v * 3 + k]); o.max[k] = max(o.max[k], m.xyz[v * 3 + k]) } }
        }
        val tc = IntArray(objs.size); for (id in objTri) if (id >= 0) tc[id]++
        return Segmentation(planes, objs.mapIndexed { i, o -> Obj(tc[i], o.min, o.max) }, objTri)
    }

    /** One point per [cell] (the first one seen): xyz (+ parallel [extra] arrays of 1 int per point, if given). */
    fun voxelDownsample(xyz: FloatArray, cell: Float): IntArray {
        val seen = LongIntMap(xyz.size / 6 + 16)
        val out = IntList()
        for (i in 0 until xyz.size / 3) {
            val k = key(floor(xyz[i * 3] / cell).toInt(), floor(xyz[i * 3 + 1] / cell).toInt(), floor(xyz[i * 3 + 2] / cell).toInt())
            if (seen.get(k) < 0) { seen.put(k, i); out.add(i) }
        }
        return out.toArray()
    }

    /**
     * Statistical outlier removal: mean distance to the [k] nearest neighbours (searched in a hash grid of
     * [radius] cells, neighbours beyond [radius] count as [radius]); points whose mean exceeds
     * global mean + [stdMul] * std are removed. Returns keep flags.
     */
    fun statisticalOutliers(xyz: FloatArray, k: Int = 8, radius: Float, stdMul: Float = 2f): BooleanArray {
        val n = xyz.size / 3
        if (n == 0) return BooleanArray(0)
        val cellOf = LongIntMap(n / 2 + 16)
        val next = IntArray(n) { -1 }
        for (i in 0 until n) {
            val key = key(floor(xyz[i * 3] / radius).toInt(), floor(xyz[i * 3 + 1] / radius).toInt(), floor(xyz[i * 3 + 2] / radius).toInt())
            next[i] = cellOf.get(key); cellOf.put(key, i)
        }
        val meanD = FloatArray(n)
        val best = FloatArray(k)
        for (i in 0 until n) {
            best.fill(radius)
            val x = xyz[i * 3]; val y = xyz[i * 3 + 1]; val z = xyz[i * 3 + 2]
            val ci = floor(x / radius).toInt(); val cj = floor(y / radius).toInt(); val ck = floor(z / radius).toInt()
            for (a in -1..1) for (b in -1..1) for (c in -1..1) {
                var j = cellOf.get(key(ci + a, cj + b, ck + c))
                while (j >= 0) {
                    if (j != i) {
                        val dx = xyz[j * 3] - x; val dy = xyz[j * 3 + 1] - y; val dz = xyz[j * 3 + 2] - z
                        val d = sqrt(dx * dx + dy * dy + dz * dz)
                        if (d < best[k - 1]) {
                            var p = k - 1
                            while (p > 0 && best[p - 1] > d) { best[p] = best[p - 1]; p-- }
                            best[p] = d
                        }
                    }
                    j = next[j]
                }
            }
            meanD[i] = best.sum() / k
        }
        var s = 0.0; var s2 = 0.0
        for (d in meanD) { s += d; s2 += d.toDouble() * d }
        val mean = s / n; val std = sqrt(max(0.0, s2 / n - mean * mean))
        val lim = mean + stdMul * std
        return BooleanArray(n) { meanD[it] <= lim }
    }

    fun key(i: Int, j: Int, k: Int): Long =
        (i.toLong() and 0xfffff) or ((j.toLong() and 0xfffff) shl 20) or ((k.toLong() and 0xfffff) shl 40)

    /** Height colour (blue at the floor -> green -> red at 2 m), same ramp as the live view and viewer. */
    fun heightColor(y: Float, floor: Float): Int {
        val h = ((y - floor) / 2f).coerceIn(0f, 1f)
        val r: Float; val g: Float; val b: Float
        if (h < 0.5f) { val t = h * 2; r = 0.1f; g = 0.4f + 0.6f * t; b = 1f - 0.6f * t }
        else { val t = (h - 0.5f) * 2; r = 0.1f + 0.9f * t; g = 1f - 0.75f * t; b = 0.4f - 0.2f * t }
        return ((r * 255).toInt() shl 16) or ((g * 255).toInt() shl 8) or (b * 255).toInt()
    }

    // ------------------------------------------------------------------ PLY output
    /** Binary little-endian PLY: float x y z nx ny nz, uchar red green blue; faces as uchar count + int indices. */
    fun writeMeshPly(f: File, m: TriMesh, colors: IntArray, comment: String = "", faces: Boolean = true) {
        f.parentFile?.mkdirs()
        FileOutputStream(f).buffered(1 shl 16).use { o ->
            val hdr = StringBuilder("ply\nformat binary_little_endian 1.0\n")
            if (comment.isNotEmpty()) comment.lines().forEach { hdr.append("comment ").append(it).append('\n') }
            hdr.append("element vertex ${m.vertexCount}\nproperty float x\nproperty float y\nproperty float z\n")
            hdr.append("property float nx\nproperty float ny\nproperty float nz\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n")
            if (faces) hdr.append("element face ${m.triangleCount}\nproperty list uchar int vertex_indices\n")
            hdr.append("end_header\n")
            o.write(hdr.toString().toByteArray(Charsets.US_ASCII))
            val bb = ByteBuffer.allocate(27 * 4096).order(ByteOrder.LITTLE_ENDIAN)
            for (i in 0 until m.vertexCount) {
                for (k in 0 until 3) bb.putFloat(m.xyz[i * 3 + k])
                for (k in 0 until 3) bb.putFloat(m.nrm[i * 3 + k])
                val c = colors[i]
                bb.put((c shr 16 and 255).toByte()); bb.put((c shr 8 and 255).toByte()); bb.put((c and 255).toByte())
                if (bb.remaining() < 27) { o.write(bb.array(), 0, bb.position()); bb.clear() }
            }
            o.write(bb.array(), 0, bb.position()); bb.clear()
            for (t in 0 until m.triangleCount) {
                bb.put(3); bb.putInt(m.tris[t * 3]); bb.putInt(m.tris[t * 3 + 1]); bb.putInt(m.tris[t * 3 + 2])
                if (bb.remaining() < 13) { o.write(bb.array(), 0, bb.position()); bb.clear() }
            }
            o.write(bb.array(), 0, bb.position())
        }
    }

    /** Binary little-endian PLY points: float x y z nx ny nz, uchar red green blue. */
    fun writePointsPly(f: File, xyz: FloatArray, nrm: FloatArray, colors: IntArray, comment: String = "") {
        require(xyz.size / 3 == colors.size && nrm.size == xyz.size)
        writeMeshPly(f, TriMesh(xyz, nrm, null, IntArray(0)), colors, comment, faces = false)
    }
}
