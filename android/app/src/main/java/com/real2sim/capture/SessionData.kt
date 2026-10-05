package com.real2sim.capture

import java.io.File
import java.io.RandomAccessFile
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.Locale
import java.util.zip.Inflater
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.min

/*
 * Pure-Kotlin readers for a recorded session folder (no Android classes, so they run in JVM unit
 * tests). The on-disk layout is the one SessionWriter / ArRecorder / Camera2Recorder / SensorRecorder /
 * VideoEncoder / MapBuilder write:
 *
 *   session.json                      metadata (mode, start/end_uptime, counts, ...)
 *   frames.jsonl                      mode A: one line per video sample {i,t,w,h,K,T,track,d,c,sd,dw,dh,...}
 *   video.mp4 + video.mp4.pts.csv     mode A: ARCore CPU image video; pts.csv = frame,pts_us,t_ns
 *   depth.zlib.bin / depth_smooth.zlib.bin   raw-deflate uint16 LE millimetres, ranges [off,len] in frames.jsonl
 *   cams/<lens>.mp4 (+ .pts.csv, .jsonl)     mode B: one video per lens, per-sample {i,t,w,h,K,exp,iso,...}
 *   cams/tof_depth.jsonl + .zlib.bin  mode B: ToF DEPTH16 (uint16 mm), {i,t,w,h,d,c,K}
 *   cams/raw/<lens>_<t>.dng, cams/calibration.json
 *   accel.csv gyro.csv mag.csv imu.csv altimeter.csv location.csv status.csv clock.csv
 *   extras/sensors_raw.csv (t,sensor,v0..v5,accuracy), extras/device_status.csv, extras/gnss_status.csv
 *   extras/map/{map.json,occupancy.bin,occupancy.png,trajectory.csv,points.ply}   mode A live map
 *   extras/recon/{mesh.ply,points.ply,objects.json,recon.json}   mode A TSDF reconstruction (DepthFusion)
 *
 * Every `t` is seconds on CLOCK_BOOTTIME.
 */

// ------------------------------------------------------------------------------------------ JSON

/** Small JSON reader and pretty-printer: objects -> LinkedHashMap, arrays -> List, numbers -> Double. */
object MiniJson {
    fun parse(s: String): Any? {
        val p = Parser(s)
        val v = p.value()
        p.ws()
        if (p.i != s.length) throw IllegalArgumentException("trailing data at ${p.i}")
        return v
    }

    fun parseOrNull(s: String): Any? = try { parse(s) } catch (_: Exception) { null }

    @Suppress("UNCHECKED_CAST")
    fun obj(s: String): Map<String, Any?>? = parseOrNull(s) as? Map<String, Any?>

    private class Parser(val s: String) {
        var i = 0
        fun ws() { while (i < s.length && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) i++ }
        fun err(): Nothing = throw IllegalArgumentException("bad JSON at $i")
        fun value(): Any? {
            ws()
            if (i >= s.length) err()
            return when (val c = s[i]) {
                '{' -> obj()
                '[' -> arr()
                '"' -> str()
                't' -> lit("true", true)
                'f' -> lit("false", false)
                'n' -> lit("null", null)
                else -> if (c == '-' || c in '0'..'9') num() else err()
            }
        }
        fun lit(word: String, v: Any?): Any? { if (!s.startsWith(word, i)) err(); i += word.length; return v }
        fun num(): Double {
            val st = i
            if (s[i] == '-') i++
            while (i < s.length && (s[i] in '0'..'9' || s[i] == '.' || s[i] == 'e' || s[i] == 'E' || s[i] == '+' || s[i] == '-')) i++
            return s.substring(st, i).toDoubleOrNull() ?: err()
        }
        fun str(): String {
            i++ // opening quote
            val sb = StringBuilder()
            while (true) {
                if (i >= s.length) err()
                val c = s[i++]
                when (c) {
                    '"' -> return sb.toString()
                    '\\' -> {
                        if (i >= s.length) err()
                        when (s[i++]) {
                            '"' -> sb.append('"'); '\\' -> sb.append('\\'); '/' -> sb.append('/')
                            'b' -> sb.append('\b'); 'f' -> sb.append('\u000c'); 'n' -> sb.append('\n')
                            'r' -> sb.append('\r'); 't' -> sb.append('\t')
                            'u' -> { if (i + 4 > s.length) err(); sb.append(s.substring(i, i + 4).toInt(16).toChar()); i += 4 }
                            else -> err()
                        }
                    }
                    else -> sb.append(c)
                }
            }
        }
        fun arr(): List<Any?> {
            i++
            val out = ArrayList<Any?>()
            ws()
            if (i < s.length && s[i] == ']') { i++; return out }
            while (true) {
                out.add(value()); ws()
                if (i >= s.length) err()
                when (s[i++]) { ',' -> continue; ']' -> return out; else -> err() }
            }
        }
        fun obj(): Map<String, Any?> {
            i++
            val out = LinkedHashMap<String, Any?>()
            ws()
            if (i < s.length && s[i] == '}') { i++; return out }
            while (true) {
                ws()
                if (i >= s.length || s[i] != '"') err()
                val k = str(); ws()
                if (i >= s.length || s[i++] != ':') err()
                out[k] = value(); ws()
                if (i >= s.length) err()
                when (s[i++]) { ',' -> continue; '}' -> return out; else -> err() }
            }
        }
    }

    /** Indented JSON text (keys in their original order). Whole numbers print without ".0". */
    fun pretty(v: Any?, indent: Int = 2): String = StringBuilder().also { write(it, v, 0, indent) }.toString()

    private fun write(sb: StringBuilder, v: Any?, level: Int, ind: Int) {
        fun pad(l: Int) { repeat(l * ind) { sb.append(' ') } }
        when (v) {
            null -> sb.append("null")
            is String -> quote(sb, v)
            is Boolean -> sb.append(v)
            is Number -> sb.append(num(v.toDouble()))
            is Map<*, *> -> {
                if (v.isEmpty()) { sb.append("{}"); return }
                sb.append("{\n")
                var first = true
                for ((k, x) in v) {
                    if (!first) sb.append(",\n"); first = false
                    pad(level + 1); quote(sb, k.toString()); sb.append(": "); write(sb, x, level + 1, ind)
                }
                sb.append('\n'); pad(level); sb.append('}')
            }
            is List<*> -> {
                if (v.isEmpty()) { sb.append("[]"); return }
                // short lists of scalars (K, T, sizes) stay on one line
                if (v.size <= 16 && v.all { it == null || it is Number || it is Boolean }) {
                    sb.append('['); v.forEachIndexed { k, x -> if (k > 0) sb.append(", "); write(sb, x, level, ind) }; sb.append(']'); return
                }
                sb.append("[\n")
                v.forEachIndexed { k, x -> if (k > 0) sb.append(",\n"); pad(level + 1); write(sb, x, level + 1, ind) }
                sb.append('\n'); pad(level); sb.append(']')
            }
            else -> quote(sb, v.toString())
        }
    }

    fun num(d: Double): String = when {
        d.isNaN() || d.isInfinite() -> "null"
        d == Math.rint(d) && abs(d) < 1e15 -> d.toLong().toString()
        else -> d.toString()
    }

    private fun quote(sb: StringBuilder, s: String) {
        sb.append('"')
        for (c in s) when (c) {
            '"' -> sb.append("\\\""); '\\' -> sb.append("\\\\"); '\n' -> sb.append("\\n"); '\r' -> sb.append("\\r"); '\t' -> sb.append("\\t")
            else -> if (c < ' ') sb.append(String.format(Locale.US, "\\u%04x", c.code)) else sb.append(c)
        }
        sb.append('"')
    }
}

@Suppress("UNCHECKED_CAST")
fun Map<String, Any?>.obj(key: String): Map<String, Any?>? = this[key] as? Map<String, Any?>
fun Map<String, Any?>.num(key: String): Double? = (this[key] as? Number)?.toDouble()
fun Map<String, Any?>.str(key: String): String? = this[key] as? String
fun Map<String, Any?>.doubles(key: String): DoubleArray? =
    (this[key] as? List<*>)?.let { l -> if (l.all { it is Number }) DoubleArray(l.size) { (l[it] as Number).toDouble() } else null }

// ------------------------------------------------------------------------------------------ CSV

/** Numeric CSV, column-major. Non-numeric cells are NaN ("true"/"false" are 1/0). */
class CsvTable(val columns: List<String>, val data: List<DoubleArray>) {
    val rows: Int get() = data.firstOrNull()?.size ?: 0
    fun col(name: String): DoubleArray? = columns.indexOf(name).takeIf { it >= 0 }?.let { data[it] }
    fun has(name: String) = columns.contains(name)

    companion object {
        fun cell(s: String): Double = when (s) {
            "true" -> 1.0
            "false" -> 0.0
            else -> s.toDoubleOrNull() ?: Double.NaN
        }

        /** Parses header + rows, keeping every [stride]-th data row. */
        fun parse(lines: Sequence<String>, stride: Int = 1): CsvTable {
            val it = lines.iterator()
            if (!it.hasNext()) return CsvTable(emptyList(), emptyList())
            val cols = it.next().split(',').map { c -> c.trim() }
            val acc = List(cols.size) { DoubleBuilder() }
            var k = 0
            while (it.hasNext()) {
                val line = it.next()
                if (line.isBlank()) continue
                if (k++ % stride != 0) continue
                val parts = line.split(',')
                for (c in cols.indices) acc[c].add(if (c < parts.size) cell(parts[c].trim()) else Double.NaN)
            }
            return CsvTable(cols, acc.map { b -> b.toArray() })
        }

        /** Reads a file, sub-sampling rows so that at most about [maxRows] remain. */
        fun read(f: File, maxRows: Int = 40_000): CsvTable? {
            if (!f.isFile) return null
            val stride = strideFor(f, maxRows)
            return f.bufferedReader().useLines { parse(it, stride) }
        }

        /** Row stride from file size and the length of the first lines. */
        fun strideFor(f: File, maxRows: Int): Int {
            val sample = f.bufferedReader().useLines { s -> s.drop(1).take(50).toList() }
            if (sample.isEmpty()) return 1
            val avg = sample.sumOf { it.length + 1 }.toDouble() / sample.size
            val est = (f.length() / avg).toLong()
            return max(1L, (est + maxRows - 1) / maxRows).toInt()
        }
    }
}

class DoubleBuilder(cap: Int = 1024) {
    private var a = DoubleArray(cap)
    var size = 0; private set
    fun add(v: Double) { if (size == a.size) a = a.copyOf(a.size * 2); a[size++] = v }
    fun toArray(): DoubleArray = a.copyOf(size)
}

/**
 * extras/sensors_raw.csv (t,sensor,v0..v5,accuracy) split per sensor name -> table t,v0..vN
 * (columns that are empty for a sensor are dropped). [skip] names are ignored.
 */
object RawSensors {
    fun parse(lines: Sequence<String>, skip: Set<String> = emptySet(), strides: Map<String, Int> = emptyMap()): Map<String, CsvTable> {
        val t = HashMap<String, DoubleBuilder>()
        val v = HashMap<String, List<DoubleBuilder>>()
        val seen = HashMap<String, Int>()
        var first = true
        for (line in lines) {
            if (first) { first = false; if (line.startsWith("t,")) continue }
            val c1 = line.indexOf(','); if (c1 < 0) continue
            val c2 = line.indexOf(',', c1 + 1); if (c2 < 0) continue
            val name = line.substring(c1 + 1, c2)
            if (name in skip) continue
            val n = seen.getOrDefault(name, 0); seen[name] = n + 1
            if (n % (strides[name] ?: 1) != 0) continue
            val parts = line.split(',')
            t.getOrPut(name) { DoubleBuilder() }.add(CsvTable.cell(parts[0]))
            val vs = v.getOrPut(name) { List(6) { DoubleBuilder() } }
            for (k in 0 until 6) vs[k].add(if (2 + k < parts.size) CsvTable.cell(parts[2 + k]) else Double.NaN)
        }
        val out = sortedMapOf<String, CsvTable>()
        for ((name, tb) in t) {
            val cols = mutableListOf("t"); val data = mutableListOf(tb.toArray())
            v[name]!!.forEachIndexed { k, b -> val a = b.toArray(); if (a.any { !it.isNaN() }) { cols.add("v$k"); data.add(a) } }
            out[name] = CsvTable(cols, data)
        }
        return out
    }

    /** Counts rows per sensor (first pass), so the second pass can sub-sample busy sensors. */
    fun counts(lines: Sequence<String>): Map<String, Int> {
        val out = HashMap<String, Int>()
        for (line in lines) {
            val c1 = line.indexOf(','); if (c1 < 0) continue
            val c2 = line.indexOf(',', c1 + 1); if (c2 < 0) continue
            val name = line.substring(c1 + 1, c2)
            if (name == "sensor") continue
            out[name] = (out[name] ?: 0) + 1
        }
        return out
    }

    fun read(f: File, skip: Set<String>, maxRows: Int = 20_000): Map<String, CsvTable> {
        if (!f.isFile) return emptyMap()
        val counts = f.bufferedReader().useLines { counts(it) }
        val strides = counts.mapValues { (_, n) -> max(1, (n + maxRows - 1) / maxRows) }
        return f.bufferedReader().useLines { parse(it, skip, strides) }
    }
}

// ------------------------------------------------------------------------------------------ frames / depth

/**
 * Byte range of one raw-deflate depth map in a .zlib.bin blob; [confOff]/[confLen] the matching confidence
 * map (levels 0..2) in conf.zlib.bin next to depth.zlib.bin, or -1 / 0.
 */
data class DepthRef(val t: Double, val off: Long, val len: Int, val w: Int, val h: Int, val confOff: Long = -1, val confLen: Int = 0)

/** One video sample (frames.jsonl or cams/<lens>.jsonl line). K = fx,fy,cx,cy; T = 4x4 row-major camera-to-world. */
class FrameRec(
    val i: Int, val t: Double, val w: Int, val h: Int,
    val K: DoubleArray?, val T: DoubleArray?, val track: String?,
    val exp: Double, val iso: Double, val amb: Double,
) {
    val hasPose: Boolean get() = T != null && T.size == 16
    fun x() = T!![3]; fun y() = T!![7]; fun z() = T!![11]
    /** Yaw of the camera's forward axis in the x/z plane (same convention as ArRecorder.heading / MapView). */
    fun heading(): Double = atan2(-T!![10], -T!![2])
}

class FrameData(val frames: List<FrameRec>, val depth: List<DepthRef>, val smooth: List<DepthRef>) {
    val times: DoubleArray = DoubleArray(frames.size) { frames[it].t }
    val depthTimes: DoubleArray = DoubleArray(depth.size) { depth[it].t }
    val smoothTimes: DoubleArray = DoubleArray(smooth.size) { smooth[it].t }
    val poses: List<FrameRec> = frames.filter { it.hasPose }
    val poseTimes: DoubleArray = DoubleArray(poses.size) { poses[it].t }
}

object Frames {
    /** [rangeKey] array [off,len] + size keys -> DepthRef (t from the line). */
    fun depthRef(o: Map<String, Any?>, rangeKey: String, wKey: String, hKey: String): DepthRef? {
        val r = o.doubles(rangeKey) ?: return null
        if (r.size != 2) return null
        val t = o.num("t") ?: return null
        val w = o.num(wKey) ?: return null
        val h = o.num(hKey) ?: return null
        return DepthRef(t, r[0].toLong(), r[1].toInt(), w.toInt(), h.toInt())
    }

    fun frame(o: Map<String, Any?>, index: Int): FrameRec? {
        val t = o.num("t") ?: return null
        return FrameRec(
            (o.num("i") ?: index.toDouble()).toInt(), t,
            (o.num("w") ?: 0.0).toInt(), (o.num("h") ?: 0.0).toInt(),
            o.doubles("K"), o.doubles("T")?.takeIf { it.size == 16 }, o.str("track"),
            o.num("exp") ?: Double.NaN, o.num("iso") ?: Double.NaN, o.num("amb") ?: Double.NaN,
        )
    }

    /**
     * Video-sample lines. Depth: mode A lines carry "d"/"sd" with "dw"/"dh"; ToF lines (cams/tof_depth.jsonl)
     * carry "d" with "w"/"h" — pass [depthSizeKeys] accordingly.
     */
    fun parse(lines: Sequence<String>, depthSizeKeys: Pair<String, String> = "dw" to "dh"): FrameData {
        val frames = ArrayList<FrameRec>(); val depth = ArrayList<DepthRef>(); val smooth = ArrayList<DepthRef>()
        var k = 0
        for (line in lines) {
            if (line.isBlank()) continue
            val o = MiniJson.obj(line) ?: continue
            frame(o, k)?.let { frames.add(it) }
            depthRef(o, "d", depthSizeKeys.first, depthSizeKeys.second)?.let { r ->
                val c = if (depthSizeKeys.first == "dw") o.doubles("c") else null   // mode A: raw confidence range
                depth.add(if (c != null && c.size == 2) r.copy(confOff = c[0].toLong(), confLen = c[1].toInt()) else r)
            }
            depthRef(o, "sd", depthSizeKeys.first, depthSizeKeys.second)?.let { smooth.add(it) }
            k++
        }
        frames.sortBy { it.t }; depth.sortBy { it.t }; smooth.sortBy { it.t }
        return FrameData(frames, depth, smooth)
    }

    fun read(f: File, depthSizeKeys: Pair<String, String> = "dw" to "dh"): FrameData =
        if (f.isFile) f.bufferedReader().useLines { parse(it, depthSizeKeys) } else FrameData(emptyList(), emptyList(), emptyList())
}

/** Index of the value in ascending [ts] nearest to [t]; -1 when empty. */
fun nearestIndex(ts: DoubleArray, t: Double): Int {
    if (ts.isEmpty()) return -1
    var lo = 0; var hi = ts.size - 1
    while (lo < hi) { val mid = (lo + hi) ushr 1; if (ts[mid] < t) lo = mid + 1 else hi = mid }
    if (lo > 0 && abs(ts[lo - 1] - t) <= abs(ts[lo] - t)) lo--
    return lo
}

/** Index of the last value <= t (clamped to 0); -1 when empty. */
fun floorIndex(ts: DoubleArray, t: Double): Int {
    if (ts.isEmpty()) return -1
    var lo = 0; var hi = ts.size
    while (lo < hi) { val mid = (lo + hi) ushr 1; if (ts[mid] <= t) lo = mid + 1 else hi = mid }
    return max(0, lo - 1)
}

/** <video>.pts.csv (frame,pts_us,t_ns): sample presentation times and their CLOCK_BOOTTIME times. */
class PtsTable(val ptsUs: LongArray, val tNs: LongArray) {
    val size get() = ptsUs.size
    /** Session time (s) of the first video sample: playback position 0. */
    val firstT: Double? get() = if (tNs.isEmpty()) null else tNs[0] / 1e9
    val durationS: Double get() = if (ptsUs.size < 2) 0.0 else (ptsUs.last() - ptsUs[0]) / 1e6

    /** Session time of the sample shown at playback position [posS] (the muxer starts the track at the first pts). */
    fun sessionTime(posS: Double): Double? {
        if (ptsUs.isEmpty()) return null
        val target = ptsUs[0] + (posS * 1e6).toLong()
        var lo = 0; var hi = ptsUs.size
        while (lo < hi) { val mid = (lo + hi) ushr 1; if (ptsUs[mid] <= target) lo = mid + 1 else hi = mid }
        return tNs[max(0, lo - 1)] / 1e9
    }

    companion object {
        fun parse(lines: Sequence<String>): PtsTable {
            val p = ArrayList<Long>(); val t = ArrayList<Long>()
            for (line in lines) {
                val parts = line.split(',')
                if (parts.size < 3) continue
                val a = parts[1].trim().toLongOrNull() ?: continue
                val b = parts[2].trim().toLongOrNull() ?: continue
                p.add(a); t.add(b)
            }
            return PtsTable(p.toLongArray(), t.toLongArray())
        }
        fun read(f: File): PtsTable? = if (f.isFile) f.bufferedReader().useLines { parse(it) } else null
    }
}

object Depth {
    /** Inflates a raw-deflate (nowrap) chunk of known output size; null on size mismatch or bad data. */
    fun inflateRaw(packed: ByteArray, expected: Int): ByteArray? {
        if (packed.isEmpty() || expected <= 0) return null
        val inf = Inflater(true)
        return try {
            // nowrap streams need one extra dummy byte at the end of the input
            val input = packed.copyOf(packed.size + 1)
            inf.setInput(input)
            val out = ByteArray(expected)
            var n = 0
            while (n < expected && !inf.finished()) {
                val r = inf.inflate(out, n, expected - n)
                if (r == 0 && (inf.needsInput() || inf.needsDictionary())) break
                n += r
            }
            if (n == expected) out else null
        } catch (_: Exception) { null } finally { inf.end() }
    }

    fun readRange(f: File, off: Long, len: Int): ByteArray? = try {
        RandomAccessFile(f, "r").use { r ->
            if (off < 0 || off + len > r.length()) null else ByteArray(len).also { r.seek(off); r.readFully(it) }
        }
    } catch (_: Exception) { null }

    /** Millimetres (uint16 LE) of one depth map, or null. */
    fun load(blob: File, ref: DepthRef): ShortArray? {
        val packed = readRange(blob, ref.off, ref.len) ?: return null
        val raw = inflateRaw(packed, ref.w * ref.h * 2) ?: return null
        val out = ShortArray(ref.w * ref.h)
        ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(out)
        return out
    }

    /** Turbo palette (DepthViz): near red -> yellow -> green -> cyan -> blue far; 0 mm transparent. */
    fun colorize(mm: ShortArray, maxM: Float = 5f): IntArray = IntArray(mm.size) { i ->
        val v = mm[i].toInt() and 0xffff
        if (v == 0) 0 else DepthViz.color(v / 1000f, maxM)
    }

    /** Confidence levels (0..2) of a raw depth map: conf.zlib.bin next to [depthBlob], or null. */
    fun loadConf(depthBlob: File, ref: DepthRef): ByteArray? {
        if (ref.confOff < 0 || ref.confLen <= 0 || depthBlob.name != "depth.zlib.bin") return null
        val f = File(depthBlob.parentFile, "conf.zlib.bin")
        val packed = readRange(f, ref.confOff, ref.confLen) ?: return null
        return inflateRaw(packed, ref.w * ref.h)
    }

    /**
     * Viewer overlay: depth through the confidence + edge-aware filter (raw depth with its confidence when
     * recorded), turbo colours, filtered-out pixels transparent.
     */
    fun colorizeFiltered(blob: File, ref: DepthRef, mm: ShortArray, maxM: Float = 5f): IntArray =
        DepthViz.filteredArgb(mm, ref.w, ref.h, loadConf(blob, ref), confLevels = true, maxM = maxM)

    /** Median of the non-zero values (mm), or 0. */
    fun medianMm(mm: ShortArray): Int {
        val v = mm.map { it.toInt() and 0xffff }.filter { it > 0 }.sorted()
        return if (v.isEmpty()) 0 else v[v.size / 2]
    }
}

// ------------------------------------------------------------------------------------------ PLY

/** Vertices (xyz), optional colours (0xRRGGBB) and triangles (vertex indices) of a PLY file. */
class PlyData(val xyz: FloatArray, val rgb: IntArray?, val tris: IntArray) {
    val vertexCount get() = xyz.size / 3
}

object Ply {
    private class Prop(val name: String, val type: String, val listCount: String?)
    private class Element(val name: String, val count: Int, val props: MutableList<Prop> = mutableListOf())

    private fun size(type: String): Int = when (type) {
        "char", "uchar", "int8", "uint8" -> 1
        "short", "ushort", "int16", "uint16" -> 2
        "int", "uint", "float", "int32", "uint32", "float32" -> 4
        "double", "float64" -> 8
        else -> throw IllegalArgumentException("ply type $type")
    }

    private fun read(b: ByteBuffer, type: String): Double = when (type) {
        "char", "int8" -> b.get().toDouble()
        "uchar", "uint8" -> (b.get().toInt() and 0xff).toDouble()
        "short", "int16" -> b.short.toDouble()
        "ushort", "uint16" -> (b.short.toInt() and 0xffff).toDouble()
        "int", "int32" -> b.int.toDouble()
        "uint", "uint32" -> (b.int.toLong() and 0xffffffffL).toDouble()
        "float", "float32" -> b.float.toDouble()
        "double", "float64" -> b.double
        else -> throw IllegalArgumentException("ply type $type")
    }

    /** binary_little_endian, binary_big_endian or ascii PLY with a vertex element (x,y,z[,red,green,blue]) and optional faces. */
    fun parse(bytes: ByteArray, maxVertices: Int = 2_000_000): PlyData? {
        val endTag = "end_header".toByteArray()
        var hEnd = -1
        outer@ for (i in 0..bytes.size - endTag.size) {
            for (k in endTag.indices) if (bytes[i + k] != endTag[k]) continue@outer
            hEnd = i; break
        }
        if (hEnd < 0) return null
        var body = hEnd + endTag.size
        while (body < bytes.size && bytes[body] != '\n'.code.toByte()) body++
        body++
        val header = String(bytes, 0, hEnd, Charsets.US_ASCII).lines().map { it.trim() }
        if (header.firstOrNull() != "ply") return null
        var format = ""
        val elements = mutableListOf<Element>()
        for (l in header) {
            val p = l.split(Regex("\\s+"))
            when (p[0]) {
                "format" -> format = p.getOrElse(1) { "" }
                "element" -> elements.add(Element(p[1], p.getOrNull(2)?.toIntOrNull() ?: 0))
                "property" -> elements.lastOrNull()?.props?.add(
                    if (p.getOrNull(1) == "list") Prop(p[4], p[3], p[2]) else Prop(p[2], p[1], null))
            }
        }
        val vEl = elements.firstOrNull { it.name == "vertex" } ?: return null
        if (vEl.count > maxVertices) return null
        val xyz = FloatArray(vEl.count * 3)
        val names = vEl.props.map { it.name }
        val ix = names.indexOf("x"); val iy = names.indexOf("y"); val iz = names.indexOf("z")
        if (ix < 0 || iy < 0 || iz < 0) return null
        val ir = names.indexOf("red"); val ig = names.indexOf("green"); val ib = names.indexOf("blue")
        val rgb = if (ir >= 0 && ig >= 0 && ib >= 0) IntArray(vEl.count) else null
        val tris = ArrayList<Int>()
        val vals = DoubleArray(vEl.props.size)

        fun handleVertex(n: Int) {
            xyz[n * 3] = vals[ix].toFloat(); xyz[n * 3 + 1] = vals[iy].toFloat(); xyz[n * 3 + 2] = vals[iz].toFloat()
            rgb?.set(n, (vals[ir].toInt().coerceIn(0, 255) shl 16) or (vals[ig].toInt().coerceIn(0, 255) shl 8) or vals[ib].toInt().coerceIn(0, 255))
        }
        fun handleFace(idx: List<Int>) {
            for (k in 1 until idx.size - 1) {
                val a = idx[0]; val b = idx[k]; val c = idx[k + 1]
                if (a in 0 until vEl.count && b in 0 until vEl.count && c in 0 until vEl.count) { tris.add(a); tris.add(b); tris.add(c) }
            }
        }

        try {
            if (format == "ascii") {
                val tokens = String(bytes, body, bytes.size - body, Charsets.US_ASCII).split(Regex("\\s+")).filter { it.isNotEmpty() }
                var t = 0
                for (el in elements) for (n in 0 until el.count) {
                    var face: List<Int>? = null
                    el.props.forEachIndexed { pi, pr ->
                        if (pr.listCount != null) {
                            val c = tokens[t++].toDouble().toInt()
                            val l = List(c) { tokens[t++].toDouble().toInt() }
                            if (pr.name == "vertex_indices" || pr.name == "vertex_index") face = l
                        } else if (el === vEl) vals[pi] = tokens[t++].toDouble() else t++
                    }
                    if (el === vEl) handleVertex(n)
                    face?.let { if (el.name == "face") handleFace(it) }
                }
            } else {
                val order = if (format == "binary_big_endian") ByteOrder.BIG_ENDIAN else ByteOrder.LITTLE_ENDIAN
                val b = ByteBuffer.wrap(bytes, body, bytes.size - body).order(order)
                for (el in elements) {
                    // fast path: vertex element of fixed-size scalars
                    for (n in 0 until el.count) {
                        var face: List<Int>? = null
                        el.props.forEachIndexed { pi, pr ->
                            if (pr.listCount != null) {
                                val c = read(b, pr.listCount).toInt()
                                val l = List(c) { read(b, pr.type).toInt() }
                                if (pr.name == "vertex_indices" || pr.name == "vertex_index") face = l
                            } else if (el === vEl) vals[pi] = read(b, pr.type) else b.position(b.position() + size(pr.type))
                        }
                        if (el === vEl) handleVertex(n)
                        face?.let { if (el.name == "face") handleFace(it) }
                    }
                }
            }
        } catch (_: Exception) {
            return null
        }
        return PlyData(xyz, rgb, tris.toIntArray())
    }

    fun read(f: File): PlyData? = if (f.isFile) parse(f.readBytes()) else null
}

// ------------------------------------------------------------------------------------------ charts

/** One chart: [columns] of [file] (a CSV in the session, or "raw:<sensor>" for extras/sensors_raw.csv). */
data class PlotDef(val title: String, val file: String, val columns: List<String>, val unit: String)

object SensorPlots {
    val defs = listOf(
        PlotDef("Accelerometer (raw)", "accel.csv", listOf("x", "y", "z"), "g"),
        PlotDef("Gyroscope", "gyro.csv", listOf("x", "y", "z"), "rad/s"),
        PlotDef("Magnetometer", "mag.csv", listOf("x", "y", "z"), "µT"),
        PlotDef("User acceleration", "imu.csv", listOf("ax", "ay", "az"), "g"),
        PlotDef("Rotation rate (calibrated)", "imu.csv", listOf("gx", "gy", "gz"), "rad/s"),
        PlotDef("Gravity", "imu.csv", listOf("grx", "gry", "grz"), "g"),
        PlotDef("Orientation (game rotation quaternion)", "imu.csv", listOf("qx", "qy", "qz", "qw"), ""),
        PlotDef("Heading", "imu.csv", listOf("heading"), "deg"),
        PlotDef("Relative altitude", "altimeter.csv", listOf("rel_alt_m"), "m"),
        PlotDef("Air pressure", "altimeter.csv", listOf("pressure_kpa"), "kPa"),
        PlotDef("GPS speed / accuracy", "location.csv", listOf("speed", "hacc", "vacc"), "m/s, m"),
        PlotDef("GPS altitude", "location.csv", listOf("alt"), "m"),
        PlotDef("Thermal state / battery", "status.csv", listOf("thermal", "battery"), "0-3, 0-1"),
        PlotDef("Battery temperature", "extras/device_status.csv", listOf("battery_temp_c"), "°C"),
        PlotDef("Thermal headroom (10 s)", "extras/device_status.csv", listOf("thermal_headroom_10s"), "1 = throttling"),
        PlotDef("GNSS satellites", "extras/gnss_status.csv", listOf("n_sats", "n_used"), "count"),
        PlotDef("Camera exposure / ISO / ambient", "frames", listOf("exp_ms", "iso_100", "amb"), "ms, ISO/100, -"),
    )

    /** Sensors in extras/sensors_raw.csv already shown from the FORMAT.md CSVs above. */
    val rawCovered = setOf("accel", "gyro", "gyro_uncal", "mag", "mag_uncal", "pressure", "gravity", "game_rotation_vector")

    val rawUnits = mapOf(
        "light" to "lux", "proximity" to "cm", "temperature" to "°C", "humidity" to "%",
        "accel_uncal" to "m/s² (v3..v5 bias)", "linear_accel" to "m/s²",
        "rotation_vector" to "quaternion", "geomag_rotation_vector" to "quaternion",
    )

    /** Frames table for the camera chart: t, exposure (ms), iso/100, ambient intensity. */
    fun cameraTable(frames: List<FrameRec>): CsvTable? {
        if (frames.none { !it.exp.isNaN() || !it.iso.isNaN() || !it.amb.isNaN() }) return null
        return CsvTable(listOf("t", "exp_ms", "iso_100", "amb"), listOf(
            DoubleArray(frames.size) { frames[it].t },
            DoubleArray(frames.size) { frames[it].exp * 1000 },
            DoubleArray(frames.size) { frames[it].iso / 100 },
            DoubleArray(frames.size) { frames[it].amb },
        ))
    }
}

/**
 * Min/max decimation of (x, y) into at most [buckets] x-buckets over [x0, x1]: each bucket contributes its
 * min and max (in x order), so spikes survive. NaNs are skipped. Returns interleaved x,y floats.
 */
fun decimateMinMax(xs: DoubleArray, ys: DoubleArray, x0: Double, x1: Double, buckets: Int): FloatArray {
    val n = min(xs.size, ys.size)
    if (n == 0 || buckets <= 0 || x1 <= x0) return FloatArray(0)
    if (n <= buckets * 2) {
        val out = ArrayList<Float>(n * 2)
        for (i in 0 until n) if (!xs[i].isNaN() && !ys[i].isNaN()) { out.add(xs[i].toFloat()); out.add(ys[i].toFloat()) }
        return out.toFloatArray()
    }
    val minI = IntArray(buckets) { -1 }; val maxI = IntArray(buckets) { -1 }
    val span = x1 - x0
    for (i in 0 until n) {
        val x = xs[i]; val y = ys[i]
        if (x.isNaN() || y.isNaN()) continue
        val b = (((x - x0) / span) * buckets).toInt().coerceIn(0, buckets - 1)
        if (minI[b] < 0 || y < ys[minI[b]]) minI[b] = i
        if (maxI[b] < 0 || y > ys[maxI[b]]) maxI[b] = i
    }
    val out = ArrayList<Float>(buckets * 4)
    for (b in 0 until buckets) {
        if (minI[b] < 0) continue
        val a = min(minI[b], maxI[b]); val c = max(minI[b], maxI[b])
        out.add(xs[a].toFloat()); out.add(ys[a].toFloat())
        if (c != a) { out.add(xs[c].toFloat()); out.add(ys[c].toFloat()) }
    }
    return out.toFloatArray()
}

// ------------------------------------------------------------------------------------------ session list

data class SessionSummary(
    val dir: File,
    val name: String,
    val mode: String,
    val durationS: Double?,
    val bytes: Long,
    val frames: Int?,
    val complete: Boolean,
) {
    val modeLabel: String get() = modeLabel(mode)
}

fun modeLabel(mode: String): String = when (mode) {
    "arcore_rgbd" -> "A: ARCore RGB-D"
    "multicam" -> "B: Camera2 multi-cam"
    "sensors" -> "Sensors only"
    else -> mode
}

object SessionScan {
    fun list(root: File): List<SessionSummary> =
        (root.listFiles()?.filter { it.isDirectory } ?: emptyList()).map { summarize(it) }.sortedByDescending { it.name }

    fun dirSize(dir: File): Long = dir.walkTopDown().filter { it.isFile }.sumOf { it.length() }

    fun summarize(dir: File): SessionSummary {
        val meta = File(dir, "session.json").takeIf { it.isFile }?.let { MiniJson.obj(it.readText()) }
        val mode = meta?.str("mode") ?: dir.name.substringAfter('_').substringAfter('_').ifEmpty { "?" }
        val s = meta?.num("start_uptime"); val e = meta?.num("end_uptime")
        val counts = meta?.obj("counts")
        var frames: Int? = when (mode) {
            "multicam" -> counts?.num("cam_samples")?.toInt()
            else -> counts?.num("frames")?.toInt()?.takeIf { it > 0 || mode == "arcore_rgbd" }
        }
        if (frames == null && mode == "arcore_rgbd") frames = countLines(File(dir, "frames.jsonl"))
        return SessionSummary(dir, dir.name, mode, if (s != null && e != null && e >= s) e - s else null,
            dirSize(dir), frames, File(dir, "DONE").isFile || meta != null)
    }

    fun countLines(f: File): Int? = if (f.isFile) f.bufferedReader().useLines { l -> l.count { it.isNotBlank() } } else null

    /** All files under [dir], relative path + size, sorted by path. */
    fun files(dir: File): List<Pair<String, Long>> =
        dir.walkTopDown().filter { it.isFile }.map { it.relativeTo(dir).invariantSeparatorsPath to it.length() }.sortedBy { it.first }.toList()
}

fun fmtBytes(b: Long): String = when {
    b >= 1L shl 30 -> String.format(Locale.US, "%.2f GB", b / (1L shl 30).toDouble())
    b >= 1L shl 20 -> String.format(Locale.US, "%.1f MB", b / (1L shl 20).toDouble())
    b >= 1L shl 10 -> String.format(Locale.US, "%.1f KB", b / 1024.0)
    else -> "$b B"
}

fun fmtDuration(s: Double?): String {
    if (s == null || s.isNaN()) return "?"
    val total = s.toLong()
    return if (total >= 3600) String.format(Locale.US, "%d:%02d:%02d", total / 3600, total / 60 % 60, total % 60)
    else String.format(Locale.US, "%d:%02d", total / 60, total % 60)
}
