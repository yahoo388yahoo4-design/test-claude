package com.real2sim.capture

import org.json.JSONArray
import org.json.JSONObject

/**
 * org.json refuses NaN and infinite numbers: JSONObject.put(name, Double.NaN) throws, and a NaN that
 * got in another way (JSONArray(list), JSONObject(map)) makes toString(indent) throw and toString()
 * return null. Camera metadata (distortion, intrinsics, focus range) can contain such values, so
 * everything that is serialised goes through here: non-finite numbers become JSON null.
 */
object JsonSafe {
    /** A finite number as is, NaN / +-Infinity as JSON null. */
    fun num(v: Double): Any = if (v.isFinite()) v else JSONObject.NULL

    fun num(v: Float): Any = if (v.isFinite()) v.toDouble() else JSONObject.NULL

    /** Deep copy of [v] with non-finite numbers replaced by JSON null. Maps / collections become JSONObject / JSONArray. */
    fun clean(v: Any?): Any? = when (v) {
        null -> JSONObject.NULL
        is Double -> num(v)
        is Float -> num(v)
        is JSONObject -> JSONObject().also { out ->
            val keys = v.keys()
            while (keys.hasNext()) { val k = keys.next(); out.put(k, clean(v.opt(k))) }
        }
        is JSONArray -> JSONArray().also { out -> for (i in 0 until v.length()) out.put(clean(v.opt(i))) }
        is Map<*, *> -> JSONObject().also { out -> for ((k, x) in v) out.put(k.toString(), clean(x)) }
        is Collection<*> -> JSONArray().also { out -> for (x in v) out.put(clean(x)) }
        else -> v
    }

    /** toString(indent) that never throws: falls back to a cleaned copy. */
    fun stringify(o: JSONObject, indent: Int = 0): String {
        try {
            val s: String? = if (indent > 0) o.toString(indent) else o.toString()
            if (s != null) return s
        } catch (_: Exception) {}
        val c = clean(o) as JSONObject
        return if (indent > 0) c.toString(indent) else c.toString()
    }

    /** put that never throws: non-finite numbers are stored as null, other failures are dropped. */
    fun put(o: JSONObject, name: String, v: Any?) {
        try { o.put(name, clean(v)) } catch (_: Exception) {}
    }
}
