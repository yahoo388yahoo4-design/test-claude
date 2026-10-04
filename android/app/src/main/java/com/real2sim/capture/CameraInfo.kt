package com.real2sim.capture

import android.graphics.ImageFormat
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CameraMetadata
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.params.StreamConfigurationMap
import android.os.Build
import android.util.Range
import android.util.Rational
import android.util.Size
import android.util.SizeF
import org.json.JSONArray
import org.json.JSONObject

/** Camera2 inventory: every characteristic of every (logical and physical) camera, as JSON. */
object CameraInfo {

    fun toJson(v: Any?): Any? = when (v) {
        null -> JSONObject.NULL
        is Boolean, is Int, is Long, is String -> v
        is Float -> JsonSafe.num(v)
        is Double -> JsonSafe.num(v)
        is Size -> JSONArray(listOf(v.width, v.height))
        is SizeF -> JSONArray(listOf(JsonSafe.num(v.width), JsonSafe.num(v.height)))
        is Rational -> JsonSafe.num(v.toDouble())
        is Range<*> -> JSONArray(listOf(toJson(v.lower), toJson(v.upper)))
        is android.graphics.Rect -> JSONArray(listOf(v.left, v.top, v.right, v.bottom))
        is IntArray -> JSONArray(v.toList())
        is LongArray -> JSONArray(v.toList())
        is FloatArray -> JSONArray(v.map { JsonSafe.num(it) })
        is DoubleArray -> JSONArray(v.map { JsonSafe.num(it) })
        is BooleanArray -> JSONArray(v.toList())
        is ByteArray -> JSONArray(v.map { it.toInt() })
        is Array<*> -> JSONArray(v.map { toJson(it) })
        is StreamConfigurationMap -> streamMap(v)
        is android.hardware.camera2.params.BlackLevelPattern -> JSONArray((0..3).map { v.getOffsetForIndex(it % 2, it / 2) })
        is android.hardware.camera2.params.ColorSpaceTransform -> JSONArray((0..8).map { JsonSafe.num(v.getElement(it % 3, it / 3).toDouble()) })
        is android.hardware.camera2.params.OisSample -> JSONArray(listOf(v.timestamp, JsonSafe.num(v.xshift), JsonSafe.num(v.yshift)))
        is android.hardware.camera2.params.LensShadingMap -> JSONObject().apply { put("rows", v.rowCount); put("cols", v.columnCount) }
        else -> v.toString()
    }

    private val formats = mapOf(
        ImageFormat.YUV_420_888 to "YUV_420_888", ImageFormat.JPEG to "JPEG", ImageFormat.RAW_SENSOR to "RAW_SENSOR",
        ImageFormat.RAW10 to "RAW10", ImageFormat.RAW12 to "RAW12", ImageFormat.DEPTH16 to "DEPTH16",
        ImageFormat.DEPTH_POINT_CLOUD to "DEPTH_POINT_CLOUD", ImageFormat.PRIVATE to "PRIVATE",
        ImageFormat.Y8 to "Y8", ImageFormat.HEIC to "HEIC", ImageFormat.DEPTH_JPEG to "DEPTH_JPEG",
    )

    fun streamMap(m: StreamConfigurationMap): JSONObject {
        val o = JSONObject()
        for (f in m.outputFormats) {
            val sizes = m.getOutputSizes(f) ?: continue
            o.put(formats[f] ?: "fmt_$f", JSONArray(sizes.map { s ->
                JSONArray(listOf(s.width, s.height, m.getOutputMinFrameDuration(f, s), m.getOutputStallDuration(f, s)))
            }))
        }
        val hs = m.highSpeedVideoSizes
        if (hs.isNotEmpty()) o.put("high_speed_video", JSONArray(hs.map { s -> JSONArray(listOf(s.width, s.height, toJson(m.getHighSpeedVideoFpsRangesFor(s)))) }))
        o.put("_columns", "w,h,min_frame_duration_ns,stall_ns")
        return o
    }

    fun characteristics(c: CameraCharacteristics): JSONObject {
        val o = JSONObject()
        for (k in c.keys) {
            try { o.put(k.name, toJson(c.get(k))) } catch (e: Exception) { o.put(k.name, "error: $e") }
        }
        return o
    }

    fun capabilities(c: CameraCharacteristics): List<String> {
        val names = mapOf(
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR to "MANUAL_SENSOR",
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_RAW to "RAW",
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_DEPTH_OUTPUT to "DEPTH_OUTPUT",
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA to "LOGICAL_MULTI_CAMERA",
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MOTION_TRACKING to "MOTION_TRACKING",
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_BURST_CAPTURE to "BURST_CAPTURE",
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_READ_SENSOR_SETTINGS to "READ_SENSOR_SETTINGS",
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MONOCHROME to "MONOCHROME",
            16 to "ULTRA_HIGH_RESOLUTION_SENSOR",   // API 31 constant value
            CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE to "BACKWARD_COMPATIBLE",
        )
        return (c.get(CameraCharacteristics.REQUEST_AVAILABLE_CAPABILITIES) ?: intArrayOf()).map { names[it] ?: "cap_$it" }
    }

    /** Full inventory: logical cameras, their physical members, concurrent sets, DEPTH16 support. */
    fun inventory(cm: CameraManager): JSONObject {
        val out = JSONObject()
        val cams = JSONObject()
        val seen = mutableSetOf<String>()
        fun add(id: String, parent: String?) {
            if (!seen.add(id)) return
            val c = cm.getCameraCharacteristics(id)
            val o = characteristics(c)
            o.put("_capabilities", JSONArray(capabilities(c)))
            o.put("_physical_ids", JSONArray(c.physicalCameraIds.toList()))
            if (parent != null) o.put("_logical_parent", parent)
            val map = c.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
            o.put("_has_depth16", map?.outputFormats?.contains(ImageFormat.DEPTH16) == true)
            o.put("_has_raw_sensor", map?.outputFormats?.contains(ImageFormat.RAW_SENSOR) == true)
            cams.put(id, o)
            for (p in c.physicalCameraIds) add(p, id)
        }
        for (id in cm.cameraIdList) add(id, null)
        // Hidden ids: many OEMs (Xiaomi included) expose aux cameras only by id, not in cameraIdList.
        for (i in 0 until 16) {
            val id = i.toString()
            if (id !in seen) try { add(id, null); cams.optJSONObject(id)?.put("_not_in_id_list", true) } catch (_: Exception) {}
        }
        out.put("camera_id_list", JSONArray(cm.cameraIdList.toList()))
        out.put("cameras", cams)
        if (Build.VERSION.SDK_INT >= 30) {
            out.put("concurrent_camera_ids", JSONArray(cm.concurrentCameraIds.map { JSONArray(it.toList()) }))
        }
        return out
    }

    /** The per-frame CaptureResult fields that matter for reconstruction (compact, for frames.jsonl). */
    fun resultFields(r: CaptureResult, o: JSONObject) {
        fun <T> put(name: String, k: CaptureResult.Key<T>) { try { r.get(k)?.let { JsonSafe.put(o, name, toJson(it)) } } catch (_: Exception) {} }
        put("exposure_ns", CaptureResult.SENSOR_EXPOSURE_TIME)
        put("iso", CaptureResult.SENSOR_SENSITIVITY)
        put("frame_duration_ns", CaptureResult.SENSOR_FRAME_DURATION)
        put("rolling_shutter_skew_ns", CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW)
        put("focus_distance_diopters", CaptureResult.LENS_FOCUS_DISTANCE)
        put("focal_length_mm", CaptureResult.LENS_FOCAL_LENGTH)
        put("aperture", CaptureResult.LENS_APERTURE)
        put("focus_range", CaptureResult.LENS_FOCUS_RANGE)
        put("lens_state", CaptureResult.LENS_STATE)
        put("ois_mode", CaptureResult.LENS_OPTICAL_STABILIZATION_MODE)
        put("lens_intrinsics", CaptureResult.LENS_INTRINSIC_CALIBRATION)
        put("lens_distortion", CaptureResult.LENS_DISTORTION)
        put("lens_pose_translation", CaptureResult.LENS_POSE_TRANSLATION)
        put("lens_pose_rotation", CaptureResult.LENS_POSE_ROTATION)
        put("ois_samples", CaptureResult.STATISTICS_OIS_SAMPLES)
        put("ae_state", CaptureResult.CONTROL_AE_STATE)
        put("af_state", CaptureResult.CONTROL_AF_STATE)
        put("awb_state", CaptureResult.CONTROL_AWB_STATE)
        put("wb_gains", CaptureResult.COLOR_CORRECTION_GAINS)
        put("ccm", CaptureResult.COLOR_CORRECTION_TRANSFORM)
        put("crop_region", CaptureResult.SCALER_CROP_REGION)
        if (Build.VERSION.SDK_INT >= 30) put("zoom_ratio", CaptureResult.CONTROL_ZOOM_RATIO)
        put("noise_profile", CaptureResult.SENSOR_NOISE_PROFILE)
        put("neutral_point", CaptureResult.SENSOR_NEUTRAL_COLOR_POINT)
        put("video_stab_mode", CaptureResult.CONTROL_VIDEO_STABILIZATION_MODE)
        if (Build.VERSION.SDK_INT >= 31) put("active_physical_id", CaptureResult.LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID)
    }
}
