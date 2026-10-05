package com.real2sim.capture

import android.content.Context
import android.content.SharedPreferences
import androidx.core.content.edit

/** Capture modes on Android, worded like the iOS `CaptureMode` (short title in the picker, long title below it). */
enum class CaptureMode(val id: String, val shortTitle: String, val title: String) {
    RGBD("arcore_rgbd", "RGB-D", "A · RGB-D + poses (ARCore)"),
    MULTICAM("multicam", "Multi-cam", "B · Multi-camera + RAW (no poses)"),
    SENSORS("sensors", "Sensors", "C · Sensors only (IMU / GNSS)"),
}

/**
 * Main-screen settings (the Settings sheet), persisted in SharedPreferences "capture" like the iOS
 * `CaptureSettings` in UserDefaults. `upload_url` is shared with the Sessions screen.
 */
class CaptureSettings(private val prefs: SharedPreferences) {
    constructor(ctx: Context) : this(ctx.getSharedPreferences("capture", Context.MODE_PRIVATE))

    var mode: CaptureMode
        get() = CaptureMode.values().getOrElse(prefs.getInt("mode", 0)) { CaptureMode.RGBD }
        set(v) = prefs.edit { putInt("mode", v.ordinal) }
    var recordAudio by flag("audio", false)
    var rawDng by flag("raw_dng", false)
    var arcoreMp4 by flag("arcore_mp4", false)
    var lock by flag("lock_ae_af_awb", false)
    var hiResStills by flag("hires_stills", false)
    var oisOff by flag("ois_off", false)
    var geospatial by flag("geospatial", false)
    var uploadUrl: String
        get() = prefs.getString("upload_url", "") ?: ""
        set(v) = prefs.edit { putString("upload_url", v.trim()) }

    private fun flag(key: String, def: Boolean) = object : kotlin.properties.ReadWriteProperty<Any?, Boolean> {
        override fun getValue(thisRef: Any?, property: kotlin.reflect.KProperty<*>) = prefs.getBoolean(key, def)
        override fun setValue(thisRef: Any?, property: kotlin.reflect.KProperty<*>, value: Boolean) = prefs.edit { putBoolean(key, value) }
    }
}
