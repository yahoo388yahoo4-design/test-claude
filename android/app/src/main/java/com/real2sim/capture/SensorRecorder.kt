package com.real2sim.capture

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.location.GnssMeasurementsEvent
import android.location.GnssStatus
import android.location.Location
import android.location.LocationManager
import android.location.OnNmeaMessageListener
import android.media.MediaRecorder
import android.os.BatteryManager
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.PowerManager
import android.os.SystemClock
import androidx.core.location.LocationListenerCompat
import org.json.JSONArray
import org.json.JSONObject
import java.util.concurrent.Executor

/**
 * Everything that is not a camera: IMU at the fastest rate the HAL gives, magnetometer, rotation
 * vectors, barometer, temperature/humidity/light, location (fused + GPS), raw GNSS measurements,
 * NMEA, thermal + battery status, optional audio. All rows carry CLOCK_BOOTTIME ns.
 */
class SensorRecorder(private val ctx: Context, private val s: SessionWriter) : SensorEventListener {
    private val sm = ctx.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val lm = ctx.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private val pm = ctx.getSystemService(Context.POWER_SERVICE) as PowerManager
    private val thread = HandlerThread("sensors", android.os.Process.THREAD_PRIORITY_URGENT_DISPLAY).apply { start() }
    private val handler = Handler(thread.looper)
    private val executor = Executor { handler.post(it) }
    private val names = mutableMapOf<Int, String>()
    private val counts = mutableMapOf<String, Long>()
    private var audio: MediaRecorder? = null
    private var statusTicker: Runnable? = null

    private val wanted = listOf(
        Sensor.TYPE_ACCELEROMETER to "accel",
        Sensor.TYPE_ACCELEROMETER_UNCALIBRATED to "accel_uncal",
        Sensor.TYPE_GYROSCOPE to "gyro",
        Sensor.TYPE_GYROSCOPE_UNCALIBRATED to "gyro_uncal",
        Sensor.TYPE_MAGNETIC_FIELD to "mag",
        Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED to "mag_uncal",
        Sensor.TYPE_GRAVITY to "gravity",
        Sensor.TYPE_LINEAR_ACCELERATION to "linear_accel",
        Sensor.TYPE_ROTATION_VECTOR to "rotation_vector",
        Sensor.TYPE_GAME_ROTATION_VECTOR to "game_rotation_vector",
        Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR to "geomag_rotation_vector",
        Sensor.TYPE_PRESSURE to "pressure",
        Sensor.TYPE_AMBIENT_TEMPERATURE to "temperature",
        Sensor.TYPE_RELATIVE_HUMIDITY to "humidity",
        Sensor.TYPE_LIGHT to "light",
        Sensor.TYPE_PROXIMITY to "proximity",
    )

    private val rawHeader = "t,sensor,v0,v1,v2,v3,v4,v5,accuracy"
    private val G = SensorManager.GRAVITY_EARTH

    // Latest value per stream for the fused imu.csv row (emitted on every gyro sample).
    private var linAcc = FloatArray(3); private var grav = FloatArray(3); private var quat = FloatArray(4)
    private var magV = FloatArray(3); private var magAcc = -1; private var heading = -1.0
    private var p0: Float = Float.NaN

    fun start(recordAudio: Boolean) {
        val inventory = JSONArray()
        for ((type, name) in wanted) {
            val sensor = sm.getDefaultSensor(type) ?: continue
            names[type] = name
            inventory.put(JSONObject().apply {
                put("name", name); put("hw_name", sensor.name); put("vendor", sensor.vendor)
                put("min_delay_us", sensor.minDelay); put("max_range", sensor.maximumRange.toDouble())
                put("resolution", sensor.resolution.toDouble()); put("fifo_max", sensor.fifoMaxEventCount)
                put("power_ma", sensor.power.toDouble()); put("id", sensor.id)
            })
            // SENSOR_DELAY_FASTEST = 0 us -> HAL maximum (needs HIGH_SAMPLING_RATE_SENSORS above 200 Hz on 12+).
            sm.registerListener(this, sensor, SensorManager.SENSOR_DELAY_FASTEST, 0, handler)
        }
        s.meta.optJSONObject("android")?.put("sensors", inventory)
        startLocation()
        startStatus()
        if (recordAudio) startAudio()
    }

    /**
     * Files in FORMAT.md terms (iOS units and signs, so one converter serves both phones):
     *   accel.csv  t,x,y,z  raw accelerometer in g, iOS sign (= -Android m/s^2 / 9.80665)
     *   gyro.csv   t,x,y,z  raw gyro rad/s (uncalibrated if available, else calibrated)
     *   mag.csv    t,x,y,z  raw magnetometer uT
     *   imu.csv    fused row per calibrated-gyro sample (user accel g, gyro, gravity g, game-rotation quat, mag, heading)
     *   altimeter.csv t,rel_alt_m,pressure_kpa
     * and extras/sensors_raw.csv with every event of every sensor in Android-native SI units.
     * Device axes are the same on both platforms (x right, y top of screen, z out of screen).
     */
    override fun onSensorChanged(e: SensorEvent) {
        val name = names[e.sensor.type] ?: return
        val v = e.values
        val t = "%.9f".format(java.util.Locale.US, e.timestamp / 1e9)
        val sb = StringBuilder(96).append(t).append(',').append(name)
        for (i in 0 until 6) { sb.append(','); if (i < v.size) sb.append(v[i]) }
        sb.append(',').append(e.accuracy)
        s.csv("extras/sensors_raw.csv", rawHeader, sb.toString())
        counts[name] = (counts[name] ?: 0L) + 1
        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> s.csv("accel.csv", "t,x,y,z", "$t,${-v[0] / G},${-v[1] / G},${-v[2] / G}")
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED -> s.csv("gyro.csv", "t,x,y,z", "$t,${v[0]},${v[1]},${v[2]}")
            Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED -> s.csv("mag.csv", "t,x,y,z", "$t,${v[0]},${v[1]},${v[2]}")
            Sensor.TYPE_MAGNETIC_FIELD -> {
                magV = v.copyOf(3); magAcc = e.accuracy
                if (names.values.none { it == "mag_uncal" }) s.csv("mag.csv", "t,x,y,z", "$t,${v[0]},${v[1]},${v[2]}")
            }
            Sensor.TYPE_LINEAR_ACCELERATION -> linAcc = v.copyOf(3)
            Sensor.TYPE_GRAVITY -> grav = v.copyOf(3)
            Sensor.TYPE_GAME_ROTATION_VECTOR -> quat = floatArrayOf(v[0], v[1], v[2], if (v.size > 3) v[3] else 0f)
            Sensor.TYPE_ROTATION_VECTOR -> {
                val r = FloatArray(9); SensorManager.getRotationMatrixFromVector(r, v)
                val o = FloatArray(3); SensorManager.getOrientation(r, o)
                heading = (Math.toDegrees(o[0].toDouble()) + 360.0) % 360.0
            }
            Sensor.TYPE_PRESSURE -> {
                if (p0.isNaN()) p0 = v[0]
                val rel = SensorManager.getAltitude(p0, v[0]) - 0f
                s.csv("altimeter.csv", "t,rel_alt_m,pressure_kpa", "$t,$rel,${v[0] / 10f}")
            }
            Sensor.TYPE_GYROSCOPE -> {
                if (names.values.none { it == "gyro_uncal" }) s.csv("gyro.csv", "t,x,y,z", "$t,${v[0]},${v[1]},${v[2]}")
                s.csv("imu.csv", "t,ax,ay,az,gx,gy,gz,grx,gry,grz,qx,qy,qz,qw,mx,my,mz,mag_acc,heading",
                    "$t,${-linAcc[0] / G},${-linAcc[1] / G},${-linAcc[2] / G},${v[0]},${v[1]},${v[2]}," +
                        "${-grav[0] / G},${-grav[1] / G},${-grav[2] / G},${quat[0]},${quat[1]},${quat[2]},${quat[3]}," +
                        "${magV[0]},${magV[1]},${magV[2]},${magAccIos()},$heading")
            }
        }
    }

    /** Android SENSOR_STATUS_* (0 unreliable..3 high) -> CoreMotion-style -1..2. */
    private fun magAccIos() = if (magAcc < 0) -1 else (magAcc - 1).coerceIn(-1, 2)

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    // ---------------------------------------------------------------- location / GNSS
    // LocationListenerCompat (not a SAM lambda): on API 29 onProviderEnabled/Disabled/onStatusChanged are still
    // abstract in the platform interface, so a lambda-generated class would throw AbstractMethodError on toggle.
    private val locListener = object : LocationListenerCompat {
        override fun onLocationChanged(loc: Location) {
            // FORMAT.md: t, unix, lat, lon, alt, ellipsoidal_alt, hacc, vacc, speed, speed_acc, course, course_acc, floor, simulated
            // Some providers (seen on the emulator) fill elapsedRealtimeNanos with unix time: fall back to now.
            val now = SystemClock.elapsedRealtimeNanos()
            val t = (if (loc.elapsedRealtimeNanos > 0 && kotlin.math.abs(loc.elapsedRealtimeNanos - now) < 3_600_000_000_000L) loc.elapsedRealtimeNanos else now) / 1e9
            fun f(has: Boolean, v: Number) = if (has) v.toString() else ""
            val msl = if (Build.VERSION.SDK_INT >= 34 && loc.hasMslAltitude()) loc.mslAltitudeMeters.toString() else ""
            val line = listOf(
                "%.9f".format(java.util.Locale.US, t), "%.3f".format(java.util.Locale.US, loc.time / 1000.0), loc.latitude, loc.longitude,
                msl, f(loc.hasAltitude(), loc.altitude), f(loc.hasAccuracy(), loc.accuracy), f(loc.hasVerticalAccuracy(), loc.verticalAccuracyMeters),
                f(loc.hasSpeed(), loc.speed), f(loc.hasSpeedAccuracy(), loc.speedAccuracyMetersPerSecond),
                f(loc.hasBearing(), loc.bearing), f(loc.hasBearingAccuracy(), loc.bearingAccuracyDegrees), "",
                if (Build.VERSION.SDK_INT >= 31) loc.isMock else false,
            ).joinToString(",")
            s.csv("location.csv", "t,unix,lat,lon,alt,ellipsoidal_alt,hacc,vacc,speed,speed_acc,course,course_acc,floor,simulated", line)
            s.csv("extras/location_providers.csv", "t,provider", "%.9f,%s".format(java.util.Locale.US, t, loc.provider))
        }
    }

    private val gnssCb = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(ev: GnssMeasurementsEvent) {
            val c = ev.clock
            // Wall/boot pairing for the GNSS clock: elapsedRealtimeNanos is on the same clock as everything else.
            val tBoot = "%.9f".format(java.util.Locale.US, (if (c.hasElapsedRealtimeNanos()) c.elapsedRealtimeNanos else SystemClock.elapsedRealtimeNanos()) / 1e9)
            for (m in ev.measurements) {
                val line = listOf(
                    tBoot, c.timeNanos, if (c.hasFullBiasNanos()) c.fullBiasNanos else "", if (c.hasBiasNanos()) c.biasNanos else "",
                    m.svid, m.constellationType, m.timeOffsetNanos, m.state, m.receivedSvTimeNanos, m.receivedSvTimeUncertaintyNanos,
                    m.cn0DbHz, m.pseudorangeRateMetersPerSecond, m.pseudorangeRateUncertaintyMetersPerSecond,
                    m.accumulatedDeltaRangeState, m.accumulatedDeltaRangeMeters, m.accumulatedDeltaRangeUncertaintyMeters,
                    if (m.hasCarrierFrequencyHz()) m.carrierFrequencyHz else "", m.multipathIndicator,
                    if (m.hasSnrInDb()) m.snrInDb else "",
                ).joinToString(",")
                s.csv("extras/gnss_raw.csv",
                    "t,clock_time_ns,full_bias_ns,bias_ns,svid,constellation,time_offset_ns,state,rx_sv_time_ns,rx_sv_time_unc_ns,cn0_dbhz,prr_mps,prr_unc_mps,adr_state,adr_m,adr_unc_m,carrier_hz,multipath,snr_db",
                    line)
            }
        }
    }

    private val statusCb = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(st: GnssStatus) {
            val t = "%.9f".format(java.util.Locale.US, SystemClock.elapsedRealtimeNanos() / 1e9)
            var used = 0
            for (i in 0 until st.satelliteCount) if (st.usedInFix(i)) used++
            s.csv("extras/gnss_status.csv", "t,n_sats,n_used", "$t,${st.satelliteCount},$used")
        }
    }

    private val nmea = OnNmeaMessageListener { msg, _ ->
        s.csv("extras/nmea.csv", "t,sentence", "${SystemClock.elapsedRealtimeNanos() / 1e9},\"${msg.trim()}\"")
    }

    @SuppressLint("MissingPermission")
    private fun startLocation() {
        try {
            val providers = mutableListOf<String>()
            if (Build.VERSION.SDK_INT >= 31 && lm.hasProvider(LocationManager.FUSED_PROVIDER)) providers += LocationManager.FUSED_PROVIDER
            if (lm.isProviderEnabled(LocationManager.GPS_PROVIDER)) providers += LocationManager.GPS_PROVIDER
            if (lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) providers += LocationManager.NETWORK_PROVIDER
            for (p in providers) lm.requestLocationUpdates(p, 0L, 0f, locListener, thread.looper)
            if (Build.VERSION.SDK_INT >= 30) {
                lm.registerGnssMeasurementsCallback(executor, gnssCb)
                lm.registerGnssStatusCallback(executor, statusCb)
                lm.addNmeaListener(executor, nmea)
            } else {
                @Suppress("DEPRECATION") lm.registerGnssMeasurementsCallback(gnssCb, handler)
                lm.registerGnssStatusCallback(statusCb, handler)
                lm.addNmeaListener(nmea, handler)
            }
            s.meta.optJSONObject("android")?.put("location_providers", JSONArray(providers))
            if (Build.VERSION.SDK_INT >= 30) {
                val caps = lm.gnssCapabilities
                s.meta.optJSONObject("android")?.put("gnss_capabilities", JSONObject().apply {
                    put("raw", caps.toString())
                    if (Build.VERSION.SDK_INT >= 31) {
                        put("measurements", caps.hasMeasurements())
                        put("navigation_messages", caps.hasNavigationMessages())
                        put("antenna_info", caps.hasAntennaInfo())
                    }
                })
                s.meta.optJSONObject("android")?.put("gnss_hardware_model", lm.gnssHardwareModelName ?: "")
            }
        } catch (e: SecurityException) {
            s.meta.put("location_error", "permission denied")
        } catch (e: Exception) {
            s.meta.put("location_error", e.toString())
        }
    }

    // ---------------------------------------------------------------- thermal / battery (1 Hz)
    private fun startStatus() {
        val r = object : Runnable {
            override fun run() {
                val (unix, t) = s.clockPair()
                val ts = "%.9f".format(java.util.Locale.US, t)
                val us = "%.3f".format(java.util.Locale.US, unix)
                s.csv("clock.csv", "uptime,unix", "$ts,$us")
                val bat = ctx.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED))
                val level = bat?.getIntExtra(BatteryManager.EXTRA_LEVEL, -1) ?: -1
                val scale = bat?.getIntExtra(BatteryManager.EXTRA_SCALE, 100) ?: 100
                val temp = (bat?.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0) ?: 0) / 10.0
                val volt = bat?.getIntExtra(BatteryManager.EXTRA_VOLTAGE, 0) ?: 0
                val st = bat?.getIntExtra(BatteryManager.EXTRA_STATUS, 0) ?: 0
                val thermal = pm.currentThermalStatus
                val headroom = if (Build.VERSION.SDK_INT >= 30) pm.getThermalHeadroom(10) else Float.NaN
                // FORMAT.md status.csv: thermal 0 nominal..3 critical (Android 0..6 squeezed), battery 0..1
                val thermalIos = when { thermal <= 0 -> 0; thermal <= 2 -> thermal; else -> 3 }
                val state = when (st) { BatteryManager.BATTERY_STATUS_CHARGING -> "charging"; BatteryManager.BATTERY_STATUS_FULL -> "full"
                    BatteryManager.BATTERY_STATUS_DISCHARGING, BatteryManager.BATTERY_STATUS_NOT_CHARGING -> "unplugged"; else -> "unknown" }
                s.csv("status.csv", "t,unix,thermal,battery,battery_state,low_power",
                    "$ts,$us,$thermalIos,${level.toDouble() / scale},$state,${pm.isPowerSaveMode}")
                s.csv("extras/device_status.csv", "t,battery_temp_c,battery_mv,thermal_status_android,thermal_headroom_10s",
                    "$ts,$temp,$volt,$thermal,$headroom")
                handler.postDelayed(this, 1000)
            }
        }
        statusTicker = r
        handler.post(r)
    }

    // ---------------------------------------------------------------- audio
    private fun startAudio() {
        try {
            val f = s.file("audio.m4a")
            val rec = if (Build.VERSION.SDK_INT >= 31) MediaRecorder(ctx) else @Suppress("DEPRECATION") MediaRecorder()
            rec.setAudioSource(MediaRecorder.AudioSource.UNPROCESSED.takeIf { Build.VERSION.SDK_INT >= 24 } ?: MediaRecorder.AudioSource.MIC)
            rec.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4)
            rec.setAudioEncoder(MediaRecorder.AudioEncoder.AAC)
            rec.setAudioSamplingRate(48000)
            rec.setAudioEncodingBitRate(192000)
            rec.setOutputFile(f.absolutePath)
            rec.prepare()
            val t0 = SystemClock.elapsedRealtimeNanos()
            rec.start()
            val t1 = SystemClock.elapsedRealtimeNanos()
            s.meta.put("audio", JSONObject().apply {
                put("file", "audio.m4a"); put("start_t_before", t0 / 1e9); put("start_t_after", t1 / 1e9)
                put("note", "MediaRecorder start latency is not exposed; sync to video by cross-correlation if needed")
            })
            audio = rec
        } catch (e: Exception) {
            s.meta.put("audio_error", e.toString())
        }
    }

    fun stop() {
        sm.unregisterListener(this)
        try {
            lm.removeUpdates(locListener)
            lm.unregisterGnssMeasurementsCallback(gnssCb)
            lm.unregisterGnssStatusCallback(statusCb)
            lm.removeNmeaListener(nmea)
        } catch (_: Exception) {}
        statusTicker?.let { handler.removeCallbacks(it) }
        audio?.let { try { it.stop() } catch (_: Exception) {}; it.release() }
        audio = null
        // Let in-flight callbacks drain, then record per-sensor counts.
        val done = java.util.concurrent.CountDownLatch(1)
        handler.post { done.countDown() }
        done.await(2, java.util.concurrent.TimeUnit.SECONDS)
        s.meta.put("sensor_counts", JSONObject(counts as Map<*, *>))
        thread.quitSafely()
    }

    fun summary(): String = counts.entries.joinToString(" ") { "${it.key}=${it.value}" }
}
