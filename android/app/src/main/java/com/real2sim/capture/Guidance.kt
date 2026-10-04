package com.real2sim.capture

import android.content.Context
import android.media.AudioManager
import android.media.ToneGenerator
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.speech.tts.TextToSpeech
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * Spoken and audible guidance for the navigation mode:
 *  - turn-by-turn speech ("turn left 40 degrees", "go straight 1.5 meters", "goal reached"), rate-limited
 *    and only when the instruction changes;
 *  - a parking-sensor beep whose rate rises as the nearest obstacle ahead gets closer.
 */
class Guidance(ctx: Context) {
    @Volatile var voice = true
    @Volatile var beeps = true
    private var tts: TextToSpeech? = null
    private var ready = false
    private val tone = try { ToneGenerator(AudioManager.STREAM_NOTIFICATION, 70) } catch (_: Exception) { null }
    private val h = Handler(Looper.getMainLooper())
    private var lastSpoken = ""
    private var lastSpokenMs = 0L
    @Volatile private var obstacleM = Float.POSITIVE_INFINITY
    private var beeping = false

    init {
        tts = TextToSpeech(ctx) { st -> ready = st == TextToSpeech.SUCCESS; if (ready) tts?.language = Locale.US }
    }

    fun say(text: String, force: Boolean = false, minGapMs: Long = 3500) {
        if (!voice || !ready) return
        val now = SystemClock.elapsedRealtime()
        if (!force && (text == lastSpoken && now - lastSpokenMs < 8000 || now - lastSpokenMs < minGapMs)) return
        lastSpoken = text; lastSpokenMs = now
        tts?.speak(text, TextToSpeech.QUEUE_FLUSH, null, "g$now")
    }

    /** Instruction text for a bearing (rad, CCW positive) and remaining distance (m). */
    fun instruction(bearing: Float, remaining: Float): String {
        val deg = Math.toDegrees(bearing.toDouble())
        return when {
            abs(deg) > 150 -> "Turn around"
            abs(deg) > 20 -> "Turn ${if (deg > 0) "left" else "right"} ${(abs(deg) / 5).roundToInt() * 5} degrees"
            abs(deg) > 8 -> "Bear ${if (deg > 0) "left" else "right"}, ${meters(remaining)}"
            else -> "Go straight ${meters(remaining)}"
        }
    }

    fun meters(m: Float) = if (m < 1f) "${(m * 100 / 5).roundToInt() * 5} centimeters" else "%.1f meters".format(Locale.US, m)

    /** Nearest obstacle ahead in metres; the beep rate follows it (silent beyond 1.2 m). */
    fun obstacle(m: Float) {
        obstacleM = m
        if (beeps && m < 1.2f && !beeping) { beeping = true; h.post(beep) }
    }

    private val beep = object : Runnable {
        override fun run() {
            val m = obstacleM
            if (!beeps || m >= 1.2f) { beeping = false; return }
            tone?.startTone(if (m < 0.3f) ToneGenerator.TONE_PROP_BEEP2 else ToneGenerator.TONE_PROP_BEEP, 60)
            h.postDelayed(this, (80 + (m / 1.2f) * 700).toLong())
        }
    }

    fun shutdown() {
        beeps = false
        tts?.shutdown(); tts = null
        tone?.release()
    }
}
