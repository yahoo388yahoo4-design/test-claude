package com.real2sim.capture.voice

import android.Manifest
import android.app.Activity
import android.app.AlertDialog
import android.content.pm.PackageManager
import android.graphics.Color
import android.graphics.drawable.GradientDrawable
import android.text.InputType
import android.view.Gravity
import android.view.View
import android.widget.Button
import android.widget.CheckBox
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.Spinner
import android.widget.TextView
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.json.JSONObject
import kotlin.math.abs

/** What voice commands can do on the nav screen; implemented by NavActivity. Called on the main thread. */
interface NavActions {
    fun robotConnected(): Boolean
    fun stopAll()
    /** Drive straight [meters] (+ forward) at [speedCms]; switches Guide to Manual. */
    fun move(meters: Double, speedCms: Double)
    fun turn(degrees: Double, speedDps: Double)
    /** World x, z (m) and heading (rad) of the robot, or null while not tracking. */
    fun pose(): DoubleArray?
    /** Sets a goal; [drive] = switch to Auto and follow it. */
    fun goTo(x: Double, z: Double, label: String, drive: Boolean)
    /** Goal relative to the robot: [forward] m ahead, [left] m to the left. */
    fun goToRelative(forward: Double, left: Double, drive: Boolean)
    fun setMode(mode: String)
    fun mode(): String
    fun clearGoal()
    /** A move / turn / path follow is in progress. */
    fun busy(): Boolean
    fun statusJSON(): JSONObject
    fun spokenStatus(): String
    fun labels(): List<String> = emptyList()
    /** Silences the guidance voice while the microphone listens. */
    fun quietGuidance() {}
}

/**
 * Voice control in the style of Home Assistant's Assist: speech to text -> built-in sentence matcher (or an
 * LLM with tool calls) -> nav actions -> spoken reply. "Stop" stops the robot from the live transcript
 * before the sentence ends, without any model. Motion goes through the nav screen's own commands, so its
 * obstacle stop and speed limits apply. Same behaviour as the iPhone app (ios/R2SCapture/Voice).
 */
class VoiceAssistant(private val act: Activity, private val nav: NavActions, private val log: (String) -> Unit) {
    enum class Phase { IDLE, LISTENING, THINKING, ACTING, SPEAKING }

    var settings = VoiceSettings.load(act); private set
    var phase = Phase.IDLE; private set(v) { field = v; onPhase?.invoke(v) }
    var onPhase: ((Phase) -> Unit)? = null
    var onLine: ((String, Boolean) -> Unit)? = null
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Main)
    private var stt: SpeechToText? = null
    private var tts: TextToSpeechOut? = null
    private var agent: OpenAIChatAgent? = null
    private var runner: Job? = null
    private var stopped = false
    private val places = LinkedHashMap<String, DoubleArray>()

    init { rebuild() }

    fun update(s: VoiceSettings) { settings = s; s.save(act); rebuild() }

    private fun rebuild() {
        stt?.cancel(); tts?.stop(); tts?.shutdown()
        stt = if (settings.stt == "device") DeviceSTT(act, settings) else NetworkSTT(settings)
        tts = when (settings.tts) {
            "device" -> DeviceTTS(act, settings)
            "wyoming", "openai" -> NetworkTTS(settings) { e -> line(e, false) }
            else -> null
        }
        agent = if (settings.llm == "openai") OpenAIChatAgent(settings) else null
    }

    fun shutdown() { cancel(); tts?.shutdown(); scope.cancel() }

    /** Mic button: listen, or end the utterance now. */
    fun toggle() {
        when (phase) {
            Phase.LISTENING -> stt?.finish()
            Phase.THINKING -> {}
            Phase.SPEAKING -> { tts?.stop(); listen() }
            else -> listen()
        }
    }

    fun listen() {
        if (settings.privateMode) settings.nonLocalEndpoints().firstOrNull()?.let { return fail("Private mode: $it is not on your local network") }
        if (act.checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            act.requestPermissions(arrayOf(Manifest.permission.RECORD_AUDIO), 77)
            return fail("Allow the microphone, then tap the mic again")
        }
        nav.quietGuidance()
        tts?.stop()
        stopped = false
        phase = Phase.LISTENING
        stt?.start({ text ->
            onLine?.invoke("… $text", true)
            if (!stopped && IntentParser.containsStop(text)) { stopped = true; emergencyStop() }
        }) { r ->
            r.onSuccess { if (it.isEmpty()) phase = Phase.IDLE else handle(it) }
                .onFailure { fail(it.message ?: "speech recognition failed") }
        }
    }

    fun cancel() { stt?.cancel(); tts?.stop(); runner?.cancel(); phase = Phase.IDLE }

    private fun emergencyStop() { runner?.cancel(); nav.stopAll(); line("Stopped.", false) }

    private fun allLabels() = nav.labels() + places.keys

    fun handle(text: String) {
        line(text, true)
        if (IntentParser.containsStop(text)) { if (!stopped) emergencyStop(); say("Stopped."); return }
        val labels = allLabels()
        if (settings.llm == "rules" || settings.preferLocal) {
            IntentParser.parse(text, labels)?.let { run(it, null); return }
        }
        when (settings.llm) {
            "rules" -> say("Sorry, I only know commands like go forward one meter, turn left, go to the kitchen, or stop.")
            "device" -> {
                phase = Phase.THINKING
                scope.launch {
                    val why = GeminiNano.status()
                    if (why.isNotEmpty()) {
                        // No on-device model: fall back to the built-in commands.
                        IntentParser.parse(text, labels)?.let { run(it, null) } ?: say("Sorry, I didn't understand. $why.")
                        return@launch
                    }
                    try { val r = GeminiNano.ask(text, stateSummary(), labels); run(r.actions, r.say) }
                    catch (e: Throwable) { fail("On-device model: ${e.message}") }
                }
            }
            "openai" -> {
                val a = agent ?: return
                phase = Phase.THINKING
                val state = stateSummary()
                scope.launch {
                    try {
                        val r = withContext(Dispatchers.IO) {
                            a.ask(text, state, labels) { runBlockingMain { nav.statusJSON().put("saved_places", places.keys.toList()).toString() } }
                        }
                        run(r.actions, r.say)
                    } catch (e: Throwable) { fail("LLM: ${e.message}") }
                }
            }
        }
    }

    private fun <T> runBlockingMain(f: () -> T): T {
        var out: T? = null
        val latch = java.util.concurrent.CountDownLatch(1)
        act.runOnUiThread { out = f(); latch.countDown() }
        latch.await()
        @Suppress("UNCHECKED_CAST") return out as T
    }

    private fun run(actions: List<VoiceAction>, reply: String?) {
        runner?.cancel()
        val acts = actions.map(VoiceLimits::clamp)
        if (acts.isEmpty()) { say(if (!reply.isNullOrBlank()) reply else "Sorry, I didn't understand that."); return }
        val confirm = if (!reply.isNullOrBlank()) reply else confirmation(acts)
        phase = Phase.ACTING
        runner = scope.launch {
            for ((i, a) in acts.withIndex()) {
                if (!isActive) return@launch
                val msg = execute(a)
                if (i == 0) say(msg ?: confirm) else if (msg != null) say(msg)
                if (i < acts.size - 1 && a.isMotion) waitIdle(a)
            }
            if (phase == Phase.ACTING) phase = Phase.IDLE
        }
    }

    private fun execute(a: VoiceAction): String? {
        val linked = nav.robotConnected()
        when (a) {
            VoiceAction.Stop -> nav.stopAll()
            is VoiceAction.Move -> { if (!linked) return "The robot isn't connected."; nav.move(a.meters, settings.moveSpeed) }
            is VoiceAction.Turn -> { if (!linked) return "The robot isn't connected."; nav.turn(a.degrees, settings.turnSpeed) }
            is VoiceAction.GoTo -> {
                val name = IntentParser.matchLabel(a.target, places.keys.toList())
                val p = name?.let { places[it] } ?: return run {
                    val known = allLabels().take(6).joinToString(", ")
                    if (known.isEmpty()) "I don't know where ${a.target} is. Say: remember this place as ${a.target}, or tap the map."
                    else "I don't know where ${a.target} is. I know $known."
                }
                nav.goTo(p[0], p[1], name, linked)
                return if (linked) null else "Guiding you to $name."
            }
            is VoiceAction.GoToPoint -> { if (nav.pose() == null) return "I'm not tracking yet."; nav.goToRelative(a.forward, a.left, linked) }
            is VoiceAction.SetMode -> nav.setMode(a.mode)
            VoiceAction.ClearGoal -> nav.clearGoal()
            is VoiceAction.SavePlace -> {
                val p = nav.pose() ?: return "I'm not tracking yet, so I can't remember this place."
                places[a.name] = p
                return "OK, this is ${a.name}."
            }
            VoiceAction.Status -> return nav.spokenStatus()
            is VoiceAction.Say -> return a.text
        }
        return null
    }

    private suspend fun waitIdle(a: VoiceAction) {
        val limitMs = when (a) {
            is VoiceAction.Move -> (abs(a.meters) / maxOf(0.05, settings.moveSpeed / 100) * 3 + 5) * 1000
            is VoiceAction.Turn -> (abs(a.degrees) / maxOf(5.0, settings.turnSpeed) * 3 + 5) * 1000
            else -> 180_000.0
        }
        val t0 = System.currentTimeMillis()
        delay(400)
        while (System.currentTimeMillis() - t0 < limitMs && nav.busy()) delay(200)
    }

    private fun confirmation(acts: List<VoiceAction>) = acts.mapNotNull { a ->
        when (a) {
            VoiceAction.Stop -> "Stopping"
            is VoiceAction.Move -> (if (a.meters < 0) "Backing up " else "Moving forward ") +
                if (abs(a.meters) < 1) "%.0f centimeters".format(abs(a.meters) * 100) else "%.1f meters".format(abs(a.meters))
            is VoiceAction.Turn -> "Turning %s %.0f degrees".format(if (a.degrees >= 0) "left" else "right", abs(a.degrees))
            is VoiceAction.GoTo -> "Going to ${a.target}"
            is VoiceAction.GoToPoint -> "Going there"
            is VoiceAction.SetMode -> "${a.mode.replaceFirstChar { it.uppercase() }} mode"
            VoiceAction.ClearGoal -> "Goal cleared"
            is VoiceAction.SavePlace -> "Saved ${a.name}"
            VoiceAction.Status -> null
            is VoiceAction.Say -> a.text
        }
    }.joinToString(", then ") + "."

    private fun stateSummary(): String {
        val o = nav.statusJSON()
        o.put("saved_places", places.keys.toList())
        return o.toString()
    }

    private fun say(text: String) {
        val t = text.trim()
        if (t.isEmpty()) { if (phase != Phase.ACTING) phase = Phase.IDLE; return }
        line(t, false)
        val out = tts
        if (!settings.speakReplies || out == null) { if (phase != Phase.ACTING) phase = Phase.IDLE; return }
        if (phase != Phase.ACTING) phase = Phase.SPEAKING
        out.speak(t) { if (phase == Phase.SPEAKING) phase = Phase.IDLE }
    }

    private fun line(t: String, user: Boolean) { onLine?.invoke(t, user); log(if (user) "you: $t" else "robot: $t") }

    private fun fail(msg: String) { line(msg, false); phase = Phase.IDLE }

    // ------------------------------------------------------------------ UI

    /** Round mic button for the nav screen; long-press opens the voice settings. */
    fun micButton(): Button = Button(act).apply {
        text = "🎤"; textSize = 22f; isAllCaps = false
        val bg = GradientDrawable().apply { shape = GradientDrawable.OVAL; setColor(Color.argb(200, 30, 30, 30)); setStroke(3, Color.WHITE) }
        background = bg
        setOnClickListener { toggle() }
        setOnLongClickListener { showSettings(); true }
        contentDescription = "Voice command (long-press for settings)"
        onPhase = { p ->
            act.runOnUiThread {
                bg.setColor(when (p) {
                    Phase.LISTENING -> Color.rgb(210, 40, 40)
                    Phase.THINKING -> Color.rgb(230, 140, 20)
                    Phase.SPEAKING, Phase.ACTING -> Color.rgb(40, 110, 220)
                    Phase.IDLE -> Color.argb(200, 30, 30, 30)
                })
                text = when (p) { Phase.LISTENING -> "●"; Phase.THINKING -> "…"; Phase.SPEAKING -> "🔊"; else -> "🎤" }
            }
        }
    }

    fun showSettings() {
        val s = settings.copy()
        val dp = act.resources.displayMetrics.density
        val col = LinearLayout(act).apply { orientation = LinearLayout.VERTICAL; setPadding((16 * dp).toInt(), (8 * dp).toInt(), (16 * dp).toInt(), 0) }
        fun head(t: String) = col.addView(TextView(act).apply { text = t; textSize = 15f; setTypeface(typeface, android.graphics.Typeface.BOLD); setPadding(0, (12 * dp).toInt(), 0, 0) })
        fun note(t: String) = col.addView(TextView(act).apply { text = t; textSize = 12f; setTextColor(Color.GRAY) })
        fun field(label: String, v: String, set: (String) -> Unit) = col.addView(LinearLayout(act).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            addView(TextView(act).apply { text = label; width = (110 * dp).toInt() })
            addView(EditText(act).apply { setText(v); inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI; setSingleLine()
                addTextChangedListener(object : android.text.TextWatcher {
                    override fun afterTextChanged(e: android.text.Editable?) { set(e.toString()) }
                    override fun beforeTextChanged(a: CharSequence?, b: Int, c: Int, d: Int) {}
                    override fun onTextChanged(a: CharSequence?, b: Int, c: Int, d: Int) {}
                }) }, LinearLayout.LayoutParams(0, -2, 1f))
        })
        fun pick(label: String, options: List<Pair<String, String>>, cur: String, set: (String) -> Unit) = col.addView(LinearLayout(act).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
            addView(TextView(act).apply { text = label; width = (110 * dp).toInt() })
            addView(Spinner(act).apply {
                adapter = android.widget.ArrayAdapter(act, android.R.layout.simple_spinner_dropdown_item, options.map { it.second })
                setSelection(maxOf(0, options.indexOfFirst { it.first == cur }))
                onItemSelectedListener = object : android.widget.AdapterView.OnItemSelectedListener {
                    override fun onItemSelected(p: android.widget.AdapterView<*>?, v: View?, pos: Int, id: Long) { set(options[pos].first) }
                    override fun onNothingSelected(p: android.widget.AdapterView<*>?) {}
                }
            })
        })
        fun check(label: String, v: Boolean, set: (Boolean) -> Unit) = col.addView(CheckBox(act).apply { text = label; isChecked = v; setOnCheckedChangeListener { _, c -> set(c) } })

        head("Where voice is processed")
        note("Default: Google / on-device speech, the built-in commands and Gemini Nano where the phone has it. \"My server\" uses Whisper and Piper over Wyoming (ports 10300 / 10200) and Ollama (11434) on one computer, the same servers Home Assistant's local voice uses.")
        var host = "192.168.1.10"
        field("Server IP", host) { host = it }
        col.addView(LinearLayout(act).apply {
            addView(Button(act).apply { text = "On this phone"; isAllCaps = false; setOnClickListener { update(VoiceSettings.onPhone(s)); dlg?.dismiss(); showSettings() } })
            addView(Button(act).apply { text = "Use my server"; isAllCaps = false; setOnClickListener { update(VoiceSettings.localServer(s, host.trim())); dlg?.dismiss(); showSettings() } })
        })
        check("Fully private (on-device speech, LAN servers only)", s.privateMode) { s.privateMode = it }
        if (s.privateMode && s.nonLocalEndpoints().isNotEmpty()) note("Not on your local network: ${s.nonLocalEndpoints().joinToString()}")

        head("Speech to text")
        pick("Engine", listOf("device" to "Phone (Google / on-device)", "wyoming" to "Wyoming (Whisper)", "openai" to "OpenAI-compatible server"), s.stt) { s.stt = it }
        field("Language", s.language) { s.language = it }
        check("Prefer on-device recognition", s.onDeviceSTT) { s.onDeviceSTT = it }
        field("Wyoming STT", s.wyomingSTT) { s.wyomingSTT = it }
        field("STT URL", s.sttURL) { s.sttURL = it }
        field("STT model", s.sttModel) { s.sttModel = it }

        head("Understanding")
        pick("Engine", listOf("rules" to "Built-in commands only", "device" to "Gemini Nano (on-device)", "openai" to "Ollama / OpenAI-compatible"), s.llm) { s.llm = it }
        check("Built-in commands first", s.preferLocal) { s.preferLocal = it }
        field("LLM URL", s.llmURL) { s.llmURL = it }
        field("Model", s.llmModel) { s.llmModel = it }
        check("Tool calling (off = JSON replies)", s.llmTools) { s.llmTools = it }
        field("API key", s.apiKey) { s.apiKey = it }
        note("Built-in: stop, go forward 1 meter, back up 30 centimeters, turn left 45 degrees, turn around, remember this place as the kitchen, go to the kitchen, where are you, auto mode, cancel; join steps with \"then\". \"Stop\" always stops at once, without the model.")

        head("Voice reply")
        pick("Engine", listOf("device" to "Phone voice", "wyoming" to "Wyoming (Piper)", "openai" to "OpenAI-compatible server", "off" to "Off (text only)"), s.tts) { s.tts = it }
        field("Wyoming TTS", s.wyomingTTS) { s.wyomingTTS = it }
        field("Piper voice", s.piperVoice) { s.piperVoice = it }
        field("TTS URL", s.ttsURL) { s.ttsURL = it }
        field("TTS model", s.ttsModel) { s.ttsModel = it }
        field("TTS voice", s.ttsVoice) { s.ttsVoice = it }
        check("Speak replies", s.speakReplies) { s.speakReplies = it }

        head("Voice-driven motion")
        field("Move cm/s", "%.0f".format(s.moveSpeed)) { it.toDoubleOrNull()?.let { v -> s.moveSpeed = v.coerceIn(5.0, 60.0) } }
        field("Turn deg/s", "%.0f".format(s.turnSpeed)) { it.toDoubleOrNull()?.let { v -> s.turnSpeed = v.coerceIn(10.0, 120.0) } }

        head("Try a command")
        var typed = ""
        field("Command", "") { typed = it }

        dlg = AlertDialog.Builder(act).setTitle("Voice control")
            .setView(ScrollView(act).apply { addView(col) })
            .setPositiveButton("Save") { _, _ -> update(s) }
            .setNeutralButton("Run command") { _, _ -> update(s); if (typed.isNotBlank()) handle(typed) }
            .setNegativeButton("Cancel", null)
            .show()
    }
    private var dlg: AlertDialog? = null
}
