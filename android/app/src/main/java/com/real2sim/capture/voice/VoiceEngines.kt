package com.real2sim.capture.voice

import android.annotation.SuppressLint
import android.content.Context
import android.content.Intent
import android.media.AudioAttributes
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.AudioTrack
import android.media.MediaRecorder
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.speech.RecognitionListener
import android.speech.RecognizerIntent
import android.speech.SpeechRecognizer
import android.speech.tts.TextToSpeech
import android.speech.tts.UtteranceProgressListener
import com.google.mlkit.genai.common.FeatureStatus
import com.google.mlkit.genai.prompt.Generation
import com.google.mlkit.genai.prompt.GenerativeModel
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.MultipartBody
import okhttp3.OkHttpClient
import okhttp3.Request
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.InputStream
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.util.Locale
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread

// Speech to text, text to speech and LLM back ends: the phone's own (Google / on-device) by default, or
// fully local servers on your network (Wyoming Whisper / Piper, Ollama or any OpenAI-compatible server).

private val main = Handler(Looper.getMainLooper())
internal val http: OkHttpClient = OkHttpClient.Builder().connectTimeout(5, TimeUnit.SECONDS).readTimeout(60, TimeUnit.SECONDS).build()

interface SpeechToText {
    /** One utterance; callbacks on the main thread. [done] gets "" when nothing was heard. */
    fun start(partial: (String) -> Unit, done: (Result<String>) -> Unit)
    fun finish()
    fun cancel()
}

/** Android SpeechRecognizer: Google's recogniser, or the on-device one (Android 12+) in private mode. */
class DeviceSTT(private val ctx: Context, private val s: VoiceSettings) : SpeechToText {
    private var rec: SpeechRecognizer? = null
    private var last = ""
    private var doneCb: ((Result<String>) -> Unit)? = null

    override fun start(partial: (String) -> Unit, done: (Result<String>) -> Unit) {
        doneCb = done
        last = ""
        val onDevice = (s.onDeviceSTT || s.privateMode) && Build.VERSION.SDK_INT >= 31 && SpeechRecognizer.isOnDeviceRecognitionAvailable(ctx)
        if (s.privateMode && !onDevice) return complete(Result.failure(Exception("This phone has no on-device speech recogniser; private mode needs it (or use a Wyoming server)")))
        if (!onDevice && !SpeechRecognizer.isRecognitionAvailable(ctx)) return complete(Result.failure(Exception("No speech recogniser on this phone (install Google app / Speech Services)")))
        val r = if (onDevice && Build.VERSION.SDK_INT >= 31) SpeechRecognizer.createOnDeviceSpeechRecognizer(ctx) else SpeechRecognizer.createSpeechRecognizer(ctx)
        rec = r
        r.setRecognitionListener(object : RecognitionListener {
            override fun onReadyForSpeech(params: Bundle?) {}
            override fun onBeginningOfSpeech() {}
            override fun onRmsChanged(rmsdB: Float) {}
            override fun onBufferReceived(buffer: ByteArray?) {}
            override fun onEndOfSpeech() {}
            override fun onEvent(eventType: Int, params: Bundle?) {}
            override fun onPartialResults(b: Bundle?) {
                b?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)?.firstOrNull()?.let { if (it.isNotBlank()) { last = it; partial(it) } }
            }
            override fun onResults(b: Bundle?) {
                complete(Result.success(b?.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION)?.firstOrNull() ?: last))
            }
            override fun onError(error: Int) {
                when (error) {
                    SpeechRecognizer.ERROR_NO_MATCH, SpeechRecognizer.ERROR_SPEECH_TIMEOUT -> complete(Result.success(last))
                    SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS -> complete(Result.failure(Exception("Microphone permission is off")))
                    else -> complete(if (last.isNotEmpty()) Result.success(last) else Result.failure(Exception("Speech recogniser error $error")))
                }
            }
        })
        val i = Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH)
            .putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
            .putExtra(RecognizerIntent.EXTRA_LANGUAGE, s.language)
            .putExtra(RecognizerIntent.EXTRA_PARTIAL_RESULTS, true)
            .putExtra(RecognizerIntent.EXTRA_PREFER_OFFLINE, s.onDeviceSTT || s.privateMode)
            .putExtra(RecognizerIntent.EXTRA_SPEECH_INPUT_COMPLETE_SILENCE_LENGTH_MILLIS, 1000L)
        r.startListening(i)
    }

    override fun finish() { rec?.stopListening() }
    override fun cancel() { doneCb = null; rec?.cancel(); rec?.destroy(); rec = null }

    private fun complete(r: Result<String>) {
        val cb = doneCb ?: return
        doneCb = null
        main.post { rec?.destroy(); rec = null; cb(r.map { it.trim() }) }
    }
}

/** Minimal blocking Wyoming client over TCP. */
class WyomingClient(host: String, port: Int) {
    private val sock = Socket().apply { connect(InetSocketAddress(host, port), 5000); soTimeout = 30000 }
    private val out: OutputStream = sock.getOutputStream()
    private val input: InputStream = sock.getInputStream()
    private val dec = WyomingDecoder()
    private val queue = ArrayDeque<WyomingEvent>()

    @Synchronized fun send(e: WyomingEvent) { out.write(e.encode()); out.flush() }

    fun next(): WyomingEvent {
        while (queue.isEmpty()) {
            val buf = ByteArray(65536)
            val n = input.read(buf)
            if (n < 0) throw Exception("Wyoming server closed the connection")
            queue.addAll(dec.feed(buf.copyOf(n)))
        }
        return queue.removeFirst()
    }

    fun close() = try { sock.close() } catch (_: Exception) {}
}

/** Microphone at 16 kHz, end of speech by [EnergyVAD]; streamed to Wyoming or uploaded as one WAV. */
class NetworkSTT(private val s: VoiceSettings) : SpeechToText {
    @Volatile private var stopNow = false
    @Volatile private var cancelled = false

    @SuppressLint("MissingPermission")
    override fun start(partial: (String) -> Unit, done: (Result<String>) -> Unit) {
        stopNow = false; cancelled = false
        thread(name = "voice-stt") {
            val res = runCatching { listen(partial) }
            if (!cancelled) main.post { done(res.map { it.trim() }) }
        }
    }

    @SuppressLint("MissingPermission")
    private fun listen(partial: (String) -> Unit): String {
        val rate = 16000
        var w: WyomingClient? = null
        if (s.stt == "wyoming") {
            val (h, p) = VoiceURL.hostPort(s.wyomingSTT, 10300) ?: throw Exception("Bad Wyoming STT address")
            w = try { WyomingClient(h, p) } catch (e: Exception) { throw Exception("Wyoming STT $h:$p: ${e.message}") }
            w.send(WyomingEvent("transcribe", JSONObject().put("language", s.language.take(2))))
            w.send(WyomingEvent("audio-start", WyomingEvent.audioFormat(rate)))
        }
        val minBuf = AudioRecord.getMinBufferSize(rate, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT)
        val rec = AudioRecord(MediaRecorder.AudioSource.VOICE_RECOGNITION, rate, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, maxOf(minBuf, 16000))
        if (rec.state != AudioRecord.STATE_INITIALIZED) { w?.close(); throw Exception("Microphone unavailable") }
        val vad = EnergyVAD(rate)
        val pcm = ByteArrayOutputStream()
        val chunk = ShortArray(480)
        var heard = false
        main.post { partial("…") }
        rec.startRecording()
        try {
            while (!stopNow && !cancelled) {
                val n = rec.read(chunk, 0, chunk.size)
                if (n <= 0) continue
                val bytes = ByteArray(n * 2)
                for (i in 0 until n) { bytes[2 * i] = (chunk[i].toInt() and 0xff).toByte(); bytes[2 * i + 1] = (chunk[i].toInt() shr 8).toByte() }
                if (w != null) w.send(WyomingEvent("audio-chunk", WyomingEvent.audioFormat(rate), bytes)) else pcm.write(bytes)
                when (vad.feed(chunk, n)) {
                    EnergyVAD.Result.DONE -> { heard = true; break }
                    EnergyVAD.Result.TIMEOUT -> break
                    EnergyVAD.Result.SPEAKING -> heard = true
                    else -> {}
                }
            }
        } finally { rec.stop(); rec.release() }
        if (cancelled || !heard) { w?.close(); return "" }
        if (w != null) {
            try {
                w.send(WyomingEvent("audio-stop"))
                while (true) {
                    val e = w.next()
                    if (e.type == "transcript") return e.data.optString("text")
                    if (e.type == "error") throw Exception(e.data.optString("text", "Wyoming error"))
                }
            } finally { w.close() }
        }
        return OpenAIHttp.transcribe(WAV.encode(pcm.toByteArray(), rate), s)
    }

    override fun finish() { stopNow = true }
    override fun cancel() { cancelled = true }
}

interface TextToSpeechOut {
    fun speak(text: String, done: () -> Unit)
    fun stop()
    fun shutdown() {}
}

class DeviceTTS(ctx: Context, private val s: VoiceSettings) : TextToSpeechOut {
    @Volatile private var ready = false
    private val pending = ArrayList<Pair<String, () -> Unit>>()
    private val callbacks = HashMap<String, () -> Unit>()
    private var n = 0
    private val tts: TextToSpeech = TextToSpeech(ctx.applicationContext) { st ->
        main.post {
            ready = st == TextToSpeech.SUCCESS
            if (ready) {
                tts.language = Locale.forLanguageTag(s.language)
                tts.setSpeechRate(s.speechRate.toFloat())
                pending.forEach { (t, d) -> speak(t, d) }
            } else pending.forEach { it.second() }
            pending.clear()
        }
    }

    init {
        tts.setOnUtteranceProgressListener(object : UtteranceProgressListener() {
            override fun onStart(id: String?) {}
            override fun onDone(id: String?) { fire(id) }
            @Deprecated("Deprecated in Java") override fun onError(id: String?) { fire(id) }
            override fun onStop(id: String?, interrupted: Boolean) { fire(id) }
        })
    }

    private fun fire(id: String?) = main.post { id?.let { callbacks.remove(it)?.invoke() } }

    override fun speak(text: String, done: () -> Unit) {
        if (!ready) { pending.add(text to done); return }
        val id = "r2s-${n++}"
        callbacks[id] = done
        tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, id)
    }

    override fun stop() { tts.stop(); val cbs = callbacks.values.toList(); callbacks.clear(); cbs.forEach { it() } }
    override fun shutdown() { tts.shutdown() }
}

/** Piper over Wyoming, or OpenAI-compatible /v1/audio/speech; played with AudioTrack. */
class NetworkTTS(private val s: VoiceSettings, private val onError: (String) -> Unit) : TextToSpeechOut {
    @Volatile private var track: AudioTrack? = null
    @Volatile private var gen = 0

    override fun speak(text: String, done: () -> Unit) {
        val my = ++gen
        thread(name = "voice-tts") {
            try {
                val (rate, ch, pcm) = if (s.tts == "wyoming") wyoming(text) else
                    WAV.decode(OpenAIHttp.speech(text, s)) ?: throw Exception("server did not return 16-bit WAV")
                if (my != gen) return@thread
                play(rate, ch, pcm)
            } catch (e: Exception) { main.post { onError("TTS: ${e.message}") } }
            main.post(done)
        }
    }

    private fun wyoming(text: String): Triple<Int, Int, ByteArray> {
        val (h, p) = VoiceURL.hostPort(s.wyomingTTS, 10200) ?: throw Exception("Bad Wyoming TTS address")
        val w = WyomingClient(h, p)
        try {
            val d = JSONObject().put("text", text)
            if (s.piperVoice.isNotBlank()) d.put("voice", JSONObject().put("name", s.piperVoice))
            w.send(WyomingEvent("synthesize", d))
            val pcm = ByteArrayOutputStream()
            var rate = 22050; var ch = 1
            while (true) {
                val e = w.next()
                when (e.type) {
                    "audio-start" -> { rate = e.data.optInt("rate", rate); ch = e.data.optInt("channels", ch) }
                    "audio-chunk" -> { rate = e.data.optInt("rate", rate); pcm.write(e.payload) }
                    "audio-stop" -> return Triple(rate, ch, pcm.toByteArray())
                    "error" -> throw Exception(e.data.optString("text", "Wyoming TTS error"))
                }
            }
        } finally { w.close() }
    }

    private fun play(rate: Int, ch: Int, pcm: ByteArray) {
        if (pcm.isEmpty()) return
        val t = AudioTrack.Builder()
            .setAudioAttributes(AudioAttributes.Builder().setUsage(AudioAttributes.USAGE_ASSISTANT).setContentType(AudioAttributes.CONTENT_TYPE_SPEECH).build())
            .setAudioFormat(AudioFormat.Builder().setSampleRate(rate).setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                .setChannelMask(if (ch == 2) AudioFormat.CHANNEL_OUT_STEREO else AudioFormat.CHANNEL_OUT_MONO).build())
            .setTransferMode(AudioTrack.MODE_STATIC).setBufferSizeInBytes(pcm.size).build()
        track = t
        t.write(pcm, 0, pcm.size)
        t.play()
        val ms = pcm.size * 1000L / (rate * 2 * ch)
        val end = System.currentTimeMillis() + ms + 150
        while (System.currentTimeMillis() < end && track === t && t.playState == AudioTrack.PLAYSTATE_PLAYING) Thread.sleep(30)
        t.release()
        if (track === t) track = null
    }

    override fun stop() { gen++; try { track?.stop() } catch (_: Exception) {} }
}

object OpenAIHttp {
    private fun Request.Builder.auth(s: VoiceSettings) = apply { if (s.apiKey.isNotBlank()) header("Authorization", "Bearer ${s.apiKey}") }

    fun transcribe(wav: ByteArray, s: VoiceSettings): String {
        val body = MultipartBody.Builder().setType(MultipartBody.FORM)
            .addFormDataPart("model", s.sttModel).addFormDataPart("language", s.language.take(2)).addFormDataPart("response_format", "json")
            .addFormDataPart("file", "speech.wav", wav.toRequestBody("audio/wav".toMediaType())).build()
        val r = Request.Builder().url(VoiceURL.openAIBase(s.sttURL) + "/audio/transcriptions").auth(s).post(body).build()
        http.newCall(r).execute().use { resp ->
            val txt = resp.body?.string() ?: ""
            if (!resp.isSuccessful) throw Exception("HTTP ${resp.code}: ${txt.take(200)}")
            return try { JSONObject(txt).optString("text") } catch (_: Exception) { txt }
        }
    }

    fun speech(text: String, s: VoiceSettings): ByteArray {
        val j = JSONObject().put("model", s.ttsModel).put("input", text).put("voice", s.ttsVoice).put("response_format", "wav")
        val r = Request.Builder().url(VoiceURL.openAIBase(s.ttsURL) + "/audio/speech").auth(s)
            .post(j.toString().toRequestBody("application/json".toMediaType())).build()
        http.newCall(r).execute().use { resp ->
            if (!resp.isSuccessful) throw Exception("HTTP ${resp.code}: ${resp.body?.string()?.take(200)}")
            return resp.body?.bytes() ?: ByteArray(0)
        }
    }

    fun chat(body: JSONObject, s: VoiceSettings): String {
        val r = Request.Builder().url(VoiceURL.openAIBase(s.llmURL) + "/chat/completions").auth(s)
            .post(body.toString().toRequestBody("application/json".toMediaType())).build()
        http.newCall(r).execute().use { resp ->
            val txt = resp.body?.string() ?: ""
            if (!resp.isSuccessful) throw Exception("HTTP ${resp.code}: ${txt.take(200)}")
            return txt
        }
    }
}

data class LLMReply(val actions: List<VoiceAction>, val say: String)

/** OpenAI-compatible chat with tool calls (Ollama /v1, llama.cpp, LM Studio, vLLM, LocalAI, hosted). */
class OpenAIChatAgent(private val s: VoiceSettings) {
    private val history = ArrayList<JSONObject>()

    /** Blocking; call off the main thread. [status] answers get_status inside the loop. */
    fun ask(text: String, state: String, labels: List<String>, status: () -> String): LLMReply {
        val messages = JSONArray()
        messages.put(JSONObject().put("role", "system").put("content", VoiceTools.systemPrompt(state, labels, !s.llmTools)))
        history.takeLast(8).forEach { messages.put(it) }
        messages.put(JSONObject().put("role", "user").put("content", text))
        val actions = ArrayList<VoiceAction>()
        var say = ""
        for (round in 0 until 4) {
            val body = JSONObject().put("model", s.llmModel).put("messages", messages).put("temperature", 0.2).put("stream", false)
            if (s.llmTools) body.put("tools", VoiceTools.openAITools()).put("tool_choice", "auto")
            val raw = OpenAIHttp.chat(body, s)
            val (content, calls) = VoiceTools.parseChatResponse(raw) ?: throw Exception("Unexpected reply: ${raw.take(200)}")
            if (calls.isEmpty()) {
                val j = VoiceTools.parseJSONReply(content)
                if (j != null && (!s.llmTools || j.first.isNotEmpty())) { actions += j.first; say = j.second } else say = content
                break
            }
            val tc = JSONArray()
            calls.forEach { tc.put(JSONObject().put("id", it.id).put("type", "function").put("function", JSONObject().put("name", it.name).put("arguments", it.arguments))) }
            messages.put(JSONObject().put("role", "assistant").put("content", content).put("tool_calls", tc))
            var needAnswer = false
            for (c in calls) {
                val a = VoiceTools.action(c.name, c.arguments)
                val result = when {
                    a == null -> { needAnswer = true; "{\"ok\":false,\"error\":\"unknown tool or bad arguments\"}" }
                    a == VoiceAction.Status -> { needAnswer = true; status() }
                    else -> { actions += a; JSONObject().put("ok", true).put("started", a.summary).toString() }
                }
                messages.put(JSONObject().put("role", "tool").put("tool_call_id", c.id).put("name", c.name).put("content", result))
            }
            say = content
            if (!needAnswer && content.isNotEmpty()) break
        }
        history += JSONObject().put("role", "user").put("content", text)
        history += JSONObject().put("role", "assistant").put("content", say.ifEmpty { actions.joinToString(", ") { it.summary } })
        return LLMReply(actions, say)
    }
}

/** Gemini Nano through ML Kit GenAI Prompt API (Pixel 9+/10, Galaxy S25 and other AICore phones). */
object GeminiNano {
    private var model: GenerativeModel? = null

    private fun client(): GenerativeModel = model ?: Generation.getClient().also { model = it }

    /** "" when ready, otherwise why not. Starts the model download when the phone offers it. */
    suspend fun status(): String = withContext(Dispatchers.IO) {
        try {
            when (client().checkStatus()) {
                FeatureStatus.AVAILABLE -> ""
                FeatureStatus.DOWNLOADABLE -> { startDownload(); "Gemini Nano is downloading; try again in a few minutes" }
                FeatureStatus.DOWNLOADING -> "Gemini Nano is still downloading"
                else -> "Gemini Nano is not available on this phone"
            }
        } catch (e: Throwable) { "Gemini Nano unavailable (${e.message ?: e.javaClass.simpleName})" }
    }

    private fun startDownload() {
        thread(name = "nano-download") {
            try { kotlinx.coroutines.runBlocking { client().download().collect { } } } catch (_: Throwable) {}
        }
    }

    suspend fun ask(text: String, state: String, labels: List<String>): LLMReply = withContext(Dispatchers.IO) {
        val prompt = VoiceTools.systemPrompt(state, labels, jsonOnly = true) + "\n\nUser: " + text
        val r = client().generateContent(prompt)
        val out = r.candidates.firstOrNull()?.text ?: ""
        VoiceTools.parseJSONReply(out)?.let { LLMReply(it.first, it.second) } ?: LLMReply(emptyList(), out)
    }
}
