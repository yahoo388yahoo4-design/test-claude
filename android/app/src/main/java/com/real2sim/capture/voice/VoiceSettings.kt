package com.real2sim.capture.voice

import android.content.Context
import org.json.JSONObject

/**
 * Voice control settings (same options as the iPhone app). Engines:
 *  - stt: "device" (Android SpeechRecognizer: Google or the phone's on-device recogniser), "wyoming"
 *    (wyoming-faster-whisper / Home Assistant's Whisper add-on, port 10300), "openai" (OpenAI-compatible
 *    /v1/audio/transcriptions: speaches / faster-whisper-server, LocalAI, whisper.cpp)
 *  - llm: "rules" (built-in commands only), "device" (Gemini Nano through ML Kit GenAI, supported phones),
 *    "openai" (Ollama / llama.cpp / LM Studio / vLLM / hosted, OpenAI-compatible chat with tool calls)
 *  - tts: "device" (Android TextToSpeech), "wyoming" (wyoming-piper, port 10200), "openai"
 *    (/v1/audio/speech: Kokoro-FastAPI, openedai-speech, LocalAI), "off"
 */
data class VoiceSettings(
    var stt: String = "device",
    var language: String = "en-US",
    var onDeviceSTT: Boolean = true,
    var wyomingSTT: String = "192.168.1.10:10300",
    var sttURL: String = "http://192.168.1.10:8000",
    var sttModel: String = "Systran/faster-whisper-small",
    var tts: String = "device",
    var speechRate: Double = 1.0,
    var wyomingTTS: String = "192.168.1.10:10200",
    var piperVoice: String = "",
    var ttsURL: String = "http://192.168.1.10:8880",
    var ttsModel: String = "kokoro",
    var ttsVoice: String = "af_heart",
    var llm: String = "device",
    var preferLocal: Boolean = true,
    var llmURL: String = "http://192.168.1.10:11434",
    var llmModel: String = "qwen2.5:7b-instruct",
    var llmTools: Boolean = true,
    var apiKey: String = "",
    var privateMode: Boolean = false,
    var speakReplies: Boolean = true,
    var moveSpeed: Double = 20.0,
    var turnSpeed: Double = 45.0,
) {
    fun toJson(): JSONObject = JSONObject().put("stt", stt).put("language", language).put("onDeviceSTT", onDeviceSTT)
        .put("wyomingSTT", wyomingSTT).put("sttURL", sttURL).put("sttModel", sttModel).put("tts", tts).put("speechRate", speechRate)
        .put("wyomingTTS", wyomingTTS).put("piperVoice", piperVoice).put("ttsURL", ttsURL).put("ttsModel", ttsModel).put("ttsVoice", ttsVoice)
        .put("llm", llm).put("preferLocal", preferLocal).put("llmURL", llmURL).put("llmModel", llmModel).put("llmTools", llmTools)
        .put("apiKey", apiKey).put("privateMode", privateMode).put("speakReplies", speakReplies)
        .put("moveSpeed", moveSpeed).put("turnSpeed", turnSpeed)

    /** Server addresses in use that are not on the local network (refused in private mode). */
    fun nonLocalEndpoints(): List<String> = buildList {
        if (stt == "wyoming" && !VoiceURL.isLocal(wyomingSTT)) add(wyomingSTT)
        if (stt == "openai" && !VoiceURL.isLocal(sttURL)) add(sttURL)
        if (tts == "wyoming" && !VoiceURL.isLocal(wyomingTTS)) add(wyomingTTS)
        if (tts == "openai" && !VoiceURL.isLocal(ttsURL)) add(ttsURL)
        if (llm == "openai" && !VoiceURL.isLocal(llmURL)) add(llmURL)
    }

    fun save(ctx: Context) = ctx.getSharedPreferences("voice", Context.MODE_PRIVATE).edit().putString("settings", toJson().toString()).apply()

    companion object {
        fun fromJson(o: JSONObject): VoiceSettings {
            val d = VoiceSettings()
            return VoiceSettings(
                o.optString("stt", d.stt), o.optString("language", d.language), o.optBoolean("onDeviceSTT", d.onDeviceSTT),
                o.optString("wyomingSTT", d.wyomingSTT), o.optString("sttURL", d.sttURL), o.optString("sttModel", d.sttModel),
                o.optString("tts", d.tts), o.optDouble("speechRate", d.speechRate), o.optString("wyomingTTS", d.wyomingTTS),
                o.optString("piperVoice", d.piperVoice), o.optString("ttsURL", d.ttsURL), o.optString("ttsModel", d.ttsModel),
                o.optString("ttsVoice", d.ttsVoice), o.optString("llm", d.llm), o.optBoolean("preferLocal", d.preferLocal),
                o.optString("llmURL", d.llmURL), o.optString("llmModel", d.llmModel), o.optBoolean("llmTools", d.llmTools),
                o.optString("apiKey", d.apiKey), o.optBoolean("privateMode", d.privateMode), o.optBoolean("speakReplies", d.speakReplies),
                o.optDouble("moveSpeed", d.moveSpeed), o.optDouble("turnSpeed", d.turnSpeed))
        }

        fun load(ctx: Context): VoiceSettings {
            val s = ctx.getSharedPreferences("voice", Context.MODE_PRIVATE).getString("settings", null) ?: return VoiceSettings()
            return try { fromJson(JSONObject(s)) } catch (_: Exception) { VoiceSettings() }
        }

        /** Home-Assistant-style local stack on one computer: Whisper + Piper over Wyoming, Ollama. */
        fun localServer(base: VoiceSettings, host: String) = base.copy(
            stt = "wyoming", wyomingSTT = "$host:10300", tts = "wyoming", wyomingTTS = "$host:10200",
            llm = "openai", llmURL = "http://$host:11434", privateMode = true)

        fun onPhone(base: VoiceSettings) = base.copy(stt = "device", onDeviceSTT = true, tts = "device", llm = "device")
    }
}
