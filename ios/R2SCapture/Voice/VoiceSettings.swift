import Foundation
import Security

/// Speech to text.
enum VoiceSTT: String, Codable, CaseIterable, Identifiable {
    case device      // Apple Speech (on-device when the phone supports it)
    case wyoming     // Wyoming server: wyoming-faster-whisper, Home Assistant's Whisper add-on (port 10300)
    case openai      // OpenAI-compatible /v1/audio/transcriptions: faster-whisper-server / speaches, LocalAI, whisper.cpp

    var id: String { rawValue }
    var title: String {
        switch self {
        case .device: return "iPhone (Apple Speech)"
        case .wyoming: return "Wyoming (Whisper)"
        case .openai: return "OpenAI-compatible server"
        }
    }
}

/// Text to speech.
enum VoiceTTS: String, Codable, CaseIterable, Identifiable {
    case device      // AVSpeechSynthesizer
    case wyoming     // Wyoming server: wyoming-piper, Home Assistant's Piper add-on (port 10200)
    case openai      // OpenAI-compatible /v1/audio/speech: Kokoro-FastAPI, openedai-speech (Piper), LocalAI
    case off

    var id: String { rawValue }
    var title: String {
        switch self {
        case .device: return "iPhone voice"
        case .wyoming: return "Wyoming (Piper)"
        case .openai: return "OpenAI-compatible server"
        case .off: return "Off (text only)"
        }
    }
}

/// What understands the command ("conversation agent" in Home Assistant terms).
enum VoiceLLM: String, Codable, CaseIterable, Identifiable {
    case rules       // built-in sentence matcher only, no model
    case device      // Apple Intelligence on-device model (iOS 26+, Apple Intelligence phones)
    case openai      // OpenAI-compatible chat completions with tool calls: Ollama, llama.cpp, LM Studio, vLLM, …

    var id: String { rawValue }
    var title: String {
        switch self {
        case .rules: return "Built-in commands only"
        case .device: return "Apple on-device model"
        case .openai: return "Ollama / OpenAI-compatible"
        }
    }
}

/// Voice control settings, stored apart from NavSettings. Missing keys (older versions) take defaults.
struct VoiceSettings: Codable, Equatable {
    var stt: VoiceSTT = .device
    var language = "en-US"
    /// Apple Speech: never send audio to Apple (needs a phone and language with on-device recognition).
    var onDeviceSTT = true
    var wyomingSTT = "192.168.1.10:10300"
    var sttURL = "http://192.168.1.10:8000"
    var sttModel = "Systran/faster-whisper-small"

    var tts: VoiceTTS = .device
    var deviceVoice = ""                  // AVSpeechSynthesisVoice identifier, "" = default for the language
    var speechRate = 0.52
    var wyomingTTS = "192.168.1.10:10200"
    var piperVoice = ""                   // e.g. en_US-lessac-medium, "" = server default
    var ttsURL = "http://192.168.1.10:8880"
    var ttsModel = "kokoro"
    var ttsVoice = "af_heart"

    var llm: VoiceLLM = .device
    /// Try the built-in sentence matcher first; only unknown sentences go to the model.
    var preferLocal = true
    var llmURL = "http://192.168.1.10:11434"
    var llmModel = "qwen2.5:7b-instruct"
    /// Send OpenAI `tools`; off = ask for a JSON reply instead (models without tool support).
    var llmTools = true
    /// Fully private: on-device Apple Speech only, and only LAN / Tailscale addresses for servers.
    var privateMode = false
    /// Spoken and on-screen replies; motion still happens when off.
    var speakReplies = true
    /// Speed for voice "move" commands, cm/s (capped by the nav max speed).
    var moveSpeed = 20.0
    var turnSpeed = 45.0

    private static let key = "r2s.voiceSettings.v1"

    static func load() -> VoiceSettings {
        let def = VoiceSettings()
        guard let saved = UserDefaults.standard.data(forKey: key),
              let savedObj = try? JSONSerialization.jsonObject(with: saved) as? [String: Any],
              let defData = try? JSONEncoder().encode(def),
              var obj = try? JSONSerialization.jsonObject(with: defData) as? [String: Any] else { return def }
        obj.merge(savedObj) { _, new in new }
        guard let merged = try? JSONSerialization.data(withJSONObject: obj),
              let s = try? JSONDecoder().decode(VoiceSettings.self, from: merged) else { return def }
        return s
    }

    func save() {
        if let data = try? JSONEncoder().encode(self) { UserDefaults.standard.set(data, forKey: VoiceSettings.key) }
    }

    /// Everything on the phone: Apple Speech (on-device), built-in commands + Apple's on-device model, iPhone voice.
    static var devicePreset: VoiceSettings {
        var s = VoiceSettings.load()
        s.stt = .device; s.onDeviceSTT = true; s.tts = .device; s.llm = .device
        return s
    }

    /// Home-Assistant-style local voice stack on a computer on your network: Whisper and Piper over
    /// Wyoming, Ollama for the model. Nothing leaves the LAN.
    static func localServerPreset(host: String) -> VoiceSettings {
        var s = VoiceSettings.load()
        let h = host.trimmingCharacters(in: .whitespaces)
        s.stt = .wyoming; s.wyomingSTT = "\(h):10300"
        s.tts = .wyoming; s.wyomingTTS = "\(h):10200"
        s.llm = .openai; s.llmURL = "http://\(h):11434"
        s.privateMode = true
        return s
    }

    /// Server addresses in use that are not on the local network (shown as a warning in private mode).
    var nonLocalEndpoints: [String] {
        var out: [String] = []
        if stt == .wyoming && !VoiceURL.isLocal(wyomingSTT) { out.append(wyomingSTT) }
        if stt == .openai && !VoiceURL.isLocal(sttURL) { out.append(sttURL) }
        if tts == .wyoming && !VoiceURL.isLocal(wyomingTTS) { out.append(wyomingTTS) }
        if tts == .openai && !VoiceURL.isLocal(ttsURL) { out.append(ttsURL) }
        if llm == .openai && !VoiceURL.isLocal(llmURL) { out.append(llmURL) }
        return out
    }
}

/// API key for a hosted OpenAI-compatible server, kept in the keychain (not needed for Ollama).
enum VoiceKeychain {
    private static let account = "r2s.voice.llmKey"

    static var apiKey: String {
        get {
            let q: [String: Any] = [kSecClass as String: kSecClassGenericPassword, kSecAttrAccount as String: account,
                                    kSecReturnData as String: true, kSecMatchLimit as String: kSecMatchLimitOne]
            var out: CFTypeRef?
            guard SecItemCopyMatching(q as CFDictionary, &out) == errSecSuccess, let d = out as? Data else { return "" }
            return String(decoding: d, as: UTF8.self)
        }
        set {
            let q: [String: Any] = [kSecClass as String: kSecClassGenericPassword, kSecAttrAccount as String: account]
            SecItemDelete(q as CFDictionary)
            guard !newValue.isEmpty else { return }
            var add = q
            add[kSecValueData as String] = Data(newValue.utf8)
            add[kSecAttrAccessible as String] = kSecAttrAccessibleAfterFirstUnlockThisDeviceOnly
            SecItemAdd(add as CFDictionary, nil)
        }
    }
}
