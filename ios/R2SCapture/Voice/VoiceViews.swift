import AVFoundation
import SwiftUI

/// Mic button and the last lines of the conversation, shown above the nav screen's bottom panel.
/// Tap the mic to talk (tap again to end early); the gear opens voice settings.
struct VoiceOverlay: View {
    @ObservedObject var voice: VoiceAssistant
    @State private var showSettings = false

    var body: some View {
        HStack(alignment: .bottom, spacing: 8) {
            VStack(alignment: .leading, spacing: 3) {
                ForEach(voice.lines.suffix(3)) { l in
                    Text(l.text)
                        .font(.caption)
                        .foregroundStyle(l.user ? Color.cyan : Color.white)
                        .lineLimit(2)
                }
                if !voice.partial.isEmpty {
                    Text(voice.partial).font(.caption.italic()).foregroundStyle(.yellow).lineLimit(2)
                }
            }
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(voice.lines.isEmpty && voice.partial.isEmpty ? 0 : 6)
            .background(voice.lines.isEmpty && voice.partial.isEmpty ? Color.clear : Color.black.opacity(0.45),
                        in: RoundedRectangle(cornerRadius: 8))
            .onTapGesture { voice.cancel() }
            VStack(spacing: 6) {
                Button { showSettings = true } label: {
                    Image(systemName: "gearshape").font(.caption).foregroundStyle(.white)
                        .frame(width: 30, height: 30)
                        .background(Color.black.opacity(0.45), in: Circle())
                }
                Button { voice.toggle() } label: {
                    ZStack {
                        Circle().fill(color).frame(width: 58, height: 58)
                        if voice.phase == .thinking {
                            ProgressView().tint(.white)
                        } else {
                            Image(systemName: icon).font(.title2.weight(.semibold)).foregroundStyle(.white)
                        }
                    }
                    .overlay(Circle().stroke(Color.white.opacity(0.7), lineWidth: voice.phase == .listening ? 3 : 1))
                }
                .accessibilityLabel("Voice command")
            }
        }
        .padding(.horizontal, 10)
        .sheet(isPresented: $showSettings) { VoiceSettingsView(voice: voice) }
    }

    private var icon: String {
        switch voice.phase {
        case .listening: return "waveform"
        case .speaking: return "speaker.wave.2.fill"
        case .acting: return "figure.walk.motion"
        default: return "mic.fill"
        }
    }

    private var color: Color {
        switch voice.phase {
        case .listening: return .red
        case .thinking: return .orange
        case .speaking, .acting: return .blue
        case .idle: return Color.black.opacity(0.6)
        }
    }
}

struct VoiceSettingsView: View {
    @ObservedObject var voice: VoiceAssistant
    @Environment(\.dismiss) private var dismiss
    @State private var serverHost = "192.168.1.10"
    @State private var apiKey = VoiceKeychain.apiKey
    @State private var typed = ""

    var body: some View {
        NavigationStack {
            Form {
                Section {
                    Text(voice.privacyNote).font(.footnote)
                    Button("Everything on this iPhone") { voice.settings = VoiceSettings.devicePreset }
                    HStack {
                        TextField("server IP", text: $serverHost)
                            .keyboardType(.numbersAndPunctuation).autocorrectionDisabled().textInputAutocapitalization(.never)
                        Button("Use my server") { voice.settings = VoiceSettings.localServerPreset(host: serverHost) }
                    }
                    Toggle("Fully private", isOn: $voice.settings.privateMode)
                    if voice.settings.privateMode && !voice.settings.nonLocalEndpoints.isEmpty {
                        Text("Not on your local network: \(voice.settings.nonLocalEndpoints.joined(separator: ", "))")
                            .font(.footnote).foregroundStyle(.red)
                    }
                } header: { Text("Where voice is processed") } footer: {
                    Text("\"Use my server\" sets Whisper (Wyoming, port 10300), Piper (Wyoming, port 10200) and Ollama (port 11434) on that computer, the same servers Home Assistant's local voice uses. Fully private forces on-device Apple Speech and refuses servers outside your network or Tailscale.")
                }

                Section("Speech to text") {
                    Picker("Engine", selection: $voice.settings.stt) {
                        ForEach(VoiceSTT.allCases) { Text($0.title).tag($0) }
                    }
                    TextField("Language (en-US)", text: $voice.settings.language)
                        .autocorrectionDisabled().textInputAutocapitalization(.never)
                    switch voice.settings.stt {
                    case .device:
                        Toggle("On-device only", isOn: $voice.settings.onDeviceSTT)
                    case .wyoming:
                        field("Wyoming host:port", $voice.settings.wyomingSTT)
                    case .openai:
                        field("Server URL", $voice.settings.sttURL)
                        field("Model", $voice.settings.sttModel)
                    }
                }

                Section {
                    Picker("Engine", selection: $voice.settings.llm) {
                        ForEach(VoiceLLM.allCases) { Text($0.title).tag($0) }
                    }
                    Toggle("Built-in commands first", isOn: $voice.settings.preferLocal)
                    switch voice.settings.llm {
                    case .rules:
                        EmptyView()
                    case .device:
                        Text(AppleLLM.isAvailable ? "Apple on-device model is ready." : AppleLLM.unavailableReason)
                            .font(.footnote).foregroundStyle(AppleLLM.isAvailable ? Color.green : Color.orange)
                    case .openai:
                        field("Server URL (Ollama: http://IP:11434)", $voice.settings.llmURL)
                        field("Model", $voice.settings.llmModel)
                        Toggle("Tool calling", isOn: $voice.settings.llmTools)
                        SecureField("API key (hosted servers only)", text: $apiKey)
                            .onSubmit { VoiceKeychain.apiKey = apiKey }
                    }
                } header: { Text("Understanding") } footer: {
                    Text("Built-in commands: stop, go forward 1 meter, back up 30 centimeters, turn left 45 degrees, turn around, go to the table, where are you, auto mode, cancel; join steps with \"then\". Everything else goes to the model, which drives through the same commands. \"Stop\" always stops at once, without the model.")
                }

                Section("Voice reply") {
                    Picker("Engine", selection: $voice.settings.tts) {
                        ForEach(VoiceTTS.allCases) { Text($0.title).tag($0) }
                    }
                    switch voice.settings.tts {
                    case .device:
                        Picker("Voice", selection: $voice.settings.deviceVoice) {
                            Text("Default").tag("")
                            ForEach(deviceVoices, id: \.identifier) { v in
                                Text("\(v.name) (\(v.language))").tag(v.identifier)
                            }
                        }
                        Slider(value: $voice.settings.speechRate, in: 0.35...0.65) { Text("Rate") }
                    case .wyoming:
                        field("Wyoming host:port", $voice.settings.wyomingTTS)
                        field("Piper voice (optional)", $voice.settings.piperVoice)
                    case .openai:
                        field("Server URL", $voice.settings.ttsURL)
                        field("Model", $voice.settings.ttsModel)
                        field("Voice", $voice.settings.ttsVoice)
                    case .off:
                        EmptyView()
                    }
                    Toggle("Speak replies", isOn: $voice.settings.speakReplies)
                }

                Section("Voice-driven motion") {
                    Stepper(String(format: "Move speed %.0f cm/s", voice.settings.moveSpeed), value: $voice.settings.moveSpeed, in: 5...60, step: 5)
                    Stepper(String(format: "Turn speed %.0f°/s", voice.settings.turnSpeed), value: $voice.settings.turnSpeed, in: 10...120, step: 5)
                }

                Section("Try a command") {
                    HStack {
                        TextField("e.g. turn left then go forward 1 meter", text: $typed)
                        Button("Run") { voice.handle(typed); typed = "" }.disabled(typed.isEmpty)
                    }
                    if !voice.lastError.isEmpty { Text(voice.lastError).font(.footnote).foregroundStyle(.red) }
                }
            }
            .navigationTitle("Voice control")
            .toolbar { Button("Done") { VoiceKeychain.apiKey = apiKey; dismiss() } }
        }
    }

    private var deviceVoices: [AVSpeechSynthesisVoice] {
        let lang = String(voice.settings.language.prefix(2))
        return AVSpeechSynthesisVoice.speechVoices().filter { $0.language.hasPrefix(lang) }.sorted { $0.name < $1.name }
    }

    private func field(_ title: String, _ value: Binding<String>) -> some View {
        TextField(title, text: value)
            .keyboardType(.URL).autocorrectionDisabled().textInputAutocapitalization(.never)
    }
}
