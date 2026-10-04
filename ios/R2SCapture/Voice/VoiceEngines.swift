import AVFoundation
import Foundation
import Network
import Speech
#if canImport(FoundationModels)
import FoundationModels
#endif

// Speech to text, text to speech and LLM back ends for voice control. Each has a device option (Apple
// frameworks) and fully local network options (Wyoming / OpenAI-compatible servers on your LAN).

enum VoiceError: LocalizedError {
    case permission(String)
    case unavailable(String)
    case server(String)
    case timeout(String)

    var errorDescription: String? {
        switch self {
        case .permission(let s), .unavailable(let s), .server(let s): return s
        case .timeout(let s): return "\(s) timed out"
        }
    }
}

// MARK: - Audio session

enum VoiceAudio {
    /// Record and play at once, loud speaker, ducking other audio. Also fine for the nav screen's guidance
    /// voice and beeps (VoiceGuide), which keep playing.
    static func activate() {
        let s = AVAudioSession.sharedInstance()
        try? s.setCategory(.playAndRecord, mode: .default,
                           options: [.defaultToSpeaker, .duckOthers, .mixWithOthers, .allowBluetoothA2DP])
        try? s.setActive(true)
    }

    static func requestPermissions(speech: Bool) async -> String? {
        let mic: Bool = await withCheckedContinuation { c in
            AVAudioApplication.requestRecordPermission { c.resume(returning: $0) }
        }
        if !mic { return "Microphone permission is off (Settings › R2S Capture)" }
        if speech {
            let st: SFSpeechRecognizerAuthorizationStatus = await withCheckedContinuation { c in
                SFSpeechRecognizer.requestAuthorization { c.resume(returning: $0) }
            }
            if st != .authorized { return "Speech recognition permission is off (Settings › R2S Capture)" }
        }
        return nil
    }
}

// MARK: - Speech to text

protocol SpeechToText: AnyObject {
    /// Listens for one utterance. `partial` gets interim text (device recogniser only); `done` gets the
    /// final text ("" = nothing heard) or an error. Both on the main queue.
    func start(partial: @escaping (String) -> Void, done: @escaping (Result<String, Error>) -> Void)
    /// Stop listening now and return what was heard so far.
    func finish()
    func cancel()
}

/// Apple Speech, streaming from the microphone, with on-device recognition when asked and supported.
final class AppleSTT: SpeechToText {
    private let settings: VoiceSettings
    private let audio = AVAudioEngine()
    private var task: SFSpeechRecognitionTask?
    private var request: SFSpeechAudioBufferRecognitionRequest?
    private var lastText = ""
    private var lastChange = Date()
    private var started = Date()
    private var timer: Timer?
    private var doneCB: ((Result<String, Error>) -> Void)?

    init(settings: VoiceSettings) { self.settings = settings }

    func start(partial: @escaping (String) -> Void, done: @escaping (Result<String, Error>) -> Void) {
        doneCB = done
        guard let rec = SFSpeechRecognizer(locale: Locale(identifier: settings.language)), rec.isAvailable else {
            return complete(.failure(VoiceError.unavailable("Speech recognition is not available for \(settings.language)")))
        }
        let wantOnDevice = settings.onDeviceSTT || settings.privateMode
        if settings.privateMode && !rec.supportsOnDeviceRecognition {
            return complete(.failure(VoiceError.unavailable("On-device recognition is not available for \(settings.language); private mode needs it (or use a Wyoming server)")))
        }
        let req = SFSpeechAudioBufferRecognitionRequest()
        req.shouldReportPartialResults = true
        req.requiresOnDeviceRecognition = wantOnDevice && rec.supportsOnDeviceRecognition
        req.addsPunctuation = false
        req.contextualStrings = ["stop", "turn left", "turn right", "go forward", "go back", "go to the"]
        request = req
        VoiceAudio.activate()
        let input = audio.inputNode
        let fmt = input.outputFormat(forBus: 0)
        input.removeTap(onBus: 0)
        input.installTap(onBus: 0, bufferSize: 1024, format: fmt) { [weak req] buf, _ in req?.append(buf) }
        audio.prepare()
        do { try audio.start() } catch { return complete(.failure(error)) }
        lastText = ""
        started = Date()
        lastChange = Date()
        task = rec.recognitionTask(with: req) { [weak self] result, error in
            DispatchQueue.main.async {
                guard let self = self else { return }
                if let r = result {
                    let t = r.bestTranscription.formattedString
                    if t != self.lastText { self.lastText = t; self.lastChange = Date(); partial(t) }
                    if r.isFinal { self.complete(.success(t)) }
                } else if let e = error {
                    // "No speech detected" ends a silent session; report it as empty.
                    self.complete(self.lastText.isEmpty ? .success("") : .success(self.lastText))
                    _ = e
                }
            }
        }
        // Apple's recogniser does not end a live utterance quickly by itself: end after 1.2 s without
        // new words, 6 s without any, or 12 s in total.
        timer = Timer.scheduledTimer(withTimeInterval: 0.2, repeats: true) { [weak self] _ in
            guard let self = self else { return }
            let quiet = Date().timeIntervalSince(self.lastChange)
            let total = Date().timeIntervalSince(self.started)
            if (!self.lastText.isEmpty && quiet > 1.2) || (self.lastText.isEmpty && total > 6) || total > 12 {
                self.finish()
            }
        }
    }

    func finish() {
        timer?.invalidate(); timer = nil
        stopAudio()
        request?.endAudio()
        // isFinal follows shortly; fall back to the last partial.
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
            guard let self = self else { return }
            self.complete(.success(self.lastText))
        }
    }

    func cancel() {
        doneCB = nil
        timer?.invalidate(); timer = nil
        stopAudio()
        task?.cancel()
        task = nil
    }

    private func stopAudio() {
        if audio.isRunning { audio.stop() }
        audio.inputNode.removeTap(onBus: 0)
    }

    private func complete(_ r: Result<String, Error>) {
        guard let cb = doneCB else { return }
        doneCB = nil
        timer?.invalidate(); timer = nil
        stopAudio()
        task?.cancel()
        task = nil
        cb(r)
    }
}

/// Microphone -> 16 kHz mono 16-bit chunks, for the network recognisers.
final class MicPCM16 {
    static let rate = 16000
    private let audio = AVAudioEngine()
    private var converter: AVAudioConverter?
    private let outFormat = AVAudioFormat(commonFormat: .pcmFormatInt16, sampleRate: Double(MicPCM16.rate), channels: 1, interleaved: true)!

    /// `chunk` is called on the audio thread with each converted chunk.
    func start(chunk: @escaping (Data) -> Void) throws {
        VoiceAudio.activate()
        let input = audio.inputNode
        let inFmt = input.outputFormat(forBus: 0)
        converter = AVAudioConverter(from: inFmt, to: outFormat)
        input.removeTap(onBus: 0)
        input.installTap(onBus: 0, bufferSize: 2048, format: inFmt) { [weak self] buf, _ in
            guard let self = self, let conv = self.converter else { return }
            let cap = AVAudioFrameCount(Double(buf.frameLength) * self.outFormat.sampleRate / inFmt.sampleRate) + 32
            guard let out = AVAudioPCMBuffer(pcmFormat: self.outFormat, frameCapacity: cap) else { return }
            var fed = false
            var err: NSError?
            conv.convert(to: out, error: &err) { _, status in
                if fed { status.pointee = .noDataNow; return nil }
                fed = true
                status.pointee = .haveData
                return buf
            }
            guard err == nil, out.frameLength > 0, let p = out.int16ChannelData else { return }
            chunk(Data(bytes: p[0], count: Int(out.frameLength) * 2))
        }
        audio.prepare()
        try audio.start()
    }

    func stop() {
        if audio.isRunning { audio.stop() }
        audio.inputNode.removeTap(onBus: 0)
    }
}

/// Wyoming ASR (Home Assistant's Whisper): streams audio while you speak, ends on silence.
/// OpenAI-compatible: records until silence, then uploads one WAV.
final class NetworkSTT: SpeechToText {
    private let settings: VoiceSettings
    private let mic = MicPCM16()
    private var vad = EnergyVAD(rate: MicPCM16.rate)
    private let q = DispatchQueue(label: "r2s.voice.stt")
    private var pcm = Data()
    private var wyoming: WyomingClient?
    private var finished = false
    private var doneCB: ((Result<String, Error>) -> Void)?

    init(settings: VoiceSettings) { self.settings = settings }

    func start(partial: @escaping (String) -> Void, done: @escaping (Result<String, Error>) -> Void) {
        doneCB = done
        finished = false
        pcm = Data()
        vad = EnergyVAD(rate: MicPCM16.rate)
        let lang = String(settings.language.prefix(2))
        Task { @MainActor in
            // Wyoming: open the stream before the first audio chunk so the events stay in order.
            if self.settings.stt == .wyoming {
                guard let (host, port) = VoiceURL.hostPort(self.settings.wyomingSTT, defaultPort: 10300) else {
                    return self.complete(.failure(VoiceError.server("Bad Wyoming STT address")))
                }
                let w = WyomingClient(host: host, port: port)
                do {
                    try await w.open()
                } catch {
                    return self.complete(.failure(VoiceError.server("Wyoming STT \(host):\(port): \(error.localizedDescription)")))
                }
                w.send(WyomingEvent(type: "transcribe", data: ["language": lang]))
                w.send(WyomingEvent(type: "audio-start", data: WyomingEvent.audioFormat(rate: MicPCM16.rate)))
                self.wyoming = w
            }
            guard self.doneCB != nil else { return }     // cancelled meanwhile
            do {
                try self.mic.start { [weak self] d in self?.q.async { self?.onAudio(d) } }
                partial("…")
            } catch { self.complete(.failure(error)) }
        }
    }

    private func onAudio(_ d: Data) {
        guard !finished else { return }
        if settings.stt == .wyoming {
            wyoming?.send(WyomingEvent(type: "audio-chunk", data: WyomingEvent.audioFormat(rate: MicPCM16.rate), payload: d))
        } else {
            pcm.append(d)
        }
        switch vad.feed(WAV.samples(d)) {
        case .done: endUtterance()
        case .timeout: finished = true; mic.stop(); wyoming?.close(); complete(.success(""))
        default: break
        }
    }

    func finish() { q.async { self.endUtterance() } }

    private func endUtterance() {
        guard !finished else { return }
        finished = true
        mic.stop()
        if settings.stt == .wyoming, let w = wyoming {
            w.send(WyomingEvent(type: "audio-stop"))
            Task {
                do {
                    while true {
                        let ev = try await w.next(timeout: 20)
                        if ev.type == "transcript" { self.complete(.success((ev.data["text"] as? String) ?? "")); break }
                        if ev.type == "error" { throw VoiceError.server((ev.data["text"] as? String) ?? "Wyoming error") }
                    }
                } catch { self.complete(.failure(error)) }
                w.close()
            }
        } else {
            let wav = WAV.encode(pcm16: pcm, rate: MicPCM16.rate)
            Task {
                do { self.complete(.success(try await OpenAIAudio.transcribe(wav: wav, settings: self.settings))) }
                catch { self.complete(.failure(error)) }
            }
        }
    }

    func cancel() {
        doneCB = nil
        q.async { self.finished = true }
        mic.stop()
        wyoming?.close()
    }

    private func complete(_ r: Result<String, Error>) {
        DispatchQueue.main.async {
            guard let cb = self.doneCB else { return }
            self.doneCB = nil
            cb(r.map { $0.trimmingCharacters(in: .whitespacesAndNewlines) })
        }
    }
}

// MARK: - Text to speech

protocol TextToSpeech: AnyObject {
    func speak(_ text: String, done: @escaping () -> Void)
    func stop()
}

final class AppleTTS: NSObject, TextToSpeech, AVSpeechSynthesizerDelegate {
    private let synth = AVSpeechSynthesizer()
    private let settings: VoiceSettings
    private var doneCB: (() -> Void)?

    init(settings: VoiceSettings) {
        self.settings = settings
        super.init()
        synth.delegate = self
    }

    func speak(_ text: String, done: @escaping () -> Void) {
        doneCB = done
        VoiceAudio.activate()
        let u = AVSpeechUtterance(string: text)
        u.rate = Float(settings.speechRate)
        if !settings.deviceVoice.isEmpty, let v = AVSpeechSynthesisVoice(identifier: settings.deviceVoice) {
            u.voice = v
        } else {
            u.voice = AVSpeechSynthesisVoice(language: settings.language)
        }
        synth.speak(u)
    }

    func stop() { synth.stopSpeaking(at: .immediate) }

    func speechSynthesizer(_ s: AVSpeechSynthesizer, didFinish u: AVSpeechUtterance) { fire() }
    func speechSynthesizer(_ s: AVSpeechSynthesizer, didCancel u: AVSpeechUtterance) { fire() }

    private func fire() {
        DispatchQueue.main.async {
            let cb = self.doneCB
            self.doneCB = nil
            cb?()
        }
    }
}

/// Piper over Wyoming, or an OpenAI-compatible /v1/audio/speech server; played with AVAudioPlayer.
final class NetworkTTS: NSObject, TextToSpeech, AVAudioPlayerDelegate {
    private let settings: VoiceSettings
    private var player: AVAudioPlayer?
    private var doneCB: (() -> Void)?
    private var task: Task<Void, Never>?
    var onError: ((String) -> Void)?

    init(settings: VoiceSettings) { self.settings = settings }

    func speak(_ text: String, done: @escaping () -> Void) {
        doneCB = done
        task = Task {
            do {
                let wav = self.settings.tts == .wyoming
                    ? try await self.wyomingSynth(text)
                    : try await OpenAIAudio.speech(text: text, settings: self.settings)
                await MainActor.run {
                    VoiceAudio.activate()
                    self.player = try? AVAudioPlayer(data: wav)
                    self.player?.delegate = self
                    if self.player?.play() != true { self.fire() }
                }
            } catch {
                await MainActor.run { self.onError?("TTS: \(error.localizedDescription)"); self.fire() }
            }
        }
    }

    private func wyomingSynth(_ text: String) async throws -> Data {
        guard let (host, port) = VoiceURL.hostPort(settings.wyomingTTS, defaultPort: 10200) else {
            throw VoiceError.server("Bad Wyoming TTS address")
        }
        let w = WyomingClient(host: host, port: port)
        defer { w.close() }
        try await w.open()
        var data: [String: Any] = ["text": text]
        if !settings.piperVoice.isEmpty { data["voice"] = ["name": settings.piperVoice] }
        w.send(WyomingEvent(type: "synthesize", data: data))
        var pcm = Data()
        var rate = 22050, channels = 1
        while true {
            let ev = try await w.next(timeout: 30)
            switch ev.type {
            case "audio-start":
                rate = (ev.data["rate"] as? Int) ?? rate
                channels = (ev.data["channels"] as? Int) ?? channels
            case "audio-chunk":
                if let r = ev.data["rate"] as? Int { rate = r }
                pcm.append(ev.payload)
            case "audio-stop":
                return WAV.encode(pcm16: pcm, rate: rate, channels: channels)
            case "error":
                throw VoiceError.server((ev.data["text"] as? String) ?? "Wyoming TTS error")
            default:
                break
            }
        }
    }

    func stop() {
        task?.cancel()
        player?.stop()
        fire()
    }

    func audioPlayerDidFinishPlaying(_ p: AVAudioPlayer, successfully flag: Bool) { fire() }

    private func fire() {
        DispatchQueue.main.async {
            let cb = self.doneCB
            self.doneCB = nil
            cb?()
        }
    }
}

// MARK: - OpenAI-compatible HTTP (audio)

enum OpenAIAudio {
    static func authorise(_ r: inout URLRequest) {
        let key = VoiceKeychain.apiKey
        if !key.isEmpty { r.setValue("Bearer \(key)", forHTTPHeaderField: "Authorization") }
    }

    static func transcribe(wav: Data, settings: VoiceSettings) async throws -> String {
        guard let url = URL(string: VoiceURL.openAIBase(settings.sttURL) + "/audio/transcriptions") else {
            throw VoiceError.server("Bad STT URL")
        }
        let boundary = "r2s-\(UUID().uuidString)"
        var body = Data()
        func field(_ name: String, _ value: String) {
            body.append(Data("--\(boundary)\r\nContent-Disposition: form-data; name=\"\(name)\"\r\n\r\n\(value)\r\n".utf8))
        }
        field("model", settings.sttModel)
        field("language", String(settings.language.prefix(2)))
        field("response_format", "json")
        body.append(Data("--\(boundary)\r\nContent-Disposition: form-data; name=\"file\"; filename=\"speech.wav\"\r\nContent-Type: audio/wav\r\n\r\n".utf8))
        body.append(wav)
        body.append(Data("\r\n--\(boundary)--\r\n".utf8))
        var r = URLRequest(url: url, timeoutInterval: 30)
        r.httpMethod = "POST"
        r.setValue("multipart/form-data; boundary=\(boundary)", forHTTPHeaderField: "Content-Type")
        authorise(&r)
        let (data, resp) = try await URLSession.shared.upload(for: r, from: body)
        try check(resp, data)
        if let o = try? JSONSerialization.jsonObject(with: data) as? [String: Any], let t = o["text"] as? String { return t }
        return String(decoding: data, as: UTF8.self)
    }

    static func speech(text: String, settings: VoiceSettings) async throws -> Data {
        guard let url = URL(string: VoiceURL.openAIBase(settings.ttsURL) + "/audio/speech") else {
            throw VoiceError.server("Bad TTS URL")
        }
        var r = URLRequest(url: url, timeoutInterval: 30)
        r.httpMethod = "POST"
        r.setValue("application/json", forHTTPHeaderField: "Content-Type")
        authorise(&r)
        r.httpBody = try JSONSerialization.data(withJSONObject: [
            "model": settings.ttsModel, "input": text, "voice": settings.ttsVoice, "response_format": "wav",
        ])
        let (data, resp) = try await URLSession.shared.data(for: r)
        try check(resp, data)
        return data
    }

    static func check(_ resp: URLResponse, _ data: Data) throws {
        guard let h = resp as? HTTPURLResponse else { return }
        guard (200..<300).contains(h.statusCode) else {
            throw VoiceError.server("HTTP \(h.statusCode): \(String(decoding: data.prefix(200), as: UTF8.self))")
        }
    }
}

// MARK: - LLM

/// Result of asking the model: actions to run and what to say.
struct LLMReply {
    var actions: [VoiceAction] = []
    var say = ""
}

/// OpenAI-compatible chat completions with tool calls (Ollama's /v1 endpoint, llama.cpp server, LM Studio,
/// vLLM, LocalAI, or a hosted service with an API key). `status` answers get_status inside the loop.
final class OpenAIChatAgent {
    private let settings: VoiceSettings
    private var history: [[String: Any]] = []

    init(settings: VoiceSettings) { self.settings = settings }

    func ask(_ text: String, state: String, labels: [String], status: @escaping () async -> String) async throws -> LLMReply {
        let system = VoiceTools.systemPrompt(state: state, labels: labels, jsonOnly: !settings.llmTools)
        var messages: [[String: Any]] = [["role": "system", "content": system]]
        messages += history.suffix(8)
        messages.append(["role": "user", "content": text])
        var reply = LLMReply()
        for _ in 0..<4 {
            var body: [String: Any] = ["model": settings.llmModel, "messages": messages, "temperature": 0.2, "stream": false]
            if settings.llmTools { body["tools"] = VoiceTools.openAITools; body["tool_choice"] = "auto" }
            let data = try await post(body)
            guard let (content, calls) = VoiceTools.parseChatResponse(data) else {
                throw VoiceError.server("Unexpected reply: \(String(decoding: data.prefix(200), as: UTF8.self))")
            }
            if calls.isEmpty {
                if let j = VoiceTools.parseJSONReply(content), !settings.llmTools || !j.actions.isEmpty {
                    reply.actions += j.actions
                    reply.say = j.say
                } else {
                    reply.say = content
                }
                break
            }
            // Run get_status here and loop so the model can answer with it; motion tools are collected
            // and run by the caller after the model is done.
            var assistant: [String: Any] = ["role": "assistant", "content": content]
            assistant["tool_calls"] = calls.map { ["id": $0.id, "type": "function", "function": ["name": $0.name, "arguments": $0.arguments]] }
            messages.append(assistant)
            var needAnswer = false
            for c in calls {
                let result: String
                if let a = VoiceTools.action(name: c.name, args: c.arguments) {
                    if a == .status {
                        result = await status()
                        needAnswer = true
                    } else {
                        reply.actions.append(a)
                        result = "{\"ok\":true,\"started\":\"\(a.summary)\"}"
                    }
                } else {
                    result = "{\"ok\":false,\"error\":\"unknown tool or bad arguments\"}"
                    needAnswer = true
                }
                messages.append(["role": "tool", "tool_call_id": c.id, "name": c.name, "content": result])
            }
            reply.say = content
            if !needAnswer && !content.isEmpty { break }
        }
        history.append(["role": "user", "content": text])
        history.append(["role": "assistant", "content": reply.say.isEmpty ? reply.actions.map(\.summary).joined(separator: ", ") : reply.say])
        return reply
    }

    func reset() { history.removeAll() }

    private func post(_ body: [String: Any]) async throws -> Data {
        guard let url = URL(string: VoiceURL.openAIBase(settings.llmURL) + "/chat/completions") else {
            throw VoiceError.server("Bad LLM URL")
        }
        var r = URLRequest(url: url, timeoutInterval: 45)
        r.httpMethod = "POST"
        r.setValue("application/json", forHTTPHeaderField: "Content-Type")
        OpenAIAudio.authorise(&r)
        r.httpBody = try JSONSerialization.data(withJSONObject: body)
        let (data, resp) = try await URLSession.shared.data(for: r)
        try OpenAIAudio.check(resp, data)
        return data
    }
}

/// Apple's on-device foundation model (iOS 26, Apple Intelligence). Asked for a JSON reply rather than
/// Tool conformances so the same parser serves every model.
enum AppleLLM {
    static var isAvailable: Bool {
        #if canImport(FoundationModels)
        if #available(iOS 26.0, *) { return SystemLanguageModel.default.isAvailable }
        #endif
        return false
    }

    static var unavailableReason: String {
        #if canImport(FoundationModels)
        if #available(iOS 26.0, *) {
            switch SystemLanguageModel.default.availability {
            case .available: return ""
            case .unavailable(let r): return "Apple on-device model unavailable (\(r))"
            @unknown default: return "Apple on-device model unavailable"
            }
        }
        return "Apple on-device model needs iOS 26"
        #else
        return "This build has no Apple on-device model (built with an older Xcode)"
        #endif
    }

    static func ask(_ text: String, state: String, labels: [String]) async throws -> LLMReply {
        #if canImport(FoundationModels)
        if #available(iOS 26.0, *) {
            let session = LanguageModelSession(instructions: VoiceTools.systemPrompt(state: state, labels: labels, jsonOnly: true))
            let r = try await session.respond(to: text)
            let content = r.content
            if let j = VoiceTools.parseJSONReply(content) { return LLMReply(actions: j.actions, say: j.say) }
            return LLMReply(actions: [], say: content)
        }
        #endif
        throw VoiceError.unavailable(unavailableReason)
    }
}

// MARK: - Wyoming TCP client

final class WyomingClient {
    private let conn: NWConnection
    private let q = DispatchQueue(label: "r2s.wyoming")
    private let decoder = WyomingDecoder()
    private var pending: [WyomingEvent] = []
    private var waiter: CheckedContinuation<WyomingEvent, Error>?
    private var waiterID = 0
    private var failure: Error?
    private var opened = false

    init(host: String, port: Int) {
        conn = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(integerLiteral: UInt16(clamping: port)), using: .tcp)
    }

    func open(timeout: Double = 5) async throws {
        try await withCheckedThrowingContinuation { (c: CheckedContinuation<Void, Error>) in
            var resumed = false
            func finish(_ r: Result<Void, Error>) {
                guard !resumed else { return }
                resumed = true
                c.resume(with: r)
            }
            conn.stateUpdateHandler = { [weak self] st in
                switch st {
                case .ready:
                    self?.opened = true
                    self?.receive()
                    finish(.success(()))
                case .failed(let e), .waiting(let e):
                    self?.fail(e)
                    finish(.failure(e))
                case .cancelled:
                    finish(.failure(VoiceError.server("connection closed")))
                default: break
                }
            }
            conn.start(queue: q)
            q.asyncAfter(deadline: .now() + timeout) { finish(.failure(VoiceError.timeout("connect"))) }
        }
    }

    func send(_ e: WyomingEvent) {
        conn.send(content: e.encode(), completion: .contentProcessed { _ in })
    }

    func next(timeout: Double) async throws -> WyomingEvent {
        try await withCheckedThrowingContinuation { (c: CheckedContinuation<WyomingEvent, Error>) in
            q.async {
                if !self.pending.isEmpty { return c.resume(returning: self.pending.removeFirst()) }
                if let f = self.failure { return c.resume(throwing: f) }
                self.waiterID += 1
                let id = self.waiterID
                self.waiter = c
                self.q.asyncAfter(deadline: .now() + timeout) {
                    if self.waiterID == id, let w = self.waiter {
                        self.waiter = nil
                        w.resume(throwing: VoiceError.timeout("Wyoming server reply"))
                    }
                }
            }
        }
    }

    func close() { conn.cancel() }

    private func receive() {
        conn.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, _, complete, error in
            guard let self = self else { return }
            if let d = data, !d.isEmpty {
                self.pending += self.decoder.feed(d)
                if let w = self.waiter, !self.pending.isEmpty {
                    self.waiter = nil
                    w.resume(returning: self.pending.removeFirst())
                }
            }
            if let e = error { self.fail(e); return }
            if complete { self.fail(VoiceError.server("server closed the connection")); return }
            self.receive()
        }
    }

    private func fail(_ e: Error) {
        failure = e
        if let w = waiter {
            waiter = nil
            w.resume(throwing: e)
        }
    }
}
