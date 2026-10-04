import Foundation
import SwiftUI
import UIKit

/// Voice control for the navigation screen, in the style of Home Assistant's Assist pipeline:
/// speech to text -> built-in sentence matcher (or an LLM with tool calls) -> nav actions -> spoken reply.
///
/// Safety: "stop" (and halt / freeze / emergency) stops the robot as soon as it appears in the live
/// transcript, before the sentence ends and without any model. Every motion command is clamped
/// (VoiceLimits) and runs through NavEngine, so the obstacle stop and speed limits still apply.
final class VoiceAssistant: ObservableObject {
    enum Phase: Equatable { case idle, listening, thinking, acting, speaking }

    struct Line: Identifiable {
        let id = UUID()
        let user: Bool
        let text: String
    }

    @Published private(set) var phase: Phase = .idle
    @Published private(set) var partial = ""
    @Published private(set) var lines: [Line] = []
    @Published private(set) var lastError = ""
    @Published var settings = VoiceSettings.load() {
        didSet {
            settings.save()
            rebuild()
        }
    }

    private weak var engine: NavEngine?
    private var stt: SpeechToText?
    private var tts: TextToSpeech?
    private var agent: OpenAIChatAgent?
    private var runner: Task<Void, Never>?
    private var stoppedThisUtterance = false

    init() { rebuild() }

    func attach(_ e: NavEngine) { engine = e }

    var privacyNote: String {
        var parts: [String] = []
        switch settings.stt {
        case .device: parts.append(settings.onDeviceSTT || settings.privateMode ? "speech on iPhone" : "speech via Apple (may use Apple servers)")
        case .wyoming, .openai: parts.append("speech on your server")
        }
        switch settings.llm {
        case .rules: parts.append("built-in commands")
        case .device: parts.append(AppleLLM.isAvailable ? "Apple on-device model" : "built-in commands (no Apple model)")
        case .openai: parts.append(VoiceURL.isLocal(settings.llmURL) ? "LLM on your network" : "LLM in the cloud")
        }
        return parts.joined(separator: " · ")
    }

    private func rebuild() {
        stt?.cancel()
        tts?.stop()
        stt = settings.stt == .device ? AppleSTT(settings: settings) : NetworkSTT(settings: settings)
        switch settings.tts {
        case .device: tts = AppleTTS(settings: settings)
        case .wyoming, .openai:
            let n = NetworkTTS(settings: settings)
            n.onError = { [weak self] e in self?.lastError = e }
            tts = n
        case .off: tts = nil
        }
        agent = settings.llm == .openai ? OpenAIChatAgent(settings: settings) : nil
    }

    // MARK: listening

    /// Mic button: start listening, or end the utterance now if already listening.
    func toggle() {
        switch phase {
        case .listening:
            stt?.finish()
        case .speaking:
            tts?.stop()
            listen()
        case .idle, .acting:
            listen()
        case .thinking:
            break
        }
    }

    func listen() {
        if settings.privateMode, let bad = settings.nonLocalEndpoints.first {
            fail("Private mode: \(bad) is not on your local network")
            return
        }
        Task { @MainActor in
            if let err = await VoiceAudio.requestPermissions(speech: settings.stt == .device) {
                fail(err)
                return
            }
            engine?.voice.stopAll()          // keep guidance speech out of the microphone
            tts?.stop()
            lastError = ""
            partial = ""
            stoppedThisUtterance = false
            phase = .listening
            UIImpactFeedbackGenerator(style: .light).impactOccurred()
            stt?.start(partial: { [weak self] text in
                guard let self = self else { return }
                self.partial = text
                if !self.stoppedThisUtterance && IntentParser.containsStop(text) {
                    self.stoppedThisUtterance = true
                    self.emergencyStop()
                }
            }, done: { [weak self] result in
                guard let self = self else { return }
                self.partial = ""
                switch result {
                case .success(let text):
                    if text.isEmpty {
                        self.phase = .idle
                    } else {
                        self.handle(text)
                    }
                case .failure(let e):
                    self.fail(e.localizedDescription)
                }
            })
        }
    }

    func cancel() {
        stt?.cancel()
        tts?.stop()
        runner?.cancel()
        phase = .idle
    }

    private func emergencyStop() {
        runner?.cancel()
        engine?.stop()
        add(false, "Stopped.")
    }

    // MARK: understanding

    func handle(_ text: String) {
        add(true, text)
        if IntentParser.containsStop(text) {
            if !stoppedThisUtterance { emergencyStop() }
            say("Stopped.")
            return
        }
        let labels = engine?.roomItems.map(\.label) ?? []
        if settings.llm == .rules || settings.preferLocal || (settings.llm == .device && !AppleLLM.isAvailable),
           let actions = IntentParser.parse(text, labels: labels) {
            run(actions, reply: nil)
            return
        }
        switch settings.llm {
        case .rules:
            say("Sorry, I only know commands like go forward one meter, turn left, go to the table, or stop.")
        case .device:
            guard AppleLLM.isAvailable else {
                say("Sorry, I didn't understand. \(AppleLLM.unavailableReason).")
                return
            }
            phase = .thinking
            let state = stateSummary()
            Task { @MainActor in
                do {
                    let r = try await AppleLLM.ask(text, state: state, labels: labels)
                    self.run(r.actions, reply: r.say)
                } catch {
                    self.fail("On-device model: \(error.localizedDescription)")
                }
            }
        case .openai:
            guard let agent = agent else { return }
            phase = .thinking
            let state = stateSummary()
            Task { @MainActor in
                do {
                    let r = try await agent.ask(text, state: state, labels: labels) { [weak self] in
                        await MainActor.run { self?.statusJSON() ?? "{}" }
                    }
                    self.run(r.actions, reply: r.say)
                } catch {
                    self.fail("LLM: \(error.localizedDescription)")
                }
            }
        }
    }

    // MARK: acting

    private func run(_ actions: [VoiceAction], reply: String?) {
        runner?.cancel()
        let acts = actions.map(VoiceLimits.clamp)
        if acts.isEmpty {
            say(reply?.isEmpty == false ? reply! : "Sorry, I didn't understand that.")
            return
        }
        // Speak the model's reply (or our own confirmation) while the first action starts.
        let confirm = (reply?.isEmpty == false) ? reply! : confirmation(acts)
        phase = .acting
        runner = Task { @MainActor in
            for (i, a) in acts.enumerated() {
                if Task.isCancelled { return }
                let msg = await self.execute(a)
                if i == 0 {
                    if let m = msg { self.say(m) } else { self.say(confirm) }
                } else if let m = msg {
                    self.say(m)
                }
                if i < acts.count - 1 { await self.waitForCompletion(of: a) }
            }
            if self.phase == .acting { self.phase = .idle }
        }
    }

    /// Runs one action. Returns a message to say instead of the confirmation (errors, status answers).
    @MainActor
    private func execute(_ a: VoiceAction) async -> String? {
        guard let e = engine else { return "Navigation is not running." }
        let linked = e.link.state.isConnected
        switch a {
        case .stop:
            e.stop()
        case .move(let m):
            guard linked else { return "The robot isn't connected." }
            if e.mode == .guide { e.mode = .manual }
            let speed = min(settings.moveSpeed, e.settings.maxSpeed * 100)
            e.move(cm: m * 100, speed: speed)
        case .turn(let d):
            guard linked else { return "The robot isn't connected." }
            if e.mode == .guide { e.mode = .manual }
            e.turn(degrees: d, speed: min(settings.turnSpeed, e.settings.maxTurnRateDeg))
        case .goTo(let target):
            let items = e.roomItems
            guard let label = IntentParser.matchLabel(target, labels: items.map(\.label)),
                  let item = items.first(where: { $0.label == label }) else {
                let known = items.prefix(6).map(\.label).joined(separator: ", ")
                return known.isEmpty ? "I don't know where \(target) is. Load a room scan first, or tap the map."
                    : "I don't know where \(target) is. I know \(known)."
            }
            e.setGoal(item: item)
            if linked { e.mode = .auto; e.go() }
            return linked ? nil : "Guiding you to the \(label)."
        case .goToPoint(let f, let l):
            guard let pose = e.hud.pose else { return "I'm not tracking yet." }
            let fwd = pose.forward
            let left = P2(-sin(pose.theta), -cos(pose.theta))
            e.setGoal(pose.p + fwd * f + left * l, label: "voice point")
            if linked { e.mode = .auto; e.go() }
        case .setMode(let m):
            if let mode = NavDriveMode(rawValue: m) { e.mode = mode }
        case .clearGoal:
            e.clearGoal()
        case .status:
            return stateSummary(spoken: true)
        case .say(let s):
            return s
        }
        return nil
    }

    /// Waits for a move / turn / go-to to finish before the next step of a multi-step command.
    @MainActor
    private func waitForCompletion(of a: VoiceAction) async {
        guard let e = engine, a.isMotion else { return }
        let limit: Double
        switch a {
        case .move(let m): limit = abs(m) / max(0.05, settings.moveSpeed / 100) * 3 + 5
        case .turn(let d): limit = abs(d) / max(5, settings.turnSpeed) * 3 + 5
        default: limit = 180
        }
        let start = Date()
        try? await Task.sleep(nanoseconds: 400_000_000)
        while Date().timeIntervalSince(start) < limit && !Task.isCancelled {
            let s = e.hud.state
            let busy = s.hasPrefix("Moving") || s.hasPrefix("Turning") || s.hasPrefix("Following") || s.hasPrefix("Backing")
            if !busy { return }
            try? await Task.sleep(nanoseconds: 200_000_000)
        }
    }

    private func confirmation(_ acts: [VoiceAction]) -> String {
        func one(_ a: VoiceAction) -> String {
            switch a {
            case .stop: return "Stopping"
            case .move(let m):
                let d = abs(m) < 1 ? String(format: "%.0f centimeters", abs(m) * 100) : String(format: "%.1f meters", abs(m))
                return (m < 0 ? "Backing up " : "Moving forward ") + d
            case .turn(let d): return String(format: "Turning %@ %.0f degrees", d >= 0 ? "left" : "right", abs(d))
            case .goTo(let t): return "Going to the \(t)"
            case .goToPoint: return "Going there"
            case .setMode(let m): return "\(m.capitalized) mode"
            case .clearGoal: return "Goal cleared"
            case .status: return ""
            case .say(let s): return s
            }
        }
        return acts.map(one).filter { !$0.isEmpty }.joined(separator: ", then ") + "."
    }

    // MARK: state for the model and for "status"

    @MainActor
    func stateSummary(spoken: Bool = false) -> String {
        guard let e = engine else { return "navigation not running" }
        let h = e.hud
        var parts: [String] = []
        if let p = h.pose {
            parts.append(spoken ? "I'm tracking" : String(format: "position x %.2f z %.2f m, heading %.0f°", p.p.x, p.p.z, deg(p.theta)))
        } else {
            parts.append("not tracking yet")
        }
        parts.append("mode \(e.mode.rawValue)")
        if h.goal != nil {
            let g = h.goalLabel.isEmpty ? "a point" : "the \(h.goalLabel)"
            parts.append(String(format: "goal %@, %.1f meters to go", g, h.remaining))
        }
        if h.frontGap.isFinite { parts.append(String(format: "%.1f meters clear ahead", h.frontGap)) }
        parts.append(e.link.state.isConnected ? "robot connected" : "robot not connected")
        if !e.link.robotStatus.isEmpty && !spoken { parts.append("robot \(e.link.robotStatus)") }
        if spoken, let v = e.link.robotStatus.split(separator: "·").first(where: { $0.contains("V") }) {
            parts.append("battery \(v.trimmingCharacters(in: .whitespaces).replacingOccurrences(of: "V", with: "volts"))")
        }
        parts.append("state \(h.state)")
        return parts.joined(separator: spoken ? ", " : "; ") + (spoken ? "." : "")
    }

    @MainActor
    func statusJSON() -> String {
        guard let e = engine else { return "{}" }
        let h = e.hud
        var o: [String: Any] = ["mode": e.mode.rawValue, "state": h.state, "robot_connected": e.link.state.isConnected,
                                "robot_status": e.link.robotStatus, "tracking": h.trackingOK]
        if let p = h.pose { o["pose"] = ["x": p.p.x, "z": p.p.z, "heading_deg": deg(p.theta)] }
        if h.goal != nil { o["goal"] = ["label": h.goalLabel, "remaining_m": h.remaining] }
        for (k, v) in [("front_m", h.front), ("left_m", h.left), ("right_m", h.right), ("rear_m", h.rear)] where v.isFinite {
            o[k] = (v * 100).rounded() / 100
        }
        o["known_objects"] = e.roomItems.map(\.label)
        guard let d = try? JSONSerialization.data(withJSONObject: o) else { return "{}" }
        return String(decoding: d, as: UTF8.self)
    }

    // MARK: output

    private func say(_ text: String) {
        let t = text.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !t.isEmpty else { if phase != .acting { phase = .idle }; return }
        add(false, t)
        guard settings.speakReplies, let tts = tts else {
            if phase != .acting { phase = .idle }
            return
        }
        let wasActing = phase == .acting
        if !wasActing { phase = .speaking }
        tts.speak(t) { [weak self] in
            guard let self = self else { return }
            if self.phase == .speaking { self.phase = .idle }
        }
    }

    private func add(_ user: Bool, _ text: String) {
        lines.append(Line(user: user, text: text))
        if lines.count > 30 { lines.removeFirst(lines.count - 30) }
    }

    private func fail(_ msg: String) {
        lastError = msg
        add(false, msg)
        phase = .idle
    }
}
