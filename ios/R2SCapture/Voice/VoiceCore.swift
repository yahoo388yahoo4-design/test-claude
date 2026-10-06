import Foundation

// Platform-independent part of voice control (Foundation only, so ios/NavTests can run it on Linux):
// what a command means (VoiceAction), the local sentence matcher that handles common commands without an
// LLM (like Home Assistant's "prefer handling commands locally"), the LLM tool definitions and response
// parsing (OpenAI-compatible tool calls, or a JSON reply for models without tool support), the Wyoming
// protocol framing used by Home Assistant's Whisper and Piper servers, an energy voice-activity detector,
// and WAV helpers. The Android app has the same logic in voice/VoiceCore.kt; keep them in step.

// MARK: - Actions

/// One thing the voice assistant can make the robot do. Distances in metres (+ forward), angles in
/// degrees (+ left), the same conventions as robot/PROTOCOL.md.
enum VoiceAction: Equatable {
    case stop
    case move(meters: Double)
    case turn(degrees: Double)
    case goTo(target: String)
    case goToPoint(forward: Double, left: Double)
    case setMode(String)            // "guide", "auto", "manual"
    case clearGoal
    case savePlace(String)         // remember the robot's current position under this name
    case status
    case say(String)

    var isMotion: Bool {
        switch self {
        case .move, .turn, .goTo, .goToPoint: return true
        default: return false
        }
    }

    /// Short text for the transcript ("move 1.0 m").
    var summary: String {
        switch self {
        case .stop: return "stop"
        case .move(let m): return String(format: "move %.2f m", m)
        case .turn(let d): return String(format: "turn %.0f°", d)
        case .goTo(let t): return "go to \(t)"
        case .goToPoint(let f, let l): return String(format: "go to point %.1f m ahead, %.1f m left", f, l)
        case .setMode(let m): return "\(m) mode"
        case .clearGoal: return "clear goal"
        case .savePlace(let n): return "remember \(n)"
        case .status: return "status"
        case .say(let s): return "say \"\(s)\""
        }
    }
}

/// Hard limits on what one voice command may do; the nav screen's own speed limits still apply.
enum VoiceLimits {
    static let maxMove = 5.0          // m per command
    static let maxTurn = 360.0        // deg per command
    static let maxPoint = 10.0        // m for go_to_point

    static func clamp(_ a: VoiceAction) -> VoiceAction {
        switch a {
        case .move(let m): return .move(meters: max(-maxMove, min(maxMove, m.isFinite ? m : 0)))
        case .turn(let d): return .turn(degrees: max(-maxTurn, min(maxTurn, d.isFinite ? d : 0)))
        case .goToPoint(let f, let l):
            return .goToPoint(forward: max(-maxPoint, min(maxPoint, f.isFinite ? f : 0)),
                              left: max(-maxPoint, min(maxPoint, l.isFinite ? l : 0)))
        default: return a
        }
    }
}

// MARK: - Local sentence matcher

/// Understands the common commands (English) without any network or model. Returns nil when the sentence
/// is not one of them, so the caller can hand it to the LLM.
enum IntentParser {
    static let stopWords: Set<String> = ["stop", "halt", "freeze", "emergency", "abort", "whoa"]

    /// True if a (partial) transcript contains a stop word: checked on every partial result so the robot
    /// stops before the sentence is even finished.
    static func containsStop(_ text: String) -> Bool {
        words(normalise(text)).contains { stopWords.contains($0) }
    }

    static func parse(_ text: String, labels: [String] = []) -> [VoiceAction]? {
        let t = normalise(text)
        guard !t.isEmpty else { return nil }
        if containsStop(t) { return [.stop] }
        // "move forward one meter then turn left" -> two steps
        let parts = splitSteps(t)
        var out: [VoiceAction] = []
        for p in parts {
            guard let a = parseOne(p, labels: labels) else { return nil }
            out.append(a)
        }
        return out.isEmpty ? nil : out
    }

    static let verbs: Set<String> = ["go", "move", "drive", "turn", "rotate", "spin", "come", "back", "head", "stop", "navigate", "take", "find"]

    /// "move forward one and a half meters and then turn left" -> ["move forward one and a half meters", "turn left"]:
    /// splits on "then", and on "and" only when a command verb follows.
    static func splitSteps(_ t: String) -> [String] {
        var steps: [[String]] = [[]]
        let w = t.split(separator: " ").map(String.init)
        var i = 0
        while i < w.count {
            if w[i] == "then" || (w[i] == "and" && i + 1 < w.count && (w[i + 1] == "then" || verbs.contains(w[i + 1]))) {
                if !(steps.last ?? []).isEmpty { steps.append([]) }
                if w[i] == "and" && i + 1 < w.count && w[i + 1] == "then" { i += 1 }
            } else {
                steps[steps.count - 1].append(w[i])
            }
            i += 1
        }
        return steps.filter { !$0.isEmpty }.map { $0.joined(separator: " ") }
    }

    static func parseOne(_ t: String, labels: [String]) -> VoiceAction? {
        let w = words(t)
        let has: (String) -> Bool = { w.contains($0) }
        for lead in ["remember this place as", "remember this spot as", "remember this position as", "remember this as",
                     "save this place as", "save this spot as", "save this position as", "call this place", "call this spot",
                     "mark this place as", "mark this spot as", "this place is", "this is the"] where t.hasPrefix(lead + " ") {
            let name = stripArticles(String(t.dropFirst(lead.count + 1)))
            return name.isEmpty ? nil : .savePlace(name)
        }
        if t.hasPrefix("cancel") || t.contains("clear the goal") || t.contains("clear goal") || t == "never mind" {
            return .clearGoal
        }
        for m in ["guide", "auto", "manual"] where (has(m) || (m == "auto" && has("automatic"))) && has("mode") {
            return .setMode(m)
        }
        if t.contains("where are you") || t.contains("where am i") || has("status") || has("battery")
            || t.contains("how far") || t.contains("what do you see") || t.contains("what is around")
            || t.contains("what's around") || t.contains("are you connected") {
            return .status
        }
        // go to / drive to / take me to <object>
        for lead in ["go to", "drive to", "navigate to", "take me to", "head to", "move to", "go over to", "bring me to", "find"] {
            if t.hasPrefix(lead + " ") || t.contains(" " + lead + " ") {
                let rest = String(t[t.range(of: lead + " ")!.upperBound...])
                let target = stripArticles(rest)
                if target.isEmpty { return nil }
                if let l = matchLabel(target, labels: labels) { return .goTo(target: l) }
                return nil   // unknown place: let the LLM (or the reply) handle it
            }
        }
        if t.contains("turn around") || t.contains("about face") { return .turn(degrees: 180) }
        if has("turn") || has("rotate") || has("spin") {
            let left = has("left") || has("counterclockwise") || has("anticlockwise")
            let right = has("right") || has("clockwise")
            guard left != right else { return nil }
            var deg = number(in: w) ?? (has("little") || has("bit") || has("slightly") ? 15 : 90)
            if has("half") && !has("degrees") { deg = 180 }        // "turn half way around"
            return .turn(degrees: left ? deg : -deg)
        }
        let back = has("back") || has("backward") || has("backwards") || has("reverse")
        let fwd = has("forward") || has("forwards") || has("ahead") || has("straight")
        if (has("go") || has("move") || has("drive") || has("come") || has("roll") || back || fwd) && (back || fwd) {
            var m = distance(in: w) ?? (has("little") || has("bit") ? 0.2 : 0.5)
            if back { m = -abs(m) }
            return .move(meters: m)
        }
        return nil
    }

    // MARK: helpers

    /// Lower case, punctuation to spaces (keeping decimals such as 1.5 and "°"), filler words removed.
    static func normalise(_ s: String) -> String {
        let chars = Array(s.lowercased().replacingOccurrences(of: "’", with: "'"))
        var res = ""
        for (i, c) in chars.enumerated() {
            if c == "." && i > 0 && i + 1 < chars.count && chars[i - 1].isNumber && chars[i + 1].isNumber {
                res.append(c)
            } else if c == "°" {
                res += " degrees "
            } else if c.isLetter || c.isNumber || c == "'" {
                res.append(c)
            } else {
                res.append(" ")
            }
        }
        var w = res.split(separator: " ").map(String.init).filter { $0 != "please" }
        // robot name / wake phrase at the start ("hey robot, go forward")
        while let f = w.first, ["hey", "ok", "okay", "robot"].contains(f) { w.removeFirst() }
        return w.joined(separator: " ")
    }

    static func words(_ s: String) -> [String] {
        s.lowercased().split(whereSeparator: { !$0.isLetter && !$0.isNumber && $0 != "." && $0 != "'" }).map(String.init)
    }

    static func stripArticles(_ s: String) -> String {
        var w = words(s)
        while let f = w.first, ["the", "a", "an", "my", "that", "this"].contains(f) { w.removeFirst() }
        if let i = w.firstIndex(where: { ["please", "now", "then"].contains($0) }) { w = Array(w[..<i]) }
        return w.joined(separator: " ")
    }

    static let numberWords: [String: Double] = [
        "zero": 0, "one": 1, "a": 1, "an": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7,
        "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "fifteen": 15, "twenty": 20, "thirty": 30,
        "forty": 40, "fourty": 40, "fifty": 50, "sixty": 60, "seventy": 70, "eighty": 80, "ninety": 90,
        "hundred": 100, "half": 0.5, "quarter": 0.25, "twice": 2,
    ]

    /// First number in the words: digits ("1.5"), or words ("one and a half", "forty five").
    static func number(in w: [String]) -> Double? {
        var i = 0
        while i < w.count {
            if let d = Double(w[i]) { return d }
            if let v = numberWords[w[i]], !(["a", "an"].contains(w[i]) && i + 1 < w.count && numberWords[w[i + 1]] == nil && !unitWords.contains(w[i + 1])) {
                var total = v
                var j = i + 1
                // "forty five", "one hundred", "one and a half"
                if j < w.count, let v2 = numberWords[w[j]], v >= 20, v2 < 10, v2 >= 1 { total += v2; j += 1 }
                if j < w.count, w[j] == "hundred" { total *= 100; j += 1 }
                if j + 2 < w.count, w[j] == "and", ["a", "one"].contains(w[j + 1]), w[j + 2] == "half" { total += 0.5 }
                if ["a", "an"].contains(w[i]), j < w.count, w[j] == "half" { total = 0.5 }
                if ["a", "an"].contains(w[i]), j < w.count, w[j] == "quarter" { total = 0.25 }
                return total
            }
            i += 1
        }
        return nil
    }

    static let unitWords: Set<String> = ["m", "meter", "meters", "metre", "metres", "cm", "centimeter", "centimeters",
                                         "centimetre", "centimetres", "foot", "feet", "ft", "inch", "inches", "step", "steps"]

    /// Distance in metres from "1.5 meters", "fifty centimeters", "two feet", "3 steps"; bare numbers above
    /// 5 are taken as centimetres ("move forward 50").
    static func distance(in w: [String]) -> Double? {
        guard let n = number(in: w) else { return nil }
        if w.contains(where: { ["cm", "centimeter", "centimeters", "centimetre", "centimetres"].contains($0) }) { return n / 100 }
        if w.contains(where: { ["foot", "feet", "ft"].contains($0) }) { return n * 0.3048 }
        if w.contains(where: { ["inch", "inches"].contains($0) }) { return n * 0.0254 }
        if w.contains(where: { ["step", "steps"].contains($0) }) { return n * 0.5 }
        if w.contains(where: { ["m", "meter", "meters", "metre", "metres"].contains($0) }) { return n }
        return n > 5 ? n / 100 : n
    }

    /// Best matching object label: exact, then contained, then most shared words.
    static func matchLabel(_ target: String, labels: [String]) -> String? {
        let t = target.lowercased()
        if let l = labels.first(where: { $0.lowercased() == t }) { return l }
        if let l = labels.first(where: { $0.lowercased().contains(t) || t.contains($0.lowercased()) }) { return l }
        let tw = Set(words(t).map(singular))
        var best: (String, Int)?
        for l in labels {
            let n = Set(words(l).map(singular)).intersection(tw).count
            if n > 0 && n > (best?.1 ?? 0) { best = (l, n) }
        }
        return best?.0
    }

    static func singular(_ s: String) -> String { s.count > 3 && s.hasSuffix("s") ? String(s.dropLast()) : s }
}

// MARK: - LLM tools

enum VoiceTools {
    /// OpenAI-compatible `tools` array (Ollama, llama.cpp server, LM Studio, vLLM, LocalAI, OpenAI).
    static var openAITools: [[String: Any]] {
        func fn(_ name: String, _ desc: String, _ props: [String: [String: Any]] = [:], required: [String] = []) -> [String: Any] {
            ["type": "function",
             "function": ["name": name, "description": desc,
                          "parameters": ["type": "object", "properties": props, "required": required] as [String: Any]] as [String: Any]]
        }
        return [
            fn("stop", "Stop the robot immediately."),
            fn("move", "Drive straight. Positive = forward, negative = backward.",
               ["distance_m": ["type": "number", "description": "metres, -5 to 5"]], required: ["distance_m"]),
            fn("turn", "Turn in place. Positive = left (counter-clockwise), negative = right.",
               ["degrees": ["type": "number", "description": "-360 to 360"]], required: ["degrees"]),
            fn("go_to", "Plan a path to a known object or place and follow it.",
               ["target": ["type": "string", "description": "one of the known object names"]], required: ["target"]),
            fn("go_to_point", "Plan a path to a point relative to the robot.",
               ["forward_m": ["type": "number"], "left_m": ["type": "number", "description": "positive = left"]],
               required: ["forward_m", "left_m"]),
            fn("set_mode", "Switch navigation mode: guide (voice and arrows only), auto (drive the robot along the path), manual.",
               ["mode": ["type": "string", "enum": ["guide", "auto", "manual"]]], required: ["mode"]),
            fn("clear_goal", "Forget the current goal and stop following it."),
            fn("save_place", "Remember the robot's current position under a name, for go_to later.",
               ["name": ["type": "string"]], required: ["name"]),
            fn("get_status", "Get the robot's position, goal, obstacles, link and battery."),
        ]
    }

    static func systemPrompt(state: String, labels: [String], jsonOnly: Bool) -> String {
        var s = """
        You control a small wheeled robot with a phone as its sensor head, by voice. Units: metres and degrees; \
        positive turn = left. Only do what the user asked; ask back if a command is unclear or unsafe. \
        Prefer one short spoken reply (under 20 words, no markdown, no emoji).
        Known objects and places: \(labels.isEmpty ? "none yet (load a room scan, or save places)" : labels.joined(separator: ", ")).
        Current state: \(state)
        """
        if jsonOnly {
            s += """

            Reply with ONLY a JSON object, no other text: \
            {"actions":[{"tool":"move","distance_m":1.0}],"say":"Moving one metre."}. \
            Tools: stop; move{distance_m}; turn{degrees}; go_to{target}; go_to_point{forward_m,left_m}; \
            set_mode{mode: guide|auto|manual}; clear_goal; save_place{name}; get_status. Use "actions":[] to only answer.
            """
        }
        return s
    }

    /// One tool call -> action. `args` is the JSON arguments as a string (OpenAI) or an object (Ollama).
    static func action(name: String, args: Any?) -> VoiceAction? {
        var a: [String: Any] = [:]
        if let s = args as? String, let d = s.data(using: .utf8), let o = try? JSONSerialization.jsonObject(with: d) as? [String: Any] {
            a = o
        } else if let o = args as? [String: Any] {
            a = o
        }
        func num(_ k: String) -> Double? {
            if let v = a[k] as? Double { return v }
            if let v = a[k] as? Int { return Double(v) }
            if let v = a[k] as? NSNumber { return v.doubleValue }
            if let v = a[k] as? String { return Double(v) }
            return nil
        }
        let action: VoiceAction?
        switch name {
        case "stop": action = .stop
        case "move": action = num("distance_m").map { .move(meters: $0) }
        case "turn": action = num("degrees").map { .turn(degrees: $0) }
        case "go_to": action = (a["target"] as? String).map { .goTo(target: $0) }
        case "go_to_point":
            if let f = num("forward_m") { action = .goToPoint(forward: f, left: num("left_m") ?? 0) } else { action = nil }
        case "set_mode":
            if let m = a["mode"] as? String, ["guide", "auto", "manual"].contains(m) { action = .setMode(m) } else { action = nil }
        case "clear_goal": action = .clearGoal
        case "save_place": action = (a["name"] as? String).map { .savePlace($0.lowercased()) }
        case "get_status": action = .status
        default: action = nil
        }
        return action.map(VoiceLimits.clamp)
    }

    struct ToolCall: Equatable {
        var id: String
        var name: String
        var arguments: String
    }

    /// Parses an OpenAI chat-completions response: the assistant text and any tool calls.
    static func parseChatResponse(_ data: Data) -> (text: String, calls: [ToolCall])? {
        guard let o = try? JSONSerialization.jsonObject(with: data) as? [String: Any] else { return nil }
        var msg: [String: Any]?
        if let choices = o["choices"] as? [[String: Any]], let first = choices.first {
            msg = first["message"] as? [String: Any]
        } else if let m = o["message"] as? [String: Any] {   // Ollama /api/chat
            msg = m
        }
        guard let m = msg else { return nil }
        let text = (m["content"] as? String) ?? ""
        var calls: [ToolCall] = []
        for (i, c) in ((m["tool_calls"] as? [[String: Any]]) ?? []).enumerated() {
            guard let f = c["function"] as? [String: Any], let name = f["name"] as? String else { continue }
            var args = "{}"
            if let s = f["arguments"] as? String {
                args = s
            } else if let o = f["arguments"], JSONSerialization.isValidJSONObject(o),
                      let d = try? JSONSerialization.data(withJSONObject: o) {
                args = String(decoding: d, as: UTF8.self)
            }
            calls.append(ToolCall(id: (c["id"] as? String) ?? "call_\(i)", name: name, arguments: args))
        }
        return (text, calls)
    }

    /// For models without tool calling (and Apple's on-device model): finds the first JSON object in the
    /// reply and reads {"actions":[{"tool":...}], "say":"..."}.
    static func parseJSONReply(_ text: String) -> (actions: [VoiceAction], say: String)? {
        guard let start = text.firstIndex(of: "{") else { return nil }
        var depth = 0
        var inString = false
        var escaped = false
        var end: String.Index?
        var i = start
        while i < text.endIndex {
            let c = text[i]
            if inString {
                if escaped { escaped = false } else if c == "\\" { escaped = true } else if c == "\"" { inString = false }
            } else if c == "\"" {
                inString = true
            } else if c == "{" {
                depth += 1
            } else if c == "}" {
                depth -= 1
                if depth == 0 { end = i; break }
            }
            i = text.index(after: i)
        }
        guard let e = end, let d = String(text[start...e]).data(using: .utf8),
              let o = try? JSONSerialization.jsonObject(with: d) as? [String: Any] else { return nil }
        var actions: [VoiceAction] = []
        for a in (o["actions"] as? [[String: Any]]) ?? [] {
            guard let name = (a["tool"] as? String) ?? (a["name"] as? String) else { continue }
            var args = a
            if let inner = a["arguments"] as? [String: Any] { args = inner }
            if let act = action(name: name, args: args) { actions.append(act) }
        }
        return (actions, (o["say"] as? String) ?? "")
    }
}

// MARK: - Wyoming protocol (Home Assistant's Whisper / Piper / openWakeWord servers)

/// One Wyoming event: a JSON header line, then `data_length` bytes of JSON data and `payload_length` bytes
/// of payload (audio). https://github.com/OHF-Voice/wyoming
struct WyomingEvent {
    var type: String
    var data: [String: Any] = [:]
    var payload = Data()

    func encode() -> Data {
        var header: [String: Any] = ["type": type, "version": "1.5.3"]
        var dataBytes = Data()
        if !data.isEmpty, JSONSerialization.isValidJSONObject(data),
           let d = try? JSONSerialization.data(withJSONObject: data, options: [.sortedKeys]) {
            dataBytes = d
            header["data_length"] = d.count
        }
        if !payload.isEmpty { header["payload_length"] = payload.count }
        var out = (try? JSONSerialization.data(withJSONObject: header, options: [.sortedKeys])) ?? Data()
        out.append(0x0A)
        out.append(dataBytes)
        out.append(payload)
        return out
    }

    static func audioFormat(rate: Int, width: Int = 2, channels: Int = 1) -> [String: Any] {
        ["rate": rate, "width": width, "channels": channels]
    }
}

/// Incremental Wyoming reader: feed bytes as they arrive, get complete events out.
final class WyomingDecoder {
    private var buf = Data()

    func feed(_ d: Data) -> [WyomingEvent] {
        buf.append(d)
        var out: [WyomingEvent] = []
        while true {
            guard let nl = buf.firstIndex(of: 0x0A) else { break }
            let headerData = buf[buf.startIndex..<nl]
            guard let header = try? JSONSerialization.jsonObject(with: Data(headerData)) as? [String: Any],
                  let type = header["type"] as? String else {
                buf.removeSubrange(buf.startIndex...nl)    // skip a garbled line
                continue
            }
            let dl = (header["data_length"] as? Int) ?? 0
            let pl = (header["payload_length"] as? Int) ?? 0
            let bodyStart = buf.index(after: nl)
            guard buf.distance(from: bodyStart, to: buf.endIndex) >= dl + pl else { break }
            var ev = WyomingEvent(type: type)
            if let inline = header["data"] as? [String: Any] { ev.data = inline }
            let dataEnd = buf.index(bodyStart, offsetBy: dl)
            if dl > 0, let extra = try? JSONSerialization.jsonObject(with: Data(buf[bodyStart..<dataEnd])) as? [String: Any] {
                ev.data.merge(extra) { _, new in new }
            }
            let payEnd = buf.index(dataEnd, offsetBy: pl)
            ev.payload = Data(buf[dataEnd..<payEnd])
            buf.removeSubrange(buf.startIndex..<payEnd)
            out.append(ev)
        }
        return out
    }
}

// MARK: - Voice activity detection

/// End-of-utterance detector for streaming 16-bit mono audio to a remote STT (the device recognisers do
/// their own). Adapts to the room's noise floor; speech = 3x the floor (and above a minimum level).
struct EnergyVAD {
    let rate: Int
    var silenceToEnd = 0.8         // s of quiet after speech that ends the utterance
    var maxUtterance = 10.0        // s
    var noSpeechTimeout = 6.0      // s before giving up if nobody speaks
    private(set) var floor = 200.0
    private(set) var speechStarted = false
    private(set) var elapsed = 0.0
    private var quiet = 0.0

    init(rate: Int) { self.rate = rate }

    enum Result { case waiting, speaking, done, timeout }

    /// Feed the next chunk of samples.
    mutating func feed(_ samples: [Int16]) -> Result {
        guard !samples.isEmpty else { return speechStarted ? .speaking : .waiting }
        var sum = 0.0
        for s in samples { let x = Double(s); sum += x * x }
        let rms = (sum / Double(samples.count)).squareRoot()
        let dt = Double(samples.count) / Double(rate)
        elapsed += dt
        let loud = rms > max(floor * 3, 400)
        if !loud { floor = 0.95 * floor + 0.05 * max(rms, 30) }
        if loud {
            speechStarted = true
            quiet = 0
        } else if speechStarted {
            quiet += dt
        }
        if speechStarted && quiet >= silenceToEnd { return .done }
        if elapsed >= maxUtterance { return speechStarted ? .done : .timeout }
        if !speechStarted && elapsed >= noSpeechTimeout { return .timeout }
        return speechStarted ? .speaking : .waiting
    }
}

// MARK: - WAV

enum WAV {
    static func encode(pcm16 pcm: Data, rate: Int, channels: Int = 1) -> Data {
        func u32(_ v: Int) -> Data { var x = UInt32(v).littleEndian; return Data(bytes: &x, count: 4) }
        func u16(_ v: Int) -> Data { var x = UInt16(v).littleEndian; return Data(bytes: &x, count: 2) }
        var d = Data("RIFF".utf8)
        d.append(u32(36 + pcm.count))
        d.append(Data("WAVEfmt ".utf8))
        d.append(u32(16)); d.append(u16(1)); d.append(u16(channels)); d.append(u32(rate))
        d.append(u32(rate * channels * 2)); d.append(u16(channels * 2)); d.append(u16(16))
        d.append(Data("data".utf8))
        d.append(u32(pcm.count))
        d.append(pcm)
        return d
    }

    static func samples(_ pcm: Data) -> [Int16] {
        var out = [Int16](repeating: 0, count: pcm.count / 2)
        pcm.withUnsafeBytes { raw in
            for i in 0..<out.count {
                out[i] = Int16(littleEndian: raw.loadUnaligned(fromByteOffset: i * 2, as: Int16.self))
            }
        }
        return out
    }

    static func data(_ samples: [Int16]) -> Data {
        var d = Data(capacity: samples.count * 2)
        for s in samples { var x = s.littleEndian; d.append(Data(bytes: &x, count: 2)) }
        return d
    }
}

// MARK: - Endpoints

enum VoiceURL {
    /// "192.168.1.5:10300" or "tcp://host:port" -> (host, port).
    static func hostPort(_ s: String, defaultPort: Int) -> (String, Int)? {
        var t = s.trimmingCharacters(in: .whitespaces)
        if let r = t.range(of: "://") { t = String(t[r.upperBound...]) }
        t = t.split(separator: "/").first.map(String.init) ?? t
        guard !t.isEmpty else { return nil }
        if let c = t.lastIndex(of: ":"), let p = Int(t[t.index(after: c)...]) {
            return (String(t[..<c]), p)
        }
        return (t, defaultPort)
    }

    /// Base URL for an OpenAI-compatible server: "http://host:11434" -> "http://host:11434/v1".
    static func openAIBase(_ s: String) -> String {
        var t = s.trimmingCharacters(in: .whitespaces)
        if !t.hasPrefix("http://") && !t.hasPrefix("https://") { t = "http://" + t }
        while t.hasSuffix("/") { t.removeLast() }
        if !t.hasSuffix("/v1") && !t.contains("/v1/") { t += "/v1" }
        return t
    }

    /// True for addresses that stay on the local network (used by the "private" check in settings).
    static func isLocal(_ s: String) -> Bool {
        guard let (h, _) = hostPort(s, defaultPort: 0) else { return false }
        let host = h.lowercased()
        if host == "localhost" || host.hasSuffix(".local") || host.hasSuffix(".lan") || host.hasSuffix(".home.arpa") { return true }
        let p = host.split(separator: ".").compactMap { Int($0) }
        guard p.count == 4 else { return !host.contains(".") }   // bare hostnames are LAN names
        return p[0] == 10 || p[0] == 127 || (p[0] == 192 && p[1] == 168) || (p[0] == 172 && (16...31).contains(p[1]))
            || (p[0] == 100 && (64...127).contains(p[1]))       // Tailscale / CGNAT
            || (p[0] == 169 && p[1] == 254)
    }
}
