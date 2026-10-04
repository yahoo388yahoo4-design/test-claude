import Foundation

// Tests for ios/R2SCapture/Voice/VoiceCore.swift (local sentence matcher, LLM tool parsing, Wyoming
// framing, VAD, WAV). Run: ios/NavTests/run_tests.sh

var failures = 0
func check(_ cond: Bool, _ msg: String, line: Int = #line) {
    if cond { print("  ok  \(msg)") } else { failures += 1; print("FAIL  \(msg)  (line \(line))") }
}

let labels = ["table", "chair 2", "sofa", "kitchen counter", "door"]
func p(_ s: String) -> [VoiceAction]? { IntentParser.parse(s, labels: labels) }

print("local intents")
check(p("Stop!") == [.stop], "stop")
check(p("hey robot, freeze") == [.stop], "freeze with wake phrase")
check(p("go to the table and stop") == [.stop], "a stop anywhere wins")
check(!IntentParser.containsStop("please go to the st"), "partial without a stop word")
check(IntentParser.containsStop("go forward no stop"), "stop in a partial transcript")
check(p("move forward 1.5 meters") == [.move(meters: 1.5)], "move 1.5 m")
check(p("go forward fifty centimeters") == [.move(meters: 0.5)], "fifty centimeters")
check(p("drive ahead two feet") == [.move(meters: 0.6096)], "two feet")
check(p("go back a little") == [.move(meters: -0.2)], "back a little")
check(p("move forward 50") == [.move(meters: 0.5)], "bare 50 = cm")
check(p("move forward one and a half meters") == [.move(meters: 1.5)], "one and a half meters")
check(p("turn left") == [.turn(degrees: 90)], "turn left = 90")
check(p("turn right forty five degrees") == [.turn(degrees: -45)], "right 45")
check(p("rotate left 30°") == [.turn(degrees: 30)], "30° symbol")
check(p("turn around") == [.turn(degrees: 180)], "turn around")
check(p("turn slightly right") == [.turn(degrees: -15)], "slightly right")
check(p("go to the kitchen counter") == [.goTo(target: "kitchen counter")], "go to kitchen counter")
check(p("take me to the chair") == [.goTo(target: "chair 2")], "chair -> chair 2")
check(p("navigate to sofas") == [.goTo(target: "sofa")], "plural")
check(p("go to the garage") == nil, "unknown place -> LLM")
check(p("move forward 1 meter then turn left and go to the door") == [.move(meters: 1), .turn(degrees: 90), .goTo(target: "door")], "three steps")
check(p("where are you") == [.status], "status")
check(p("switch to auto mode") == [.setMode("auto")], "auto mode")
check(p("cancel") == [.clearGoal], "cancel")
check(p("remember this place as the kitchen") == [.savePlace("kitchen")], "save place")
check(p("call this spot charging dock") == [.savePlace("charging dock")], "call this spot")
check(VoiceTools.action(name: "save_place", args: #"{"name":"Desk"}"#) == .savePlace("desk"), "save_place tool")
check(p("what's the meaning of life") == nil, "chit-chat -> LLM")
check(p("") == nil, "empty")

print("tool calls")
let resp = """
{"choices":[{"message":{"role":"assistant","content":"On my way.","tool_calls":[
 {"id":"c1","type":"function","function":{"name":"move","arguments":"{\\"distance_m\\": 12}"}},
 {"id":"c2","type":"function","function":{"name":"turn","arguments":{"degrees":-90}}},
 {"id":"c3","type":"function","function":{"name":"go_to","arguments":"{\\"target\\":\\"sofa\\"}"}}]}}]}
"""
if let r = VoiceTools.parseChatResponse(Data(resp.utf8)) {
    check(r.text == "On my way.", "assistant text")
    check(r.calls.count == 3, "three tool calls")
    let acts = r.calls.compactMap { VoiceTools.action(name: $0.name, args: $0.arguments) }
    check(acts == [.move(meters: 5), .turn(degrees: -90), .goTo(target: "sofa")], "actions, move clamped to 5 m: \(acts)")
} else { check(false, "parse chat response") }
let ollama = #"{"message":{"role":"assistant","content":"","tool_calls":[{"function":{"name":"stop","arguments":{}}}]}}"#
check(VoiceTools.parseChatResponse(Data(ollama.utf8))?.calls.first?.name == "stop", "Ollama /api/chat shape")
check(VoiceTools.action(name: "set_mode", args: #"{"mode":"warp"}"#) == nil, "bad enum rejected")
check(VoiceTools.action(name: "rm_rf", args: nil) == nil, "unknown tool rejected")
let jr = VoiceTools.parseJSONReply("Sure! {\"actions\":[{\"tool\":\"turn\",\"degrees\":45},{\"tool\":\"move\",\"arguments\":{\"distance_m\":-1}}],\"say\":\"Turning {left}.\"} done")
check(jr?.actions == [.turn(degrees: 45), .move(meters: -1)] && jr?.say == "Turning {left}.", "JSON reply with braces in strings")
check(VoiceTools.parseJSONReply("no json here") == nil, "no JSON")
check(JSONSerialization.isValidJSONObject(["tools": VoiceTools.openAITools]), "tools schema is valid JSON")

print("wyoming")
let ev = WyomingEvent(type: "audio-chunk", data: WyomingEvent.audioFormat(rate: 16000), payload: Data([1, 2, 3, 4]))
let bytes = ev.encode()
let dec = WyomingDecoder()
var got: [WyomingEvent] = []
for b in bytes { got += dec.feed(Data([b])) }            // byte by byte
check(got.count == 1 && got[0].type == "audio-chunk" && got[0].payload == Data([1, 2, 3, 4]) && (got[0].data["rate"] as? Int) == 16000, "round trip byte by byte")
let two = WyomingEvent(type: "transcript", data: ["text": "go forward"]).encode() + WyomingEvent(type: "audio-stop").encode()
let evs = WyomingDecoder().feed(two)
check(evs.map(\.type) == ["transcript", "audio-stop"] && (evs[0].data["text"] as? String) == "go forward", "two events in one read")
let inline = Data(#"{"type":"transcript","data":{"text":"hi"}}"#.utf8) + Data([0x0A])
check((WyomingDecoder().feed(inline).first?.data["text"] as? String) == "hi", "inline data (older servers)")

print("vad")
var vad = EnergyVAD(rate: 16000)
var r = EnergyVAD.Result.waiting
let quiet = [Int16](repeating: 50, count: 480)
let loud = (0..<480).map { Int16(4000 * sin(Double($0) * 0.3)) }
for _ in 0..<20 { r = vad.feed(quiet) }
check(r == .waiting, "quiet room: waiting")
for _ in 0..<20 { r = vad.feed(loud) }
check(r == .speaking, "speech detected")
var n = 0
while r == .speaking && n < 100 { r = vad.feed(quiet); n += 1 }
check(r == .done && n >= 25 && n <= 28, "ends after ~0.8 s of quiet (\(n) chunks of 30 ms)")
var v2 = EnergyVAD(rate: 16000)
var r2 = EnergyVAD.Result.waiting
for _ in 0..<250 { r2 = v2.feed(quiet); if r2 != .waiting { break } }
check(r2 == .timeout, "nobody speaks -> timeout")

print("wav and urls")
let pcm = WAV.data([0, 1000, -1000, 32767])
check(WAV.samples(pcm) == [0, 1000, -1000, 32767], "pcm round trip")
check(WAV.encode(pcm16: pcm, rate: 16000).count == 44 + 8, "wav header")
check(VoiceURL.hostPort("tcp://192.168.1.5:10300", defaultPort: 1)! == ("192.168.1.5", 10300), "tcp url")
check(VoiceURL.hostPort("piper.local", defaultPort: 10200)! == ("piper.local", 10200), "default port")
check(VoiceURL.openAIBase("192.168.1.5:11434") == "http://192.168.1.5:11434/v1", "ollama base")
check(VoiceURL.openAIBase("https://api.example.com/v1/") == "https://api.example.com/v1", "keeps /v1")
check(VoiceURL.isLocal("http://192.168.1.5:11434") && VoiceURL.isLocal("mini.local:8000") && VoiceURL.isLocal("100.101.1.2"), "LAN / Tailscale = local")
check(!VoiceURL.isLocal("https://api.openai.com/v1"), "cloud is not local")

print(failures == 0 ? "ALL VOICE TESTS PASSED" : "\(failures) VOICE TEST(S) FAILED")
exit(failures == 0 ? 0 : 1)
