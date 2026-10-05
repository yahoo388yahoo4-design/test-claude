package com.real2sim.capture.voice

import org.json.JSONArray
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt

// Platform-independent part of voice control, the same logic as the iPhone app's
// ios/R2SCapture/Voice/VoiceCore.swift (keep them in step): actions, the built-in sentence matcher that
// handles common commands without an LLM (like Home Assistant's "prefer handling commands locally"), LLM
// tool definitions and response parsing, Wyoming framing (Home Assistant's Whisper / Piper servers), an
// energy voice-activity detector and WAV helpers.

/** Distances in metres (+ forward), angles in degrees (+ left), as in robot/PROTOCOL.md. */
sealed class VoiceAction {
    object Stop : VoiceAction()
    data class Move(val meters: Double) : VoiceAction()
    data class Turn(val degrees: Double) : VoiceAction()
    data class GoTo(val target: String) : VoiceAction()
    data class GoToPoint(val forward: Double, val left: Double) : VoiceAction()
    data class SetMode(val mode: String) : VoiceAction()
    object ClearGoal : VoiceAction()
    data class SavePlace(val name: String) : VoiceAction()
    object Status : VoiceAction()
    data class Say(val text: String) : VoiceAction()

    val isMotion get() = this is Move || this is Turn || this is GoTo || this is GoToPoint

    val summary: String get() = when (this) {
        Stop -> "stop"
        is Move -> "move %.2f m".format(meters)
        is Turn -> "turn %.0f°".format(degrees)
        is GoTo -> "go to $target"
        is GoToPoint -> "go to point %.1f m ahead, %.1f m left".format(forward, left)
        is SetMode -> "$mode mode"
        ClearGoal -> "clear goal"
        is SavePlace -> "remember $name"
        Status -> "status"
        is Say -> "say \"$text\""
    }

    override fun toString() = summary
}

object VoiceLimits {
    const val MAX_MOVE = 5.0
    const val MAX_TURN = 360.0
    const val MAX_POINT = 10.0
    private fun c(x: Double, m: Double) = if (x.isFinite()) x.coerceIn(-m, m) else 0.0
    fun clamp(a: VoiceAction): VoiceAction = when (a) {
        is VoiceAction.Move -> VoiceAction.Move(c(a.meters, MAX_MOVE))
        is VoiceAction.Turn -> VoiceAction.Turn(c(a.degrees, MAX_TURN))
        is VoiceAction.GoToPoint -> VoiceAction.GoToPoint(c(a.forward, MAX_POINT), c(a.left, MAX_POINT))
        else -> a
    }
}

/** Understands the common commands (English) with no network or model; null = hand it to the LLM. */
object IntentParser {
    val stopWords = setOf("stop", "halt", "freeze", "emergency", "abort", "whoa")

    /** Checked on every partial transcript, so the robot stops before the sentence ends. */
    fun containsStop(text: String) = words(text).any { it in stopWords }

    fun parse(text: String, labels: List<String> = emptyList()): List<VoiceAction>? {
        val t = normalise(text)
        if (t.isEmpty()) return null
        if (containsStop(t)) return listOf(VoiceAction.Stop)
        val out = ArrayList<VoiceAction>()
        for (p in splitSteps(t)) out.add(parseOne(p, labels) ?: return null)
        return out.ifEmpty { null }
    }

    private val verbs = setOf("go", "move", "drive", "turn", "rotate", "spin", "come", "back", "head", "stop", "navigate", "take", "find")

    fun splitSteps(t: String): List<String> {
        val steps = arrayListOf(ArrayList<String>())
        val w = t.split(" ").filter { it.isNotEmpty() }
        var i = 0
        while (i < w.size) {
            val next = w.getOrNull(i + 1)
            if (w[i] == "then" || (w[i] == "and" && next != null && (next == "then" || next in verbs))) {
                if (steps.last().isNotEmpty()) steps.add(ArrayList())
                if (w[i] == "and" && next == "then") i++
            } else steps.last().add(w[i])
            i++
        }
        return steps.filter { it.isNotEmpty() }.map { it.joinToString(" ") }
    }

    private val placeLeads = listOf("remember this place as", "remember this spot as", "remember this position as", "remember this as",
        "save this place as", "save this spot as", "save this position as", "call this place", "call this spot",
        "mark this place as", "mark this spot as", "this place is", "this is the")
    private val goLeads = listOf("go to", "drive to", "navigate to", "take me to", "head to", "move to", "go over to", "bring me to", "find")

    fun parseOne(t: String, labels: List<String>): VoiceAction? {
        val w = words(t)
        fun has(s: String) = s in w
        for (lead in placeLeads) if (t.startsWith("$lead ")) {
            val name = stripArticles(t.substring(lead.length + 1))
            return if (name.isEmpty()) null else VoiceAction.SavePlace(name)
        }
        if (t.startsWith("cancel") || t.contains("clear the goal") || t.contains("clear goal") || t == "never mind") return VoiceAction.ClearGoal
        for (m in listOf("guide", "auto", "manual")) if ((has(m) || (m == "auto" && has("automatic"))) && has("mode")) return VoiceAction.SetMode(m)
        if (t.contains("where are you") || t.contains("where am i") || has("status") || has("battery") || t.contains("how far") ||
            t.contains("what do you see") || t.contains("what is around") || t.contains("what's around") || t.contains("are you connected")) return VoiceAction.Status
        for (lead in goLeads) {
            val idx = when {
                t.startsWith("$lead ") -> 0
                t.contains(" $lead ") -> t.indexOf(" $lead ") + 1
                else -> -1
            }
            if (idx >= 0) {
                val target = stripArticles(t.substring(idx + lead.length + 1))
                if (target.isEmpty()) return null
                return matchLabel(target, labels)?.let { VoiceAction.GoTo(it) }
            }
        }
        if (t.contains("turn around") || t.contains("about face")) return VoiceAction.Turn(180.0)
        if (has("turn") || has("rotate") || has("spin")) {
            val left = has("left") || has("counterclockwise") || has("anticlockwise")
            val right = has("right") || has("clockwise")
            if (left == right) return null
            var deg = number(w) ?: if (has("little") || has("bit") || has("slightly")) 15.0 else 90.0
            if (has("half") && !has("degrees")) deg = 180.0
            return VoiceAction.Turn(if (left) deg else -deg)
        }
        val back = has("back") || has("backward") || has("backwards") || has("reverse")
        val fwd = has("forward") || has("forwards") || has("ahead") || has("straight")
        if ((back || fwd)) {
            var m = distance(w) ?: if (has("little") || has("bit")) 0.2 else 0.5
            if (back) m = -abs(m)
            return VoiceAction.Move(m)
        }
        return null
    }

    fun normalise(s: String): String {
        val chars = s.lowercase().replace('’', '\'').toCharArray()
        val res = StringBuilder()
        for ((i, c) in chars.withIndex()) {
            when {
                c == '.' && i > 0 && i + 1 < chars.size && chars[i - 1].isDigit() && chars[i + 1].isDigit() -> res.append(c)
                c == '°' -> res.append(" degrees ")
                c.isLetterOrDigit() || c == '\'' -> res.append(c)
                else -> res.append(' ')
            }
        }
        val w = res.split(" ").filter { it.isNotEmpty() && it != "please" }.toMutableList()
        while (w.isNotEmpty() && w[0] in setOf("hey", "ok", "okay", "robot")) w.removeAt(0)
        return w.joinToString(" ")
    }

    fun words(s: String) = s.lowercase().split(Regex("[^\\p{L}\\p{N}.']+")).filter { it.isNotEmpty() }

    fun stripArticles(s: String): String {
        var w = words(s)
        while (w.isNotEmpty() && w[0] in setOf("the", "a", "an", "my", "that", "this")) w = w.drop(1)
        val i = w.indexOfFirst { it in setOf("please", "now", "then") }
        if (i >= 0) w = w.take(i)
        return w.joinToString(" ")
    }

    private val numberWords = mapOf(
        "zero" to 0.0, "one" to 1.0, "a" to 1.0, "an" to 1.0, "two" to 2.0, "three" to 3.0, "four" to 4.0, "five" to 5.0,
        "six" to 6.0, "seven" to 7.0, "eight" to 8.0, "nine" to 9.0, "ten" to 10.0, "eleven" to 11.0, "twelve" to 12.0,
        "fifteen" to 15.0, "twenty" to 20.0, "thirty" to 30.0, "forty" to 40.0, "fourty" to 40.0, "fifty" to 50.0,
        "sixty" to 60.0, "seventy" to 70.0, "eighty" to 80.0, "ninety" to 90.0, "hundred" to 100.0, "half" to 0.5,
        "quarter" to 0.25, "twice" to 2.0)
    private val unitWords = setOf("m", "meter", "meters", "metre", "metres", "cm", "centimeter", "centimeters", "centimetre",
        "centimetres", "foot", "feet", "ft", "inch", "inches", "step", "steps")

    fun number(w: List<String>): Double? {
        for (i in w.indices) {
            w[i].toDoubleOrNull()?.let { return it }
            val v = numberWords[w[i]] ?: continue
            val nxt = w.getOrNull(i + 1)
            if (w[i] in setOf("a", "an") && nxt != null && numberWords[nxt] == null && nxt !in unitWords) continue
            if (w[i] in setOf("a", "an") && nxt == null) continue
            var total = v
            var j = i + 1
            val v2 = w.getOrNull(j)?.let { numberWords[it] }
            if (v2 != null && v >= 20 && v2 < 10 && v2 >= 1) { total += v2; j++ }
            if (w.getOrNull(j) == "hundred") { total *= 100; j++ }
            if (w.getOrNull(j) == "and" && w.getOrNull(j + 1) in setOf("a", "one") && w.getOrNull(j + 2) == "half") total += 0.5
            if (w[i] in setOf("a", "an") && w.getOrNull(j) == "half") total = 0.5
            if (w[i] in setOf("a", "an") && w.getOrNull(j) == "quarter") total = 0.25
            return total
        }
        return null
    }

    fun distance(w: List<String>): Double? {
        val n = number(w) ?: return null
        return when {
            w.any { it in setOf("cm", "centimeter", "centimeters", "centimetre", "centimetres") } -> n / 100
            w.any { it in setOf("foot", "feet", "ft") } -> n * 0.3048
            w.any { it in setOf("inch", "inches") } -> n * 0.0254
            w.any { it in setOf("step", "steps") } -> n * 0.5
            w.any { it in setOf("m", "meter", "meters", "metre", "metres") } -> n
            else -> if (n > 5) n / 100 else n
        }
    }

    private fun singular(s: String) = if (s.length > 3 && s.endsWith("s")) s.dropLast(1) else s

    fun matchLabel(target: String, labels: List<String>): String? {
        val t = target.lowercase()
        labels.firstOrNull { it.lowercase() == t }?.let { return it }
        labels.firstOrNull { it.lowercase().contains(t) || t.contains(it.lowercase()) }?.let { return it }
        val tw = words(t).map(::singular).toSet()
        return labels.map { it to words(it).map(::singular).toSet().intersect(tw).size }.filter { it.second > 0 }.maxByOrNull { it.second }?.first
    }
}

object VoiceTools {
    private fun fn(name: String, desc: String, props: JSONObject = JSONObject(), required: List<String> = emptyList()) =
        JSONObject().put("type", "function").put("function", JSONObject().put("name", name).put("description", desc)
            .put("parameters", JSONObject().put("type", "object").put("properties", props).put("required", JSONArray(required))))
    private fun num(desc: String? = null) = JSONObject().put("type", "number").apply { desc?.let { put("description", it) } }

    /** OpenAI-compatible `tools` (Ollama, llama.cpp server, LM Studio, vLLM, LocalAI, OpenAI). */
    fun openAITools(): JSONArray = JSONArray(listOf(
        fn("stop", "Stop the robot immediately."),
        fn("move", "Drive straight. Positive = forward, negative = backward.", JSONObject().put("distance_m", num("metres, -5 to 5")), listOf("distance_m")),
        fn("turn", "Turn in place. Positive = left (counter-clockwise), negative = right.", JSONObject().put("degrees", num("-360 to 360")), listOf("degrees")),
        fn("go_to", "Plan a path to a known object or saved place and follow it.",
            JSONObject().put("target", JSONObject().put("type", "string").put("description", "one of the known names")), listOf("target")),
        fn("go_to_point", "Plan a path to a point relative to the robot.",
            JSONObject().put("forward_m", num()).put("left_m", num("positive = left")), listOf("forward_m", "left_m")),
        fn("set_mode", "Switch navigation mode: guide (voice and arrows only), auto (drive the robot along the path), manual.",
            JSONObject().put("mode", JSONObject().put("type", "string").put("enum", JSONArray(listOf("guide", "auto", "manual")))), listOf("mode")),
        fn("clear_goal", "Forget the current goal and stop following it."),
        fn("save_place", "Remember the robot's current position under a name, for go_to later.",
            JSONObject().put("name", JSONObject().put("type", "string")), listOf("name")),
        fn("get_status", "Get the robot's position, goal, obstacles, link and battery."),
    ))

    fun systemPrompt(state: String, labels: List<String>, jsonOnly: Boolean): String {
        var s = "You control a small wheeled robot with a phone as its sensor head, by voice. Units: metres and degrees; " +
            "positive turn = left. Only do what the user asked; ask back if a command is unclear or unsafe. " +
            "Prefer one short spoken reply (under 20 words, no markdown, no emoji).\n" +
            "Known objects and places: ${if (labels.isEmpty()) "none yet (load a room scan, or save places)" else labels.joinToString(", ")}.\n" +
            "Current state: $state"
        if (jsonOnly) s += "\nReply with ONLY a JSON object, no other text: " +
            "{\"actions\":[{\"tool\":\"move\",\"distance_m\":1.0}],\"say\":\"Moving one metre.\"}. " +
            "Tools: stop; move{distance_m}; turn{degrees}; go_to{target}; go_to_point{forward_m,left_m}; " +
            "set_mode{mode: guide|auto|manual}; clear_goal; save_place{name}; get_status. Use \"actions\":[] to only answer."
        return s
    }

    /** One tool call -> action; [args] is a JSON string (OpenAI) or a JSONObject (Ollama). */
    fun action(name: String, args: Any?): VoiceAction? {
        val a = when (args) {
            is JSONObject -> args
            is String -> try { JSONObject(args) } catch (_: Exception) { JSONObject() }
            else -> JSONObject()
        }
        fun n(k: String): Double? = if (a.has(k)) a.optDouble(k).takeIf { !it.isNaN() } else null
        val act = when (name) {
            "stop" -> VoiceAction.Stop
            "move" -> n("distance_m")?.let { VoiceAction.Move(it) }
            "turn" -> n("degrees")?.let { VoiceAction.Turn(it) }
            "go_to" -> a.optString("target").takeIf { it.isNotEmpty() }?.let { VoiceAction.GoTo(it) }
            "go_to_point" -> n("forward_m")?.let { VoiceAction.GoToPoint(it, n("left_m") ?: 0.0) }
            "set_mode" -> a.optString("mode").takeIf { it in setOf("guide", "auto", "manual") }?.let { VoiceAction.SetMode(it) }
            "clear_goal" -> VoiceAction.ClearGoal
            "save_place" -> a.optString("name").takeIf { it.isNotEmpty() }?.let { VoiceAction.SavePlace(it.lowercase()) }
            "get_status" -> VoiceAction.Status
            else -> null
        }
        return act?.let(VoiceLimits::clamp)
    }

    data class ToolCall(val id: String, val name: String, val arguments: String)

    /** OpenAI chat-completions (or Ollama /api/chat) response -> assistant text and tool calls. */
    fun parseChatResponse(body: String): Pair<String, List<ToolCall>>? {
        val o = try { JSONObject(body) } catch (_: Exception) { return null }
        val m = o.optJSONArray("choices")?.optJSONObject(0)?.optJSONObject("message") ?: o.optJSONObject("message") ?: return null
        val text = if (m.isNull("content")) "" else m.optString("content")
        val calls = ArrayList<ToolCall>()
        val tc = m.optJSONArray("tool_calls")
        if (tc != null) for (i in 0 until tc.length()) {
            val c = tc.optJSONObject(i) ?: continue
            val f = c.optJSONObject("function") ?: continue
            val name = f.optString("name").ifEmpty { continue }
            val args = when (val x = f.opt("arguments")) { is JSONObject -> x.toString(); is String -> x; else -> "{}" }
            calls.add(ToolCall(c.optString("id").ifEmpty { "call_$i" }, name, args))
        }
        return text to calls
    }

    /** For models without tool calling: first JSON object in the reply, {"actions":[{"tool":..}],"say":".."}. */
    fun parseJSONReply(text: String): Pair<List<VoiceAction>, String>? {
        val start = text.indexOf('{').takeIf { it >= 0 } ?: return null
        var depth = 0; var inStr = false; var esc = false; var end = -1
        for (i in start until text.length) {
            val c = text[i]
            if (inStr) { if (esc) esc = false else if (c == '\\') esc = true else if (c == '"') inStr = false }
            else if (c == '"') inStr = true
            else if (c == '{') depth++
            else if (c == '}') { depth--; if (depth == 0) { end = i; break } }
        }
        if (end < 0) return null
        val o = try { JSONObject(text.substring(start, end + 1)) } catch (_: Exception) { return null }
        val acts = ArrayList<VoiceAction>()
        val arr = o.optJSONArray("actions")
        if (arr != null) for (i in 0 until arr.length()) {
            val a = arr.optJSONObject(i) ?: continue
            val name = a.optString("tool").ifEmpty { a.optString("name") }.ifEmpty { continue }
            action(name, a.optJSONObject("arguments") ?: a)?.let(acts::add)
        }
        return acts to o.optString("say")
    }
}

/** Wyoming event (https://github.com/OHF-Voice/wyoming): JSON header line + data JSON + payload bytes. */
class WyomingEvent(val type: String, val data: JSONObject = JSONObject(), val payload: ByteArray = ByteArray(0)) {
    fun encode(): ByteArray {
        val header = JSONObject().put("type", type).put("version", "1.5.3")
        val dataBytes = if (data.length() > 0) data.toString().toByteArray() else ByteArray(0)
        if (dataBytes.isNotEmpty()) header.put("data_length", dataBytes.size)
        if (payload.isNotEmpty()) header.put("payload_length", payload.size)
        val out = ByteArrayOutputStream()
        out.write(header.toString().toByteArray()); out.write('\n'.code)
        out.write(dataBytes); out.write(payload)
        return out.toByteArray()
    }

    companion object {
        fun audioFormat(rate: Int, width: Int = 2, channels: Int = 1) = JSONObject().put("rate", rate).put("width", width).put("channels", channels)
    }
}

class WyomingDecoder {
    private var buf = ByteArray(0)

    fun feed(d: ByteArray): List<WyomingEvent> {
        buf += d
        val out = ArrayList<WyomingEvent>()
        while (true) {
            val nl = buf.indexOf('\n'.code.toByte()).takeIf { it >= 0 } ?: break
            val header = try { JSONObject(String(buf, 0, nl)) } catch (_: Exception) { buf = buf.copyOfRange(nl + 1, buf.size); continue }
            val type = header.optString("type").ifEmpty { buf = buf.copyOfRange(nl + 1, buf.size); continue }
            val dl = header.optInt("data_length", 0); val pl = header.optInt("payload_length", 0)
            if (buf.size - (nl + 1) < dl + pl) break
            val data = header.optJSONObject("data") ?: JSONObject()
            if (dl > 0) try {
                val extra = JSONObject(String(buf, nl + 1, dl))
                extra.keys().forEach { data.put(it, extra.get(it)) }
            } catch (_: Exception) {}
            val payload = buf.copyOfRange(nl + 1 + dl, nl + 1 + dl + pl)
            buf = buf.copyOfRange(nl + 1 + dl + pl, buf.size)
            out.add(WyomingEvent(type, data, payload))
        }
        return out
    }
}

/** End-of-utterance detector for 16-bit mono audio sent to a remote STT (same as the iPhone's). */
class EnergyVAD(private val rate: Int) {
    var silenceToEnd = 0.8; var maxUtterance = 10.0; var noSpeechTimeout = 6.0
    var floor = 200.0; private set
    var speechStarted = false; private set
    private var elapsed = 0.0; private var quiet = 0.0

    enum class Result { WAITING, SPEAKING, DONE, TIMEOUT }

    fun feed(samples: ShortArray, n: Int = samples.size): Result {
        if (n == 0) return if (speechStarted) Result.SPEAKING else Result.WAITING
        var sum = 0.0
        for (i in 0 until n) { val x = samples[i].toDouble(); sum += x * x }
        val rms = sqrt(sum / n)
        val dt = n.toDouble() / rate
        elapsed += dt
        val loud = rms > max(floor * 3, 400.0)
        if (!loud) floor = 0.95 * floor + 0.05 * max(rms, 30.0)
        if (loud) { speechStarted = true; quiet = 0.0 } else if (speechStarted) quiet += dt
        if (speechStarted && quiet >= silenceToEnd) return Result.DONE
        if (elapsed >= maxUtterance) return if (speechStarted) Result.DONE else Result.TIMEOUT
        if (!speechStarted && elapsed >= noSpeechTimeout) return Result.TIMEOUT
        return if (speechStarted) Result.SPEAKING else Result.WAITING
    }
}

object WAV {
    fun encode(pcm: ByteArray, rate: Int, channels: Int = 1): ByteArray {
        val b = ByteBuffer.allocate(44 + pcm.size).order(ByteOrder.LITTLE_ENDIAN)
        b.put("RIFF".toByteArray()).putInt(36 + pcm.size).put("WAVEfmt ".toByteArray())
        b.putInt(16).putShort(1).putShort(channels.toShort()).putInt(rate).putInt(rate * channels * 2)
        b.putShort((channels * 2).toShort()).putShort(16).put("data".toByteArray()).putInt(pcm.size).put(pcm)
        return b.array()
    }

    /** Parses a 16-bit PCM WAV: (rate, channels, pcm). */
    fun decode(wav: ByteArray): Triple<Int, Int, ByteArray>? {
        if (wav.size < 44 || String(wav, 0, 4) != "RIFF") return null
        val b = ByteBuffer.wrap(wav).order(ByteOrder.LITTLE_ENDIAN)
        var p = 12; var rate = 0; var ch = 1
        while (p + 8 <= wav.size) {
            val id = String(wav, p, 4); val len = b.getInt(p + 4)
            if (id == "fmt ") { ch = b.getShort(p + 10).toInt(); rate = b.getInt(p + 12) }
            if (id == "data") {
                // streamed WAVs (Piper HTTP) may carry 0 or 0xFFFFFFFF as the data length
                val end = if (len <= 0 || p + 8 + len > wav.size) wav.size else p + 8 + len
                return Triple(rate, ch, wav.copyOfRange(p + 8, end))
            }
            p += 8 + len + (len and 1)
        }
        return null
    }
}

object VoiceURL {
    fun hostPort(s: String, defaultPort: Int): Pair<String, Int>? {
        var t = s.trim().substringAfter("://")
        t = t.substringBefore('/')
        if (t.isEmpty()) return null
        val c = t.lastIndexOf(':')
        if (c > 0) t.substring(c + 1).toIntOrNull()?.let { return t.substring(0, c) to it }
        return t to defaultPort
    }

    fun openAIBase(s: String): String {
        var t = s.trim()
        if (!t.startsWith("http://") && !t.startsWith("https://")) t = "http://$t"
        t = t.trimEnd('/')
        if (!t.endsWith("/v1") && !t.contains("/v1/")) t += "/v1"
        return t
    }

    fun isLocal(s: String): Boolean {
        val h = hostPort(s, 0)?.first?.lowercase() ?: return false
        if (h == "localhost" || h.endsWith(".local") || h.endsWith(".lan") || h.endsWith(".home.arpa")) return true
        val p = h.split(".").mapNotNull { it.toIntOrNull() }
        if (p.size != 4) return !h.contains(".")
        return p[0] == 10 || p[0] == 127 || (p[0] == 192 && p[1] == 168) || (p[0] == 172 && p[1] in 16..31) ||
            (p[0] == 100 && p[1] in 64..127) || (p[0] == 169 && p[1] == 254)
    }
}
