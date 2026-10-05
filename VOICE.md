# Voice control for navigation mode (iPhone and Android)

Both apps' Nav screens have a round **mic** button. Tap it, say a command, and the robot does it; the
phone answers out loud. The design follows Home Assistant's voice assistant: speech to text, then a
built-in command matcher, then (only if that did not understand) a language model that calls the
same navigation actions as tools, then text to speech.

```
mic ─► speech to text ─► built-in commands ──(not understood)──► LLM with tools ─► actions ─► spoken reply
         │                       ▲
         └─ "stop" in the live transcript stops the robot at once, before any of this
```

**Stop never waits for a model.** Any of *stop, halt, freeze, abort, emergency, whoa* in the live
transcript stops the robot and cancels the goal, even mid-sentence.

## Commands

| Say | Does |
|---|---|
| stop / freeze / halt | stops at once |
| go forward 1 meter, back up 30 centimeters, move forward 2 feet, go back a little | timed or closed-loop `move` |
| turn left, turn right 45 degrees, rotate left a little, turn around | `turn` (default 90°, "a little" 15°) |
| remember this place as the kitchen | names the robot's current position for this session |
| go to the kitchen / table / door | goal = a place you named, or an object RoomPlan found in the loaded map; Auto mode drives there |
| go 2 meters ahead and 1 meter to the left (LLM) | goal relative to the robot |
| auto / guide / manual mode | switches the mode |
| cancel, clear the goal | drops the goal |
| where are you, status, battery | spoken status: pose, goal, distance left, obstacles, battery |

Join steps with *then* or *and*: "move forward 1 meter then turn left and go to the door". Limits:
5 m per move, 360° per turn, 10 m for a relative goal. Moves and turns need a connected robot;
without one, voice goals still work in Guide mode (spoken directions).

Anything else ("drive over by the couch and look around") goes to the language model, which sees the
robot's state (pose, mode, goal, front gap, battery) and the known place names, and answers with tool
calls: `stop`, `move`, `turn`, `go_to`, `go_to_point`, `set_mode`, `clear_goal`, `save_place`,
`get_status`. Unknown tools and out-of-range values are dropped.

## Engines (defaults first)

| | iPhone | Android | Your own server |
|---|---|---|---|
| Speech to text | Apple Speech, on-device when the language supports it | Google / the phone's recogniser; on-device recogniser on Android 12+ when "Prefer on-device" is on | Wyoming (wyoming-faster-whisper, Home Assistant's Whisper add-on, port 10300) or an OpenAI-compatible `/v1/audio/transcriptions` (speaches, LocalAI, whisper.cpp server) |
| Understanding | Apple Intelligence on-device model (iOS 26, supported iPhones); built-in commands otherwise | Gemini Nano through ML Kit GenAI (supported phones; downloads on first use); built-in commands otherwise | Ollama, llama.cpp, LM Studio, vLLM or any OpenAI-compatible `/v1/chat/completions`, with tool calls (or JSON replies for models without tools) |
| Voice reply | iPhone voices (AVSpeechSynthesizer) | Android TextToSpeech | Wyoming Piper (port 10200) or an OpenAI-compatible `/v1/audio/speech` (Kokoro-FastAPI, openedai-speech, LocalAI) |

Settings: iPhone, the gear next to the mic; Android, long-press the mic. Each part is chosen
separately, so e.g. phone speech recognition + Ollama + phone voice is fine.

## Fully private mode

**Fully private** keeps every byte on the phone or your own network:

* speech recognition on the phone is forced to on-device only (if the phone has no on-device model for
  the language, it says so instead of falling back to the cloud);
* any server address that is not on the local network is refused. Local means `localhost`, `*.local`,
  10.x, 172.16–31.x, 192.168.x and 100.64/10 (Tailscale), so a home server reached over Tailscale counts.

The on-phone default (Apple / Google on-device recognition, Apple Intelligence or Gemini Nano, phone
voice) is already private when the on-device models are available. With "Use my server" the phone
only does the microphone and speaker.

### One-computer local server (same pieces as Home Assistant's local voice)

On any Linux box or Mac on your network (a GPU helps the LLM, Whisper small runs fine on a CPU):

```bash
docker run -d --name whisper -p 10300:10300 -v whisper:/data rhasspy/wyoming-whisper \
    --model small-int8 --language en
docker run -d --name piper -p 10200:10200 -v piper:/data rhasspy/wyoming-piper \
    --voice en_US-lessac-medium
docker run -d --name ollama -p 11434:11434 -v ollama:/root/.ollama ollama/ollama   # add --gpus all on NVIDIA
docker exec ollama ollama pull qwen2.5:7b-instruct
```

Then in the app's voice settings type the computer's IP and tap **Use my server**: it sets Whisper
(Wyoming, 10300), Piper (Wyoming, 10200), Ollama (`http://<ip>:11434`, model `qwen2.5:7b-instruct`)
and turns on Fully private. Any model with tool calling works (qwen2.5, llama3.1/3.2, mistral-nemo);
for small models without tools, turn off **Tool calling** and the app asks for a JSON reply instead.

If Home Assistant already runs the Whisper and Piper add-ons, point the app at the Home Assistant
machine's IP on the same ports. OpenAI-compatible alternatives:

```bash
docker run -d -p 8000:8000 ghcr.io/speaches-ai/speaches:latest-cpu       # STT, model Systran/faster-whisper-small
docker run -d -p 8880:8880 ghcr.io/remsky/kokoro-fastapi-cpu:latest      # TTS, model kokoro, voice af_heart
```

A hosted OpenAI-compatible API also works when Fully private is off (put its key in **API key**; on
the iPhone it is kept in the keychain).

## Permissions

The first tap asks for the microphone and (iPhone) speech recognition. Neither app records audio
except while the mic button is lit, and no audio is stored.

## Code

* iPhone: `ios/R2SCapture/Voice/` (`VoiceCore.swift` holds the parser, tool schema, Wyoming codec and
  VAD and is tested by `ios/NavTests/voice/main.swift`).
* Android: `android/app/src/main/java/com/real2sim/capture/voice/` (`VoiceCore.kt` is the same logic,
  tested by `android/app/src/test/.../VoiceAndUsbRobotTest.kt`).

Not yet tried with real robots or on every phone; in particular Apple Intelligence and Gemini Nano
availability depends on the phone model, region and language.
