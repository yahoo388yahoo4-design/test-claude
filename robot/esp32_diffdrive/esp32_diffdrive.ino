// R2S Capture navigation mode: reference ESP32 receiver for a differential-drive base (robot/PROTOCOL.md).
//
// Transports (both on by default, same JSON messages):
//   * Wi-Fi WebSocket on port 8777. By default the ESP32 makes its own access point "R2S-Robot"
//     (password "r2srobot"); join it with the iPhone and use ws://192.168.4.1:8777/robot in the app.
//     Set WIFI_SSID to join your own network instead (the serial monitor prints the IP).
//   * Bluetooth LE Nordic UART Service, advertised as "R2S-Robot": newline-terminated JSON.
//
// Motor driver: TB6612FNG or L298N style (IN1 / IN2 direction + PWM enable per side). DRV8833-style
// drivers with two PWM inputs per motor: set DRIVER_TWO_PWM 1.
//
// Arduino IDE / arduino-cli: board "ESP32 Dev Module" (esp32 core 3.x), libraries "WebSockets" by
// Markus Sattler and "ArduinoJson" 7.x. Wi-Fi + BLE together need Tools > Partition Scheme > "Huge APP"
// (or set USE_BLE 0):
//   arduino-cli compile -b esp32:esp32:esp32:PartitionScheme=huge_app robot/esp32_diffdrive
// Verified to compile with esp32 core 3.3.12, WebSockets 2.7.2, ArduinoJson 7.4.3 (not yet run on a robot).
//
// Safety: a "vel" command is valid for WATCHDOG_MS; without a fresh one the motors stop. "move" and
// "turn" run open-loop from timing (no encoders); the app's default closed-loop mode does not need them.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

#define USE_BLE 1
#if USE_BLE
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#endif

// ---------- configuration ----------
static const char *ROBOT_NAME = "R2S-Robot";
static const char *WIFI_SSID = "";            // "" = soft access point
static const char *WIFI_PASS = "";
static const char *AP_PASS = "r2srobot";      // >= 8 chars
static const uint16_t WS_PORT = 8777;

static const float WHEEL_BASE = 0.30f;        // m, distance between the wheels
static const float MAX_WHEEL = 0.50f;         // m/s at 100% PWM (measure it: drive 1 s at full power)
static const float MAX_SPEED = 0.50f;         // cap on commanded forward speed, m/s
static const float MIN_DUTY = 0.12f;          // below this the motors stall; small commands are lifted to it
static const uint32_t WATCHDOG_MS = 500;
static const uint32_t STATUS_MS = 500;

#define DRIVER_TWO_PWM 0
// Left motor
static const int L_IN1 = 26, L_IN2 = 27, L_PWM = 25;
// Right motor
static const int R_IN1 = 14, R_IN2 = 12, R_PWM = 13;
static const int STBY = 33;                   // TB6612 standby pin (-1 if unused)
static const bool INVERT_LEFT = false, INVERT_RIGHT = false;
static const int BATTERY_ADC = -1;            // ADC pin through a divider, -1 = none
static const float BATTERY_DIVIDER = 4.0f;
static const int PWM_FREQ = 20000, PWM_BITS = 10;

// ---------- state ----------
WebSocketsServer ws(WS_PORT);
float curV = 0, curW = 0, curL = 0, curR = 0;
uint32_t velExpires = 0;      // millis() when the last vel expires (0 = none)
uint32_t jobEnd = 0;          // millis() when a timed move / turn ends (0 = none)
long jobSeq = -1;
bool estop = false;
uint32_t lastStatus = 0;

// ---------- motors ----------
static void motorWrite(int in1, int in2, int pwm, float frac, bool invert) {
  if (invert) frac = -frac;
  frac = constrain(frac, -1.0f, 1.0f);
  const int maxDuty = (1 << PWM_BITS) - 1;
  float mag = fabsf(frac);
  if (mag > 0.01f && mag < MIN_DUTY) mag = MIN_DUTY;
  int duty = (int)(mag * maxDuty);
#if DRIVER_TWO_PWM
  // in1 / in2 are both PWM pins (DRV8833): drive one, hold the other low.
  if (frac > 0.01f) { ledcWrite(in1, duty); ledcWrite(in2, 0); }
  else if (frac < -0.01f) { ledcWrite(in1, 0); ledcWrite(in2, duty); }
  else { ledcWrite(in1, 0); ledcWrite(in2, 0); }
  (void)pwm;
#else
  if (frac > 0.01f) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (frac < -0.01f) { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }
  else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); duty = 0; }
  ledcWrite(pwm, duty);
#endif
}

// Body velocity (m/s, rad/s, + = left) -> wheel speeds, scaled together if one saturates.
static void drive(float v, float w) {
  if (estop) { v = 0; w = 0; }
  v = constrain(v, -MAX_SPEED, MAX_SPEED);
  float l = v - w * WHEEL_BASE / 2, r = v + w * WHEEL_BASE / 2;
  float peak = max(fabsf(l), fabsf(r));
  if (peak > MAX_WHEEL) { l *= MAX_WHEEL / peak; r *= MAX_WHEEL / peak; }
  curV = v; curW = w; curL = l; curR = r;
  motorWrite(L_IN1, L_IN2, L_PWM, l / MAX_WHEEL, INVERT_LEFT);
  motorWrite(R_IN1, R_IN2, R_PWM, r / MAX_WHEEL, INVERT_RIGHT);
}

// ---------- protocol ----------
static void sendAll(const JsonDocument &doc);

static void sendDone(long seq) {
  JsonDocument d;
  d["type"] = "done";
  d["seq"] = seq;
  sendAll(d);
}

static void handleMessage(const char *text, size_t len) {
  JsonDocument msg;
  if (deserializeJson(msg, text, len)) {
    JsonDocument e; e["type"] = "error"; e["error"] = "bad json"; sendAll(e);
    return;
  }
  const char *type = msg["type"] | "";
  long seq = msg["seq"] | -1L;
  if (!strcmp(type, "hello")) {
    JsonDocument r;
    r["type"] = "hello"; r["proto"] = 1; r["name"] = ROBOT_NAME;
    JsonArray caps = r["caps"].to<JsonArray>();
    caps.add("vel"); caps.add("move"); caps.add("turn"); caps.add("stop"); caps.add("status");
    r["wheel_base"] = WHEEL_BASE; r["max_wheel"] = MAX_WHEEL;
    sendAll(r);
  } else if (!strcmp(type, "ping")) {
    JsonDocument r;
    r["type"] = "pong"; r["seq"] = seq; r["t"] = msg["t"];
    sendAll(r);
  } else if (!strcmp(type, "vel")) {
    jobEnd = 0;
    drive(msg["v"] | 0.0f, msg["w"] | 0.0f);
    velExpires = millis() + WATCHDOG_MS;
  } else if (!strcmp(type, "stop")) {
    jobEnd = 0; velExpires = 0;
    drive(0, 0);
  } else if (!strcmp(type, "move")) {
    float dist = (msg["dist_cm"] | 0.0f) / 100.0f;
    float speed = fabsf(msg["speed_cms"] | 20.0f) / 100.0f;
    if (speed < 0.01f) speed = 0.2f;
    speed = min(speed, MAX_SPEED);
    drive(dist < 0 ? -speed : speed, 0);
    jobEnd = millis() + (uint32_t)(fabsf(dist) / speed * 1000.0f);
    jobSeq = seq; velExpires = 0;
  } else if (!strcmp(type, "turn")) {
    float ang = (msg["deg"] | 0.0f) * PI / 180.0f;
    float rate = fabsf(msg["speed_dps"] | 45.0f) * PI / 180.0f;
    if (rate < 0.05f) rate = PI / 4;
    drive(0, ang < 0 ? -rate : rate);
    jobEnd = millis() + (uint32_t)(fabsf(ang) / rate * 1000.0f);
    jobSeq = seq; velExpires = 0;
  } else if (!strcmp(type, "estop")) {
    estop = msg["on"] | true;
    jobEnd = 0;
    drive(0, 0);
  } else {
    JsonDocument e; e["type"] = "error"; e["seq"] = seq; e["error"] = "unknown type"; sendAll(e);
  }
}

// ---------- WebSocket ----------
static void onWs(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: Serial.printf("ws client %u connected\n", num); break;
    case WStype_DISCONNECTED:
      Serial.printf("ws client %u gone, stopping\n", num);
      jobEnd = 0; velExpires = 0; drive(0, 0);
      break;
    case WStype_TEXT: handleMessage((const char *)payload, length); break;
    default: break;
  }
}

// ---------- BLE Nordic UART ----------
#if USE_BLE
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // phone writes
#define NUS_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // robot notifies
BLECharacteristic *txChar = nullptr;
bool bleConnected = false;
String bleInbox;
String bleQueue;   // complete lines received in the BLE callback, handled in loop()
portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }
  void onDisconnect(BLEServer *s) override {
    bleConnected = false;
    jobEnd = 0; velExpires = 0; drive(0, 0);
    s->getAdvertising()->start();
  }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    portENTER_CRITICAL(&bleMux);
    bleInbox += v;
    int nl;
    while ((nl = bleInbox.indexOf('\n')) >= 0) {
      bleQueue += bleInbox.substring(0, nl + 1);
      bleInbox.remove(0, nl + 1);
    }
    if (bleInbox.length() > 2048) bleInbox = "";
    portEXIT_CRITICAL(&bleMux);
  }
};

static void bleSetup() {
  BLEDevice::init(ROBOT_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

static void bleSend(const String &line) {
  if (!bleConnected || !txChar) return;
  String s = line + "\n";
  // Notifications are limited by the MTU; send in 180-byte pieces (the phone re-joins lines).
  for (size_t i = 0; i < s.length(); i += 180) {
    String part = s.substring(i, min(s.length(), i + 180));
    txChar->setValue((uint8_t *)part.c_str(), part.length());
    txChar->notify();
  }
}
#endif

static void sendAll(const JsonDocument &doc) {
  String out;
  serializeJson(doc, out);
  ws.broadcastTXT(out);
#if USE_BLE
  bleSend(out);
#endif
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
#if DRIVER_TWO_PWM
  ledcAttach(L_IN1, PWM_FREQ, PWM_BITS); ledcAttach(L_IN2, PWM_FREQ, PWM_BITS);
  ledcAttach(R_IN1, PWM_FREQ, PWM_BITS); ledcAttach(R_IN2, PWM_FREQ, PWM_BITS);
#else
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  ledcAttach(L_PWM, PWM_FREQ, PWM_BITS);
  ledcAttach(R_PWM, PWM_FREQ, PWM_BITS);
#endif
  if (STBY >= 0) { pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH); }
  drive(0, 0);

  if (strlen(WIFI_SSID) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("joining Wi-Fi");
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print('.'); }
    Serial.println();
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("ws://%s:%u/robot\n", WiFi.localIP().toString().c_str(), WS_PORT);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ROBOT_NAME, AP_PASS);
    Serial.printf("access point %s / %s -> ws://%s:%u/robot\n", ROBOT_NAME, AP_PASS,
                  WiFi.softAPIP().toString().c_str(), WS_PORT);
  }
  ws.begin();
  ws.onEvent(onWs);
#if USE_BLE
  bleSetup();
  Serial.println("BLE UART advertising as R2S-Robot");
#endif
}

void loop() {
  ws.loop();
#if USE_BLE
  String lines;
  portENTER_CRITICAL(&bleMux);
  lines = bleQueue;
  bleQueue = "";
  portEXIT_CRITICAL(&bleMux);
  int start = 0, nl;
  while ((nl = lines.indexOf('\n', start)) >= 0) {
    String line = lines.substring(start, nl);
    if (line.length()) handleMessage(line.c_str(), line.length());
    start = nl + 1;
  }
#endif
  uint32_t now = millis();
  if (jobEnd && (int32_t)(now - jobEnd) >= 0) {
    jobEnd = 0;
    drive(0, 0);
    sendDone(jobSeq);
  }
  if (!jobEnd && velExpires && (int32_t)(now - velExpires) >= 0) {
    velExpires = 0;
    drive(0, 0);   // watchdog
  }
  if (now - lastStatus >= STATUS_MS) {
    lastStatus = now;
    JsonDocument s;
    s["type"] = "status";
    s["v"] = curV; s["w"] = curW; s["left"] = curL; s["right"] = curR;
    s["busy"] = jobEnd != 0; s["estop"] = estop;
    if (BATTERY_ADC >= 0) s["battery_v"] = analogReadMilliVolts(BATTERY_ADC) / 1000.0f * BATTERY_DIVIDER;
    sendAll(s);
  }
}
