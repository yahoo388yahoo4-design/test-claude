// Motor bridge for robot/receiver.py --backend serial: an Arduino (Uno, Nano, ...) next to the robot's
// computer drives an L298N / TB6612 from lines "M <left> <right>\n", each -1000..1000 (fraction of full
// power). Stops if no line arrives for 500 ms. Replies "OK" to each line.

static const int L_IN1 = 7, L_IN2 = 8, L_PWM = 5;    // PWM pins: 5 and 6 on an Uno
static const int R_IN1 = 9, R_IN2 = 10, R_PWM = 6;
static const unsigned long WATCHDOG_MS = 500;

unsigned long lastCmd = 0;
char buf[40];
uint8_t len = 0;

void motor(int in1, int in2, int pwm, int v) {
  v = constrain(v, -1000, 1000);
  digitalWrite(in1, v > 0 ? HIGH : LOW);
  digitalWrite(in2, v < 0 ? HIGH : LOW);
  analogWrite(pwm, (long)abs(v) * 255 / 1000);
}

void setup() {
  Serial.begin(115200);
  int pins[] = {L_IN1, L_IN2, L_PWM, R_IN1, R_IN2, R_PWM};
  for (int p : pins) pinMode(p, OUTPUT);
  motor(L_IN1, L_IN2, L_PWM, 0);
  motor(R_IN1, R_IN2, R_PWM, 0);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf[len] = 0;
      int l, r;
      if (sscanf(buf, "M %d %d", &l, &r) == 2) {
        motor(L_IN1, L_IN2, L_PWM, l);
        motor(R_IN1, R_IN2, R_PWM, r);
        lastCmd = millis();
        Serial.println("OK");
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
  if (millis() - lastCmd > WATCHDOG_MS) {
    motor(L_IN1, L_IN2, L_PWM, 0);
    motor(R_IN1, R_IN2, R_PWM, 0);
  }
}
