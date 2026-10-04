// Motor board for robot/receiver.py --backend serial (115200 baud).
//   in : "V <left_cms> <right_cms>\n"      wheel speed targets, cm/s (signed)
//   out: "E <left_ticks> <right_ticks>\n"  encoder counts, 20 Hz (if ENCODERS)
//        "B <volts>\n"                     battery, 1 Hz
// Works with an L298N / TB6612 style driver (PWM + 2 direction pins per motor). Stops if no "V"
// line arrives for 300 ms. Speed control is a feed-forward + PI loop on encoder speed, or plain
// feed-forward (CMS_TO_PWM) without encoders.

#define ENCODERS 1
const int PWM_L = 5, IN1_L = 7, IN2_L = 8;
const int PWM_R = 6, IN1_R = 9, IN2_R = 10;
const int ENC_L = 2, ENC_LB = 4, ENC_R = 3, ENC_RB = 11;   // A channels on interrupt pins
const int BATT_PIN = A0; const float BATT_DIV = 3.0;       // voltage divider ratio
const float TICKS_PER_CM = 20.0;                            // match --ticks-per-cm
const float CMS_TO_PWM = 5.0;                               // feed-forward: pwm per cm/s
const float KP = 3.0, KI = 8.0;

volatile long ticksL = 0, ticksR = 0;
float tgtL = 0, tgtR = 0, iL = 0, iR = 0;
long lastL = 0, lastR = 0;
unsigned long lastCmd = 0, lastCtl = 0, lastEnc = 0, lastBatt = 0;
char line[48]; int n = 0;

void isrL() { ticksL += digitalRead(ENC_LB) ? -1 : 1; }
void isrR() { ticksR += digitalRead(ENC_RB) ? 1 : -1; }   // mirrored motor

void drive(int pwmPin, int a, int b, float pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(a, pwm > 0); digitalWrite(b, pwm < 0);
  analogWrite(pwmPin, (int)fabs(pwm));
}

void setup() {
  Serial.begin(115200);
  int outs[] = {PWM_L, IN1_L, IN2_L, PWM_R, IN1_R, IN2_R};
  for (int p : outs) pinMode(p, OUTPUT);
#if ENCODERS
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP); pinMode(ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrR, RISING);
#endif
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[n] = 0; n = 0;
      float l, r;
      if (line[0] == 'V' && sscanf(line + 1, "%f %f", &l, &r) == 2) {}   // sscanf %f is absent on AVR
      if (line[0] == 'V') {
        char *p = line + 1;
        l = strtod(p, &p); r = strtod(p, &p);
        tgtL = l; tgtR = r; lastCmd = millis();
      }
    } else if (n < (int)sizeof(line) - 1) line[n++] = c;
  }
  unsigned long now = millis();
  if (now - lastCmd > 300) { tgtL = tgtR = 0; }
  if (now - lastCtl >= 20) {
    float dt = (now - lastCtl) / 1000.0; lastCtl = now;
#if ENCODERS
    noInterrupts(); long tl = ticksL, tr = ticksR; interrupts();
    float vL = (tl - lastL) / TICKS_PER_CM / dt, vR = (tr - lastR) / TICKS_PER_CM / dt;
    lastL = tl; lastR = tr;
    float eL = tgtL - vL, eR = tgtR - vR;
    iL = (tgtL == 0) ? 0 : constrain(iL + eL * dt, -30, 30);
    iR = (tgtR == 0) ? 0 : constrain(iR + eR * dt, -30, 30);
    drive(PWM_L, IN1_L, IN2_L, tgtL == 0 ? 0 : tgtL * CMS_TO_PWM + KP * eL + KI * iL);
    drive(PWM_R, IN1_R, IN2_R, tgtR == 0 ? 0 : tgtR * CMS_TO_PWM + KP * eR + KI * iR);
#else
    drive(PWM_L, IN1_L, IN2_L, tgtL * CMS_TO_PWM);
    drive(PWM_R, IN1_R, IN2_R, tgtR * CMS_TO_PWM);
#endif
  }
#if ENCODERS
  if (now - lastEnc >= 50) {
    lastEnc = now;
    noInterrupts(); long tl = ticksL, tr = ticksR; interrupts();
    Serial.print("E "); Serial.print(tl); Serial.print(' '); Serial.println(tr);
  }
#endif
  if (now - lastBatt >= 1000) {
    lastBatt = now;
    Serial.print("B "); Serial.println(analogRead(BATT_PIN) * 5.0 / 1023.0 * BATT_DIV, 2);
  }
}
