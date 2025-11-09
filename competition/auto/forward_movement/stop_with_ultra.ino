#define L_IN1 10
#define L_IN2 12
#define L_PWM 11
#define R_IN1 8
#define R_IN2 4
#define R_PWM 6

#define L_ENC 2
#define R_ENC 3

#define US_TRIG 7
#define US_ECHO 5

const float STOP_CM = 4.0f;

// ===== Encoder & RPM =====
volatile long L_pulses = 0;
volatile long R_pulses = 0;
const int PPR = 20;
const unsigned long TS_MS = 100;

// ===== PI gains =====
float Kp = 2.0f;
float Ki = 0.6f;

// ===== Controller state =====
float targetRPM = 50.0f;
float L_int = 0;
float R_int = 0;
int L_pwm = 0;
int R_pwm = 0;
const int PWM_MIN = 30;
const int PWM_MAX = 255;
const float INT_LIM = 200.0f;

// ===== Run/Halt state =====
volatile bool stateRun = true;

// ===== Interrupts =====
void L_isr() { L_pulses++; }
void R_isr() { R_pulses++; }

// ===== Ultrasonic helpers =====
float readDistanceCM() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  long duration = pulseIn(US_ECHO, HIGH, 30000);  // 30ms timeout
  if (duration == 0) {
    return -1.0f;  // no echo
  }
  return duration * 0.0343f * 0.5f;  // cm
}

float readDistanceAverage(int minSamples = 50, int maxAttempts = 80) {
  long attempts = 0;
  long count = 0;
  double sum = 0.0;

  while (count < minSamples && attempts < maxAttempts) {
    float d = readDistanceCM();
    attempts++;
    if (d > 0) {
      sum += d;
      count++;
    }
    delay(2);
  }

  if (count == 0) {
    return -1.0f;
  }
  return (float)(sum / (double)count);
}

// ===== Motor helpers =====
void stopAndHold() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, LOW);
}

bool isHalted() {
  return stateRun == false;
}

void clearHalt() {
  stateRun = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== ROBOT INIT ===");

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_ENC, INPUT_PULLUP);
  pinMode(R_ENC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC), L_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC), R_isr, RISING);

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  digitalWrite(L_IN1, LOW);
  digitalWrite(L_IN2, HIGH);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);

  Serial.println("Setup OK.");
}

void loop() {
  // ---------- Ultrasonic gate ----------
  float quick = readDistanceCM();
  if (quick > 0 && quick <= STOP_CM) {
    float avg = readDistanceAverage(50, 80);
    Serial.print("[US] quick=");
    Serial.print(quick, 1);
    Serial.print(" cm, avg50=");
    Serial.print(avg, 1);
    Serial.println(" cm");

    if (avg > 0 && avg <= STOP_CM) {
      stateRun = false;
      targetRPM = 0.0f;
      L_int = 0.0f;
      R_int = 0.0f;
      L_pwm = 0;
      R_pwm = 0;
      stopAndHold();
      Serial.println("[HALT] Avg <= 4 cm. Motors stopped. Ready for next function.");
    } else if (avg > STOP_CM) {
      Serial.println("[US] Avg > 4 cm. Continue RUN.");
    } else {
      stateRun = false;
      stopAndHold();
      Serial.println("[HALT] Avg invalid (no valid echoes). Stopping for safety.");
    }
  }

  if (!stateRun) {
    delay(50);
    return;
  }

  // ---------- RUN: RPM PI control ----------
  static unsigned long t0 = millis();
  if (millis() - t0 >= TS_MS) {
    t0 += TS_MS;

    noInterrupts();
    long Lp = L_pulses;
    L_pulses = 0;
    long Rp = R_pulses;
    R_pulses = 0;
    interrupts();

    float L_rpm = (Lp * 600.0f) / (PPR * TS_MS);
    float R_rpm = (Rp * 600.0f) / (PPR * TS_MS);

    float eL = targetRPM - L_rpm;
    float eR = targetRPM - R_rpm;

    L_int += eL;
    R_int += eR;

    if (L_int > INT_LIM) L_int = INT_LIM;
    if (L_int < -INT_LIM) L_int = -INT_LIM;
    if (R_int > INT_LIM) R_int = INT_LIM;
    if (R_int < -INT_LIM) R_int = -INT_LIM;

    float uL = Kp * eL + Ki * L_int;
    float uR = Kp * eR + Ki * R_int;

    L_pwm = (int)(L_pwm + uL);
    R_pwm = (int)(R_pwm + uR);

    if (L_pwm > PWM_MAX) L_pwm = PWM_MAX;
    if (L_pwm < PWM_MIN) L_pwm = PWM_MIN;
    if (R_pwm > PWM_MAX) R_pwm = PWM_MAX;
    if (R_pwm < PWM_MIN) R_pwm = PWM_MIN;

    analogWrite(L_PWM, L_pwm);
    analogWrite(R_PWM, R_pwm);

    Serial.print("[RUN] Lp:");
    Serial.print(Lp);
    Serial.print(" Rp:");
    Serial.print(Rp);
    Serial.print(" | Lrpm:");
    Serial.print(L_rpm, 1);
    Serial.print(" Rrpm:");
    Serial.print(R_rpm, 1);
    Serial.print(" | PWM(L,R):");
    Serial.print(L_pwm);
    Serial.print(",");
    Serial.print(R_pwm);
    Serial.println();
  }
}
