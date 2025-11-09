// ===== Pins (edit to your wiring) =====
#define L_IN1 10
#define L_IN2 12
#define L_PWM 11
#define R_IN1 8
#define R_IN2 4
#define R_PWM 6

#define L_ENC 2   // must be interrupt-capable
#define R_ENC 3   // must be interrupt-capable

// ===== Ultrasonic pins =====
#define US_TRIG 7
#define US_ECHO 5

// ===== Stop threshold =====
const float STOP_CM = 4.0f;   // threshold for safety stop

// ===== Encoder & RPM =====
volatile long L_pulses=0, R_pulses=0;
const int PPR = 20;
const unsigned long TS_MS = 100;

// ===== PI gains =====
float Kp = 2.0f, Ki = 0.6f;

// ===== Controller state =====
float targetRPM = 50.0f;
float L_int=0, R_int=0;
int   L_pwm=0, R_pwm=0;
const int PWM_MIN = 30;
const int PWM_MAX = 255;
const float INT_LIM = 200.0f;

// ===== Run/Halt state =====
enum RobotState { RUN, HALT };
volatile RobotState state = RUN;

// ===== Interrupts =====
void L_isr(){ L_pulses++; }
void R_isr(){ R_pulses++; }

// ===== Ultrasonic helpers =====
float readDistanceCM() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  long duration = pulseIn(US_ECHO, HIGH, 30000);  // 30ms timeout
  if (duration == 0) return -1.0f;                 // no echo
  return duration * 0.0343f * 0.5f;               // cm
}

// Average at least `minSamples` VALID readings (distance >0).
// Try up to `maxAttempts` to collect them. Returns -1 if none valid.
float readDistanceAverage(int minSamples = 50, int maxAttempts = 80) {
  long attempts = 0, count = 0;
  double sum = 0.0;
  while (count < minSamples && attempts < maxAttempts) {
    float d = readDistanceCM();
    attempts++;
    if (d > 0) { sum += d; count++; }
    delay(2); // small gap to avoid sensor ringing
  }
  if (count == 0) return -1.0f;
  return (float)(sum / (double)count);
}

// ===== Motor helpers =====
void stopAndHold() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
  // idle (coast). For short-brake use HIGH/HIGH on each side.
  digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}

bool isHalted() { return state == HALT; }
void clearHalt() { state = RUN; }

void setup(){
  Serial.begin(115200);
  Serial.println("=== ROBOT INIT ===");

  pinMode(L_IN1,OUTPUT); pinMode(L_IN2,OUTPUT); pinMode(L_PWM,OUTPUT);
  pinMode(R_IN1,OUTPUT); pinMode(R_IN2,OUTPUT); pinMode(R_PWM,OUTPUT);
  pinMode(L_ENC,INPUT_PULLUP); pinMode(R_ENC,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC), L_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC), R_isr, RISING);

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);

  // forward direction (for RUN state)
  digitalWrite(L_IN1,LOW); digitalWrite(L_IN2,HIGH);
  digitalWrite(R_IN1,LOW); digitalWrite(R_IN2,HIGH);

  Serial.println("Setup OK.");
}

void loop(){
  // ---------- Ultrasonic gate ----------
  float quick = readDistanceCM();
  if (quick > 0 && quick <= STOP_CM) {
    // Close object detected -> confirm with averaged 50+ samples
    float avg = readDistanceAverage(50, 80);
    Serial.print("[US] quick="); Serial.print(quick,1);
    Serial.print(" cm, avg50="); Serial.print(avg,1); Serial.println(" cm");

    if (avg > 0 && avg <= STOP_CM) {
      // HARD STOP & READY
      state = HALT;
      targetRPM = 0.0f;
      L_int = R_int = 0.0f;
      L_pwm = R_pwm = 0;
      stopAndHold();
      Serial.println("[HALT] Avg <= 4 cm. Motors stopped. Ready for next function.");
    } else if (avg > STOP_CM) {
      // Safe -> continue RUN
      // (no state change)
      Serial.println("[US] Avg > 4 cm. Continue RUN.");
    } else {
      // avg invalid -> be safe; stop
      state = HALT;
      stopAndHold();
      Serial.println("[HALT] Avg invalid (no valid echoes). Stopping for safety.");
    }
  }

  if (state == HALT) {
    delay(50);
    return;
  }

  // ---------- RUN: RPM PI control ----------
  static unsigned long t0 = millis();
  if (millis() - t0 >= TS_MS){
    t0 += TS_MS;

    noInterrupts();
    long Lp = L_pulses; L_pulses = 0;
    long Rp = R_pulses; R_pulses = 0;
    interrupts();

    float L_rpm = (Lp * 600.0f) / (PPR * TS_MS);
    float R_rpm = (Rp * 600.0f) / (PPR * TS_MS);

    float eL = targetRPM - L_rpm;
    float eR = targetRPM - R_rpm;

    L_int += eL;  R_int += eR;
    L_int = constrain(L_int, -INT_LIM, INT_LIM);
    R_int = constrain(R_int, -INT_LIM, INT_LIM);

    float uL = Kp*eL + Ki*L_int;
    float uR = Kp*eR + Ki*R_int;

    L_pwm = constrain((int)(L_pwm + uL), PWM_MIN, PWM_MAX);
    R_pwm = constrain((int)(R_pwm + uR), PWM_MIN, PWM_MAX);

    analogWrite(L_PWM, L_pwm);
    analogWrite(R_PWM, R_pwm);

    // Debug line
    Serial.print("[RUN] Lp:"); Serial.print(Lp);
    Serial.print(" Rp:"); Serial.print(Rp);
    Serial.print(" | Lrpm:"); Serial.print(L_rpm,1);
    Serial.print(" Rrpm:"); Serial.print(R_rpm,1);
    Serial.print(" | PWM(L,R):"); Serial.print(L_pwm); Serial.print(","); Serial.print(R_pwm);
    Serial.println();
  }
}
