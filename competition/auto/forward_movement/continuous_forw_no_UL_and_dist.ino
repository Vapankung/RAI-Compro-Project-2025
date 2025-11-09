// ===== Pins (edit to your wiring) =====
#define L_IN1 11
#define L_IN2 12
#define L_PWM 10
#define R_IN1 8
#define R_IN2 4
#define R_PWM 6

#define L_ENC 2   // must be interrupt-capable
#define R_ENC 3   // must be interrupt-capable

// ===== Encoder & RPM =====
volatile long L_pulses=0, R_pulses=0;
const int PPR = 20;               // 20 slots, counting RISING edges
const unsigned long TS_MS = 100;  // control period = 100 ms

// ===== PI gains (start here, then tweak) =====
float Kp = 2.0f, Ki = 0.6f;       // start with PI; add D only if needed

// ===== Controller state =====
float targetRPM = 50.0f;          // same target for both wheels
float L_int=0, R_int=0;           // integrators
int   L_pwm=0, R_pwm=0;           // outputs
const int PWM_MIN = 30;           // overcome deadband
const int PWM_MAX = 255;
const float INT_LIM = 200.0f;     // anti-windup clamp

void L_isr(){ L_pulses++; }
void R_isr(){ R_pulses++; }

void setup(){
  Serial.begin(115200);

  pinMode(L_IN1,OUTPUT); pinMode(L_IN2,OUTPUT); pinMode(L_PWM,OUTPUT);
  pinMode(R_IN1,OUTPUT); pinMode(R_IN2,OUTPUT); pinMode(R_PWM,OUTPUT);
  pinMode(L_ENC,INPUT_PULLUP); pinMode(R_ENC,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC), L_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC), R_isr, RISING);

  // forward direction
  digitalWrite(L_IN1,LOW); digitalWrite(L_IN2,HIGH);
  digitalWrite(R_IN1,LOW); digitalWrite(R_IN2,HIGH);
}

void loop(){
  static unsigned long t0 = millis();
  if (millis() - t0 >= TS_MS){
    t0 += TS_MS;

    noInterrupts();
    long Lp = L_pulses; L_pulses = 0;
    long Rp = R_pulses; R_pulses = 0;
    interrupts();

    // pulses in 100 ms -> RPM
    // rev = pulses/PPR;  rev per sec = (pulses/PPR)/(TS_MS/1000)
    // RPM = rev/s * 60
    float L_rpm = (Lp * 600.0f) / (PPR * (TS_MS));  // 600 = 60*1000/TS_MS
    float R_rpm = (Rp * 600.0f) / (PPR * (TS_MS));

    // PI control per wheel
    float eL = targetRPM - L_rpm;
    float eR = targetRPM - R_rpm;

    L_int += eL;  R_int += eR;
    // anti-windup
    if (L_int > INT_LIM) L_int = INT_LIM; if (L_int < -INT_LIM) L_int = -INT_LIM;
    if (R_int > INT_LIM) R_int = INT_LIM; if (R_int < -INT_LIM) R_int = -INT_LIM;

    float uL = Kp*eL + Ki*L_int;
    float uR = Kp*eR + Ki*R_int;

    // update PWM with clamp & deadband
    L_pwm = constrain((int)(L_pwm + uL), PWM_MIN, PWM_MAX);
    R_pwm = constrain((int)(R_pwm + uR), PWM_MIN, PWM_MAX);

    analogWrite(L_PWM, L_pwm);
    analogWrite(R_PWM, R_pwm);

    // debug
    Serial.print("L_rpm:"); Serial.print(L_rpm,1);
    Serial.print(", R_rpm:"); Serial.print(R_rpm,1);
    Serial.print(", L_pwm:"); Serial.print(L_pwm);
    Serial.print(", R_pwm:"); Serial.print(R_pwm);
    Serial.println();
  }
}
