 #include <Wire.h>

// ============================================
// BMI160 + Motor Control for 90° Turn
// ============================================

// --- Motor driver pins (edit to your wiring) ---
#define L_IN1 11
#define L_IN2 12
#define L_PWM 10
#define R_IN1 8
#define R_IN2 4
#define R_PWM 6

// --- BMI160 registers ---
#define BMI160_ADDR_LOW  0x68
#define BMI160_ADDR_HIGH 0x69
#define BMI160_REG_CHIP_ID 0x00
#define BMI160_REG_GYR_X_L 0x0C
#define BMI160_REG_GYR_X_H 0x0D
#define BMI160_REG_GYR_Y_L 0x0E
#define BMI160_REG_GYR_Y_H 0x0F
#define BMI160_REG_GYR_Z_L 0x10
#define BMI160_REG_GYR_Z_H 0x11
#define BMI160_REG_CMD 0x7E
#define BMI160_CMD_SOFTRESET 0xB6
#define BMI160_CMD_GYRO_NORMAL 0x15
#define BMI160_REG_GYR_RANGE 0x43
#define BMI160_REG_GYR_CONF 0x42

// ------------------ BMI160 minimal driver ------------------
struct BMI160 {
  uint8_t addr = BMI160_ADDR_LOW;

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool readMulti(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((int)addr, (int)len);
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
  }

  bool beginAt(uint8_t a) {
    addr = a;
    writeReg(BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
    delay(100);
    writeReg(BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL);
    delay(100);
    writeReg(BMI160_REG_GYR_RANGE, 0x00); // ±2000 dps
    uint8_t id;
    Wire.beginTransmission(addr);
    Wire.write(BMI160_REG_CHIP_ID);
    Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 1);
    id = Wire.read();
    return id == 0xD1;
  }

  bool beginAuto() {
    return beginAt(BMI160_ADDR_LOW) || beginAt(BMI160_ADDR_HIGH);
  }

  bool readGyroRaw(int16_t g[3]) {
    uint8_t buf[6];
    if (!readMulti(BMI160_REG_GYR_X_L, buf, 6)) return false;
    g[0] = (int16_t)((buf[1] << 8) | buf[0]);
    g[1] = (int16_t)((buf[3] << 8) | buf[2]);
    g[2] = (int16_t)((buf[5] << 8) | buf[4]);
    return true;
  }
};

// ------------------ Gyro Integrator ------------------
struct GyroIntegrator {
  BMI160 *dev;
  float offset = 0;
  float angle = 0;
  unsigned long lastMs;
  float LSB_PER_DPS = 16.4f;  // ±2000 dps scale

  bool begin(BMI160 &d) {
    dev = &d;
    lastMs = millis();
    return true;
  }

  bool calibrate() {
    long sum = 0;
    for (int i = 0; i < 200; i++) {
      int16_t g[3];
      dev->readGyroRaw(g);
      sum += g[2];
      delay(10);
    }
    offset = sum / 200.0f;
    return true;
  }

  void reset() { angle = 0; }

  bool update() {
    int16_t g[3];
    if (!dev->readGyroRaw(g)) return false;
    float w = (g[2] - offset) / LSB_PER_DPS;  // Z-axis dps
    unsigned long now = millis();
    float dt = (now - lastMs) / 1000.0f;
    lastMs = now;
    angle += w * dt;
    return true;
  }
};

// ------------------ Globals ------------------
BMI160 bmi;
GyroIntegrator gyro;

// ------------------ Motor helpers ------------------
void turnRight(uint8_t speed) {
  digitalWrite(L_IN1, HIGH);
  digitalWrite(L_IN2, LOW);
  digitalWrite(R_IN1, LOW);
  digitalWrite(R_IN2, HIGH);
  analogWrite(L_PWM, speed);
  analogWrite(R_PWM, speed);
}

void stopMotors() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}

// ------------------ Setup ------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  Serial.println("Init BMI160...");
  if (!bmi.beginAuto()) {
    Serial.println("BMI160 failed.");
    while (1);
  }
  Serial.println("BMI160 ready.");

  gyro.begin(bmi);
  Serial.println("Calibrating...");
  gyro.calibrate();
  gyro.reset();
  Serial.println("Calibration done.");

  delay(1000);
  Serial.println("Turning 90 degrees CW...");
}

// ------------------ Loop ------------------
void loop() {
  gyro.update();
  Serial.print("Angle = ");
  Serial.println(gyro.angle);

  // Turn until ~90 degrees reached
  if (abs(gyro.angle) < 90.0) {
    turnRight(150); // adjust PWM as needed
  } else {
    stopMotors();
    Serial.println("Turn complete!");
    while (1);  // stop forever
  }

  delay(10);
}
