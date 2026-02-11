#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <DFRobot_BMI160.h>
#include <AS5600.h>   // robtillaart/AS5600

// =======================================================
//                         PINS
// =======================================================

// ---- Motor PWM pins (your confirmed pins) ----
#define FL_FWD  18
#define FL_BKWD 19
#define FR_FWD  16
#define FR_BKWD 17
#define BL_FWD  14
#define BL_BKWD 27
#define BR_FWD  13
#define BR_BKWD 12

// ---- Hall sensor + Servo gate ----
// TODO: set these to your real pins if different
#define HALL_PIN   15
#define SERVO_PIN  23

// =======================================================
//                     PWM CONFIG (LEDC)
// =======================================================
static const int PWM_FREQ = 20000;  // 20 kHz
static const int PWM_RES  = 8;      // 0..255

static const int CH_FL_FWD  = 0;
static const int CH_FL_BKWD = 1;
static const int CH_FR_FWD  = 2;
static const int CH_FR_BKWD = 3;
static const int CH_BL_FWD  = 4;
static const int CH_BL_BKWD = 5;
static const int CH_BR_FWD  = 6;
static const int CH_BR_BKWD = 7;

// =======================================================
//                   CMD LIMITS + SAFETY
// =======================================================
static const float MAX_VX = 0.50f;
static const float MAX_VY = 0.50f;
static const float MAX_WZ = 1.50f;
static const uint32_t DEADMAN_MS = 500;

static float cmd_vx = 0.0f;
static float cmd_vy = 0.0f;
static float cmd_wz = 0.0f;
static uint32_t last_cmd_ms = 0;

// =======================================================
//                         I2C
// =======================================================
#define SDA_PIN 21
#define SCL_PIN 22

// TCA9548A mux (default per your schematic)
static const uint8_t TCA_ADDR = 0x70;

// AS5600 encoder mux channels (assumption you approved)
static const uint8_t CH_FL = 0;
static const uint8_t CH_FR = 1;
static const uint8_t CH_BL = 2;
static const uint8_t CH_BR = 3;

// =======================================================
//                     TOF (VL53L0X)
// =======================================================
#define XSHUT_LEFT  25
#define XSHUT_RIGHT 26

static const uint8_t ADDR_LEFT  = 0x30;
static const uint8_t ADDR_RIGHT = 0x31;

static const uint32_t TOF_PERIOD_MS = 50; // 20 Hz
static uint32_t last_tof_ms = 0;

Adafruit_VL53L0X tofL;
Adafruit_VL53L0X tofR;

static uint16_t tof_left_mm_filt = 0;
static uint16_t tof_right_mm_filt = 0;

static uint16_t ema_u16(uint16_t prev, uint16_t now, float alpha) {
  if (prev == 0) return now;
  return (uint16_t)roundf(alpha * prev + (1.0f - alpha) * now);
}

static void xshutAllLow() {
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(50);
}

static bool initDualVL53L0X() {
  xshutAllLow();

  // LEFT
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(80);
  if (!tofL.begin(0x29, false, &Wire)) return false;
  tofL.setAddress(ADDR_LEFT);
  delay(5);

  // RIGHT
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(80);
  if (!tofR.begin(0x29, false, &Wire)) return false;
  tofR.setAddress(ADDR_RIGHT);
  delay(5);

  return true;
}

static void readToF() {
  VL53L0X_RangingMeasurementData_t mL, mR;
  tofL.rangingTest(&mL, false);
  tofR.rangingTest(&mR, false);

  uint16_t left_mm  = (mL.RangeStatus == 0) ? mL.RangeMilliMeter : 0;
  uint16_t right_mm = (mR.RangeStatus == 0) ? mR.RangeMilliMeter : 0;

  tof_left_mm_filt  = ema_u16(tof_left_mm_filt, left_mm, 0.7f);
  tof_right_mm_filt = ema_u16(tof_right_mm_filt, right_mm, 0.7f);
}

// =======================================================
//                      IMU (BMI160)
// =======================================================
Adafruit_BMI160 bmi160;
static const uint32_t IMU_PERIOD_MS = 20; // 50 Hz
static uint32_t last_imu_ms = 0;

static float yaw_deg = 0.0f;
static float gyro_z_dps = 0.0f;
static uint32_t last_yaw_ms = 0;

static bool gyro_cal_done = false;
static float gyro_z_bias_dps = 0.0f;

static void updateYawFromGyro(float gz_dps) {
  uint32_t now = millis();
  if (last_yaw_ms == 0) last_yaw_ms = now;
  float dt = (now - last_yaw_ms) / 1000.0f;
  last_yaw_ms = now;

  yaw_deg += gz_dps * dt;

  if (yaw_deg > 180.0f) yaw_deg -= 360.0f;
  if (yaw_deg < -180.0f) yaw_deg += 360.0f;
}

static void calibrateGyroBias() {
  const uint32_t duration_ms = 2000;
  uint32_t start = millis();
  float sum = 0.0f;
  int n = 0;

  while (millis() - start < duration_ms) {
    sensors_event_t accel, gyro, temp;
    bmi160.getEvent(&accel, &gyro, &temp);
    float gz_dps = gyro.gyro.z * (180.0f / PI);
    sum += gz_dps;
    n++;
    delay(5);
  }
  gyro_z_bias_dps = (n > 0) ? (sum / (float)n) : 0.0f;
  gyro_cal_done = true;
}

// =======================================================
//               AS5600 ENCODERS via TCA9548A
// =======================================================
AS5600 as5600;

// Reads AS5600 raw angle (0..4095) on a mux channel.
// Returns -1 on failure.
static int readAS5600_raw_on_channel(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  if (Wire.endTransmission() != 0) {
    return -1;
  }
  // AS5600 library reads from current I2C bus selection
  int raw = (int)as5600.rawAngle();  // 0..4095
  if (raw < 0 || raw > 4095) return -1;
  return raw;
}

// Unwrap logic: convert raw angle to signed delta in [-2048, 2047]
static int16_t angleDelta(int prev, int cur) {
  int d = cur - prev;
  if (d >  2048) d -= 4096;
  if (d < -2048) d += 4096;
  return (int16_t)d;
}

// Periodic encoder update
static const uint32_t ENC_PERIOD_MS = 20; // 50 Hz
static uint32_t last_enc_ms = 0;

// last raw angles
static int fl_last = -1, fr_last = -1, bl_last = -1, br_last = -1;

// "ticks" = delta rawAngle counts per period (like quadrature tick deltas)
static int16_t ticks_fl = 0, ticks_fr = 0, ticks_bl = 0, ticks_br = 0;

static void encInitReadOnce() {
  fl_last = readAS5600_raw_on_channel(CH_FL);
  fr_last = readAS5600_raw_on_channel(CH_FR);
  bl_last = readAS5600_raw_on_channel(CH_BL);
  br_last = readAS5600_raw_on_channel(CH_BR);
}

static void updateEncoders() {
  int fl = readAS5600_raw_on_channel(CH_FL);
  int fr = readAS5600_raw_on_channel(CH_FR);
  int bl = readAS5600_raw_on_channel(CH_BL);
  int br = readAS5600_raw_on_channel(CH_BR);

  // If any read fails, keep previous ticks as 0 for safety (don’t inject garbage)
  ticks_fl = (fl_last >= 0 && fl >= 0) ? angleDelta(fl_last, fl) : 0;
  ticks_fr = (fr_last >= 0 && fr >= 0) ? angleDelta(fr_last, fr) : 0;
  ticks_bl = (bl_last >= 0 && bl >= 0) ? angleDelta(bl_last, bl) : 0;
  ticks_br = (br_last >= 0 && br >= 0) ? angleDelta(br_last, br) : 0;

  if (fl >= 0) fl_last = fl;
  if (fr >= 0) fr_last = fr;
  if (bl >= 0) bl_last = bl;
  if (br >= 0) br_last = br;
}

// =======================================================
//                 SERVO GATE (SEPARATOR)
// =======================================================
Servo gateServo;

enum GatePos : uint8_t { GATE_CLOSED=0, GATE_GEO=1, GATE_NEB=2 };
static GatePos gate_pos = GATE_CLOSED;

// TODO: tune for your mechanism
static const int SERVO_US_CLOSED = 1100;
static const int SERVO_US_GEO    = 1500;
static const int SERVO_US_NEB    = 1900;

static void writeGateServo() {
  int us = SERVO_US_CLOSED;
  if (gate_pos == GATE_GEO) us = SERVO_US_GEO;
  else if (gate_pos == GATE_NEB) us = SERVO_US_NEB;
  gateServo.writeMicroseconds(us);
}

// =======================================================
//                      HELPERS
// =======================================================
static inline int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void setupPinPwm(int pin, int ch) {
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
  ledcWrite(ch, 0);
}

static void setWheel(int ch_fwd, int ch_bkwd, int pwm) {
  pwm = clampi(pwm, -255, 255);
  if (pwm > 0) {
    ledcWrite(ch_bkwd, 0);
    ledcWrite(ch_fwd, pwm);
  } else if (pwm < 0) {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_bkwd, -pwm);
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_bkwd, 0);
  }
}

static void stopAll() {
  setWheel(CH_FL_FWD, CH_FL_BKWD, 0);
  setWheel(CH_FR_FWD, CH_FR_BKWD, 0);
  setWheel(CH_BL_FWD, CH_BL_BKWD, 0);
  setWheel(CH_BR_FWD, CH_BR_BKWD, 0);
}

static void applyCmdClamp(float &vx, float &vy, float &wz) {
  if (vx >  MAX_VX) vx =  MAX_VX;
  if (vx < -MAX_VX) vx = -MAX_VX;
  if (vy >  MAX_VY) vy =  MAX_VY;
  if (vy < -MAX_VY) vy = -MAX_VY;
  if (wz >  MAX_WZ) wz =  MAX_WZ;
  if (wz < -MAX_WZ) wz = -MAX_WZ;
}

// =======================================================
//                 DRIVE MIXING (MECANUM)
// =======================================================
static void driveFromCmd(float vx, float vy, float wz) {
  float a = (MAX_VX > 0) ? (vx / MAX_VX) : 0.0f;
  float b = (MAX_VY > 0) ? (vy / MAX_VY) : 0.0f;
  float c = (MAX_WZ > 0) ? (wz / MAX_WZ) : 0.0f;

  float fl = a - b - c;
  float fr = a + b + c;
  float bl = a + b - c;
  float br = a - b + c;

  float m = max(max(fabs(fl), fabs(fr)), max(fabs(bl), fabs(br)));
  if (m > 1.0f) { fl /= m; fr /= m; bl /= m; br /= m; }

  int pwm_fl = (int)roundf(fl * 255.0f);
  int pwm_fr = (int)roundf(fr * 255.0f);
  int pwm_bl = (int)roundf(bl * 255.0f);
  int pwm_br = (int)roundf(br * 255.0f);

  setWheel(CH_FL_FWD, CH_FL_BKWD, pwm_fl);
  setWheel(CH_FR_FWD, CH_FR_BKWD, pwm_fr);
  setWheel(CH_BL_FWD, CH_BL_BKWD, pwm_bl);
  setWheel(CH_BR_FWD, CH_BR_BKWD, pwm_br);
}

// =======================================================
//                 SERIAL PROTOCOL (COMMANDS)
// =======================================================
// Commands:
//   V,vx,vy,wz\n   (floats)
//   G,pos\n        pos: 0=CLOSED 1=GEO 2=NEB
//   X\n            emergency stop (motors + gate closed)

static bool parseCmdVel(const String &line, float &vx, float &vy, float &wz) {
  if (line.length() < 2) return false;
  if (line[0] != 'V') return false;

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return false;

  vx = line.substring(c1 + 1, c2).toFloat();
  vy = line.substring(c2 + 1, c3).toFloat();
  wz = line.substring(c3 + 1).toFloat();
  return true;
}

static bool parseGate(const String &line, GatePos &pos) {
  if (line.length() < 3) return false;
  if (line[0] != 'G') return false;

  int c1 = line.indexOf(',');
  if (c1 < 0) return false;
  int v = line.substring(c1 + 1).toInt();
  v = clampi(v, 0, 2);
  pos = (GatePos)v;
  return true;
}

// =======================================================
//               SERIAL PROTOCOL (STATUS FRAME)
// =======================================================
// Status at fixed rate (20 Hz):
//   S,TOF,l,r,IMU,yaw,gyz,ENC,fl,fr,bl,br,H,hall,GP,gate
static const uint32_t STATUS_PERIOD_MS = 50;
static uint32_t last_status_ms = 0;

static void publishStatus() {
  int hall = digitalRead(HALL_PIN) ? 1 : 0;

  Serial.print("S,TOF,");
  Serial.print(tof_left_mm_filt);
  Serial.print(",");
  Serial.print(tof_right_mm_filt);

  Serial.print(",IMU,");
  Serial.print(yaw_deg, 2);
  Serial.print(",");
  Serial.print(gyro_z_dps, 2);

  Serial.print(",ENC,");
  Serial.print(ticks_fl); Serial.print(",");
  Serial.print(ticks_fr); Serial.print(",");
  Serial.print(ticks_bl); Serial.print(",");
  Serial.print(ticks_br);

  Serial.print(",H,");
  Serial.print(hall);

  Serial.print(",GP,");
  Serial.println((int)gate_pos);
}

// =======================================================
//                         SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // PWM setup
  setupPinPwm(FL_FWD,  CH_FL_FWD);
  setupPinPwm(FL_BKWD, CH_FL_BKWD);
  setupPinPwm(FR_FWD,  CH_FR_FWD);
  setupPinPwm(FR_BKWD, CH_FR_BKWD);
  setupPinPwm(BL_FWD,  CH_BL_FWD);
  setupPinPwm(BL_BKWD, CH_BL_BKWD);
  setupPinPwm(BR_FWD,  CH_BR_FWD);
  setupPinPwm(BR_BKWD, CH_BR_BKWD);

  stopAll();
  last_cmd_ms = millis();

  // Inputs
  pinMode(HALL_PIN, INPUT_PULLUP);

  // I2C init ONCE
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Init AS5600 (library uses current Wire bus selection)
  // (No special init required besides I2C being up.)

  // Servo gate
  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2500);
  gate_pos = GATE_CLOSED;
  writeGateServo();

  // ToF init
  bool tof_ok = initDualVL53L0X();

  // IMU init (BMI160)
  bool imu_ok = bmi160.begin(BMI160_I2CADDR_DEFAULT, &Wire);

  // Initialize encoder baseline angles
  encInitReadOnce();

  Serial.println("ESP32 READY (AS5600+TCA)");
  Serial.println("Commands:");
  Serial.println("  V,vx,vy,wz");
  Serial.println("  G,pos (0=CLOSED 1=GEO 2=NEB)");
  Serial.println("  X (stop)");
  Serial.print("TOF INIT: "); Serial.println(tof_ok ? "OK" : "FAIL");
  Serial.print("IMU INIT: "); Serial.println(imu_ok ? "OK" : "FAIL");

  if (imu_ok) {
    Serial.println("Calibrating gyro bias... keep robot still ~2s");
    calibrateGyroBias();
    Serial.print("Gyro Z bias (dps): ");
    Serial.println(gyro_z_bias_dps, 3);
  }

  uint32_t now = millis();
  last_tof_ms = last_imu_ms = last_enc_ms = last_status_ms = now;
  last_yaw_ms = now;
}

// =======================================================
//                          LOOP
// =======================================================
void loop() {
  // ----- Deadman -----
  if (millis() - last_cmd_ms > DEADMAN_MS) {
    cmd_vx = cmd_vy = cmd_wz = 0.0f;
  }

  // ----- Serial RX -----
  static String buf;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      buf.trim();

      if (buf == "X") {
        cmd_vx = cmd_vy = cmd_wz = 0.0f;
        gate_pos = GATE_CLOSED;
        writeGateServo();
        stopAll();
      } else {
        float vx, vy, wz;
        GatePos gp;

        if (parseCmdVel(buf, vx, vy, wz)) {
          applyCmdClamp(vx, vy, wz);
          cmd_vx = vx;
          cmd_vy = vy;
          cmd_wz = wz;
          last_cmd_ms = millis();
        } else if (parseGate(buf, gp)) {
          gate_pos = gp;
          writeGateServo();
        }
      }

      buf = "";
    } else if (ch != '\r') {
      buf += ch;
      if (buf.length() > 160) buf = "";
    }
  }

  uint32_t now = millis();

  // ----- ToF update -----
  if (now - last_tof_ms >= TOF_PERIOD_MS) {
    last_tof_ms = now;
    readToF();
  }

  // ----- IMU update -----
  if (now - last_imu_ms >= IMU_PERIOD_MS) {
    last_imu_ms = now;
    sensors_event_t accel, gyro, temp;
    bmi160.getEvent(&accel, &gyro, &temp);

    float gz_dps_raw = gyro.gyro.z * (180.0f / PI);
    gyro_z_dps = gyro_cal_done ? (gz_dps_raw - gyro_z_bias_dps) : gz_dps_raw;
    updateYawFromGyro(gyro_z_dps);
  }

  // ----- Encoder update (AS5600 over mux) -----
  if (now - last_enc_ms >= ENC_PERIOD_MS) {
    last_enc_ms = now;
    updateEncoders();
  }

  // ----- Drive + servo -----
  driveFromCmd(cmd_vx, cmd_vy, cmd_wz);

  // ----- Status publish -----
  if (now - last_status_ms >= STATUS_PERIOD_MS) {
    last_status_ms = now;
    publishStatus();
  }

  delay(5);
}
