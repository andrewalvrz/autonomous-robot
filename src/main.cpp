#include <Arduino.h>

// ---- Pins (from your header, joystick ignored) ----
#define M1_FWD  13
#define M1_BKWD 12
#define M2_FWD  14
#define M2_BKWD 27
#define M3_FWD  26
#define M3_BKWD 25
#define M4_FWD  33
#define M4_BKWD 32

// ---- PWM config ----
static const int PWM_FREQ = 20000;  // 20 kHz
static const int PWM_RES  = 8;      // 0-255

// LEDC channels (ESP32 has many; we’ll assign 8 total)
static const int CH_M1_FWD  = 0;
static const int CH_M1_BKWD = 1;
static const int CH_M2_FWD  = 2;
static const int CH_M2_BKWD = 3;
static const int CH_M3_FWD  = 4;
static const int CH_M3_BKWD = 5;
static const int CH_M4_FWD  = 6;
static const int CH_M4_BKWD = 7;

// ---- Safety + scaling ----
static const float MAX_LIN = 0.5f;     // m/s (just used for scaling)
static const float MAX_ANG = 1.5f;     // rad/s (just used for scaling)
static const uint32_t DEADMAN_MS = 500;

// Current commanded velocities
static float cmd_lin = 0.0f;
static float cmd_ang = 0.0f;
static uint32_t last_cmd_ms = 0;

// Utility clamp
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Convert [-1..1] to PWM [0..255]
static uint8_t dutyFromNorm(float n) {
  n = clampf(n, -1.0f, 1.0f);
  return (uint8_t)(fabsf(n) * 255.0f);
}

// Set one motor with signed command in [-1..1]
static void setMotor(int ch_fwd, int ch_bkwd, float u) {
  u = clampf(u, -1.0f, 1.0f);
  uint8_t duty = dutyFromNorm(u);

  if (u > 0.0f) {
    ledcWrite(ch_fwd, duty);
    ledcWrite(ch_bkwd, 0);
  } else if (u < 0.0f) {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_bkwd, duty);
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_bkwd, 0);
  }
}

static void stopAll() {
  setMotor(CH_M1_FWD, CH_M1_BKWD, 0);
  setMotor(CH_M2_FWD, CH_M2_BKWD, 0);
  setMotor(CH_M3_FWD, CH_M3_BKWD, 0);
  setMotor(CH_M4_FWD, CH_M4_BKWD, 0);
}

// Mix cmd_vel (lin, ang) into left/right normalized commands
static void applyCmdVel(float lin, float ang) {
  // Normalize lin/ang into roughly [-1..1] ranges
  float lin_n = clampf(lin / MAX_LIN, -1.0f, 1.0f);
  float ang_n = clampf(ang / MAX_ANG, -1.0f, 1.0f);

  // Differential mixing
  float left  = lin_n - ang_n;
  float right = lin_n + ang_n;

  // Clamp after mixing
  left  = clampf(left,  -1.0f, 1.0f);
  right = clampf(right, -1.0f, 1.0f);

  // Assign: left = M1 + M3, right = M2 + M4
  setMotor(CH_M1_FWD, CH_M1_BKWD, left);
  setMotor(CH_M3_FWD, CH_M3_BKWD, left);

  setMotor(CH_M2_FWD, CH_M2_BKWD, right);
  setMotor(CH_M4_FWD, CH_M4_BKWD, right);
}

// Parse line: "V,lin,ang"
static bool parseVelLine(const String& line, float &lin_out, float &ang_out) {
  // Basic format check
  if (line.length() < 3) return false;
  if (line.charAt(0) != 'V') return false;
  if (line.charAt(1) != ',') return false;

  int comma2 = line.indexOf(',', 2);
  if (comma2 < 0) return false;

  String s_lin = line.substring(2, comma2);
  String s_ang = line.substring(comma2 + 1);

  lin_out = s_lin.toFloat();
  ang_out = s_ang.toFloat();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Setup PWM channels and attach to pins
  ledcSetup(CH_M1_FWD,  PWM_FREQ, PWM_RES); ledcAttachPin(M1_FWD,  CH_M1_FWD);
  ledcSetup(CH_M1_BKWD, PWM_FREQ, PWM_RES); ledcAttachPin(M1_BKWD, CH_M1_BKWD);

  ledcSetup(CH_M2_FWD,  PWM_FREQ, PWM_RES); ledcAttachPin(M2_FWD,  CH_M2_FWD);
  ledcSetup(CH_M2_BKWD, PWM_FREQ, PWM_RES); ledcAttachPin(M2_BKWD, CH_M2_BKWD);

  ledcSetup(CH_M3_FWD,  PWM_FREQ, PWM_RES); ledcAttachPin(M3_FWD,  CH_M3_FWD);
  ledcSetup(CH_M3_BKWD, PWM_FREQ, PWM_RES); ledcAttachPin(M3_BKWD, CH_M3_BKWD);

  ledcSetup(CH_M4_FWD,  PWM_FREQ, PWM_RES); ledcAttachPin(M4_FWD,  CH_M4_FWD);
  ledcSetup(CH_M4_BKWD, PWM_FREQ, PWM_RES); ledcAttachPin(M4_BKWD, CH_M4_BKWD);

  stopAll();
  last_cmd_ms = millis();

  Serial.println("READY: send 'V,lin,ang' e.g. V,0.20,0.00");
}

void loop() {
  // Deadman safety
  if (millis() - last_cmd_ms > DEADMAN_MS) {
    stopAll();
  } else {
    applyCmdVel(cmd_lin, cmd_ang);
  }

  // Read serial lines
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    float lin, ang;
    if (parseVelLine(line, lin, ang)) {
      cmd_lin = lin;
      cmd_ang = ang;
      last_cmd_ms = millis();
      // Optional debug:
      // Serial.printf("OK V lin=%.3f ang=%.3f\n", cmd_lin, cmd_ang);
    } else {
      // Optional debug:
      // Serial.println("ERR bad line");
    }
  }
}
