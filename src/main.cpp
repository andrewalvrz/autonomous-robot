#include <Arduino.h>

// ================== YOUR CONFIRMED PINS ==================
#define FL_FWD  18
#define FL_BKWD 19

#define FR_FWD  16
#define FR_BKWD 17

#define BL_FWD  14
#define BL_BKWD 27

#define BR_FWD  13
#define BR_BKWD 12

// ================== PWM CONFIG ==================
static const int PWM_FREQ = 20000;  // 20 kHz
static const int PWM_RES  = 8;      // 0..255

// LEDC channels (8 pins => 8 channels)
static const int CH_FL_FWD  = 0;
static const int CH_FL_BKWD = 1;
static const int CH_FR_FWD  = 2;
static const int CH_FR_BKWD = 3;
static const int CH_BL_FWD  = 4;
static const int CH_BL_BKWD = 5;
static const int CH_BR_FWD  = 6;
static const int CH_BR_BKWD = 7;

// ================== CMD LIMITS + DEADMAN ==================
static const float MAX_VX = 0.50f;   // m/s (for scaling)
static const float MAX_VY = 0.50f;   // m/s
static const float MAX_WZ = 1.50f;   // rad/s
static const uint32_t DEADMAN_MS = 500;

static float cmd_vx = 0.0f;
static float cmd_vy = 0.0f;
static float cmd_wz = 0.0f;
static uint32_t last_cmd_ms = 0;

// ================== HELPERS ==================
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

// set one wheel with signed pwm in [-255..255]
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

// Parse: "V,vx,vy,wz\n"
static bool parseCmdLine(const String &line, float &vx, float &vy, float &wz) {
  if (line.length() < 2) return false;
  if (line[0] != 'V') return false;

  // Expect commas
  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return false;

  vx = line.substring(c1 + 1, c2).toFloat();
  vy = line.substring(c2 + 1, c3).toFloat();
  wz = line.substring(c3 + 1).toFloat();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);

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

  Serial.println("ESP32 READY");
  Serial.println("Protocol: V,vx,vy,wz");
}

void loop() {
  // Deadman safety
  if (millis() - last_cmd_ms > DEADMAN_MS) {
    cmd_vx = cmd_vy = cmd_wz = 0.0f;
  }

  // Read serial lines
  static String buf;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      float vx, vy, wz;
      if (parseCmdLine(buf, vx, vy, wz)) {
        // clamp to expected ranges
        if (vx >  MAX_VX) vx =  MAX_VX;
        if (vx < -MAX_VX) vx = -MAX_VX;
        if (vy >  MAX_VY) vy =  MAX_VY;
        if (vy < -MAX_VY) vy = -MAX_VY;
        if (wz >  MAX_WZ) wz =  MAX_WZ;
        if (wz < -MAX_WZ) wz = -MAX_WZ;

        cmd_vx = vx;
        cmd_vy = vy;
        cmd_wz = wz;
        last_cmd_ms = millis();

        // Debug (comment out later)
        Serial.print("RX ");
        Serial.print(cmd_vx, 3); Serial.print(" ");
        Serial.print(cmd_vy, 3); Serial.print(" ");
        Serial.println(cmd_wz, 3);
      }
      buf = "";
    } else if (ch != '\r') {
      buf += ch;
      if (buf.length() > 120) buf = ""; // safety
    }
  }

  // ================== HOLONOMIC MIXING ==================
  // Normalize to -1..1
  float a = (MAX_VX > 0) ? (cmd_vx / MAX_VX) : 0.0f;
  float b = (MAX_VY > 0) ? (cmd_vy / MAX_VY) : 0.0f;
  float c = (MAX_WZ > 0) ? (cmd_wz / MAX_WZ) : 0.0f;

  // Common mecanum/omni 4-wheel mix (works for most holonomic builds)
  float fl = a - b - c;
  float fr = a + b + c;
  float bl = a + b - c;
  float br = a - b + c;

  // Normalize so max magnitude is 1.0
  float m = max(max(fabs(fl), fabs(fr)), max(fabs(bl), fabs(br)));
  if (m > 1.0f) {
    fl /= m; fr /= m; bl /= m; br /= m;
  }

  int pwm_fl = (int)roundf(fl * 255.0f);
  int pwm_fr = (int)roundf(fr * 255.0f);
  int pwm_bl = (int)roundf(bl * 255.0f);
  int pwm_br = (int)roundf(br * 255.0f);

  setWheel(CH_FL_FWD, CH_FL_BKWD, pwm_fl);
  setWheel(CH_FR_FWD, CH_FR_BKWD, pwm_fr);
  setWheel(CH_BL_FWD, CH_BL_BKWD, pwm_bl);
  setWheel(CH_BR_FWD, CH_BR_BKWD, pwm_br);

  delay(10); // ~100 Hz motor update
}
