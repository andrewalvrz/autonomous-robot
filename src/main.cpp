#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <DFRobot_BMI160.h>
#include "fast_functions.h"

// ── I2C Addresses ─────────────────────────────────────────────
#define MUX_ADDR      0x70
#define AS5600_ADDR   0x36
#define VL53L0X_ADDR  0x29
#define BMI160_ADDR   0x68

// ── Mux Channels ──────────────────────────────────────────────
#define CH_ENC_M1    0
#define CH_ENC_M2    1
#define CH_ENC_M3    2
#define CH_ENC_M4    3
#define CH_TOF_LEFT  4
#define CH_TOF_RIGHT 5

// ── Sensor objects ────────────────────────────────────────────
Adafruit_VL53L0X tofLeft;
Adafruit_VL53L0X tofRight;
DFRobot_BMI160   bmi160;

// ── Status flags ──────────────────────────────────────────────
bool muxOk      = false;
bool encOk[4]   = {false, false, false, false};
bool tofLeftOk  = false;
bool tofRightOk = false;
bool imuOk      = false;

// ── Encoder tracking ──────────────────────────────────────────
uint16_t encPrev[4]  = {0, 0, 0, 0};
int32_t  encTicks[4] = {0, 0, 0, 0};
bool     encInit[4]  = {false, false, false, false};

// ─────────────────────────────────────────────────────────────
//  MUX
// ─────────────────────────────────────────────────────────────
bool muxSelect(uint8_t ch) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << ch);
    return Wire.endTransmission() == 0;
}

void muxDeselect() {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
}

bool i2cPing(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// ─────────────────────────────────────────────────────────────
//  AS5600 ENCODER
// ─────────────────────────────────────────────────────────────
uint16_t as5600Read(uint8_t muxCh) {
    if (!muxSelect(muxCh)) return 0xFFFF;
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E); // Angle high byte register
    if (Wire.endTransmission(false) != 0) return 0xFFFF;
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() < 2) return 0xFFFF;
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    return ((hi << 8) | lo) & 0x0FFF;
}

void encoderUpdate(uint8_t idx, uint8_t muxCh) {
    uint16_t raw = as5600Read(muxCh);
    if (raw == 0xFFFF) return;

    if (!encInit[idx]) {
        encPrev[idx] = raw;
        encInit[idx] = true;
        return;
    }

    int16_t delta = (int16_t)raw - (int16_t)encPrev[idx];
    if (delta >  2048) delta -= 4096;
    if (delta < -2048) delta += 4096;

    encTicks[idx] += delta;
    encPrev[idx]   = raw;
}

// ─────────────────────────────────────────────────────────────
//  INIT
// ─────────────────────────────────────────────────────────────
void initAll() {
    Serial.println("\n========================================");
    Serial.println("  ROVER STAGE 1 DIAGNOSTIC");
    Serial.println("========================================\n");

    // I2C
    Wire.begin(21, 22);
    Wire.setClock(400000);
    delay(100);
    Serial.println("--- I2C: GPIO21=SDA GPIO22=SCL 400kHz ---\n");

    // MUX
    Wire.beginTransmission(MUX_ADDR);
    muxOk = (Wire.endTransmission() == 0);
    Serial.printf("[%s] TCA9548A mux at 0x70\n", muxOk ? "OK  " : "FAIL");

    if (!muxOk) {
        Serial.println("\n[FATAL] Mux not found — check wiring. Halting.");
        while (true) delay(1000);
    }
    Serial.println();

    // ENCODERS
    Serial.println("--- AS5600 Encoders ---");
    const char* encNames[] = {"M1 (Ch0)", "M2 (Ch1)", "M3 (Ch2)", "M4 (Ch3)"};
    for (uint8_t i = 0; i < 4; i++) {
        muxSelect(i);
        delay(2);
        encOk[i] = i2cPing(AS5600_ADDR);
        Serial.printf("[%s] AS5600 %s\n", encOk[i] ? "OK  " : "FAIL", encNames[i]);
    }
    muxDeselect();
    Serial.println();

    // TOF
    Serial.println("--- VL53L0X ToF Sensors ---");
    muxSelect(CH_TOF_LEFT);
    delay(2);
    tofLeftOk = tofLeft.begin(VL53L0X_ADDR, &Wire);
    Serial.printf("[%s] VL53L0X Left  (Ch%d)\n", tofLeftOk  ? "OK  " : "FAIL", CH_TOF_LEFT);

    muxSelect(CH_TOF_RIGHT);
    delay(2);
    tofRightOk = tofRight.begin(VL53L0X_ADDR, &Wire);
    Serial.printf("[%s] VL53L0X Right (Ch%d)\n", tofRightOk ? "OK  " : "FAIL", CH_TOF_RIGHT);
    muxDeselect();
    Serial.println();

    // BMI160 — direct on I2C bus, no mux needed
    Serial.println("--- BMI160 IMU ---");
    Wire.beginTransmission(BMI160_ADDR);
    bool bmiPing = (Wire.endTransmission() == 0);
    Serial.printf("[%s] BMI160 ping at 0x68\n", bmiPing ? "OK  " : "FAIL");

    if (bmiPing) {
        bmi160.softReset();
        delay(10);
        int8_t result = bmi160.I2cInit(BMI160_ADDR);
        imuOk = (result == BMI160_OK);
        Serial.printf("[%s] BMI160 init\n", imuOk ? "OK  " : "FAIL");
    }
    Serial.println();

    // MOTORS
    motorsInit();
    motorsOFF();
    Serial.println("[OK  ] Motors initialized (OFF)");
    Serial.println();

    Serial.println("========================================");
    Serial.println("  INIT COMPLETE — Live readings below");
    Serial.println("  Serial commands:");
    Serial.println("  'f' = motors forward (speed 80)");
    Serial.println("  's' = motors stop");
    Serial.println("  'r' = reset encoder counts");
    Serial.println("========================================\n");
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    initAll();
}

// ─────────────────────────────────────────────────────────────
//  LOOP — live readings at 5Hz
// ─────────────────────────────────────────────────────────────
uint32_t lastPrint = 0;

void loop() {

    // ── Serial commands ───────────────────────────────────────
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'f') {
            motorsSetSpeed(80);
            motorsFWD();
            Serial.println("[CMD] Motors FORWARD at speed 80");
        }
        else if (c == 's') {
            motorsOFF();
            Serial.println("[CMD] Motors STOP");
        }
        else if (c == 'r') {
            for (uint8_t i = 0; i < 4; i++) {
                encTicks[i] = 0;
                encInit[i]  = false;
            }
            Serial.println("[CMD] Encoder counts reset");
        }
    }

    // ── Update encoders ───────────────────────────────────────
    for (uint8_t i = 0; i < 4; i++) {
        if (encOk[i]) encoderUpdate(i, i);
    }

    // ── Print at 5Hz ──────────────────────────────────────────
    if (millis() - lastPrint < 200) return;
    lastPrint = millis();

    Serial.println("─────────────────────────────────────");

    // Encoders
    Serial.println("ENCODERS (ticks):");
    Serial.printf("  M1: %6ld  M2: %6ld  M3: %6ld  M4: %6ld\n",
        encTicks[0], encTicks[1], encTicks[2], encTicks[3]);

    // IMU
    if (imuOk) {
        int16_t buf[6];
        bmi160.getAccelGyroData(buf);
        // buf[0-2] = gyro X/Y/Z, buf[3-5] = accel X/Y/Z
        float gxd = buf[0] / 65.6f;
        float gyd = buf[1] / 65.6f;
        float gzd = buf[2] / 65.6f;
        float axg = buf[3] / 8192.0f;
        float ayg = buf[4] / 8192.0f;
        float azg = buf[5] / 8192.0f;
        Serial.println("IMU (accel g / gyro deg/s):");
        Serial.printf("  Accel: X=%6.3f  Y=%6.3f  Z=%6.3f\n", axg, ayg, azg);
        Serial.printf("  Gyro:  X=%6.2f  Y=%6.2f  Z=%6.2f\n", gxd, gyd, gzd);
    } else {
        Serial.println("IMU: NOT AVAILABLE");
    }

    // ToF
    Serial.println("TOF (mm):");
    VL53L0X_RangingMeasurementData_t measure;

    if (tofLeftOk) {
        muxSelect(CH_TOF_LEFT);
        delayMicroseconds(300);
        tofLeft.rangingTest(&measure, false);
        if (measure.RangeStatus != 4)
            Serial.printf("  Left:  %4d mm\n", measure.RangeMilliMeter);
        else
            Serial.println("  Left:  out of range");
    } else {
        Serial.println("  Left:  NOT AVAILABLE");
    }

    if (tofRightOk) {
        muxSelect(CH_TOF_RIGHT);
        delayMicroseconds(300);
        tofRight.rangingTest(&measure, false);
        if (measure.RangeStatus != 4)
            Serial.printf("  Right: %4d mm\n", measure.RangeMilliMeter);
        else
            Serial.println("  Right: out of range");
    } else {
        Serial.println("  Right: NOT AVAILABLE");
    }

    muxDeselect();
    Serial.println();
}