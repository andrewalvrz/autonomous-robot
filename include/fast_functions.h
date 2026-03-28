#ifndef FAST_FUNCTIONS_H
#define FAST_FUNCTIONS_H

#include <Arduino.h>
#include <driver/gpio.h>
#include "soc/gpio_struct.h"
#include <ESP32Servo.h>

// ============================================================
//  PINOUT SUMMARY — ESP32 DevKit V1
// ============================================================
//
//  I2C (used by MUX, encoders, ToF, IMU in main.cpp)
//    SDA  →  GPIO 21
//    SCL  →  GPIO 22
//
//  Motors (H-bridge PWM, LEDC channels 0–7)
//    M1_FWD   →  GPIO 18    CH 0
//    M1_BKWD  →  GPIO 19    CH 1
//    M2_FWD   →  GPIO 16    CH 2
//    M2_BKWD  →  GPIO 17    CH 3
//    M3_FWD   →  GPIO 27    CH 4
//    M3_BKWD  →  GPIO 26    CH 5
//    M4_FWD   →  GPIO 13    CH 6
//    M4_BKWD  →  GPIO 12    CH 7
//
//  Servos (ESP32Servo, 50 Hz)
//    servo_door  →  GPIO 23
//    servo_arm   →  GPIO 33   
//
//  Photosensor
//    PHOTO_PIN  →  GPIO 25   (ADC2_CH8 — analog read)
//                  ⚠ ADC2 unavailable while Wi-Fi is active.
//                  Using serial micro-ROS transport = fine.
//                  If switching to Wi-Fi transport, move to
//                  ADC1: GPIO 32, 33, 34, 35, 36, or 39.
//
//  Collection motor (digital output)
//    COLLECTION  →  GPIO 32   ⚠ Moved from 21 (21 = I2C SDA)
//
// ============================================================

// ── Photosensor ──────────────────────────────────────────────
#define PHOTO_PIN  25
#define photo      PHOTO_PIN   // legacy alias

// ── Servos ───────────────────────────────────────────────────
constexpr uint8_t SERVO_DOOR_PIN  = 23;
constexpr uint8_t SERVO_ARM_PIN   = 33;    // Moved from 22 (I2C SCL conflict)

static const uint16_t SERVO_MIN_US            = 500;
static const uint16_t SERVO_MAX_US            = 2400;
static const uint8_t  SERVO_DOOR_CLOSED_ANGLE = 90;
static const uint8_t  SERVO_DOOR_OPEN_ANGLE   = 0;
static const uint8_t  SERVO_ARM_HOME_ANGLE    = 0;
static const uint8_t  SERVO_ARM_DROP_ANGLE    = 90;
static const uint8_t  SERVO_ARM_LIFT_ANGLE    = 180;

Servo doorServo;
Servo armServo;

// ── Collection motor ─────────────────────────────────────────
#define COLLECTION  32         // Moved from 21 (I2C SDA conflict)

// ── Drive motors ─────────────────────────────────────────────
#define M1_FWD   18
#define M1_BKWD  19
#define M2_FWD   16
#define M2_BKWD  17
#define M3_FWD   27
#define M3_BKWD  26
#define M4_FWD   13
#define M4_BKWD  12

// ── GPIO bitmasks ─────────────────────────────────────────────
constexpr uint32_t ALL_MOTOR_MASK =
    (1UL << M1_FWD)  | (1UL << M1_BKWD) |
    (1UL << M2_FWD)  | (1UL << M2_BKWD) |
    (1UL << M3_FWD)  | (1UL << M3_BKWD) |
    (1UL << M4_FWD)  | (1UL << M4_BKWD);

constexpr uint32_t FWD_MASK  = (1UL << M1_FWD)  | (1UL << M2_FWD)  | (1UL << M3_FWD)  | (1UL << M4_FWD);
constexpr uint32_t BKWD_MASK = (1UL << M1_BKWD) | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_BKWD);

constexpr uint32_t STRAFE_LEFT_MASK  = (1UL << M1_BKWD) | (1UL << M2_FWD)  | (1UL << M3_FWD)  | (1UL << M4_BKWD);
constexpr uint32_t STRAFE_RIGHT_MASK = (1UL << M1_FWD)  | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_FWD);

constexpr uint32_t ROT_LEFT_MASK  = (1UL << M1_BKWD) | (1UL << M3_BKWD) | (1UL << M2_FWD)  | (1UL << M4_FWD);
constexpr uint32_t ROT_RIGHT_MASK = (1UL << M1_FWD)  | (1UL << M3_FWD)  | (1UL << M2_BKWD) | (1UL << M4_BKWD);

constexpr uint32_t SERVO_LOW_MASK  = (1UL << SERVO_DOOR_PIN);            // GPIO23 → low bank
constexpr uint32_t SERVO_HIGH_MASK = (1UL << (SERVO_ARM_PIN - 32));      // GPIO33 → high bank
constexpr uint32_t COLLECTION_MASK = (1UL << (COLLECTION - 32));  // GPIO32+ → high GPIO bank

// ── LEDC PWM (channels 0–7) ───────────────────────────────────
constexpr uint32_t PWM_FREQ_HZ  = 20000;
constexpr uint8_t  PWM_RES_BITS = 8;
constexpr uint8_t  CH_M1_FWD   = 2;  // 0-1 reserved for ESP32Servo
constexpr uint8_t  CH_M1_BKWD  = 3;
constexpr uint8_t  CH_M2_FWD   = 4;
constexpr uint8_t  CH_M2_BKWD  = 5;
constexpr uint8_t  CH_M3_FWD   = 6;
constexpr uint8_t  CH_M3_BKWD  = 7;
constexpr uint8_t  CH_M4_FWD   = 8;
constexpr uint8_t  CH_M4_BKWD  = 9;

static uint8_t motorSpeed = 255;

// ── Motor functions ───────────────────────────────────────────
inline void motorsSetSpeed(uint8_t speed) { motorSpeed = speed; }

inline void motorsOFF() {
    ledcWrite(CH_M1_FWD, 0); ledcWrite(CH_M1_BKWD, 0);
    ledcWrite(CH_M2_FWD, 0); ledcWrite(CH_M2_BKWD, 0);
    ledcWrite(CH_M3_FWD, 0); ledcWrite(CH_M3_BKWD, 0);
    ledcWrite(CH_M4_FWD, 0); ledcWrite(CH_M4_BKWD, 0);
}

inline void motorsFWD() {
    ledcWrite(CH_M1_BKWD, 0); ledcWrite(CH_M2_BKWD, 0);
    ledcWrite(CH_M3_BKWD, 0); ledcWrite(CH_M4_BKWD, 0);
    ledcWrite(CH_M1_FWD, motorSpeed); ledcWrite(CH_M2_FWD, motorSpeed);
    ledcWrite(CH_M3_FWD, motorSpeed); ledcWrite(CH_M4_FWD, motorSpeed);
}

inline void motorsBKWD() {
    ledcWrite(CH_M1_FWD, 0); ledcWrite(CH_M2_FWD, 0);
    ledcWrite(CH_M3_FWD, 0); ledcWrite(CH_M4_FWD, 0);
    ledcWrite(CH_M1_BKWD, motorSpeed); ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed); ledcWrite(CH_M4_BKWD, motorSpeed);
}

inline void motorsSTRAFE_LEFT() {
    motorsOFF();
    ledcWrite(CH_M1_BKWD, motorSpeed); ledcWrite(CH_M2_FWD,  motorSpeed);
    ledcWrite(CH_M3_FWD,  motorSpeed); ledcWrite(CH_M4_BKWD, motorSpeed);
}

inline void motorsSTRAFE_RIGHT() {
    motorsOFF();
    ledcWrite(CH_M1_FWD,  motorSpeed); ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed); ledcWrite(CH_M4_FWD,  motorSpeed);
}

inline void motorsROT_LEFT() {
    motorsOFF();
    ledcWrite(CH_M1_BKWD, motorSpeed); ledcWrite(CH_M3_BKWD, motorSpeed);
    ledcWrite(CH_M2_FWD,  motorSpeed); ledcWrite(CH_M4_FWD,  motorSpeed);
}

inline void motorsROT_RIGHT() {
    motorsOFF();
    ledcWrite(CH_M1_FWD,  motorSpeed); ledcWrite(CH_M3_FWD,  motorSpeed);
    ledcWrite(CH_M2_BKWD, motorSpeed); ledcWrite(CH_M4_BKWD, motorSpeed);
}

inline void motorsSetWheelPwm(int pwmM1, int pwmM2, int pwmM3, int pwmM4) {
    auto apply = [](int fwdCh, int bkwdCh, int pwm) {
        if (pwm >= 0) { ledcWrite(bkwdCh, 0);   ledcWrite(fwdCh,  pwm);  }
        else          { ledcWrite(fwdCh,  0);    ledcWrite(bkwdCh, -pwm); }
    };
    apply(CH_M1_FWD, CH_M1_BKWD, pwmM1);
    apply(CH_M2_FWD, CH_M2_BKWD, pwmM2);
    apply(CH_M3_FWD, CH_M3_BKWD, pwmM3);
    apply(CH_M4_FWD, CH_M4_BKWD, pwmM4);
}

inline void motorsInit() {
    ledcSetup(CH_M1_FWD,  PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M1_FWD,  CH_M1_FWD);
    ledcSetup(CH_M1_BKWD, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M1_BKWD, CH_M1_BKWD);
    ledcSetup(CH_M2_FWD,  PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M2_FWD,  CH_M2_FWD);
    ledcSetup(CH_M2_BKWD, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M2_BKWD, CH_M2_BKWD);
    ledcSetup(CH_M3_FWD,  PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M3_FWD,  CH_M3_FWD);
    ledcSetup(CH_M3_BKWD, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M3_BKWD, CH_M3_BKWD);
    ledcSetup(CH_M4_FWD,  PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M4_FWD,  CH_M4_FWD);
    ledcSetup(CH_M4_BKWD, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(M4_BKWD, CH_M4_BKWD);
    motorsOFF();
}

// ── Collection motor ──────────────────────────────────────────
inline void collectionInit() {
    gpio_config_t conf = {};
    conf.pin_bit_mask  = (1ULL << COLLECTION);  // gpio_config uses uint64_t
    conf.mode          = GPIO_MODE_OUTPUT;
    conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);
    GPIO.out1_w1tc.val = COLLECTION_MASK;  // GPIO32+ high bank
}

// ── Servo functions ───────────────────────────────────────────
inline void servosInit() {
    doorServo.setPeriodHertz(50);
    armServo.setPeriodHertz(50);
    doorServo.attach(SERVO_DOOR_PIN, SERVO_MIN_US, SERVO_MAX_US);  // takes ch 0
    armServo.attach(SERVO_ARM_PIN,   SERVO_MIN_US, SERVO_MAX_US);  // takes ch 1
    doorServo.write(SERVO_DOOR_CLOSED_ANGLE);
    armServo.write(SERVO_ARM_HOME_ANGLE);
    delay(250);
}

inline void openDoor(uint32_t waitMs = 0)  { doorServo.write(SERVO_DOOR_OPEN_ANGLE);   delay(waitMs); }
inline void closeDoor(uint32_t waitMs = 0) { doorServo.write(SERVO_DOOR_CLOSED_ANGLE); delay(waitMs); }

inline void dropFlag(uint32_t waitMs = 0) {
    armServo.write(SERVO_ARM_DROP_ANGLE);
    delay(waitMs);
}

inline void liftBin(uint32_t waitMs = 0) {
    armServo.write(SERVO_ARM_DROP_ANGLE); delay(waitMs);
    armServo.write(SERVO_ARM_LIFT_ANGLE); delay(waitMs);
}

inline void servoRotationTest(uint8_t cycles = 2, uint32_t dwellMs = 400) {
    for (uint8_t i = 0; i < cycles; i++) {
        openDoor(dwellMs); closeDoor(dwellMs);
        armServo.write(SERVO_ARM_HOME_ANGLE); delay(dwellMs);
        armServo.write(SERVO_ARM_DROP_ANGLE); delay(dwellMs);
        armServo.write(SERVO_ARM_LIFT_ANGLE); delay(dwellMs);
        armServo.write(SERVO_ARM_HOME_ANGLE); delay(dwellMs);
    }
}

// ── Photosensor ───────────────────────────────────────────────
// Returns raw 12-bit ADC value (0–4095).
inline int photoRead() { return analogRead(PHOTO_PIN); }

// Returns true when light level exceeds threshold.
inline bool photoIsLight(int threshold = 1000) { return analogRead(PHOTO_PIN) >= threshold; }

#endif // FAST_FUNCTIONS_H