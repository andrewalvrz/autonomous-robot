#ifndef FAST_FUNCTIONS_H
#define FAST_FUNCTIONS_H

#include <Arduino.h>
#include <driver/gpio.h>
#include "soc/gpio_struct.h"

#define M1_FWD  19   // was 18
#define M1_BKWD 18   // was 19
#define M2_FWD  17   // was 16
#define M2_BKWD 16   // was 17
#define M3_FWD  26   // was 27
#define M3_BKWD 27   // was 26
#define M4_FWD  12   // was 13
#define M4_BKWD 13   // was 12

// This is to mask all the gpio pins simultaneously using bitwise ORs
// exp: GPIO pin 2 needs to be enabled (1), In a 32 bit system : 0010 -pin 2 enabled.
constexpr uint32_t ALL_MOTOR_MASK = (1UL << M1_FWD) | (1UL << M1_BKWD) | (1UL << M2_FWD) | (1UL << M2_BKWD) |
                                    (1UL << M3_FWD) | (1UL << M3_BKWD) | (1UL << M4_FWD) | (1UL << M4_BKWD);

constexpr uint32_t FWD_MASK =       (1UL << M1_FWD) | (1UL << M2_FWD) | (1UL << M3_FWD) | (1UL << M4_FWD);

constexpr uint32_t BKWD_MASK =      (1UL << M1_BKWD) | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_BKWD);

// Holonomic strafing (aligns with original motors_left/motors_right mapping)
constexpr uint32_t STRAFE_LEFT_MASK  = (1UL << M1_BKWD) | (1UL << M2_FWD)  | (1UL << M3_FWD)  | (1UL << M4_BKWD);
constexpr uint32_t STRAFE_RIGHT_MASK = (1UL << M1_FWD)  | (1UL << M2_BKWD) | (1UL << M3_BKWD) | (1UL << M4_FWD);

// In-place rotation (matches motors_Rleft/motors_Rright)
constexpr uint32_t ROT_LEFT_MASK  = (1UL << M1_BKWD) | (1UL << M3_BKWD) | (1UL << M2_FWD)  | (1UL << M4_FWD);
constexpr uint32_t ROT_RIGHT_MASK = (1UL << M1_FWD)  | (1UL << M3_FWD)  | (1UL << M2_BKWD) | (1UL << M4_BKWD);

// LEDC PWM setup
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_RES_BITS = 8;
constexpr uint8_t CH_M1_FWD = 0;
constexpr uint8_t CH_M1_BKWD = 1;
constexpr uint8_t CH_M2_FWD = 2;
constexpr uint8_t CH_M2_BKWD = 3;
constexpr uint8_t CH_M3_FWD = 4;
constexpr uint8_t CH_M3_BKWD = 5;
constexpr uint8_t CH_M4_FWD = 6;
constexpr uint8_t CH_M4_BKWD = 7;
static uint8_t motorSpeed = 255;

// Inline lets the functions compile to direct registers instead of pushing arguments or returning (optimized)
inline void motorsSetSpeed(uint8_t speed){
    motorSpeed = speed;
}

inline void motorsOFF(){
    ledcWrite(CH_M1_FWD, 0);
    ledcWrite(CH_M1_BKWD, 0);
    ledcWrite(CH_M2_FWD, 0);
    ledcWrite(CH_M2_BKWD, 0);
    ledcWrite(CH_M3_FWD, 0);
    ledcWrite(CH_M3_BKWD, 0);
    ledcWrite(CH_M4_FWD, 0);
    ledcWrite(CH_M4_BKWD, 0);
}
inline void motorsFWD(){
    ledcWrite(CH_M1_BKWD, 0);
    ledcWrite(CH_M2_BKWD, 0);
    ledcWrite(CH_M3_BKWD, 0);
    ledcWrite(CH_M4_BKWD, 0);
    ledcWrite(CH_M1_FWD, motorSpeed);
    ledcWrite(CH_M2_FWD, motorSpeed);
    ledcWrite(CH_M3_FWD, motorSpeed);
    ledcWrite(CH_M4_FWD, motorSpeed);
}
inline void motorsBKWD(){
    ledcWrite(CH_M1_FWD, 0);
    ledcWrite(CH_M2_FWD, 0);
    ledcWrite(CH_M3_FWD, 0);
    ledcWrite(CH_M4_FWD, 0);
    ledcWrite(CH_M1_BKWD, motorSpeed);
    ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed);
    ledcWrite(CH_M4_BKWD, motorSpeed);
}
inline void motorsSTRAFE_LEFT(){
    motorsOFF();
    ledcWrite(CH_M1_BKWD, motorSpeed);
    ledcWrite(CH_M2_FWD, motorSpeed);
    ledcWrite(CH_M3_FWD, motorSpeed);
    ledcWrite(CH_M4_BKWD, motorSpeed);
}
inline void motorsSTRAFE_RIGHT(){
    motorsOFF();
    ledcWrite(CH_M1_FWD, motorSpeed);
    ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed);
    ledcWrite(CH_M4_FWD, motorSpeed);
}
inline void motorsROT_LEFT(){
    motorsOFF();
    ledcWrite(CH_M1_BKWD, motorSpeed);
    ledcWrite(CH_M3_BKWD, motorSpeed);
    ledcWrite(CH_M2_FWD, motorSpeed);
    ledcWrite(CH_M4_FWD, motorSpeed);
}
inline void motorsROT_RIGHT(){
    motorsOFF();
    ledcWrite(CH_M1_FWD, motorSpeed);
    ledcWrite(CH_M3_FWD, motorSpeed);
    ledcWrite(CH_M2_BKWD, motorSpeed);
    ledcWrite(CH_M4_BKWD, motorSpeed);
}

inline void motorsSetWheelPwm(int pwmM1, int pwmM2, int pwmM3, int pwmM4) {
    auto apply = [](int fwdCh, int bkwdCh, int pwm) {
        if (pwm >= 0) {
            ledcWrite(bkwdCh, 0);
            ledcWrite(fwdCh, pwm);
        } else {
            ledcWrite(fwdCh, 0);
            ledcWrite(bkwdCh, -pwm);
        }
    };

    apply(CH_M1_FWD, CH_M1_BKWD, pwmM1);
    apply(CH_M2_FWD, CH_M2_BKWD, pwmM2);
    apply(CH_M3_FWD, CH_M3_BKWD, pwmM3);
    apply(CH_M4_FWD, CH_M4_BKWD, pwmM4);
}

inline void motorsInit(){
    ledcSetup(CH_M1_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M1_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M2_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M2_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M3_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M3_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M4_FWD, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(CH_M4_BKWD, PWM_FREQ_HZ, PWM_RES_BITS);

    ledcAttachPin(M1_FWD, CH_M1_FWD);
    ledcAttachPin(M1_BKWD, CH_M1_BKWD);
    ledcAttachPin(M2_FWD, CH_M2_FWD);
    ledcAttachPin(M2_BKWD, CH_M2_BKWD);
    ledcAttachPin(M3_FWD, CH_M3_FWD);
    ledcAttachPin(M3_BKWD, CH_M3_BKWD);
    ledcAttachPin(M4_FWD, CH_M4_FWD);
    ledcAttachPin(M4_BKWD, CH_M4_BKWD);
    motorsOFF();
}

//April Tag sensing. 
#endif