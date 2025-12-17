/*
 * by Alejandro Choapa
 * this header file is for pin assignments and simple scaling constants
 */

#pragma once

// ===== PCA9685 settings =====
#ifndef PCA9685_I2C_ADDR
#define PCA9685_I2C_ADDR   0x40
#endif
#ifndef PCA9685_SERVO_HZ
#define PCA9685_SERVO_HZ   50
#endif

// ===== Servo pulse limits (µs) =====
#ifndef SERVO_MIN_US
#define SERVO_MIN_US       1000
#endif
#ifndef SERVO_MAX_US
#define SERVO_MAX_US       2000
#endif
#ifndef SERVO_MID_US
#define SERVO_MID_US       1500
#endif

// ===== PCA9685 channel mapping =====
#ifndef CH_AIL_L
#define CH_AIL_L  0
#endif
#ifndef CH_AIL_R
#define CH_AIL_R  1
#endif
#ifndef CH_ELE_L
#define CH_ELE_L  2
#endif
#ifndef CH_ELE_R
#define CH_ELE_R  3
#endif
#ifndef CH_RUD
#define CH_RUD    4
#endif

// ===== Mixer options =====
#ifndef AIL_DIFF
#define AIL_DIFF   0.20f   // 0..1
#endif
#ifndef GAIN_AIL
#define GAIN_AIL   1.0f
#endif
#ifndef GAIN_ELE
#define GAIN_ELE   1.0f
#endif
#ifndef GAIN_RUD
#define GAIN_RUD   1.0f
#endif

// ===== EMI input (NEW) =====
#ifndef PIN_EMI
#define PIN_EMI A0           // analog pin for EMI probe
#endif
#ifndef EMI_ALERT_LEVEL
#define EMI_ALERT_LEVEL 0.85f // normalized 0..1—raise alert if above
#endif
