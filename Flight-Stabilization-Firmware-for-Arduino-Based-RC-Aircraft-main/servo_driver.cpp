//Code by Alejandro Choapa
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.hpp"
#include "servo_driver.hpp"

static Adafruit_PWMServoDriver pwm(PCA9685_I2C_ADDR);

static inline uint16_t usToTicks(int us) {
  if (us < 500)  us = 500;     // hard floor
  if (us > 2500) us = 2500;    // hard ceiling
  float ticks = us * 0.2048f;
  if (ticks < 0)    ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)(ticks + 0.5f);
}

void servosInit() {
  pwm.begin();
  pwm.setPWMFreq(PCA9685_SERVO_HZ);
  delay(10);
}

void servosNeutral() {
  // Only neutralize the five channels we use by default
  pwm.setPWM(CH_AIL_L, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_AIL_R, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_ELE_L, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_ELE_R, 0, usToTicks(SERVO_MID_US));
  pwm.setPWM(CH_RUD  , 0, usToTicks(SERVO_MID_US));
}

void servoWriteUs(uint8_t ch, int us) {
  // clamp to configured range first
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  pwm.setPWM(ch, 0, usToTicks(us));
}

int servoClampUs(int us) {
  if (us < SERVO_MIN_US) return SERVO_MIN_US;
  if (us > SERVO_MAX_US) return SERVO_MAX_US;
  return us;
}

int servoShapeUs(int baseUs, int dir, int trimUs) {
  int delta = baseUs - SERVO_MID_US;
  int shaped = SERVO_MID_US + (dir * delta) + trimUs;
  return servoClampUs(shaped);
}

int servoAileronDiffUs(int cmdUs) {
  int delta = cmdUs - SERVO_MID_US;
  if (delta > 0) {
    delta = (int)(delta * (1.0f + AIL_DIFF));
  } else {
    delta = (int)(delta * (1.0f - AIL_DIFF));
  }
  return servoClampUs(SERVO_MID_US + delta);
}
