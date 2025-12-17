//Code by Alejandro Choapa
//main.cpp minimal entry for your IMU + Stabilizer firmware, 5-servo + EMI read.

#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"
#include "types.hpp"
#include "imu.hpp"
#include "stabilizer.hpp"

static IMU imu;
static Stabilizer stab;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.begin(0x68)) {
    Serial.println(F("ERROR: IMU init failed"));
    while (true) { delay(500); }
  }

  // Map the 5 flight surfaces to PCA9685 channels
  stab.begin(CH_AIL_L, CH_AIL_R, CH_ELE_L, CH_ELE_R, CH_RUD);

  // Servo limits and neutral pulse
  stab.setLimits(SERVO_MID_US, SERVO_MIN_US, SERVO_MAX_US);

  // Mechanical directions
  stab.setPolarity(+1, -1, +1, +1, +1);

  // EMI pin
  pinMode(PIN_EMI, INPUT);
}

void loop() {
  static uint32_t last_us = micros();
  const uint32_t now_us = micros();
  const float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;

  // sensors for attitude
  imu.update(dt);
  const Attitude& att = imu.attitude();

  //stabilize and actuate 5 servos
  stab.step(dt, att);

  // EMI read
  const int raw = analogRead(PIN_EMI);
  const float emiNorm = raw / 1023.0f;

  // tie this into telemetry or a failsafe flag
  if (emiNorm > EMI_ALERT_LEVEL) {
    Serial.print(F("EMI HIGH: ")); Serial.println(emiNorm, 3);
  }

  // very light telemetry
  Serial.print(att.roll); Serial.print(',');
  // Serial.print(att.pitch); Serial.print(',');
  // Serial.print(att.yaw); Serial.print(',');
  // Serial.println(emiNorm, 3);
}


