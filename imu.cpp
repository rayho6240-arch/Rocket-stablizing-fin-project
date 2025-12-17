/*
 * Code by Alejandro Choapa
 * This file implements the initialization, bias averaging, and the exact fusion math and yaw
 */


#include "imu.hpp"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <cmath>

static Adafruit_MPU6050 mpu;             // TU-local driver instance
static constexpr float R2D   = 180.0f / 3.14159265f;
static constexpr float ALPHA = 0.98f;

bool IMU::begin(uint8_t addr) {
  if (!mpu.begin(addr)) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  delay(100);
  calibrate(400);
  return true;
}

void IMU::calibrate(uint16_t N) {
  sensors_event_t a, g, t;
  float sx = 0.f, sy = 0.f, sz = 0.f;

  for (uint16_t i = 0; i < N; ++i) {
    mpu.getEvent(&a, &g, &t);
    sx += g.gyro.x;
    sy += g.gyro.y;
    sz += g.gyro.z;
    delay(2);
  }

  const float invN = 1.0f / static_cast<float>(N);
  gx_off_ = (sx * invN) * R2D;
  gy_off_ = (sy * invN) * R2D;
  gz_off_ = (sz * invN) * R2D;
}

void IMU::update(float dt) {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Bias corrected gyro rates in degrees
  const float gx = g.gyro.x * R2D - gx_off_;
  const float gy = g.gyro.y * R2D - gy_off_;
  const float gz = g.gyro.z * R2D - gz_off_;

  // Roll/pitch from accelerometer, consider gravity
  const float roll_acc  = atan2f(a.acceleration.y, a.acceleration.z) * R2D;
  const float pitch_acc = atan2f(
                            -a.acceleration.x,
                            sqrtf(a.acceleration.y * a.acceleration.y +
                                  a.acceleration.z * a.acceleration.z)) * R2D;

  // Complementary filter: fuse gyro (fast) with accel (absolute)
  att_.roll  = ALPHA * (att_.roll  + gx * dt) + (1.0f - ALPHA) * roll_acc;
  att_.pitch = ALPHA * (att_.pitch + gy * dt) + (1.0f - ALPHA) * pitch_acc;

  // Yaw integrates gyro Z (will drift without magnetometer)
  att_.yaw += gz * dt;

  // Wrap yaw to [-180, 180]
  if (att_.yaw >  180.f) att_.yaw -= 360.f;
  if (att_.yaw < -180.f) att_.yaw += 360.f;
}
