//Code by Alejandro Choapa
/*
 * This file contains the class that owns the MPU6050: it initializes ranges/filters and runs a short stationary calibration.
 */

#pragma once
#include <Arduino.h> //includes arduino core types
#include "types.hpp"

class IMU //declaring the class
{
public:
  bool begin(uint8_t addr = 0x68); //INitializes the MPU6050 sensor
  //use a bool so the function returns false if the sensor is not found/ working
  void calibrate(uint16_t samples = 400);// calibrate for when the aircraft is stationary
  void update(float dtime);//reads and filters the data to update the roll/pitch/alt
  Attitude get() const { return att_; }//returns the latest attitude estimate

private:
  float gx_off_{}, gy_off_{}, gz_off_{}; //stores gyro biases for X/Y/Z computed by the calibrate function
  Attitude att_{}; // filters data
};
