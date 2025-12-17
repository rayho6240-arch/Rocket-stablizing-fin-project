/*
 * code by Alejandro Choapa
 * this file holds 3 PID controllers (roll/pitch/yaw), three Servo objects, axis polarity flags, and safe pulse limits.
 * Throught each step it computes values from the PIDs, inverts elevator by convention, clamps to mins and maxs, and writes clean pulses
 */

#pragma once
#include <Servo.h> // arduino servo library
#include "pid.hpp"
#include "types.hpp"

class Stabilizer //begins the class
{
public:
  void begin(int pinAil, int pinEle, int pinRud); // initializes the 3 servors
  void setPolarity(float sRoll, float sPitch, float sYaw); // modifies each axis so directions can be flip
  void setLimits(int center, int min_us, int max_us); // modifies pulse width and hard min/max
  void step(float dt, const Attitude& att); // zero setpoints (auto-level)

  //
  PID& roll(){ return roll_;}
  PID& pitch(){ return pitch_;}
  PID& yaw(){ return yaw_;}

private:
  Servo aileron, hstabilizer, rudder; //servos that represent horizontal and vertical stabilizer and rudder
  //aileron: roll an left/right bank
  //hstabilizer: pitch, nose up/down
  //rudder: control nore left/right


  PID roll_{1.0f,0.02f,0.015f}, pitch_{1.0f,0.02f,0.015f}, yaw_{0.8f,0.01f,0.f}; // controls per axis
  float sRoll_{+1.f}, sPitch_{+1.f}, sYaw_{+1.f}; //needed for servo orientation, linkage geometry, and IMU frame.
  int center_{1500}, min_{1100}, max_{1900};
  // started with 1000-2000 but throught testing it was concluded that the values most optimal values for frame and efficiency are 1100-1900
  int safePulse(int center, float d) const; //function that converts data from PID into an integer pulse width
};
