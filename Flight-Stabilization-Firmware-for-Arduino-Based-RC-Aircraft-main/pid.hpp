//Code by Alejandro Choapa

/*
 * the main purpose of this header file is to receive the aircraft’s angular error per axis.
 * and then compute how much servo correction is needed.
 */

#pragma once

struct PID  //encapsulates all data and functions for a PID controller
{
  float ce{}; //current error
  float ae{}; //reacts to accumulated error
  float roe{}; // rate of change of error
  float iacc{}; //accumulates the integral term over time
  float stored_meas{}; // stores measured value to compute derivative term

  //umax & umin define the output limits
  float umin{-400.f};
  float umax{400.f};

  //imin and imax
  float imin{-300.f};
  float imax{300.f};

  // this function will compute new control outputs based on these variables
  float update(float setpoint, float measured_value, float dtime)
  {
    const float e = setpoint - measured_value; //calculates current error between setpoint and measurement
    iacc += ae * e * dtime; //Integrates error over time
    if (iacc > imax) iacc = imax;
    if (iacc < imin) iacc = imin;
    //computes term based on the rate of change
    const float d_meas = (measured_value - stored_meas) / dtime; // D on measurement
    stored_meas = measured_value; // stores the current measurement
    float u = ce*e - roe*d_meas + iacc; // computes controls
    if (u > umax) u = umax;
    if (u < umin) u = umin;
    return u; // returns control output
  }
  void reset() { iacc = 0.f; stored_meas = 0.f; }
};
