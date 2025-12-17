//Code by Alejandro Choapa

#pragma once
#include <stdint.h>
#include "types.hpp"

class Stabilizer {
public:
  // Map flight surfaces to 5 outputs
  void begin(int chAilL, int chAilR, int chEleL, int chEleR, int chRud);

  // Pulse limits and center
  void setLimits(int midUs, int minUs, int maxUs);

  // Mechanical direction per surface
  void setPolarity(int dirAilL, int dirAilR, int dirEleL, int dirEleR, int dirRud);
  void setTrim(int tAilL, int tAilR, int tEleL, int tEleR, int tRud);

  // One control that applies simple proportional damping and mixing
  void step(float dt, const Attitude& att);

private:
  // Channels
  int chAileronLeft_ = 0, chAileronRight_ = 1, chElevatorLeft_ = 2, chElevatorRight_ = 3, chRud_ = 4;

  // Limits
  int mid_ = 1500, min_ = 1000, max_ = 2000;

  // Polarity
  int dAilL_ = +1, dAilR_ = -1, dEleL_ = +1, dEleR_ = +1, dRud_ = +1;

  // Trims
  int trAilL_ = 0, trAilR_ = 0, trEleL_ = 0, trEleR_ = 0, trRud_ = 0;


  float kRoll_  = 6.0f;
  float kPitch_ = 6.0f;
  float kYaw_   = 6.0f;
};
