/*
 * code by Alejandro Choapa
 * this cpp file implements the flow: attach&center, polarity/limit setters, a safePulse() guard, and the axis update/writes
 */

#include <Arduino.h>
#include "config.hpp"
#include "servo_driver.hpp"
#include "stabilizer.hpp"

void Stabilizer::begin(int chAilL, int chAilR, int chEleL, int chEleR, int chRud)
{
  chAileronLeft_ = chAilL; chAileronRight_ = chAilR;
  chElevatorLeft_ = chEleL; chElevatorRight_ = chEleR;
  chRudder_  = chRud;

  servosInit();
  servosNeutral();
}

void Stabilizer::setLimits(int midUs, int minUs, int maxUs)
{
  mid_ = midUs; min_ = minUs; max_ = maxUs;
}

void Stabilizer::setPolarity(int dirAilL, int dirAilR, int dirEleL, int dirEleR, int dirRud)
{
  dAilL_ = (dirAilL >= 0) ? +1 : -1;
  dAilR_ = (dirAilR >= 0) ? +1 : -1;
  dEleL_ = (dirEleL >= 0) ? +1 : -1;
  dEleR_ = (dirEleR >= 0) ? +1 : -1;
  dRud_  = (dirRud  >= 0) ? +1 : -1;
}

void Stabilizer::setTrim(int tAilL, int tAilR, int tEleL, int tEleR, int tRud)
{
  trAilL_ = tAilL;
  trAilR_ = tAilR;
  trEleL_ = tEleL;
  trEleR_ = tEleR;
  trRud_  = tRud;
}

void Stabilizer::step(float /*dt*/, const Attitude& att)
{
  // Basic stabilizing commands around mid:
  int rollCmdUs  = mid_ - (int)(kRoll_ * att.roll);
  int pitchCmdUs = mid_ - (int)(kPitch_ * att.pitch);
  int yawCmdUs   = mid_ - (int)(kYaw_ * att.yaw);

  // Mixers
  // Ailerons are opposite sign L/R from rollCmdUs, then apply differential
  int ailL = mid_ + (int)((rollCmdUs - mid_) * GAIN_AIL);
  int ailR = mid_ - (int)((rollCmdUs - mid_) * GAIN_AIL);
  ailL = servoAileronDiffUs(ailL);
  ailR = servoAileronDiffUs(ailR);

  // Elevators both halves same from pitch
  int ele = mid_ + (int)((pitchCmdUs - mid_) * GAIN_ELE);
  int eleL = ele;
  int eleR = ele;

  // Rudder from yaw
  int rud = mid_ + (int)((yawCmdUs - mid_) * GAIN_RUD);

  // Polarity + trims + clamp
  ailL = servoShapeUs(ailL, dAilL_, trAilL_);
  ailR = servoShapeUs(ailR, dAilR_, trAilR_);
  eleL = servoShapeUs(eleL, dEleL_, trEleL_);
  eleR = servoShapeUs(eleR, dEleR_, trEleR_);
  rud  = servoShapeUs(rud , dRud_ , trRud_ );

  // Write to PCA9685 channels
  servoWriteUs(chAileronLeft_, ailL);
  servoWriteUs(chAileronRight_, ailR);
  servoWriteUs(chElevatorLeft_, eleL);
  servoWriteUs(chElevatorRight_, eleR);
  servoWriteUs(chRudder_, rud);
}
