/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.c - implementation of the PID regulator
 */

// TODO: remove this
#ifdef IMPROVED_BARO_Z_HOLD
#define PID_FILTER_ALL
#endif

#include "pid.h"
#include "cal.h"

void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter) {
  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit = DEFAULT_PID_OUTPUT_LIMIT;
  pid->dt = dt;
	pid->rdt = 1.0 / dt;
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError) {
    float output = 0.0f;

    if (updateError)
        pid->error = pid->desired - measured;

    output += pid->kp * pid->error;

    float deriv = (pid->error - pid->prevError) * pid->rdt;
    #ifdef PID_FILTER_ALL
      pid->deriv = deriv;
    #else
      if (pid->enableDFilter){
        pid->deriv = lpf2pApply(&pid->dFilter, deriv);
      } else {
        pid->deriv = deriv;
      }
    #endif
    if (isnan(pid->deriv)) {
      pid->deriv = 0;
    }

    output += pid->kd * pid->deriv;

    pid->integ += pid->error * pid->dt;

    // Constrain the integral (unless the iLimit is zero)
    if (pid->iLimit != 0)
    	pid->integ = fConstrain(pid->integ, -pid->iLimit, pid->iLimit);

    output += pid->ki * pid->integ;
    
    #ifdef PID_FILTER_ALL
      //filter complete output instead of only D component to compensate for increased noise from increased barometer influence
      if (pid->enableDFilter)
        output = lpf2pApply(&pid->dFilter, output);
      else
        output = output;

      if (isnan(output))
        output = 0;
     #endif

    // Constrain the total PID output (unless the outputLimit is zero)
    if (pid->outputLimit != 0)
      output = fConstrain(output, -pid->outputLimit, pid->outputLimit);

    pid->prevError = pid->error;
    return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
  pid->iLimit = limit;
}


void pidReset(PidObject* pid) {
  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
}

void pidSetDesired(PidObject* pid, const float desired) {
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid) {
  return pid->desired;
}

bool pidIsActive(PidObject* pid) {
  bool isActive = true;

  if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
    isActive = false;

  return isActive;
}

void filterReset(PidObject* pid, const float samplingRate, const float cutoffFreq, bool enableDFilter) {
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
}
