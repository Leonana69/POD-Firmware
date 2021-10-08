#ifndef __CONTROLLER_PID_POSITION_H__
#define __CONTROLLER_PID_POSITION_H__

#include "stabilizer_types.h"

void controllerPidPositionInit();

void controllerPidPositionUpdate(float* thrust, attitude_t *attitude,
																 setpoint_t *setpoint, const state_t *state);

void controllerPidPositionValReset(uint8_t rpy);

void controllerPidPositionRateReset(uint8_t rpy);

void controllerPidPositionResetAll(bool resetFilter);

#endif
