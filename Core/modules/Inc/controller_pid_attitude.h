#ifndef __CONTROLLER_PID_ATTITUDE_H__
#define __CONTROLLER_PID_ATTITUDE_H__

#include "stabilizer_types.h"

void controllerPidAttitudeInit();

void controllerPidAttitudeRateUpdate(rate_t measure, rate_t target, acc_t *output);

void controllerPidAttitudeValUpdate(attitude_t measure, attitude_t target, rate_t *output);

void controllerPidAttitudeValReset(uint8_t rpy);

void controllerPidAttitudeRateReset(uint8_t rpy);

void controllerPidAttitudeResetAll();

#endif
