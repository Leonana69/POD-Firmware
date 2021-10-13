/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * power_distribution.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "cal.h"
#include "motors.h"
#include "debug.h"
#include "config.h"

static bool isInit = false;
static bool motorSetEnable = false;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

// TODO: test this value for brushless motor
static uint16_t idleThrust = MOTORS_IDLE_THRUST;

void powerDistributionInit() {
	if (isInit)
		return;

  motorsInit();
	powerStop();
	
	isInit = true;
}

bool powerDistributionTest() {
  return isInit && motorsTest();
}

void powerStop() {
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistributionUpdate(const control_t *control) {
	/*! QUAD_FORMATION_X */
	float r = control->roll / 2.0f;
	float p = control->pitch / 2.0f;
	motorPower.m1 = capValueUint16((uint32_t)(control->thrust - r + p + control->yaw));
	motorPower.m2 = capValueUint16((uint32_t)(control->thrust - r - p - control->yaw));
	motorPower.m3 = capValueUint16((uint32_t)(control->thrust + r - p + control->yaw));
	motorPower.m4 = capValueUint16((uint32_t)(control->thrust + r + p - control->yaw));

  if (motorSetEnable) {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  } else {
    if (motorPower.m1 < idleThrust)
      motorPower.m1 = idleThrust;
    if (motorPower.m2 < idleThrust)
      motorPower.m2 = idleThrust;
    if (motorPower.m3 < idleThrust)
      motorPower.m3 = idleThrust;
    if (motorPower.m4 < idleThrust)
      motorPower.m4 = idleThrust;

    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}

/*! @brief Override power distribution to motors. */
PARAM_GROUP_START(motorPowerSet)

/*! @brief Nonzero to override controller with set values */
PARAM_ADD_CORE(PARAM_UINT8, enable, &motorSetEnable)
/*! @brief motor power for m1, m2, m3, m4: `0 - UINT16_MAX` */
PARAM_ADD_CORE(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD_CORE(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD_CORE(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD_CORE(PARAM_UINT16, m4, &motorPowerSet.m4)

PARAM_GROUP_STOP(motorPowerSet)

/*! @brief Power distribution parameters */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

/*! @brief Motor output related log variables. */
LOG_GROUP_START(motor)
/*! @brief Motor power (PWM value) for M1, M2, M3, M4 [0 - UINT16_MAX] */
LOG_ADD_CORE(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD_CORE(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD_CORE(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD_CORE(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
