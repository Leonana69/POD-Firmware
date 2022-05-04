/*
*    ||          ____  _ __
* +------+      / __ )(_) /_______________ _____  ___
* | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
* +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
*  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
*
* Crazyflie control firmware
*
* Copyright (C) 2021 Bitcraze AB
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
* supervisor.c - Keep track of system state
*/

#include <math.h>
#include <stdlib.h>

#include "log.h"
#include "pm.h"
#include "stabilizer.h"
#include "supervisor.h"
#include "debug.h"
#include "motors.h"

/* Minimum summed motor PWM that means we are flying */
#define SUPERVISOR_FLIGHT_THRESHOLD 4000
/* Number of times in a row we need to see a condition before acting upon it */
#define SUPERVISOR_HYSTERESIS_THRESHOLD 30

static bool isFlying = false;
static bool isTumbled = false;
static bool isLocked = true;

bool supervisorCanFly() {
	return (!isLocked) && (!isTumbled);
}

bool supervisorIsFlying() {
	return isFlying;
}

bool supervisorIsTumbled() {
	return isTumbled;
}

void supervisorUnlockDrone() {
	isLocked = false;
}

void supervisorLockDrone() {
	isLocked = true;
}

static bool isFlyingCheck() {
	int sumRatio = 0;
	for (int i = 0; i < 4; ++i)
		sumRatio += motorsGetValue(i);

	return sumRatio > SUPERVISOR_FLIGHT_THRESHOLD;
}

static bool isTumbledCheck(const sensorData_t *data) {
	const float tolerance = 0.1;
	static uint32_t hysteresis = 0;
	// We need a SUPERVISOR_HYSTERESIS_THRESHOLD amount of readings that indicate
	// that we are tumbled before we act on it. This is to reduce false positives.
	if (data->accel.z < tolerance) {
		if (hysteresis++ > SUPERVISOR_HYSTERESIS_THRESHOLD)
			return true;
	} else hysteresis = 0;
	return false;
}

void supervisorUpdate(const sensorData_t *data) {
	isFlying = isFlyingCheck();
	isTumbled = isTumbledCheck(data);

	if (isTumbled && isFlying)
		stabilizerSetEmergencyStop();
}

bool supervisorKalmanIsStateWithinBounds(const kalmanCoreData_t* this) {
	// TODO: set as define
	float maxPosition = 100; //meters
	float maxVelocity = 10; //meters per second
	for (int i = 0; i < 3; i++) {
		if (maxPosition > 0.0f) {
			if (this->S[KC_STATE_X + i] > maxPosition) {
				return false;
			} else if (this->S[KC_STATE_X + i] < -maxPosition) {
				return false;
			}
		}

		if (maxVelocity > 0.0f) {
			if (this->S[KC_STATE_PX + i] > maxVelocity) {
				return false;
			} else if (this->S[KC_STATE_PX + i] < -maxVelocity) {
				return false;
			}
		}
	}

  return true;
}

/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(spv)
LOG_ADD_CORE(LOG_UINT8, isFlying, &isFlying)
LOG_ADD_CORE(LOG_UINT8, isTumbled, &isTumbled)
LOG_GROUP_STOP(spv)
