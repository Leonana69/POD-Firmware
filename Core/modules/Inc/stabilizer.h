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
 * stabilizer.h: Stabilizer orchestrator
 */
#ifndef __STABILIZER_H__
#define __STABILIZER_H__

#include <stdbool.h>
#include <stdint.h>

#define EMERGENCY_STOP_TIMEOUT_DISABLED (-1)

void stabilizerInit();
bool stabilizerTest();

/**
 * Enable emergency stop, will shut-off energy to the motors.
 */
void stabilizerSetEmergencyStop();

/**
 * Disable emergency stop, will enable energy to the motors.
 */
void stabilizerResetEmergencyStop();

/**
 * Restart the countdown until emergercy stop will be enabled.
 *
 * @param timeout Timeout in stabilizer loop tick. The stabilizer loop rate is
 *                RATE_MAIN_LOOP.
 */
void stabilizerSetEmergencyStopTimeout(int timeout);

#endif /* __STABILIZER_H__ */
