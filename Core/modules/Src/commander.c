/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 */
#include <string.h>

#include "commander.h"
#include "cal.h"
#include "crtp_commander.h"
// #include "crtp_commander_high_level.h"

// #include "cf_math.h"
#include "param.h"
#include "static_mem.h"

static bool isInit = false;

static state_t lastState;
static uint32_t lastUpdate;
static bool enableHighLevel = false;

STATIC_MEM_MUTEX_ALLOC(setpointMutex);

const static setpoint_t nullSetpoint;
static setpoint_t currentSetpoint;

/* Public functions */
void commanderInit() {
	STATIC_MUTEX_CREATE(setpointMutex);
	currentSetpoint = nullSetpoint;

  crtpCommanderInit();
	// TODO: fix high level
  // crtpCommanderHighLevelInit();
  lastUpdate = osKernelGetTickCount();

  isInit = true;
}

void commanderSetSetpoint(setpoint_t *setpoint) {
  osMutexAcquire(setpointMutex, osWaitForever);

  setpoint->timestamp = osKernelGetTickCount();
  memcpy(&currentSetpoint, setpoint, sizeof(setpoint_t));
 
  osMutexRelease(setpointMutex);
  // Send the high-level planner to idle so it will forget its current state
  // and start over if we switch from low-level to high-level in the future.
  // TODO: fix high level
  // crtpCommanderHighLevelStop();
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state) {
	memcpy(setpoint, &currentSetpoint, sizeof(setpoint_t));
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = osKernelGetTickCount();

  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
		// TODO: enable high level
    // if (enableHighLevel) {
    //   crtpCommanderHighLevelGetSetpoint(setpoint, state);
    // }
    // if (!enableHighLevel || crtpCommanderHighLevelIsStopped()) {
    //   memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    // }
  } else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    // Keep Z as it is
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;

    // guojun: clear setpoint
    memset(setpoint, 0, sizeof(setpoint_t));
  }
  // This copying is not strictly necessary because stabilizer.c already keeps
  // a static state_t containing the most recent state estimate. However, it is
  // not accessible by the public interface.
  lastState = *state;
}

bool commanderTest() {
  return isInit;
}

uint32_t commanderGetInactivityTime() {
  return osKernelGetTickCount() - lastUpdate;
}

/**
 *
 * The high level commander handles the setpoints from within the firmware
 * based on a predefined trajectory. This was merged as part of the
 * [Crazyswarm](%https://crazyswarm.readthedocs.io/en/latest/) project of the
 * [USC ACT lab](%https://act.usc.edu/) (see this
 * [blogpost](%https://www.bitcraze.io/2018/02/merging-crazyswarm-functionality-into-the-official-crazyflie-firmware/)).
 * The high-level commander uses a planner to generate smooth trajectories
 * based on actions like ‘take off’, ‘go to’ or ‘land’ with 7th order
 * polynomials. The planner generates a group of setpoints, which will be
 * handled by the High level commander and send one by one to the commander
 * framework.
 *
 * It is also possible to upload your own custom trajectory to the memory of
 * the Crazyflie, which you can try out with the script
 * `examples/autonomous_sequence_high_level of.py` in the Crazyflie python
 * library repository.
 */
PARAM_GROUP_START(commander)

/**
 *  @brief Enable high level commander
 */
PARAM_ADD_CORE(PARAM_UINT8, enHighLevel, &enableHighLevel)

PARAM_GROUP_STOP(commander)
