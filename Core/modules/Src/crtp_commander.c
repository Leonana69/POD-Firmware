/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
#include <stddef.h>

#include "crtp_commander.h"

#include "cfassert.h"
#include "commander.h"
#include "crtp.h"
#include "stabilizer.h"
#include "debug.h"

static bool isInit = false;
static setpoint_t setpoint;
static void commanderSetpointCrtpCB(CRTPPacket* pk);
static void commanderGenericCrtpCB(CRTPPacket* pk);

void crtpCommanderInit() {
	if (isInit)
		return;

	crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderSetpointCrtpCB);
	crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderGenericCrtpCB);
	isInit = true;
}

bool crtpCommanderTest() {
	return isInit;
}

enum crtpSetpointGenericChannel {
	SET_SETPOINT_CHANNEL = 0,
	SET_SETPOINT_USB_CHANNEL = 1,
	KEEP_ALIVE_CHANNEL = 2,
	CONFIG_USB_CHANNEL = 3,
};

/* Channel 1 of the generic commander port is used for "meta-commands"
 * that alter the behavior of the commander itself, e.g. mode switching.
 * Although we use the generic commander port due to increasing pressure on the
 * 4-bit space of ports numbers, meta-commands that are unrelated to
 * streaming generic setpoint control modes are permitted.
 *
 * The packet format for meta-commands is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * TYPE is an 8-bit value. The remainder of the data depends on the command.
 * The maximum data size is 29 bytes.
 */

/* Decoder switch */

void commanderSetpointCrtpCB(CRTPPacket* pk) {
	if (pk->channel == SET_SETPOINT_CHANNEL) {
		stabilizerSetEmergencyStopTimeout(300);
		crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
		commanderSetSetpoint(&setpoint);
	} else if (pk->channel == SET_SETPOINT_USB_CHANNEL) {
		crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
		commanderSetSetpoint(&setpoint);
	}
}

void commanderGenericCrtpCB(CRTPPacket* pk) {
	if (pk->channel != SET_SETPOINT_USB_CHANNEL)
		stabilizerSetEmergencyStopTimeout(300);

	static int cnt = 0;
	static bool enableUsbControl = false;

	switch (pk->channel) {
	case SET_SETPOINT_CHANNEL:
		crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
		commanderSetSetpoint(&setpoint);
		break;
	case SET_SETPOINT_USB_CHANNEL:
		if (enableUsbControl) {
			crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
			commanderSetSetpoint(&setpoint);
		}
		break;
	case KEEP_ALIVE_CHANNEL:
		// debug
		if ((cnt++ % 10) == 0)
		DEBUG_PRINT_CONSOLE("k:%.2f,%.1f\n", setpoint.velocity.x, setpoint.velocity.y);
		commanderSetSetpoint(&setpoint);
		break;
	case CONFIG_USB_CHANNEL:
		enableUsbControl = (bool) pk->data[0];
		DEBUG_PRINT_CONSOLE("cuc:%d\n", pk->data[0]);
		break;
	default:
		/*! Do nothing */
		DEBUG_PRINT_CONSOLE("Invalid commander type!\n");
		break;
	}
}
