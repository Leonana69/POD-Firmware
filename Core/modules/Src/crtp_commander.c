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

static void commanderCrtpCB(CRTPPacket* pk);

void crtpCommanderInit() {
  if (isInit)
    return;

  crtpRegisterPortCB(CRTP_PORT_SETPOINT, commanderCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_SETPOINT_GENERIC, commanderCrtpCB);
  isInit = true;
}

bool crtpCommanderTest() {
	return isInit;
}

enum crtpCommanderChannel {
  DEFAULT_CHANNEL = 0,
  KEEP_ALIVE_CHANNEL = 1,
};

enum crtpSetpointGenericChannel {
  SET_SETPOINT_CHANNEL = 0,
  META_COMMAND_CHANNEL = 1,
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

/* To add a new packet:
 *   1 - Add a new type in the metaCommand_e enum.
 *   2 - Implement a decoder function with good documentation about the data
 *       structure and the intent of the packet.
 *   3 - Add the decoder function to the metaCommandDecoders array.
 *   4 - Create a new params group for your handler if necessary
 *   5 - Pull-request your change :-)
 */

/* ---===== 1 - metaCommand_e enum =====--- */
enum metaCommand_e {
  metaNotifySetpointsStop = 0,
  nMetaCommands,
};

typedef void (*metaCommandDecoder_t)(const void *data, size_t datalen);

/* ---===== 2 - Decoding functions =====--- */

/* notifySetpointsStop meta-command. See commander.h function
 * commanderNotifySetpointsStop() for description and motivation.
 */
struct notifySetpointsStopPacket {
  uint32_t remainValidMillisecs;
} __attribute__((packed));

void notifySetpointsStopDecoder(const void *data, size_t datalen) {
  ASSERT(datalen == sizeof(struct notifySetpointsStopPacket));
  const struct notifySetpointsStopPacket *values = data;
  commanderNotifySetpointsStop(values->remainValidMillisecs);
}

 /* ---===== packetDecoders array =====--- */
const static metaCommandDecoder_t metaCommandDecoders[] = {
  [metaNotifySetpointsStop] = notifySetpointsStopDecoder,
};

/* Decoder switch */
static void commanderCrtpCB(CRTPPacket* pk) {
  static setpoint_t setpoint;
  uint8_t metaCmd;

  stabilizerSetEmergencyStopTimeout(300);

  static int cnt = 0;

  if (pk->port == CRTP_PORT_SETPOINT) {
    switch (pk->channel) {
      default:
      case DEFAULT_CHANNEL:
        crtpCommanderRpytDecodeSetpoint(&setpoint, pk);
        break;
      case KEEP_ALIVE_CHANNEL:
        // debug
        if ((cnt++ % 10) == 0)
          DEBUG_PRINT_CONSOLE("k:%.2f\n", setpoint.position.z);
        break;
    }
    
    commanderSetSetpoint(&setpoint);
  } else if (pk->port == CRTP_PORT_SETPOINT_GENERIC) {
    switch (pk->channel) {
    case SET_SETPOINT_CHANNEL:
      crtpCommanderGenericDecodeSetpoint(&setpoint, pk);
      commanderSetSetpoint(&setpoint);
      // debug
      if (cnt > 30)
        DEBUG_PRINT_CONSOLE("s:%.2f\n", setpoint.position.z);
      break;
    case META_COMMAND_CHANNEL:
        metaCmd = pk->data[0];
        if (metaCmd < nMetaCommands && (metaCommandDecoders[metaCmd] != NULL))
          metaCommandDecoders[metaCmd](pk->data + 1, pk->size - 1);
      break;
    default:
      /* Do nothing */
      break;
    }
  }
}
