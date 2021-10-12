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
 * configblock.c - Simple static implementation of the config block
 */
#define DEBUG_MODULE "CFGBLK"

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include "config.h"
#include "debug.h"
#include "configblock.h"
#include "eeprom.h"
#include "cal.h"

#include "_i2c.h"


/* Internal format of the config block */
#define MAGIC 0x43427830
#define VERSION 1
#define HEADER_SIZE_BYTES 5 // magic + version
#define OVERHEAD_SIZE_BYTES (HEADER_SIZE_BYTES + 1) // + cksum

// Current version
struct configblock_s {
  /* header */
  uint32_t magic;
  uint8_t  version;
  /* Content */
  uint8_t radioChannel;
  uint8_t radioSpeed;
  float calibPitch;
  float calibRoll;
  uint8_t radioAddress_upper;
  uint32_t radioAddress_lower;
  /* Simple modulo 256 checksum */
  uint8_t cksum;
} __attribute__((__packed__));

typedef struct configblock_s configblock_t;

static configblock_t configblock;
static configblock_t configblockDefault = {
    .magic = MAGIC,
    .version = VERSION,
    .radioChannel = RADIO_CHANNEL,
    .radioSpeed = RADIO_DATARATE,
    .calibPitch = 0.0,
    .calibRoll = 0.0,
    .radioAddress_upper = ((uint64_t)RADIO_ADDRESS >> 32),
    .radioAddress_lower = (RADIO_ADDRESS & 0xFFFFFFFFULL),
};

static bool isInit = false;
static bool cb_ok = false;

static bool configblockCheckChecksum(configblock_t *configblock);
static bool configblockWrite(configblock_t *configblock);

int configblockInit(void) {
  if (isInit)
    return 0;

  eepromInit();

  if (eepromTestConnection()) {
    if (eepromReadBuffer((uint8_t *)&configblock, 0, sizeof(configblock))) {
      // Verify the config block
      if (configblock.magic == MAGIC && configblock.version == VERSION &&
          configblockCheckChecksum(&configblock)) {
        DEBUG_PRINT("EEPROM: v%d, verification [OK]\n", configblock.version);
        cb_ok = true;
      } else
        DEBUG_PRINT("Verification [FAIL]: %lx %d\n", configblock.magic, configblock.version);
    }
  } else {
    DEBUG_PRINT("EEPROM Connection [FAIL]\n");
    return -1;
  }

  if (cb_ok == false) {
    DEBUG_PRINT("Try to write default configuration...\n");
    // Copy default data to used structure.
    memcpy((uint8_t *)&configblock, (uint8_t *)&configblockDefault, sizeof(configblock));
    // Write default configuration to eeprom
    if (configblockWrite(&configblockDefault))
      cb_ok = true;
    else {
      DEBUG_PRINT("Write failed!\n");
      return -1;
    }
  }

  isInit = true;
  return 0;
}

bool configblockTest(void) {
  return eepromTest();
}

static bool configblockCheckChecksum(configblock_t *configblock) {
  return (configblock->cksum == calculate_cksum(configblock, sizeof(configblock_t) - 1));
}

static bool configblockWrite(configblock_t *configblock) {
  // Write default configuration to eeprom
  configblock->cksum = calculate_cksum(configblock, sizeof(configblock_t) - 1);
  if (!eepromWriteBuffer((uint8_t *)configblock, 0, sizeof(configblock_t))) {
    return false;
  }

  return true;
}

/* Static accessors */
int configblockGetRadioChannel(void) {
  if (cb_ok)
    return configblock.radioChannel;
  else
    return RADIO_CHANNEL;
}

int configblockGetRadioSpeed(void) {
  if (cb_ok)
    return configblock.radioSpeed;
  else
    return RADIO_DATARATE;
}

uint64_t configblockGetRadioAddress(void) {
  if (cb_ok)
    return ((uint64_t)configblock.radioAddress_upper << 32) | (uint64_t)configblock.radioAddress_lower;
  else
    return RADIO_ADDRESS;
}

float configblockGetCalibPitch(void) {
  if (cb_ok)
    return configblock.calibPitch;
  else
    return 0;
}

float configblockGetCalibRoll(void) {
  if (cb_ok)
    return configblock.calibRoll;
  else
    return 0;
}
