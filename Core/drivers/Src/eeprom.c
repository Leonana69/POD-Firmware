/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
 * Copyright (C) 2011-2020 Bitcraze AB
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
 * @file eeprom.c
 * Driver for the 24AA64F eeprom.
 *
 */
#define DEBUG_MODULE "EEPROM"

#include <string.h>
#include "cmsis_os2.h"

#include "eeprom.h"
#include "debug.h"
#include "eprintf.h"
#include "mem.h"

#include "_i2c.h"


static uint32_t handleMemGetSize(void) { return EEPROM_SIZE; }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_EEPROM,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

static uint8_t devAddr;
static I2CDrv *I2Cx;
static bool isInit;

bool eepromInit() {
  if (isInit)
    return true;

  memoryRegisterHandler(&memDef);

  I2Cx = &eepromI2C;
  devAddr = EEPROM_I2C_ADDR;
  isInit = true;

  return true;
}

bool eepromTest(void) {
  return eepromTestConnection();
}

bool eepromTestConnection(void) {
  uint8_t tmp;

  if (!isInit)
    return false;

  return i2cMemReadDma16(I2Cx, devAddr, 0, 1, &tmp);
}

bool eepromReadBuffer(uint8_t* buffer, uint16_t readAddr, uint16_t len) {
  if ((uint32_t)readAddr + len > EEPROM_SIZE)
    return false;

  return i2cMemReadDma16(I2Cx, devAddr, readAddr, len, buffer);
}

bool eepromWriteBuffer(const uint8_t* buffer, uint16_t writeAddr, uint16_t len) {
  bool status = false;

  unsigned char pageBuffer[32];
  int bufferIndex = 0;
  int leftToWrite = len;
  uint16_t currentAddress = writeAddr;
  uint16_t pageAddress = writeAddr;

  if ((uint32_t)writeAddr + len > EEPROM_SIZE)
     return false;

  while (leftToWrite > 0) {
    int pageIndex = 0;
    pageAddress = currentAddress;
    do {
      pageBuffer[pageIndex++] = buffer[bufferIndex++];
      leftToWrite -= 1;
      currentAddress += 1;
    } while ((leftToWrite > 0) && (currentAddress % 32 != 0));

    // Writing page
    for (int retry = 0; retry < 10; retry++) {
      status = i2cMemWrite16(I2Cx, devAddr, pageAddress, pageIndex, pageBuffer);
      if (status)
        break;
			osDelay(6);
    }
    if (!status)
      return false;

    // Waiting for page to be written
    for (int retry = 0; retry < 30; retry++) {
      uint8_t dummy;
			status = i2cMemWrite16(I2Cx, devAddr, 0xFFFF, 1, &dummy);
      if (status)
        break;
			osDelay(1);
    }
    if (!status)
      return false;
  }

  return status;
}

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  if (memAddr + readLen <= EEPROM_SIZE) {
    if (eepromReadBuffer(buffer, memAddr, readLen)) {
      result = true;
    }
  }

  return result;
}

static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  if (memAddr + writeLen <= EEPROM_SIZE) {
    if (eepromWriteBuffer(buffer, memAddr, writeLen)) {
      result = true;
    }
  }

  return result;
}
