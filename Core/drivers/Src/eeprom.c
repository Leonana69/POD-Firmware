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

#include "FreeRTOS.h"
#include "task.h"
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
  bool status;
  status = eepromTestConnection();
  if (status)
    DEBUG_PRINT("I2C connection [OK].\n");
  else
    DEBUG_PRINT("I2C connection [FAIL].\n");
  return status;
}

bool eepromTestConnection(void) {
  uint8_t tmp;
  HAL_StatusTypeDef status;

  if (!isInit)
    return false;

	DEBUG_PRINT("EEPROM TEST CONNECTION: 0x%x\n", devAddr);

  status = I2CRead16(I2Cx, devAddr, 0, 1, &tmp);
  return status == HAL_OK;
}

bool eepromReadBuffer(uint8_t* buffer, uint16_t readAddr, uint16_t len) {
  HAL_StatusTypeDef status;

  if ((uint32_t)readAddr + len > EEPROM_SIZE)
    return false;

  status = I2CRead16(I2Cx, devAddr, readAddr, len, buffer);

  return status == HAL_OK;
}

bool eepromWriteBuffer(const uint8_t* buffer, uint16_t writeAddr, uint16_t len) {
  HAL_StatusTypeDef status = HAL_ERROR;;

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
      status = I2CWrite16(I2Cx, devAddr, pageAddress, pageIndex, pageBuffer);
      if (status == HAL_OK)
        break;
      // vTaskDelay(M2T(6));
			osDelay(6);
    }
    if (status != HAL_OK)
      return false;

    // Waiting for page to be written
    for (int retry = 0; retry < 30; retry++) {
      uint8_t dummy;
      // status = i2cdevWrite(I2Cx, devAddr, 1, &dummy);
			status = I2CWrite16(I2Cx, devAddr, 0xFFFF, 1, &dummy);
      if (status == HAL_OK)
        break;
      // vTaskDelay(M2T(1));
			osDelay(1);
    }
    if (status != HAL_OK)
      return false;
  }

  return status == HAL_OK;
}

bool eepromWritePage(uint8_t* buffer, uint16_t writeAddr) {
 //TODO: implement
  return false;
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
