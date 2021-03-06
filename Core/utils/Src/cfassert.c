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
 * cfassert.c - Assert implementation
 */

#define DEBUG_MODULE "SYS"

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "cfassert.h"
#include "led.h"
#include "power_distribution.h"
#include "debug.h"

#define MAGIC_ASSERT_INDICATOR 0x2f8a001f

enum snapshotType_e {
  snapshotTypeNone = 0,
  snapshotTypeFile = 1,
  snapshotTypeHardFault = 2,
  snapshotTypeText = 3,
};

typedef struct SNAPSHOT_DATA {
  uint32_t magicNumber;
  enum snapshotType_e type;
  union {
    struct {
      const char* fileName;
      int line;
    } file;
    struct {
      unsigned int r0;
      unsigned int r1;
      unsigned int r2;
      unsigned int r3;
      unsigned int r12;
      unsigned int lr;
      unsigned int pc;
      unsigned int psr;
    } hardfault;
    struct {
      const char* text;
    } text;
  };
} SNAPSHOT_DATA;

// The .nzds section is not cleared at startup, data here will survive a
// reset (by the watch dog for instance)
SNAPSHOT_DATA snapshot __attribute__((section(".nzds"))) = {
  .magicNumber = 0,
  .type = snapshotTypeNone,
};


void assertFail(char *exp, char *file, int line) {
  portDISABLE_INTERRUPTS();
  storeAssertFileData(file, line);
  DEBUG_PRINT("Assert failed %s:%d\n", file, line);
  ledClearAll();
  ledSet(ERR_LED1, 1);
  ledSet(ERR_LED2, 1);
  powerStop();
  HAL_NVIC_SystemReset();
}

void storeAssertFileData(const char *file, int line) {
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = snapshotTypeFile;
  snapshot.file.fileName = file;
  snapshot.file.line = line;
}

void storeAssertHardfaultData(
  unsigned int r0,
  unsigned int r1,
  unsigned int r2,
  unsigned int r3,
  unsigned int r12,
  unsigned int lr,
  unsigned int pc,
  unsigned int psr) {
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = snapshotTypeHardFault;
  snapshot.hardfault.r0 = r0;
  snapshot.hardfault.r1 = r1;
  snapshot.hardfault.r2 = r2;
  snapshot.hardfault.r3 = r3;
  snapshot.hardfault.r12 = r12;
  snapshot.hardfault.lr = lr;
  snapshot.hardfault.pc = pc;
  snapshot.hardfault.psr = psr;
}

void storeAssertTextData(const char *text) {
  snapshot.magicNumber = MAGIC_ASSERT_INDICATOR;
  snapshot.type = snapshotTypeText;
  snapshot.text.text = text;
}

static void clearAssertData() {
  snapshot.type = snapshotTypeNone;
}

void printAssertSnapshotData() {
  if (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) {
    switch (snapshot.type) {
      case snapshotTypeFile:
        DEBUG_PRINT("Assert failed at %s:%d\n", snapshot.file.fileName, snapshot.file.line);
        break;
      case snapshotTypeHardFault:
        DEBUG_PRINT("Hardfault. r0: %X, r1: %X, r2: %X, r3: %X, r12: %X, lr: %X, pc: %X, psr: %X\n",
          snapshot.hardfault.r0,
          snapshot.hardfault.r1,
          snapshot.hardfault.r2,
          snapshot.hardfault.r3,
          snapshot.hardfault.r12,
          snapshot.hardfault.lr,
          snapshot.hardfault.pc,
          snapshot.hardfault.psr);
        break;
      case snapshotTypeText:
        DEBUG_PRINT("Assert failed: %s\n", snapshot.text.text);
        break;
      default:
        DEBUG_PRINT("Assert failed, but unknown type\n");
        break;
    }
  } else {
    DEBUG_PRINT("No assert information found\n");
  }
}

static bool isAssertRegistered() {
  return (MAGIC_ASSERT_INDICATOR == snapshot.magicNumber) && (snapshot.type != snapshotTypeNone);
}

bool cfAssertNormalStartTest(void) {
  bool wasNormalStart = true;

	if (isAssertRegistered()) {
		wasNormalStart = false;
		DEBUG_PRINT("The system resumed after a failed assert [WARNING]\n");
		printAssertSnapshotData();
    clearAssertData();
	}

	return wasNormalStart;
}
