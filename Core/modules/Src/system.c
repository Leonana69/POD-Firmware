/*
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
 * system.c - Top level module implementation
 */
#define DEBUG_MODULE "SYS"

#include "_usart.h"
#include "_tim.h"
#include "_i2c.h"
#include "_spi.h"

// TODO: add IWDG
// #include "iwdg.h"
#include "cmsis_os2.h"
#include "debug.h"
#include "led.h"
// #include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "pm.h"

#include "system.h"
#include "usec_timer.h"
// #include "platform.h"
// #include "storage.h"
#include "configblock.h"
#include "worker.h"
#include "crtp.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "console.h"
// #include "usblink.h"
#include "mem.h"
#include "sysload.h"
// #include "proximity.h"
// #include "watchdog.h"
// #include "deck.h"
// #include "extrx.h"
#include "tof.h"
#include "flow.h"
#include "static_mem.h"
#include "cfassert.h"

#include <string.h>

#include "sensors_bmi088_bmp388.h"

/* Private variable */
static bool selftestPassed;
static bool isInit = false;

static char nrf_version[16];

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/*! System wide synchronisation */
STATIC_MEM_SEMAPHORE_ALLOC(canStartSemaphore);

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void) {
  _UART_Init();
  _TIM_Init();
  _I2C_Init();
  _SPI_Init();
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

// This must be the first module to be initialized!
static void systemInit(void) {
  if (isInit)
    return;

  STATIC_SEMAPHORE_CREATE(canStartSemaphore, 1, 0);
  ledInit();
  ledSet(CHG_LED, 1);
  usecTimerInit();
  commInit();
  configblockInit();
  workerInit();
  ledseqInit();
  pmInit();
  sysLoadInit();
  commanderInit();
  memInit();
  stabilizerInit();
  tofInit();
  flowInit();

  DEBUG_PRINT("----------------------------\n");
  DEBUG_PRINT_CONSOLE("Console Init.\n");
  
  DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS + 8)), *((int*)(MCU_ID_ADDRESS + 4)),
              *((int*)(MCU_ID_ADDRESS + 0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

  isInit = true;
}

static bool systemTest() {
  bool pass = isInit;
  pass &= commTest();
  pass &= configblockTest();
  pass &= workerTest();
  pass &= ledseqTest();
  pass &= pmTest();
  pass &= sysLoadTest();
  pass &= commanderTest();
  pass &= memTest();
  pass &= stabilizerTest();
  pass &= tofTest();
  pass &= flowTest();
  return pass;
}

void systemTask(void *arg) {
  /*! Init all modules */
  systemInit();
  // systemRequestNRFVersion();

  /* Start the firmware */
  if (systemTest()) {
    DEBUG_PRINT("Self test passed!\n");
    selftestPassed = 1;
    systemStart();
    ledseqRun(&seq_alive);
    ledseqRun(&seq_testPassed);
  } else {
    selftestPassed = 0;
    if (systemTest()) {
      while (1) {
        ledseqRun(&seq_testFailed);
        osDelay(2000);
        // System can be forced to start by setting the param to 1 from the cfclient
        if (selftestPassed) {
	        DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    } else {
      ledInit();
      ledSet(SYS_LED, true);
    }
  }
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  workerLoop();

  // Should never reach this point!
  DEBUG_PRINT("RUN INTO WRONG POINT!\n");
  while (1)
    osDelay(osWaitForever);
}


/* Global system variables */
void systemStart() {
  osSemaphoreRelease(canStartSemaphore);
#ifndef DEBUG
  // TODO: init IWDG
  // watchdogInit();
#endif
}

void systemWaitStart(void) {
  // This permits to guarantee that the system task is initialized before other
  // tasks waits for the start event.
  while (!isInit)
    osDelay(10);
  osSemaphoreAcquire(canStartSemaphore, osWaitForever);
  osSemaphoreRelease(canStartSemaphore);
}

void systemRequestShutdown() {
  SyslinkPacket slp;

  slp.type = SYSLINK_PM_ONOFF_SWITCHOFF;
  slp.length = 0;
  syslinkSendPacket(&slp);
}

void systemRequestNRFVersion() {
  SyslinkPacket slp;

  slp.type = SYSLINK_SYS_NRF_VERSION;
  slp.length = 0;
  syslinkSendPacket(&slp);
}

void systemSyslinkReceive(SyslinkPacket *slp) {
  if (slp->type == SYSLINK_SYS_NRF_VERSION) {
    size_t len = slp->length - 2;

    if (sizeof(nrf_version) - 1 <=  len) {
      len = sizeof(nrf_version) - 1;
    }
    memcpy(&nrf_version, &slp->data[0], len);
    DEBUG_PRINT("NRF51 version: %s\n", nrf_version);
  }
}

/*
 * This function must be defined if set configUSE_IDLE_HOOK = 1
 */
// TODO: realize this in freeRTOS.c
// void vApplicationIdleHook( void ) {
//   // DEBUG_PRINT("%d\n", xTaskGetTickCount());
//   static uint32_t tickOfLatestWatchdogReset = M2T(0);

//   portTickType tickCount = xTaskGetTickCount();

//   if (tickCount - tickOfLatestWatchdogReset > M2T(WATCHDOG_RESET_PERIOD_MS)) {
//     tickOfLatestWatchdogReset = tickCount;
//     DEBUG_PRINT("IWDG\n");
//     // TODO: add IWDG
//     // HAL_IWDG_Refresh(&hiwdg);
//   }

//   // Enter sleep mode. Does not work when debugging chip with SWD.
//   // Currently saves about 20mA STM32F405 current consumption (~30%).
// #ifndef DEBUG
//   { __asm volatile ("wfi"); }
// #endif
// }

/**
 * This parameter group contain read-only parameters pertaining to the CPU
 * in the Crazyflie.
 *
 * These could be used to identify an unique quad.
 */

PARAM_GROUP_START(cpu)

/**
 * @brief Size in kB of the device flash memory
 */
PARAM_ADD_CORE(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)

/**
 * @brief Byte `0 - 3` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS + 0)

/**
 * @brief Byte `4 - 7` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS + 4)

/**
 * @brief Byte `8 - 11` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS + 8)

PARAM_GROUP_STOP(cpu)

PARAM_GROUP_START(system)

/**
 * @brief All tests passed when booting
 */
PARAM_ADD_CORE(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)

PARAM_GROUP_STOP(sytem)
