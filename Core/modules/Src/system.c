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

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// TODO: add IWDG
// #include "iwdg.h"
#include "cmsis_os2.h"
#include "debug.h"
#include "led.h"
// #include "version.h"
#include "config.h"
// #include "param.h"
// #include "log.h"
// #include "ledseq.h"
// #include "pm.h"

#include "system.h"
#include "usec_timer.h"
// #include "platform.h"
// #include "storage.h"
#include "configblock.h"
#include "worker.h"
#include "crtp.h"
// #include "freeRTOSdebug.h"
// #include "uart_syslink.h"
// #include "uart1.h"
// #include "uart2.h"
// #include "comm.h"
// #include "stabilizer.h"
// #include "commander.h"
// #include "console.h"
// #include "usblink.h"
// #include "mem.h"
// #include "proximity.h"
// #include "watchdog.h"
// #include "queuemonitor.h"
// #include "buzzer.h"
// #include "sound.h"
// #include "sysload.h"
// #include "estimator_kalman.h"
// #include "deck.h"
// #include "extrx.h"
// #include "app.h"
#include "static_mem.h"
// #include "peer_localization.h"
// #include "i2cdev.h"
#include "cfassert.h"

#ifndef START_DISARMED
#define ARM_INIT true
#else
#define ARM_INIT false
#endif

/* Private variable */
static bool selftestPassed;
static bool armed = ARM_INIT;
static bool forceArm;
static bool isInit;

// static char nrf_version[16];

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* System wide synchronisation */
STATIC_MEM_MUTEX_ALLOC(canStartMutex);

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void) {
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

// This must be the first module to be initialized!
void systemInit(void) {
  if (isInit)
    return;

  canStartMutex = STATIC_MUTEX_CREATE(canStartMutex);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  // usblinkInit();
  // sysLoadInit();

  /* Initialized here so that DEBUG_PRINT (buffered) can be used early */
  // I don't like the SEGGER debugger
  // debugInit();
  crtpInit();
  // consoleInit();

  DEBUG_PRINT("----------------------------\n");
  // DEBUG_PRINT("%s is up and running!\n", platformConfigGetDeviceTypeName());
  // guojun: UART debug
  
  // if (V_PRODUCTION_RELEASE) {
  //   DEBUG_PRINT("Production release %s\n", V_STAG);
  // } else {
  //   DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
  //               V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  // }
  DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS + 8)), *((int*)(MCU_ID_ADDRESS + 4)),
              *((int*)(MCU_ID_ADDRESS + 0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

  configblockInit();
  // storageInit();
  workerInit();
  // adcInit();
  // ledseqInit();
  // pmInit();
  // TODO: remove these unnessasory modules
  // buzzerInit();
  // peerLocalizationInit();

#ifdef APP_ENABLED
  appInit();
#endif

  isInit = true;
}

bool systemTest() {
  bool pass = isInit;
  // TODO: enable test
  // pass &= ledseqTest();
  // pass &= pmTest();
  // pass &= workerTest();
  // pass &= buzzerTest();
  return pass;
}

/* Private functions implementation */

void systemTask(void *arg) {
  bool pass = true;

  ledInit();
  ledSet(CHG_LED, 1);

#ifdef DEBUG_QUEUE_MONITOR
  queueMonitorInit();
#endif

#ifdef ENABLE_UART1
  uart1Init(9600);
#endif

#ifdef ENABLE_UART2
  uart2Init(115200);
#endif
  // TODO: check if this is essential
  usecTimerInit();

  // init by stm32cubemx
  // i2cdevInit(I2C3_DEV);
  // i2cdevInit(I2C1_DEV);

  // // Init the high-levels modules
  systemInit();
  // commInit();
  // commanderInit();

  // StateEstimatorType estimator = anyEstimator;
  // estimatorKalmanTaskInit();
  // deckInit();
  // estimator = deckGetRequiredEstimator();
  // stabilizerInit(estimator);
  // if (deckGetRequiredLowInterferenceRadioMode() && platformConfigPhysicalLayoutAntennasAreClose())
  //   platformSetLowInterferenceRadioMode();

  // soundInit();
  // memInit();

#ifdef PROXIMITY_ENABLED
  proximityInit();
#endif

  // systemRequestNRFVersion();

  /* Test the modules */
  // DEBUG_PRINT("About to run tests in system.c.\n");
  // if (systemTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("system [FAIL]\n");
  // }
  // if (configblockTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("configblock [FAIL]\n");
  // }
  // if (storageTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("storage [FAIL]\n");
  // }
  // if (commTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("comm [FAIL]\n");
  // }
  // if (commanderTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("commander [FAIL]\n");
  // }
  // if (stabilizerTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("stabilizer [FAIL]\n");
  // }
  // if (estimatorKalmanTaskTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("estimatorKalmanTask [FAIL]\n");
  // }
  // if (deckTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("deck [FAIL]\n");
  // }
  // if (soundTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("sound [FAIL]\n");
  // }
  // if (memTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("mem [FAIL]\n");
  // }
  // if (watchdogNormalStartTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("watchdogNormalStart [FAIL]\n");
  // }
  // if (cfAssertNormalStartTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("cfAssertNormalStart [FAIL]\n");
  // }
  // if (peerLocalizationTest() == false) {
  //   pass = false;
  //   DEBUG_PRINT("peerLocalization [FAIL]\n");
  // }

  /* Start the firmware */
  if (pass) {
    DEBUG_PRINT("Self test passed!\n");
    selftestPassed = 1;
    systemStart();
    // TODO: add sound and ledseq
    // soundSetEffect(SND_STARTUP);
    // ledseqRun(&seq_alive);
    // ledseqRun(&seq_testPassed);
  } else {
    selftestPassed = 0;
    if (systemTest()) {
      while(1) {
        // TODO: add ledseq
        // ledseqRun(&seq_testFailed);
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
  while(1)
    vTaskDelay(portMAX_DELAY);
}


/* Global system variables */
void systemStart() {
  DEBUG_PRINT("SYSTEM START\n ");
  xSemaphoreGive(canStartMutex);
#ifndef DEBUG
  // TODO: init IWDG
  // watchdogInit();
#endif
}

void systemWaitStart(void) {
  // This permits to guarantee that the system task is initialized before other
  // tasks waits for the start event.
  while (!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

void systemSetArmed(bool val) {
  armed = val;
}

bool systemIsArmed() {
  return armed || forceArm;
}

// void systemRequestShutdown() {
//   SyslinkPacket slp;

//   slp.type = SYSLINK_PM_ONOFF_SWITCHOFF;
//   slp.length = 0;
//   syslinkSendPacket(&slp);
// }

// void systemRequestNRFVersion() {
//   SyslinkPacket slp;

//   slp.type = SYSLINK_SYS_NRF_VERSION;
//   slp.length = 0;
//   syslinkSendPacket(&slp);
// }

// void systemSyslinkReceive(SyslinkPacket *slp)
// {
//   if (slp->type == SYSLINK_SYS_NRF_VERSION)
//   {
//     size_t len = slp->length - 2;

//     if (sizeof(nrf_version) - 1 <=  len) {
//       len = sizeof(nrf_version) - 1;
//     }
//     memcpy(&nrf_version, &slp->data[0], len);
//     DEBUG_PRINT("NRF51 version: %s\n", nrf_version);
//   }
// }

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
// TODO: add PARAM
// PARAM_GROUP_START(cpu)

// /**
//  * @brief Size in kB of the device flash memory
//  */
// PARAM_ADD_CORE(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)

// /**
//  * @brief Byte `0 - 3` of device unique id
//  */
// PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS+0)

// /**
//  * @brief Byte `4 - 7` of device unique id
//  */
// PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS+4)

// /**
//  * @brief Byte `8 - 11` of device unique id
//  */
// PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS+8)

// PARAM_GROUP_STOP(cpu)

// PARAM_GROUP_START(system)

// /**
//  * @brief All tests passed when booting
//  */
// PARAM_ADD_CORE(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)

// /**
//  * @brief Set to nonzero to force system to be armed
//  */
// PARAM_ADD(PARAM_INT8, forceArm, &forceArm)

// PARAM_GROUP_STOP(sytem)

// /**
//  *  System loggable variables to check different system states.
//  */
// LOG_GROUP_START(sys)
// /**
//  * @brief If zero, arming system is preventing motors to start
//  */
// LOG_ADD(LOG_INT8, armed, &armed)
// LOG_GROUP_STOP(sys)