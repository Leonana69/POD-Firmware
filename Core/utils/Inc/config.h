/**
 *    ||        ____  _ __  ______
 * +------+    / __ )(_) /_/  ____/_________ ____  ____
 * | 0xBC |   / __ /  / __/ /    / ___/ __ `/_  / / _  \
 * +------+  / /_/ / / /_/ /___ / /  / /_/ / / /_/   __/
 *  ||  ||  /_____/_/\__/\____//_/   \__,_/ /___/ \___/
 *
 * CrazyLoader firmware
 *
 * Copyright (C) 2011-2013 Bitcraze AB
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
 * config.h - CrazyLoader config include file
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "usec_timer.h"
#define PROTOCOL_VERSION 4
#ifdef STM32F4XX
  #define QUAD_FORMATION_X

  #define CONFIG_BLOCK_ADDRESS    (2048 * (64 - 1))
  #define MCU_ID_ADDRESS          0x1FFF7A10
  #define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
  #ifndef FREERTOS_HEAP_SIZE
    #define FREERTOS_HEAP_SIZE      30000
  #endif
  // #define FREERTOS_MIN_STACK_SIZE 150       // M4-FPU register setup is bigger so stack needs to be bigger
  // #define FREERTOS_MCU_CLOCK_HZ   168000000

  // #define configGENERATE_RUN_TIME_STATS 1
  // #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() usecTimerInit()
  // #define portGET_RUN_TIME_COUNTER_VALUE() usecTimerStamp()
#endif

#define WATCHDOG_RESET_PERIOD_MS 80

#define MIN_THRUST  1000
#define MAX_THRUST  60000

#define CONTROLLER_TYPE controllerTypePID

//The radio channel. From 0 to 125
#define RADIO_CHANNEL 80
#define RADIO_DATARATE RADIO_RATE_2M
#define RADIO_ADDRESS 0xE7E7E7E7E7ULL
#define RADIO_RATE_250K 0
#define RADIO_RATE_1M 1
#define RADIO_RATE_2M 2

/*
 * Peripheral
 */

// ****** UART ******
// #define uartMain        huart6
#define debugUart       huart3
#define nrfUart         huart6

// ****** TIM ******
#define usecTim         htim7

// ****** I2C ******
#define eepromI2CHandle hi2c1
#define sensorI2CHandle hi2c3

// ****** DMA ******
#define nrfUartTxDmaHandle hdma_usart6_tx
/*
 * GPIO
 */
#define NRF_FC_GPIO_Port  GPIOA
#define NRF_FC_Pin        GPIO_PIN_4
#define NRF_TX_GPIO_Port  GPIOC
#define NRF_TX_Pin        GPIO_PIN_7

#define GREEN_L_Pin       GPIO_PIN_0
#define GREEN_L_GPIO_Port GPIOC
#define RED_L_Pin         GPIO_PIN_1
#define RED_L_GPIO_Port   GPIOC
#define GREEN_R_Pin       GPIO_PIN_2
#define GREEN_R_GPIO_Port GPIOC
#define RED_R_Pin         GPIO_PIN_3
#define RED_R_GPIO_Port   GPIOC
#define BLUE_L_Pin        GPIO_PIN_2
#define BLUE_L_GPIO_Port  GPIOD

#define osDelayMax        0xFFFF

/*
 * FreeRTOS
 */
// Task names
#define SYSTEM_TASK_NAME        "SYSTEM"
#define LEDSEQCMD_TASK_NAME     "LEDSEQCMD"
#define ADC_TASK_NAME           "ADC"
#define PM_TASK_NAME            "PWRMGNT"
#define CRTP_TX_TASK_NAME       "CRTP-TX"
#define CRTP_RX_TASK_NAME       "CRTP-RX"
#define CRTP_RXTX_TASK_NAME     "CRTP-RXTX"
#define LOG_TASK_NAME           "LOG"
#define MEM_TASK_NAME           "MEM"
#define PARAM_TASK_NAME         "PARAM"
#define SENSORS_TASK_NAME       "SENSORS"
#define STABILIZER_TASK_NAME    "STABILIZER"
#define NRF24LINK_TASK_NAME     "NRF24LINK"
#define ESKYLINK_TASK_NAME      "ESKYLINK"
#define SYSLINK_TASK_NAME       "SYSLINK"
#define USBLINK_TASK_NAME       "USBLINK"
#define PROXIMITY_TASK_NAME     "PROXIMITY"
#define EXTRX_TASK_NAME         "EXTRX"
#define UART_RX_TASK_NAME       "UART"
#define ZRANGER_TASK_NAME       "ZRANGER"
#define ZRANGER2_TASK_NAME      "ZRANGER2"
#define FLOW_TASK_NAME          "FLOW"
#define USDLOG_TASK_NAME        "USDLOG"
#define USDWRITE_TASK_NAME      "USDWRITE"
#define PCA9685_TASK_NAME       "PCA9685"
#define CMD_HIGH_LEVEL_TASK_NAME "CMDHL"
#define MULTIRANGER_TASK_NAME   "MR"
#define BQ_OSD_TASK_NAME        "BQ_OSDTASK"
#define GTGPS_DECK_TASK_NAME    "GTGPS"
#define LIGHTHOUSE_TASK_NAME    "LH"
#define LPS_DECK_TASK_NAME      "LPS"
#define OA_DECK_TASK_NAME       "OA"
#define UART1_TEST_TASK_NAME    "UART1TEST"
#define UART2_TEST_TASK_NAME    "UART2TEST"
#define KALMAN_TASK_NAME        "KALMAN"
#define ACTIVE_MARKER_TASK_NAME "ACTIVEMARKER-DECK"
#define AI_DECK_GAP_TASK_NAME   "AI-DECK-GAP"
#define AI_DECK_NINA_TASK_NAME  "AI-DECK-NINA"
#define UART2_TASK_NAME         "UART2"
#define CRTP_SRV_TASK_NAME      "CRTP-SRV"
#define PLATFORM_SRV_TASK_NAME  "PLATFORM-SRV"

// Task priorities
#define STABILIZER_TASK_PRI     5
#define SENSORS_TASK_PRI        4
#define ADC_TASK_PRI            3
#define FLOW_TASK_PRI           3
#define MULTIRANGER_TASK_PRI    3
#define SYSTEM_TASK_PRI         2
#define CRTP_TX_TASK_PRI        2
#define CRTP_RX_TASK_PRI        2
#define EXTRX_TASK_PRI          2
#define ZRANGER_TASK_PRI        2
#define ZRANGER2_TASK_PRI       2
#define LOG_TASK_PRI            1
#define MEM_TASK_PRI            1
#define PARAM_TASK_PRI          1
#define PROXIMITY_TASK_PRI      0
#define PM_TASK_PRI             0
#define USDLOG_TASK_PRI         1
#define USDWRITE_TASK_PRI       0
#define PCA9685_TASK_PRI        2
#define CMD_HIGH_LEVEL_TASK_PRI 2
#define BQ_OSD_TASK_PRI         1
#define GTGPS_DECK_TASK_PRI     1
#define LIGHTHOUSE_TASK_PRI     3
#define LPS_DECK_TASK_PRI       3
#define OA_DECK_TASK_PRI        3
#define UART1_TEST_TASK_PRI     1
#define UART2_TEST_TASK_PRI     1
#define KALMAN_TASK_PRI         2
#define LEDSEQCMD_TASK_PRI      1

#define SYSLINK_TASK_PRI        3
#define USBLINK_TASK_PRI        3
#define ACTIVE_MARKER_TASK_PRI  3
#define AI_DECK_TASK_PRI        3
#define UART2_TASK_PRI          3
#define CRTP_SRV_TASK_PRI       0
#define PLATFORM_SRV_TASK_PRI   0

// Task stack sizes
#define SYSTEM_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define LEDSEQCMD_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define ADC_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define PM_TASK_STACKSIZE             configMINIMAL_STACK_SIZE
#define CRTP_TX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define CRTP_RX_TASK_STACKSIZE        (2* configMINIMAL_STACK_SIZE)
#define CRTP_RXTX_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define LOG_TASK_STACKSIZE            (2 * configMINIMAL_STACK_SIZE)
#define MEM_TASK_STACKSIZE            (2 * configMINIMAL_STACK_SIZE)
#define PARAM_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define SENSORS_TASK_STACKSIZE        (2 * configMINIMAL_STACK_SIZE)
#define STABILIZER_TASK_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
#define NRF24LINK_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define ESKYLINK_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define SYSLINK_TASK_STACKSIZE        (2 * configMINIMAL_STACK_SIZE)
#define USBLINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define PROXIMITY_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define EXTRX_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define UART_RX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define ZRANGER_TASK_STACKSIZE        (2 * configMINIMAL_STACK_SIZE)
#define ZRANGER2_TASK_STACKSIZE       (2 * configMINIMAL_STACK_SIZE)
#define FLOW_TASK_STACKSIZE           (2 * configMINIMAL_STACK_SIZE)
#define USDLOG_TASK_STACKSIZE         (2 * configMINIMAL_STACK_SIZE)
#define USDWRITE_TASK_STACKSIZE       (3 * configMINIMAL_STACK_SIZE)
#define PCA9685_TASK_STACKSIZE        (2 * configMINIMAL_STACK_SIZE)
#define CMD_HIGH_LEVEL_TASK_STACKSIZE (2 * configMINIMAL_STACK_SIZE)
#define MULTIRANGER_TASK_STACKSIZE    (2 * configMINIMAL_STACK_SIZE)
#define ACTIVEMARKER_TASK_STACKSIZE   configMINIMAL_STACK_SIZE
#define AI_DECK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define UART2_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define CRTP_SRV_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define PLATFORM_SRV_TASK_STACKSIZE   configMINIMAL_STACK_SIZE

#endif /* __CONFIG_H__ */
