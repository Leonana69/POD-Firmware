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

//The radio channel. From 0 to 125
#define RADIO_CHANNEL 80
#define RADIO_DATARATE RADIO_RATE_2M
#define RADIO_ADDRESS 0xE7E7E7E7E7ULL
#define RADIO_RATE_250K 0
#define RADIO_RATE_1M 1
#define RADIO_RATE_2M 2

#ifdef POD
  // #define DEBUG_PRINT_ON_UART
  #define BASE_THRUST 18000
  #define MIN_THRUST  1000
  #define MAX_THRUST  35000
  #define THRUST_SCALE 1000
  #define MOTORS_IDLE_THRUST 0
  #define POD_MASS (0.9f)

  #define CONTROLLER_TYPE CONTROLLER_PID
  #define ESTIMATOR_TYPE  ESTIMATOR_KALMAN
  /*! @brief Sensors config
   * SENSORS_TYPE:  SENSORS_BMI088_BMP388
   *                SENSORS_BMI270_BMP384
   * SENSORS_INTERFACE: SENSOR_INTF_I2C
   *                    SENSOR_INTF_SPI
   */
  #define SENSORS_TYPE SENSORS_BMI270_BMP384
  #define SENSORS_INTERFACE SENSOR_INTF_SPI

  #define FRONT_DIS_TYPE FRONT_DIS_VL53L1X
  /*! @brief Motor config
   * MOTOR_TYPE:  MOTORS_PWM
   *              MOTORS_DSHOT
   */
  #define MOTORS_TYPE MOTORS_DSHOT
  /*! TIM */
  #define USEC_TIM         htim7
  /* cf2.1 use t2c4 for m2 and t4c4 for m4,
   * for dshot to work, we need dma which is not available for t4c4, and it's shared between t2c2 and t2c4
   */
  #define MOTOR1_TIM       htim2
  #define MOTOR1_CHANNEL   TIM_CHANNEL_2
  #define MOTOR2_TIM       htim2
  #define MOTOR2_CHANNEL   TIM_CHANNEL_3
  #define MOTOR3_TIM       htim2
  #define MOTOR3_CHANNEL   TIM_CHANNEL_1
  #define MOTOR4_TIM       htim4
  #define MOTOR4_CHANNEL   TIM_CHANNEL_3

  #define MOTOR1_TIM_DMA_CC TIM_DMA_CC2
  #define MOTOR2_TIM_DMA_CC TIM_DMA_CC3
  #define MOTOR3_TIM_DMA_CC TIM_DMA_CC1
  #define MOTOR4_TIM_DMA_CC TIM_DMA_CC3

  /*! GPIO */
  #define NRF_FC_GPIO_Port  GPIOA
  #define NRF_FC_Pin        GPIO_PIN_4

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

  #define PMW3901_CS_Pin GPIO_PIN_4
  #define PMW3901_CS_GPIO_Port GPIOB

  #define SENSOR_CS_Pin GPIO_PIN_13
  #define SENSOR_CS_GPIO_Port GPIOC

  /*! I2C */
  #define eepromI2CHandle hi2c1
  #define sensorI2CHandle hi2c3
  #define tofI2CHandle hi2c1

  /*! SPI */
  #define PMW3901SpiHandle hspi1
  #define sensorSpiHandle hspi2

  /*! UART */
  #define debugUart       huart3
  #define nrfUart         huart6
#else // cf2
  #define BASE_THRUST 36000
  #define MIN_THRUST  1000
  #define MAX_THRUST  60000
  #define THRUST_SCALE 1000
  #define MOTORS_IDLE_THRUST 0
  #define POD_MASS (0.036f)

  #define CONTROLLER_TYPE CONTROLLER_PID
  #define ESTIMATOR_TYPE  ESTIMATOR_KALMAN
  /*! @brief Sensors config
   * SENSORS_TYPE:  SENSORS_BMI088_BMP388
   *                SENSORS_BMI270_BMP384
   * SENSORS_INTERFACE: SENSOR_INTF_I2C
   *                    SENSOR_INTF_SPI
   */
  #define SENSORS_TYPE SENSORS_BMI088_BMP388
  #define SENSORS_INTERFACE SENSOR_INTF_I2C

  #define FRONT_DIS_TYPE FRONT_DIS_NONE
  /*! @brief Motor config
   * MOTOR_TYPE:  MOTORS_PWM
   *              MOTORS_DSHOT
   */
  #define MOTORS_TYPE MOTORS_PWM
  /*! TIM */
  #define USEC_TIM         htim7

  #define MOTOR1_TIM       htim2
  #define MOTOR1_CHANNEL   TIM_CHANNEL_2
  #define MOTOR2_TIM       htim2
  #define MOTOR2_CHANNEL   TIM_CHANNEL_4
  #define MOTOR3_TIM       htim2
  #define MOTOR3_CHANNEL   TIM_CHANNEL_1
  #define MOTOR4_TIM       htim4
  #define MOTOR4_CHANNEL   TIM_CHANNEL_4

  /*! GPIO */
  #define NRF_FC_GPIO_Port  GPIOA
  #define NRF_FC_Pin        GPIO_PIN_4

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

  #define PMW3901_CS_Pin GPIO_PIN_4
  #define PMW3901_CS_GPIO_Port GPIOB

  #define SENSOR_CS_Pin GPIO_PIN_13
  #define SENSOR_CS_GPIO_Port GPIOC

  /*! I2C */
  #define eepromI2CHandle hi2c1
  #define sensorI2CHandle hi2c3
  #define tofI2CHandle hi2c1

  /*! SPI */
  #define PMW3901SpiHandle hspi1
  #define sensorSpiHandle hspi2

  /*! UART */
  #define debugUart       huart3
  #define nrfUart         huart6
#endif



// ****** DMA ******
#define nrfUartTxDmaHandle hdma_usart6_tx

/*
 * FreeRTOS
 */
// Task names
#define SYSTEM_TASK_NAME        "SYSTEM"
#define LEDSEQCMD_TASK_NAME     "LEDSEQCMD"
#define PM_TASK_NAME            "PWRMGNT"
#define CRTP_RADIO_TX_TASK_NAME "CRTP-R-TX"
#define CRTP_RADIO_RX_TASK_NAME "CRTP-R-RX"
#define CRTP_USB_TX_TASK_NAME "CRTP-U-TX"
#define CRTP_USB_RX_TASK_NAME "CRTP-U-RX"
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
#define BQ_OSD_TASK_NAME        "BQ_OSDTASK"
#define GTGPS_DECK_TASK_NAME    "GTGPS"
#define OA_DECK_TASK_NAME       "OA"
#define UART1_TEST_TASK_NAME    "UART1TEST"
#define UART2_TEST_TASK_NAME    "UART2TEST"
#define KALMAN_TASK_NAME        "KALMAN"
#define ACTIVE_MARKER_TASK_NAME "ACTIVEMARKER-DECK"
#define UART2_TASK_NAME         "UART2"
#define CRTP_SRV_TASK_NAME      "CRTP-SRV"
#define PLATFORM_SRV_TASK_NAME  "PLATFORM-SRV"
#define TOF_TASK_NAME           "TOF"
#define FRONT_DIS_TASK_NAME     "FDIS"

// Task priorities
#define STABILIZER_TASK_PRI     5
#define SENSORS_TASK_PRI        4
#define FLOW_TASK_PRI           3
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
#define OA_DECK_TASK_PRI        3
#define UART1_TEST_TASK_PRI     1
#define UART2_TEST_TASK_PRI     1
#define KALMAN_TASK_PRI         2
#define LEDSEQCMD_TASK_PRI      1

#define SYSLINK_TASK_PRI        3
#define USBLINK_TASK_PRI        3
#define ACTIVE_MARKER_TASK_PRI  3
#define UART2_TASK_PRI          3
#define CRTP_SRV_TASK_PRI       0
#define PLATFORM_SRV_TASK_PRI   0

#define TOF_TASK_PRI            2
#define FRONT_DIS_TASK_PRI      2

// Task stack sizes
#define SYSTEM_TASK_STACKSIZE         (2 * configMINIMAL_STACK_SIZE)
#define LEDSEQCMD_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define PM_TASK_STACKSIZE             configMINIMAL_STACK_SIZE
#define CRTP_RADIO_TX_TASK_STACKSIZE  configMINIMAL_STACK_SIZE
#define CRTP_RADIO_RX_TASK_STACKSIZE  (2 * configMINIMAL_STACK_SIZE)
#define CRTP_USB_TX_TASK_STACKSIZE    configMINIMAL_STACK_SIZE
#define CRTP_USB_RX_TASK_STACKSIZE    configMINIMAL_STACK_SIZE
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
#define ACTIVEMARKER_TASK_STACKSIZE   configMINIMAL_STACK_SIZE
#define UART2_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define CRTP_SRV_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define PLATFORM_SRV_TASK_STACKSIZE   configMINIMAL_STACK_SIZE

#define TOF_TASK_STACKSIZE            (2 * configMINIMAL_STACK_SIZE)
#define FRONT_DIS_TASK_STACKSIZE      configMINIMAL_STACK_SIZE

#endif /* __CONFIG_H__ */
