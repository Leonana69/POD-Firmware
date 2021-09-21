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

#include "usec_time.h"

#ifdef STM32F4XX
  #define QUAD_FORMATION_X

  #define CONFIG_BLOCK_ADDRESS    (2048 * (64 - 1))
  #define MCU_ID_ADDRESS          0x1FFF7A10
  #define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
  #ifndef FREERTOS_HEAP_SIZE
    #define FREERTOS_HEAP_SIZE      30000
  #endif
  #define FREERTOS_MIN_STACK_SIZE 150       // M4-FPU register setup is bigger so stack needs to be bigger
  #define FREERTOS_MCU_CLOCK_HZ   168000000

  #define configGENERATE_RUN_TIME_STATS 1
  #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() initUsecTimer()
  #define portGET_RUN_TIME_COUNTER_VALUE() usecTimestamp()
#endif


/*
 * Peripheral
 */

// ****** UART ******
// #define uartMain        huart6
#define uartDebug       huart3

// ****** TIM ******
#define usecTim         htim7

/*
 * GPIO
 */
#define NRF_FC_PIN_PORT GPIOA
#define NRF_FC_PIN      GPIO_PIN_4
#define NRF_TX_PIN_PORT GPIOC
#define NRF_TX_PIN      GPIO_PIN_7

#endif /* __CONFIG_H__ */
