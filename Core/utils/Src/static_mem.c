/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
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
 * @file static_mem.c
 * @brief Support for OS objects that use static memory, such as
 * queues, tasks and semaphores
 *
 * @copyright Copyright (c) 2019 Bitcraze AB
 */

#include <FreeRTOS.h>
#include <string.h>
#include "static_mem.h"

/**
 * @brief configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task.
 */
// TODO: figure out if we need this
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t xIdleTaskTCB;
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*———————————————————–*/

/**
 * @brief configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task.
 */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
  NO_DMA_CCM_SAFE_ZERO_INIT static StaticTask_t xTimerTaskTCB;
  NO_DMA_CCM_SAFE_ZERO_INIT static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

  *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
  *ppxTimerTaskStackBuffer = uxTimerTaskStack;
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

osThreadAttr_t* getOsThreadAttr_t(char* name, void* cb_mem, uint32_t cb_size, void* stack_mem, uint32_t stack_size, osPriority_t priority) {
  static osThreadAttr_t attr;
  memset(&attr, 0, sizeof(attr));
  attr.name = name;
  attr.priority = priority;
  attr.cb_mem = cb_mem;
  attr.cb_size = cb_size;
  attr.stack_mem = stack_mem;
  attr.stack_size = stack_size;
  return &attr;
}

osMessageQueueAttr_t* getOsMessageQueueAttr_t(char* name, void* cb_mem, uint32_t cb_size, void* mq_mem, uint32_t mq_size) {
  static osMessageQueueAttr_t attr;
  memset(&attr, 0, sizeof(attr));
  attr.name = name;
  attr.cb_mem = cb_mem;
  attr.cb_size = cb_size;
  attr.mq_mem = mq_mem;
  attr.mq_size = mq_size;
  return &attr;
}

osMutexAttr_t* getOsMutexAttr_t(char* name, void* cb_mem, uint32_t cb_size) {
  static osMutexAttr_t attr;
  memset(&attr, 0, sizeof(attr));
  attr.name = name;
  attr.cb_mem = cb_mem;
  attr.cb_size = cb_size;
  return &attr;
}
