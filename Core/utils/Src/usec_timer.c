/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 *
 * usec_time.c - microsecond-resolution timer and timestamps.
 */
#include <stdbool.h>
#include "usec_timer.h"
#include "cfassert.h"

// TODO: remove this
// #include "nvicconf.h"
#include "stm32fxxx.h"

#include "_tim.h"
#include "config.h"

static bool isInit = false;
static uint32_t usecTimerHighCount;

void usecTimerInit() {
  if (isInit)
    return;

  usecTimerHighCount = 0;

  isInit = true;
}

uint64_t usecTimerStamp() {
  IF_DEBUG_ASSERT(isInit);
  uint32_t high0;
  __atomic_load(&usecTimerHighCount, &high0, __ATOMIC_SEQ_CST);
  
  uint32_t low = __HAL_TIM_GET_COUNTER(&USEC_TIM);
  uint32_t high;
  __atomic_load(&usecTimerHighCount, &high, __ATOMIC_SEQ_CST);

  // There was no increment in between
  if (high == high0) {
    return (((uint64_t)high) << 16) + low;
  }
  // There was an increment, but we don't expect another one soon
  return (((uint64_t)high) << 16) + __HAL_TIM_GET_COUNTER(&USEC_TIM);
}

void usecInc() {
  __sync_fetch_and_add(&usecTimerHighCount, 1);
}

void sleepus(uint32_t us) {
  uint64_t start = usecTimerStamp();
  while ((start + us) > usecTimerStamp()) {
  }
}