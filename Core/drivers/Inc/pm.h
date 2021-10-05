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
 * pm.h - Power Management driver and functions.
 */

#ifndef __PM_H__
#define __PM_H__

#include <stdbool.h>

#include "config.h"

// #include "adc.h"
#include "syslink.h"
// #include "deck.h"

#define PM_BAT_LOW_VOLTAGE   3.2f
#define PM_BAT_LOW_TIMEOUT   5000
#define PM_BAT_CRITICAL_LOW_VOLTAGE   3.0f
#define PM_BAT_CRITICAL_LOW_TIMEOUT   5000
#define PM_SYSTEM_SHUTDOWN_TIMEOUT    (1000 * 60 * 5)


#define PM_BAT_DIVIDER                3.0f
#define PM_BAT_ADC_FOR_3_VOLT         (int32_t)(((3.0f / PM_BAT_DIVIDER) / 2.8f) * 4096)
#define PM_BAT_ADC_FOR_1p2_VOLT       (int32_t)(((1.2f / PM_BAT_DIVIDER) / 2.8f) * 4096)

#define PM_BAT_IIR_SHIFT     8
/**
 * Set PM_BAT_WANTED_LPF_CUTOFF_HZ to the wanted cut-off freq in Hz.
 */
#define PM_BAT_WANTED_LPF_CUTOFF_HZ   1

/**
 * Attenuation should be between 1 to 256.
 *
 * f0 = fs / 2*pi*attenuation.
 * attenuation = fs / 2*pi*f0
 */
#define PM_BAT_IIR_LPF_ATTENUATION (int)(ADC_SAMPLING_FREQ / (int)(2 * 3.1415f * PM_BAT_WANTED_LPF_CUTOFF_HZ))
#define PM_BAT_IIR_LPF_ATT_FACTOR  (int)((1 << PM_BAT_IIR_SHIFT) / PM_BAT_IIR_LPF_ATTENUATION)

typedef enum {
  battery,
  charging,
  charged,
  lowPower,
  shutDown,
} PmStates;

typedef void (*graceful_shutdown_callback_t)();

void pmInit(void);
bool pmTest(void);

/**
 * Power management task
 */
void pmTask(void *param);

void pmSyslinkUpdate(SyslinkPacket *slp);

/**
 * Returns the battery voltage i volts as a float
 */
float pmGetBatteryVoltage(void);

/**
 * Returns the min battery voltage i volts as a float
 */
float pmGetBatteryVoltageMin(void);

/**
 * Returns the max battery voltage i volts as a float
 */
float pmGetBatteryVoltageMax(void);

/**
 * Updates and calculates battery values.
 * Should be called for every new adcValues sample.
 */
// void pmBatteryUpdate(AdcGroup* adcValues);

/**
 * Returns true if the battery is below its low capacity threshold for an
 * extended period of time.
 */
bool pmIsBatteryLow(void);

/**
 * Returns true if the charger is currently connected
 */
bool pmIsChargerConnected(void);

/**
 * Returns true if the battery is currently charging
 */
bool pmIsCharging(void);

/**
 * Returns true if the battery is currently in use
 */
bool pmIsDischarging(void);

/**
 * Register a callback to be run when the NRF51 signals shutdown
 */
bool pmRegisterGracefulShutdownCallback(graceful_shutdown_callback_t cb);

#endif /* __PM_H__ */
