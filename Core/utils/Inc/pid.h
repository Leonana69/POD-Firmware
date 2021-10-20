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
 *
 * pid.h - implementation of the PID regulator
 */
#ifndef __PID_H__
#define __PID_H__

#include <stdbool.h>
#include "filter.h"

#define	PID_ROLL 	(1)
#define	PID_PITCH (1 << 1)
#define	PID_YAW 	(1 << 2)

#define	PID_X (1)
#define	PID_Y	(1 << 1)
#define	PID_Z (1 << 2)

typedef struct {
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float rate;					//< reciprocal of dt

	float dt;           //< delta-time dt
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative

	float iLimit;       //< integral limit, absolute value. '0' means no limit.
	float oLimit;  			//< total PID output limit, absolute value. '0' means no limit.

	lpf2pData dFilter;  //< filter for D term
	bool enableDFilter; //< filter for D term enable flag
} PidObject;

typedef struct {
	float kp;
	float ki;
	float kd;
	float rate;

	float iLimit;
	float oLimit;

	bool enableDFilter;
	float cutoffFreq;
} PidParam;

typedef struct {
	PidObject val;
	PidObject rate;
} CascadePidObject;

typedef struct {
	PidParam val;
	PidParam rate;
} CascadePidParam;

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] rate        Rate
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
 void pidInit(PidObject *pid, PidParam *param);

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidSetIntegralLimit(PidObject* pid, const float limit);

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidReset(PidObject* pid);

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 */
float pidUpdate(PidObject* pid, const float measured, const float target, const bool updateError);

// TODO: add comments
void pidSetError(PidObject* pid, const float error);

/**
 * Reset the Dfilter and set cutoff frequency
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
*/
void filterReset(PidObject* pid, const float samplingRate, const float cutoffFreq, bool enableDFilter);

#endif /* __PID_H__ */
