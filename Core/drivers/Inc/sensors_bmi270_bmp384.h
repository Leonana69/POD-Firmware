/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 */

#ifndef __SENSORS_BMI270_BMP384_H__
#define __SENSORS_BMI270_BMP384_H__

#include "sensors.h"

void sensorsBmi270Bmp384Init(SensorsInterfaceType interface);
bool sensorsBmi270Bmp384Test(void);
bool sensorsBmi270Bmp384AreCalibrated(void);
void sensorsBmi270Bmp384Acquire(sensorData_t *sensors);
void sensorsBmi270Bmp384WaitDataReady(void);
bool sensorsBmi270Bmp384ReadGyro(Axis3f *gyro);
bool sensorsBmi270Bmp384ReadAccel(Axis3f *accel);
bool sensorsBmi270Bmp384ReadBaro(baro_t *baro);
void sensorsBmi270Bmp384SetAccelMode(AccelModes mode);
void sensorsBmi270Bmp384DataAvailableCallback(void);

#endif // __SENSORS_BMI270_BMP384_H__
