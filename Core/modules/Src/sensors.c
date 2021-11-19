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
 * sensors.c - Abstraction layer for sensors on a platform. It acts as a
 * proxy to use the correct sensor based on device type.
 */

#define DEBUG_MODULE "SENSORS"

#include "sensors.h"
#include "debug.h"
#include "config.h"

#include "sensors_bmi088_bmp388.h"
#include "sensors_bmi270_bmp384.h"

typedef struct {
  void (*init)(SensorsInterfaceType);
  bool (*test)(void);
  bool (*areCalibrated)(void);
  void (*acquire)(sensorData_t *sensors);
  void (*waitDataReady)(void);
  bool (*readGyro)(Axis3f *gyro);
  bool (*readAccel)(Axis3f *acc);
  bool (*readBaro)(baro_t *baro);
  void (*setAccelMode)(AccelModes mode);
  void (*dataAvailableCallback)(void);
	const char *name;
} Sensors;

static const Sensors sensorsFunctions = {
#if (SENSORS_TYPE == SENSORS_BMI088_BMP388)
  .init = sensorsBmi088Bmp388Init,
  .test = sensorsBmi088Bmp388Test,
  .areCalibrated = sensorsBmi088Bmp388AreCalibrated,
  .acquire = sensorsBmi088Bmp388Acquire,
  .waitDataReady = sensorsBmi088Bmp388WaitDataReady,
  .readGyro = sensorsBmi088Bmp388ReadGyro,
  .readAccel = sensorsBmi088Bmp388ReadAccel,
  .readBaro = sensorsBmi088Bmp388ReadBaro,
  .setAccelMode = sensorsBmi088Bmp388SetAccelMode,
  .dataAvailableCallback = sensorsBmi088Bmp388DataAvailableCallback,
  .name = "BMI088BMP388",
#elif (SENSORS_TYPE == SENSORS_BMI270_BMP384)
  .init = sensorsBmi270Bmp384Init,
  .test = sensorsBmi270Bmp384Test,
  .areCalibrated = sensorsBmi270Bmp384AreCalibrated,
  .acquire = sensorsBmi270Bmp384Acquire,
  .waitDataReady = sensorsBmi270Bmp384WaitDataReady,
  .readGyro = sensorsBmi270Bmp384ReadGyro,
  .readAccel = sensorsBmi270Bmp384ReadAccel,
  .readBaro = sensorsBmi270Bmp384ReadBaro,
  .setAccelMode = sensorsBmi270Bmp384SetAccelMode,
  .dataAvailableCallback = sensorsBmi270Bmp384DataAvailableCallback,
  .name = "BMI270BMP384",
#endif
};

void sensorsInit() {
  DEBUG_PRINT("Using %s (%d) sensors.\n", sensorsFunctions.name, SENSORS_TYPE);
  sensorsFunctions.init(SENSORS_INTERFACE);
}

bool sensorsTest() {
  return sensorsFunctions.test();
}

bool sensorsAreCalibrated() {
  return sensorsFunctions.areCalibrated();
}

void sensorsAcquire(sensorData_t *sensors) {
  sensorsFunctions.acquire(sensors);
}

void sensorsWaitDataReady() {
  sensorsFunctions.waitDataReady();
}

bool sensorsReadGyro(Axis3f *gyro) {
  return sensorsFunctions.readGyro(gyro);
}

bool sensorsReadAccel(Axis3f *acc) {
  return sensorsFunctions.readAccel(acc);
}

bool sensorsReadBaro(baro_t *baro) {
  return sensorsFunctions.readBaro(baro);
}

void sensorsSetAccelMode(AccelModes mode) {
  sensorsFunctions.setAccelMode(mode);
}

void sensorsAvailableCallback() {
  sensorsFunctions.dataAvailableCallback();
}

