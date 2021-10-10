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

// https://gcc.gnu.org/onlinedocs/cpp/Stringizing.html
#define xstr(s) str(s)
#define str(s) #s

#define SENSOR_INCLUDED_BMI088_BMP388

#if defined(SENSOR_INCLUDED_BMI088_BMP388) || defined(SENSOR_INCLUDED_BMI088_SPI_BMP388)
  #include "sensors_bmi088_bmp388.h"
#endif

static SensorsType currentSensors = SENSORS_TYPE;

typedef struct {
  void (*init)(SensorsInterfaceType);
  bool (*test)(void);
  bool (*areCalibrated)(void);
  bool (*manufacturingTest)(void);
  void (*acquire)(sensorData_t *sensors, const uint32_t tick);
  void (*waitDataReady)(void);
  bool (*readGyro)(Axis3f *gyro);
  bool (*readAcc)(Axis3f *acc);
  bool (*readBaro)(baro_t *baro);
  void (*setAccMode)(accModes accMode);
  void (*dataAvailableCallback)(void);
	const char *name;
} Sensors;

// ignore unused func
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static void nullFunction(void) {}
#pragma GCC diagnostic pop

static const Sensors sensorsFunctions[SENSORS_COUNT] = {
#ifdef SENSOR_INCLUDED_BMI088_BMP388
  {
    .init = sensorsBmi088Bmp388Init,
    .test = sensorsBmi088Bmp388Test,
    .areCalibrated = sensorsBmi088Bmp388AreCalibrated,
    .manufacturingTest = sensorsBmi088Bmp388ManufacturingTest,
    .acquire = sensorsBmi088Bmp388Acquire,
    .waitDataReady = sensorsBmi088Bmp388WaitDataReady,
    .readGyro = sensorsBmi088Bmp388ReadGyro,
    .readAcc = sensorsBmi088Bmp388ReadAcc,
    .readBaro = sensorsBmi088Bmp388ReadBaro,
    .setAccMode = sensorsBmi088Bmp388SetAccMode,
    .dataAvailableCallback = sensorsBmi088Bmp388DataAvailableCallback,
		.name = "BMI088BMP388",
  },
#endif
};

void sensorsInit() {
  sensorsFunctions[currentSensors].init(SENSORS_INTERFACE);
	DEBUG_PRINT("Using %s (%d) sensors\n", sensorsGetName(), currentSensors);
}

bool sensorsTest(void) {
  return sensorsFunctions[currentSensors].test();
}

bool sensorsAreCalibrated(void) {
  return sensorsFunctions[currentSensors].areCalibrated();
}

bool sensorsManufacturingTest(void){
  return sensorsFunctions[currentSensors].manufacturingTest;
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick) {
  sensorsFunctions[currentSensors].acquire(sensors, tick);
}

void sensorsWaitDataReady(void) {
  sensorsFunctions[currentSensors].waitDataReady();
}

bool sensorsReadGyro(Axis3f *gyro) {
  return sensorsFunctions[currentSensors].readGyro(gyro);
}

bool sensorsReadAcc(Axis3f *acc) {
  return sensorsFunctions[currentSensors].readAcc(acc);
}

bool sensorsReadBaro(baro_t *baro) {
  return sensorsFunctions[currentSensors].readBaro(baro);
}

void sensorsSetAccMode(accModes accMode) {
  sensorsFunctions[currentSensors].setAccMode(accMode);
}

SensorsType sensorsGetType() {
  return currentSensors;
}

const char* sensorsGetName() {
  return sensorsFunctions[currentSensors].name;
}

void __attribute__((used)) EXTI14_Callback(void) {
  sensorsFunctions[currentSensors].dataAvailableCallback();
}
