/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2018 Bitcraze AB
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
 * sensors_bmi270_bmp384.c: IMU sensor driver for the bmi270 and bmp384 bosch sensors
 */

#define DEBUG_MODULE "IMU"

#include <math.h>
#include <string.h>
#include "sensors_bmi270_bmp384.h"

#include "_i2c.h"
#include "cal.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "debug.h"
#include "ledseq.h"
#include "filter.h"
#include "static_mem.h"
#include "estimator.h"
#include "usec_timer.h"

#include "bmi270.h"
#include "bmp3.h"

static bool isInit = false;
static SensorsInterfaceType currentInterface;

#define SENSORS_READ_RATE_HZ  1000
#define SENSORS_READ_BARO_HZ  50
#define SENSORS_BARO_ENABLE   false
#define SENSORS_DELAY_BARO    (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ)

static float accelValue2Gravity;
static float gyroVale2Degree;

#define GYRO_DIM 3
#define GYRO_MIN_BIAS_TIMEOUT_MS 1000

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_GYRO_BIAS_SAMPLES_NBR 512
#define SENSORS_ACCEL_SCALE_SAMPLES_NBR 200

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_LIMIT 1000

typedef struct {
  Axis3f bias;
  Axis3f variance;
  Axis3f mean;
  bool isBufferFilled;
  Axis3i16* bufHead;
  Axis3i16 buffer[SENSORS_GYRO_BIAS_SAMPLES_NBR];
} BiasObj;

/*! @brief mutexs */
STATIC_MEM_SEMAPHORE_ALLOC(readDataReady);
STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

STATIC_MEM_MUTEX_ALLOC(accelDataMutex);
STATIC_MEM_MUTEX_ALLOC(gyroDataMutex);
STATIC_MEM_MUTEX_ALLOC(baroDataMutex);
static Axis3f accelData;
static Axis3f gyroData;
static baro_t baroData;

/*! @brief This structure containing relevant bmi270 info */
struct bmi2_dev bmi2Dev;
static uint8_t intfAddr = BMI2_I2C_PRIM_ADDR;
enum { ACCEL, GYRO };
struct bmi2_sens_config bmi270Config[2];
/*! @brief This structure containing relevant bmp3 info */
static struct bmp3_dev bmp3Dev;
static struct bmp3_settings bmp3Settings;
static uint8_t baroIntfAddr = BMP3_ADDR_I2C_SEC;

static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

/*! @brief Accel and gyro calibration data */
static bool accelScaleFound = false;
static bool gyroBiasFound = false;
static float accelScale;
NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasObj;
static Axis3f gyroBias;

/*! @brief Low Pass filter */
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accelLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static void processGyroBias(int16_t gx, int16_t gy, int16_t gz);
static void processAccelScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsAddBiasValue(int16_t x, int16_t y, int16_t z);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

static void sensorsTaskInit(void);
static void sensorsDeviceInit(void);
static bool sensorsBmi270Bmp384ManufacturingTest(void);

/*! @brief Sensor delay_us function */
static void sensorsUsDelay(uint32_t period, void *intf_ptr) {
  uint32_t period_ms = (period + 500) / 1000;
  if (period_ms == 0) {
    osDelay(1);
  } else
    osDelay(period_ms);
}

// TODO: weird temp and pressure data
static bool sensorBaroGet(struct bmp3_data *dataOut) {
  return bmp3_get_sensor_data(BMP3_PRESS_TEMP, dataOut, &bmp3Dev) == BMP3_OK;
}

// TODO: pressure to asl conversion
static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature) {
  baroScaled->pressure = pressure * 0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsBmi270Bmp384ReadGyro(Axis3f *gyro) {
	osStatus_t status;
	status = osMutexAcquire(gyroDataMutex, 0);
	memcpy(gyro, &gyroData, sizeof(gyroData));
	osMutexRelease(gyroDataMutex);
	return status == osOK;
}

bool sensorsBmi270Bmp384ReadAccel(Axis3f *accel) {
  osStatus_t status;
	status = osMutexAcquire(accelDataMutex, 0);
	memcpy(accel, &accelData, sizeof(accelData));
	osMutexRelease(accelDataMutex);
	return status == osOK;
}

bool sensorsBmi270Bmp384ReadBaro(baro_t *baro) {
  osStatus_t status;
	status = osMutexAcquire(baroDataMutex, 0);
	memcpy(baro, &baroData, sizeof(baroData));
	osMutexRelease(baroDataMutex);
	return status == osOK;
}

void sensorsBmi270Bmp384Acquire(sensorData_t *sensors) {
  sensorsBmi270Bmp384ReadGyro(&sensors->gyro);
  sensorsBmi270Bmp384ReadAccel(&sensors->accel);
  sensorsBmi270Bmp384ReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi270Bmp384AreCalibrated() {
  return gyroBiasFound && accelScaleFound;
}

static void sensorsTask(void *param) {
	struct bmi2_sensor_data bmi270Data[2];
	bmi270Data[ACCEL].type = BMI2_ACCEL;
	bmi270Data[GYRO].type = BMI2_GYRO;

  struct bmp3_data bmp3Baro;
  systemWaitStart();
  Axis3f accelScaled;
  measurement_t measurement;
  /** wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  uint32_t lastWakeTime = osKernelGetTickCount();
  while (1) {
    lastWakeTime += 1;
		osDelayUntil(lastWakeTime);

    sensorData.interruptTimestamp = imuIntTimestamp;

    /*! get data from chosen sensors */
		bmi270_get_sensor_data(bmi270Data, 2, &bmi2Dev);

    /*! calibrate if necessary */
    if (!gyroBiasFound)
      processGyroBias(bmi270Data[GYRO].sens_data.gyr.x, bmi270Data[GYRO].sens_data.gyr.y, bmi270Data[GYRO].sens_data.gyr.z);
    else if (!accelScaleFound)
      processAccelScale(bmi270Data[ACCEL].sens_data.acc.x, bmi270Data[ACCEL].sens_data.acc.y, bmi270Data[ACCEL].sens_data.acc.z);

    /*! Gyro compensation */
    sensorData.gyro.x = (bmi270Data[GYRO].sens_data.gyr.x - gyroBias.x) * gyroVale2Degree;
    sensorData.gyro.y = (bmi270Data[GYRO].sens_data.gyr.y - gyroBias.y) * gyroVale2Degree;
    sensorData.gyro.z = (bmi270Data[GYRO].sens_data.gyr.z - gyroBias.z) * gyroVale2Degree;
    applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

    measurement.type = MeasurementTypeGyroscope;
    measurement.data.gyroscope.gyro = sensorData.gyro;
    estimatorEnqueue(&measurement);

    /*! Accel compensation */
    accelScaled.x = bmi270Data[ACCEL].sens_data.acc.x * accelValue2Gravity / accelScale;
    accelScaled.y = bmi270Data[ACCEL].sens_data.acc.y * accelValue2Gravity / accelScale;
    accelScaled.z = bmi270Data[ACCEL].sens_data.acc.z * accelValue2Gravity / accelScale;

    
    sensorsAccAlignToGravity(&accelScaled, &sensorData.accel);
    applyAxis3fLpf((lpf2pData*)(&accelLpf), &sensorData.accel);

    measurement.type = MeasurementTypeAcceleration;
    measurement.data.acceleration.accel = sensorData.accel;
    estimatorEnqueue(&measurement);

    // TODO: not sure if this works well
    osMutexAcquire(accelDataMutex, osWaitForever);
    memcpy(&accelData, &sensorData.accel, sizeof(accelData));
    osMutexRelease(accelDataMutex);
    osMutexAcquire(gyroDataMutex, osWaitForever);
    memcpy(&gyroData, &sensorData.gyro, sizeof(gyroData));
    osMutexRelease(gyroDataMutex);

		static uint8_t baroMeasDelay = 0;
		if (SENSORS_BARO_ENABLE && ++baroMeasDelay == SENSORS_DELAY_BARO) {
      sensorBaroGet(&bmp3Baro);
			sensorsScaleBaro(&sensorData.baro, bmp3Baro.pressure, bmp3Baro.temperature);

			measurement.type = MeasurementTypeBarometer;
			measurement.data.barometer.baro = sensorData.baro;
			estimatorEnqueue(&measurement);

      osMutexAcquire(baroDataMutex, osWaitForever);
      memcpy(&baroData, &sensorData.baro, sizeof(baroData));
      osMutexRelease(baroDataMutex);
			baroMeasDelay = 0;
		}
    osSemaphoreRelease(readDataReady);
  }
}

void sensorsBmi270Bmp384WaitDataReady() {
  osSemaphoreAcquire(readDataReady, osWaitForever);
}

static void sensorsDeviceInit(void) {
  int8_t rslt;
	/*! wait for sensor to start */
	osDelay(100);

	/*! BMI270 */

	bmi2Dev.intf_ptr = &intfAddr;
	bmi2Dev.delay_us = &sensorsUsDelay;
	bmi2Dev.read_write_len = 32;
    bmi2Dev.config_file_ptr = NULL;
	if (currentInterface == SENSOR_INTF_I2C) {
		bmi2Dev.intf = BMI2_I2C_INTF;
        bmi2Dev.read = &i2cSensorsRead;
		bmi2Dev.write = &i2cSensorsWrite;
	} else {
		bmi2Dev.intf = BMI2_SPI_INTF;
		// TODO: spi read write
	}

	rslt = bmi270_init(&bmi2Dev);
	if (rslt != BMI2_OK)
		DEBUG_PRINT("BMI270 Init [FAILED].\n");
	else {
		DEBUG_PRINT("BMI270 Init [OK].\n");
    bmi270Config[ACCEL].type = BMI2_ACCEL;
    bmi270Config[GYRO].type = BMI2_GYRO;
    rslt = bmi2_get_sensor_config(bmi270Config, 2, &bmi2Dev);
    if (rslt != BMI2_OK)
			DEBUG_PRINT("BMI270 Accel Gyro Config [FAILED].\n");

    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, &bmi2Dev);
    if (rslt != BMI2_OK)
			DEBUG_PRINT("BMI270 Int2 Config [FAILED].\n");

    bmi270Config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    bmi270Config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_16G;
		bmi270Config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    bmi270Config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

		bmi270Config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;
		bmi270Config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;
		bmi270Config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
		bmi270Config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
		bmi270Config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    accelValue2Gravity = (float)16 / 32768.0f;
		gyroVale2Degree = (float)2000 / 32768.0f;

		rslt = bmi2_set_sensor_config(bmi270Config, 2, &bmi2Dev);
		if (rslt != BMI2_OK)
			DEBUG_PRINT("BMI270 Accel Gyro Meas Config [FAILED].\n");

		bmi2Dev.delay_us(10000, NULL);
	}

  /*! BMP384 */
  // TODO: remove this
  if (!SENSORS_BARO_ENABLE) goto BMPEND;
  bmp3Dev.chip_id = BMP3_CHIP_ID;
	bmp3Dev.intf_ptr = &baroIntfAddr;
  bmp3Dev.intf = BMP3_I2C_INTF;
  bmp3Dev.read = &i2cSensorsRead;
  bmp3Dev.write = &i2cSensorsWrite;
  bmp3Dev.delay_us = &sensorsUsDelay;
	rslt = bmp3_init(&bmp3Dev);
  if (rslt != BMP3_OK)
  	DEBUG_PRINT("BMP384 Init [FAILED].\n");
 	else {
		DEBUG_PRINT("BMP384 Init [OK].\n");
		bmp3Settings.int_settings.drdy_en = BMP3_ENABLE;
		bmp3Settings.press_en = BMP3_ENABLE;
		bmp3Settings.temp_en = BMP3_ENABLE;

		bmp3Settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
		bmp3Settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
		bmp3Settings.odr_filter.odr = BMP3_ODR_50_HZ;
		bmp3Settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
		uint16_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                 BMP3_SEL_DRDY_EN | BMP3_SEL_IIR_FILTER;

		rslt = bmp3_set_sensor_settings(settings_sel, &bmp3Settings, &bmp3Dev);
		if (rslt != BMP3_OK)
  		DEBUG_PRINT("BMP384 Setting [FAILED].\n");
		bmp3Settings.op_mode = BMP3_MODE_NORMAL;
		rslt = bmp3_set_op_mode(&bmp3Settings, &bmp3Dev);
		if (rslt != BMP3_OK)
  		DEBUG_PRINT("BMP384 OP Mode [FAILED].\n");
		bmp3Dev.delay_us(10000, NULL);
	}
BMPEND:
  /*! Init second order filters for accel and gyro */
  for (int i = 0; i < 3; i++) {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accelLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = cosf(radians(configblockGetCalibPitch()));
  sinPitch = sinf(radians(configblockGetCalibPitch()));
  cosRoll = cosf(radians(configblockGetCalibRoll()));
  sinRoll = sinf(radians(configblockGetCalibRoll()));
}

static void sensorsTaskInit() {
	STATIC_MUTEX_CREATE(accelDataMutex);
	STATIC_MUTEX_CREATE(gyroDataMutex);
	STATIC_MUTEX_CREATE(baroDataMutex);

  STATIC_SEMAPHORE_CREATE(readDataReady, 1, 0);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

void sensorsBmi270Bmp384Init(SensorsInterfaceType interface) {
	if (isInit)
		return;

	currentInterface = interface;
  accelScale = 1.0f;
  gyroBiasObj.isBufferFilled = false;
  gyroBiasObj.bufHead = gyroBiasObj.buffer;
  gyroBias.x = 0;
  gyroBias.y = 0;
  gyroBias.z = 0;
  sensorsDeviceInit();
  sensorsTaskInit();
	isInit = true;
}

bool sensorsBmi270Bmp384Test(void) {
  return isInit && sensorsBmi270Bmp384ManufacturingTest();
}

/**
 * Calculates accelerometer scale out of SENSORS_ACCEL_SCALE_SAMPLES_NBR samples. Should be called when
 * platform is stable.
 */
static void processAccelScale(int16_t ax, int16_t ay, int16_t az) {
  static float sum = 0;
  static int cnt = 0;
  if (!accelScaleFound) {
    sum += sqrtf((float)(ax * ax + ay * ay + az * az)) * accelValue2Gravity;
    cnt++;

    if (cnt == SENSORS_ACCEL_SCALE_SAMPLES_NBR) {
      accelScale = sum / SENSORS_ACCEL_SCALE_SAMPLES_NBR;
      accelScaleFound = true;
    }
  }
}

/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static void processGyroBias(int16_t gx, int16_t gy, int16_t gz) {
  sensorsAddBiasValue(gx, gy, gz);

  if (!gyroBiasFound && gyroBiasObj.isBufferFilled) {
    sensorsCalculateVarianceAndMean(&gyroBiasObj, &gyroBiasObj.variance, &gyroBiasObj.mean);
    if (gyroBiasObj.variance.x < GYRO_VARIANCE_LIMIT &&
        gyroBiasObj.variance.y < GYRO_VARIANCE_LIMIT &&
        gyroBiasObj.variance.z < GYRO_VARIANCE_LIMIT) {
      gyroBiasObj.bias.x = gyroBiasObj.mean.x;
      gyroBiasObj.bias.y = gyroBiasObj.mean.y;
      gyroBiasObj.bias.z = gyroBiasObj.mean.z;
      gyroBiasFound = true;
    }
    if (gyroBiasFound) {
      gyroBias.x = gyroBiasObj.bias.x;
      gyroBias.y = gyroBiasObj.bias.y;
      gyroBias.z = gyroBiasObj.bias.z;
      ledseqRun(&seq_calibrated);
    }   
  }
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut) {
  int64_t sum[GYRO_DIM] = { 0 };
  int64_t sumSq[GYRO_DIM] = { 0 };
  for (int i = 0; i < SENSORS_GYRO_BIAS_SAMPLES_NBR; i++) {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  meanOut->x = (float)sum[0] / SENSORS_GYRO_BIAS_SAMPLES_NBR;
  meanOut->y = (float)sum[1] / SENSORS_GYRO_BIAS_SAMPLES_NBR;
  meanOut->z = (float)sum[2] / SENSORS_GYRO_BIAS_SAMPLES_NBR;

	varOut->x = sumSq[0] / SENSORS_GYRO_BIAS_SAMPLES_NBR + meanOut->x * meanOut->x;
	varOut->y = sumSq[1] / SENSORS_GYRO_BIAS_SAMPLES_NBR + meanOut->y * meanOut->y;
	varOut->z = sumSq[2] / SENSORS_GYRO_BIAS_SAMPLES_NBR + meanOut->z * meanOut->z;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(int16_t x, int16_t y, int16_t z) {
  gyroBiasObj.bufHead->x = x;
  gyroBiasObj.bufHead->y = y;
  gyroBiasObj.bufHead->z = z;
  gyroBiasObj.bufHead++;

  if (gyroBiasObj.bufHead >= &gyroBiasObj.buffer[SENSORS_GYRO_BIAS_SAMPLES_NBR]) {
    gyroBiasObj.bufHead = gyroBiasObj.buffer;
    gyroBiasObj.isBufferFilled = true;
  }
}

bool sensorsBmi270Bmp384ManufacturingTest(void) {
  // TODO: check selftest, it will cause the zero-accel data
  bool testStatus = true;
  // int8_t rslt = 0;
  // rslt = bmi08a_perform_selftest(&bmi08xDev);
  // if (rslt != BMI08X_W_SELF_TEST_FAIL) {
  //   DEBUG_PRINT("BMI088 Accel self-test [OK].\n");
  // } else {
  //   DEBUG_PRINT("BMI088 Accel self-test [FAILED].\n");
  //   testStatus = false;
  // }
  /*! Gyro test is conflict with interrupt, following code will cause system to reboot. */
  // rslt = bmi08g_perform_selftest(&bmi08xDev);
  // if (rslt != BMI08X_W_SELF_TEST_FAIL) {
  //   DEBUG_PRINT_UART("BMI088 Gyro self-test [OK].\n");
  // } else {
  //   DEBUG_PRINT_UART("BMI088 Gyro self-test [FAILED].\n");
  //   testStatus = false;
  // }
  return testStatus;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out) {
  Axis3f rx;
  Axis3f ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

// TODO: fix mode change
void sensorsBmi270Bmp384SetAccelMode(AccelModes mode) {
  switch (mode) {
    case ACCEL_MODE_PROPTEST:
      /* set bandwidth and range of accel (280Hz cut-off according to datasheet) */
      bmi270Config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    	bmi270Config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_16G;
			bmi270Config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
      if (bmi2_set_sensor_config(bmi270Config, 1, &bmi2Dev) != BMI2_OK)
        // TODO: REMOVE UART
        DEBUG_PRINT("BMI270 Accel Config [FAIL]\n");
      for (uint8_t i = 0; i < 3; i++)
        lpf2pInit(&accelLpf[i], 1000, 500);
      break;
    case ACCEL_MODE_FLIGHT:
    default:
      /* set bandwidth and range of accel (145Hz cut-off according to datasheet) */
      bmi270Config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;
    	bmi270Config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_16G;
			bmi270Config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
      if (bmi2_set_sensor_config(bmi270Config, 1, &bmi2Dev) != BMI2_OK)
        DEBUG_PRINT("BMI270 Accel Config [FAIL]\n");
      for (uint8_t i = 0; i < 3; i++)
        lpf2pInit(&accelLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
      break;
  }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in) {
  for (uint8_t i = 0; i < 3; i++)
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
}

void sensorsBmi270Bmp384DataAvailableCallback(void) {
  imuIntTimestamp = usecTimerStamp();
}
