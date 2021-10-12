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
 * sensors_bmi088_bmp388.c: IMU sensor driver for the *88 bosch sensors
 */

#define DEBUG_MODULE "IMU"

#include <math.h>
#include <string.h>
#include "sensors_bmi088_bmp388.h"

// #include "FreeRTOS.h"
// #include "semphr.h"
// #include "task.h"
#include "_i2c.h"
#include "cal.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "debug.h"
// #include "nvicconf.h"
#include "ledseq.h"
#include "filter.h"
#include "static_mem.h"
#include "estimator.h"
#include "usec_timer.h"

#include "bmi08x.h"
#include "bmp3.h"

static bool isInit = false;
SensorsInterfaceType currentInterface;

#define SENSORS_READ_RATE_HZ            1000
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ)

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
STATIC_MEM_SEMAPHORE_ALLOC(devDataReady);
STATIC_MEM_SEMAPHORE_ALLOC(readDataReady);
STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

STATIC_MEM_MUTEX_ALLOC(accelDataMutex);
STATIC_MEM_MUTEX_ALLOC(gyroDataMutex);
STATIC_MEM_MUTEX_ALLOC(baroDataMutex);
static Axis3i16 accelData;
static Axis3i16 gyroData;
static baro_t baroData;

/*! @brief This structure containing relevant bmi08x info */
static struct bmi08x_dev bmi08xDev;
static struct bmi08x_gyro_int_channel_cfg gyroIntConfig;
static uint8_t accelIntfAddr = BMI08X_ACCEL_I2C_ADDR_PRIMARY;
static uint8_t gyroIntfAddr = BMI08X_GYRO_I2C_ADDR_SECONDARY;
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
static bool sensorsBmi088Bmp388ManufacturingTest(void);

/*! @brief Sensor delay_us function */
static void sensorsUsDelay(uint32_t period, void *intf_ptr) {
  uint32_t period_ms = period / 1000;
  if (period < 1000) {
    osDelay(1);
  } else
    osDelay(period_ms);
}

static bool sensorsGyroGet(struct bmi08x_sensor_data *dataOut) {
  return bmi08g_get_data(dataOut, &bmi08xDev) == BMI08X_OK;
}

static bool sensorsAccelGet(struct bmi08x_sensor_data *dataOut) {
	return bmi08a_get_data(dataOut, &bmi08xDev) == BMI08X_OK;
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

bool sensorsBmi088Bmp388ReadGyro(Axis3f *gyro) {
	osStatus_t status;
	status = osMutexAcquire(gyroDataMutex, 0);
	memcpy(gyro, &gyroData, sizeof(gyroData));
	osMutexRelease(gyroDataMutex);
	return status == osOK;
}

bool sensorsBmi088Bmp388ReadAccel(Axis3f *accel) {
  osStatus_t status;
	status = osMutexAcquire(accelDataMutex, 0);
	memcpy(accel, &accelData, sizeof(accelData));
	osMutexRelease(accelDataMutex);
	return status == osOK;
}

bool sensorsBmi088Bmp388ReadBaro(baro_t *baro) {
  osStatus_t status;
	status = osMutexAcquire(baroDataMutex, 0);
	memcpy(baro, &baroData, sizeof(baroData));
	osMutexRelease(baroDataMutex);
	return status == osOK;
}

void sensorsBmi088Bmp388Acquire(sensorData_t *sensors) {
  sensorsBmi088Bmp388ReadGyro(&sensors->gyro);
  sensorsBmi088Bmp388ReadAccel(&sensors->accel);
  sensorsBmi088Bmp388ReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi088Bmp388AreCalibrated() {
  return gyroBiasFound && accelScaleFound;
}

static void sensorsTask(void *param) {
	struct bmi08x_sensor_data bmi08xAccel;
	struct bmi08x_sensor_data bmi08xGyro;
  struct bmp3_data bmp3Baro;
  systemWaitStart();

  Axis3f accelScaled;
  measurement_t measurement;
  /** wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1) {
    if (osOK == osSemaphoreAcquire(devDataReady, osWaitForever)) {
      sensorData.interruptTimestamp = imuIntTimestamp;

      /*! get data from chosen sensors */
      sensorsGyroGet(&bmi08xGyro);
      sensorsAccelGet(&bmi08xAccel);

      /*! calibrate if necessary */
      if (!gyroBiasFound)
        processGyroBias(bmi08xGyro.x, bmi08xGyro.y, bmi08xGyro.z);
      else if (!accelScaleFound)
        processAccelScale(bmi08xAccel.x, bmi08xAccel.y, bmi08xAccel.z);

      /*! Gyro compensation */
      sensorData.gyro.x = (bmi08xGyro.x - gyroBias.x) * gyroVale2Degree;
      sensorData.gyro.y = (bmi08xGyro.y - gyroBias.y) * gyroVale2Degree;
      sensorData.gyro.z = (bmi08xGyro.z - gyroBias.z) * gyroVale2Degree;
      applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

      measurement.type = MeasurementTypeGyroscope;
      measurement.data.gyroscope.gyro = sensorData.gyro;
      estimatorEnqueue(&measurement);

      /*! Accel compensation */
      accelScaled.x = bmi08xAccel.x * accelValue2Gravity / accelScale;
      accelScaled.y = bmi08xAccel.y * accelValue2Gravity / accelScale;
      accelScaled.z = bmi08xAccel.z * accelValue2Gravity / accelScale;
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
    }

		static uint8_t baroMeasDelay = 0;
		if (++baroMeasDelay == SENSORS_DELAY_BARO) {
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

void sensorsBmi088Bmp388WaitDataReady() {
  osSemaphoreAcquire(readDataReady, osWaitForever);
}

static void sensorsDeviceInit(void) {
  int8_t rslt;
	/*! wait for sensor to start */
	osDelay(500);

	/*! BMI088 */
	bmi08xDev.intf_ptr_accel = &accelIntfAddr;
	bmi08xDev.intf_ptr_gyro = &gyroIntfAddr;
	bmi08xDev.delay_us = &sensorsUsDelay;
	bmi08xDev.read_write_len = 32;
	bmi08xDev.variant = BMI088_VARIANT;
	if (currentInterface == SENSOR_INTF_I2C) {
		bmi08xDev.intf = BMI08X_I2C_INTF;
		bmi08xDev.read = &i2cSensorsRead;
		bmi08xDev.write = &i2cSensorsWrite;
	} else {
		bmi08xDev.intf = BMI08X_SPI_INTF;
		// TODO: spi read write
	}
	rslt = bmi08a_init(&bmi08xDev);
  DEBUG_PRINT_UART("SDI: 2\n");
	rslt |= bmi08g_init(&bmi08xDev);
  DEBUG_PRINT_UART("SDI: 3\n");
	// TODO: remove uart
	if (rslt != BMI08X_OK)
		DEBUG_PRINT_UART("BMI088 Init [FAILED].\n");
	else {
		DEBUG_PRINT_UART("BMI088 Init [OK].\n");
		bmi08xDev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
		bmi08xDev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
		bmi08xDev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
		bmi08xDev.accel_cfg.bw = BMI08X_ACCEL_BW_OSR4;
		accelValue2Gravity = (float)24 / 32768.0f;
		rslt = bmi08a_set_power_mode(&bmi08xDev);
		rslt |= bmi08a_set_meas_conf(&bmi08xDev);
		if (rslt != BMI08X_OK)
			DEBUG_PRINT_UART("BMI088 Accel Meas Config [FAILED].\n");
		
		bmi08xDev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
		bmi08xDev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
		bmi08xDev.gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
		bmi08xDev.gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
		gyroVale2Degree = (float)2000 / 32768.0f;
		rslt = bmi08g_set_power_mode(&bmi08xDev);
		rslt |= bmi08g_set_meas_conf(&bmi08xDev);
		if (rslt != BMI08X_OK)
			DEBUG_PRINT_UART("BMI088 Gyro Meas Config [FAILED].\n");

		gyroIntConfig.int_channel = BMI08X_INT_CHANNEL_3;
		gyroIntConfig.int_type = BMI08X_GYRO_INT_DATA_RDY;
		gyroIntConfig.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		gyroIntConfig.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		gyroIntConfig.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
		rslt = bmi08g_set_int_config(&gyroIntConfig, &bmi08xDev);
		if (rslt != BMI08X_OK)
			DEBUG_PRINT_UART("BMI088 Gyro Int Config [FAILED].\n");

		bmi08xDev.delay_us(10000, NULL);
	}

  /*! BMP388 */
  bmp3Dev.chip_id = BMP3_CHIP_ID;
	bmp3Dev.intf_ptr = &baroIntfAddr;
  bmp3Dev.intf = BMP3_I2C_INTF;
  bmp3Dev.read = &i2cSensorsRead;
  bmp3Dev.write = &i2cSensorsWrite;
  bmp3Dev.delay_us = &sensorsUsDelay;
	rslt = bmp3_init(&bmp3Dev);
  if (rslt != BMP3_OK)
  	DEBUG_PRINT_UART("BMP388 Init [FAILED].\n");
 	else {
		DEBUG_PRINT_UART("BMP388 Init [OK].\n");
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
  		DEBUG_PRINT_UART("BMP388 Setting [FAILED].\n");
		bmp3Settings.op_mode = BMP3_MODE_NORMAL;
		rslt = bmp3_set_op_mode(&bmp3Settings, &bmp3Dev);
		if (rslt != BMP3_OK)
  		DEBUG_PRINT_UART("BMP388 OP Mode [FAILED].\n");
		bmp3Dev.delay_us(10000, NULL);
	}

  /*! Init second order filters for accel and gyro */
  for (int i = 0; i < 3; i++) {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accelLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = radians(cosf(configblockGetCalibPitch()));
  sinPitch = radians(sinf(configblockGetCalibPitch()));
  cosRoll = radians(cosf(configblockGetCalibRoll()));
  sinRoll = radians(sinf(configblockGetCalibRoll()));
}

static void sensorsTaskInit() {
	STATIC_MUTEX_CREATE(accelDataMutex);
	STATIC_MUTEX_CREATE(gyroDataMutex);
	STATIC_MUTEX_CREATE(baroDataMutex);

	STATIC_SEMAPHORE_CREATE(devDataReady, 1, 0);
  STATIC_SEMAPHORE_CREATE(readDataReady, 1, 0);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

void sensorsBmi088Bmp388Init(SensorsInterfaceType interface) {
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
  // sensorsTaskInit();
	isInit = true;
}

bool sensorsBmi088Bmp388Test(void) {
  return isInit && sensorsBmi088Bmp388ManufacturingTest();
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

bool sensorsBmi088Bmp388ManufacturingTest(void) {
  bool testStatus = true;

  int8_t rslt = 0;
  rslt = bmi08a_perform_selftest(&bmi08xDev);
  if (rslt != BMI08X_W_SELF_TEST_FAIL) {
    DEBUG_PRINT("BMI088 Accel self-test [OK].\n");
  } else {
    DEBUG_PRINT("BMI088 Accel self-test [FAILED].\n");
    testStatus = false;
  }

  rslt = bmi08g_perform_selftest(&bmi08xDev);
  if (rslt != BMI08X_W_SELF_TEST_FAIL) {
    DEBUG_PRINT("BMI088 Gyro self-test [OK].\n");
  } else {
    DEBUG_PRINT("BMI088 Gyro self-test [FAILED].\n");
    testStatus = false;
  }
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

void sensorsBmi088Bmp388SetAccMode(accModes accMode) {
  switch (accMode) {
    case ACC_MODE_PROPTEST:
      /* set bandwidth and range of accel (280Hz cut-off according to datasheet) */
      bmi08xDev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
      bmi08xDev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
      bmi08xDev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
      if (bmi08a_set_meas_conf(&bmi08xDev) != BMI08X_OK)
        // TODO: REMOVE UART
        DEBUG_PRINT_UART("ACC config [FAIL]\n");
      for (uint8_t i = 0; i < 3; i++)
        lpf2pInit(&accelLpf[i], 1000, 500);
      break;
    case ACC_MODE_FLIGHT:
    default:
      /* set bandwidth and range of accel (145Hz cut-off according to datasheet) */
      bmi08xDev.accel_cfg.bw = BMI08X_ACCEL_BW_OSR4;
      bmi08xDev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
      bmi08xDev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
      if (bmi08a_set_meas_conf(&bmi08xDev) != BMI08X_OK)
        DEBUG_PRINT_UART("ACC config [FAIL]\n");
      for (uint8_t i = 0; i < 3; i++)
        lpf2pInit(&accelLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
      break;
  }
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f* in) {
  for (uint8_t i = 0; i < 3; i++)
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
}

void sensorsBmi088Bmp388DataAvailableCallback(void) {
  imuIntTimestamp = usecTimerStamp();
  osSemaphoreRelease(devDataReady);
}
