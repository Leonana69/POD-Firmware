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

#include "sensors_bmi088_bmp388.h"

// #include "FreeRTOS.h"
// #include "semphr.h"
// #include "task.h"

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
// #include "bmp3.h"

// #include "sensors_bmi088_common.h"
static bool isInit = false;
SensorsInterfaceType currentInterface;

#define SENSORS_READ_RATE_HZ            1000
#define SENSORS_STARTUP_TIME_MS         1000
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ / SENSORS_READ_BARO_HZ)

#define SENSORS_BMI088_GYRO_FS_CFG      BMI088_GYRO_RANGE_2000_DPS
#define SENSORS_BMI088_DEG_PER_LSB_CFG  (2.0f * 2000.0f) / 65536.0f

#define SENSORS_BMI088_ACCEL_FS_CFG     BMI088_ACCEL_RANGE_24G
#define SENSORS_BMI088_G_PER_LSB_CFG    (2.0f * (float)24) / 65536.0f
#define SENSORS_BMI088_1G_IN_LSB        (65536 / 24 / 2)

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT   1000 // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX          5.0f // Max degrees off

#define GYRO_DIM 3
#define GYRO_MIN_BIAS_TIMEOUT_MS 1000

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE              10000
#define GYRO_VARIANCE_THRESHOLD_X       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z       (GYRO_VARIANCE_BASE)

#define SENSORS_ACC_SCALE_SAMPLES  200

typedef struct {
  Axis3f bias;
  Axis3f variance;
  Axis3f mean;
  bool isBiasValueFound;
  bool isBufferFilled;
  Axis3i16* bufHead;
  Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

/* initialize necessary variables */
// TODO: realize 388
// static struct bmp3_dev   bmp388Dev;

STATIC_MEM_MUTEX_ALLOC(accelMutex);
STATIC_MEM_MUTEX_ALLOC(gyroMutex);
STATIC_MEM_MUTEX_ALLOC(baroMutex);
static Axis3f accelData;
static Axis3f gyroData;
static baro_t baroData;

/*! @brief This structure containing relevant bmi08x info */
struct bmi08x_dev bmi08xdev;
/*! @brief variable to hold the bmi08x accel data */
struct bmi08x_sensor_data bmi08x_accel;
/*! @brief variable to hold the bmi08x gyro data */
struct bmi08x_sensor_data bmi08x_gyro;
/*! bmi08x accel int config */
struct bmi08x_accel_int_channel_cfg accel_int_config;
/*! bmi08x gyro int config */
struct bmi08x_gyro_int_channel_cfg gyro_int_config;

STATIC_MEM_SEMAPHORE_ALLOC(devDataReady);
STATIC_MEM_SEMAPHORE_ALLOC(readDataReady);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 accelRaw;
static Axis3i16 gyroRaw;

NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f gyroBiasStdDev;
#endif
static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;
static bool accScaleFound = false;
static uint32_t accScaleSumCount = 0;

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);

static bool isBarometerPresent = false;
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO;

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);

static void sensorsTaskInit(void);
static void sensorsI2CInit();
// TODO: do this
// static void sensorsSPIInit();

// Communication routines

/*!
 * @brief Generic burst read
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
bstdr_ret_t bmi088_burst_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  /**< Burst read code comes here */
  if (i2cdevReadReg8(I2C3_DEV, dev_id, reg_addr, (uint16_t) len, reg_data))
  {
    return BSTDR_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}

/*!
 * @brief Generic burst write
 *
 * @param [out] dev_id I2C address, SPI chip select or user desired identifier
 *
 * @return Zero if successful, otherwise an error code
 */
bstdr_ret_t bmi088_burst_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  /**< Burst write code comes here */
  if (i2cdevWriteReg8(I2C3_DEV, dev_id,reg_addr,(uint16_t) len, reg_data))
  {
    return BSTDR_OK;
  }
  else
  {
    return BSTDR_E_CON_ERROR;
  }
}

/*!
 * @brief Generic burst read
 *
 * @param [in] period Delay period in milliseconds
 *
 * @return None
 */
void bmi088_ms_delay(uint32_t period)
{
  /**< Delay code comes */
  vTaskDelay(M2T(period)); // Delay a while to let the device stabilize
}

static uint16_t sensorsGyroGet(Axis3i16* dataOut)
{
  return bmi088_get_gyro_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

static void sensorsAccelGet(Axis3i16* dataOut)
{
  bmi088_get_accel_data((struct bmi088_sensor_data*)dataOut, &bmi088Dev);
}

static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature)
{
  baroScaled->pressure = pressure*0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsBmi088Bmp388ReadGyro(Axis3f *gyro) {
	osStatus_t s;
	s = osMutexAcquire(gyroMutex, 0);
	gyro = gyroData;
	osMutexRelease(gyroMutex);
	return (s == osOK);
}

bool sensorsBmi088Bmp388ReadAcc(Axis3f *accel) {
  osStatus_t s;
	s = osMutexAcquire(accelMutex, 0);
	accel = accelData;
	osMutexRelease(accelMutex);
	return (s == osOK);
}

bool sensorsBmi088Bmp388ReadBaro(baro_t *baro) {
  osStatus_t s;
	s = osMutexAcquire(baroMutex, 0);
	baro = baroData;
	osMutexRelease(baroMutex);
	return (s == osOK);
}

void sensorsBmi088Bmp388Acquire(sensorData_t *sensors) {
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadBaro(&sensors->baro);
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi088Bmp388AreCalibrated() {
  return gyroBiasFound;
}

static void sensorsTask(void *param) {
  systemWaitStart();

  Axis3f accScaled;
  measurement_t measurement;
  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1) {
    if (osOK == osSemaphoreAcquire(devDataReady, osWaitForever)) {
      sensorData.interruptTimestamp = imuIntTimestamp;

      /* get data from chosen sensors */
      sensorsGyroGet(&gyroRaw);
      sensorsAccelGet(&accelRaw);

      /* calibrate if necessary */
#ifdef GYRO_BIAS_LIGHT_WEIGHT
      gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
      gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif
      if (gyroBiasFound)
      {
         processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
      }
      /* Gyro */
      sensorData.gyro.x =  (gyroRaw.x - gyroBias.x) * SENSORS_BMI088_DEG_PER_LSB_CFG;
      sensorData.gyro.y =  (gyroRaw.y - gyroBias.y) * SENSORS_BMI088_DEG_PER_LSB_CFG;
      sensorData.gyro.z =  (gyroRaw.z - gyroBias.z) * SENSORS_BMI088_DEG_PER_LSB_CFG;
      applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);

      measurement.type = MeasurementTypeGyroscope;
      measurement.data.gyroscope.gyro = sensorData.gyro;
      estimatorEnqueue(&measurement);

      /* Accelerometer */
      accScaled.x = accelRaw.x * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
      accScaled.y = accelRaw.y * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
      accScaled.z = accelRaw.z * SENSORS_BMI088_G_PER_LSB_CFG / accScale;
      sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
      applyAxis3fLpf((lpf2pData*)(&accLpf), &sensorData.acc);

      measurement.type = MeasurementTypeAcceleration;
      measurement.data.acceleration.acc = sensorData.acc;
      estimatorEnqueue(&measurement);
    }

    if (isBarometerPresent)
    {
      static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
      if (--baroMeasDelay == 0)
      {
        uint8_t sensor_comp = BMP3_PRESS | BMP3_TEMP;
        struct bmp3_data data;
        baro_t* baro388 = &sensorData.baro;
        /* Temperature and Pressure data are read and stored in the bmp3_data instance */
        bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);
        sensorsScaleBaro(baro388, data.pressure, data.temperature);

        measurement.type = MeasurementTypeBarometer;
        measurement.data.barometer.baro = sensorData.baro;
        estimatorEnqueue(&measurement);

        baroMeasDelay = baroMeasDelayMin;
      }
    }
    xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
    xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
    if (isBarometerPresent)
    {
      xQueueOverwrite(barometerDataQueue, &sensorData.baro);
    }

    xSemaphoreGive(dataReady);
  }
}

void sensorsBmi088Bmp388WaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

static void sensorsDeviceInit(void) {
  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

  /* BMI088
   * The bmi088Dev structure should have been filled in by the backend
   * (i2c/spi) drivers at this point.
  */

  /* BMI088 GYRO */
  rslt = bmi088_gyro_init(&bmi088Dev); // initialize the device
  if (rslt == BSTDR_OK) {
    struct bmi088_int_cfg intConfig;

    DEBUG_PRINT("BMI088 Gyro connection [OK].\n");
    /* set power mode of gyro */
    bmi088Dev.gyro_cfg.power = BMI088_GYRO_PM_NORMAL;
    rslt |= bmi088_set_gyro_power_mode(&bmi088Dev);
    /* set bandwidth and range of gyro */
    bmi088Dev.gyro_cfg.bw = BMI088_GYRO_BW_116_ODR_1000_HZ;
    bmi088Dev.gyro_cfg.range = SENSORS_BMI088_GYRO_FS_CFG;
    bmi088Dev.gyro_cfg.odr = BMI088_GYRO_BW_116_ODR_1000_HZ;
    rslt |= bmi088_set_gyro_meas_conf(&bmi088Dev);

    intConfig.gyro_int_channel = BMI088_INT_CHANNEL_3;
    intConfig.gyro_int_type = BMI088_GYRO_DATA_RDY_INT;
    intConfig.gyro_int_pin_3_cfg.enable_int_pin = 1;
    intConfig.gyro_int_pin_3_cfg.lvl = 1;
    intConfig.gyro_int_pin_3_cfg.output_mode = 0;
    /* Setting the interrupt configuration */
    rslt = bmi088_set_gyro_int_config(&intConfig, &bmi088Dev);

    bmi088Dev.delay_ms(50);
    struct bmi088_sensor_data gyr;
    rslt |= bmi088_get_gyro_data(&gyr, &bmi088Dev);
  }
  else
  {
    DEBUG_PRINT("BMI088 Gyro connection [FAIL]\n");
  }

  /* BMI088 ACCEL */
  rslt |= bmi088_accel_switch_control(&bmi088Dev, BMI088_ACCEL_POWER_ENABLE);
  bmi088Dev.delay_ms(5);

  rslt = bmi088_accel_init(&bmi088Dev); // initialize the device
  if (rslt == BSTDR_OK)
  {
    DEBUG_PRINT("BMI088 Accel connection [OK]\n");
    /* set power mode of accel */
    bmi088Dev.accel_cfg.power = BMI088_ACCEL_PM_ACTIVE;
    rslt |= bmi088_set_accel_power_mode(&bmi088Dev);
    bmi088Dev.delay_ms(10);

    /* set bandwidth and range of accel */
    bmi088Dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
    bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
    bmi088Dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
    rslt |= bmi088_set_accel_meas_conf(&bmi088Dev);

    struct bmi088_sensor_data acc;
    rslt |= bmi088_get_accel_data(&acc, &bmi088Dev);
  }
  else
  {
    DEBUG_PRINT("BMI088 Accel connection [FAIL]\n");
  }

  /* BMP388 */
  bmp388Dev.dev_id = BMP3_I2C_ADDR_SEC;
  bmp388Dev.intf = BMP3_I2C_INTF;
  bmp388Dev.read = bmi088_burst_read;
  bmp388Dev.write = bmi088_burst_write;
  bmp388Dev.delay_ms = bmi088_ms_delay;

  int i = 3;
  do {
    bmp388Dev.delay_ms(1);
    // For some reason it often doesn't work first time
    rslt = bmp3_init(&bmp388Dev);
  } while (rslt != BMP3_OK && i-- > 0);

  if (rslt == BMP3_OK) {
    isBarometerPresent = true;
    DEBUG_PRINT("BMP388 I2C connection [OK]\n");
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;
    /* Select the pressure and temperature sensor to be enabled */
    bmp388Dev.settings.press_en = BMP3_ENABLE;
    bmp388Dev.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    bmp388Dev.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    bmp388Dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    bmp388Dev.settings.odr_filter.odr = BMP3_ODR_50_HZ;
    bmp388Dev.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, &bmp388Dev);

    /* Set the power mode to normal mode */
    bmp388Dev.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&bmp388Dev);


    bmp388Dev.delay_ms(20); // wait before first read out
    // read out data
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &bmp388Dev);

    /* Print the temperature and pressure data */
//    DEBUG_PRINT("BMP388 T:%0.2f  P:%0.2f\n",data.temperature, data.pressure/100.0f);
    baroMeasDelayMin = SENSORS_DELAY_BARO;
  }
  else
  {
    DEBUG_PRINT("BMP388 I2C connection [FAIL]\n");
    return;
  }

  // Init second order filer for accelerometer and gyro
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
  }

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);
}

static void sensorsTaskInit(void) {
	STATIC_MUTEX_CREATE(accelMutex);
	STATIC_MUTEX_CREATE(gyroMutex);
	STATIC_MUTEX_CREATE(baroMutex);

	STATIC_SEMAPHORE_CREATE(devDataReady);
  STATIC_SEMAPHORE_CREATE(readDataReady);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

static void sensorsI2CInit() {
	DEBUG_PRINT("BMI088: Using I2C interface.\n");
}

static void sensorsBmi088Bmp388Init(SensorsInterfaceType interface) {
	if (isInit)
		return;
	currentInterface = interface;
	// TODO: check this
  sensorsBiasObjInit(&gyroBiasRunning);

	int8_t rslt;
  rslt = bmi08a_init(&bmi08xdev);

  sensorsDeviceInit();
  sensorsTaskInit();
	isInit = true;
}

void sensorsBmi088Bmp388Init_SPI(void)
{
  if (isInit)
    {
      return;
    }

    DEBUG_PRINT("BMI088: Using SPI interface.\n");
    sensorsBmi088_SPI_deviceInit(&bmi088Dev);
    sensorsBmi088Bmp388Init();
}

static bool gyroSelftest()
{
  bool testStatus = true;

  int i = 3;
  uint16_t readResult = BMI088_OK;
  do {
    // For some reason it often doesn't work first time on the Roadrunner
    readResult = sensorsGyroGet(&gyroRaw);
  } while (readResult != BMI088_OK && i-- > 0);

  if ((readResult != BMI088_OK) || (gyroRaw.x == 0 && gyroRaw.y == 0 && gyroRaw.z == 0))
  {
    DEBUG_PRINT("BMI088 gyro returning x=0 y=0 z=0 [FAILED]\n");
    testStatus = false;
  }

  int8_t gyroResult = 0;
  bmi088_perform_gyro_selftest(&gyroResult, &bmi088Dev);
  if (gyroResult == BMI088_SELFTEST_PASS)
  {
    DEBUG_PRINT("BMI088 gyro self-test [OK]\n");
  }
  else
  {
    DEBUG_PRINT("BMI088 gyro self-test [FAILED]\n");
    testStatus = false;
  }

  return testStatus;
}

bool sensorsBmi088Bmp388Test(void) {
  bool testStatus = true;

  if (!isInit){
    DEBUG_PRINT("Uninitialized\n");
    testStatus = false;
  }

  if (!gyroSelftest())
    testStatus = false;
  return testStatus;
}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az) {
  if (!accScaleFound) {
    accScaleSum += sqrtf(powf(ax * SENSORS_BMI088_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_BMI088_G_PER_LSB_CFG, 2) + powf(az * SENSORS_BMI088_G_PER_LSB_CFG, 2));
    accScaleSumCount++;

    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accScaleFound = true;
    }
  }

  return accScaleFound;
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT

#define SENSORS_BIAS_SAMPLES       1000
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  static uint32_t gyroBiasSampleCount = 0;
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;

  if (!gyroBiasNoBuffFound)
  {
    // If the gyro has not yet been calibrated:
    // Add the current sample to the running mean and variance
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    gyroBiasSampleCount += 1;

    // If we then have enough samples, calculate the mean and standard deviation
    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES)
    {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
      gyroBiasNoBuffFound = true;
    }
  }

  return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut) {
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

  if (!gyroBiasRunning.isBiasValueFound) {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound) {
      soundSetEffect(SND_CALIB);
      ledseqRun(&seq_calibrated);
    }
  }

  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;

  return gyroBiasRunning.isBiasValueFound;
}
#endif

static void sensorsBiasObjInit(BiasObj* bias) {
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut) {
  uint32_t i;
  int64_t sum[GYRO_DIM] = {0};
  int64_t sumSq[GYRO_DIM] = {0};
	// TODO: check this
  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

  meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut) {
  int32_t sum[GYRO_DIM] = { 0 };

  for (int i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z) {
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool sensorsFindBiasValue(BiasObj* bias) {
  static int32_t varianceSampleTime;
  bool foundBias = false;

  if (bias->isBufferFilled) {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }

  return foundBias;
}

bool sensorsBmi088Bmp388ManufacturingTest(void) {
  bool testStatus = true;
  if (!gyroSelftest())
    testStatus = false;

  int8_t accResult = 0;
  bmi088_perform_accel_selftest(&accResult, &bmi088Dev);
  if (accResult == BMI088_SELFTEST_PASS) {
    DEBUG_PRINT("BMI088 acc self-test [OK]\n");
  } else {
    DEBUG_PRINT("BMI088 acc self-test [FAILED]\n");
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
//      bmi088_accel_soft_reset(&bmi088Dev);
      /* set bandwidth and range of accel (280Hz cut-off according to datasheet) */
      bmi088Dev.accel_cfg.bw = BMI088_ACCEL_BW_NORMAL;
      bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
      bmi088Dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
      if (bmi088_set_accel_meas_conf(&bmi088Dev) != BMI088_OK) {
        DEBUG_PRINT("ACC config [FAIL]\n");
      }
      for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i],  1000, 500);
      }
      break;
    case ACC_MODE_FLIGHT:
    default:
      /* set bandwidth and range of accel (145Hz cut-off according to datasheet) */
      bmi088Dev.accel_cfg.bw = BMI088_ACCEL_BW_OSR4;
      bmi088Dev.accel_cfg.range = SENSORS_BMI088_ACCEL_FS_CFG;
      bmi088Dev.accel_cfg.odr = BMI088_ACCEL_ODR_1600_HZ;
      if (bmi088_set_accel_meas_conf(&bmi088Dev) != BMI088_OK) {
        DEBUG_PRINT("ACC config [FAIL]\n");
      }
      for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
      }
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

PARAM_GROUP_START(imu_sensors)

/**
 * @brief Nonzero if BMP388 barometer is present
 */
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP388, &isBarometerPresent)
PARAM_GROUP_STOP(imu_sensors)
