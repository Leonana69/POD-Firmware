/* pwm3901.c: PMW3901 driver */
#define DEBUG_MODULE "PMW"
#include "config.h"
#include "_spi.h"
#include "flow.h"
#include "debug.h"
#include "static_mem.h"
#include "system.h"
#include "stabilizer_types.h"
#include "estimator.h"
#include <stdlib.h>
#include "cal.h"

#define PMW_RATE RATE_50_HZ
#define DELTA_LIMIT 150
#define FLOW_STDDEV 2.0f

static bool isInit = false;
static SPIDrv *SPIx;

static void flowTask();
STATIC_MEM_TASK_ALLOC(flowTask, FLOW_TASK_STACKSIZE);

#define PMW3901_EN_CS() HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_RESET)
#define PMW3901_DIS_CS() HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_SET)
static bool pmw3901Init();
static void pmw3901ReadMotion(motionBurst_t *motion);

void flowInit() {
  if (isInit)
    return;
  SPIx = &pmw3901SPI;
  if (pmw3901Init())
    STATIC_MEM_TASK_CREATE(flowTask, flowTask, FLOW_TASK_NAME, NULL, FLOW_TASK_PRI);
  
  isInit = true;
}

bool flowTest() {
	return isInit;
}

void flowTask() {
  systemWaitStart();
  uint32_t lastWakeTime = osKernelGetTickCount();
  uint32_t wakeDelay = osKernelGetTickFreq() / PMW_RATE;
  motionBurst_t motionData;
  flowMeasurement_t flowData;

  while (1) {
    osDelayUntil(lastWakeTime + wakeDelay);
		lastWakeTime = osKernelGetTickCount();
    pmw3901ReadMotion(&motionData);
    int16_t dx = -motionData.deltaY;
    int16_t dy = -motionData.deltaX;
    if (abs(dx) < DELTA_LIMIT && abs(dy) < DELTA_LIMIT && motionData.motion == 0xB0) {
      flowData.stdDevX = FLOW_STDDEV;
      flowData.stdDevY = FLOW_STDDEV;
      flowData.dt = 1.0f / PMW_RATE;
      flowData.dpixelx = (float)dx;
      flowData.dpixely = (float)dy;

      estimatorEnqueueFlow(&flowData);
    }
  }
}

static void registerWrite(uint8_t reg, uint8_t value) {
  // Set MSB to 1 for write
  reg |= 0x80u;
  PMW3901_EN_CS();

  spiWriteDma(SPIx, &reg, 1);
  spiWriteDma(SPIx, &value, 1);

  PMW3901_DIS_CS();
}

static uint8_t registerRead(uint8_t reg) {
  uint8_t data = 0;
  // Set MSB to 0 for read
  reg &= ~0x80u;
  PMW3901_EN_CS();

  spiWriteDma(SPIx, &reg, 1);
  spiReadDma(SPIx, &data, 1);

  PMW3901_DIS_CS();
  return data;
}

/** @brief Write the secret sauce registers.
 * Don't ask what these do, the datasheet refuses to explain.
 * They are some proprietary calibration magic.
 * @Source: https://github.com/pimoroni/pmw3901-python/blob/master/library/pmw3901/__init__.py
 */
static void InitRegisters() {
  registerWrite(0x7F, 0x00);
  registerWrite(0x55, 0x01);
  registerWrite(0x50, 0x07);
  registerWrite(0x7F, 0x0E);
  registerWrite(0x43, 0x10);

  if (registerRead(0x67) & 0b10000000)
    registerWrite(0x48, 0x04);
  else
    registerWrite(0x48, 0x02);

  registerWrite(0x7F, 0x00);
  registerWrite(0x51, 0x7B);
  registerWrite(0x50, 0x00);
  registerWrite(0x55, 0x00);
  registerWrite(0x7F, 0x0E);

  if (registerRead(0x73) == 0x00) {
    uint8_t c1 = registerRead(0x70);
    uint8_t c2 = registerRead(0x71);
    if (c1 < 28)
      c1 += 14;
    if (c1 > 28)
      c1 += 11;
    c1 = max(0, min(0x3F, c1));
    c2 = (c2 * 45) / 100;
    registerWrite(0x7F, 0x00);
    registerWrite(0x61, 0xAD);
    registerWrite(0x51, 0x70);
    registerWrite(0x7F, 0x0E);
    registerWrite(0x70, c1);
    registerWrite(0x71, c2);
  }

  registerWrite(0x7F, 0x00);
  registerWrite(0x61, 0xAD);
  registerWrite(0x7F, 0x03);
  registerWrite(0x40, 0x00);
  registerWrite(0x7F, 0x05);
  registerWrite(0x41, 0xB3);
  registerWrite(0x43, 0xF1);
  registerWrite(0x45, 0x14);
  registerWrite(0x5B, 0x32);
  registerWrite(0x5F, 0x34);
  registerWrite(0x7B, 0x08);
  registerWrite(0x7F, 0x06);
  registerWrite(0x44, 0x1B);
  registerWrite(0x40, 0xBF);
  registerWrite(0x4E, 0x3F);
  registerWrite(0x7F, 0x08);
  registerWrite(0x65, 0x20);
  registerWrite(0x6A, 0x18);
  registerWrite(0x7F, 0x09);
  registerWrite(0x4F, 0xAF);
  registerWrite(0x5F, 0x40);
  registerWrite(0x48, 0x80);
  registerWrite(0x49, 0x80);
  registerWrite(0x57, 0x77);
  registerWrite(0x60, 0x78);
  registerWrite(0x61, 0x78);
  registerWrite(0x62, 0x08);
  registerWrite(0x63, 0x50);
  registerWrite(0x7F, 0x0A);
  registerWrite(0x45, 0x60);
  registerWrite(0x7F, 0x00);
  registerWrite(0x4D, 0x11);
  registerWrite(0x55, 0x80);
  registerWrite(0x74, 0x1F);
  registerWrite(0x75, 0x1F);
  registerWrite(0x4A, 0x78);
  registerWrite(0x4B, 0x78);
  registerWrite(0x44, 0x08);
  registerWrite(0x45, 0x50);
  registerWrite(0x64, 0xFF);
  registerWrite(0x65, 0x1F);
  registerWrite(0x7F, 0x14);
  registerWrite(0x65, 0x67);
  registerWrite(0x66, 0x08);
  registerWrite(0x63, 0x70);
  registerWrite(0x7F, 0x15);
  registerWrite(0x48, 0x48);
  registerWrite(0x7F, 0x07);
  registerWrite(0x41, 0x0D);
  registerWrite(0x43, 0x14);
  registerWrite(0x4B, 0x0E);
  registerWrite(0x45, 0x0F);
  registerWrite(0x44, 0x42);
  registerWrite(0x4C, 0x80);
  registerWrite(0x7F, 0x10);
  registerWrite(0x5B, 0x02);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x41);
  registerWrite(0x70, 0x00);

  osDelay(10);

  registerWrite(0x32, 0x44);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x40);
  registerWrite(0x7F, 0x06);
  registerWrite(0x62, 0xF0);
  registerWrite(0x63, 0x00);
  registerWrite(0x7F, 0x0D);
  registerWrite(0x48, 0xC0);
  registerWrite(0x6F, 0xD5);
  registerWrite(0x7F, 0x00);
  registerWrite(0x5B, 0xA0);
  registerWrite(0x4E, 0xA8);
  registerWrite(0x5A, 0x50);
  registerWrite(0x40, 0x80);
}

bool pmw3901Init() {
  /*! reset device */
	PMW3901_DIS_CS();
  osDelay(2);
  PMW3901_EN_CS();
  osDelay(2);
  PMW3901_DIS_CS();
  osDelay(2);
  uint8_t chipId = registerRead(0x00);
  uint8_t invChipId = registerRead(0x5f);

  if (chipId == 0x49 && invChipId == 0xB6) {
    /*! Power on reset */
    registerWrite(0x3a, 0x5a);
    osDelay(5);

    /*! Reading the motion registers one time */
    registerRead(0x02);
    registerRead(0x03);
    registerRead(0x04);
    registerRead(0x05);
    registerRead(0x06);
    osDelay(1);

    InitRegisters();
    DEBUG_PRINT("PMW3901 Init [OK].\n");
    return true;
  }
  return false;
}

void pmw3901ReadMotion(motionBurst_t *motion) {
  motion->motion = registerRead(0x02);
  motion->deltaX = (registerRead(0x04) << 8) | registerRead(0x03);
  motion->deltaY = (registerRead(0x06) << 8) | registerRead(0x05);
  motion->squal = registerRead(0x07);
  motion->shutter = (registerRead(0x0C) << 8) | registerRead(0x0B);
}
