#ifndef __UI2C_H__
#define __UI2C_H__

/*
 * I2C
 */
#include "i2c.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"

typedef struct {
    I2C_HandleTypeDef hi2c;
    osMutexId_t i2cBusMutex;
    StaticSemaphore_t i2cBusMutexBuffer;
} I2CDrv;

extern I2CDrv eepromI2C;
extern I2CDrv sensorI2C;

void _I2C_Init();
HAL_StatusTypeDef I2CRead16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);
HAL_StatusTypeDef I2CWrite16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);

HAL_StatusTypeDef I2CRead8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);
HAL_StatusTypeDef I2CWrite8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);
#endif