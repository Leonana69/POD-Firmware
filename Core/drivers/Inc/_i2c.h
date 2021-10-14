#ifndef __UI2C_H__
#define __UI2C_H__

/*
 * I2C
 */
#include "i2c.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "stdbool.h"

typedef struct {
	I2C_HandleTypeDef* hi2c;
	osSemaphoreId_t i2cBusSemaphore;
	StaticSemaphore_t i2cBusSemaphoreBuffer;
} I2CDrv;

extern I2CDrv eepromI2C;
extern I2CDrv sensorI2C;

void _I2C_Init();
bool I2CRead16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);
bool I2CWrite16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);

bool I2CRead8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);
bool I2CWrite8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);

int8_t i2cSensorsRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t i2cSensorsWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

void sensorsI2cDmaIsr();
#endif
