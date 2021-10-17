#ifndef __UI2C_H__
#define __UI2C_H__

/*! I2C wrappers */
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
extern I2CDrv tofI2C;

void _I2C_Init();
bool i2cMemReadDma16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);
bool i2cMemWrite16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);

bool i2cMemReadDma8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);
bool i2cMemWrite8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data);

uint8_t i2cTofReadDma(I2CDrv *dev, uint32_t devAddr, uint32_t regAddr, uint16_t len, uint8_t *data);
uint8_t i2cTofWrite(I2CDrv *dev, uint32_t devAddr, uint32_t regAddr, uint16_t len, uint8_t *data);

int8_t i2cSensorsRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
int8_t i2cSensorsWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);

void sensorsI2cDmaIsr();
void eepromI2cDmaIsr();

#endif
