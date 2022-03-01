#include <string.h>

#include "_i2c.h"
#include "config.h"
#include "static_mem.h"

#include "debug.h"

I2CDrv eepromI2C;
I2CDrv sensorI2C;
I2CDrv tofI2C;

void _I2C_Init() {
	eepromI2C.hi2c = &eepromI2CHandle;
	sensorI2C.hi2c = &sensorI2CHandle;
	tofI2C.hi2c = &tofI2CHandle;
	eepromI2C.i2cRxDmaSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("EEPROMI2C", &eepromI2C.i2cRxDmaSemaphoreBuffer, sizeof(eepromI2C.i2cRxDmaSemaphoreBuffer)));
	sensorI2C.i2cRxDmaSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("SENSORI2C", &sensorI2C.i2cRxDmaSemaphoreBuffer, sizeof(sensorI2C.i2cRxDmaSemaphoreBuffer)));
	/*! eeprom and tof share the same i2c */
	tofI2C.i2cRxDmaSemaphore = eepromI2C.i2cRxDmaSemaphore;
}

bool i2cMemReadDma16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read_DMA(dev->hi2c, devAddr << 1, memAddr, I2C_MEMADD_SIZE_16BIT, data, len);
	osSemaphoreAcquire(dev->i2cRxDmaSemaphore, osWaitForever);
	return status == HAL_OK;
}

bool i2cMemWrite16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(dev->hi2c, devAddr << 1, memAddr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
	return status == HAL_OK;
}

bool i2cMemReadDma8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read_DMA(dev->hi2c, devAddr << 1, memAddr, I2C_MEMADD_SIZE_8BIT, data, len);
	osSemaphoreAcquire(dev->i2cRxDmaSemaphore, osWaitForever);
	return status == HAL_OK;
}

bool i2cMemWrite8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(dev->hi2c, devAddr << 1, memAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	return status == HAL_OK;
}

uint8_t i2cTofReadDma(I2CDrv *dev, uint32_t devAddr, uint32_t regAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	static uint8_t buffer[2];
	buffer[0] = regAddr >> 8;
	buffer[1] = regAddr & 0xFF;
	HAL_I2C_Master_Transmit(dev->hi2c, devAddr << 1, buffer, 2, 1000);
	status = HAL_I2C_Master_Receive_DMA(dev->hi2c, devAddr << 1, data, len);
	osSemaphoreAcquire(dev->i2cRxDmaSemaphore, osWaitForever);
	return status;
}

uint8_t i2cTofWrite(I2CDrv *dev, uint32_t devAddr, uint32_t regAddr, uint16_t len, uint8_t *data) {
	static uint8_t buffer[256];
	buffer[0] = regAddr >> 8;
	buffer[1] = regAddr & 0xFF;
	memcpy(&buffer[2], data, len);
	return HAL_I2C_Master_Transmit(dev->hi2c, devAddr << 1, buffer, len + 2, 1000);
}

void eepromI2cRxDmaIsr() {
	osSemaphoreRelease(eepromI2C.i2cRxDmaSemaphore);
}

/*! @brief Sensor I2C read function */
int8_t i2cSensorsRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr) {
	HAL_StatusTypeDef status;
	uint16_t DevAddress = *(uint8_t*)intfPtr << 1;
	HAL_I2C_Master_Transmit(sensorI2C.hi2c, DevAddress, &regAddr, 1, 1000);
	status = HAL_I2C_Master_Receive_DMA(sensorI2C.hi2c, DevAddress, regData, len);
	osSemaphoreAcquire(sensorI2C.i2cRxDmaSemaphore, osWaitForever);
	/**
	 * HAL_StatusTypeDef: 0, 1, 2, 3
	 * BMI08X_INTF_RET_TYPE: 0, -1, -2, ..., -9
	 */
	return -status;
}

/*! @brief Sensor I2C write function */
int8_t i2cSensorsWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr) {
	static uint8_t sBuffer[32];
	HAL_StatusTypeDef status;
	uint16_t DevAddress = *(uint8_t*)intfPtr << 1;
	memset(sBuffer, 0, 32);
	sBuffer[0] = regAddr;
	memcpy(sBuffer + 1, regData, len);
	status = HAL_I2C_Master_Transmit(sensorI2C.hi2c, DevAddress, sBuffer, len + 1, 1000);
	return -status;
}

void sensorsI2cRxDmaIsr() {
	osSemaphoreRelease(sensorI2C.i2cRxDmaSemaphore);
}