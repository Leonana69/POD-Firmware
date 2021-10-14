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
	sensorI2C.hi2c = &sensorsI2CHandle;
	tofI2C.hi2c = &tofI2CHandle;
	eepromI2C.i2cBusSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("EEPROMI2C", &eepromI2C.i2cBusSemaphoreBuffer, sizeof(eepromI2C.i2cBusSemaphoreBuffer)));
	sensorI2C.i2cBusSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("SENSORI2C", &sensorI2C.i2cBusSemaphoreBuffer, sizeof(sensorI2C.i2cBusSemaphoreBuffer)));
}

bool i2cMemRead16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, data, len, 100);

	return status == HAL_OK;
}

bool i2cMemWrite16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
	return status == HAL_OK;
}

bool i2cMemRead8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	return status == HAL_OK;
}

bool i2cMemWrite8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	return status == HAL_OK;
}

bool i2cReadDma(I2CDrv *dev, uint8_t devAddr, uint8_t regAddr, uint32_t len, uint8_t *data) {
	HAL_I2C_Master_Transmit(sensorI2C.hi2c, devAddr, &regAddr, 1, 1000);
	return 1;
}

bool i2cWriteDma(I2CDrv *dev, uint8_t devAddr, uint8_t regAddr, uint32_t len, uint8_t *data) {
	// HAL_I2C_Master_Transmit(sensorI2C.hi2c, devAddr, &regAddr, 1, 1000);
	return 1;
}

/*! @brief Sensor I2C read function */
int8_t i2cSensorsRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr) {
	HAL_StatusTypeDef status;
  uint16_t DevAddress = *(uint8_t*)intfPtr << 1;
  HAL_I2C_Master_Transmit(sensorI2C.hi2c, DevAddress, &regAddr, 1, 1000);
  status = HAL_I2C_Master_Receive_DMA(sensorI2C.hi2c, DevAddress, regData, len);
  osSemaphoreAcquire(sensorI2C.i2cBusSemaphore, osWaitForever);
	/**
	 * HAL_StatusTypeDef: 0, 1, 2, 3
	 * BMI08X_INTF_RET_TYPE: 0, -1, -2, ..., -9
	 */
  return -status;
}

/*! @brief Sensor I2C write function */
int8_t i2cSensorsWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr) {
  static uint8_t sBuffer[33];
	HAL_StatusTypeDef status;
  uint16_t DevAddress = *(uint8_t*)intfPtr << 1;
  memset(sBuffer, 0, 33);
  sBuffer[0] = regAddr;
  memcpy(sBuffer + 1, regData, len);
  status = HAL_I2C_Master_Transmit(sensorI2C.hi2c, DevAddress, sBuffer, len + 1, 1000);
  return -status;
}

void sensorsI2cDmaIsr() {
	osSemaphoreRelease(sensorI2C.i2cBusSemaphore);
}