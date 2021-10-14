#include <string.h>

#include "_i2c.h"
#include "config.h"
#include "static_mem.h"

#include "debug.h"

I2CDrv eepromI2C;
I2CDrv sensorI2C;

STATIC_MEM_MUTEX_ALLOC(eepromI2CMutex);

void _I2C_Init() {
	eepromI2C.hi2c = &eepromI2CHandle;
	sensorI2C.hi2c = &sensorsI2CHandle;
	STATIC_MUTEX_CREATE(eepromI2CMutex);
	eepromI2C.i2cBusSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("EEPROMI2C", &eepromI2C.i2cBusSemaphoreBuffer, sizeof(eepromI2C.i2cBusSemaphoreBuffer)));
	sensorI2C.i2cBusSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("SENSORI2C", &sensorI2C.i2cBusSemaphoreBuffer, sizeof(sensorI2C.i2cBusSemaphoreBuffer)));
}

bool I2CRead16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	osMutexAcquire(eepromI2CMutex, osWaitForever);
	status = HAL_I2C_Mem_Read(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
	osMutexRelease(eepromI2CMutex);
	return status == HAL_OK;
}

bool I2CWrite16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	osMutexAcquire(eepromI2CMutex, osWaitForever);
	status = HAL_I2C_Mem_Write(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
	osMutexRelease(eepromI2CMutex);
	return status == HAL_OK;
}

bool I2CRead8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	osMutexAcquire(eepromI2CMutex, osWaitForever);
	status = HAL_I2C_Mem_Read(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	osMutexRelease(eepromI2CMutex);
	return status == HAL_OK;
}

bool I2CWrite8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
	HAL_StatusTypeDef status;
	osMutexAcquire(eepromI2CMutex, osWaitForever);
	status = HAL_I2C_Mem_Write(dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
	osMutexRelease(eepromI2CMutex);
	return status == HAL_OK;
}

/*! @brief Sensor I2C read function */
int8_t i2cSensorsRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
	HAL_StatusTypeDef status;
  uint16_t DevAddress = *(uint8_t*)intf_ptr << 1;
  HAL_I2C_Master_Transmit(sensorI2C.hi2c, DevAddress, &reg_addr, 1, 1000);
  status = HAL_I2C_Master_Receive_DMA(sensorI2C.hi2c, DevAddress, reg_data, len);
  osSemaphoreAcquire(sensorI2C.i2cBusSemaphore, osWaitForever);
	/**
	 * HAL_StatusTypeDef: 0, 1, 2, 3
	 * BMI08X_INTF_RET_TYPE: 0, -1, -2, ..., -9
	 */
  return -status;
}

/*! @brief Sensor I2C write function */
int8_t i2cSensorsWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
  static uint8_t sBuffer[33];
	HAL_StatusTypeDef status;
  uint16_t DevAddress = *(uint8_t*)intf_ptr << 1;
  memset(sBuffer, 0, 33);
  sBuffer[0] = reg_addr;
  memcpy(sBuffer + 1, reg_data, len);
  status = HAL_I2C_Master_Transmit(sensorI2C.hi2c, DevAddress, sBuffer, len + 1, 1000);
  return -status;
}

void sensorsI2cDmaIsr() {
	osSemaphoreRelease(sensorI2C.i2cBusSemaphore);
}