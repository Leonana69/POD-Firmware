#include "_i2c.h"
#include "config.h"
#include "static_mem.h"

#include "debug.h"

I2CDrv eepromI2C;
I2CDrv sensorI2C;

void _I2C_Init() {
    eepromI2C.hi2c = eepromI2CHandle;
    sensorI2C.hi2c = sensorI2CHandle;
    eepromI2C.i2cBusMutex = osMutexNew(getOsMutexAttr_t("EEPROMI2C", &eepromI2C.i2cBusMutexBuffer, sizeof(eepromI2C.i2cBusMutexBuffer)));
    sensorI2C.i2cBusMutex = osMutexNew(getOsMutexAttr_t("SENSORI2C", &sensorI2C.i2cBusMutexBuffer, sizeof(sensorI2C.i2cBusMutexBuffer)));
}

HAL_StatusTypeDef I2CRead16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
    HAL_StatusTypeDef status = HAL_OK;
    osMutexAcquire(dev->i2cBusMutex, osWaitForever);
    status = HAL_I2C_Mem_Read(&dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
    osMutexRelease(dev->i2cBusMutex);
    return status;
}

HAL_StatusTypeDef I2CWrite16(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
    HAL_StatusTypeDef status = HAL_OK;
    osMutexAcquire(dev->i2cBusMutex, osWaitForever);
    status = HAL_I2C_Mem_Write(&dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, data, len, 100);
    osMutexRelease(dev->i2cBusMutex);
    return status;
}

HAL_StatusTypeDef I2CRead8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
    HAL_StatusTypeDef status = HAL_OK;
    osMutexAcquire(dev->i2cBusMutex, osWaitForever);
    status = HAL_I2C_Mem_Read(&dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
    osMutexRelease(dev->i2cBusMutex);
    return status;
}

HAL_StatusTypeDef I2CWrite8(I2CDrv *dev, uint32_t devAddr, uint32_t memAddr, uint16_t len, uint8_t *data) {
    HAL_StatusTypeDef status = HAL_OK;
    osMutexAcquire(dev->i2cBusMutex, osWaitForever);
    status = HAL_I2C_Mem_Write(&dev->hi2c, devAddr, memAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
    osMutexRelease(dev->i2cBusMutex);
    return status;
}
