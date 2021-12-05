#include <string.h>
#include "_spi.h"
#include "config.h"
#include "debug.h"
#include "static_mem.h"

SPIDrv pmw3901SPI;
SPIDrv sensorSPI;

void _SPI_Init() {
	pmw3901SPI.hspi = &PMW3901SpiHandle;
	pmw3901SPI.spiRxDmaSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("PMW3901RX", &pmw3901SPI.spiRxDmaSemaphoreBuffer, sizeof(pmw3901SPI.spiRxDmaSemaphoreBuffer)));
	pmw3901SPI.spiTxDmaSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("PMW3901TX", &pmw3901SPI.spiTxDmaSemaphoreBuffer, sizeof(pmw3901SPI.spiTxDmaSemaphoreBuffer)));

	sensorSPI.hspi = &sensorSpiHandle;
	sensorSPI.spiRxDmaSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("SENSORRX", &sensorSPI.spiRxDmaSemaphoreBuffer, sizeof(sensorSPI.spiRxDmaSemaphoreBuffer)));
}

void pmw3901SpiRxDmaIsr() {
	osSemaphoreRelease(pmw3901SPI.spiRxDmaSemaphore);
}

void pmw3901SpiTxDmaIsr() {
	osSemaphoreRelease(pmw3901SPI.spiTxDmaSemaphore);
}

bool spiReadDma(SPIDrv *dev, uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef status;
	status = HAL_SPI_Receive_DMA(dev->hspi, data, len);
	osSemaphoreAcquire(dev->spiRxDmaSemaphore, osWaitForever);
	return status == HAL_OK;
}

bool spiWriteDma(SPIDrv *dev, uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef status;
	status = HAL_SPI_Transmit_DMA(dev->hspi, data, len);
	osSemaphoreAcquire(dev->spiTxDmaSemaphore, osWaitForever);
	return status == HAL_OK;
}

void sensorsSpiRxDmaIsr() {
	osSemaphoreRelease(sensorSPI.spiRxDmaSemaphore);
}

int8_t spiSensorsRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr) {
	HAL_StatusTypeDef status;
	SENSOR_EN_CS();
	HAL_SPI_Transmit(sensorSPI.hspi, &regAddr, 1, 1000);
	// HAL_Delay(1); // remove
	status = HAL_SPI_Receive(sensorSPI.hspi, regData, len, 100);
	// DEBUG_PRINT("wait\n");
	// osSemaphoreAcquire(sensorSPI.spiRxDmaSemaphore, osWaitForever);
	SENSOR_DIS_CS();
	return -status;
}

int8_t spiSensorsWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr) {
	static uint8_t sBuffer[32];
	HAL_StatusTypeDef status;
	memset(sBuffer, 0, 32);
	sBuffer[0] = regAddr;
	memcpy(sBuffer + 1, regData, len);
	SENSOR_EN_CS();
	status = HAL_SPI_Transmit(sensorSPI.hspi, sBuffer, len + 1, 100);
	SENSOR_DIS_CS();
	return -status;
}