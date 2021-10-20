#include <string.h>
#include "_spi.h"
#include "config.h"
#include "debug.h"
#include "static_mem.h"

SPIDrv pmw3901SPI;

void _SPI_Init() {
	pmw3901SPI.hspi = &PMW3901SpiHandle;
	pmw3901SPI.spiRxDmaSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("PMW3901RX", &pmw3901SPI.spiRxDmaSemaphoreBuffer, sizeof(pmw3901SPI.spiRxDmaSemaphoreBuffer)));
	pmw3901SPI.spiRxDmaSemaphore = osSemaphoreNew(1, 0, getOsSemaphoreAttr_t("PMW3901TX", &pmw3901SPI.spiRxDmaSemaphoreBuffer, sizeof(pmw3901SPI.spiRxDmaSemaphoreBuffer)));
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
	osSemaphoreAcquire(pmw3901SPI.spiRxDmaSemaphore, osWaitForever);
	return status == HAL_OK;
}

bool spiWriteDma(SPIDrv *dev, uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef status;
	status = HAL_SPI_Transmit_DMA(dev->hspi, data, len);
	osSemaphoreAcquire(pmw3901SPI.spiTxDmaSemaphore, osWaitForever);
	return status == HAL_OK;
}
