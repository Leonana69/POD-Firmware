#ifndef __USPI_H__
#define __USPI_H__

/*! SPI wrappers */
#include "spi.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "stdbool.h"

typedef struct {
	SPI_HandleTypeDef* hspi;
	osSemaphoreId_t spiRxDmaSemaphore;
	StaticSemaphore_t spiRxDmaSemaphoreBuffer;
	osSemaphoreId_t spiTxDmaSemaphore;
	StaticSemaphore_t spiTxDmaSemaphoreBuffer;
} SPIDrv;

extern SPIDrv pmw3901SPI;

void _SPI_Init();

bool spiReadDma(SPIDrv *dev, uint8_t *data, uint16_t len);
bool spiWriteDma(SPIDrv *dev, uint8_t *data, uint16_t len);

void pmw3901SpiRxDmaIsr();
void pmw3901SpiTxDmaIsr();

#endif
