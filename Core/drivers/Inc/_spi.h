#ifndef __USPI_H__
#define __USPI_H__

/*! SPI wrappers */
#include "spi.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "stdbool.h"
#include "config.h"

typedef struct {
	SPI_HandleTypeDef* hspi;
	osSemaphoreId_t spiRxDmaSemaphore;
	StaticSemaphore_t spiRxDmaSemaphoreBuffer;
	osSemaphoreId_t spiTxDmaSemaphore;
	StaticSemaphore_t spiTxDmaSemaphoreBuffer;
} SPIDrv;

extern SPIDrv pmw3901SPI;
extern SPIDrv sensorSPI;

void _SPI_Init();

#define PMW3901_EN_CS() HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_RESET)
#define PMW3901_DIS_CS() HAL_GPIO_WritePin(PMW3901_CS_GPIO_Port, PMW3901_CS_Pin, GPIO_PIN_SET)
bool spiReadDma(SPIDrv *dev, uint8_t *data, uint16_t len);
bool spiWriteDma(SPIDrv *dev, uint8_t *data, uint16_t len);

void pmw3901SpiRxDmaIsr();
void pmw3901SpiTxDmaIsr();

#define SENSOR_EN_CS() HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_RESET)
#define SENSOR_DIS_CS() HAL_GPIO_WritePin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin, GPIO_PIN_SET)
void sensorsSpiRxDmaIsr();
int8_t spiSensorsRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
int8_t spiSensorsWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);

#endif
