#include "_usart.h"
#include "static_mem.h"
#include "config.h"
#include "cmsis_os2.h"
#include "syslink.h"
#include "dma.h"
#include <string.h>

#include "queuemonitor.h"
#include "debug.h"
#include "crtp.h"

#include "led.h"

STATIC_MEM_SEMAPHORE_ALLOC(nrfUartWaitSemaphore);
STATIC_MEM_MUTEX_ALLOC(nrfUartBusMutex);
STATIC_MEM_QUEUE_ALLOC(syslinkPacketDelivery, 8, sizeof(SyslinkPacket));

static uint8_t nrfUartTxDmaBuffer[64];

extern DMA_HandleTypeDef nrfUartTxDmaHandle;

void _UART_Init(void) {
	STATIC_SEMAPHORE_CREATE(nrfUartWaitSemaphore, 1, 0);
	STATIC_MUTEX_CREATE(nrfUartBusMutex);

	STATIC_MEM_QUEUE_CREATE(syslinkPacketDelivery);
	DEBUG_QUEUE_MONITOR_REGISTER(syslinkPacketDelivery);
	// TODO: check this
	// HAL_DMA_RegisterCallback(&nrfUartTxDmaHandle, HAL_DMA_XFER_HALFCPLT_CB_ID, nrfUartDmaIsr);
	__HAL_UART_ENABLE_IT(&nrfUart, UART_IT_RXNE);
}

int debugUartPutchar(int c) {
	HAL_UART_Transmit(&debugUart, (uint8_t*) &c, 1, 100);
	return (unsigned char) c;
}

void nrfUartSendData(uint32_t size, uint8_t *data) {
	HAL_UART_Transmit(&nrfUart, data, size, 100);
}

// static uint8_t *outDataIsr;
// static uint8_t dataIndexIsr;
// static uint8_t dataSizeIsr;

// void nrfUartSendDataIsrBlocking(uint32_t size, uint8_t *data) {
// 	osSemaphoreAcquire(nrfUartBusyS, osWaitForever);
// 	outDataIsr = data;
// 	dataSizeIsr = size;
// 	dataIndexIsr = 1;
// 	nrfUartSendData(1, &data[0]);
// 	__HAL_UART_ENABLE_IT(&nrfUart, UART_IT_TXE);
// 	// USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, ENABLE);
// 	osSemaphoreAcquire(nrfUartWaitSemaphore, osWaitForever);
// 	outDataIsr = 0;
// 	osSemaphoreRelease(nrfUartBusyS);
// }

int nrfUartPutchar(int ch) {
	nrfUartSendData(1, (uint8_t *)&ch);
	return (unsigned char)ch;
}

void nrfUartSendDataDmaBlocking(uint32_t size, uint8_t *data) {      
	osMutexAcquire(nrfUartBusMutex, osWaitForever);
	while (HAL_DMA_GetState(&nrfUartTxDmaHandle) != HAL_DMA_STATE_READY);
	memcpy(nrfUartTxDmaBuffer, data, size);
	HAL_UART_Transmit_DMA(&nrfUart, nrfUartTxDmaBuffer, size);
	osSemaphoreAcquire(nrfUartWaitSemaphore, osWaitForever);
	osMutexRelease(nrfUartBusMutex);
}

void nrfUartGetPacketBlocking(SyslinkPacket* packet) {
	osMessageQueueGet(syslinkPacketDelivery, packet, 0, osWaitForever);
}

void nrfUartTxenFlowCtrlIsr() {
	if (HAL_GPIO_ReadPin(NRF_FC_GPIO_Port, NRF_FC_Pin) == GPIO_PIN_SET)
		HAL_UART_DMAPause(&nrfUart);
	else
		HAL_UART_DMAResume(&nrfUart);
}

void nrfUartDmaIsr() {
	// DMA2_Stream7_IRQHandler will be called twice for each transmission: halpcplt and cplt
	static uint8_t callbackCnt = 0;
	if (callbackCnt++) {
		osSemaphoreRelease(nrfUartWaitSemaphore);
		callbackCnt = 0;
	}
}

static volatile SyslinkPacket slp = { 0 };
static volatile uint8_t dataIndex = 0;
static volatile uint8_t cksum[2] = { 0 };
static volatile SyslinkRxState rxState = waitForFirstStart;
void nrfUartHandleDataFromIsr(uint8_t c) {
	switch (rxState) {
		case waitForFirstStart:
			rxState = (c == SYSLINK_START_BYTE1) ? waitForSecondStart : waitForFirstStart;
			break;
		case waitForSecondStart:
			rxState = (c == SYSLINK_START_BYTE2) ? waitForType : waitForFirstStart;
			break;
		case waitForType:
			cksum[0] = c;
			cksum[1] = c;
			slp.type = c;
			rxState = waitForLength;
			break;
		case waitForLength:
			if (c <= SYSLINK_MTU) {
				slp.length = c;
				cksum[0] += c;
				cksum[1] += cksum[0];
				dataIndex = 0;
				rxState = (c > 0) ? waitForData : waitForChksum1;
			} else
				rxState = waitForFirstStart;
			break;
		case waitForData:
			slp.data[dataIndex] = c;
			cksum[0] += c;
			cksum[1] += cksum[0];
			dataIndex++;
			if (dataIndex == slp.length) {
				rxState = waitForChksum1;
			}
			break;
		case waitForChksum1:
			if (cksum[0] == c) {
				rxState = waitForChksum2;
			} else
				rxState = waitForFirstStart; //Checksum error
			break;
		case waitForChksum2:
			if (cksum[1] == c) {
				// Post the packet to the queue if there's room
				if (osMessageQueueGetSpace(syslinkPacketDelivery)) {
					osMessageQueuePut(syslinkPacketDelivery, (void *)&slp, 0, 0);
				} else {
					ASSERT(0); // Queue overflow
				}
			} else {
				rxState = waitForFirstStart; //Checksum error
				ASSERT(0);
			}
			rxState = waitForFirstStart;
			break;
		default:
			ASSERT(0);
			break;
  }
}

void nrfUartIsr() {
	if (__HAL_UART_GET_FLAG(&nrfUart, UART_FLAG_RXNE) != 0)
		nrfUartHandleDataFromIsr(__HAL_UART_FLUSH_DRREGISTER(&nrfUart));
	__HAL_UART_ENABLE_IT(&nrfUart, UART_IT_RXNE);

	// TODO: fix this
	// if (((nrfUart.Instance->SR & USART_SR_TXE) != RESET) && ((nrfUart.Instance->CR1 & USART_CR1_TXEIE) != RESET))
	// {
	// 	uint16_t *tmp;

	// 	/* Check that a Tx process is ongoing */
	// 	if (nrfUart.gState == HAL_UART_STATE_BUSY_TX)
	// 	{
	// 		if ((nrfUart.Init.WordLength == UART_WORDLENGTH_9B) && (nrfUart.Init.Parity == UART_PARITY_NONE))
	// 		{
	// 			tmp = (uint16_t *) nrfUart.pTxBuffPtr;
	// 			nrfUart.Instance->DR = (uint16_t)(*tmp & (uint16_t)0x01FF);
	// 			nrfUart.pTxBuffPtr += 2U;
	// 		}
	// 		else
	// 		{
	// 			nrfUart.Instance->DR = (uint8_t)(*nrfUart.pTxBuffPtr++ & (uint8_t)0x00FF);
	// 		}

	// 		if (--nrfUart.TxXferCount == 0U)
	// 		{
	// 			/* Disable the UART Transmit Complete Interrupt */
	// 			__HAL_UART_DISABLE_IT(&nrfUart, UART_IT_TXE);

	// 			/* Enable the UART Transmit Complete Interrupt */
	// 			__HAL_UART_ENABLE_IT(&nrfUart, UART_IT_TC);
	// 		}
	// 	}
	// 	return;
	// }

  // /* UART in mode Transmitter end --------------------------------------------*/
  // if (((nrfUart.Instance->SR & USART_SR_TC) != RESET) && ((nrfUart.Instance->CR1 & USART_CR1_TCIE) != RESET))
  // {
  //   /* Disable the UART Transmit Complete Interrupt */
  // 	__HAL_UART_DISABLE_IT(&nrfUart, UART_IT_TC);

  // 	/* Tx process is ended, restore &nrfUart->gState to Ready */
  // 	nrfUart.gState = HAL_UART_STATE_READY;
  //   return;
  // }

	// if (__HAL_UART_GET_FLAG(&nrfUart, UART_FLAG_RXNE) != 0) {
		// uint8_t rxDataInterrupt = (uint8_t)(huart6.Instance->DR & 0xFF);
	// 	nrfUartHandleDataFromIsr(rxDataInterrupt);
	// }
	// else if (__HAL_UART_GET_FLAG(&nrfUart, UART_FLAG_TXE)) {
		
	// 	if (outDataIsr && (dataIndexIsr < dataSizeIsr)) {
	// 		DEBUG_PRINT("tx\n");
  	//     HAL_UART_Transmit(&nrfUart, &outDataIsr[dataIndexIsr], 1, HAL_TIMEOUT);
  	//     dataIndexIsr++;
  	//   } else {

  	//     __HAL_UART_DISABLE_IT(&nrfUart, UART_IT_TXE);
  	//   	osSemaphoreRelease(nrfUartWaitSemaphore);
  	//   }
	// }
}
