#include "_usart.h"
#include "static_mem.h"
#include "config.h"
#include "cmsis_os2.h"
#include "syslink.h"
#include "dma.h"
#include <string.h>

#include "queuemonitor.h"
#include "debug.h"
#include "queue.h"

STATIC_MEM_SEMAPHORE_ALLOC(nrfUartWaitS);
STATIC_MEM_SEMAPHORE_ALLOC(nrfUartBusyS);
STATIC_MEM_QUEUE_ALLOC(syslinkPacketDelivery, 8, sizeof(SyslinkPacket));

static uint8_t nrfUartTxDmaBuffer[64];

extern DMA_HandleTypeDef nrfUartTxDmaHandle;

void _UART_Init(void) {

	nrfUartWaitS = STATIC_SEMAPHORE_CREATE(nrfUartWaitS, 1, 0);
	// not busy in the beginning
	nrfUartBusyS = STATIC_SEMAPHORE_CREATE(nrfUartBusyS, 1, 1);

	syslinkPacketDelivery = STATIC_MEM_QUEUE_CREATE(syslinkPacketDelivery);
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

static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;

void nrfUartSendDataIsrBlocking(uint32_t size, uint8_t *data) {
	osSemaphoreAcquire(nrfUartBusyS, osDelayMax);
	outDataIsr = data;
  dataSizeIsr = size;
  dataIndexIsr = 1;
	nrfUartSendData(1, &data[0]);
	__HAL_UART_ENABLE_IT(&nrfUart, UART_IT_TXE);
	// USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, ENABLE);
	osSemaphoreAcquire(nrfUartWaitS, osWaitForever);
	outDataIsr = 0;
	osSemaphoreRelease(nrfUartBusyS);
}

int nrfUartPutchar(int ch) {
	nrfUartSendData(1, (uint8_t *)&ch);
	return (unsigned char)ch;
}

// int DMA_GetCmdStatus() {
// 	if ((nrfUartTxDmaHandle.Instance->CR & (uint32_t)DMA_SxCR_EN) != 0)
//   {
//     /* The selected DMAy Streamx EN bit is set (DMA is still transferring) */
//     return 1;
//   }
//   else
//   {
//     /* The selected DMAy Streamx EN bit is cleared (DMA is disabled and 
//         all transfers are complete) */
//     return 0;
//   }
// }

// void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
// {
//   /* Clear DBM bit */
//   hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

//   /* Configure DMA Stream data length */
//   hdma->Instance->NDTR = DataLength;

//   /* Memory to Peripheral */
//   if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
//   {
//     /* Configure DMA Stream destination address */
//     hdma->Instance->PAR = DstAddress;

//     /* Configure DMA Stream source address */
//     hdma->Instance->M0AR = SrcAddress;
//   }
//   /* Peripheral to Memory */
//   else
//   {
//     /* Configure DMA Stream source address */
//     hdma->Instance->PAR = SrcAddress;

//     /* Configure DMA Stream destination address */
//     hdma->Instance->M0AR = DstAddress;
//   }
// }

void nrfUartSendDataDmaBlocking(uint32_t size, uint8_t *data) {
	DEBUG_PRINT("dma\n");
	osSemaphoreAcquire(nrfUartBusyS, osDelayMax);

	// while (DMA_GetCmdStatus() != 0);
	// memcpy(nrfUartTxDmaBuffer, data, size);
	// DMA_SetConfig(&hdma_usart6_tx, nrfUartTxDmaBuffer, &huart6.Instance->DR, size);
	// __HAL_DMA_ENABLE_IT(&hdma_usart6_tx, DMA_IT_TC);
	// huart6.Instance->CR3 |= ((uint16_t)0x0080);
	// __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, UART_FLAG_TC);
	// hdma_usart6_tx.Instance->CR |= ((uint32_t)0x00000001);

	while(HAL_DMA_GetState(&nrfUartTxDmaHandle) != HAL_DMA_STATE_READY);
	memcpy(nrfUartTxDmaBuffer, data, size);
	HAL_UART_Transmit_DMA(&nrfUart, nrfUartTxDmaBuffer, size);
	osSemaphoreAcquire(nrfUartWaitS, osWaitForever);
	osSemaphoreRelease(nrfUartBusyS);
}

void nrfUartGetPacketBlocking(SyslinkPacket* packet) {
	osMessageQueueGet(syslinkPacketDelivery, packet, 0, osDelayMax);
}

void nrfUartTxenIsr() {
	if (HAL_GPIO_ReadPin(NRF_FC_GPIO_Port, NRF_FC_Pin) == GPIO_PIN_SET)
		HAL_UART_DMAPause(&nrfUart);
	else
		HAL_UART_DMAResume(&nrfUart);
}

static uint8_t callbackCnt = 0;
void nrfUartDmaIsr() {
	// DMA2_Stream7_IRQHandler will be called twice for each transmission: halpcplt and cplt
	callbackCnt ++;
	if (callbackCnt) {
		osSemaphoreRelease(nrfUartWaitS);
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
			} else {
				rxState = waitForFirstStart; //Checksum error
				// if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)) {
				// 	// Only assert if debugger is not connected
				// 	ASSERT(0);
				// }
			}
			break;
		case waitForChksum2:
			if (cksum[1] == c) {
				// Post the packet to the queue if there's room
				if (osMessageQueueGetSpace(syslinkPacketDelivery)) {
					osMessageQueuePut(syslinkPacketDelivery, (void *)&slp, 0, 0);
					// xQueueSendFromISR(syslinkPacketDelivery, (void *)&slp, pxHigherPriorityTaskWoken);
					if (slp.type != 4)
					DEBUG_PRINT("#s:%d,%d,%sn", slp.type, slp.length, slp.data);
				} else {
						ASSERT(0); // Queue overflow
				}
				// else if (!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)) {
				// 	// Only assert if debugger is not connected
				// 	DEBUG_PRINT("over\n");
				// 	while(1);
				// 	ASSERT(0); // Queue overflow
				// }
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
	if (__HAL_UART_GET_FLAG(&nrfUart, UART_FLAG_RXNE)) {
		uint8_t rxDataInterrupt = (uint8_t)(huart6.Instance->DR & 0xFF);
		nrfUartHandleDataFromIsr(rxDataInterrupt);
	}
	// else if (__HAL_UART_GET_FLAG(&nrfUart, UART_FLAG_TXE)) {
		
	// 	if (outDataIsr && (dataIndexIsr < dataSizeIsr)) {
	// 		DEBUG_PRINT("tx\n");
  //     HAL_UART_Transmit(&nrfUart, &outDataIsr[dataIndexIsr], 1, HAL_TIMEOUT);
  //     dataIndexIsr++;
  //   } else {

  //     __HAL_UART_DISABLE_IT(&nrfUart, UART_IT_TXE);
  //   	osSemaphoreRelease(nrfUartWaitS);
  //   }
	// }
}
