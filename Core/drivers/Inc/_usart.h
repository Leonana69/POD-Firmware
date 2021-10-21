#ifndef __UUSART_H__
#define __UUSART_H__

/*
 * USART
 */
#include <stdbool.h>
#include "usart.h"
#include "syslink.h"

void _UART_Init(void);

void nrfUartTxenFlowCtrlIsr();
void nrfUartRxDmaIsr();
void nrfUartSendDataDmaBlocking(uint32_t size, uint8_t *data);
void nrfUartGetPacketBlocking(SyslinkPacket* packet);
void nrfUartRxIsr();

int debugUartPutchar(int c);

#endif