/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * crtp.c - CrazyRealtimeTransferProtocol stack
 */

#include <stdbool.h>
#include <errno.h>

#include "config.h"
#include "crtp.h"
#include "cfassert.h"
#include "static_mem.h"
#include "log.h"
#include "debug.h"
#include "optimization.h"

static bool isInit;

static int nopFunc(void) { return ENETDOWN; }
static struct crtpLinkOperations nopLink = {
	.setEnable = (void*) nopFunc,
	.sendPacket = (void*) nopFunc,
	.receivePacket = (void*) nopFunc,
};

static struct crtpLinkOperations *link[CRTP_LINK_NUMBER] = { &nopLink, &nopLink };

#define STATS_INTERVAL 500
static struct {
	uint32_t rxCount;
	uint32_t txCount;

	uint16_t rxRate;
	uint16_t txRate;

	uint32_t nextStatisticsTime;
	uint32_t previousStatisticsTime;
} stats;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_RADIO_TX_QUEUE_SIZE 120
#define CRTP_RADIO_RX_QUEUE_SIZE 16
#define CRTP_USB_RX_QUEUE_SIZE 8
#define CRTP_USB_TX_QUEUE_SIZE 4

static osMessageQueueId_t txQueue;
static osMessageQueueId_t queues[CRTP_NBR_OF_PORTS];

static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];
static void updateStats();

static void crtpRadioTxTask();
static void crtpRadioRxTask();
static void crtpUsbRxTask();
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtpRadioTxTask, CRTP_RADIO_TX_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtpRadioRxTask, CRTP_RADIO_RX_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(crtpUsbRxTask, CRTP_USB_RX_TASK_STACKSIZE);

void crtpInit(void) {
	if (isInit)
		return;

	txQueue = osMessageQueueNew(CRTP_RADIO_TX_QUEUE_SIZE, sizeof(CRTPPacket), NULL);

	STATIC_MEM_TASK_CREATE(crtpRadioTxTask, crtpRadioTxTask, CRTP_RADIO_TX_TASK_NAME, NULL, CRTP_TX_TASK_PRI);
	STATIC_MEM_TASK_CREATE(crtpRadioRxTask, crtpRadioRxTask, CRTP_RADIO_RX_TASK_NAME, NULL, CRTP_RX_TASK_PRI);
	STATIC_MEM_TASK_CREATE(crtpUsbRxTask, crtpUsbRxTask, CRTP_USB_RX_TASK_NAME, NULL, CRTP_RX_TASK_PRI);

	isInit = true;
}

bool crtpTest(void) {
	return isInit;
}

void crtpInitTaskQueue(CRTPPort portId) {
	ASSERT(queues[portId] == NULL);
	queues[portId] = osMessageQueueNew(CRTP_RADIO_RX_QUEUE_SIZE, sizeof(CRTPPacket), NULL);
}

int crtpReceivePacket(CRTPPort portId, CRTPPacket *p) {
	ASSERT(queues[portId]);
	ASSERT(p);
	return osMessageQueueGet(queues[portId], p, NULL, 0);
}

int crtpReceivePacketBlock(CRTPPort portId, CRTPPacket *p) {
	ASSERT(queues[portId]);
	ASSERT(p);
	return osMessageQueueGet(queues[portId], p, NULL, osWaitForever);
}

int crtpReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait) {
	ASSERT(queues[portId]);
	ASSERT(p);
	return osMessageQueueGet(queues[portId], p, NULL, wait);
}

int crtpGetFreeTxQueuePackets(void) {
	return osMessageQueueGetSpace(txQueue);
}

void crtpRadioTxTask() {
	CRTPPacket p;

	while (link[CRTP_LINK_RADIO] == &nopLink || link[CRTP_LINK_USB] == &nopLink)
		osDelay(100);

	while (1) {
		if (osMessageQueueGet(txQueue, &p, 0, osWaitForever) == osOK) {
			/*! Keep testing, if the link[CRTP_LINK_RADIO] changes to USB it will go though */
			while (link[CRTP_LINK_RADIO]->sendPacket(&p) == false)
				osDelay(10);
			stats.txCount++;
			updateStats();
		}
	}
}

void crtpRadioRxTask() {
	CRTPPacket p;
	while (link[CRTP_LINK_RADIO] == &nopLink) osDelay(100);

	while (1) {
		if (!link[CRTP_LINK_RADIO]->receivePacket(&p)) {
			if (queues[p.port])
				/*! Block, since we should never drop a packet */
				osMessageQueuePut(queues[p.port], &p, 0, osWaitForever);

			if (callbacks[p.port])
				callbacks[p.port](&p);

			stats.rxCount++;
			updateStats();
		}
	}
}

void crtpUsbRxTask() {
	CRTPPacket p;
	while (link[CRTP_LINK_USB] == &nopLink) osDelay(100);

	while (1) {
		if (!link[CRTP_LINK_USB]->receivePacket(&p)) {
			// DEBUG_PRINT_CONSOLE("$ USB\n");
			// if (queues[p.port])
			// 	/*! Block, since we should never drop a packet */
			// 	osMessageQueuePut(queues[p.port], &p, 0, osWaitForever);
			// DEBUG_PRINT_CONSOLE("R:%d\n", p.port);

			if (callbacks[p.port])
				callbacks[p.port](&p);
		}
	}
}

void crtpRegisterPortCB(int port, CrtpCallback cb) {
	if (port > CRTP_NBR_OF_PORTS)
		return;

	callbacks[port] = cb;
}

int crtpSendPacket(CRTPPacket *p) {
	ASSERT(p);
	ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

	return osMessageQueuePut(txQueue, p, 0, 0);
}

int crtpSendPacketBlock(CRTPPacket *p) {
	ASSERT(p);
	ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

	return osMessageQueuePut(txQueue, p, 0, osWaitForever);
}

int crtpReset() {
	osMessageQueueReset(txQueue);
	for (int i = 0; i < CRTP_LINK_NUMBER; i++)
		if (link[i]->reset)
			link[i]->reset();
	return 0;
}

bool crtpIsConnected(CRTPLink type) {
	if (link[type]->isConnected)
		return link[type]->isConnected();
	return true;
}

void crtpSetLink(CRTPLink type, struct crtpLinkOperations *lk) {
	ASSERT(lk);

	link[type] = lk;
	link[type]->setEnable(true);
}

static void clearStats() {
	stats.rxCount = 0;
	stats.txCount = 0;
}

static void updateStats() {
	uint32_t now = osKernelGetTickCount();
	if (now > stats.nextStatisticsTime) {
		float interval = now - stats.previousStatisticsTime;
		stats.rxRate = (uint16_t)(1000.0f * stats.rxCount / interval);
		stats.txRate = (uint16_t)(1000.0f * stats.txCount / interval);

		clearStats();
		stats.previousStatisticsTime = now;
		stats.nextStatisticsTime = now + STATS_INTERVAL;
	}
}

LOG_GROUP_START(crtp)
LOG_ADD(LOG_UINT16, rxRate, &stats.rxRate)
LOG_ADD(LOG_UINT16, txRate, &stats.txRate)
LOG_GROUP_STOP(crtp)
