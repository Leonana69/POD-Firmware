#include <stdbool.h>
#include <string.h>

#include "config.h"
#include "usblink.h"
#include "crtp.h"
#include "static_mem.h"
#include "cfassert.h"
#include "debug.h"

#include "usbd_cdc_if.h"

static bool isInit = false;
STATIC_MEM_QUEUE_ALLOC(crtpPacketDeliveryUsb, 3, sizeof(CRTPPacket));
static uint8_t sendBuffer[64];

static int usblinkSendPacket(CRTPPacket *p);
static int usblinkSetEnable(bool enable);
static int usblinkReceivePacket(CRTPPacket *p);

static struct crtpLinkOperations usblinkOp = {
	.setEnable		= usblinkSetEnable,
	.sendPacket		= usblinkSendPacket,
	.receivePacket  = usblinkReceivePacket,
};

void usblinkMessagePut(CRTPPacket *p) {
	osMessageQueuePut(crtpPacketDeliveryUsb, p, 0, 0);
}

static int usblinkReceivePacket(CRTPPacket *p) {
	if (osMessageQueueGet(crtpPacketDeliveryUsb, p, NULL, CRTP_LINK_RECEIVE_TIMEOUT) == osOK)
		return 0;
	return -1;
}

static int usblinkSendPacket(CRTPPacket *p) {
	int dataSize;

	ASSERT(p->size < 32);

	sendBuffer[0] = p->header;

	if (p->size <= CRTP_MAX_DATA_SIZE) {
		memcpy(&sendBuffer[1], p->data, p->size);
	}
	dataSize = p->size + 1;

	return CDC_Transmit_FS(sendBuffer, dataSize);
}

static int usblinkSetEnable(bool enable) {
	return 0;
}

/*
 * Public functions
 */
void usblinkInit() {
	if (isInit)
		return;

	STATIC_MEM_QUEUE_CREATE(crtpPacketDeliveryUsb);

	isInit = true;
}

bool usblinkTest() {
	return isInit;
}

struct crtpLinkOperations *usblinkGetLink() {
	return &usblinkOp;
}
