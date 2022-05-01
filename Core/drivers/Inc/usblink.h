#ifndef __USBLINK_H__
#define __USBLINK_H__

#include <stdbool.h>
#include "crtp.h"

void usblinkInit();
bool usblinkTest();
void usblinkMessagePut(CRTPPacket *p);
struct crtpLinkOperations *usblinkGetLink();

#endif //__USBLINK_H__
