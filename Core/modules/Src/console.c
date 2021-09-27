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
 * console.c - Used to send console data to client
 */

#include <string.h>

/*FreeRtos includes*/
#include "cmsis_os2.h"
#include "console.h"
#include "crtp.h"
#include "debug.h"
#include "config.h"

#ifdef STM32F40_41xxx
#include "stm32f4xx.h"
#else
#include "stm32f10x.h"
#ifndef SCB_ICSR_VECTACTIVE_Msk
#define SCB_ICSR_VECTACTIVE_Msk 0x1FFUL
#endif
#endif

static CRTPPacket messageToPrint;
static bool messageSendingIsPending = false;
static osSemaphoreId_t synch = NULL;

static const char bufferFullMsg[] = "<F>\n";
static bool isInit;

static void addBufferFullMarker();

/**
 * Send the data to the client
 * returns TRUE if successful otherwise FALSE
 */
static bool consoleSendMessage(void) {
  if (crtpSendPacket(&messageToPrint) == osOK) {
    messageToPrint.size = 0;
    messageSendingIsPending = false;
    return true;
  }

  return false;
}

void consoleInit() {
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  synch = osSemaphoreNew(1, 1, NULL);
  messageSendingIsPending = false;

  isInit = true;
}

bool consoleTest(void) {
  return isInit;
}

int consolePutchar(int ch) {
	if (!isInit)
    return 0;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt)
    return consolePutcharFromISR(ch);
  if (osSemaphoreAcquire(synch, osDelayMax) == osOK) {
    // Try to send if we already have a pending message
    if (messageSendingIsPending)
      consoleSendMessage();

    if (!messageSendingIsPending) {
      if (messageToPrint.size < CRTP_MAX_DATA_SIZE) {
        messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
        messageToPrint.size++;
      }

      if (ch == '\n' || messageToPrint.size >= CRTP_MAX_DATA_SIZE) {
        if (crtpGetFreeTxQueuePackets() == 1)
          addBufferFullMarker();
        messageSendingIsPending = true;
        consoleSendMessage();
      }
    }
    osSemaphoreRelease(synch);
  }
  return (unsigned char)ch;
}

int consolePutcharFromISR(int ch) {
	// if called from ISR, timout should be 0
  if (osSemaphoreAcquire(synch, 0) == osOK) {
    if (messageToPrint.size < CRTP_MAX_DATA_SIZE) {
      messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
      messageToPrint.size++;
    }
    osSemaphoreRelease(synch);
  }

  return ch;
}

int consolePuts(char *str) {
  int ret = 0;

  while (*str)
    ret |= consolePutchar(*str++);

  return ret;
}

void consoleFlush(void) {
  if (osSemaphoreAcquire(synch, osDelayMax) == osOK) {
    consoleSendMessage();
    osSemaphoreRelease(synch);
  }
}


static int findMarkerStart() {
  int start = messageToPrint.size;
  // If last char is new line, rewind one char since the marker contains a new line.
  if (start > 0 && messageToPrint.data[start - 1] == '\n')
    start -= 1;

  return start;
}

static void addBufferFullMarker() {
  // Try to add the marker after the message if it fits in the buffer, otherwise overwrite the end of the message 
  int endMarker = findMarkerStart() + sizeof(bufferFullMsg);
  if (endMarker >= (CRTP_MAX_DATA_SIZE)) 
    endMarker = CRTP_MAX_DATA_SIZE;

  int startMarker = endMarker - sizeof(bufferFullMsg);
  memcpy(&messageToPrint.data[startMarker], bufferFullMsg, sizeof(bufferFullMsg));
  messageToPrint.size = startMarker + sizeof(bufferFullMsg);
}
