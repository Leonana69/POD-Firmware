/*
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
 * comm.c - High level communication module
 */

#include "config.h"

#include "crtp.h"
#include "console.h"
#include "crtp_link.h"
#include "crtp_platform.h"
#include "param.h"
#include "log.h"
#include "_usart.h"
#include "radiolink.h"

#include "syslink.h"
// #include "crtp_localization_service.h"

#include "debug.h"

static bool isInit;

void commInit(void) {
  if (isInit)
    return;

	crtpInit();
  consoleInit();
  radiolinkInit();
  // usblinkInit();
  // TODO: add choice between usb and radio
  crtpSetLink(radiolinkGetLink());

  crtpLinkInit();
  crtpPlatformInit();
  logInit();
  paramInit();

  // locSrvInit();
  isInit = true;
}

bool commTest(void) {
  bool pass = isInit;
  
  pass &= crtpTest();
  pass &= consoleTest();
  pass &= radiolinkTest();
  // pass &= usblinkTest();
  pass &= crtpLinkTest();
  pass &= crtpPlatformTest();
  pass &= logTest();
  pass &= paramTest();
  return pass;
}

