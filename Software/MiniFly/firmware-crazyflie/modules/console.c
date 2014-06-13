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

#include <rtthread.h>

#include "crtp.h"

CRTPPacket messageToPrint;
rt_mutex_t synch = RT_NULL;

static rt_bool_t isInit;

/**
 * Send the data to the client
 */
static void consoleSendMessage(void)
{
  crtpSendPacketBlock(&messageToPrint);
  messageToPrint.size = 0;
}

void consoleInit(void)
{
  if (isInit)
    return;

  messageToPrint.size = 0;
  messageToPrint.header = CRTP_HEADER(CRTP_PORT_CONSOLE, 0);
  //vSemaphoreCreateBinary(synch);
  synch = rt_mutex_create("cs_lock", RT_IPC_FLAG_FIFO);
  isInit = RT_TRUE;
}

rt_bool_t consoleTest(void)
{
  return isInit;
}

int consolePutchar(int ch)
{
  if (rt_mutex_take(synch,RT_WAITING_FOREVER) == RT_EOK)
  {
    messageToPrint.data[messageToPrint.size] = (unsigned char)ch;
    messageToPrint.size++;
    if (ch == '\n' || messageToPrint.size == CRTP_MAX_DATA_SIZE)
    {
      consoleSendMessage();
    }
    rt_mutex_release(synch);
  }
  
  return (unsigned char)ch;
}

int consolePuts(char *str)
{
  int ret = 0;
  
  while(*str)
    ret |= consolePutchar(*str++);
  
  return ret;
}

void consoleFlush(void)
{
	if (rt_mutex_take(synch, RT_WAITING_FOREVER) == RT_EOK)
  {
    consoleSendMessage();
	rt_mutex_release(synch);
  }
}
