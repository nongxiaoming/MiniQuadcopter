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

#include <errno.h>

#include <rtthread.h>

#include "uart.h"

#include "config.h"

#include "crtp.h"
#include "info.h"
#include "cfassert.h"

static rt_bool_t isInit;

static int nopFunc(void);
static struct crtpLinkOperations nopLink = {
  .setEnable         = (void*) nopFunc,
  .sendPacket        = (void*) nopFunc,
  .receivePacket     = (void*) nopFunc,
}; 

static struct crtpLinkOperations *link = &nopLink;

static rt_mq_t  tmpQueue;

static rt_mq_t  rxQueue;

#define CRTP_NBR_OF_PORTS 16
#define CRTP_TX_QUEUE_SIZE 20
#define CRTP_RX_QUEUE_SIZE 2

static void crtpTxTask(void *param);
static void crtpRxTask(void *param);

static rt_mq_t queues[CRTP_NBR_OF_PORTS];
static volatile CrtpCallback callbacks[CRTP_NBR_OF_PORTS];

void crtpInit(void)
{
  if(isInit)
    return;

  //tmpQueue = xQueueCreate(CRTP_TX_QUEUE_SIZE, sizeof(CRTPPacket));
  tmpQueue = rt_mq_create("crtp_tmp", sizeof(CRTPPacket), CRTP_TX_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
  //rxQueue = xQueueCreate(CRTP_RX_QUEUE_SIZE, sizeof(CRTPPacket));
  rxQueue = rt_mq_create("crtp_rx", sizeof(CRTPPacket), CRTP_RX_QUEUE_SIZE, RT_IPC_FLAG_FIFO);
  /* Start Rx/Tx tasks */
  xTaskCreate(crtpTxTask, (const signed char * const)"CRTP-Tx",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  xTaskCreate(crtpRxTask, (const signed char * const)"CRTP-Rx",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  
  isInit = RT_TRUE;
}

rt_bool_t crtpTest(void)
{
  return isInit;
}

void crtpInitTaskQueue(CRTPPort portId)
{
  RT_ASSERT(queues[portId] == RT_NULL);
  
  queues[portId] = xQueueCreate(1, sizeof(CRTPPacket));
}

int crtpReceivePacket(CRTPPort portId, CRTPPacket *p)
{
  RT_ASSERT(queues[portId]);
  RT_ASSERT(p);
    
  return xQueueReceive(queues[portId], p, 0);
}

int crtpReceivePacketBlock(CRTPPort portId, CRTPPacket *p)
{
  RT_ASSERT(queues[portId]);
  RT_ASSERT(p);
  
  return xQueueReceive(queues[portId], p, portMAX_DELAY);
}


int crtpReceivePacketWait(CRTPPort portId, CRTPPacket *p, int wait) {
  RT_ASSERT(queues[portId]);
  RT_ASSERT(p);
  
  return xQueueReceive(queues[portId], p, M2T(wait));
}

void crtpTxTask(void *param)
{
  CRTPPacket p;

  while (RT_TRUE)
  {
    if (xQueueReceive(tmpQueue, &p, portMAX_DELAY) == pdRT_TRUE)
    {
      link->sendPacket(&p);
    }
  }
}

void crtpRxTask(void *param)
{
  CRTPPacket p;
  static unsigned int droppedPacket=0;

  while (RT_TRUE)
  {
    if (!link->receivePacket(&p))
    {
      if(queues[p.port])
      {
        // TODO: If full, remove one packet and then send
        xQueueSend(queues[p.port], &p, 0);
      } else {
        droppedPacket++;
      }
      
      if(callbacks[p.port])
        callbacks[p.port](&p);  //Dangerous?
    }
  }
}

void crtpRegisterPortCB(int port, CrtpCallback cb)
{
  if (port>CRTP_NBR_OF_PORTS)
    return;
  
  callbacks[port] = cb;
}

int crtpSendPacket(CRTPPacket *p)
{
  RT_ASSERT(p); 

  return xQueueSend(tmpQueue, p, 0);
}

int crtpSendPacketBlock(CRTPPacket *p)
{
  RT_ASSERT(p); 

  return xQueueSend(tmpQueue, p, portMAX_DELAY);
}

int crtpReset(void)
{
  xQueueReset(tmpQueue);
  if (link->reset) {
    link->reset();
  }

  return 0;
}

rt_bool_t crtpIsConnected(void)
{
  if (link->isConnected)
    return link->isConnected();
  return RT_TRUE;
}

void crtpPacketReveived(CRTPPacket *p)
{
  portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdRT_FALSE;
  xQueueSendFromISR(rxQueue, p, &xHigherPriorityTaskWoken);
}

void crtpSetLink(struct crtpLinkOperations * lk)
{
  if(link)
    link->setEnable(RT_FALSE);

  if (lk)
    link = lk;
  else
    link = &nopLink;

  link->setEnable(RT_TRUE);
}

static int nopFunc(void)
{
  return ENETDOWN;
}
