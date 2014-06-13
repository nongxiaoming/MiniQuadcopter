/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * radiolink.c: nRF24L01 implementation of the CRTP link
 */

#include <errno.h>
#include <rtthread.h>
#include "board.h"

#include "nrf24l01.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"


static rt_bool_t isInit;

#define RADIO_CONNECTED_TIMEOUT   M2T(2000)

/* Synchronisation */
rt_event_t dataRdy;
/* Data queue */
rt_mq_t txQueue;
rt_mq_t rxQueue;

static rt_uint32_t lastPacketTick;

//Union used to efficiently handle the packets (Private type)
typedef union
{
  CRTPPacket crtp;
  struct {
    uint8_t size;
    uint8_t data[32];
  } __attribute__((packed)) raw;
} RadioPacket;

static struct {
  rt_bool_t enabled;
} state;

static void interruptCallback()
{
  //portBASE_TYPE  xHigherPriorityTaskWoken = pdRT_FALSE;

  //To unlock RadioTask
  rt_event_send(dataRdy, 0x01);

 // if(xHigherPriorityTaskWoken)
  //  vPortYieldFromISR();
}

// 'Class' functions, called from callbacks
static int setEnable(rt_bool_t enable)
{
  nrfSetEnable(enable);
  state.enabled = enable;

  return 0;
}

static int sendPacket(CRTPPacket * pk)
{
  if (!state.enabled)
    return ENETDOWN;
  //xQueueSend( txQueue, pk, portMAX_DELAY);
  rt_mq_send(txQueue,pk, sizeof(CRTPPacket));
  return 0;
}

static int receivePacket(CRTPPacket * pk)
{
  if (!state.enabled)
    return ENETDOWN;

  rt_mq_recv(rxQueue, pk, sizeof(CRTPPacket), RT_WAITING_FOREVER);
  
  return 0;
}

static int reset(void)
{
  //xQueueReset(txQueue);
  rt_mq_detach(txQueue);
  nrfFlushTx();

  return 0;
}

static rt_bool_t isConnected(void)
{
  if ((rt_tick_get() - lastPacketTick) > RADIO_CONNECTED_TIMEOUT)
    return RT_FALSE;

  return RT_TRUE;
}

static struct crtpLinkOperations radioOp =
{
  .setEnable         = setEnable,
  .sendPacket        = sendPacket,
  .receivePacket     = receivePacket,
  .isConnected       = isConnected,
  .reset             = reset,
};

/* Radio task handles the CRTP packet transfers as well as the radio link
 * specific communications (eg. Scann and ID ports, communication error handling
 * and so much other cool things that I don't have time for it ...)
 */
static void radiolinkTask(void * arg)
{
	rt_uint32_t e = 0;
  unsigned char dataLen;
  static RadioPacket pk;

  //Packets handling loop
  while(1)
  {
    ledseqRun(LED_GREEN, seq_linkup);

    rt_event_recv(dataRdy,0x01,RT_EVENT_FLAG_AND|RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER,&e);
    lastPacketTick = rt_tick_get();
    
    nrfSetEnable(RT_FALSE);
    
    //Fetch all the data (Loop until the RX Fifo is NOT empty)
    while( !(nrfRead1Reg(REG_FIFO_STATUS)&0x01) )
    {
      dataLen = nrfRxLength(0);

      if (dataLen>32)          //If a packet has a wrong size it is dropped
        nrfFlushRx();
      else                     //Else, it is processed
      {
        //Fetch the data
        pk.raw.size = dataLen-1;
        nrfReadRX((char *)pk.raw.data, dataLen);

        //Push it in the queue (If overflow, the packet is dropped)
        if (!CRTP_IS_NULL_PACKET(pk.crtp))  //Don't follow the NULL packets
			rt_mq_send(rxQueue, &pk, sizeof(RadioPacket));
      }
    }

    //Push the data to send (Loop until the TX Fifo is full or there is no more data to send)
	while( (rt_mq_recv(txQueue, &pk, sizeof(RadioPacket), 5)==RT_EOK) && !(nrfRead1Reg(REG_FIFO_STATUS)&0x20) )
    {
      //xQueueReceive(txQueue, &pk, 0);
		//rt_mq_recv(txQueue, &pk, sizeof(RadioPacket), RT_WAITING_FOREVER);
      pk.raw.size++;

      nrfWriteAck(0, (char*) pk.raw.data, pk.raw.size);
    }

    //clear the interruptions flags
    nrfWrite1Reg(REG_STATUS, 0x70);
    
    //Re-enable the radio
    nrfSetEnable(RT_TRUE);
  }
}

static void radiolinkInitNRF24L01P(void)
{
  int i;
  char radioAddress[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

  //Set the radio channel
  nrfSetChannel(configblockGetRadioChannel());
  //Set the radio data rate
  nrfSetDatarate(configblockGetRadioSpeed());
  //Set radio address
  nrfSetAddress(0, radioAddress);

  //Power the radio, Enable the DS interruption, set the radio in PRX mode
  nrfWrite1Reg(REG_CONFIG, 0x3F);
  rt_thread_delay(M2T(2)); //Wait for the chip to be ready
  // Enable the dynamic payload size and the ack payload for the pipe 0
  nrfWrite1Reg(REG_FEATURE, 0x06);
  nrfWrite1Reg(REG_DYNPD, 0x01);

  //Flush RX
  for(i=0;i<3;i++)
    nrfFlushRx();
  //Flush TX
  for(i=0;i<3;i++)
    nrfFlushTx();
}

/*
 * Public functions
 */

void radiolinkInit(void)
{
	rt_thread_t radio_thread;
  if(isInit)
    return;

  nrfInit();

  nrfSetInterruptCallback(interruptCallback);

  //vTaskSetApplicationTaskTag(0, (void*)TASK_RADIO_ID_NBR);

  /* Initialise the semaphores */
  //vSemaphoreCreateBinary(dataRdy);
  dataRdy = rt_event_create("dataRdy", RT_IPC_FLAG_FIFO);
  /* Queue init */
  //rxQueue = xQueueCreate(3, sizeof(RadioPacket));
  rxQueue = rt_mq_create("rx_mq", sizeof(RadioPacket), 3, RT_IPC_FLAG_FIFO);
  //txQueue = xQueueCreate(3, sizeof(RadioPacket));
  txQueue = rt_mq_create("tx_mq", sizeof(RadioPacket), 3, RT_IPC_FLAG_FIFO);

  radiolinkInitNRF24L01P();

    /* Launch the Radio link task */
 // xTaskCreate(radiolinkTask, (const signed char * const)"RadioLink",
  //            configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);
  radio_thread = rt_thread_create("radiolink", radiolinkTask, RT_NULL, 512, 10, 5);
  if (radio_thread != RT_NULL)
	  rt_thread_startup(radio_thread);

  isInit = RT_TRUE;
}

rt_bool_t radiolinkTest(void)
{
  return nrfTest();
}

struct crtpLinkOperations * radiolinkGetLink()
{
  return &radioOp;
}

void radiolinkReInit(void)
{
  if (!isInit)
    return;

  radiolinkInitNRF24L01P();
}
