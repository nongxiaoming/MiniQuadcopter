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
 * eskylink.c: esky 2.4GHz-compatible link driver
 */
/*
 * Experimental code!
 * This link implements the ESky remote protocol using the nRF24L01p chip and
 * sends CRTP packets to the commander.
 *
 * Thanks to 'dvdouden' for documenting the protocol!
 *  -> http://www.deviationtx.com/forum/protocol-development/1059-esky-protocol?q=/forum/protocol-development/1059-esky-protocol
 *  -> http://sourceforge.net/p/arduinorclib/wiki/Esky%20Radio/
 */

#include <errno.h>
#include <string.h>
#include <rtthread.h>

#include "nrf24l01.h"
#include "crtp.h"
#include "configblock.h"
#include "ledseq.h"


/* FIXME: This might be a bit tight range? */
#define PPM_ZERO 1500
#define PPM_RANGE 500
#define PPM_MIN 1000
#define PPM_MAX 2000

static bool isInit;

static char address[4] = {0x00, 0x00, 0x00, 0xBB};
static char packet[32];

/* Synchronisation */
rt_event_t dataRdy;
/* Data queue */
rt_mq_t rxQueue;

static struct {
  bool enabled;
  
  bool paired;
  uint8_t band;
} state;

static void interruptCallback()
{
  //portBASE_TYPE  xHigherPriorityTaskWoken = pdRT_FALSE;

  //To unlock RadioTask
  //xSemaphoreGiveFromISR(dataRdy, &xHigherPriorityTaskWoken);
	rt_event_send(dataRdy, 0x01);
  //if(xHigherPriorityTaskWoken)
   // vPortYieldFromISR();
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
  
  // NOP!
  
  return 0;
}

static int receivePacket(CRTPPacket * pk)
{
  if (!state.enabled)
    return ENETDOWN;

 // xQueueReceive( rxQueue, pk, portMAX_DELAY);
  rt_mq_recv(rxQueue, pk, sizeof(CRTPPacket), RT_WAITING_FOREVER);
  return 0;
}

static struct crtpLinkOperations eskyOp =
{
  .setEnable         = setEnable,
  .sendPacket        = sendPacket,
  .receivePacket     = receivePacket,
};

static int eskylinkFetchData(char * packet, int dataLen)
{
  nrfSetEnable(RT_FALSE);

  //Fetch the data
  nrfReadRX(packet, dataLen);

  //clear the interruptions flags
  nrfWrite1Reg(REG_STATUS, 0x70);
  
  nrfSetEnable(RT_TRUE);
  
  return dataLen;
}

static void eskylinkInitPairing(void)
{
  int i;
  
  //Power the radio, Enable the DR interruption, set the radio in PRX mode with 2bytes CRC
  nrfWrite1Reg(REG_CONFIG, 0x3F);
  rt_thread_delay(M2T(2)); //Wait for the chip to be ready
  
   //Set the radio channel, pairing channel is 50
  nrfSetChannel(50);
  //Set the radio data rate
  nrfSetDatarate(RADIO_RATE_1M);

  nrfWrite1Reg(REG_SETUP_AW, VAL_SETUP_AW_3B); // 3 bytes address
  address[0] = address[1] = address[2] = 0;
  nrfWriteReg(REG_RX_ADDR_P0, address, 3);     // Pipe address == 0
  nrfWrite1Reg(REG_EN_RXADDR, 0x01);
  nrfWrite1Reg(REG_FEATURE, 0x00);             // No dynamic size payload
  nrfWrite1Reg(REG_DYNPD, 0x00);
  nrfWrite1Reg(REG_RX_PW_P0, 13);              //13 bytes payload
  nrfWrite1Reg(REG_EN_AA, 0);                  //Disable shockburst

  //Flush RX
  for(i=0;i<3;i++)
    nrfFlushRx();
  //Flush TX
  for(i=0;i<3;i++)
    nrfFlushTx();
}

static void eskylinkInitPaired(int channel)
{
  nrfSetChannel(channel);
  nrfSetDatarate(RADIO_RATE_1M);

  nrfWrite1Reg(REG_SETUP_AW, VAL_SETUP_AW_4B); // 4 bytes address
  nrfWriteReg(REG_RX_ADDR_P0, address, 4);     // Pipe address == from pairing packet
  nrfWrite1Reg(REG_EN_RXADDR, 0x01);
  nrfWrite1Reg(REG_FEATURE, 0x00);             // No dynamic size payload
  nrfWrite1Reg(REG_DYNPD, 0x00);
  nrfWrite1Reg(REG_RX_PW_P0, 13);              //13 bytes payload
  nrfWrite1Reg(REG_EN_AA, 0);                  //Disable shockburst
}

//FIXME: A lot of parameters shall be configurable
static void eskylinkDecode(char* packet)
{
  static CRTPPacket crtpPacket;
  float pitch, roll, yaw;
  uint16_t thrust;
  
  pitch = ((packet[2]<<8) | packet[3])-PPM_ZERO;
  if (roll<(-PPM_RANGE)) roll = -PPM_RANGE;
  if (roll>PPM_RANGE) roll = PPM_RANGE;
  pitch *= 20.0/PPM_RANGE;
  
  roll = ((packet[0]<<8) | packet[1])-PPM_ZERO;
  if (roll<(-PPM_RANGE)) roll = -PPM_RANGE;
  if (roll>PPM_RANGE) roll = PPM_RANGE;
  roll *= 20.0/PPM_RANGE;
  
  yaw = ((packet[6]<<8) | packet[7])-PPM_ZERO;
  if (yaw<(-PPM_RANGE)) yaw = -PPM_RANGE;
  if (yaw>PPM_RANGE) yaw = PPM_RANGE;
  yaw *= 200.0/PPM_RANGE;
  
  thrust = ((packet[4]<<8) | packet[5])-PPM_MIN;
  if (thrust<0) thrust = 0;
  if (thrust>(2*PPM_RANGE)) thrust = 2*PPM_RANGE;
  thrust *= 55000/(2*PPM_RANGE);
  
  crtpPacket.port = CRTP_PORT_COMMANDER;
  memcpy(&crtpPacket.data[0],  (char*)&roll,   4);
  memcpy(&crtpPacket.data[4],  (char*)&pitch,  4);
  memcpy(&crtpPacket.data[8],  (char*)&yaw,    4);
  memcpy(&crtpPacket.data[12], (char*)&thrust, 2);
  
 // xQueueSend(rxQueue, &crtpPacket, 0);
  rt_mq_send(rxQueue, &crtpPacket, sizeof(CRTPPacket));
}

static void eskylinkTask(void * arg)
{
  int channel = 7;
  int channel1 = -1; //As long as channel1<0 the copter is in scann mode
  int channel2 = 0;
  rt_uint32_t e;
  //Waiting for pairing packet
  while (!state.paired)
  {
    //xSemaphoreTake(dataRdy, portMAX_DELAY);
	  rt_event_recv(dataRdy, 0x01, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &e);
    ledseqRun(LED_GREEN, seq_linkup);
    
    eskylinkFetchData(packet, 13);

    if (packet[4]==0x18 && packet[5]==0x29)
    {
      address[2]=packet[0];
      address[1]=packet[1];
      address[0]=packet[2];
      state.band = packet[3];
      state.paired = RT_TRUE;
    }
  }

  ledseqRun(LED_GREEN, seq_testPassed);

  nrfSetEnable(RT_FALSE);
  eskylinkInitPaired(channel);
  nrfSetEnable(RT_TRUE);

  //Paired! handling packets.
  while(1)
  {
	  if (rt_event_recv(dataRdy, 0x01, RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, M2T(10),&e) == RT_EOK)
    {
      ledseqRun(LED_GREEN, seq_linkup);
    
      eskylinkFetchData(packet, 13);
      eskylinkDecode(packet);
      
      if (channel1<0) //Channels found!
      {
        channel1 = channel;
        channel2 = channel1+37;
        if (channel2>83) channel2 = channel1 - 37;
      }
    }
    else
    { 
      if (channel1<0)
      {
        channel++;
        if(channel>83) channel=7;
        nrfSetEnable(RT_FALSE);
        nrfSetChannel(channel);
        nrfSetEnable(RT_FALSE);
      }
      else
      {
        if (channel == channel1)
          channel = channel2;
        else
          channel = channel1;
        
        nrfSetEnable(RT_FALSE);
        nrfSetChannel(channel);
        nrfSetEnable(RT_TRUE);
      }
      
    }
  }
}

/*
 * Public functions
 */

void eskylinkInit()
{
	rt_thread_t eskylink_thread;
  if(isInit)
    return;

  nrfInit();

  nrfSetInterruptCallback(interruptCallback);

  //vTaskSetApplicationTaskTag(0, (void*)TASK_RADIO_ID_NBR);

  /* Initialise the semaphores */
  //vSemaphoreCreateBinary(dataRdy);
  dataRdy=rt_event_create("data_rdy", RT_IPC_FLAG_FIFO);
  /* Queue init */
  rxQueue = rt_mq_create("rx_mq",sizeof(CRTPPacket),3,RT_IPC_FLAG_FIFO);

  eskylinkInitPairing();

    /* Launch the Radio link task */
  ////xTaskCreate(eskylinkTask, (const signed char * const)"EskyLink",
  ////            configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);
  eskylink_thread = rt_thread_create("eskylink", eskylinkTask, RT_NULL, 512, 12, 5);
  if (eskylink_thread != RT_NULL)
  {
	  rt_thread_startup(eskylink_thread);
  }
  isInit = RT_TRUE;
}

rt_bool_t eskylinkTest()
{
  return nrfTest();
}

struct crtpLinkOperations * eskylinkGetLink()
{
  return &eskyOp;
}

//FIXME: To implement!
void eskylinkReInit(void)
{
  ;
}
