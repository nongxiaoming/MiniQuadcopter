/*
 * File      : stm32f20x_40x_spi.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-09-01    xiaonong       first implementation.
 */

#include "drv_nrf24l01.h"


/* Defines for the SPI and GPIO pins used to drive the SPI Flash */
#define RADIO_GPIO_CS             GPIO_Pin_6
#define RADIO_GPIO_CS_PORT        GPIOB
#define RADIO_GPIO_CS_PERIF       RCC_APB2Periph_GPIOB


#define RADIO_GPIO_CE             GPIO_Pin_8
#define RADIO_GPIO_CE_PORT        GPIOB
#define RADIO_GPIO_CE_PERIF       RCC_APB2Periph_GPIOB

#define RADIO_GPIO_IRQ            GPIO_Pin_15
#define RADIO_GPIO_IRQ_PORT       GPIOA
#define RADIO_GPIO_IRQ_PERIF      RCC_APB2Periph_GPIOA
#define RADIO_GPIO_IRQ_SRC_PORT   GPIO_PortSourceGPIOA
#define RADIO_GPIO_IRQ_SRC        GPIO_PinSource15
#define RADIO_GPIO_IRQ_LINE       EXTI_Line15
#define RADIO_GPIO_IRQ_CHANNEL    EXTI15_10_IRQn

#define RADIO_SPI                 SPI1
#define RADIO_SPI_CLK             RCC_APB2Periph_SPI1
#define RADIO_GPIO_SPI_PORT       GPIOB
#define RADIO_GPIO_SPI_CLK        RCC_APB2Periph_GPIOB
#define RADIO_GPIO_SPI_SCK        GPIO_Pin_3
#define RADIO_GPIO_SPI_MISO       GPIO_Pin_4
#define RADIO_GPIO_SPI_MOSI       GPIO_Pin_5

#define RADIO_EN_CS()  GPIO_ResetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CS() GPIO_SetBits(RADIO_GPIO_CS_PORT, RADIO_GPIO_CS)
#define RADIO_DIS_CE() GPIO_ResetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)
#define RADIO_EN_CE()  GPIO_SetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE)

#define CE_PULSE() { RADIO_EN_CE(); rt_thread_delay(1); RADIO_DIS_CE();}

static  struct {
  char dataRate;
  char power;
  char arc;
  char ard;
  char contCarrier;
} radioConf = {
  /*.dataRate =*/ DATA_RATE_2M,
  /*.power =*/ RADIO_POWER_0dBm,
  /*.arc =*/ 3,
  /*.ard =*/ ARD_PLOAD | 32,
  /*.contCarrier =*/ 0,
};
//Ack payload length by ARD step (steps of 250uS) (From nrf24l01 doc p.34)
static const unsigned char ardStep[3][6] = { {0, 0, 8, 16, 24, 32}, //250Kps
                                              {15, 32},              //1Mps
                                              {5, 32},               //2Mps
                                            };

static const unsigned char setupDataRate[] = {0x20, 0x00, 0x08};

static char SPI_SendByte(char byte)
{

  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(RADIO_SPI, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(RADIO_SPI);
}

static char SPI_ReceiveByte(void)
{
  return SPI_SendByte(DUMMY_BYTE);
}

//Nop command, permit to get the status byte
char NRF24L01_Nop()
{
  char status;
  
  RADIO_EN_CS();
  status = SPI_SendByte(CMD_NOP);
  RADIO_DIS_CS();
  
  return status;
}

char NRF24L01_FlushTx()
{
  char status;
  
  RADIO_EN_CS();
  status = SPI_SendByte(CMD_FLUSH_TX);
  RADIO_DIS_CS();
  
  return status;
}

char NRF24L01_FlushRx()
{
  char status;
  
  RADIO_EN_CS();
  status = SPI_SendByte(CMD_FLUSH_RX);
  RADIO_DIS_CS();
  
  return status;
}

char NRF24L01_ReadReg(char addr)
{
  char value;
  
  RADIO_EN_CS();
  SPI_SendByte(CMD_R_REG | (addr&0x1F));
  value = SPI_ReceiveByte();
  RADIO_DIS_CS();
  
  return value;
}

char NRF24L01_WriteReg(char addr, char value)
{
  char status;
  
  RADIO_EN_CS();
  status = SPI_SendByte(CMD_W_REG | (addr&0x1F));
  SPI_SendByte(value);
  RADIO_DIS_CS();
  
  return value;
}

// Send a packet.
void NRF24L01_TxPacket(char *payload, char len)
{
  int i;

  //Send the packet in the TX buffer
  RADIO_EN_CS();
  SPI_SendByte(CMD_W_TX_PAYLOAD);
  for(i=0;i<len;i++)
    SPI_SendByte(payload[i]);
  RADIO_DIS_CS();
  
  //Pulse CE
  CE_PULSE();
  
  return;
}

//Send a packed in no-ack mode
void NRF24L01_TxPacketNoAck(char *payload, char len)
{
  int i;

  //Send the packet in the TX buffer
  RADIO_EN_CS();
  SPI_SendByte(CMD_W_PAYLOAD_NO_ACK);
  for(i=0;i<len;i++)
    SPI_SendByte(payload[i]);
  RADIO_DIS_CS();
  
  //Pulse CE
  CE_PULSE();
  
  return;
}

//Fetch the next act payload
//Return the payload length
char NRF24L01_RxPacket(char *payload)
{
  int len;
  int i;

  //Get the packet length
  RADIO_EN_CS();
  SPI_SendByte(CMD_RX_PL_WID);
  len = SPI_ReceiveByte();
  RADIO_DIS_CS();  
  
  if (len>0 && len<33)
  {
    //Read the packet from the RX buffer
    RADIO_EN_CS();
    SPI_SendByte(CMD_R_RX_PAYLOAD);
    for(i=0;i<len;i++)
      payload[i] = SPI_ReceiveByte();
    RADIO_DIS_CS();
  } else {
    len=0;
  }
  
  //Pulse CE
  //CE_PULSE();
  
  return len;
}

//Send a packet and receive the ACK
//Return true in case of success.
//Polling implementation
unsigned char NRF24L01_SendPacket(char *payload, char len, char *ackPayload, char *ackLen)
{
  char status = 0;
  
  //Send the packet
  NRF24L01_TxPacket(payload, len);
  //Wait for something to happen
  while(((status=NRF24L01_Nop())&0x70) == 0);
  
  // Clear the flags
  NRF24L01_WriteReg(REG_STATUS, 0x70);
  
  //Return FALSE if the packet has not been transmited
  if (status&BIT_MAX_RT) {
    NRF24L01_FlushTx();
    return 0;
  }
    
  //Receive the ackPayload if any has been received
  if (status&BIT_RX_DR)
    *ackLen = NRF24L01_RxPacket(ackPayload);
  else 
    *ackLen = 0;
  
  NRF24L01_FlushRx();
  
  return status&BIT_TX_DS;
}

//Send a packet and don't wait for the Acknoledge
void NRF24L01_SendPacketNoAck(char *payload, char len)
{
  //Wait for the TX fifo not to be full
  while((NRF24L01_Nop()&0x01) != 0);

  //Send the packet
  NRF24L01_TxPacketNoAck(payload, len);

  //Nothing to wait for, the packet is 'just' sent!
}

//Raw registers update (for internal use)
void NRF24L01_UpdateRetr()
{
  char ard=0;
  unsigned char nbytes;
  
  if (radioConf.ard & ARD_PLOAD)
  {
    nbytes = ((radioConf.ard&0x7F)>32)?32:(radioConf.ard&0x7F);
    for (ard=0; ardStep[radioConf.dataRate][ard]<nbytes; ard++)
      continue;
  } else
    ard = radioConf.ard & 0x0F;
  
  NRF24L01_WriteReg(REG_SETUP_RETR, (ard<<4) | (radioConf.arc&0x0F)); 
}

void NRF24L01_UpdateRfSetup()
{
  unsigned char setup=0;
  
  setup = setupDataRate[radioConf.dataRate];
  setup |= radioConf.power<<1;
  
  if (radioConf.contCarrier)
    setup |= 0x90;
  
  NRF24L01_WriteReg(REG_RF_SETUP, setup);
}

//Set the radio channel.
void NRF24L01_SetChannel(char channel)
{
  //Test the input
  if(channel<0 || channel>125)
    return;
   
  //Change the channel
  RADIO_DIS_CE();
  NRF24L01_WriteReg(REG_RF_CH, channel);
  
  //CE is continously activated if in continous carrier mode
  if(radioConf.contCarrier)
    RADIO_EN_CE();
}

//Set the radio datarate
void NRF24L01_SetDataRate(unsigned char dr)
{
  if (dr>=3)
    return;
  
  radioConf.dataRate = dr;
  
  NRF24L01_UpdateRfSetup();
  NRF24L01_UpdateRetr();
}

char NRF24L01_GetDataRate()
{
  return radioConf.dataRate;
}
void NRF24L01_SetPower(char power)
{
  radioConf.power = power&0x03;
  
  NRF24L01_UpdateRfSetup();
}

void NRF24L01_SetArd(char ard)
{
  radioConf.ard = ard;
  
  NRF24L01_UpdateRetr(); 
}

void NRF24L01_SetArc(char arc)
{
  radioConf.arc = arc;
  
  NRF24L01_UpdateRetr();
}

void NRF24L01_SetContCarrier(rt_bool_t contCarrier)
{
  radioConf.contCarrier = contCarrier?1:0;
  
  RADIO_DIS_CE();
  
  NRF24L01_UpdateRfSetup();
  
  if(contCarrier)
    RADIO_EN_CE();
}
//Set the TX and RX address
void NRF24L01_SetAddress(char* address)
{
  int i;

  RADIO_EN_CS();
  SPI_SendByte(CMD_W_REG | REG_TX_ADDR);
  for(i=0; i<5; i++)
    SPI_SendByte(address[i]);
  RADIO_DIS_CS();

  RADIO_EN_CS();
  SPI_SendByte(CMD_W_REG | REG_RX_ADDR_P0);
  for(i=0; i<5; i++)
    SPI_SendByte(address[i]);
  RADIO_DIS_CS();
}

//Get the radio power detector value
uint8_t NRF24L01_GetRpd(void)
{
    return NRF24L01_ReadReg(REG_RPD);
}

//Get the number of retry to send the last packet
uint8_t NRF24L01_GetTxRetry(void)
{
    return NRF24L01_ReadReg(REG_OBSERVE_TX)&0x0F;
}

void rt_hw_nrf24l01_init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  //if (isInit==TRUE)
   // return;

  /* Enable the EXTI interrupt router */
 // extiInit();

  /* Enable SPI and GPIO clocks */
  RCC_APB2PeriphClockCmd(RADIO_SPI_CLK | RADIO_GPIO_SPI_CLK | RADIO_GPIO_CS_PERIF | 
                         RADIO_GPIO_CE_PERIF | RADIO_GPIO_IRQ_PERIF, ENABLE);
	
  /* Disable JTAG */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
	/* Remap SPI1 */
	GPIO_PinRemapConfig(GPIO_Remap_SPI1 , ENABLE);
  
  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_SCK |  RADIO_GPIO_SPI_MOSI | RADIO_GPIO_SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RADIO_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure MISO */
  // GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_SPI_MISO;
  // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  // GPIO_Init(RADIO_GPIO_SPI_PORT, &GPIO_InitStructure);

  /* Configure I/O for the Chip select */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CS;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RADIO_GPIO_CS_PORT, &GPIO_InitStructure);

  /* Configure the interruption (EXTI Source) */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_IRQ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(RADIO_GPIO_IRQ_PORT, &GPIO_InitStructure);

  GPIO_EXTILineConfig(RADIO_GPIO_IRQ_SRC_PORT, RADIO_GPIO_IRQ_SRC);
  EXTI_InitStructure.EXTI_Line = RADIO_GPIO_IRQ_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);


  /* disable the chip select */
  RADIO_DIS_CS();

  /* Configure I/O for the Chip Enable */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CE;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RADIO_GPIO_CE_PORT, &GPIO_InitStructure);

  /* disable the chip enable */
  RADIO_DIS_CE();

  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(RADIO_SPI, &SPI_InitStructure);

  /* Enable the SPI  */
  SPI_Cmd(RADIO_SPI, ENABLE);
  
  //isInit = TRUE;
}
