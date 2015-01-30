#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include "drv_nrf24l01.h"
#include "stm32f10x.h"

/* Registers address definition */
#define REG_CONFIG              0x00
#define REG_EN_AA               0x01
#define REG_EN_RXADDR           0x02
#define REG_SETUP_AW            0x03
#define REG_SETUP_RETR          0x04
#define REG_RF_CH               0x05
#define REG_RF_SETUP            0x06
#define REG_STATUS              0x07
#define REG_OBSERVE_TX          0x08
#define REG_RPD                 0x09
#define REG_RX_ADDR_P0          0x0A
#define REG_RX_ADDR_P1          0x0B
#define REG_RX_ADDR_P2          0x0C
#define REG_RX_ADDR_P3          0x0D
#define REG_RX_ADDR_P4          0x0E
#define REG_RX_ADDR_P5          0x0F
#define REG_TX_ADDR             0x10
#define REG_RX_PW_P0            0x11
#define REG_RX_PW_P1            0x12
#define REG_RX_PW_P2            0x13
#define REG_RX_PW_P3            0x14
#define REG_RX_PW_P4            0x15
#define REG_RX_PW_P5            0x16
#define REG_FIFO_STATUS         0x17
#define REG_DYNPD               0x1C
#define REG_FEATURE             0x1D

/* nRF24L SPI commands */
#define CMD_R_REG               0x00
#define CMD_W_REG               0x20
#define CMD_R_RX_PAYLOAD        0x61
#define CMD_W_TX_PAYLOAD        0xA0
#define CMD_FLUSH_TX            0xE1
#define CMD_FLUSH_RX            0xE2
#define CMD_REUSE_TX_PL         0xE3
#define CMD_ACTIVATE            0x50
#define CMD_RX_PL_WID           0x60
#define CMD01_W_ACK_PAYLOAD(P)  (0xA8|(P&0x0F))
#define CMD_W_PAYLOAD_NO_ACK    0xD0
#define CMD_NOP                 0xFF

#define VAL_RF_SETUP_250K 0x26
#define VAL_RF_SETUP_1M   0x06
#define VAL_RF_SETUP_2M   0x0E

#define VAL_SETUP_AW_3B 1
#define VAL_SETUP_AW_4B 2
#define VAL_SETUP_AW_5B 3

/* nRF24l01 Hardware connect
*  nRF24l01 CE   <---> PC14
*  nRF24l01 IRQ  <---> PA1
**/
#define RADIO_GPIO_CE             GPIO_Pin_14
#define RADIO_GPIO_CE_PORT        GPIOC
#define RADIO_GPIO_CE_PERIF       RCC_APB2Periph_GPIOC

#define RADIO_GPIO_IRQ            GPIO_Pin_1
#define RADIO_GPIO_IRQ_PORT       GPIOA
#define RADIO_GPIO_IRQ_PERIF      RCC_APB2Periph_GPIOA
#define RADIO_GPIO_IRQ_SRC_PORT   GPIO_PortSourceGPIOA
#define RADIO_GPIO_IRQ_SRC        GPIO_PinSource1
#define RADIO_GPIO_IRQ_LINE       EXTI_Line1
#define RADIO_GPIO_IRQ_CHANNEL    EXTI1_IRQn

typedef struct
{
  struct rt_spi_device *spi;     /* SPI device */
	nrf24l01_mode_t mode;          /* nRF24l01 work mode */
	 /* nrf enable */
  void (*enable)(rt_bool_t enable);
} nrf24l01_dev_t;

static const char Tx_Address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
static const char Rx_Address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

static void RCC_Configuration(void)
{
   /* Enable SPI and GPIO clocks */
  RCC_APB2PeriphClockCmd(RADIO_GPIO_CE_PERIF | RADIO_GPIO_IRQ_PERIF, ENABLE);
   
  PWR_BackupAccessCmd(ENABLE);
        
  RCC_LSEConfig(RCC_LSE_OFF);
  
  BKP_TamperPinCmd(DISABLE);   
  
  PWR_BackupAccessCmd(DISABLE);
}
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
	 /* Configure the interruption (EXTI Source) */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_IRQ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(RADIO_GPIO_IRQ_PORT, &GPIO_InitStructure);
	
	/* Configure I/O for the Chip Enable */
  GPIO_InitStructure.GPIO_Pin = RADIO_GPIO_CE;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(RADIO_GPIO_CE_PORT, &GPIO_InitStructure);

  /* disable the chip enable */
  GPIO_ResetBits(RADIO_GPIO_CE_PORT, RADIO_GPIO_CE);
}
static void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_EXTILineConfig(RADIO_GPIO_IRQ_SRC_PORT, RADIO_GPIO_IRQ_SRC);
  EXTI_InitStructure.EXTI_Line = RADIO_GPIO_IRQ_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/* Write len bytes a nRF24L register. 5 Bytes max */
rt_inline rt_uint8_t write_registers(nrf24l01_dev_t *dev,rt_uint8_t reg, char *buffer, int len)
{
   rt_uint8_t command,status;

  command = CMD_W_REG | (reg&0x1F);
  	/* enable nrf24l01 */
  dev->enable(RT_TRUE);
	rt_spi_transfer(dev->spi,&command,&status,1);
	/* send register value */
	rt_spi_transfer(dev->spi, buffer, RT_NULL, len);
	/* disable nrf24l01 */
  dev->enable(RT_FALSE);
	
  return status;
}

rt_inline rt_uint8_t write_one_register(nrf24l01_dev_t *dev,rt_uint8_t reg, rt_uint8_t value)
{
   rt_uint8_t command,status;

  command = CMD_W_REG | (reg&0x1F);
  	/* enable nrf24l01 */
  dev->enable(RT_TRUE);
	rt_spi_transfer(dev->spi,&command,&status,1);
	/* send register value */
	rt_spi_transfer(dev->spi, &value, RT_NULL, 1);
	/* disable nrf24l01 */
  dev->enable(RT_FALSE);
	
  return status;
}
rt_inline rt_uint8_t flush_rx(nrf24l01_dev_t *dev)
{
  rt_uint8_t command,status;

	command = CMD_FLUSH_RX;
	/* enable nrf24l01 */
  dev->enable(RT_TRUE);
	rt_spi_transfer(dev->spi,&command,&status,1);
	/* disable nrf24l01 */
  dev->enable(RT_FALSE);

  return status;
}

rt_inline rt_uint8_t flush_tx(nrf24l01_dev_t *dev)
{
  rt_uint8_t command,status;

	command = CMD_FLUSH_TX;
	/* enable nrf24l01 */
  dev->enable(RT_TRUE);
	rt_spi_transfer(dev->spi,&command,&status,1);
	/* disable nrf24l01 */
  dev->enable(RT_FALSE);

  return status;
}
rt_inline rt_uint8_t set_rx_address(nrf24l01_dev_t *dev,rt_uint8_t pipe, const char* address)
{
  int len = 5;
 	rt_uint8_t command,status;
	
  RT_ASSERT(pipe<6);

  if (pipe > 1)
    len = 1;

  /* Send the write command with the address */
	command = CMD_W_REG | ((REG_RX_ADDR_P0 + pipe)&0x1F);
	/* enable nrf24l01 */
  dev->enable(RT_TRUE);
	
	rt_spi_transfer(dev->spi,&command,&status,1);
	/* send address */
	rt_spi_transfer(dev->spi, address, RT_NULL, len);
	
	/* disable nrf24l01 */
  dev->enable(RT_FALSE);

  return status;
}
rt_inline rt_uint8_t set_tx_address(nrf24l01_dev_t *dev, const char* address)
{
  int len = 5;
 	rt_uint8_t command,status;
	
  /* Send the write command with the address */
	command = CMD_W_REG | (REG_TX_ADDR&0x1F);
	/* enable nrf24l01 */
  dev->enable(RT_TRUE);
	
	rt_spi_transfer(dev->spi,&command,&status,1);
	/* send address */
	rt_spi_transfer(dev->spi, address, RT_NULL, len);
	
	/* disable nrf24l01 */
  dev->enable(RT_FALSE);

  return status;
}
rt_inline void set_buadrate(nrf24l01_dev_t *dev,nrf24l01_buadrate_t buadrate)
{
  switch(buadrate)
  {
    case NRF24L01_BUADRATE_250K:
      write_one_register(dev,REG_RF_SETUP, VAL_RF_SETUP_250K);
      break;
    case NRF24L01_BUADRATE_1M:
      write_one_register(dev,REG_RF_SETUP, VAL_RF_SETUP_1M);
      break;
    case NRF24L01_BUADRATE_2M:
      write_one_register(dev,REG_RF_SETUP, VAL_RF_SETUP_2M);
      break;
  }  
}

static rt_size_t nrf24l01_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{

        return (0);
}

static rt_size_t nrf24l01_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{

        return (0);

}

/**
 * The following is rt-thread device operating interface
 */
static rt_err_t nrf24l01_init(rt_device_t dev)
{
	 int i;
	 nrf24l01_dev_t *nrf24l01;
   RT_ASSERT(dev!=NULL);
	 RT_ASSERT(dev->user_data!=RT_NULL);
	 nrf24l01 = (nrf24l01_dev_t *) dev->user_data;

  //Set the radio channel
  nrfSetChannel(configblockGetRadioChannel());
  //Set the radio data rate
  set_buadrate(nrf24l01,NRF24L01_BUADRATE_1M);
  //Set radio rx address
  set_rx_address(nrf24l01, 0, Rx_Address);
  //Set radio rx address
  set_tx_address(nrf24l01, Tx_Address);
	
  //Power the radio, Enable the DS interruption, set the radio in PRX mode
  write_one_register(nrf24l01,REG_CONFIG, 0x3F);
  rt_thread_delay(2); //Wait for the chip to be ready
  // Enable the dynamic payload size and the ack payload for the pipe 0
  write_one_register(nrf24l01,REG_FEATURE, 0x06);
  write_one_register(nrf24l01,REG_DYNPD, 0x01);

  //Flush RX
  for(i=0;i<3;i++)
    flush_rx(nrf24l01);
  //Flush TX
  for(i=0;i<3;i++)
    flush_tx(nrf24l01);
		
    return RT_EOK;
}

static rt_err_t nrf24l01_open(rt_device_t dev, rt_uint16_t oflag)
{

    if (dev->user_data == RT_NULL)
    {
        return RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}

static rt_err_t nrf24l01_close(rt_device_t dev)
{
    return RT_EOK;
}
static rt_err_t nrf24l01_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    return RT_EOK;
}
rt_err_t rt_hw_nrf24l01_init(const char *spi_name, nrf24l01_mode_t mode)
{   
	  struct rt_spi_device *spi;
    struct rt_spi_configuration spi_cfg;
    rt_device_t device;
    nrf24l01_dev_t *nrf24l01;
	  spi = (struct rt_spi_device *)rt_device_find(spi_name);
    if (spi == RT_NULL)
    {
        rt_kprintf("spi device %s not found!\n", spi_name);
        return -RT_ERROR;
    }
    nrf24l01 = (nrf24l01_dev_t *)rt_malloc(sizeof(nrf24l01_dev_t));
    if(nrf24l01 == RT_NULL)
		{
		 rt_kprintf("no memory to malloc nrf24l01 device!\n");
		 return -RT_ERROR;
		}
    nrf24l01->spi = spi;
    nrf24l01->mode = mode;
    
		 /* config spi */
    spi_cfg.data_width = 8;
    spi_cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
    spi_cfg.max_hz = 18000000; /* 20M for test. */
    rt_spi_configure(spi, &spi_cfg);

    device = (rt_device_t)rt_malloc(sizeof(struct rt_device));
   if(device==RT_NULL)
    {  
	    rt_kprintf("no memory to malloc nrf24l01 device!\n");
		  rt_free(nrf24l01);
      return -RT_ERROR;
    }

    /* Register nRF24l01 device */
    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->init        = nrf24l01_init;
    device->open        = nrf24l01_open;
    device->close       = nrf24l01_close;
    device->read        = nrf24l01_read;
    device->write       = nrf24l01_write;
    device->control     = nrf24l01_control;
    device->user_data   = nrf24l01;
		
    RCC_Configuration();
		GPIO_Configuration();
		EXTI_Configuration();
		NVIC_Configuration();
		
    rt_device_register(device, "nrf24l01", RT_DEVICE_FLAG_RDWR);
		
		return RT_EOK;
}



