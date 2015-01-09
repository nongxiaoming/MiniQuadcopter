#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include "drv_nrf24l01.h"


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

typedef struct
{
  struct rt_spi_device *spi;     /* SPI device */
	nrf24l01_mode_t mode;          /* nRF24l01 work mode */
} nrf24l01_dev_t;

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

    rt_device_register(device, "nrf24l01", RT_DEVICE_FLAG_RDWR);
}



