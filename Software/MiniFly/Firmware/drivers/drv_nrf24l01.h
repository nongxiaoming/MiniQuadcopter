#ifndef __MPU6500_H
#define __MPU6500_H
#include <rtthread.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
/**
  * @brief  Internal register Memory
  */
typedef enum
{
 NRF24L01_MODE_TX1 = 0x01,
 NRF24L01_MODE_RX1,
 NRF24L01_MODE_TX2,
 NRF24L01_MODE_RX2,
}nrf24l01_mode_t;
typedef enum
{
NRF24L01_BUADRATE_250K,
NRF24L01_BUADRATE_1M,
NRF24L01_BUADRATE_2M,
}nrf24l01_buadrate_t;
rt_err_t rt_hw_nrf24l01_init(const char *spi_name, nrf24l01_mode_t mode);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

