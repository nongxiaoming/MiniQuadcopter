#ifndef __DRV_NRF24L01_H
#define __DRV_NRF24L01_H

#include <stdint.h>
#include <rtthread.h>

#include "stm32f10x.h"

#define DUMMY_BYTE    0xA5

/* Registers address definition */
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_OBSERVE_TX 0x08
#define REG_RPD 0x09
#define REG_RX_ADDR_P0 0x0A
#define REG_RX_ADDR_P1 0x0B
#define REG_RX_ADDR_P2 0x0C
#define REG_RX_ADDR_P3 0x0D
#define REG_RX_ADDR_P4 0x0E
#define REG_RX_ADDR_P5 0x0F
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11
#define REG_RX_PW_P1 0x12
#define REG_RX_PW_P2 0x13
#define REG_RX_PW_P3 0x14
#define REG_RX_PW_P4 0x15
#define REG_RX_PW_P5 0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#define VAL_RF_SETUP_250K 0x26
#define VAL_RF_SETUP_1M   0x06
#define VAL_RF_SETUP_2M   0x0E

#define BIT_RX_DR (1<<6)
#define BIT_TX_DS (1<<5)
#define BIT_MAX_RT (1<<4)

#define VAL_SETUP_AW_3B 1
#define VAL_SETUP_AW_4B 2
#define VAL_SETUP_AW_5B 3

/* nRF24L SPI commands */
#define CMD_R_REG              0x00
#define CMD_W_REG              0x20
#define CMD_R_RX_PAYLOAD       0x61
#define CMD_W_TX_PAYLOAD       0xA0
#define CMD_FLUSH_TX           0xE1
#define CMD_FLUSH_RX           0xE2
#define CMD_REUSE_TX_PL        0xE3
#define CMD_ACTIVATE           0x50
#define CMD_RX_PL_WID          0x60
#define CMD_W_ACK_PAYLOAD(P)  (0xA8|(P&0x0F))
#define CMD_W_PAYLOAD_NO_ACK   0xD0
#define CMD_NOP                0xFF


#define ARD_RAW 0
#define ARD_PLOAD 0x80

//SET_DATA_RATE parameter
#define DATA_RATE_250K 0
#define DATA_RATE_1M   1
#define DATA_RATE_2M   2

//SET_RADIO_POWER parameter
#define RADIO_POWER_M18dBm 0
#define RADIO_POWER_M12dBm 1
#define RADIO_POWER_M6dBm  2
#define RADIO_POWER_0dBm   3

void rt_hw_nrf24l01_init(void);

#endif /* __DRV_NRF24L01_H */
