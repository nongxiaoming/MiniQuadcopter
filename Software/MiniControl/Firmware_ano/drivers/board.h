
#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f10x.h"
#include "string.h"


#define ARMAPI extern "C"

/***************LED GPIO定义******************/
#define ANO_RCC_LED			RCC_APB2Periph_GPIOC
#define ANO_GPIO_LED		GPIOC
#define ANO_Pin_LED			GPIO_Pin_13
/*********************************************/

/***************SPI2 GPIO定义******************/
#define ANO_GPIO_SPI2		GPIOB
#define ANO_GPIO_CE2		GPIOB

#define SPI2_Pin_CE2		GPIO_Pin_5
#define SPI2_Pin_CSN		GPIO_Pin_12
#define RCC_GPIO_SPI2		RCC_APB2Periph_GPIOB
#define RCC_GPIO_CE2		RCC_APB2Periph_GPIOB
/*********************************************/

/***************硬件中断优先级******************/
#define NVIC_UART_P	5
#define NVIC_UART_S	1
/***********************************************/

#include "ANO_Config.h"
#include "ANO_Drv_Usart1.h"
#include "ANO_Drv_ADC.h"
#include "ANO_Drv_SPI2.h"
#include "ANO_Drv_Nrf24l01.h"
#include "ANO_Drv_EEPROM.h"

void ANO_Remoter_board_Init(void);
void SysTick_IRQ(void);


#endif /* __BOARD_H__ */

// 




