#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"

#define LED0 0
#define LED1 1
#define LED_NUM  2
#define LED0_PIN 10
#define LED1_PIN 11

#define led_hw_on(led) GPIOB->BRR=0x01<<(LED0_PIN+led)
#define led_hw_off(led) GPIOB->BSRR=0x01<<(LED0_PIN+led)



/******************************************************************************************
*函数名：led_hw_init()
* 参数：void
* 返回值：void
* 功能：LED初始化
*********************************************************************************************/
void led_hw_init(void);
#endif
