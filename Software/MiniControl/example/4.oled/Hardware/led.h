#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"

#define LED0 GPIO_Pin_10
#define LED1 GPIO_Pin_11

#define led_hw_off(led) GPIO_SetBits(GPIOB,led)
#define led_hw_on(led) GPIO_ResetBits(GPIOB,led)


/******************************************************************************************
*函数名：led_hw_init()
* 参数：void
* 返回值：void
* 功能：LED初始化
*********************************************************************************************/
void led_hw_init(void);
#endif
