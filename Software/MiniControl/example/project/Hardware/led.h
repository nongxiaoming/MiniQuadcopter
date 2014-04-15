#ifndef __LED_H
#define __LED_H
#include "stm32f10x_conf.h"
#define On      0
#define Off     1
#define LED0(x) (x?(GPIO_SetBits(GPIOB,GPIO_Pin_10)):(GPIO_ResetBits(GPIOB,GPIO_Pin_10)))
#define LED1(x) (x?(GPIO_SetBits(GPIOB,GPIO_Pin_11)):(GPIO_ResetBits(GPIOB,GPIO_Pin_11)))
void LED_GPIO_Config(void);
#endif
