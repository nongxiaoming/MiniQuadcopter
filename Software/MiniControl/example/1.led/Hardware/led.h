#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"

#define LED0 GPIO_Pin_10
#define LED1 GPIO_Pin_11

#define led_hw_off(led) GPIO_SetBits(GPIOB,led)
#define led_hw_on(led) GPIO_ResetBits(GPIOB,led)


/******************************************************************************************
*��������led_hw_init()
* ������void
* ����ֵ��void
* ���ܣ�LED��ʼ��
*********************************************************************************************/
void led_hw_init(void);
#endif
