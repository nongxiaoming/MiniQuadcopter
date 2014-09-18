#include "led.h"
/******************************************************************************************
*��������led_hw_init()
* ������void
* ����ֵ��void
* ���ܣ�LED��ʼ��
*********************************************************************************************/
void led_hw_init(void)
{
     // enable the clock of  GPIOB.
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // clear the PB10,PB11 GPIO config. 
	  GPIOB->CRH&=~(GPIO_CRH_CNF10|GPIO_CRH_CNF11);
	 //set the PB10,PB11 as output
    GPIOB->CRH |= GPIO_CRH_MODE10| GPIO_CRH_MODE11;
    //turn off the led0,led1
	  GPIOB->ODR |= GPIO_ODR_ODR10|GPIO_ODR_ODR11;
}
