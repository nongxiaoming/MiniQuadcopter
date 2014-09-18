#include "led.h"
/******************************************************************************************
*函数名：led_hw_init()
* 参数：void
* 返回值：void
* 功能：LED初始化
*********************************************************************************************/
void led_hw_init(void)
{
      // enable the clock of GPIOA and GPIOB.
    RCC->APB2RSTR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_IOPBEN;

    // clear the PB10,PB11 GPIO config. 
	  GPIOB->CRH&=~(0xff<<3);
	  //set the PB10,PB11 as output
    GPIOB->CRH |= GPIO_CRH_MODE10| GPIO_CRH_MODE11;
    //turn off the led0,led1
	  GPIOB->ODR |= GPIO_ODR_ODR10|GPIO_ODR_ODR11;
}
