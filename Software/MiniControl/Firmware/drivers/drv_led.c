/*
 * File      : led.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */
#include <rtthread.h>
#include <stm32f10x.h>
#include "drv_led.h"

static rt_bool_t led_inited = RT_FALSE;

rt_err_t led_hw_init(void)
{
	 if(led_inited==RT_TRUE)
		 {
      return RT_EOK;
     }
    // enable the clock of  GPIOB.
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // clear the PB10,PB11 GPIO config. 
	  GPIOB->CRH&=~(0xff<<3);
	  //set the PB10,PB11 as output
    GPIOB->CRH |= GPIO_CRH_MODE10| GPIO_CRH_MODE11;
    //turn off the led0,led1
	  GPIOB->ODR |= GPIO_ODR_ODR10|GPIO_ODR_ODR11;
	
	  led_inited = RT_TRUE;
		 return RT_EOK;
}


#ifdef RT_USING_FINSH
#include <finsh.h>

void led(rt_uint32_t led, rt_uint32_t value)
{
    /* init led configuration if it's not inited. */
    if (RT_FALSE==led_inited)
    {
        rt_hw_led_init();
    }

    if ( led < LED_NUM )
    {
        /* set led status */
        if(value>0)
					{
          led_hw_on(led);
          }else
					{
          led_hw_off(led);
          }
    }
}
FINSH_FUNCTION_EXPORT(led, set led[0 - 1] on[1] or off[0].)
#endif

