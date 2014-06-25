#include "stm32f10x.h"
#include <stdio.h>
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "drv_ssd1306.h"


int main(void)
{
 led_hw_init();
	glcd_test();
  while (1)
  {
	led_hw_on(LED0);
	delay_ms(500);
	led_hw_off(LED0);
	delay_ms(500);
    led_hw_on(LED1);
	delay_ms(500);
	led_hw_off(LED1);
	delay_ms(500);
  }
}
