#include "stm32f10x.h"
#include <stdio.h>
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"


extern uint16_t ADCConvertedValue[6];

int main(void)
{
 led_hw_init();
 USART1_Config();
 ADC1_DMA_Init();
  while (1)
  {
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	led_hw_on(LED0);
	delay_ms(100);
	led_hw_off(LED0);
	delay_ms(100);
	printf("ch1:%d,ch2:%d,ch3:%d,ch4:%d,ch5:%d,ch6:%d\r\n",
		  ADCConvertedValue[0],
		  ADCConvertedValue[1],
		  ADCConvertedValue[2],
		  ADCConvertedValue[3],
		  ADCConvertedValue[4],
		  ADCConvertedValue[5]);
    led_hw_on(LED1);
	delay_ms(100);
	led_hw_off(LED1);
	delay_ms(100);
  }
}
