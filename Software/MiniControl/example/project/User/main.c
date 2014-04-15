#include "stm32f10x.h"
#include <stdio.h>
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "mpu6050.h"
#include "hmc5883l.h"


int main(void)
{
	short temp1[3];
short temp2[4];
short temp3;
double temp;
int	ang;
u8 i;
LED_GPIO_Config();
USART1_Config();
MPU6050_Init();
delay_ms (200);	
	InitCmp();
  while (1)
  {
		
		LED0 (On);
		delay_ms(500);
		LED0 (Off);
		delay_ms(500);
	 	//i=MPU6050ReadID();
		MPU6050ReadAcc(temp1);
	  delay_ms(20);
		MPU6050ReadGyro(temp2);
		delay_ms(20);
		printf("M,%d,%d,%d,%d,%d,%d\r\n",temp1[0],temp1[1],temp1[2],temp2[0],temp2[1],temp2[2]);
		delay_ms(50);
			ReadCmpOut(temp2,&ang);
		//ReadACCDevData(temp1);
		printf("A,%d,%d,%d\r\n",temp2[0],temp2[1],temp2[2]);
		//printf("%d,%d,%d",temp2[0],temp2[1],temp2[2]);
// 		MPU6050ReadTemp(&temp3);
// 		temp=35+ ((double) (temp3 + 13200)) / 280;
// 		printf("ÎÂ¶È£º%x\r\n",i);
  }
}
