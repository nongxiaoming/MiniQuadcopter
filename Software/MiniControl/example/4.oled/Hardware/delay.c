#include "delay.h"
/********************************************************************
*函数名：delay_ms()
* 参数：uint32_t
* 返回值：void
* 功能：简单的软件延时函数实现
**************************************************************************/
void delay_ms(uint32_t n)
{
  uint32_t a,b;
  for(a=0;a<n;a++)
   for(b=0;b<10000;b++);
}
