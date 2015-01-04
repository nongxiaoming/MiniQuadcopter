/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Config.cpp
 * 描述    ：配置文件
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "config.h"

Config config;


Config::Config(void)
{
	
}

//指示灯
void Config::Pilot_Light(void)
{
	static u8 cnt = 0;
	
	if(f.ARMED)
	{
		cnt++;
		switch(cnt)
		{
			case 1:
				//LED_ON();
				break;
			case 25:
				//LED_OFF();
				break;
			case 50:
				cnt = 0;
				break;
		}
	}
	else
	{
		//LED_ON(); 
	}
	
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
