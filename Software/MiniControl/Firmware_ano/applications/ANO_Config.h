#ifndef __ANO_CONFIG_H
#define __ANO_CONFIG_H

#include "board.h"
#include "ANO_Scheduler.h"
#include "ANO_DT.h"
#include "ANO_RC.h"
#include "ANO_Param.h"



/*-------------------无线数据发送方式选择-----------------*/
//#define ANO_DT_USE_Bluetooth
#define ANO_DT_USE_NRF24l01
/*--------------------------------------------------------*/


class ANO_Config
{
	
public:
	
	ANO_Config();
	
	class Flag{
		public:
			uint8_t ARMED;
			uint8_t STICKMODE;
			uint8_t ACCELMODE;
	}f;
	
};

extern ANO_Config ano;

#endif

