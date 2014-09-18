#ifndef _MS5611_H_
#define _MS5611_H_
#include "ANO_TC_STM32F1_I2C.h"
#include "stm32f10x.h"

extern vs32  BaroAlt,BaroOffset;
extern uint32_t ms5611_ut;  // static result of temperature measurement
extern uint32_t ms5611_up;  // static result of pressure measurement

void MS5611_loop(void);
u8 ms5611_reset(void);
u8 MS5611_Init(void);
u8 ms5611_start_t(void);
u8 ms5611_start_p(void);
u8 ms5611_read_adc_t(void);
u8 ms5611_read_adc_p(void);
u8 ms5611_calculate(void);
vs32 MS5611_GetValue(void);
void MS5611_CalOffset(void);

#endif
