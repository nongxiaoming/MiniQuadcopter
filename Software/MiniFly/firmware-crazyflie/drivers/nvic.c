/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * nvic.c - Contains all Cortex-M3 processor exceptions handlers
 */
#include "exti.h"
#include "adc.h"
#include "led.h"
#include "uart.h"
#include "i2croutines.h"
#include "i2cdev.h"

#define DONT_DISCARD __attribute__((used))

void nvicInit(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}


void DONT_DISCARD DMA1_Channel1_IRQHandler(void)
{
  adcInterruptHandler();
}

void DONT_DISCARD DMA1_Channel2_IRQHandler(void)
{
#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA)
  uartDmaIsr();
#endif
}

void DONT_DISCARD DMA1_Channel4_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c2();
}

void DONT_DISCARD DMA1_Channel5_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c2();
}

void DONT_DISCARD DMA1_Channel6_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c1();
}

void DONT_DISCARD DMA1_Channel7_IRQHandler(void)
{
  i2cDmaInterruptHandlerI2c1();
}


void DONT_DISCARD EXTI9_5_IRQHandler(void)
{
  extiInterruptHandler();
}

void DONT_DISCARD USART3_IRQHandler(void)
{
  uartIsr();
}

void DONT_DISCARD TIM1_UP_IRQHandler(void)
{
  extern uint32_t usecTimerHighCount;

  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

  __sync_fetch_and_add(&usecTimerHighCount, 1);
}

void DONT_DISCARD I2C1_EV_IRQHandler(void)
{
  i2cInterruptHandlerI2c1();
}

void DONT_DISCARD I2C1_ER_IRQHandler(void)
{
  i2cErrorInterruptHandlerI2c1();
}

void DONT_DISCARD I2C2_EV_IRQHandler(void)
{

}

void DONT_DISCARD I2C2_ER_IRQHandler(void)
{
  I2C_ClearFlag(I2C2, 0x1000FFFF);
}

