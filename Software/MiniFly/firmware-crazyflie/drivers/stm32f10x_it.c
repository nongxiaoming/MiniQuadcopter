/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <board.h>
#include <rtthread.h>
#include "exti.h"
#include "adc.h"
#include "led.h"
#include "i2croutines.h"
#include "i2cdev.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

void  DMA1_Channel1_IRQHandler(void)
{
	adcInterruptHandler();
}

void  DMA1_Channel2_IRQHandler(void)
{
#if defined(UART_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA)
	uartDmaIsr();
#endif
}

void  DMA1_Channel4_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c2();
}

void  DMA1_Channel5_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c2();
}

void  DMA1_Channel6_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c1();
}

void  DMA1_Channel7_IRQHandler(void)
{
	i2cDmaInterruptHandlerI2c1();
}


void  EXTI1_IRQHandler(void)
{
	nrfIsr();
	EXTI_ClearITPendingBit(EXTI_Line1);
}

void  TIM1_UP_IRQHandler(void)
{
	extern uint32_t usecTimerHighCount;

	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

	__sync_fetch_and_add(&usecTimerHighCount, 1);
}

void  I2C1_EV_IRQHandler(void)
{
	i2cInterruptHandlerI2c1();
}

void  I2C1_ER_IRQHandler(void)
{
	i2cErrorInterruptHandlerI2c1();
}

void  I2C2_EV_IRQHandler(void)
{

}

void  I2C2_ER_IRQHandler(void)
{
	I2C_ClearFlag(I2C2, 0x1000FFFF);
}

/**
  * @}
  */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
