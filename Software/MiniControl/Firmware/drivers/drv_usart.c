/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2014, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-10-25     xiaonong      the first version
 */

#include "stm32f10x.h"
#include "drv_usart.h"
#include "board.h"


/* USART1 */
#define UART1_GPIO_TX		GPIO_Pin_9
#define UART1_GPIO_RX		GPIO_Pin_10
#define UART1_GPIO			GPIOA



static rt_err_t USART_Configuration(void)
{
    USART_InitTypeDef USART_InitStructure;

     /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);
    /* enable interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    return RT_EOK;
}


static int stm32_putc(USART_TypeDef* uart, char c)
{

    while (!(uart->SR & USART_FLAG_TXE));
    uart->DR = c;

    return 1;
}

static int stm32_getc(USART_TypeDef* uart)
{
    int ch;

    ch = -1;
    if (uart->SR & USART_FLAG_RXNE)
    {
        ch = uart->DR & 0xff;
    }

    return ch;
}



void USART1_IRQHandler(void)
{

    /* enter interrupt */
    rt_interrupt_enter();
	
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
 
        /* clear interrupt */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(USART1, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}


static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
   
	 /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);
   
    /* Configure USART Rx/tx PIN */
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
}

static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void rt_hw_usart_init(void)
{
 
    GPIO_Configuration();
    USART_Configuration();
    NVIC_Configuration();

}

#include "stdio.h"

int fputc(int ch,FILE *f)
{
  stm32_putc(USART1,(char)ch);
  return ch;																	   
}


