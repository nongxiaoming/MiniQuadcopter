/*
 * File      : usart.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006-2013, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2010-03-29     Bernard      remove interrupt Tx and DMA Rx mode
 * 2013-05-13     aozima       update for kehong-lingtai.
 */

#include "stm32f10x.h"
#include "drv_usart.h"
#include "board.h"


/* USART1 */
#define UART1_GPIO_TX		GPIO_Pin_9
#define UART1_GPIO_RX		GPIO_Pin_10
#define UART1_GPIO			GPIOA


/* STM32 uart driver */
struct stm32_uart
{
    USART_TypeDef* uart_device;
    IRQn_Type irq;
};

static rt_err_t stm32_configure(struct stm32_uart *serial, rt_uint32_t baudrate)
{
    struct stm32_uart* uart;
    USART_InitTypeDef USART_InitStructure;

    RT_ASSERT(serial != RT_NULL);

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart->uart_device, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(uart->uart_device, ENABLE);
    /* enable interrupt */
    USART_ITConfig(uart->uart_device, USART_IT_RXNE, ENABLE);

    return RT_EOK;
}


static int stm32_putc(struct stm32_uart* uart, char c)
{

    while (!(uart->uart_device->SR & USART_FLAG_TXE));
    uart->uart_device->DR = c;

    return 1;
}

static int stm32_getc(struct stm32_uart* uart)
{
    int ch;

    ch = -1;
    if (uart->uart_device->SR & USART_FLAG_RXNE)
    {
        ch = uart->uart_device->DR & 0xff;
    }

    return ch;
}


#if defined(RT_USING_UART1)
/* UART1 device driver structure */

struct stm32_uart uart1 =
{
    USART1,
    USART1_IRQn,
};


void USART1_IRQHandler(void)
{
    struct stm32_uart* uart;

    uart = &uart1;

    /* enter interrupt */
    rt_interrupt_enter();
    if(USART_GetITStatus(uart->uart_device, USART_IT_RXNE) != RESET)
    {
 
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_RXNE);
    }
    if (USART_GetITStatus(uart->uart_device, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(uart->uart_device, USART_IT_TC);
    }

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* RT_USING_UART1 */


static void RCC_Configuration(void)
{
#ifdef RT_USING_UART1
    /* Enable UART GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif /* RT_USING_UART1 */


}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

#ifdef RT_USING_UART1
    /* Configure USART Rx/tx PIN */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);
#endif /* RT_USING_UART1 */

}

static void NVIC_Configuration(struct stm32_uart* uart)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = uart->irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void rt_hw_usart_init(void)
{
 
    RCC_Configuration();
    GPIO_Configuration();

#ifdef RT_USING_UART1


    NVIC_Configuration(&uart1);
#endif /* RT_USING_UART1 */




}
