#ifndef __DRV_USART_H__
#define __DRV_USART_H__

#include <rthw.h>
#include <rtthread.h>

#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))

void rt_hw_usart_init(void);

#endif
