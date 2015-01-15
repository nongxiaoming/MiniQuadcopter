/*
 * File      : board.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-09-22     Bernard      add board.h to this bsp
 */

// <<< Use Configuration Wizard in Context Menu >>>
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#include <stm32f10x.h>

/* board configuration */
#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN  ((void *)&Image$$RW_IRAM1$$ZI$$Limit)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN  (__segment_end("HEAP"))
#else
extern int __bss_end;
#define HEAP_BEGIN  ((void *)&__bss_end)
#endif

// <o> Internal SRAM memory size[Kbytes] <8-128>
//	<i>Default: 64
#define STM32_SRAM_SIZE         64
#define HEAP_END          (0x20000000 + STM32_SRAM_SIZE * 1024)


#define RT_USING_UART1
void rt_hw_board_init(void);
void rt_hw_usart_init(void);

uint32_t GetSysTime_us(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

// <<< Use Configuration Wizard in Context Menu >>>
