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


#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f10x.h"
#include "drv_usart.h"
/* board configuration */



// <o> Internal SRAM memory size[Kbytes] <8-64>
//	<i>Default: 64
#define STM32_SRAM_SIZE         20
#define STM32_SRAM_END          (0x20000000 + STM32_SRAM_SIZE * 1024)


void rt_hw_board_init(void);

#endif /* __BOARD_H__ */

