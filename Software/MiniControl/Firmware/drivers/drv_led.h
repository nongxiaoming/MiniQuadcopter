/*
 * File      : led.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-07-01     xiaonong      the first version
 */

#ifndef __DRV_LED_H
#define __DRV_LED_H

#include <rtthread.h>

#define LED_NUM  2

#define LED0_PIN 10
#define LED1_PIN 11

rt_err_t led_hw_init(void);

#define led_hw_on(i) GPIOB->BRR|=0x01<<(LED0_PIN+i)
#define led_hw_off(i) GPIOB->BSRR|=0x01<<(LED0_PIN+i)

#endif
