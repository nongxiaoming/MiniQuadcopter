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
 * motors.c - Motor driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 */

#include <rtthread.h>
#include "board.h"
#ifdef RT_USING_FINSH
#include "finsh.h"
#endif
#include "motors.h"


// HW defines
#define MOTOR1_TIM_PERIF     RCC_APB1Periph_TIM2
#define MOTOR1_TIM           TIM2
#define MOTOR1_TIM_DBG       DBGMCU_TIM2_STOP
//#define MOTOR1_TIM_REMAP     GPIO_PartialRemap_TIM2

#define MOTOR2_TIM_PERIF     RCC_APB1Periph_TIM4
#define MOTOR2_TIM           TIM4
#define MOTOR2_TIM_DBG       DBGMCU_TIM4_STOP

#define MOTOR3_TIM_PERIF     RCC_APB1Periph_TIM3
#define MOTOR3_TIM           TIM3
#define MOTOR3_TIM_DBG       DBGMCU_TIM3_STOP
#define MOTOR3_TIM_REMAP     GPIO_FullRemap_TIM3

#define MOTOR4_TIM_PERIF     RCC_APB1Periph_TIM3
#define MOTOR4_TIM           TIM3
#define MOTOR4_TIM_DBG       DBGMCU_TIM3_STOP
#define MOTOR4_TIM_REMAP      GPIO_FullRemap_TIM3


#define MOTOR1_GPIO_PERIF          RCC_APB2Periph_GPIOA
#define MOTOR1_GPIO_PORT           GPIOA
#define MOTOR1_GPIO_PIN            GPIO_Pin_3 // T2_CH4
#define MOTOR2_GPIO_PERIF          RCC_APB2Periph_GPIOB
#define MOTOR2_GPIO_PORT           GPIOB
#define MOTOR2_GPIO_PIN            GPIO_Pin_9 // T4_CH4
#define MOTOR3_GPIO_PERIF          RCC_APB2Periph_GPIOC
#define MOTOR3_GPIO_PORT           GPIOC
#define MOTOR3_GPIO_PIN            GPIO_Pin_9 // T3_CH4
#define MOTOR4_GPIO_PERIF          RCC_APB2Periph_GPIOC
#define MOTOR4_GPIO_PORT           GPIOC
#define MOTOR4_GPIO_PIN            GPIO_Pin_8 // T3_CH3


/* Utils Conversion macro */
#ifdef BRUSHLESS_MOTORCONTROLLER
  #define C_BITS_TO_16(X) (0xFFFF * (X - MOTORS_PWM_CNT_FOR_1MS) / MOTORS_PWM_CNT_FOR_1MS)
  #define C_16_TO_BITS(X) (MOTORS_PWM_CNT_FOR_1MS + ((X * MOTORS_PWM_CNT_FOR_1MS) / 0xFFFF))
#else
  #define C_BITS_TO_16(X) ((X)<<(16-MOTORS_PWM_BITS))
  #define C_16_TO_BITS(X) ((X)>>(16-MOTORS_PWM_BITS)&((1<<MOTORS_PWM_BITS)-1))
#endif

const int MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };
static rt_bool_t isInit = RT_FALSE;

/* Public functions */

//Initialization. Will set all motors ratio to 0%
void motorsInit()
{
  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Enable gpio and the timer
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO 
	                    | MOTOR1_GPIO_PERIF
						| MOTOR2_GPIO_PERIF 
						| MOTOR3_GPIO_PERIF 
						| MOTOR4_GPIO_PERIF, ENABLE);

  RCC_APB1PeriphClockCmd( MOTOR1_TIM_PERIF 
	                    | MOTOR2_TIM_PERIF 
	                    | MOTOR3_TIM_PERIF
	                    | MOTOR4_TIM_PERIF, ENABLE);

  GPIO_AFIODeInit();  
  //Remap MOTO3 TIM
  GPIO_PinRemapConfig(MOTOR3_TIM_REMAP, ENABLE);

  //Remap MOTO4 TIM
  GPIO_PinRemapConfig(MOTOR4_TIM_REMAP, ENABLE);

  // Configure the GPIO for the MOTO1 output
  GPIO_InitStructure.GPIO_Pin = MOTOR1_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MOTOR1_GPIO_PORT, &GPIO_InitStructure);

  // Configure the GPIO for the MOTO2 output
  GPIO_InitStructure.GPIO_Pin = MOTOR2_GPIO_PIN;
  GPIO_Init(MOTOR2_GPIO_PORT, &GPIO_InitStructure);

  // Configure the GPIO for the MOTO3 output
  GPIO_InitStructure.GPIO_Pin = MOTOR3_GPIO_PIN;
  GPIO_Init(MOTOR3_GPIO_PORT, &GPIO_InitStructure);

  // Configure the GPIO for the MOTO2 output
  GPIO_InitStructure.GPIO_Pin = MOTOR4_GPIO_PIN;
  GPIO_Init(MOTOR4_GPIO_PORT, &GPIO_InitStructure);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTOR1_TIM, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTOR2_TIM, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTOR3_TIM, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = MOTORS_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = MOTORS_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(MOTOR4_TIM, &TIM_TimeBaseStructure);

  //PWM channels configuration (All identical!)
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = MOTORS_POLARITY;

  TIM_OC4Init(MOTOR1_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTOR1_TIM, TIM_OCPreload_Enable);

  TIM_OC4Init(MOTOR2_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTOR2_TIM, TIM_OCPreload_Enable);

  TIM_OC4Init(MOTOR3_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(MOTOR3_TIM, TIM_OCPreload_Enable);

  TIM_OC3Init(MOTOR4_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTOR4_TIM, TIM_OCPreload_Enable);

  //Enable the timer
  TIM_Cmd(MOTOR1_TIM, ENABLE);
  TIM_Cmd(MOTOR2_TIM, ENABLE);
  TIM_Cmd(MOTOR3_TIM, ENABLE);
  TIM_Cmd(MOTOR4_TIM, ENABLE);
  //Enable the timer PWM outputs
  //TIM_CtrlPWMOutputs(MOTOR1_TIM, ENABLE);
  //TIM_CtrlPWMOutputs(MOTOR2_TIM, ENABLE);
  //TIM_CtrlPWMOutputs(MOTOR3_TIM, ENABLE);
  //TIM_CtrlPWMOutputs(MOTOR4_TIM, ENABLE);
  // Halt timer during debug halt.
  DBGMCU_Config(MOTOR1_TIM_DBG, ENABLE);
  DBGMCU_Config(MOTOR2_TIM_DBG, ENABLE);
  DBGMCU_Config(MOTOR3_TIM_DBG, ENABLE);
  DBGMCU_Config(MOTOR4_TIM_DBG, ENABLE);
  
  isInit = RT_TRUE;
}

rt_bool_t motorsTest(void)
{
#ifndef BRUSHLESS_MOTORCONTROLLER
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
	rt_thread_delay(M2T(MOTORS_TEST_ON_TIME_MS));
    motorsSetRatio(MOTORS[i], 0);
	rt_thread_delay(M2T(MOTORS_TEST_DELAY_TIME_MS));
  }
#endif

  return isInit;
}
#ifdef RT_USING_FINSH
FINSH_FUNCTION_EXPORT(motorsTest, motors test);
#endif

void motorsSetRatio(int id, uint16_t ratio)
{
  switch(id)
  {
    case MOTOR_M1:
      TIM_SetCompare4(MOTOR1_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M2:
      TIM_SetCompare4(MOTOR2_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M3:
      TIM_SetCompare4(MOTOR3_TIM, C_16_TO_BITS(ratio));
      break;
    case MOTOR_M4:
      TIM_SetCompare3(MOTOR4_TIM, C_16_TO_BITS(ratio));
      break;
  }
}

int motorsGetRatio(int id)
{
  switch(id)
  {
    case MOTOR_M1:
      return C_BITS_TO_16(TIM_GetCapture4(MOTOR1_TIM));
    case MOTOR_M2:
      return C_BITS_TO_16(TIM_GetCapture4(MOTOR2_TIM));
    case MOTOR_M3:
      return C_BITS_TO_16(TIM_GetCapture4(MOTOR3_TIM));
    case MOTOR_M4:
      return C_BITS_TO_16(TIM_GetCapture3(MOTOR4_TIM));
  }

  return -1;
}

#ifdef MOTOR_RAMPUP_TEST
// FreeRTOS Task to test the Motors driver with a rampup of each motor alone.
void motorsTestTask(void* params)
{
  int step=0;
  float rampup = 0.01;

  motorsSetRatio(MOTOR_M4, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M3, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M2, 1*(1<<16) * 0.0);
  motorsSetRatio(MOTOR_M1, 1*(1<<16) * 0.0);
  vTaskDelay(M2T(1000));

  while(1)
  {
    vTaskDelay(M2T(100));

    motorsSetRatio(MOTOR_M4, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M3, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M2, 1*(1<<16) * rampup);
    motorsSetRatio(MOTOR_M1, 1*(1<<16) * rampup);

    rampup += 0.001;
    if (rampup >= 0.1)
    {
      if(++step>3) step=0;
      rampup = 0.01;
    }
  }
}
#else
// FreeRTOS Task to test the Motors driver
void motorsTestTask(void* params)
{
  static const int sequence[] = {0.1*(1<<16), 0.15*(1<<16), 0.2*(1<<16), 0.25*(1<<16)};
  int step=0;

  //Wait 3 seconds before starting the motors
  rt_thread_delay(M2T(3000));

  while(1)
  {
    motorsSetRatio(MOTOR_M4, sequence[step%4]);
    motorsSetRatio(MOTOR_M3, sequence[(step+1)%4]);
    motorsSetRatio(MOTOR_M2, sequence[(step+2)%4]);
    motorsSetRatio(MOTOR_M1, sequence[(step+3)%4]);

    if(++step>3) step=0;

    rt_thread_delay(M2T(1000));
  }
}
#endif

