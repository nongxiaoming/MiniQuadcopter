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
 * pm.c - Power Management driver and functions.
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "led.h"
#include "log.h"
#include "adc.h"
#include "ledseq.h"
#include "commander.h"
#include "radiolink.h"

static float    batteryVoltage;
static float    batteryVoltageMin = 6.0;
static float    batteryVoltageMax = 0.0;
static int32_t  batteryVRawFilt = PM_BAT_ADC_FOR_3_VOLT;
static int32_t  batteryVRefRawFilt = PM_BAT_ADC_FOR_1p2_VOLT;
static uint32_t batteryLowTimeStamp;
static uint32_t batteryCriticalLowTimeStamp;
static bool isInit=FALSE;
static PMStates pmState;

static void pmSetBatteryVoltage(float voltage);

const static float bat671723HS25C[10] =
{
  3.00, // 00%
  3.78, // 10%
  3.83, // 20%
  3.87, // 30%
  3.89, // 40%
  3.92, // 50%
  3.96, // 60%
  4.00, // 70%
  4.04, // 80%
  4.10  // 90%
};

void pmInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  if(isInit==TRUE)
    return;

  RCC_APB2PeriphClockCmd(PM_GPIO_BAT_PERIF, ENABLE);

  // Configure battery ADC pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = PM_GPIO_BAT;
  GPIO_Init(PM_GPIO_BAT_PORT, &GPIO_InitStructure);
  
  xTaskCreate(pmTask, (const signed char * const)"PWRMGNT",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/3, NULL);
  
  isInit = TRUE;
}

bool pmTest(void)
{
  return isInit;
}

/**
 * IIR low pass filter the samples.
 */
static int16_t pmBatteryIIRLPFilter(uint16_t in, int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  // Shift to keep accuracy
  inScaled = in << PM_BAT_IIR_SHIFT;
  // Calculate IIR filter
  filttmp = filttmp + (((inScaled-filttmp) >> 8) * PM_BAT_IIR_LPF_ATT_FACTOR);
  // Scale and round
  out = (filttmp >> 8) + ((filttmp & (1 << (PM_BAT_IIR_SHIFT - 1))) >> (PM_BAT_IIR_SHIFT - 1));
  *filt = filttmp;

  return out;
}

/**
 * Sets the battery voltage and its min and max values
 */
static void pmSetBatteryVoltage(float voltage)
{
  batteryVoltage = voltage;
  if (batteryVoltageMax < voltage)
  {
    batteryVoltageMax = voltage;
  }
  if (batteryVoltageMin > voltage)
  {
    batteryVoltageMin = voltage;
  }
}


float pmGetBatteryVoltage(void)
{
  //return batteryVoltage;
	return 4.10;
}

float pmGetBatteryVoltageMin(void)
{
  return batteryVoltageMin;
}

float pmGetBatteryVoltageMax(void)
{
  return batteryVoltageMax;
}

void pmBatteryUpdate(AdcGroup* adcValues)
{
  float vBat;
  int16_t vBatRaw;
  int16_t vBatRefRaw;

  vBatRaw = pmBatteryIIRLPFilter(adcValues->vbat.val, &batteryVRawFilt);
  vBatRefRaw = pmBatteryIIRLPFilter(adcValues->vbat.vref, &batteryVRefRawFilt);

  vBat = adcConvertToVoltageFloat(vBatRaw, vBatRefRaw) * PM_BAT_DIVIDER;
  pmSetBatteryVoltage(vBat);
}


PMStates pmUpdateState()
{
  PMStates state;
  uint32_t batteryLowTime;

  batteryLowTime = xTaskGetTickCount() - batteryLowTimeStamp;

 if (batteryLowTime > PM_BAT_LOW_TIMEOUT)
  {
    state = lowPower;
  }
  else
  {
    state = battery;
  }

  return state;
}


void pmTask(void *param)
{
  PMStates pmStateOld = battery;
  uint32_t tickCount;

  vTaskSetApplicationTaskTag(0, (void*)TASK_PM_ID_NBR);

  tickCount = xTaskGetTickCount();
  batteryLowTimeStamp = tickCount;
  batteryCriticalLowTimeStamp = tickCount;

  vTaskDelay(1000);

  while(1)
  {
    vTaskDelay(100);
    tickCount = xTaskGetTickCount();

    if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE)
    {
      batteryLowTimeStamp = tickCount;
    }
    if (pmGetBatteryVoltage() > PM_BAT_CRITICAL_LOW_VOLTAGE)
    {
      batteryCriticalLowTimeStamp = tickCount;
    }

    pmState = pmUpdateState();

    if (pmState != pmStateOld)
    {
      // Actions on state change
      switch (pmState)
      {
        case lowPower:
          ledseqRun(LED_RED, seq_lowbat);
          systemSetCanFly(TRUE);
          break;
        case battery:
          ledseqStop(LED_GREEN, seq_charging);
          ledseqStop(LED_GREEN, seq_chargingMax);
          ledseqStop(LED_GREEN, seq_charged);
          systemSetCanFly(TRUE);
          //Due to voltage change radio must be restarted
          radiolinkReInit();
          break;
        default:
          systemSetCanFly(TRUE);
          break;
      }
      pmStateOld = pmState;
    }
    // Actions during state
    switch (pmState)
    {
      case lowPower:
        {
          uint32_t batteryCriticalLowTime;
          
          batteryCriticalLowTime = tickCount - batteryCriticalLowTimeStamp;
          if (batteryCriticalLowTime > PM_BAT_CRITICAL_LOW_TIMEOUT)
          {
            //pmSystemShutdown();  //关闭系统电源
          }
        }
        break;
      case battery:
        {
          if ((commanderGetInactivityTime() > PM_SYSTEM_SHUTDOWN_TIMEOUT))
          {
            //pmSystemShutdown();  //关闭系统电源
          }
        }
        break;
      default:
        break;
    }
  }
}

LOG_GROUP_START(pm)
LOG_ADD(LOG_FLOAT, vbat, &batteryVoltage)
LOG_ADD(LOG_INT8, state, &pmState)
LOG_GROUP_STOP(pm)

