/*
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
 * system.c - Top level module implementation
 */

#include <rtthread.h>

#include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"

#include "system.h"
#include "configblock.h"
#include "worker.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"

#include "console.h"

/* Private variable */
static rt_bool_t canFly;

static rt_bool_t isInit;

/* System wide synchronisation */
//xSemaphoreHandle canStartMutex;
 rt_mutex_t canStartMutex;
/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
 rt_thread_t system_thread;
 // xTaskCreate(systemTask, (const signed char * const)"SYSTEM",
    //          2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);
 system_thread=rt_thread_create("system",systemTask,RT_NULL,1024,12,5);
 if(system_thread!=RT_NULL)
 {
	 rt_thread_startup(system_thread);
 }

}

//This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = rt_mutex_create("Start",RT_IPC_FLAG_FIFO);
 // xSemaphoreTake(canStartMutex, portMAX_DELAY);
  rt_mutex_take(canStartMutex, RT_WAITING_FOREVER);

  configblockInit();
  workerInit();
  adcInit();
  ledseqInit();
    
  isInit = RT_TRUE;
}

rt_bool_t systemTest()
{
  rt_bool_t pass=isInit;
  
  pass &= adcTest();
  pass &= ledseqTest();
  pass &= workerTest();
  
  return pass;
}

/* Private functions implementation */

extern int paramsLen;

void systemTask(void *arg)
{
  rt_bool_t pass = RT_TRUE;
  
  //Init the high-levels modules
  systemInit();

  commInit();

  DEBUG("Crazyflie is up and running!\n");
  DEBUG("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
              V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  DEBUG("I am 0x%X%X%X and I have %dKB of flash!\n",
              *((int*)(0x1FFFF7E8+8)), *((int*)(0x1FFFF7E8+4)),
              *((int*)(0x1FFFF7E8+0)), *((short*)(0x1FFFF7E0)));

  commanderInit();
  stabilizerInit();
  
  //Test the modules
  pass &= systemTest();
  pass &= commTest();
  pass &= commanderTest();
  pass &= stabilizerTest();
  
  //Start the firmware
  if(1)
  {
    systemStart();
    ledseqRun(LED_RED, seq_alive);
    ledseqRun(LED_GREEN, seq_testPassed);
  }
  else
  {
    if (systemTest())
    {
      while(1)
      {
        ledseqRun(LED_RED, seq_testPassed); //Red passed == not passed!
		rt_thread_delay(M2T(2000));
      }
    }
    else
    {
      ledInit();
      ledSet(LED_RED, RT_TRUE);
    }
  }
  
  workerLoop();
  
  //Should never reach this point!
  while(1)
    rt_thread_delay(100);
}


/* Global system variables */
void systemStart()
{
  //xSemaphoreGive(canStartMutex);
  rt_mutex_release(canStartMutex);
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
	  rt_thread_delay(2);

  //xSemaphoreTake(canStartMutex, portMAX_DELAY);
  rt_mutex_take(canStartMutex, RT_WAITING_FOREVER);
  //xSemaphoreGive(canStartMutex);
  rt_mutex_release(canStartMutex);
}

void systemSetCanFly(rt_bool_t val)
{
  canFly = val;
}

rt_bool_t systemCanFly(void)
{
  return canFly;
}

/*System parameters (mostly for test, should be removed from here) */
PARAM_GROUP_START(cpu)
PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, flash, 0x1FFFF7E0)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id0, 0x1FFFF7E8+0)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id1, 0x1FFFF7E8+4)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, id2, 0x1FFFF7E8+8)
PARAM_GROUP_STOP(cpu)

/* Loggable variables */
LOG_GROUP_START(sys)
LOG_ADD(LOG_INT8, canfly, &canFly)
LOG_GROUP_STOP(sys)

