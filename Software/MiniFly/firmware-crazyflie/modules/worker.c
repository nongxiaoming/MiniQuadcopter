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
 * worker.c - Worker system that can execute asynchronous actions in tasks
 */
#include "worker.h"

#include <errno.h>
#include "console.h"

#define WORKER_QUEUE_LENGTH 5

struct worker_work {
  void (*function)(void*);
  void* arg;
};

static rt_mq_t workerQueue = RT_NULL;

void workerInit(void)
{
  if (workerQueue != RT_NULL)
    return;

  workerQueue = rt_mq_create("work_mq",sizeof(struct worker_work),WORKER_QUEUE_LENGTH,RT_IPC_FLAG_FIFO);
}

rt_bool_t workerTest()
{
	if (workerQueue != RT_NULL)
	{
		return RT_TRUE;
	}	
	else
	{
		return RT_FALSE;
	}
}

void workerLoop(void)
{
  struct worker_work work;

  if (!workerQueue)
    return;

  while (1)
  {
    //xQueueReceive(workerQueue, &work, portMAX_DELAY);
	  rt_mq_recv(workerQueue,&work,sizeof(struct worker_work),RT_WAITING_FOREVER);
    if (work.function)
      work.function(work.arg);
  }
}

int workerSchedule(void (*function)(void*), void *arg)
{
  struct worker_work work;
  
  if (!function)
    return ENOEXEC;
  
  work.function = function;
  work.arg = arg;
  if (rt_mq_send(workerQueue,&work,sizeof(struct worker_work)) != RT_EOK)
    return ENOMEM;

  return 0; 
}

