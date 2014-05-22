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
 * pidctrl.c - Used to receive/answer requests from client and to receive updated PID values from client
 */
#include <rtthread.h>
#include "crtp.h"
#include "pidctrl.h"
#include "pid.h"

typedef enum {
  pidCtrlValues = 0x00,
} PIDCrtlNbr;

void pidCrtlTask(void *param);

void pidCtrlInit()
{
  rt_thread_t pidctrl_thread;
  //xTaskCreate(pidCrtlTask, (const signed char * const)"PIDCrtl",
   //           configMINIMAL_STACK_SIZE, NULL, /*priority*/2, NULL);
  pidctrl_thread=rt_thread_create("pidctrl",pidCrtlTask,RT_NULL,512,12,5);
  if(pidctrl_thread!=RT_NULL)
  {
   rt_thread_startup(pidctrl_thread);
  }
  crtpInitTaskQueue(6);
}

void pidCrtlTask(void *param)
{
  CRTPPacket p;
  extern pid_t pidRollRate;
  extern pid_t pidPitchRate;
  extern pid_t pidYawRate;
  extern pid_t pidRoll;
  extern pid_t pidPitch;
  extern pid_t pidYaw;
  struct pidValues
  {
    rt_uint16_t rateKpRP;
    rt_uint16_t rateKiRP;
    rt_uint16_t rateKdRP;
    rt_uint16_t attKpRP;
    rt_uint16_t attKiRP;
    rt_uint16_t attKdRP;
    rt_uint16_t rateKpY;
    rt_uint16_t rateKiY;
    rt_uint16_t rateKdY;
    rt_uint16_t attKpY;
    rt_uint16_t attKiY;
    rt_uint16_t attKdY;
  }  __attribute__((packed));
  struct pidValues *pPid;

  while (RT_TRUE)
  {
    if (crtpReceivePacketBlock(6, &p) == RT_EOK)
    {
      PIDCrtlNbr pidNbr = p.channel;
      
      switch (pidNbr)
      {
        case pidCtrlValues:
          pPid = (struct pidValues *)p.data;
          {
            pidSetKp(&pidRollRate, (float)pPid->rateKpRP/100.0);
            pidSetKi(&pidRollRate, (float)pPid->rateKiRP/100.0);
            pidSetKd(&pidRollRate, (float)pPid->rateKdRP/100.0);
            pidSetKp(&pidRoll, (float)pPid->attKpRP/100.0);
            pidSetKi(&pidRoll, (float)pPid->attKiRP/100.0);
            pidSetKd(&pidRoll, (float)pPid->attKdRP/100.0);
            pidSetKp(&pidPitchRate, (float)pPid->rateKpRP/100.0);
            pidSetKi(&pidPitchRate, (float)pPid->rateKiRP/100.0);
            pidSetKd(&pidPitchRate, (float)pPid->rateKdRP/100.0);
            pidSetKp(&pidPitch, (float)pPid->attKpRP/100.0);
            pidSetKi(&pidPitch, (float)pPid->attKiRP/100.0);
            pidSetKd(&pidPitch, (float)pPid->attKdRP/100.0);
            pidSetKp(&pidYawRate, (float)pPid->rateKpY/100.0);
            pidSetKi(&pidYawRate, (float)pPid->rateKiY/100.0);
            pidSetKd(&pidYawRate, (float)pPid->rateKdY/100.0);
            pidSetKp(&pidYaw, (float)pPid->attKpY/100.0);
            pidSetKi(&pidYaw, (float)pPid->attKiY/100.0);
            pidSetKd(&pidYaw, (float)pPid->attKdY/100.0);
          }
          break;
        default:
          break;
      } 
    }
  }
}

