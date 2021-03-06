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
 */

/*
 * ledseq.c - LED sequence handler
 */
#include <rtthread.h>
#include "board.h"
#include "ledseq.h"
#include "led.h"

/* Led sequence priority */
static ledseq_t * sequences[] = {
  seq_testPassed,
  seq_lowbat,
  seq_charged,
  seq_charging,
  seq_chargingMax,
  seq_bootloader,
  seq_armed,
  seq_calibrated,
  seq_alive,
  seq_linkup,
};

/* Led sequences */
ledseq_t seq_lowbat[] = {
  { RT_TRUE, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_armed[] = {
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(250)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_calibrated[] = {
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(450)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_alive[] = {
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(1950)},
  {    0, LEDSEQ_LOOP},
};


//TODO: Change, right now is called so fast it looks like seq_lowbat
ledseq_t seq_altHold[] = {
  { RT_TRUE, LEDSEQ_WAITMS(1)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  {    0, LEDSEQ_STOP},
};

ledseq_t seq_linkup[] = {
  { RT_TRUE, LEDSEQ_WAITMS(1)},
  {RT_FALSE, LEDSEQ_WAITMS(0)},
  {    0, LEDSEQ_STOP},
};


ledseq_t seq_charged[] = {
  { RT_TRUE, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_charging[] = {
  { RT_TRUE, LEDSEQ_WAITMS(200)},
  {RT_FALSE, LEDSEQ_WAITMS(800)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_chargingMax[] = {
  { RT_TRUE, LEDSEQ_WAITMS(100)},
  {RT_FALSE, LEDSEQ_WAITMS(400)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_bootloader[] = {
  { RT_TRUE, LEDSEQ_WAITMS(500)},
  {RT_FALSE, LEDSEQ_WAITMS(500)},
  {    0, LEDSEQ_LOOP},
};

ledseq_t seq_testPassed[] = {
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  { RT_TRUE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_WAITMS(50)},
  {RT_FALSE, LEDSEQ_STOP},
};

/* Led sequence handling machine implementation */
#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

static void runLedseq(void* parameter);
static int getPrio(ledseq_t *seq);
static void updateActive(led_t led);

//State of every sequence for every led: LEDSEQ_STOP if stopped or the current 
//step
static int state[LED_NUM][SEQ_NUM];
//Active sequence for each led
static int activeSeq[LED_NUM];

static rt_timer_t timer[LED_NUM];

static rt_mutex_t ledseqSem;

static rt_bool_t isInit = RT_FALSE;

void ledseqInit()
{
  int i,j;
  
  if(isInit==RT_TRUE)
    return;
  
  ledInit();
  
  //Initialise the sequences state
  for(i=0; i<LED_NUM; i++) {
    activeSeq[i] = LEDSEQ_STOP;
    for(j=0; j<SEQ_NUM; j++)
      state[i][j] = LEDSEQ_STOP;
  }
  
  //Init the soft timers that runs the led sequences for each leds
  for (i = 0; i < LED_NUM; i++)
  {
	  timer[i] = rt_timer_create("ledseqTimer",
		  runLedseq,
		  (void*)i,
		  M2T(1000),
		  RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

  }
	  
  //vSemaphoreCreateBinary(ledseqSem);
  ledseqSem = rt_mutex_create("ledseq", RT_IPC_FLAG_FIFO);
  
  isInit = RT_TRUE;
}

rt_bool_t ledseqTest(void)
{
  return isInit & ledTest();
}

void ledseqRun(led_t led, ledseq_t *sequence)
{
  int prio = getPrio(sequence);
  
  if(prio<0) return;
  
  rt_mutex_take(ledseqSem, RT_WAITING_FOREVER);
  state[led][prio] = 0;  //Reset the seq. to its first step
  updateActive(led);
  rt_mutex_release(ledseqSem);
  
  //Run the first step if the new seq is the active sequence
  if(activeSeq[led] == prio)
    runLedseq(timer[led]);
}

void ledseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime)
{
  sequence[0].action = onTime;
  sequence[1].action = offTime;
}

void ledseqStop(led_t led, ledseq_t *sequence)
{
  int prio = getPrio(sequence);
  
  if(prio<0) return;
  
  rt_mutex_take(ledseqSem, RT_WAITING_FOREVER);
  state[led][prio] = LEDSEQ_STOP;  //Stop the seq.
  updateActive(led);
  rt_mutex_release(ledseqSem);
  
  //Run the next active sequence (if any...)
  runLedseq(timer[led]);
}

/* Center of the led sequence machine. This function is executed by the FreeRTOS
 * timer and runs the sequences
 */
static void runLedseq(void* parameter)
{
 led_t led = (led_t)parameter;
 rt_uint32_t index = (rt_uint32_t)parameter;
  ledseq_t *step;
  rt_bool_t leave=RT_FALSE;

  while(!leave) {
    int prio = activeSeq[led];
  
    if (prio == LEDSEQ_STOP)
      return;
    
    step = &sequences[prio][state[led][prio]];

    state[led][prio]++;
    
    rt_mutex_take(ledseqSem, RT_WAITING_FOREVER);
    switch(step->action)
    {
      case LEDSEQ_LOOP:
        state[led][prio] = 0;
        break;
      case LEDSEQ_STOP:
        state[led][prio] = LEDSEQ_STOP;
        updateActive(led);
        break;
      default:  //The step is a LED action and a time
        ledSet(led, step->value);
        if (step->action == 0)
          break;
        //xTimerChangePeriod(xTimer, M2T(step->action), 0);
		timer[index]->init_tick = M2T(step->action);
        //xTimerStart(xTimer, 0);
		rt_timer_start(timer[index]);
        leave=RT_TRUE;
        break;
    }
    rt_mutex_release(ledseqSem);
  }
}

//Utility functions
static int getPrio(ledseq_t *seq)
{
  int prio;

  //Find the priority of the sequence
  for(prio=0; prio<SEQ_NUM; prio++)
    if(sequences[prio]==seq) return prio;
  
  return -1; //Invalid sequence
}

static void updateActive(led_t led)
{
  int prio;
  
  activeSeq[led]=LEDSEQ_STOP;
  ledSet(led, RT_FALSE);
  
  for(prio=0;prio<SEQ_NUM;prio++)
  {
    if (state[led][prio] != LEDSEQ_STOP)
    {
      activeSeq[led]=prio;
      break;
    }
  }
}


