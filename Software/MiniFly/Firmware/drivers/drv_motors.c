#include <rtthread.h>
#include "board.h"
#include "drv_motors.h"

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
#define MOTOR4_TIM_REMAP     GPIO_FullRemap_TIM3


#define MOTOR1_GPIO_PERIF          RCC_APB2Periph_GPIOA
#define MOTOR1_GPIO_PORT           GPIOA
#define MOTOR1_GPIO_PIN            GPIO_Pin_2 // T2_CH3
#define MOTOR2_GPIO_PERIF          RCC_APB2Periph_GPIOB
#define MOTOR2_GPIO_PORT           GPIOB
#define MOTOR2_GPIO_PIN            GPIO_Pin_9 // T4_CH4
#define MOTOR3_GPIO_PERIF          RCC_APB2Periph_GPIOC
#define MOTOR3_GPIO_PORT           GPIOC
#define MOTOR3_GPIO_PIN            GPIO_Pin_9 // T3_CH4
#define MOTOR4_GPIO_PERIF          RCC_APB2Periph_GPIOC
#define MOTOR4_GPIO_PORT           GPIOC
#define MOTOR4_GPIO_PIN            GPIO_Pin_8 // T3_CH3

static rt_uint16_t pwm_value[MOTORS_NUM_MAX];

static rt_err_t rt_motors_init(rt_device_t dev)
{

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

  TIM_OC3Init(MOTOR1_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(MOTOR1_TIM, TIM_OCPreload_Enable);

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
	
	TIM_ARRPreloadConfig(MOTOR1_TIM, ENABLE);
  TIM_ARRPreloadConfig(MOTOR2_TIM, ENABLE);
  TIM_ARRPreloadConfig(MOTOR3_TIM, ENABLE);
  TIM_ARRPreloadConfig(MOTOR4_TIM, ENABLE);
  // Halt timer during debug halt.
  DBGMCU_Config(MOTOR1_TIM_DBG, ENABLE);
  DBGMCU_Config(MOTOR2_TIM_DBG, ENABLE);
  DBGMCU_Config(MOTOR3_TIM_DBG, ENABLE);
  DBGMCU_Config(MOTOR4_TIM_DBG, ENABLE);
	
  return RT_EOK;
}

static rt_err_t rt_motors_open(rt_device_t dev, rt_uint16_t oflag)
{	
    return RT_EOK;
}

static rt_err_t rt_motors_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t rt_motors_read(rt_device_t dev, rt_off_t pos, void *buffer,
                             rt_size_t size)
{
    rt_ubase_t index = 0;
    rt_uint16_t *value = buffer;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT((pos + size) <= MOTORS_NUM_MAX);

    for (index = 0; index < size; index++)
    {
        *value = pwm_value[pos+index];
			   value++;
    }
    return size;
}

static rt_size_t rt_motors_write(rt_device_t dev, rt_off_t pos,
                              const void *buffer, rt_size_t size)
{
    rt_ubase_t index = 0;
    rt_ubase_t nw = size;
   const rt_uint16_t *value = buffer;

    RT_ASSERT(dev != RT_NULL);
    RT_ASSERT((pos + size) <= MOTORS_NUM_MAX);

    for (index = 0; index < size; index++)
    {
			if((value[index] >= MOTORS_PWM_MIN)&&(value[index] <= MOTORS_PWM_MAX))
				{
				pwm_value[pos+index]= value[index];
				}
				else if(value[index] < MOTORS_PWM_MIN)
				{
			pwm_value[pos+index] = MOTORS_PWM_MIN;
				}
		else {
			pwm_value[pos+index] = MOTORS_PWM_MAX;
		  }
    }
	MOTOR1_TIM->CCR3 = pwm_value[0] - MOTORS_PWM_MIN;
	MOTOR2_TIM->CCR4 = pwm_value[1] - MOTORS_PWM_MIN;
	MOTOR3_TIM->CCR4 = pwm_value[2] - MOTORS_PWM_MIN;
	MOTOR4_TIM->CCR3 = pwm_value[3] - MOTORS_PWM_MIN;

    return size;
}

static rt_err_t rt_motors_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{

    return RT_EOK;
}

void rt_motors_hw_init(void)
{
	 rt_device_t device = RT_NULL;
	 device = rt_malloc(sizeof(struct rt_device));
	 if(device == RT_NULL)
    {
		 rt_kprintf("no mem to malloc motors device!\n");
			return ;
		}
    device->type         = RT_Device_Class_Char;
    device->rx_indicate  = RT_NULL;
    device->tx_complete  = RT_NULL;
    device->init         = rt_motors_init;
    device->open         = rt_motors_open;
    device->close        = rt_motors_close;
    device->read         = rt_motors_read;
    device->write        = rt_motors_write;
    device->control      = rt_motors_control;
    device->user_data    = RT_NULL;

    /* register a character device */
    rt_device_register(device, "motors", RT_DEVICE_FLAG_RDWR);
  
}

#ifdef RT_USING_FINSH
#include <finsh.h>
void motors_test(rt_uint32_t motors_num, rt_uint32_t value)
{
    rt_device_t dev = RT_NULL;
	  dev = rt_device_find("motors");
	 if(dev == RT_NULL)
   {
	  rt_kprintf("can not found motors device!\n");
		 return ;
	 }
	 rt_device_open(dev,RT_DEVICE_FLAG_RDWR);
	 
    rt_motors_write(dev, motors_num, &value, 1);
}
FINSH_FUNCTION_EXPORT(motors_test, e.g: motors_test(0, 100).)
#endif
