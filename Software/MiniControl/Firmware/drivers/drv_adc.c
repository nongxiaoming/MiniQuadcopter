#include "drv_adc.h"
#include "stdio.h"

 static adc_values_t ADCConvertedValue[ADC_MEAN_SIZE*2];

/* ADC GPIO Configure */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable ADC1 and GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  /* Configure PA0¡¢PA1¡¢PA2¡¢PA3 as analog input mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	                               
  GPIO_Init(GPIOA, &GPIO_InitStructure);    
  /* Configure PB0 as analog input mode */	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}
/* ADC DMA Configure */
static void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // DMA channel1 configuration
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3 * (ADC_MEAN_SIZE * 2);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  // Enable DMA channel1
  DMA_Cmd(DMA1_Channel1, ENABLE);
}
/* TIM Configure */
static void TIM_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	// Enable TIM2 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = (uint16_t)ADC_TRIG_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = ADC_TRIG_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  // TIM2 channel2 configuration in PWM mode
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  // Halt timer 2 during debug halt.
  DBGMCU_Config(DBGMCU_TIM2_STOP, ENABLE);

}
/* NVIC Configure */
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  // Enable the DMA1 channel1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
static void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 , ENABLE);
// ADC1 configuration
  ADC_DeInit(ADC1);
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC1, &ADC_InitStructure);

  // ADC1 channel sequence
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_28Cycles5);
	
  // ADC2 configuration
  ADC_DeInit(ADC2);
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC2, &ADC_InitStructure);

  // ADC2 channel sequence
  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_28Cycles5);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 2, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_17, 3, ADC_SampleTime_28Cycles5);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);
  // Calibrate ADC1
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

  // Enable ADC1 external trigger
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_TempSensorVrefintCmd(ENABLE);

  // Enable ADC2
  ADC_Cmd(ADC2, ENABLE);
  // Calibrate ADC2
  ADC_ResetCalibration(ADC2);
  while(ADC_GetResetCalibrationStatus(ADC2));
  ADC_StartCalibration(ADC2);
  while(ADC_GetCalibrationStatus(ADC2));

  // Enable ADC2 external trigger
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);	
	
}

void ADC_StartConver(void)
{
  // Enable the Transfer Complete and Half Transfer Interrupt
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_HT, ENABLE);
  // Enable ADC1 DMA
  ADC_DMACmd(ADC1, ENABLE);
  // TIM2 counter enable
  TIM_Cmd(TIM2, ENABLE);
}

void ADC_StopConver(void)
{
  TIM_Cmd(TIM2, DISABLE);
	ADC_DMACmd(ADC1, DISABLE);
}

void ADC_DMA_IRQHandler(void)
{
 
  if(DMA_GetITStatus(DMA1_IT_HT1))
  {
    DMA_ClearITPendingBit(DMA1_IT_HT1);
    printf("%d,%d,%d,%d,%d,%d\r\n",ADCConvertedValue[0].adc_value1,ADCConvertedValue[0].adc_value2,ADCConvertedValue[0].adc_value3,ADCConvertedValue[0].adc_value4,ADCConvertedValue[0].adc_value5,ADCConvertedValue[0].adc_value6);
  }
  if(DMA_GetITStatus(DMA1_IT_TC1))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC1);
  }
}

void adc_hw_init(void)
{
 GPIO_Configuration();
 TIM_Configuration();
 DMA_Configuration();
 ADC_Configuration();
 NVIC_Configuration();
 
}

