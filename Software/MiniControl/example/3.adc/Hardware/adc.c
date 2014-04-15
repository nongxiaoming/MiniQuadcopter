#include "adc.h"


__IO uint16_t ADCConvertedValue[6];
//ADC 外设的数据寄存器
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
//ADC_DR(ADC规则数据寄存器),偏移量=0x4c  ADC1(0x40012400-0x400127ff)
//so ADC1_DR_Address=0x40012400+0x4c
//==========================================================================================
//ADC1_GPIO配置
void ADC1_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable ADC1 and GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  /*配置PA0、PA1、PA2、PA3为模拟输入*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	                               //adc模式必须是模拟输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);                                       //执行上面的操作
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

//ADC1  DMA 配置
void ADC1_DMA_Init(void)
{
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
 /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
/* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1); //选择DMA的通道1
  //设定从ADC外设的数据寄存器（ADC1_DR_Address）转移到内存（ADCConcertedValue）
  //每次传输大小16位，使用DMA循环传输模式
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCConvertedValue;//数据缓冲区的地址
  //外设为数据源
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  //数据缓冲区，大小2半字
  DMA_InitStructure.DMA_BufferSize = 6;
  // 外设地址固定
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  //内存地址增加，多组adc时，使能，数据传输时，内存增加
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  //半字
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  //DMA循环传输
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  //优先级高
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  //??
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  //执行
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  //ADC独立模式	 相对于双重模式
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  //扫描模式用于多通道采集
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  //开启连续转换模式   当转换完本组（可能是一个）继续重新开始执行
  //相对于单次模式：转换一次后就结束
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  //不使用外部触发转换
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  //采集数据右对齐
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //转换组的通道数目
  ADC_InitStructure.ADC_NbrOfChannel =6;
  //执行
  ADC_Init(ADC1, &ADC_InitStructure);
  
  //配置ADC时钟，为PCLK2的8分频，即9MHz
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  /* ADC1 regular channel11 configuration */ 
  //配置ADC1的通道11为55.5个采样周期
  //默认组，adc1 ，通道11，排序为1,55.5周期
  //ADC1,ch0,序号1,55.5.。。
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1, ADC_SampleTime_239Cycles5);
  //ADC1,ch1,序号1,55.5.。。
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1,2, ADC_SampleTime_239Cycles5);
  //ADC1,ch2,序号1,55.5.。。
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2,3, ADC_SampleTime_239Cycles5);
  //ADC1,ch3,序号1,55.5.。。
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3,4, ADC_SampleTime_239Cycles5);
  //ADC1,ch8,序号1,55.5.。。
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8,5, ADC_SampleTime_239Cycles5);
   //ADC1,ch16,序号1,55.5.。。
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16,6, ADC_SampleTime_239Cycles5);


  //----------------------使能温度传感器----------------------------
  ADC_TempSensorVrefintCmd(ENABLE);

  /* Enable ADC1 DMA */
  //使能ADC_DMA
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  //使能ADC
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */ 
  //使能ADC1的复位校准寄存器  
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  //等待校准完成
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  //使能ADC1的开始校准寄存器
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  //等待完成
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  //使用软件触发，由于没有采用外部触发
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

