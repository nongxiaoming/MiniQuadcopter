#include "adc.h"


__IO uint16_t ADCConvertedValue[6];
//ADC ��������ݼĴ���
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
//ADC_DR(ADC�������ݼĴ���),ƫ����=0x4c  ADC1(0x40012400-0x400127ff)
//so ADC1_DR_Address=0x40012400+0x4c
//==========================================================================================
//ADC1_GPIO����
void ADC1_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable ADC1 and GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  /*����PA0��PA1��PA2��PA3Ϊģ������*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	                               //adcģʽ������ģ������
  GPIO_Init(GPIOA, &GPIO_InitStructure);                                       //ִ������Ĳ���
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
}

//ADC1  DMA ����
void ADC1_DMA_Init(void)
{
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
 /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
/* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1); //ѡ��DMA��ͨ��1
  //�趨��ADC��������ݼĴ�����ADC1_DR_Address��ת�Ƶ��ڴ棨ADCConcertedValue��
  //ÿ�δ����С16λ��ʹ��DMAѭ������ģʽ
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCConvertedValue;//���ݻ������ĵ�ַ
  //����Ϊ����Դ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  //���ݻ���������С2����
  DMA_InitStructure.DMA_BufferSize = 6;
  // �����ַ�̶�
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  //�ڴ��ַ���ӣ�����adcʱ��ʹ�ܣ����ݴ���ʱ���ڴ�����
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  //����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  //DMAѭ������
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  //���ȼ���
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  //??
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  //ִ��
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  //ADC����ģʽ	 �����˫��ģʽ
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  //ɨ��ģʽ���ڶ�ͨ���ɼ�
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  //��������ת��ģʽ   ��ת���걾�飨������һ�����������¿�ʼִ��
  //����ڵ���ģʽ��ת��һ�κ�ͽ���
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  //��ʹ���ⲿ����ת��
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  //�ɼ������Ҷ���
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  //ת�����ͨ����Ŀ
  ADC_InitStructure.ADC_NbrOfChannel =6;
  //ִ��
  ADC_Init(ADC1, &ADC_InitStructure);
  
  //����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  /* ADC1 regular channel11 configuration */ 
  //����ADC1��ͨ��11Ϊ55.5����������
  //Ĭ���飬adc1 ��ͨ��11������Ϊ1,55.5����
  //ADC1,ch0,���1,55.5.����
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1, ADC_SampleTime_239Cycles5);
  //ADC1,ch1,���1,55.5.����
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1,2, ADC_SampleTime_239Cycles5);
  //ADC1,ch2,���1,55.5.����
  ADC_RegularChannelConfig(ADC1, ADC_Channel_2,3, ADC_SampleTime_239Cycles5);
  //ADC1,ch3,���1,55.5.����
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3,4, ADC_SampleTime_239Cycles5);
  //ADC1,ch8,���1,55.5.����
  ADC_RegularChannelConfig(ADC1, ADC_Channel_8,5, ADC_SampleTime_239Cycles5);
   //ADC1,ch16,���1,55.5.����
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16,6, ADC_SampleTime_239Cycles5);


  //----------------------ʹ���¶ȴ�����----------------------------
  ADC_TempSensorVrefintCmd(ENABLE);

  /* Enable ADC1 DMA */
  //ʹ��ADC_DMA
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  //ʹ��ADC
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */ 
  //ʹ��ADC1�ĸ�λУ׼�Ĵ���  
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  //�ȴ�У׼���
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  //ʹ��ADC1�Ŀ�ʼУ׼�Ĵ���
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  //�ȴ����
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  //ʹ���������������û�в����ⲿ����
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}

