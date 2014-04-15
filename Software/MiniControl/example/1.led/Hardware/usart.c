#include "usart.h"
/*******************************************************************************************************************************
 * ��������USART1_Config()
 * ��������
 *����ֵ����
 * ���ܣ�����USART1������ʼ��
********************************************************************************************************************************/																		
void USART1_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;                                            //����GPIO��ʼ���ṹ��
	USART_InitTypeDef USART_InitStructure;                                        //����USART��ʼ���ṹ��
	/*����USART1��Ӧ��ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);    //ʹ��GPIOA��USART1��APB2ʱ��
	/*����USART1�ķ��͹ܽ�TXD(PA9)Ϊ�����������*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;                                       //���ó�ʼ��GPIOΪPIN9
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                               //����GPIO���ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;                                 //����GPIOģʽΪ�����������
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //��ʼ��GPIOA��PIN9
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;                                      //���ó�ʼ��GPIOΪPIN10
	/*����USART1�Ľ��չܽ�RXD(PA10)Ϊ��������*/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;                           //����GPIO��ģʽΪ�������룬������Ҫע�⣬��PIN9��ͬ
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //��ʼ��GPIOA��PIN10
	/*����USART1��ģʽ*/
    USART_InitStructure.USART_BaudRate=115200;                                    //����USART�Ĳ�����Ϊ9600
    USART_InitStructure.USART_Parity=USART_Parity_No;                             //����USART��У��λΪNone
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;                     //����USART������λΪ8bit
    USART_InitStructure.USART_StopBits=USART_StopBits_1;                          //����USART��ֹͣλΪ1
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; //ʧ��Ӳ��������
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                   //����USART��ģʽΪ���ͽ���ģʽ
    USART_Init(USART1,&USART_InitStructure);                                      //��ʼ��USART1
    USART_Cmd(USART1,ENABLE);                                                     //ʹ��USART1
}
static void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/*******************************************************************************************************************************
 * ��������USART2_Config()
 * ��������
 *����ֵ����
 * ���ܣ�����USART2������ʼ��
********************************************************************************************************************************/																		
void USART2_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;                                            //����GPIO��ʼ���ṹ��
	USART_InitTypeDef USART_InitStructure;                                        //����USART��ʼ���ṹ��
	/*����USART1��Ӧ��ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);                          //ʹ��GPIOA��APB2ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);                         //ʹ��USART2��APB1ʱ��
	/*����USART1�ķ��͹ܽ�TXD(PA9)Ϊ�����������*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;                                       //���ó�ʼ��GPIOΪPIN2
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                               //����GPIO���ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;                                 //����GPIOģʽΪ�����������
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //��ʼ��GPIOA��PIN2
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;                                      //���ó�ʼ��GPIOΪPIN3
	/*����USART1�Ľ��չܽ�RXD(PA10)Ϊ��������*/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;                           //����GPIO��ģʽΪ�������룬������Ҫע�⣬��PIN9��ͬ
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //��ʼ��GPIOA��PIN3
	/*����USART1��ģʽ*/
    USART_InitStructure.USART_BaudRate=9600;                                       //����USART�Ĳ�����Ϊ9600
    USART_InitStructure.USART_Parity=USART_Parity_No;                             //����USART��У��λΪNone
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;                     //����USART������λΪ8bit
    USART_InitStructure.USART_StopBits=USART_StopBits_1;                          //����USART��ֹͣλΪ1
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; //ʧ��Ӳ��������
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                   //����USART��ģʽΪ���ͽ���ģʽ
    USART_Init(USART2,&USART_InitStructure);                                      //��ʼ��USART2
    USART_Cmd(USART2,ENABLE);                                                     //ʹ��USART2
		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
		NVIC_Config();
}
/********************************************************************************************************************************
*��������fputc()
* ������int ch��FILE *f
* ����ֵ��int
* ���ܣ����¶���stdio.h�е�fputc()������ʹprintf()�����USART1
*********************************************************************************************************************************/
int fputc(int ch,FILE *f)
{
  USART_SendData(USART1,(unsigned char)ch);	                                       //��ch��USART1���ͳ�ȥ
  while(!(USART1->SR&USART_FLAG_TXE));											   //�ȴ��������
  return ch;																	   //���� ch
}																		           








