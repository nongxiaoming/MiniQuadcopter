#include "usart.h"
/*******************************************************************************************************************************
 * 函数名：USART1_Config()
 * 参数：无
 *返回值：无
 * 功能：配置USART1，并初始化
********************************************************************************************************************************/																		
void USART1_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;                                            //定义GPIO初始化结构体
	USART_InitTypeDef USART_InitStructure;                                        //定义USART初始化结构体
	/*配置USART1相应的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);    //使能GPIOA和USART1的APB2时钟
	/*配置USART1的发送管脚TXD(PA9)为复用推挽输出*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;                                       //设置初始化GPIO为PIN9
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                               //设置GPIO的速度为50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;                                 //设置GPIO模式为复用推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //初始化GPIOA的PIN9
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;                                      //设置初始化GPIO为PIN10
	/*配置USART1的接收管脚RXD(PA10)为浮空输入*/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;                           //设置GPIO的模式为浮空输入，这里需要注意，和PIN9不同
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //初始化GPIOA的PIN10
	/*配置USART1的模式*/
    USART_InitStructure.USART_BaudRate=115200;                                    //设置USART的波特率为9600
    USART_InitStructure.USART_Parity=USART_Parity_No;                             //设置USART的校验位为None
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;                     //设置USART的数据位为8bit
    USART_InitStructure.USART_StopBits=USART_StopBits_1;                          //设置USART的停止位为1
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; //失能硬件流控制
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                   //设置USART的模式为发送接收模式
    USART_Init(USART1,&USART_InitStructure);                                      //初始化USART1
    USART_Cmd(USART1,ENABLE);                                                     //使能USART1
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
 * 函数名：USART2_Config()
 * 参数：无
 *返回值：无
 * 功能：配置USART2，并初始化
********************************************************************************************************************************/																		
void USART2_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;                                            //定义GPIO初始化结构体
	USART_InitTypeDef USART_InitStructure;                                        //定义USART初始化结构体
	/*配置USART1相应的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);                          //使能GPIOA的APB2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);                         //使能USART2的APB1时钟
	/*配置USART1的发送管脚TXD(PA9)为复用推挽输出*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;                                       //设置初始化GPIO为PIN2
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;                               //设置GPIO的速度为50MHz
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;                                 //设置GPIO模式为复用推挽输出
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //初始化GPIOA的PIN2
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;                                      //设置初始化GPIO为PIN3
	/*配置USART1的接收管脚RXD(PA10)为浮空输入*/
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;                           //设置GPIO的模式为浮空输入，这里需要注意，和PIN9不同
	GPIO_Init(GPIOA,&GPIO_InitStructure);                                         //初始化GPIOA的PIN3
	/*配置USART1的模式*/
    USART_InitStructure.USART_BaudRate=9600;                                       //设置USART的波特率为9600
    USART_InitStructure.USART_Parity=USART_Parity_No;                             //设置USART的校验位为None
    USART_InitStructure.USART_WordLength=USART_WordLength_8b;                     //设置USART的数据位为8bit
    USART_InitStructure.USART_StopBits=USART_StopBits_1;                          //设置USART的停止位为1
    USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None; //失能硬件流控制
    USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;                   //设置USART的模式为发送接收模式
    USART_Init(USART2,&USART_InitStructure);                                      //初始化USART2
    USART_Cmd(USART2,ENABLE);                                                     //使能USART2
		USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
		NVIC_Config();
}
/********************************************************************************************************************************
*函数名：fputc()
* 参数：int ch，FILE *f
* 返回值：int
* 功能：重新定义stdio.h中的fputc()函数，使printf()输出到USART1
*********************************************************************************************************************************/
int fputc(int ch,FILE *f)
{
  USART_SendData(USART1,(unsigned char)ch);	                                       //将ch从USART1发送出去
  while(!(USART1->SR&USART_FLAG_TXE));											   //等待发送完毕
  return ch;																	   //返回 ch
}																		           








