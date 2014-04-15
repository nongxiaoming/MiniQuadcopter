#include "si2c.h"


#define DELAY  10
#define SDA_SET GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define SCL_SET GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define SDA_CLR GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define SCL_CLR GPIO_ResetBits(GPIOB,GPIO_Pin_6)

#define SDA_IN  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)


/***************************************************************************************************************
*函数名：SI2C_delay()
*参数  ：unsigned int n 延时参数
*返回值： void
*函数功能：模拟I2C的延时函数
****************************************************************************************************************/
static void SI2C_delay(unsigned int n)	
{						
   	while (n--)
	{
	}				
}


/***************************************************************************************************************
*函数名：SI2C_GPIO_Config()
*参数  :  void
*返回值： void
*函数功能：模拟I2C的管脚配置函数
****************************************************************************************************************/
void SI2C_GPIO_Config(void)	
{
	 GPIO_InitTypeDef GPIO_InitStructure;
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_Init(GPIOB,&GPIO_InitStructure);
	 GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
}

/***************************************************************************************************************
*函数名：SDA_SetDataOut()
*参数  :  void
*返回值： void
*函数功能：模拟I2C的SDA管脚配置为输出函数
****************************************************************************************************************/
void SDA_SetDataOut(void)	
{
	 GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_Init(GPIOB,&GPIO_InitStructure);
	//GPIO_SetBits(GPIOD,GPIO_Pin_4);
}

/***************************************************************************************************************
*函数名：SDA_SetDataIn()
*参数  :  void
*返回值： void
*函数功能：模拟I2C的SDA管脚配置为输入函数
****************************************************************************************************************/
void SDA_SetDataIn(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB,&GPIO_InitStructure);
}

/***************************************************************************************************************
*函数名：SI2C_Start()
*参数  :  void
*返回值： void
*函数功能：模拟I2C启动总线函数
****************************************************************************************************************/
void SI2C_Start(void)
{
    SDA_SET;                    //拉高数据线
    SCL_SET;                    //拉高时钟线
    SI2C_delay(DELAY);          //延时
    SDA_CLR;                    //产生下降沿
    SI2C_delay(DELAY);          //延时
    SCL_CLR;                    //拉低时钟线
}
/***************************************************************************************************************
*函数名：SI2C_Stop()
*参数  :  void
*返回值： void
*函数功能：模拟I2C停止总线函数
****************************************************************************************************************/
void SI2C_Stop(void)
{   
	SCL_CLR;                    //拉低时钟线
	SDA_CLR;                    //产生下降沿
    SCL_SET;                    //拉高时钟线
    SI2C_delay(DELAY);          //延时
    SDA_SET;                    //拉高数据线
    SI2C_delay(DELAY);          //延时  
}

/***************************************************************************************************************
*函数名：SI2C_SendACK()
*参数  :  unsigned char ack 发送应答位，0为有应答，1为无应答
*返回值： void
*函数功能：模拟I2C发送应答位函数
****************************************************************************************************************/
void SI2C_SendACK(unsigned char ack)
{   
	if (ack)
	{
	 SDA_SET;
	}
   else
     {
	SDA_CLR; 
	 }                                 //写应答信号
    SCL_SET;                           //拉高时钟线
    SI2C_delay(DELAY);                 //延时
    SCL_CLR ;                           //拉低时钟线
    SI2C_delay(DELAY);                 //延时
}
/***************************************************************************************************************
*函数名：SI2C_RecvACK()
*参数  :  void
*返回值： unsigned char 1为无应答，0为有应答
*函数功能：模拟I2C接受应答位函数
****************************************************************************************************************/
unsigned char SI2C_RecvACK(void)
{   
	unsigned char ack=1,wait=0xff;
	SDA_SetDataIn();
    SCL_SET;                             //拉高时钟线
    while (SDA_IN&&wait--);              //等待应答信号
    ack= SDA_IN;                         //读应答信号
    SCL_CLR;                             //拉低时钟线
    SI2C_delay(DELAY);                  //延时
	SDA_SetDataOut();
    return ack;
}

/***************************************************************************************************************
*函数名：SI2C_SendByte()
*参数  :  unsigned char dat 发送的字节
*返回值： void
*函数功能：模拟I2C发送一个字节函数
****************************************************************************************************************/
void SI2C_SendByte(unsigned char dat)
{
    unsigned char i=0;
    for (i=0; i<8; i++)          //8位计数器
    {
        if(dat&0x80)             //判断数据的最高位是否为1
        {
		SDA_SET;                //送数据口
		}else
		{
		 SDA_CLR ;               //送数据口
		}
        SCL_SET;                //拉高时钟线
        SI2C_delay(DELAY);      //延时
        SCL_CLR ;                //拉低时钟线
        SI2C_delay(DELAY);      //延时
		dat<<=1;                //数据为左移一位
    }
}
/***************************************************************************************************************
*函数名：SI2C_RecvByte()
*参数  :  void
*返回值： unsigned char 读出的一个字节数据
*函数功能：模拟I2C接收一个字节函数
****************************************************************************************************************/
unsigned char SI2C_RecvByte(void)
{
    unsigned char i;
    unsigned char dat = 0;
	SDA_SetDataIn();            //配置SDA为输入
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL_SET;                //拉高时钟线
        SI2C_delay(DELAY);      //延时
        dat|= SDA_IN;           //读数据               
        SCL_CLR;                //拉低时钟线
        SI2C_delay(DELAY);      //延时
    }
	SDA_SetDataOut();           //设置SDA为输出
    return dat;
}
/***************************************************************************************************************
*函数名：SI2C_WriteData()
*参数  :  unsigned char dev_addr, unsigned char reg_addr, unsigned char *pdata, unsigned char count 器件地址，寄存器地址，数据，数据个数
*返回值： void
*函数功能：模拟I2C写数据函数
****************************************************************************************************************/

void SI2C_WriteData(unsigned char dev_addr, unsigned char reg_addr, unsigned char *pdat, unsigned char count)			 
{
	unsigned char i;
	SI2C_Start();                                                   //启动总线
	SI2C_SendByte(dev_addr);										//写从器件地址
	SI2C_RecvACK();                                                  //等待应答
	
	SI2C_SendByte(reg_addr);											//写数据地址
	SI2C_RecvACK();                                                  //等待应答
	for(i=0; i<count; i++)
	{
		SI2C_SendByte(*pdat);									    //写数据
		SI2C_RecvACK();                                              //等待应答
		pdat++;
	}	
	SI2C_Stop();                                                    //终止总线
	SI2C_delay(DELAY);
}
/***************************************************************************************************************
*函数名：SI2C_RecvByte()
*参数  :  unsigned char dev_addr, unsigned char reg_addr, unsigned char * pdat, unsigned char count 器件地址，寄存器地址，数据，数据个数
*返回值： unsigned char 读出的一个字节数据
*函数功能：模拟I2C接收数据函数
****************************************************************************************************************/
void SI2C_ReadData(unsigned char dev_addr, unsigned char reg_addr, unsigned char * pdat, unsigned char count)
{
	unsigned char i;
	SI2C_Start();                                                   //启动总线
	SI2C_SendByte(dev_addr);										//写从器件地址
	SI2C_RecvACK();                                                  //等待应答
	SI2C_SendByte(reg_addr);									    //写数据地址
	SI2C_RecvACK();                                                  //等待应答
	SI2C_Start();                                                   //启动总线
	SI2C_SendByte(dev_addr+1);										   //写从器件地址
		SI2C_RecvACK();
    for(i = 0; i < (count-1); i++)
    {
        *pdat=SI2C_RecvByte();                                       //接收数据
        SI2C_SendACK(0);                                            //发送就答位
        pdat++;
    }
    *pdat=SI2C_RecvByte();                                            //接收数据
    SI2C_SendACK(1);                                                  //发送非应位
    SI2C_Stop();                                                       //结束总线 
}
