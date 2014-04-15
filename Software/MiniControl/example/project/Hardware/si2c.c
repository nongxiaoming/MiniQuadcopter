#include "si2c.h"


#define DELAY  10
#define SDA_SET GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define SCL_SET GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define SDA_CLR GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define SCL_CLR GPIO_ResetBits(GPIOB,GPIO_Pin_6)

#define SDA_IN  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)


/***************************************************************************************************************
*��������SI2C_delay()
*����  ��unsigned int n ��ʱ����
*����ֵ�� void
*�������ܣ�ģ��I2C����ʱ����
****************************************************************************************************************/
static void SI2C_delay(unsigned int n)	
{						
   	while (n--)
	{
	}				
}


/***************************************************************************************************************
*��������SI2C_GPIO_Config()
*����  :  void
*����ֵ�� void
*�������ܣ�ģ��I2C�Ĺܽ����ú���
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
*��������SDA_SetDataOut()
*����  :  void
*����ֵ�� void
*�������ܣ�ģ��I2C��SDA�ܽ�����Ϊ�������
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
*��������SDA_SetDataIn()
*����  :  void
*����ֵ�� void
*�������ܣ�ģ��I2C��SDA�ܽ�����Ϊ���뺯��
****************************************************************************************************************/
void SDA_SetDataIn(void)	
{
	GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
   GPIO_Init(GPIOB,&GPIO_InitStructure);
}

/***************************************************************************************************************
*��������SI2C_Start()
*����  :  void
*����ֵ�� void
*�������ܣ�ģ��I2C�������ߺ���
****************************************************************************************************************/
void SI2C_Start(void)
{
    SDA_SET;                    //����������
    SCL_SET;                    //����ʱ����
    SI2C_delay(DELAY);          //��ʱ
    SDA_CLR;                    //�����½���
    SI2C_delay(DELAY);          //��ʱ
    SCL_CLR;                    //����ʱ����
}
/***************************************************************************************************************
*��������SI2C_Stop()
*����  :  void
*����ֵ�� void
*�������ܣ�ģ��I2Cֹͣ���ߺ���
****************************************************************************************************************/
void SI2C_Stop(void)
{   
	SCL_CLR;                    //����ʱ����
	SDA_CLR;                    //�����½���
    SCL_SET;                    //����ʱ����
    SI2C_delay(DELAY);          //��ʱ
    SDA_SET;                    //����������
    SI2C_delay(DELAY);          //��ʱ  
}

/***************************************************************************************************************
*��������SI2C_SendACK()
*����  :  unsigned char ack ����Ӧ��λ��0Ϊ��Ӧ��1Ϊ��Ӧ��
*����ֵ�� void
*�������ܣ�ģ��I2C����Ӧ��λ����
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
	 }                                 //дӦ���ź�
    SCL_SET;                           //����ʱ����
    SI2C_delay(DELAY);                 //��ʱ
    SCL_CLR ;                           //����ʱ����
    SI2C_delay(DELAY);                 //��ʱ
}
/***************************************************************************************************************
*��������SI2C_RecvACK()
*����  :  void
*����ֵ�� unsigned char 1Ϊ��Ӧ��0Ϊ��Ӧ��
*�������ܣ�ģ��I2C����Ӧ��λ����
****************************************************************************************************************/
unsigned char SI2C_RecvACK(void)
{   
	unsigned char ack=1,wait=0xff;
	SDA_SetDataIn();
    SCL_SET;                             //����ʱ����
    while (SDA_IN&&wait--);              //�ȴ�Ӧ���ź�
    ack= SDA_IN;                         //��Ӧ���ź�
    SCL_CLR;                             //����ʱ����
    SI2C_delay(DELAY);                  //��ʱ
	SDA_SetDataOut();
    return ack;
}

/***************************************************************************************************************
*��������SI2C_SendByte()
*����  :  unsigned char dat ���͵��ֽ�
*����ֵ�� void
*�������ܣ�ģ��I2C����һ���ֽں���
****************************************************************************************************************/
void SI2C_SendByte(unsigned char dat)
{
    unsigned char i=0;
    for (i=0; i<8; i++)          //8λ������
    {
        if(dat&0x80)             //�ж����ݵ����λ�Ƿ�Ϊ1
        {
		SDA_SET;                //�����ݿ�
		}else
		{
		 SDA_CLR ;               //�����ݿ�
		}
        SCL_SET;                //����ʱ����
        SI2C_delay(DELAY);      //��ʱ
        SCL_CLR ;                //����ʱ����
        SI2C_delay(DELAY);      //��ʱ
		dat<<=1;                //����Ϊ����һλ
    }
}
/***************************************************************************************************************
*��������SI2C_RecvByte()
*����  :  void
*����ֵ�� unsigned char ������һ���ֽ�����
*�������ܣ�ģ��I2C����һ���ֽں���
****************************************************************************************************************/
unsigned char SI2C_RecvByte(void)
{
    unsigned char i;
    unsigned char dat = 0;
	SDA_SetDataIn();            //����SDAΪ����
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL_SET;                //����ʱ����
        SI2C_delay(DELAY);      //��ʱ
        dat|= SDA_IN;           //������               
        SCL_CLR;                //����ʱ����
        SI2C_delay(DELAY);      //��ʱ
    }
	SDA_SetDataOut();           //����SDAΪ���
    return dat;
}
/***************************************************************************************************************
*��������SI2C_WriteData()
*����  :  unsigned char dev_addr, unsigned char reg_addr, unsigned char *pdata, unsigned char count ������ַ���Ĵ�����ַ�����ݣ����ݸ���
*����ֵ�� void
*�������ܣ�ģ��I2Cд���ݺ���
****************************************************************************************************************/

void SI2C_WriteData(unsigned char dev_addr, unsigned char reg_addr, unsigned char *pdat, unsigned char count)			 
{
	unsigned char i;
	SI2C_Start();                                                   //��������
	SI2C_SendByte(dev_addr);										//д��������ַ
	SI2C_RecvACK();                                                  //�ȴ�Ӧ��
	
	SI2C_SendByte(reg_addr);											//д���ݵ�ַ
	SI2C_RecvACK();                                                  //�ȴ�Ӧ��
	for(i=0; i<count; i++)
	{
		SI2C_SendByte(*pdat);									    //д����
		SI2C_RecvACK();                                              //�ȴ�Ӧ��
		pdat++;
	}	
	SI2C_Stop();                                                    //��ֹ����
	SI2C_delay(DELAY);
}
/***************************************************************************************************************
*��������SI2C_RecvByte()
*����  :  unsigned char dev_addr, unsigned char reg_addr, unsigned char * pdat, unsigned char count ������ַ���Ĵ�����ַ�����ݣ����ݸ���
*����ֵ�� unsigned char ������һ���ֽ�����
*�������ܣ�ģ��I2C�������ݺ���
****************************************************************************************************************/
void SI2C_ReadData(unsigned char dev_addr, unsigned char reg_addr, unsigned char * pdat, unsigned char count)
{
	unsigned char i;
	SI2C_Start();                                                   //��������
	SI2C_SendByte(dev_addr);										//д��������ַ
	SI2C_RecvACK();                                                  //�ȴ�Ӧ��
	SI2C_SendByte(reg_addr);									    //д���ݵ�ַ
	SI2C_RecvACK();                                                  //�ȴ�Ӧ��
	SI2C_Start();                                                   //��������
	SI2C_SendByte(dev_addr+1);										   //д��������ַ
		SI2C_RecvACK();
    for(i = 0; i < (count-1); i++)
    {
        *pdat=SI2C_RecvByte();                                       //��������
        SI2C_SendACK(0);                                            //���;ʹ�λ
        pdat++;
    }
    *pdat=SI2C_RecvByte();                                            //��������
    SI2C_SendACK(1);                                                  //���ͷ�Ӧλ
    SI2C_Stop();                                                       //�������� 
}
