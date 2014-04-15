#include "hmc5883l.h"

void InitCmp(void)
{

    unsigned char byData[4];
    GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<0;    //��������ʱ��
	//AFIO->MAPR=0X04000000; //�ر�JTAG
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_Init(GPIOB,&GPIO_InitStructure);
	 GPIO_ResetBits(GPIOB,GPIO_Pin_3);
	
	 GPIO_SetBits(GPIOB,GPIO_Pin_3);
   // CFG_A=0xC1, CFG_B=0xA0, MODE=0x00�ɲ��Գ�ģ�麸�Ӻ�оƬ�Ƿ��ջ�����ʱ������X,Y,Z����Ϊ400-1000��������оƬ��������
    byData[0] = 0xC1; //byData[0] = 0x70; //0xC0; 	// SELF-TESTʱΪ0xC1	   			 //ƽ��������8  15hz  
    byData[1] = 0x00; //byData[1] = 0x20; // 0x40; 	//0x20; // SELF-TESTʱΪ0xA0		 //��Χ+_1.9Ga   ����820��˹
    byData[2] = 0x00; // MODE // SELF-TESTʱΪ0X00    ����Ϊ��������ģʽ  			 //

    SI2C_WriteData(DEVICE_WR_ADDR, CFG_A, byData, 3);   //����д�����Ĵ���

}
unsigned char IIC_ReadCmpData(unsigned char byAD, unsigned char * pData, 
                                unsigned char byCount)
{
    u8 i;

    SI2C_Start();                   /*��������*/
    SI2C_SendByte(byAD);                /*����������ַ*/   
    SI2C_RecvACK();

    for(i = 0; i < (byCount-1); i++)
    {
        *pData=SI2C_RecvByte();                /*��������*/
        SI2C_SendACK(0);                      /*���;ʹ�λ*/  
        pData++;
    }

    *pData=SI2C_RecvByte();
    SI2C_SendACK(1);                    /*���ͷ�Ӧλ*/
    SI2C_Stop();                    /*��������*/ 
    return(1);
}

void ReadCmpOut(short * pdat, int *angle_int)
{
    unsigned char  status;
    unsigned char byData[6];
    unsigned int wTemp;
    int i;
	float angle;
	double x=0,y=0;
    unsigned char len = 6;
 //	double x,y;
 //	double angle ;

    for (i=0; i<len; i++)
    {
        byData[i] = 0;
    }

    // ��ѯ�����Ƿ����
    while (1)
    {
        SI2C_WriteData(DEVICE_WR_ADDR, STATUS, byData, 0);	 //0x9  ��д������λ��ָ��	     0���ֽ�
        IIC_ReadCmpData(DEVICE_RD_ADDR, &status, 1);		 // ����״̬�Ĵ������ж������Ƿ����
        if (status&0x01 == 1)
        {
            break;
        }
       // Delay_Us(20);
    }

    // ���ö�ָ��
    SI2C_WriteData(DEVICE_WR_ADDR, X_M, byData, 0);	  //X_M   X��Ĵ����ߵ�ַ	 ��д

    // ������
    IIC_ReadCmpData(DEVICE_RD_ADDR, byData, len);	  //������дʱ����������,ֱ�Ӷ�ȡ�ӵ�ַ����


    wTemp  = 0;
    wTemp = byData[0] << 8;			 //��������Ϊ��λ��ǰ
    wTemp |= byData[1];
    pdat[0] = wTemp;

    wTemp  = 0;
    wTemp = byData[2] << 8;
    wTemp |= byData[3];
    pdat[1]= wTemp;
    
    
    wTemp  = 0;
    wTemp = byData[4] << 8;
    wTemp |= byData[5];
    pdat[2] = wTemp;
    
	x = (double)pdat[0]; //(float)(*pwX*XSF + XOffset);
  y = (double)pdat[1]; //(float)(*pwY*YSF + YOffset);

 angle = atan2((double)y,(double)x) * (180 / 3.14159265) + 180;

    *angle_int = (int)angle;			  //��5883l����Ƕȣ�������
	  //*angle_int = 0;
    //printf("Double:%f\r\n", angle);

}
