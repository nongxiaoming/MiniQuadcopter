#include "hmc5883l.h"

void InitCmp(void)
{

    unsigned char byData[4];
    GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<0;    //开启辅助时钟
	//AFIO->MAPR=0X04000000; //关闭JTAG
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
   GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
   GPIO_Init(GPIOB,&GPIO_InitStructure);
	 GPIO_ResetBits(GPIOB,GPIO_Pin_3);
	
	 GPIO_SetBits(GPIOB,GPIO_Pin_3);
   // CFG_A=0xC1, CFG_B=0xA0, MODE=0x00可测试出模块焊接后芯片是否被烧坏，此时如果输出X,Y,Z数据为400-1000的数据则芯片是正常的
    byData[0] = 0xC1; //byData[0] = 0x70; //0xC0; 	// SELF-TEST时为0xC1	   			 //平均采样数8  15hz  
    byData[1] = 0x00; //byData[1] = 0x20; // 0x40; 	//0x20; // SELF-TEST时为0xA0		 //范围+_1.9Ga   增益820高斯
    byData[2] = 0x00; // MODE // SELF-TEST时为0X00    设置为连续测量模式  			 //

    SI2C_WriteData(DEVICE_WR_ADDR, CFG_A, byData, 3);   //连续写三个寄存器

}
unsigned char IIC_ReadCmpData(unsigned char byAD, unsigned char * pData, 
                                unsigned char byCount)
{
    u8 i;

    SI2C_Start();                   /*启动总线*/
    SI2C_SendByte(byAD);                /*发送器件地址*/   
    SI2C_RecvACK();

    for(i = 0; i < (byCount-1); i++)
    {
        *pData=SI2C_RecvByte();                /*接收数据*/
        SI2C_SendACK(0);                      /*发送就答位*/  
        pData++;
    }

    *pData=SI2C_RecvByte();
    SI2C_SendACK(1);                    /*发送非应位*/
    SI2C_Stop();                    /*结束总线*/ 
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

    // 查询数据是否就绪
    while (1)
    {
        SI2C_WriteData(DEVICE_WR_ADDR, STATUS, byData, 0);	 //0x9  虚写，设置位置指针	     0个字节
        IIC_ReadCmpData(DEVICE_RD_ADDR, &status, 1);		 // 读出状态寄存器，判断数据是否就绪
        if (status&0x01 == 1)
        {
            break;
        }
       // Delay_Us(20);
    }

    // 设置读指针
    SI2C_WriteData(DEVICE_WR_ADDR, X_M, byData, 0);	  //X_M   X轴寄存器高地址	 虚写

    // 读数据
    IIC_ReadCmpData(DEVICE_RD_ADDR, byData, len);	  //接上面写时序，重启总线,直接读取子地址内容


    wTemp  = 0;
    wTemp = byData[0] << 8;			 //磁力数据为高位在前
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

    *angle_int = (int)angle;			  //由5883l计算角度？？？？
	  //*angle_int = 0;
    //printf("Double:%f\r\n", angle);

}
