#ifndef __SI2C_H
#define __SI2C_H
#include "stm32f10x.h"


/***************************************************************************************************************
*函数名：SI2C_GPIO_Config()
*参数  :  void
*返回值： void
*函数功能：模拟I2C的管脚配置函数
****************************************************************************************************************/
void SI2C_GPIO_Config(void)	;
/***************************************************************************************************************
*函数名：SDA_SetDataOut()
*参数  :  void
*返回值： void
*函数功能：模拟I2C的SDA管脚配置为输出函数
****************************************************************************************************************/
void SDA_SetDataOut(void);
/***************************************************************************************************************
*函数名：SDA_SetDataIn()
*参数  :  void
*返回值： void
*函数功能：模拟I2C的SDA管脚配置为输入函数
****************************************************************************************************************/
void SDA_SetDataIn(void);
/***************************************************************************************************************
*函数名：SI2C_Start()
*参数  :  void
*返回值： void
*函数功能：模拟I2C启动总线函数
****************************************************************************************************************/
void SI2C_Start(void);
/***************************************************************************************************************
*函数名：SI2C_Stop()
*参数  :  void
*返回值： void
*函数功能：模拟I2C停止总线函数
****************************************************************************************************************/
void SI2C_Stop(void);
/***************************************************************************************************************
*函数名：SI2C_SendACK()
*参数  :  unsigned char ack 发送应答位，0为有应答，1为无应答
*返回值： void
*函数功能：模拟I2C发送应答位函数
****************************************************************************************************************/
void SI2C_SendACK(unsigned char ack);
/***************************************************************************************************************
*函数名：SI2C_RecvACK()
*参数  :  void
*返回值： unsigned char 1为无应答，0为有应答
*函数功能：模拟I2C接受应答位函数
****************************************************************************************************************/
unsigned char SI2C_RecvACK(void);
/***************************************************************************************************************
*函数名：SI2C_SendByte()
*参数  :  unsigned char dat 发送的字节
*返回值： void
*函数功能：模拟I2C发送一个字节函数
****************************************************************************************************************/
void SI2C_SendByte(unsigned char dat);
/***************************************************************************************************************
*函数名：SI2C_RecvByte()
*参数  :  void
*返回值： unsigned char 读出的一个字节数据
*函数功能：模拟I2C接收一个字节函数
****************************************************************************************************************/
unsigned char SI2C_RecvByte(void);
/***************************************************************************************************************
*函数名：SI2C_WriteData()
*参数  :  unsigned char dev_addr, unsigned char reg_addr, unsigned char *pdata, unsigned char count 器件地址，寄存器地址，数据，数据个数
*返回值： void
*函数功能：模拟I2C写数据函数
****************************************************************************************************************/
void SI2C_WriteData(unsigned char dev_addr, unsigned char reg_addr, unsigned char *pdat, unsigned char count);
/***************************************************************************************************************
*函数名：SI2C_RecvByte()
*参数  :  unsigned char dev_addr, unsigned char reg_addr, unsigned char * pdat, unsigned char count 器件地址，寄存器地址，数据，数据个数
*返回值： unsigned char 读出的一个字节数据
*函数功能：模拟I2C接收数据函数
****************************************************************************************************************/
void SI2C_ReadData(unsigned char dev_addr, unsigned char reg_addr, unsigned char * pdat, unsigned char count);

#endif /*__SI2C_H*/
