#include "mpu6050.h"
void PMU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	SI2C_Start();                                                   //启动总线
	SI2C_SendByte(MPU6050_SLAVE_ADDRESS);										//写从器件地址
	SI2C_RecvACK();                                                  //等待应答
	SI2C_SendByte(reg_add);											            //写数据地址
	SI2C_RecvACK(); 
		SI2C_SendByte(reg_dat);									               //写数据
		SI2C_RecvACK();                                              //等待应答
	  SI2C_Stop();          
}
void MPU6050_Init(void)
{ 
	
//	while(MPU6050ReadID()!=0x68)
//		{
//			SI2C_GPIO_Config();
//		}
	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x03);	     //解除休眠状态
	PMU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x00);	    //陀螺仪采样率
	PMU6050_WriteReg(MPU6050_RA_CONFIG , 0x00);	
	PMU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x00);	  //配置加速度传感器
	PMU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
}
u8 MPU6050ReadID(void)
{
	unsigned char Re = 0;
    SI2C_ReadData(MPU6050_SLAVE_ADDRESS, MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
    return Re;
}
void MPU6050ReadAcc(short *accData)
{
    u8 buf[6];
    SI2C_ReadData(MPU6050_SLAVE_ADDRESS, MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}
void MPU6050ReadGyro(short *gyroData)
{
    u8 buf[6];
    SI2C_ReadData(MPU6050_SLAVE_ADDRESS, MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    SI2C_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}
