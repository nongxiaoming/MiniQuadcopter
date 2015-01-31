#include <rtthread.h>
#include "params.h"
#include "sensor.h"
#include "mpu6050.h"

struct Sensor  sensor;

static uint8_t mpu6050_buffer[14]; //接收数据缓存区
static Vector3f Acc_ADC,Gyro_ADC;
static Vector3f Gyro_dps;
static rt_device_t mpu6050 = RT_NULL;
	//加速度零偏矫正
static	void Sensor_CalOffsetAcc(void);
	//陀螺仪零偏矫正
static	void Sensor_CalOffsetGyro(void);
//MPU6050初始化，传入参数：采样率，低通滤波频率
void Sensor_Init(const char *dev_name,Orientation_t orient)
{
	//uint8_t default_filter;
  mpu6050 = rt_device_find(dev_name);
	if(mpu6050 == RT_NULL)
	{
	 rt_kprintf("can not find mpu6050 device!\n");
	 return ;
	}
	rt_device_open(mpu6050,RT_DEVICE_FLAG_RDWR);
	sensor.orientation = orient;
}

//读取加速度和角速度
void Sensor_ReadData(void)
{
	Vector3i accel_data,gyro_data;
	rt_device_read(mpu6050,MPU6050_RA_ACCEL_XOUT_H,mpu6050_buffer,sizeof(mpu6050_buffer));
    
	//MPU6050_ReadData(mpu6050_buffer);
	switch (sensor.orientation) {
		case ORIENT_TOP_0DEG:
			accel_data.y = - (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.x = (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.z = (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = - (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.x  = (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.z  = (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;
		case ORIENT_TOP_90DEG:
			accel_data.y = (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.x = (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.z = (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.x  = (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.z  = (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;
		case ORIENT_TOP_180DEG:
			accel_data.y = (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.x = - (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.z = (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.x  = - (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.z  = (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;
		case ORIENT_TOP_270DEG:
			accel_data.y = - (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.x = - (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.z = (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = - (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.x  = - (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.z  = (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;
		case ORIENT_BOTTOM_0DEG:
			accel_data.y = (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.x = (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.z = - (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.x  = (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.z  = - (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;
		case ORIENT_BOTTOM_90DEG:
			accel_data.y = - (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.x = (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.z = - (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = - (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.x  = (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.z  = - (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;
		case ORIENT_BOTTOM_180DEG:
			accel_data.y = - (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.x = - (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.z = - (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = - (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.x  = - (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.z  = - (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;
		case ORIENT_BOTTOM_270DEG:
			accel_data.y = (int16_t)(mpu6050_buffer[IDX_ACCEL_YOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_YOUT_L]);
			accel_data.x = - (int16_t)(mpu6050_buffer[IDX_ACCEL_XOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_XOUT_L]);
			accel_data.z = - (int16_t)(mpu6050_buffer[IDX_ACCEL_ZOUT_H] << 8 | mpu6050_buffer[IDX_ACCEL_ZOUT_L]);
			gyro_data.y  = (int16_t)(mpu6050_buffer[IDX_GYRO_YOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_YOUT_L]);
			gyro_data.x  = - (int16_t)(mpu6050_buffer[IDX_GYRO_XOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_XOUT_L]);
			gyro_data.z  = - (int16_t)(mpu6050_buffer[IDX_GYRO_ZOUT_H] << 8 | mpu6050_buffer[IDX_GYRO_ZOUT_L]);
			break;

		}
	accel_data.x = accel_data.x - sensor.Acc_Offset.x;  //加速度X轴
	accel_data.y = accel_data.y - sensor.Acc_Offset.y;  //加速度Y轴
	accel_data.z = accel_data.z - sensor.Acc_Offset.z;  //加速度Z轴
	
	gyro_data.x = gyro_data.x - sensor.Gyro_Offset.x;  //加速度X轴
	gyro_data.y = gyro_data.y - sensor.Gyro_Offset.y;  //加速度Y轴
	gyro_data.z = gyro_data.z - sensor.Gyro_Offset.z;  //加速度Z轴

 Acc_ADC((float)accel_data.x,(float)accel_data.y,(float)accel_data.z);
 Gyro_ADC((float)gyro_data.x,(float)gyro_data.y,(float)gyro_data.z);
	Sensor_CalOffsetGyro();
	Sensor_CalOffsetAcc();
}

Vector3f Sensor_GetAcc(void)
{
 return Acc_ADC;	
}

Vector3f Sensor_GetGyro(void)
{
 return Gyro_ADC;		
}

Vector3f Sensor_GetGyro_in_dps(void)
{
	Gyro_dps.x = radians(Gyro_ADC.x * MPU6050G_S2000DPS);   // dps
	Gyro_dps.y = radians(Gyro_ADC.y * MPU6050G_S2000DPS);   // dps
	Gyro_dps.z = radians(Gyro_ADC.z * MPU6050G_S2000DPS);   // dps	
	
	return Gyro_dps;
}

//加速度零偏矫正
static void Sensor_CalOffsetAcc(void)
{
	if(sensor.Acc_CALIBRATED)
		{
			static Vector3f	tempAcc;
			static uint16_t cnt_a=0;

			if(cnt_a==0)
			{
				sensor.Acc_Offset.x=0;
				sensor.Acc_Offset.y=0;
				sensor.Acc_Offset.z=0;
				tempAcc.x=tempAcc.y=tempAcc.z=0;
				cnt_a = 1;
				return;
			}			
			tempAcc.x += Acc_ADC.x;
			tempAcc.y += Acc_ADC.y;
			tempAcc.z += Acc_ADC.z;
			if(cnt_a == CALIBRATING_ACC_CYCLES)
			{
				sensor.Acc_Offset.x = tempAcc.x/cnt_a;
				sensor.Acc_Offset.y = tempAcc.y/cnt_a;
				sensor.Acc_Offset.z = tempAcc.z/cnt_a - ACC_1G;
				cnt_a = 0;
				sensor.Acc_CALIBRATED = 0;
				Params_setAccOffset(sensor.Acc_Offset);
				Params_Save();//保存数据
				return;
			}
			cnt_a++;		
		}	
	
}

//陀螺仪零偏矫正
static void Sensor_CalOffsetGyro(void)
{
	if(sensor.Gyro_CALIBRATED)
	{
		static Vector3f	tempGyro;
		static uint16_t cnt_g=0;
		if(cnt_g==0)
		{
			sensor.Gyro_Offset.x=0;
			sensor.Gyro_Offset.y=0;
			sensor.Gyro_Offset.z=0;
			tempGyro.x = tempGyro.y = tempGyro.z =0;
			cnt_g = 1;
			return;
		}
		tempGyro.x += Gyro_ADC.x;
		tempGyro.y += Gyro_ADC.y;
		tempGyro.z += Gyro_ADC.z;
		if(cnt_g == CALIBRATING_GYRO_CYCLES)
		{
			sensor.Gyro_Offset.x = tempGyro.x/cnt_g;
			sensor.Gyro_Offset.y = tempGyro.y/cnt_g;
			sensor.Gyro_Offset.z = tempGyro.z/cnt_g;
			cnt_g = 0;
			sensor.Gyro_CALIBRATED = 0;
			Params_setGyroOffset(sensor.Gyro_Offset);
			Params_Save();//保存数据
			return;
		}
		cnt_g++;
	}
}

