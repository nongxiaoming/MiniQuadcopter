#ifndef __SENSOR_H
#define __SENSOR_H

#include "params.h"
#include "AMath.h"

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400

#define MPU6050G_S250DPS            ((float)0.0076335f)  // 0.0087500 dps/LSB
#define MPU6050G_S500DPS            ((float)0.0152671f)  // 0.0175000 dps/LSB
#define MPU6050G_S2000DPS           ((float)0.0609756f)  // 0.0700000 dps/LSB
	
struct Sensor
{
	uint8_t Acc_CALIBRATED;
	uint8_t Gyro_CALIBRATED;
	Vector3i Acc_Offset,Gyro_Offset;
};

	//初始化Sensor
	void Sensor_Init(const char *dev_name);
	//读取加速度 角速度
	void Sensor_ReadData(void);
	//返回加速度的值
	Vector3f Sensor_GetAcc(void);
	//返回角速度的值
	Vector3f Sensor_GetGyro(void);
	//返回单位为度每秒的角速度
	Vector3f Sensor_GetGyro_in_dps(void);
extern struct Sensor sensor;


#endif
