#ifndef __SENSOR_H
#define __SENSOR_H

#include "params.h"
#include "AMath.h"

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400

#define MPU6050G_S250DPS            ((float)0.0076335f)  // 0.0087500 dps/LSB
#define MPU6050G_S500DPS            ((float)0.0152671f)  // 0.0175000 dps/LSB
#define MPU6050G_S2000DPS           ((float)0.0609756f)  // 0.0700000 dps/LSB

enum {
IDX_ACCEL_XOUT_H = 0,
IDX_ACCEL_XOUT_L,
IDX_ACCEL_YOUT_H,
IDX_ACCEL_YOUT_L,
IDX_ACCEL_ZOUT_H,
IDX_ACCEL_ZOUT_L,
IDX_TEMP_OUT_H,
IDX_TEMP_OUT_L,
IDX_GYRO_XOUT_H,
IDX_GYRO_XOUT_L,
IDX_GYRO_YOUT_H,
IDX_GYRO_YOUT_L,
IDX_GYRO_ZOUT_H,
IDX_GYRO_ZOUT_L,
BUFFER_SIZE,
};
typedef enum  { // clockwise rotation from board forward
	ORIENT_TOP_0DEG    = 0x00,
	ORIENT_TOP_90DEG   = 0x01,
	ORIENT_TOP_180DEG  = 0x02,
	ORIENT_TOP_270DEG  = 0x03,
	ORIENT_BOTTOM_0DEG  = 0x04,
	ORIENT_BOTTOM_90DEG  = 0x05,
	ORIENT_BOTTOM_180DEG  = 0x06,
	ORIENT_BOTTOM_270DEG  = 0x07,
}Orientation_t;
struct Sensor
{
	uint8_t Acc_CALIBRATED;
	uint8_t Gyro_CALIBRATED;
	Vector3i Acc_Offset,Gyro_Offset;
	Orientation_t orientation;
};

	//初始化Sensor
	void Sensor_Init(const char *dev_name,Orientation_t orient);
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
