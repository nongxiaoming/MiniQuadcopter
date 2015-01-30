#ifndef __IMU_H
#define __IMU_H

#include "config.h"

struct IMU
{

	//欧拉角表示的飞行器姿态
	Vector3f angle;
	
	Vector3f Gyro, Acc, Acc_lpf; 

	LPF2ndData_t Acc_lpf_2nd;

	float magHold;
	
  Vector3f V_error_I;
};

void IMU_Init(void);
	
	//更新传感器数据
void IMU_UpdateSensor(void);	
	
	//计算飞行器姿态
void IMU_GetAttitude(void);

extern struct IMU imu;

#endif

