/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_FlyControl.cpp
 * 描述    ：飞行控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "pidctrl.h"
#include "sensor.h"

pidctrl fc;


void pidctrl::Init()
{
	yawRate = 80;
	//重置PID参数
	PID_Reset();
	
  motor_dev = rt_device_find("motors");
	if(motor_dev == RT_NULL)
  {
	 rt_kprintf("can not find motors!\n");
		return;
	}
	rt_device_open(motor_dev,RT_DEVICE_FLAG_RDWR);
}
//重置PID参数
void pidctrl::PID_Reset(void)
{
	pid_group[PIDROLL].set_pid(70, 15, 120, 2000000);
	pid_group[PIDPITCH].set_pid(70, 30, 120, 2000000);
	pid_group[PIDYAW].set_pid(100, 50, 0, 2000000);
	pid_group[PIDLEVEL].set_pid(280, 0, 0, 0);
	pid_group[PIDMAG].set_pid(15, 0, 0, 0);
}

//飞行器姿态外环控制
void pidctrl::Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_ADC;
	
	//计算角度误差值
	errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.x * 10; 
	errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - imu.angle.y * 10; 
	
	//获取角速度
	Gyro_ADC = Sensor_GetGyro() / 4;
	
	//得到外环PID输出
	RateError[ROLL] = pid_group[PIDLEVEL].get_p(errorAngle[ROLL]) - Gyro_ADC.x;
	RateError[PITCH] = pid_group[PIDLEVEL].get_p(errorAngle[PITCH]) - Gyro_ADC.y;
	RateError[YAW] = ((int32_t)(yawRate) * rc.Command[YAW]) / 32 - Gyro_ADC.z;		
}

//飞行器姿态内环控制
void pidctrl::Attitude_Inner_Loop(void)
{
	int32_t PIDTerm[3];
	
	for(u8 i=0; i<3;i++)
	{
		//当油门低于检查值时积分清零
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)	
			pid_group[i].reset_I();
		
		//得到内环PID输出
		PIDTerm[i] = pid_group[i].get_pid(RateError[i], PID_INNER_LOOP_TIME);
	}
	
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300 - abs(rc.Command[YAW]), +300 + abs(rc.Command[YAW]));	
		
	//PID输出转为电机控制量
	Motors_Ctrl(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}	
void pidctrl::Motors_Ctrl(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	uint8_t i = 0;
	//六轴X型
	motorPWM[0] = throttle - pidTermRoll - pidTermPitch + pidTermYaw; //后右
	motorPWM[1] = throttle + pidTermRoll - pidTermPitch - pidTermYaw; //前右
	motorPWM[2] = throttle + pidTermRoll + pidTermPitch + pidTermYaw; //后左
	motorPWM[3] = throttle - pidTermRoll + pidTermPitch - pidTermYaw; //前左
	
	for (i = 0; i < MOTORS_NUM_MAX; i++) 
		motorPWM[i] = constrain_uint16(motorPWM[i], THROTTLE_MIN, THROTTLE_MAX);

	//如果未解锁，则将电机输出设置为最低
	if(!config.f.ARMED || rc.rawData[THROTTLE] < 1200)	
		for(i=0; i< MOTORS_NUM_MAX ; i++)
			motorPWM[i] = 1000;

	//写入电机PWM
	rt_device_write(motor_dev,0,motorPWM,MOTORS_NUM_MAX);
	
}	
void pidctrl::getMotorsPWM(uint16_t* pwm)
{
	for(u8 i=0; i< MOTORS_NUM_MAX ; i++)
   {
	 pwm[i] = motorPWM[i];
	 }
}
/************************ (C) COPYRIGHT 2014 ANO TECH *****END OF FILE**********************/
