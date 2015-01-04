#ifndef __PIDCTRL_H
#define __PIDCTRL_H

#include <rtthread.h>
#include "pid.h"
#include "drv_motors.h"

#define FLYANGLE_MAX 250   //最大飞行倾角25度
#define THROTTLE_MIN 1100  //油门最小值
#define THROTTLE_MAX 1900  //油门最大值

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDLEVEL,
    PIDMAG,
		PIDITEMS
};

class pidctrl
{

public:
	
	PID pid_group[PIDITEMS];
  
  //初始化
  void Init();
	//姿态外环控制
	void Attitude_Outter_Loop(void);

	//姿态内环控制
	void Attitude_Inner_Loop(void);
	
  //获取电机PWM值
  void getMotorsPWM(uint16_t* pwm);

private:
	
	uint8_t yawRate;
	int32_t RateError[3];
  uint16_t motorPWM[MOTORS_NUM_MAX];
  rt_device_t motor_dev;
	void PID_Reset(void);
  void Motors_Ctrl(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw);
};

extern pidctrl fc;

#endif























