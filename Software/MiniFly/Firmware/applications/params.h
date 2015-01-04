#ifndef __PARAMS_H
#define __PARAMS_H

#include "config.h"



typedef struct
{
	uint16_t kp;
	uint16_t ki;
	uint16_t kd;
}pid_t;

typedef struct
{
 uint32_t magic;
 Vector3i acc_offset;
 Vector3i gyro_offset;
 pid_t roll_pid;
 pid_t pitch_pid;
 pid_t yaw_pid;
 pid_t alt_pid;
 pid_t level_pid;
 pid_t mag_pid;
}params_t;

#define MAGIC 0x12345678


void Params_Init(void);
	void Params_setAccOffset(Vector3i offset);
  Vector3i Params_getAccOffset(void);
  void Params_setGyroOffset(Vector3i offset);
  Vector3i Params_getGyroOffset(void);

	void Params_setRollPid(pid_t val);

  void Params_setPitchPid(pid_t val);

  void Params_setYawPid(pid_t val);
  
  void Params_setAltPid(pid_t val);
	
  void Params_setLevelPid(pid_t val);

   void Params_setMagPid(pid_t val);
	 
  void Params_Save(void);


#endif

