/******************** (C) COPYRIGHT 2014 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：ANO_Param.cpp
 * 描述    ：参数读取和保存
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "params.h"
#include "pidctrl.h"
#include "sensor.h"
#include "string.h"


#define PARAMS_SAVE_ADDRESS    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
#define PARAMS_SAVE_Sector      FLASH_Sector_11

static params_t params;
static bool Params_Read(void);
	 
void Params_Init(void)
{

	 if(Params_Read()==true)
   {
	   fc.pid_group[PIDROLL].kD = params.roll_pid.kd;
		 fc.pid_group[PIDROLL].kI = params.roll_pid.ki;
		 fc.pid_group[PIDROLL].kP = params.roll_pid.kp;
		 
		 fc.pid_group[PIDPITCH].kD = params.pitch_pid.kd;
		 fc.pid_group[PIDPITCH].kI = params.pitch_pid.ki;
		 fc.pid_group[PIDPITCH].kP = params.pitch_pid.kp;
		 
		 fc.pid_group[PIDYAW].kD = params.yaw_pid.kd;
		 fc.pid_group[PIDYAW].kI = params.yaw_pid.ki;
		 fc.pid_group[PIDYAW].kP = params.yaw_pid.kp;
		 
		 fc.pid_group[PIDALT].kD = params.alt_pid.kd;
		 fc.pid_group[PIDALT].kI = params.alt_pid.ki;
		 fc.pid_group[PIDALT].kP = params.alt_pid.kp;
		 
		 fc.pid_group[PIDLEVEL].kD = params.level_pid.kd;
		 fc.pid_group[PIDLEVEL].kI = params.level_pid.ki;
		 fc.pid_group[PIDLEVEL].kP = params.level_pid.kp;
		 
		 fc.pid_group[PIDMAG].kD = params.mag_pid.kd;
		 fc.pid_group[PIDMAG].kI = params.mag_pid.ki;
		 fc.pid_group[PIDMAG].kP = params.mag_pid.kp;

     sensor.Acc_Offset = params.acc_offset;
		 sensor.Gyro_Offset = params.gyro_offset;

	 }else
   {
		 params.roll_pid.kd = fc.pid_group[PIDROLL].kD ;
		 params.roll_pid.ki = fc.pid_group[PIDROLL].kI ;
		 params.roll_pid.kp = fc.pid_group[PIDROLL].kP;
		 
		 params.pitch_pid.kd = fc.pid_group[PIDPITCH].kD ;
		 params.pitch_pid.ki =  fc.pid_group[PIDPITCH].kI ;
		 params.pitch_pid.kp = fc.pid_group[PIDPITCH].kP ;
		 
		 params.yaw_pid.kd = fc.pid_group[PIDYAW].kD ;
		 params.yaw_pid.ki = fc.pid_group[PIDYAW].kI ;
		 params.yaw_pid.kp = fc.pid_group[PIDYAW].kP ;
		 
		 params.alt_pid.kd = fc.pid_group[PIDALT].kD ;
		 params.alt_pid.ki = fc.pid_group[PIDALT].kI ;
		 params.alt_pid.kp = fc.pid_group[PIDALT].kP ;
		 
		 params.level_pid.kd = fc.pid_group[PIDLEVEL].kD ;
		 params.level_pid.ki = fc.pid_group[PIDLEVEL].kI ;
		 params.level_pid.kp = fc.pid_group[PIDLEVEL].kP;
		 
		 params.mag_pid.kd = fc.pid_group[PIDMAG].kD ;
		 params.mag_pid.ki = fc.pid_group[PIDMAG].kI;
		 params.mag_pid.kp = fc.pid_group[PIDMAG].kP ;

     params.acc_offset = sensor.Acc_Offset ;
		 params.gyro_offset = sensor.Gyro_Offset ;
		 
	   Params_Save();
	 }
	
}

void Params_setAccOffset(Vector3i offset)
{
 params.acc_offset = offset;
 sensor.Acc_Offset = offset;
}


void Params_setGyroOffset(Vector3i offset)
{
 params.gyro_offset = offset;
 sensor.Gyro_Offset = offset;
}


void Params_setRollPid(pid_t val)
{
params.roll_pid = val;
fc.pid_group[PIDROLL].kD = val.kd;
fc.pid_group[PIDROLL].kI = val.ki;
fc.pid_group[PIDROLL].kP = val.kp;
}


void Params_setPitchPid(pid_t val)
{
params.pitch_pid = val;
fc.pid_group[PIDPITCH].kD = val.kd;
fc.pid_group[PIDPITCH].kI = val.ki;
fc.pid_group[PIDPITCH].kP = val.kp;
}

void Params_setYawPid(pid_t val)
{
params.yaw_pid = val;
fc.pid_group[PIDYAW].kD = val.kd;
fc.pid_group[PIDYAW].kI = val.ki;
fc.pid_group[PIDYAW].kP = val.kp;
}
void Params_setAltPid(pid_t val)
{
params.alt_pid = val;
fc.pid_group[PIDALT].kD = val.kd;
fc.pid_group[PIDALT].kI = val.ki;
fc.pid_group[PIDALT].kP = val.kp;
}
void Params_setLevelPid(pid_t val)
{
params.level_pid = val;
fc.pid_group[PIDLEVEL].kD = val.kd;
fc.pid_group[PIDLEVEL].kI = val.ki;
fc.pid_group[PIDLEVEL].kP = val.kp;
}
void Params_setMagPid(pid_t val)
{
params.mag_pid = val;
fc.pid_group[PIDMAG].kD = val.kd;
fc.pid_group[PIDMAG].kI = val.ki;
fc.pid_group[PIDMAG].kP = val.kp;
}
bool Params_Read(void)
{
  
  memcpy(&params,(void*)PARAMS_SAVE_ADDRESS,sizeof(params_t));
	if(params.magic != MAGIC)
  {
	 return false;
	}
	return true;
}
void Params_Save(void)
{
	 uint32_t *data;
	 uint32_t address, address_end;
   params.magic = MAGIC;

    /* Unlock the Flash */
    FLASH_Unlock();

    data = (uint32_t *)&params;
    address = PARAMS_SAVE_ADDRESS;
    address_end = address + sizeof(params_t);
	  
	      /* Clear pending flags (if any) */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	 
	  FLASH_EraseSector(PARAMS_SAVE_Sector, VoltageRange_3);
	
    while(address < address_end)
    {
        FLASH_Status status;
        const uint32_t *data_read;

        data_read = (const uint32_t *)address;
        if(*data_read == *data)
        {
            address += 4;
            data++;
            continue;
        }

        __disable_irq();
        status = FLASH_ProgramWord(address, *data);
        __enable_irq();

        if (status == FLASH_COMPLETE)
        {
            address += 4;
            data++;
        }
        else
        {
            continue;
        }
    }
    FLASH_Lock();
}









