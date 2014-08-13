#include "MS5611.h"
#include <math.h>

#define MS5611_ADDR             0xee //    0x77

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096

vs32  BaroAlt,BaroOffset;
uint32_t ms5611_ut;  // static result of temperature measurement
uint32_t ms5611_up;  // static result of pressure measurement
uint16_t ms5611_prom[PROM_NB];  // on-chip ROM
uint8_t t_rxbuf[3],p_rxbuf[3];

static u8 iic_data_to_write;

void MS5611_loop(void)
{
	static u8 MS5611_sta = 1 , ms5611_iic_ok = 0 , MS5611_cnt = 1;
	
	if(MS5611_cnt)	MS5611_cnt--;
	
	if(MS5611_cnt==0)
	{
		switch(MS5611_sta)
		{
			case 0:
					ms5611_iic_ok = ms5611_reset();
					if(ms5611_iic_ok)
					{
						MS5611_sta = 1;
						MS5611_cnt = 20;
					}
					break;
			case 1:
					ms5611_iic_ok = ms5611_start_t();
					if(ms5611_iic_ok)
					{
						MS5611_sta = 2;
						MS5611_cnt = 10;
					}
					break;
			case 2:
					ms5611_iic_ok = ms5611_read_adc_t();
					if(ms5611_iic_ok)
					{
						MS5611_sta = 3;
						MS5611_cnt = 1;
					}
					break;
			case 3:
					ms5611_iic_ok = ms5611_start_p();
					if(ms5611_iic_ok)
					{
						MS5611_sta = 4;
						MS5611_cnt = 10;
					}
					break;
			case 4:
					ms5611_iic_ok = ms5611_read_adc_p();
					if(ms5611_iic_ok)
					{
						MS5611_sta = 5;
						MS5611_cnt = 1;
					}
					break;
			case 5:
					ms5611_iic_ok = ms5611_calculate();
					if(ms5611_iic_ok)
					{
						MS5611_sta = 1;
						MS5611_cnt = 1;
					}
					break;
		}
	}
}

u8 ms5611_reset(void)
{
		iic_data_to_write = 1;
    return ANO_TC_I2C2_Write_Buf(MS5611_ADDR, CMD_RESET, 1, &iic_data_to_write);
}
static u8 ms5611_read_prom(void)
{
    uint8_t rxbuf[2] = { 0, 0 };
		u8 check = 0;
		
		for (u8 i = 0; i < PROM_NB; i++)
		{
			check += ANO_TC_I2C2_Read_Buf(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
			ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
		}
		
		if(check==PROM_NB)
			return 1;
		else
			return 0;
}
u8 MS5611_Init(void)
{
	if(ms5611_reset()==0)
		return 0;
	else if(ms5611_read_prom()==0)
		return 0;
	else if(ms5611_start_t()==0)
		return 0;
	else
		return 1;
}
u8 ms5611_read_adc_t(void)
{
    return ANO_TC_I2C2_Read_Int(MS5611_ADDR, CMD_ADC_READ, 3, t_rxbuf); // read ADC
    //ms5611_ut = (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}
u8 ms5611_read_adc_p(void)
{
    return ANO_TC_I2C2_Read_Int(MS5611_ADDR, CMD_ADC_READ, 3, p_rxbuf); // read ADC
    //ms5611_ut = (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}
u8 ms5611_start_t(void)
{
		iic_data_to_write = 1;
    return ANO_TC_I2C2_Write_Int(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1, &iic_data_to_write); // D2 (temperature) conversion start!
}
u8 ms5611_start_p(void)
{
		iic_data_to_write = 1;
    return ANO_TC_I2C2_Write_Int(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1, &iic_data_to_write); // D1 (pressure) conversion start!
}
#define BARO_CAL_CNT 50
vs32 sum_temp = 0;
u8   sum_cnt = 0;
u8 ms5611_calculate(void)
{
    int32_t temperature, off2 = 0, sens2 = 0, delt;
    int32_t pressure;
		
		ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
		ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
		
    int32_t dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
    int64_t off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
    int64_t sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);

    if (temperature < 2000) { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
		pressure = (int)((1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
    BaroAlt = pressure-BaroOffset;
		if(sum_cnt)
		{
			sum_cnt--;
			sum_temp += BaroAlt;
			if(sum_cnt==0)
				BaroOffset = sum_temp / BARO_CAL_CNT;
		}
		return 1;
}
vs32 MS5611_GetValue(void)
{
	return BaroAlt;
}
void MS5611_CalOffset(void)
{
	BaroOffset = 0;
	sum_temp = 0;
	sum_cnt = BARO_CAL_CNT;
}
