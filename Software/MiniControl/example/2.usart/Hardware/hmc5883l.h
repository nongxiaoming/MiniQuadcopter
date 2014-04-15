#ifndef __HMC5883L_H
#define __HMC5883L_H
#include "si2c.h"
#define DEVICE_IIC_ADDR    0x1E
#define DEVICE_RD_ADDR     0x3D
#define DEVICE_WR_ADDR     0x3C		//Ð´

#define RadToDeg 57.29578

#define CFG_A       0x0
#define CFG_B       0x1
#define MODE        0x2
#define X_M         0x3
#define X_L         0x4
#define Z_M         0x5
#define Z_L         0x6
#define Y_M         0x7
#define Y_L         0x8
#define STATUS      0x9
#define IDF_A       0xA
#define IDF_B       0xB
#define IDF_C       0xC

// ¼Ä´æÆ÷µØÖ·¶¨Òå
void InitCmp(void);

void ReadCmpOut(short * pdat, int *angle_int);
#endif /*__HMC5883L_H*/

