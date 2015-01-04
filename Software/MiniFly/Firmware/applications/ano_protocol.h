#ifndef __ANO_PROTOCOL_H
#define __ANO_PROTOCOL_H
#include "stdint.h"

typedef __packed struct   
{
 uint16_t magic;
 uint8_t cmd;
 uint8_t len;
}header_t;


#endif
