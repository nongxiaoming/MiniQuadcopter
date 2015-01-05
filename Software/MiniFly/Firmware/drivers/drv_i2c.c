#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include "drv_i2c.h"

#ifdef RT_USING_I2C_BITOPS

/* GPIO define
SCL: I2C1_SCL PB10
SDA: I2C1_SDA PB11
*/
#define GPIO_PORT_I2C_SCL   GPIOB
#define RCC_I2C_SCL         RCC_APB2Periph_GPIOB
#define PIN_I2C_SCL		    GPIO_Pin_6

#define GPIO_PORT_I2C_SDA   GPIOB
#define RCC_I2C_SDA         RCC_APB2Periph_GPIOB
#define PIN_I2C_SDA		    GPIO_Pin_7

static struct rt_i2c_bus_device i2c_device;

static void gpio_set_sda(void *data, rt_int32_t state)
{
    if (state)
    {
        GPIO_SetBits(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
    }
    else
    {
        GPIO_ResetBits(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
    }
}

static void gpio_set_scl(void *data, rt_int32_t state)
{
    if (state)
    {
        GPIO_SetBits(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
    }
    else
    {
        GPIO_ResetBits(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
    }
}

static rt_int32_t gpio_get_sda(void *data)
{
    return GPIO_ReadInputDataBit(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
}

static rt_int32_t gpio_get_scl(void *data)
{
    return GPIO_ReadInputDataBit(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
}

static void gpio_udelay(rt_uint32_t us)
{
    volatile rt_int32_t i;
    for (; us > 0; us--)
    {
        i = 50;
        while(i--);
    }
}

static const struct rt_i2c_bit_ops bit_ops =
{
    RT_NULL,
    gpio_set_sda,
    gpio_set_scl,
    gpio_get_sda,
    gpio_get_scl,

    gpio_udelay,

    4,
    0
};



void rt_hw_i2c_init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure; 
  RCC_APB2PeriphClockCmd(RCC_I2C_SCL | RCC_I2C_SDA , ENABLE );
  GPIO_InitStructure.GPIO_Pin =  PIN_I2C_SCL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
  GPIO_Init(GPIO_PORT_I2C_SCL, &GPIO_InitStructure);
  GPIO_SetBits(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
	
  GPIO_InitStructure.GPIO_Pin =  PIN_I2C_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIO_PORT_I2C_SDA, &GPIO_InitStructure);	
	GPIO_SetBits(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);

  rt_memset((void *)&i2c_device, 0, sizeof(struct rt_i2c_bus_device));
  i2c_device.priv = (void *)&bit_ops;
  rt_i2c_bit_add_bus(&i2c_device, "i2c1");

}

#endif
