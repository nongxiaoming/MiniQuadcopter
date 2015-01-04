#include <rtthread.h>
#include <rtdevice.h>
#include <finsh.h>
#include "drv_mpu6050.h"


static void mpu6050_write_bit(rt_device_t dev,rt_uint8_t reg, rt_uint8_t bitNum, rt_uint8_t data);
static void mpu6050_write_bits(rt_device_t dev,rt_uint8_t reg, rt_uint8_t bitStart, rt_uint8_t length,rt_uint8_t data);
static void mpu6050_setClockSource(rt_device_t dev,rt_uint8_t source);
static void mpu6050_setFullScaleGyroRange(rt_device_t dev,rt_uint8_t range);
static void mpu6050_setFullScaleAccelRange(rt_device_t dev,rt_uint8_t range);
static void mpu6050_setSleepEnabled(rt_device_t dev,rt_uint8_t enabled);
static void mpu6050_setI2CMasterModeEnabled(rt_device_t dev,rt_uint8_t enabled);
static void mpu6050_setI2CBypassEnabled(rt_device_t dev,rt_uint8_t enabled);
static void mpu6050_setDLPF(rt_device_t dev,rt_uint8_t mode);

static rt_size_t mpu6050_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    struct rt_i2c_msg msgs[2];
    mpu6050_dev_t *mpu6500;
    rt_uint8_t read_reg = pos;

    if (dev->user_data == RT_NULL)
        return (0);

    mpu6500 = (mpu6050_dev_t *)dev->user_data;

    msgs[0].addr  = mpu6500->addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &read_reg;
    msgs[0].len   = 1;

    msgs[1].addr  = mpu6500->addr;
    msgs[1].flags = RT_I2C_RD; /* Read from slave */
    msgs[1].buf   = (rt_uint8_t *)buffer;
    msgs[1].len   = size;

    if (rt_i2c_transfer(mpu6500->i2c_bus, msgs, 2) == 2)
        return (size);
    else
        return (0);
}

static rt_size_t mpu6050_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    struct rt_i2c_msg msgs[2];
    mpu6050_dev_t *mpu6500;
    rt_uint8_t write_reg = pos;

    if (dev->user_data == RT_NULL)
        return (0);
    mpu6500 = (mpu6050_dev_t *)dev->user_data;

    msgs[0].addr  = mpu6500->addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf   = &write_reg;
    msgs[0].len   = 1;

    msgs[1].addr  = mpu6500->addr;
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msgs[1].buf   = (rt_uint8_t *)buffer;
    msgs[1].len   = size;

    if (rt_i2c_transfer(mpu6500->i2c_bus, msgs, 2) == 2)
        return (size);
    else
        return (0);

}

/**
 * The following is rt-thread device operating interface
 */
static rt_err_t mpu6050_init(rt_device_t dev)
{
  mpu6050_setSleepEnabled(dev,0); //进入工作状态
  rt_thread_delay(1);
	mpu6050_setClockSource(dev,MPU6050_CLOCK_PLL_XGYRO); //设置时钟  0x6b   0x01
	rt_thread_delay(1);
	mpu6050_setFullScaleGyroRange(dev,MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
	rt_thread_delay(1);
	mpu6050_setFullScaleAccelRange(dev,MPU6050_ACCEL_FS_8);	//加速度度最大量程 +-8G
	rt_thread_delay(1);
	mpu6050_setDLPF(dev,MPU6050_DLPF_BW_42);
	rt_thread_delay(1);
	mpu6050_setI2CMasterModeEnabled(dev,0);	 //不让MPU6050 控制AUXI2C
	rt_thread_delay(1);
	mpu6050_setI2CBypassEnabled(dev,1);	
	
    return RT_EOK;
}

static rt_err_t mpu6050_open(rt_device_t dev, rt_uint16_t oflag)
{

    if (dev->user_data == RT_NULL)
    {
        return RT_ERROR;
    }
    else
    {
        return RT_EOK;
    }
}

static rt_err_t mpu6050_close(rt_device_t dev)
{
    return RT_EOK;
}
static rt_err_t mpu6050_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    return RT_EOK;
}
void rt_hw_mpu6050_init(const char *i2c_bus_name, rt_uint16_t addr)
{
    rt_device_t device;
    mpu6050_dev_t *mpu6050;
    RT_ASSERT(i2c_bus_name != RT_NULL);
    mpu6050 = (mpu6050_dev_t *)rt_malloc(sizeof(mpu6050_dev_t));

    mpu6050->i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(i2c_bus_name);
    RT_ASSERT(mpu6050->i2c_bus != RT_NULL);
    mpu6050->addr = addr;


    device = (rt_device_t)rt_malloc(sizeof(struct rt_device));

    /* Register RTC device */
    device->type        = RT_Device_Class_Char;
    device->rx_indicate = RT_NULL;
    device->tx_complete = RT_NULL;
    device->init        = mpu6050_init;
    device->open        = mpu6050_open;
    device->close       = mpu6050_close;
    device->read        = mpu6050_read;
    device->write       = mpu6050_write;
    device->control     = mpu6050_control;
    device->user_data   = mpu6050;

    rt_device_register(device, "mpu6050", RT_DEVICE_FLAG_RDWR);
}
static void mpu6050_write_bit(rt_device_t dev,rt_uint8_t reg, rt_uint8_t bitNum, rt_uint8_t data)
{
	rt_uint8_t value;
	mpu6050_read(dev,reg,&value,1);
	value = (data != 0) ? (value | (1 << bitNum)) : (value & ~(1 << bitNum));
	mpu6050_write(dev,reg,&value,1);
}
static void mpu6050_write_bits(rt_device_t dev,rt_uint8_t reg, rt_uint8_t bitStart, rt_uint8_t length,rt_uint8_t data)
{
	rt_uint8_t value,mask;
	mpu6050_read(dev,reg,&value,1);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	value &= mask;
	value |= data;
	mpu6050_write(dev,reg,&value,1);
}
static void mpu6050_setClockSource(rt_device_t dev,rt_uint8_t source)
{
	mpu6050_write_bits(dev,MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);	
}

static void mpu6050_setFullScaleGyroRange(rt_device_t dev,rt_uint8_t range) 
{
	mpu6050_write_bits(dev,MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

static void mpu6050_setFullScaleAccelRange(rt_device_t dev,rt_uint8_t range) 
{
	mpu6050_write_bits(dev,MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

static void mpu6050_setSleepEnabled(rt_device_t dev,rt_uint8_t enabled) 
{
	mpu6050_write_bit(dev,MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

static void mpu6050_setI2CMasterModeEnabled(rt_device_t dev,rt_uint8_t enabled) 
{
	mpu6050_write_bit(dev,MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

static void mpu6050_setI2CBypassEnabled(rt_device_t dev,rt_uint8_t enabled) 
{
	mpu6050_write_bit(dev,MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

static void mpu6050_setDLPF(rt_device_t dev,rt_uint8_t mode)
{
	mpu6050_write_bits(dev,MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

rt_uint8_t mpu6050_test(void)
{
    rt_device_t mpu6050 = RT_NULL;
    rt_uint8_t mpu6050_id = 0;
    rt_uint8_t mpu_data[14];
    rt_uint16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    mpu6050 = rt_device_find("mpu6050");
    while (mpu6050 == RT_NULL)
    {
        rt_kprintf("can not find mpu6050!\n");
        rt_hw_mpu6050_init("i2c1", MPU6050_DEFAULT_ADDRESS);
        rt_thread_delay(5);
        mpu6050 = rt_device_find("mpu6050");
    }
    rt_device_open(mpu6050, RT_DEVICE_FLAG_RDONLY);
    mpu6050->read(mpu6050, MPU6050_RA_WHO_AM_I, &mpu6050_id, 1);
    rt_kprintf("mpu6050 id = %x\n", mpu6050_id);
    if (mpu6050_id != 0x68)
        return 0;
    while (1)
    {
        mpu6050->read(mpu6050, MPU6050_RA_ACCEL_XOUT_H, mpu_data, 14);
        acc_x = ((mpu_data[0] << 8) | mpu_data[1]);
        acc_y = ((mpu_data[2] << 8) | mpu_data[3]);
        acc_z = ((mpu_data[4] << 8) | mpu_data[5]);
        gyro_x = ((mpu_data[8] << 8) | mpu_data[9]);
        gyro_y = ((mpu_data[10] << 8) | mpu_data[11]);
        gyro_z = ((mpu_data[12] << 8) | mpu_data[13]);
        rt_kprintf("acc_xyz:%d,%d,%d\n", acc_x, acc_y, acc_z);
        rt_kprintf("gyro_xyz:%d,%d,%d\n", gyro_x, gyro_y, gyro_z);
        rt_thread_delay(100);
    }
    //return 0 ;
}
FINSH_FUNCTION_EXPORT(mpu6050_test, mpu6500 test);
