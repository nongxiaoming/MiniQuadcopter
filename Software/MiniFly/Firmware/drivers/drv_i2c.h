#ifndef __DRV_I2C_H
#define __DRV_I2C_H

#include "stm32f10x.h"

#define USE_I2C1
//#define USE_I2C2

#define USE_DMA

/*    -- Section 1 :                 **** Device IO Pins Selection ****
  
  Description: This section allows user to choose IO Pins for each device if possible (in accordance with 
               used product: some products have only one possibility for the IO pins).
               Each device instance (I2C1, I2C2 ..) has its specific defines: one for each Pin.
               For each device instance, you will change existing defines with adequate IO Pins and Port 
               ( Refer to Product Pin mapping in related datasheet). */
 
/* To configure SCL and SDA Pin change these defines with adequate value :
  
#define I2C1_SCL_GPIO_PORT         GPIOX                  (X : Name of the GPIO PORT  (A,B,C,....))     
#define I2C1_SCL_GPIO_CLK          RCC_APB2Periph_GPIOX   (X : Name of the GPIO PORT  (A,B,C,....))   
#define I2C1_SCL_GPIO_PIN          GPIO_Pin_X             (X : Pin number (1,2,3,....))   
  
#define I2C1_SDA_GPIO_PORT         GPIOX                  (X : Name of the GPIO PORT  (A,B,C,....)) 
#define I2C1_SDA_GPIO_CLK          RCC_APB2Periph_GPIOX   (X : Name of the GPIO PORT  (A,B,C,....))  
#define I2C1_SDA_GPIO_PIN          GPIO_Pin_X             (X : Pin number (1,2,3,....))                         */
   

/* IO Pins selection possibilities 
  
|--------|---------|--------------|-----------|-------------------------|
| Device | I2C PIN |   GPIO_PIN   | GPIO_PORT |         GPIO_CLK        |                                          
|--------|---------|--------------|-----------|-------------------------|
|        |         |  GPIO_Pin_6  |   GPIOB   |  RCC_APB2Periph_GPIOB   |                                          
|        |   SCL   |--------------|-----------|-------------------------|
|        |         |  GPIO_Pin_8  |   GPIOB   |  RCC_APB2Periph_GPIOB   |                                          
|  I2C1  |---------|--------------|-----------|-------------------------|
|        |         |  GPIO_Pin_7  |   GPIOB   |  RCC_APB2Periph_GPIOB   |                                          
|        |   SDA   |--------------|-----------|-------------------------|
|        |         |  GPIO_Pin_9  |   GPIOB   |  RCC_APB2Periph_GPIOB   |                                          
|--------|---------|--------------|-----------|-------------------------|
|        |   SCL   |  GPIO_Pin_10 |   GPIOB   |  RCC_APB2Periph_GPIOB   |                                          
|  I2C2  |---------|--------------|-----------|-------------------------|
|        |   SDA   |  GPIO_Pin_11 |   GPIOB   |  RCC_APB2Periph_GPIOB   |                                          
|--------|---------|--------------|-----------|-------------------------|    
  Note: For this family, there are only two possible configurations for I2C1:
    - Configuration 1: I2C1_SCL = PB06 and I2C1_SDA = PB07
    - Configurating 2: I2C1_SCL = PB08 and I2C1_SDA = PB09
   It is not possible to mix these configuration.
  
  */ 
  
  
/*----------- I2C1 Device -----------*/
/* For STM32F10x family there are two possible configurations of I2C1 pins:
         1- SCL --> Pin PB6 and SDA --> Pin PB7 
         2- SCL --> Pin PB8 and SDA --> Pin PB9 */
#ifdef USE_I2C1  
#define I2C1_SCL_GPIO_PORT         GPIOB       
#define I2C1_SCL_GPIO_CLK          RCC_APB2Periph_GPIOB 
#define I2C1_SCL_GPIO_PIN          GPIO_Pin_6
  
#define I2C1_SDA_GPIO_PORT         GPIOB       
#define I2C1_SDA_GPIO_CLK          RCC_APB2Periph_GPIOB 
#define I2C1_SDA_GPIO_PIN          GPIO_Pin_7 
#endif /* USE_I2C1 */
  
/*-----------I2C2 Device -----------*/
/* I2C2 Pins are defined as below :   
         1- SCL --> Pin PB10 and SDA --> Pin PB11 */
#ifdef USE_I2C2    
#define I2C2_SCL_GPIO_PORT         GPIOB       
#define I2C2_SCL_GPIO_CLK          RCC_APB2Periph_GPIOB 
#define I2C2_SCL_GPIO_PIN          GPIO_Pin_10
  
#define I2C2_SDA_GPIO_PORT         GPIOB       
#define I2C2_SDA_GPIO_CLK          RCC_APB2Periph_GPIOB 
#define I2C2_SDA_GPIO_PIN          GPIO_Pin_11 
#endif /* USE_I2C2 */

/*    -- Section 2 :           **** Device TX and RX DMA Channels Selection ****
  
  Description: This section allows user to choose TX and RX DMA Channels if possible (in accordance with 
               used product) for each device.
               Each device instance (I2C1, I2C2 ..) has its specific defines: one for DMA TX Channel and 
               another one for DMA RX Channel.
               For each device instance, you find all TX an RX DMA Channel possibilities ( Refer to Product
               Reference Manual).*/
 
/* DMA Channel selection possibilities 
  
|--------|---------|----------------|
| Device | Channel |  DMA Channel   |
|--------|---------|----------------|
|        |    TX   | DMA1_Channel6  |
|  I2C1  |---------|----------------|
|        |    RX   | DMA1_Channel7  |
|--------|---------|----------------|
|        |    TX   | DMA1_Channel4  |
|  I2C2  |---------|----------------|
|        |    RX   | DMA1_Channel5  |
|--------|---------|----------------|*/   
    
/* I2Cx TX and RX DMA channels for STM32F10x family are fixed */
  
/*----------- I2C1 Device -----------*/
#define I2C1_DMA_TX_Channel        DMA1_Channel6
#define I2C1_DMA_RX_Channel        DMA1_Channel7
  
/*----------- I2C2 Device -----------*/
#define I2C2_DMA_TX_Channel        DMA1_Channel4
#define I2C2_DMA_RX_Channel        DMA1_Channel5

/*========= I2C1 specific defines (GPIO, PINs, Clocks and DMA) =========*/   
  
#define I2C1_CLK                   RCC_APB1Periph_I2C1
#define I2C1_DR                    ((uint32_t)0x40005410)
  
#define I2C1_DMA                   DMA1
#define I2C1_DMA_CLK               RCC_AHBPeriph_DMA1 

/*========= I2C2 specific defines (GPIO, PINs, Clocks and DMA) =========*/   
  
#define I2C2_CLK                   RCC_APB1Periph_I2C2
#define I2C2_DR                    ((uint32_t)0x40005810)
  
#define I2C2_DMA                   DMA1
#define I2C2_DMA_CLK               RCC_AHBPeriph_DMA1 



#define I2C_MEM_1Byte			      1
#define I2C_MEM_2Bytes				  2


typedef enum
{
    I2C_Mode_Polling = 0x00,
    I2C_Mode_Interrupt = 0x01,
    I2C_Mode_DMA = 0x02
} I2C_ModeTypeDef;

//void I2C1_INIT();
//Status I2C_AcknowledgePolling(I2C_TypeDef* I2Cx ,uint8_t Addr);
//Status I2C_IORW(I2C_TypeDef* I2Cx, uint8_t* pBuffer, uint32_t NumByteToRead, uint16_t memAddr, uint8_t SlaveAddress , uint8_t MemType );

#endif
