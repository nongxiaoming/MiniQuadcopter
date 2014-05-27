/*
 * File      : drv_i2c.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-05-01     xiaonong      First version
 */
 
#include <rtthread.h>
#include <rtdevice.h>

#ifdef RT_USING_I2C

#include "drv_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"



struct stm32_i2c_bus
{
    struct rt_i2c_bus_device parent;
    I2C_TypeDef *I2C;
	  I2C_ModeTypeDef Mode;
	  I2C_InitTypeDef* I2C_InitStruct;
};

static rt_uint32_t stm32_i2c_start()
{
  return RT_EOK;
}
static rt_err_t stm32_i2c_stop()
{

    return RT_EOK;
}


static rt_size_t stm32_i2c_recv_bytes(I2C_TypeDef *I2Cx, struct rt_i2c_msg *msg)
{
   rt_size_t bytes = 0;
//    rt_size_t len = msg->len;
//    rt_uint32_t stat = 0;


    return bytes;
}


static rt_size_t stm32_i2c_send_bytes(I2C_TypeDef *I2Cx, struct rt_i2c_msg *msg)
{
    rt_size_t bytes = 0;
//    rt_size_t len = msg->len;
//    rt_uint32_t stat = 0;
    /* Make sure start bit is not active */


    return bytes;
}
static void i2c_set_clock(I2C_TypeDef *I2Cx, uint32_t clock)
{

}

static rt_uint32_t i2c_send_addr(struct stm32_i2c_bus *i2c_bus, struct rt_i2c_msg *msg)
{
    rt_uint16_t addr;
    rt_uint16_t flags = msg->flags;
   return 0;
}


static rt_size_t stm32_i2c_xfer(struct rt_i2c_bus_device *bus,
                              struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;
    rt_uint32_t stat = 0;
    struct stm_i2c_bus *stm_i2c = (struct stm_i2c_bus *)bus;
   
    return ret;
}


static const struct rt_i2c_bus_device_ops i2c_ops =
{
    stm32_i2c_xfer,
    RT_NULL,
    RT_NULL
};


static void i2c_gpio_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
#ifdef USE_I2C1
	{
		/* Enable I2C1 SCL and SDA Pin Clock */
  RCC_APB2PeriphClockCmd(I2C1_SCL_GPIO_CLK | I2C1_SDA_GPIO_CLK, ENABLE);
    
  /* Set GPIO frequency to 50MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* Select Output open-drain mode */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
                                    
  /* Initialize I2C1 SCL Pin */ 
  GPIO_InitStructure.GPIO_Pin = I2C1_SCL_GPIO_PIN;
  
  GPIO_Init(I2C1_SCL_GPIO_PORT, &GPIO_InitStructure);

  /* Initialize I2C1 SDA Pin */
  GPIO_InitStructure.GPIO_Pin = I2C1_SDA_GPIO_PIN;
  
  GPIO_Init(I2C1_SDA_GPIO_PORT, &GPIO_InitStructure); 		
		
		 /* Enable I2C1 DMA */
  RCC_AHBPeriphClockCmd(I2C1_DMA_CLK, ENABLE);
}
	#endif
#ifdef USE_I2C2
	{
	
	}
#endif
}
#ifdef USE_DMA
static void i2c_dma_init(void)
{
DMA_InitTypeDef  DMA_InitStructure;
#ifdef USE_I2C1
		 /* Enable I2C1 DMA */
  RCC_AHBPeriphClockCmd(I2C1_DMA_CLK, ENABLE);
  
  /* I2C1 Common Channel Configuration */
  DMA_InitStructure.DMA_BufferSize = 0xFFFF;
  DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  /* Select I2C1 DR Address register as DMA PeripheralBaseAddress */
  DMA_InitStructure.DMA_PeripheralBaseAddr = I2C1_DR;
        
    /* Select Memory to Peripheral transfer direction */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    
    /* Initialize I2C1 DMA Tx Channel */
    DMA_Init(I2C1_DMA_TX_Channel, &DMA_InitStructure);   

    /* Select Peripheral to Memory transfer direction */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    
    /* Initialize I2C1 DMA Rx Channel */
    DMA_Init(I2C1_DMA_RX_Channel, &DMA_InitStructure);   
#endif
	
}
#endif
void rt_hw_i2c_init(void)
{
 i2c_gpio_init();
 
#ifdef USE_DMA
	i2c_dma_init();
#endif
	
#ifdef USE_I2C1
	{

	}
#endif
#ifdef USE_I2C2
	{
	
	}
#endif
	

//    rt_memset((void *)&lpc_i2c1, 0, sizeof(struct lpc_i2c_bus));
//    lpc_i2c1.parent.ops = &i2c_ops;
//    lpc_i2c_register(LPC_I2C1, &lpc_i2c1, "i2c1");
}

/*================== I2C_Event_Handler ==================*/

#ifdef I2C_MASTER_MODE 
/**
  * @brief  Handles Master Start condition (SB) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval PASS or FAIL. 
  */
static uint32_t I2C_MASTER_START_Handle(struct stm32_i2c_bus *i2c_bus)
{
  #ifdef I2C_10BIT_ADDR_MODE  
  /* Declare local variable that contains Address Header */
  uint8_t I2CHeaderAddress = 0x00;
  #endif /* I2C_10BIT_ADDR_MODE */

  /* Reinitialize Timeout Value */
   i2c_bus->parent.timeout = I2C_TIMEOUT_DEFAULT;
  
  i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT"); 
  
  i2c_dbg("\n\rLOG : I2C Device Start Acknowledged"); 
  
  /* 判断是否为7 bit地址类型 */
  if (i2c_bus->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_7bit)
  {        
    i2c_dbg("\n\rLOG : I2C Device 7bit Address");
    
    /*发送I2C地址*/
    /* 如果是读操作 */
    if (i2c_bus->State == STATE_READY_RX)
    {

      /* 发送第一个字节，I2C地址和读标识 */ 
		  i2c_bus->I2C->DR=(uint8_t)((pDevInitStruct->pTransferRx->wAddr1) | I2C_OAR1_ADD0);			
      
      /* Update State to STATE_BUSY */
      i2c_bus->State = STATE_BUSY_RX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy RX");
    }    
    /* 如果是写操作 */
    else
    {        
       /* 发送第一个字节，I2C地址和写标识 */ 
		  i2c_bus->I2C->DR= (uint8_t)((pDevInitStruct->pTransferTx->wAddr1) & (~I2C_OAR1_ADD0));	
      /* Update State to STATE_BUSY */
      i2c_bus->State = STATE_BUSY_TX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy TX");
    }
    
    i2c_dbg("\n\rLOG : I2C Device Target Address Sent");
    
    /* Initialize Timeout value */
    pDevInitStruct->wTimeout = I2C_TIMEOUT_MIN + I2C_TIMEOUT_ADDR;             
  }  
 #ifdef I2C_10BIT_ADDR_MODE  
  /* 如果使用10 bit 地址模式 */
  else
  {  
    i2c_dbg("\n\rLOG : I2C Device 10bit Address");
    								      
    /* If Master run as receiver */
    if (i2c_bus->State == STATE_READY_RX)
    {
      /* Calculate RX Header Address  */ 
      I2CHeaderAddress = ((((pDevInitStruct->pTransferRx->wAddr1) & 0xFF00) >>7) | 0xF0);
    }    
    /* If Master run as Transmitter */
    else if (pDevInitStruct->State == STATE_READY_TX)
    {
      /* Calculate TX Header Address */ 
      I2CHeaderAddress = ((((pDevInitStruct->pTransferTx->wAddr1) & 0xFF00) >>7) | 0xF0); 
    }      
    /* If Master run as Receiver */
    else if (pDevInitStruct->State == STATE_BUSY_RX)
    {
      /* Calculate RX Header Address */ 
      I2CHeaderAddress = ((((pDevInitStruct->pTransferRx->wAddr1) & 0xFF00) >>7) | 0xF1);       
    }       
    
     /* 发送第一个字节，I2C地址和读写标识 */ 
		i2c_bus->I2C->DR=I2CHeaderAddress;
    
    i2c_dbg("\n\rLOG : I2C Device Target Header Sent "); 
    
    /* Initialize Timeout value */
    pDevInitStruct->wTimeout = I2C_TIMEOUT_MIN + I2C_TIMEOUT_ADD10;                 
  }   
 #endif /* I2C_10BIT_ADDR_MODE */
  
#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)\
 || defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
 #ifdef I2C_CLOSECOM_METHOD2  
  if ((pDevInitStruct->ProgModel == PROGMODEL_INTERRUPT) &&(pDevInitStruct->State == STATE_BUSY_RX) 
           && (pDevInitStruct->pTransferRx->wNumData == 2))
  {
    /* Activate POS bit */
    __I2C_HAL_ENABLE_POS(pDevInitStruct->Dev);
  }
 #endif /* 2C_CLOSECOM_METHOD2 */
#endif  
  return PASS;
}


/**
  * @brief  Handles Master address matched (ADDR) interrupt event. 
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval PASS or FAIL. 
  */
static uint32_t I2C_MASTER_ADDR_Handle(InitTypeDef* pDevInitStruct)
{   
  /* Initialize Timeout value (1 ms for each data to be sent/received) */
  if (pDevInitStruct->ProgModel != PROGMODEL_DMA)
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    pDevInitStruct->wTimeout = I2C_TIMEOUT_DEFAULT;                
  }  
  else if (pDevInitStruct->State == STATE_BUSY_TX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Tx mode */
    pDevInitStruct->wTimeout = I2C_TIMEOUT_MIN + pDevInitStruct->pTransferTx->wNumData;
  }  
  else if (pDevInitStruct->State == STATE_BUSY_RX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Rx mode */ 
    pDevInitStruct->wTimeout = I2C_TIMEOUT_MIN + pDevInitStruct->pTransferRx->wNumData;
  }  
  else
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    pDevInitStruct->wTimeout = I2C_TIMEOUT_DEFAULT;        
  }
  
  if ((pDevInitStruct->State == STATE_BUSY_RX) && (pDevInitStruct->ProgModel == PROGMODEL_INTERRUPT))
  {       
    /* Switch Programing Mode Enable DMA or IT Buffer */
    I2C_Enable_DMA_IT(pDevInitStruct, DIRECTION_RX);
  }
  
  
 #ifdef I2C_CLOSECOM_METHOD1
  __I2C_HAL_CLEAR_ADDR(pDevInitStruct->Dev); 
 #endif /* I2C_CLOSECOM_METHOD1 */
  
  /* If State is STATE_BUSY_RX and receiving one byte */  
  if ((pDevInitStruct->State == STATE_BUSY_RX) && (pDevInitStruct->pTransferRx->wNumData == 1))
  { 
    /* Disable Acknowledge */
    __I2C_HAL_DISABLE_ACK(pDevInitStruct->Dev);
    
 #ifdef I2C_CLOSECOM_METHOD2
    
  #ifdef USE_CRITICAL_CALLBACK
    /* Call Critical section Callback */
    EnterCriticalSection_UserCallback();  
  #endif /* USE_CRITICAL_CALLBACK */
    
    /* Clear ADDR Flag by reading SR1 then SR2 */
    __I2C_HAL_CLEAR_ADDR(pDevInitStruct->Dev); 
    
    /* Program Generation of Stop Condition */
    __I2C_HAL_STOP(pDevInitStruct->Dev);
    
  #ifdef USE_CCRITICAL_CALLBACK
    /* Call Critical section Callback */
    ExitCriticalSection_UserCallback();
  #endif /* USE_CRITICAL_CALLBACK */
 #else
    /* Program Generation of Stop Condition */
    __I2C_HAL_STOP(pDevInitStruct->Dev);
 #endif /* I2C_CLOSECOM_METHOD2 */
    
  }
 #ifdef I2C_CLOSECOM_METHOD2  
  else if ((pDevInitStruct->ProgModel == PROGMODEL_INTERRUPT) &&(pDevInitStruct->State == STATE_BUSY_RX) 
           && (pDevInitStruct->pTransferRx->wNumData == 2))
  {        
    /* Clear ADDR Flag by reading SR1 then SR2 */
    __I2C_HAL_CLEAR_ADDR(pDevInitStruct->Dev); 
    
    /* Disable Acknowledge */
    __I2C_HAL_DISABLE_ACK(pDevInitStruct->Dev);
  }
  else
  {
    /* Clear ADDR Flag by reading SR1 then SR2 */
    __I2C_HAL_CLEAR_ADDR(pDevInitStruct->Dev);
  }
 #endif /* I2C_CLOSECOM_METHOD2 */ 
  
#ifdef I2C_10BIT_ADDR_MODE
  /* If State is not STATE_BUSY */
  if (((pDevInitStruct->State & (STATE_READY_TX | STATE_READY_RX)) != 0) 
      && (pDevInitStruct->pI2C_Struct->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_10bit))
  {        
    /* If Master run as receiver */
    if (pDevInitStruct->State == STATE_READY_RX)
    {
      /* Update State to STATE_BUSY_RX */
      pDevInitStruct->State = STATE_BUSY_RX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy RX");
      
      /* Generate Repeated start bit  */
      __I2C_HAL_START(pDevInitStruct->Dev);
      
      /* Initialize Timeout value */
      pDevInitStruct->wTimeout = I2C_TIMEOUT_MIN + I2C_TIMEOUT_SB;          
    }
    
    /* If Master run as Transmitter */
    if  (pDevInitStruct->State == STATE_READY_TX)
    {
      /* Update State to STATE_BUSY_TX */
      pDevInitStruct->State = STATE_BUSY_TX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy TX");
    }
  }
  else if ((pDevInitStruct->wOptions & OPT_NO_MEM_ADDR) == 0)      
#endif /* I2C_10BIT_ADDR_MODE */
    
#ifndef I2C_10BIT_ADDR_MODE
    if ((pDevInitStruct->wOptions & OPT_NO_MEM_ADDR) == 0)
#endif  /* I2C_10BIT_ADDR_MODE */
      
      /* If OPT_NO_MEM_ADDR is not enabled */
    {
      /* If State is STATE_BUSY_TX */  
      if (pDevInitStruct->State == STATE_BUSY_TX)
      {         
        /* If 8 Bit register mode */
        if ((pDevInitStruct->wOptions & OPT_16BIT_REG) == 0)
        {
          /* Send Register Address */
          __I2C_HAL_SEND((pDevInitStruct->Dev), (uint8_t)((pDevInitStruct->pTransferTx->wAddr2)& 0x00FF)); 
          
          /* Wait until TXE flag is set */ 
          __I2C_TIMEOUT(__I2C_HAL_GET_TXE(pDevInitStruct->Dev), I2C_TIMEOUT_TXE);              
        }
        
#ifdef 16BIT_REG_OPTION
        /* If 16 Bit register mode */
        else
        {
          /* Send MSB Register Address */
          __I2C_HAL_SEND((pDevInitStruct->Dev), (uint8_t)(((pDevInitStruct->pTransferTx->wAddr2)& 0xFF00) >>8));  
          
          /* Wait until TXE flag is set */ 
          __I2C_TIMEOUT(__I2C_HAL_GET_TXE(pDevInitStruct->Dev), I2C_TIMEOUT_TXE); 
          
          /* Send LSB Register Address */
          __I2C_HAL_SEND((pDevInitStruct->Dev), (uint8_t)((pDevInitStruct->pTransferTx->wAddr2)& 0x00FF));  
          
          /* Wait until TXE flag is set */ 
          __I2C_TIMEOUT(__I2C_HAL_GET_TXE(pDevInitStruct->Dev), I2C_TIMEOUT_TXE); 
        }     
#endif /* 16BIT_REG_OPTION */
      }  
      
      /* Switch Programing Mode Enable DMA or IT Buffer */
      I2C_Enable_DMA_IT(pDevInitStruct, DIRECTION_TXRX);   
    }      
  return PASS;
}


 #ifdef I2C_10BIT_ADDR_MODE
/**
  * @brief  Handles Master 10bit address matched (ADD10) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval PASS or FAIL. 
  */
static uint32_t I2C_MASTER_ADD10_Handle(InitTypeDef* pDevInitStruct)
{ 
  /* Reinitialize Timeout Value */
  pDevInitStruct->wTimeout = I2C_TIMEOUT_DEFAULT;
  
  i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
  
  i2c_dbg("\n\rLOG : I2C Device Header Address Acknowledged");
  
  /* Send Address */
  /* If Master run as receiver */
  if (pDevInitStruct->State == STATE_READY_RX)
  {
    /* Send Slave Address */
    __I2C_HAL_SEND((pDevInitStruct->Dev), (uint8_t)(pDevInitStruct->pTransferRx->wAddr1));  
  }  
  /* If Master run as Transmitter */
  else if (pDevInitStruct->State == STATE_READY_TX)
  {
    /* Send Slave Address */
    __I2C_HAL_SEND((pDevInitStruct->Dev), (uint8_t)(pDevInitStruct->pTransferTx->wAddr1));        
  }
  
  i2c_dbg("\n\rLOG : I2C Device Target Address Sent");  
  
  /* Initialize Timeout value */
  pDevInitStruct->wTimeout = I2C_TIMEOUT_MIN + I2C_TIMEOUT_ADDR; 
  
  return PASS;
}
 #endif /* I2C_10BIT_ADDR_MODE */


 #ifdef I2C_IT_PROGMODEL
/**
  * @brief  Handles Master transmission (TXE) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval PASS or FAIL. 
  */
static uint32_t I2C_MASTER_TXE_Handle(InitTypeDef* pDevInitStruct)
{ 
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->ProgModel == PROGMODEL_INTERRUPT)
  {                   
    /* If Buffer end */
    if (pDevInitStruct->pTransferTx->wNumData != 0)
    {   
      /* Call TX UserCallback */
      I2C_TX_UserCallback(pDevInitStruct);
      
      /* Write Byte */
      __I2C_HAL_SEND((pDevInitStruct->Dev), (*(pDevInitStruct->pTransferTx->pbBuffer))); 
      
      /* Decrement remaining number of data */
      pDevInitStruct->pTransferTx->wNumData--;
      
      /* If Buffer end */
      if (pDevInitStruct->pTransferTx->wNumData != 0)
      {  
        /* Point to next data */
        pDevInitStruct->pTransferTx->pbBuffer++;      
      }
    }    
    else 
    {    
      /* Generate Stop Condition */
      __I2C_HAL_STOP(pDevInitStruct->Dev);
      
      i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
      
      i2c_dbg("\n\rLOG : I2C Device Generates Stop");
      
      i2c_dbg("\n\rLOG : I2C Device TX Complete");
      
      /* Disable EVENT Interrupt */
      __I2C_HAL_DISABLE_EVTIT(pDevInitStruct->Dev);
      
      i2c_dbg("\n\rLOG : I2C Device TX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __I2C_HAL_DISABLE_BUFIT(pDevInitStruct->Dev);
      
      i2c_dbg("\n\rLOG : I2C Device TX BUFF IT Disabled");
      
      /* Wait until BTF and TXE flags are reset */ 
      __I2C_TIMEOUT(!(__I2C_HAL_GET_EVENT(pDevInitStruct->Dev) & (I2C_SR1_BTF | I2C_SR1_TXE )), I2C_TIMEOUT_BUSY);
      
      /* Update State to STATE_READY */
      pDevInitStruct->State = STATE_READY; 
      
      /* Call TX Transfer complete Callback */
      I2C_TXTC_UserCallback(pDevInitStruct);       
    }        
  }
  
  return PASS;
}
 #endif /* I2C_IT_PROGMODEL */

 #if defined (I2C_IT_PROGMODEL) || defined (I2C_DMA_1BYTE_CASE)
/**
  * @brief  Handles Master reception (RXNE flag) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval PASS or FAIL. 
  */
static uint32_t I2C_MASTER_RXNE_Handle(InitTypeDef* pDevInitStruct)
{  
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->ProgModel == PROGMODEL_INTERRUPT)
  {  
 
 #ifdef I2C_CLOSECOM_METHOD1
    /* if Two bytes remaining for reception */
    if (pDevInitStruct->pTransferRx->wNumData == 2)
    {         
      /* Read Byte */
      *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
      
      /* Disable Acknowledge */
      __I2C_HAL_DISABLE_ACK(pDevInitStruct->Dev);
      
      /* Program Generation of Stop Condition */
      __I2C_HAL_STOP(pDevInitStruct->Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pTransferRx->wNumData--; 
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pTransferRx->pbBuffer++;      
    }
     /* if One byte remaining for reception */
    else if (pDevInitStruct->pTransferRx->wNumData == 1)
    {
      /* Read Byte */
      *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pTransferRx->wNumData--; 
      
      i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
      
      i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
      
      i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
    }      
 #endif /* I2C_CLOSECOM_METHOD1 */
    
 #ifdef I2C_CLOSECOM_METHOD2
    /* if less than 3 bytes remaining for reception */ 
    if (pDevInitStruct->pTransferRx->wNumData <= 3)
    {  
      /* One byte */
      if (pDevInitStruct->pTransferRx->wNumData == 1)
      {              
        /* Read Byte */
        *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pTransferRx->wNumData--; 
        
        i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
        
        i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* Two bytes */
      if (pDevInitStruct->pTransferRx->wNumData == 2)
      {           
        /* Disable Buffer interrupt */
        __I2C_HAL_DISABLE_BUFIT(pDevInitStruct->Dev);
        
        /* Wait until BTF flag is set */ 
        __I2C_TIMEOUT(__I2C_HAL_GET_BTF(pDevInitStruct->Dev), I2C_TIMEOUT_BTF); 
        
        /* Generate Stop Condition */
        __I2C_HAL_STOP(pDevInitStruct->Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pTransferRx->pbBuffer++;
        
        pDevInitStruct->pTransferRx->wNumData--; 
        
        /* Read Byte */
        *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
        
        /*Decrement remaining number of data */
        pDevInitStruct->pTransferRx->wNumData--; 
        
        /* Reset POS */
        __I2C_HAL_DISABLE_POS(pDevInitStruct->Dev);
        
        i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
        
        i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* 3 Last bytes */
      if (pDevInitStruct->pTransferRx->wNumData == 3)
      {
        /* Disable Buffer interrupt */
        __I2C_HAL_DISABLE_BUFIT(pDevInitStruct->Dev);
        
        /* Wait until BTF flag is set */ 
        __I2C_TIMEOUT(__I2C_HAL_GET_BTF(pDevInitStruct->Dev), I2C_TIMEOUT_BTF); 
        
        /* Program NACK Generation */
        __I2C_HAL_DISABLE_ACK(pDevInitStruct->Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pTransferRx->pbBuffer++;
        
        pDevInitStruct->pTransferRx->wNumData--; 
        
        /* Generate Stop Condition */
        __I2C_HAL_STOP(pDevInitStruct->Dev);        
        
        /* Read Byte */
        *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pTransferRx->pbBuffer++;
        
        pDevInitStruct->pTransferRx->wNumData--; 
        
        /* Wait until RXNE flag is set */ 
        __I2C_TIMEOUT(__I2C_HAL_GET_RXNE(pDevInitStruct->Dev), I2C_TIMEOUT_RXNE); 
        
        /* Read Byte */
        *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
        
        /* Decrement remaining number of data */
        pDevInitStruct->pTransferRx->wNumData--;   
        
        i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
        
        i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
      }          
    } 
 #endif /* I2C_CLOSECOM_METHOD2 */
    
    /* if bytes remaining for reception */ 
    else
    {
      /* Read Byte */
      *(pDevInitStruct->pTransferRx->pbBuffer) = __I2C_HAL_RECEIVE(pDevInitStruct->Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pTransferRx->pbBuffer++;
      
      pDevInitStruct->pTransferRx->wNumData--; 
      
      /* Call RX UserCallback */
      I2C_RX_UserCallback(pDevInitStruct);
    }
    
    /* If All data are received */
    if (pDevInitStruct->pTransferRx->wNumData == 0)
    {      
      i2c_dbg("\n\rLOG : I2C Device Nack and Stop Generated ");
      
      i2c_dbg("\n\rLOG : I2C Device RX Complete"); 
      
      /* Disable EVENT Interrupt */
      __I2C_HAL_DISABLE_EVTIT(pDevInitStruct->Dev);
      
      i2c_dbg("\n\rLOG : I2C Device RX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __I2C_HAL_DISABLE_BUFIT(pDevInitStruct->Dev);
      
      i2c_dbg("\n\rLOG : I2C Device RX BUFF IT Disabled");
      
      /* Clear BTF Flag */
      __I2C_HAL_CLEAR_BTF(pDevInitStruct->Dev);   
      
      /* If 1Byte DMA option is selected */
      if ((pDevInitStruct->wOptions & DMA_1BYTE_CASE) != 0)
      {
        /* Clear 1Byte DMA option from wOptions */
        pDevInitStruct->wOptions &= ~DMA_1BYTE_CASE;
        
        /* Change ProgModel to DMA */
        pDevInitStruct->ProgModel = PROGMODEL_DMA;
      }
      
      /* Wait until Busy flag is reset */ 
      __I2C_TIMEOUT(!(__I2C_HAL_GET_BUSY(pDevInitStruct->Dev)), I2C_TIMEOUT_BUSY);
      
      /* Enable ACK generation and disable POS */
      __I2C_HAL_ENABLE_ACK(pDevInitStruct->Dev);      
      __I2C_HAL_DISABLE_POS(pDevInitStruct->Dev);
      
      /* Update State to STATE_READY */
      pDevInitStruct->State = STATE_READY;
      
      /* Call RX Transfer complete Callback */
      I2C_RXTC_UserCallback(pDevInitStruct);
    }
  }  
  return PASS;
}
 #endif /* I2C_IT_PROGMODEL || I2C_DMA_1BYTE_CASE */
#endif /* I2C_MASTER_MODE */ 


/**
  * @brief  This function Configure I2C DMA and Interrupts before starting transfer phase.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @param  Direction : Transfer direction.
  * @retval PASS or FAIL. 
  */
static uint32_t I2C_Enable_DMA_IT (InitTypeDef* pDevInitStruct, DirectionTypeDef Direction)
{
  /* Switch the value of ProgModel */
  switch (pDevInitStruct->ProgModel)
  { 
    
#if defined (I2C_IT_PROGMODEL) || defined (I2C_DMA_1BYTE_CASE)
    /*----------------------------------------------------------------------------
    Interrupt mode : if ProgModel = PROGMODEL_INTERRUPT
    ---------------------------------------------------------------------------*/            
  case PROGMODEL_INTERRUPT:
   
    /* Enable BUFFER Interrupt*/
    __I2C_HAL_ENABLE_BUFIT(pDevInitStruct->Dev);
    
    i2c_dbg("\n\rLOG : I2C Device BUFF IT Enabled"); 
    
    return PASS;
#endif /* I2C_IT_PROGMODEL || I2C_DMA_1BYTE_CASE */
    
#ifdef I2C_DMA_PROGMODEL
    /*----------------------------------------------------------------------------
    DMA mode : if ProgModel = PROGMODEL_DMA
    ---------------------------------------------------------------------------*/      
    case PROGMODEL_DMA:
    
     /* Disable EVENT Interrupt */
     __I2C_HAL_DISABLE_EVTIT(pDevInitStruct->Dev);
    
     /* Enable DMA request */
     __I2C_HAL_ENABLE_DMAREQ(pDevInitStruct->Dev);
    
    /* If a data transmission will be performed */
    if ((pDevInitStruct->State == STATE_BUSY_TX) || (Direction == DIRECTION_TX))
    {
      /* Configure TX DMA Channels */
      I2C_HAL_DMATXConfig(pDevInitStruct->Dev, pDevInitStruct->pTransferTx, pDevInitStruct->wOptions);
      
      /* Disable DMA automatic NACK generation */
      __I2C_HAL_DISABLE_LAST(pDevInitStruct->Dev); 
    
      /* Enable TX DMA Channels */
      __I2C_HAL_ENABLE_DMATX(pDevInitStruct->Dev);
      
      i2c_dbg("\n\rLOG : I2C Device DMA TX Enabled");       
    }    
     /* If a data reception will be performed */
    else if ((pDevInitStruct->State == STATE_BUSY_RX) || (Direction == DIRECTION_RX))
    {
      /* Configure RX DMA Channels */
      I2C_HAL_DMARXConfig(pDevInitStruct->Dev, pDevInitStruct->pTransferRx, pDevInitStruct->wOptions);
      
      /* If Master Mode Selected */
      if(pDevInitStruct->Mode == MODE_MASTER )
      {
        /* Enable DMA automatic NACK generation */
        __I2C_HAL_ENABLE_LAST(pDevInitStruct->Dev);
      }
    
      /* Enable RX DMA Channels */
      __I2C_HAL_ENABLE_DMARX(pDevInitStruct->Dev);                  
    }
    
    return PASS; 
#endif /* I2C_DMA_PROGMODEL */
    
    /*----------------------------------------------------------------------------
    Default: return error and exit Write Operation
    ---------------------------------------------------------------------------*/      
  default:
    
    /* Update State to STATE_ERROR */
    pDevInitStruct->State = STATE_ERROR;
    
    i2c_dbg("\n\rERROR : I2C Device Error"); 
    
    /* exit function */
    return FAIL;
  }  
}

/**
  * @brief  Configure NVIC and interrupts used by I2C Device according to 
  *         enabled options
  * @param  Device : I2C Device instance.
  * @param  Options : I2C Transfer Options.
  * @retval None. 
  */
void I2C_HAL_ITInit(DevTypeDef Device, uint32_t Options)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
   
  /* Configure NVIC priority Group */ 
  HAL_NVICInit();
   
  /* Enable the IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  /* Configure NVIC for I2Cx EVT Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = I2C_IT_EVT_IRQn [Device] ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_EVT_PREPRIO[Device];
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_EVT_SUBPRIO[Device];
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* If I2C ERR Interrupt Option Bit not selected */ 
  if ((Options & OPT_I2C_ERRIT_DISABLE) == 0)    
  {
    /* Configure NVIC for I2Cx ERR Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_IT_ERR_IRQn [Device] ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_ERR_PREPRIO[Device];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_ERR_SUBPRIO[Device];
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable I2C Error Interrupts */
    __I2C_HAL_ENABLE_ERRIT(Device);
  }
  
#ifdef USE_I2C_DMA
    /* If one or more DMA TX Interrupt option Bits selected */
  if ((Options & OPT_I2C_DMA_TX_IT_MASK) != 0)    
  {      
    /* Configure NVIC for DMA TX channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn [Device] ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMATX_PREPRIO[Device];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMATX_SUBPRIO[Device];
    NVIC_Init(&NVIC_InitStructure);
    
    /* If DMA TX TC interrupt Option Bits Selected */
    if ((Options & OPT_DMATX_TCIT) != 0)
    {
      /* Enable DMA TX Channel TCIT  */
      __I2C_HAL_ENABLE_DMATX_TCIT(Device);

    }
    
    /* If DMA TX HT interrupt Option Bits Selected */
    if ((Options & OPT_DMATX_HTIT) != 0)
    {
      /* Enable DMA TX Channel HTIT  */    
       __I2C_HAL_ENABLE_DMATX_HTIT(Device);
    }
    
    /* If DMA TX TE interrupt Option Bits Selected */
    if ((Options & OPT_DMATX_TEIT) != 0)
    {
      /* Enable DMA TX Channel TEIT  */    
       __I2C_HAL_ENABLE_DMATX_TEIT(Device); 
    }
  }
  
  /* If one or more DMA RX interrupt option Bits selected */
  if ((Options & OPT_I2C_DMA_RX_IT_MASK) != 0)    
  {
    /* Configure NVIC for DMA RX channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn [Device] ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMARX_PREPRIO[Device];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMARX_SUBPRIO[Device];
    NVIC_Init(&NVIC_InitStructure);
    
    /* If DMA RX TC interrupt Option Bits Selected */
    if ((Options & OPT_DMARX_TCIT) != 0)
    {
      /* Enable DMA RX Channel TCIT  */
       __I2C_HAL_ENABLE_DMARX_TCIT(Device);  
    }
    
    /* If DMA RX HT interrupt Option Bits Selected */
    if ((Options & OPT_DMARX_HTIT) != 0)
    {
      /* Enable DMA RX Channel HTIT  */    
      __I2C_HAL_ENABLE_DMARX_HTIT(Device);  
    }
    
    /* If DMA RX TE interrupt Option Bits Selected */
    if ((Options & OPT_DMARX_TEIT) != 0)
    {
      /* Enable DMA RX Channel TEIT  */
      __I2C_HAL_ENABLE_DMARX_TEIT(Device);   
  }  
  }
#endif /* I2C_DMA_PROGMODEL */  
  
}
/*================== I2C_Interrupt_Handler ==================*/

/**
  * @brief  This function handles I2C interrupt request for preparing communication
  *         and for transfer phase in case of using Interrupt Programming Model.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval PASS. 
  */
uint32_t I2C_EV_IRQHandler( InitTypeDef* pDevInitStruct)
{     
  __IO uint16_t I2CFlagStatus = 0x0000;
  
  /* Read I2C1 Status Registers 1 and 2 */
  I2CFlagStatus = __I2C_HAL_GET_EVENT(pDevInitStruct->Dev); 
 
#ifdef I2C_MASTER_MODE
  /*----------------------------------------------------------------------------------------------*/
  /*---------------------------------- If Master Mode selected ----------------------------------*/
  if (pDevInitStruct->Mode == MODE_MASTER)
  { 
    /*----------------------------------------*/  
    /*------------- If SB event --------------*/
    if ((I2CFlagStatus & (uint16_t)I2C_EVT_SB ) != 0)
    {       
      return I2C_MASTER_START_Handle(pDevInitStruct);        
    } 
    
    /*----------------------------------------*/
    /*------------- If ADDR event ------------*/
    if((I2CFlagStatus & (uint16_t)I2C_EVT_ADDR ) != 0)
    {  
      return I2C_MASTER_ADDR_Handle(pDevInitStruct);              
    }
    
 #ifdef I2C_10BIT_ADDR_MODE
    /*----------------------------------------*/
    /*------------- If ADD10 event *----------*/
    if ((I2CFlagStatus & (uint16_t)I2C_EVT_ADD10) != 0)
    { 
      return I2C_MASTER_ADD10_Handle(pDevInitStruct);  
    }    
 #endif /* I2C_10BIT_ADDR_MODE */
    
 #ifdef USE_I2C_IT  
    /*----------------------------------------*/
    /*------------- If TXE event -------------*/
    if (((I2CFlagStatus & (uint16_t)I2C_EVT_TXE) != 0) && (pDevInitStruct->State == STATE_BUSY_TX))
    {  
      return I2C_MASTER_TXE_Handle(pDevInitStruct); 
    }
 #endif /* I2C_IT_PROGMODEL */
    
 #if defined (I2C_IT_PROGMODEL) || defined (I2C_DMA_1BYTE_CASE)    
    /*----------------------------------------*/
    /*------------- If RXNE event ------------*/
    if (((I2CFlagStatus & (uint16_t)I2C_EVT_RXNE) != 0) && (pDevInitStruct->State == STATE_BUSY_RX))
    { 
      return I2C_MASTER_RXNE_Handle(pDevInitStruct); 
    }      
 #endif /* I2C_IT_PROGMODEL || I2C_DMA_1BYTE_CASE */
  }
#endif /* I2C_MASTER_MODE */
 
#ifdef I2C_SLAVE_MODE  
  /*----------------------------------------------------------------------------------------------*/
  /*---------------------------------- If Slave Mode selected ------------------------------------*/
  if (pDevInitStruct->Mode == MODE_SLAVE)
  {  
    /*----------------------------------------*/        
    /*------------- If ADDR event ------------*/
    if ((I2CFlagStatus & (uint16_t)I2C_EVT_ADDR ) != 0)
    { 
      return I2C_SLAVE_ADDR_Handle(pDevInitStruct); 
    }    

 #ifdef I2C_IT_PROGMODEL    
    /*----------------------------------------*/
    /*------------- If TXE event -------------*/
    if ((I2CFlagStatus & (uint16_t)I2C_EVT_TXE) != 0)
    { 
      return I2C_SLAVE_TXE_Handle(pDevInitStruct); 
    }  
    
    /*----------------------------------------*/
    /*------------- If RXNE event ------------*/
    if ((I2CFlagStatus & (uint16_t)I2C_EVT_RXNE) != 0)
    { 
      return I2C_SLAVE_RXNE_Handle(pDevInitStruct); 
    }    
 #endif /* I2C_IT_PROGMODEL */
    
    /*----------------------------------------*/
    /*------------- If STOPF event ------------*/
    if ((I2CFlagStatus & (uint16_t)I2C_EVT_STOPF) != 0)
    { 
      return I2C_SLAVE_STOP_Handle(pDevInitStruct); 
    }
  }
#endif /* I2C_SLAVE_MODE */
  
  return RT_EOK;
}


/**
  * @brief  Allows to handle errors occurred during initialization or communication 
  *         in order to recover the correct communication status or call specific 
  *         user functions.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval RT_EOK. 
  */
rt_err_t I2C_ER_IRQHandler(InitTypeDef* pDevInitStruct)
{  
  /* If AF detected in Slave mode transmitter */
  if ((pDevInitStruct->Mode == MODE_SLAVE) && (pDevInitStruct->pTransferTx->wNumData == 0) &&
      ((pDevInitStruct->State == STATE_READY) || (pDevInitStruct->State == STATE_BUSY_TX)))
  {      
    /* Clear error flags that can be cleared by writing to SR register */
    __I2C_HAL_CLEAR_ERROR((pDevInitStruct->Dev));  
    
    /* If Interrupt Programming Model */
    if (pDevInitStruct->ProgModel == PROGMODEL_INTERRUPT)
    {  
#ifdef USE_I2C_IT  
      
      /* Disable EVENT Interrupt */
      __I2C_HAL_DISABLE_EVTIT(pDevInitStruct->Dev);
      
      i2c_dbg("\n\rLOG : I2C Device EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __I2C_HAL_DISABLE_BUFIT(pDevInitStruct->Dev);
      
      i2c_dbg("\n\rLOG : I2C Device BUFF IT Disabled"); 
      
      /* Wait until Busy flag is reset */ 
      __I2C_TIMEOUT(!(__I2C_HAL_GET_BUSY(pDevInitStruct->Dev)), I2C_TIMEOUT_BUSY);
      
      /* Update State to STATE_READY */
      pDevInitStruct->State = STATE_READY;      
#endif /* I2C_IT_PROGMODEL */
    }   
  }  
  else
  {
    /* Read Error Register and affect to wDevError */
    pDevInitStruct->wDevError = __I2C_HAL_GET_ERROR(pDevInitStruct->Dev);
    
    /* Set Device state to STATE_ERROR */
    pDevInitStruct->State = STATE_ERROR;
    
    i2c_dbg("\n\r\n\rERROR <I2C_ErrorHandler> : I2C Device Error"); 
    
    /* Clear error flags that can be cleared by writing to SR register */
    __I2C_HAL_CLEAR_ERROR((pDevInitStruct->Dev)); 
    
    /* If Bus error occurred ---------------------------------------------------*/
    if ((pDevInitStruct->wDevError & I2C_ERR_BERR) != 0)
    {      
      i2c_dbg("\n\rERROR : I2C Device BERR"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */
      __I2C_HAL_SWRST(pDevInitStruct->Dev);
      
      i2c_dbg("\n\r I2C Device Software reset"); 
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK
      /* Call Bus Error UserCallback */
      I2C_BERR_UserCallback(pDevInitStruct->Dev);    
#endif /* USE_MULTIPLE_ERROR_CALLBACK */
    }
    
    /* If Arbitration Loss error occurred --------------------------------------*/
    if ((pDevInitStruct->wDevError & I2C_ERR_ARLO) != 0)
    {
      i2c_dbg("\n\rERROR : I2C Device ARLO"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */    
      __I2C_HAL_SWRST(pDevInitStruct->Dev);
      
      i2c_dbg("\n\r I2C Device Software reset"); 
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Arbitration Lost UserCallback */ 
      I2C_ARLO_UserCallback(pDevInitStruct->Dev);  
#endif /* USE_MULTIPLE_ERROR_CALLBACK */    
    }
    
    /* If Overrun error occurred -----------------------------------------------*/
    if ((pDevInitStruct->wDevError & I2C_ERR_OVR) != 0)
    {
      i2c_dbg("\n\rERROR : I2C Device OVR");
      
      /* No I2C software reset is performed here in order to allow user to get back
      the last data received correctly */
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Overrun error UserCallback */
      I2C_OVR_UserCallback(pDevInitStruct->Dev);
#endif /* USE_MULTIPLE_ERROR_CALLBACK */    
    }
        
    /* If Acknowledge Failure error occurred -----------------------------------*/
    if ((pDevInitStruct->wDevError & I2C_ERR_AF) != 0)
    {        
      i2c_dbg("\n\rERROR : I2C Device AF"); 
      
      /* No I2C software reset is performed here in order to allow user to recover 
      communication */
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Acknowledge Failure UserCallback */
      I2C_AF_UserCallback(pDevInitStruct->Dev);  
#endif /* USE_MULTIPLE_ERROR_CALLBACK */   
      
    }   
        
    /* USE_SINGLE_ERROR_CALLBACK is defined in conf.h file */
#if defined(USE_SINGLE_ERROR_CALLBACK)  
    /* Call Error UserCallback */  
    I2C_ERR_UserCallback(pDevInitStruct->Dev, pDevInitStruct->wDevError);
#endif /* USE_SINGLE_ERROR_CALLBACK */
  }
  
  return PASS;
}


#ifdef USE_I2C_DMA
/**
  * @brief  Handle I2C DMA TX interrupt request when DMA programming Model is 
  *         used for data transmission. 
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval PASS. 
  */
uint32_t I2C_DMA_TX_IRQHandler(InitTypeDef* pDevInitStruct)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  pDevInitStruct->wTimeout = I2C_TIMEOUT_DEFAULT; 
  
  i2c_dbg("\n\r\n\rLOG <I2C_DMA_TX_IRQHandler> : I2C Device TX DMA ");
  
  /*------------- If TC interrupt ------------*/
  if((__I2C_HAL_GET_DMATX_TCIT(pDevInitStruct->Dev)) != 0)
  {  
    i2c_dbg("\n\rLOG : I2C Device TX Complete");
    
    /* Update remaining number of data */
    pDevInitStruct->pTransferTx->wNumData = 0;
    
    /* Call DMA TX TC UserCallback */
    I2C_DMATXTC_UserCallback(pDevInitStruct);
    
   
    /* If Master Mode selected */
    if (pDevInitStruct->Mode == MODE_MASTER) 
    {
 #ifdef I2C_MASTER_MODE 
      /* If DMA Normal mode */
      if ((pDevInitStruct->wOptions & OPT_DMATX_CIRCULAR) == 0)
      {  
        /* Disable DMA Request */
        __I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->Dev); 
        
        /* Wait until BTF flag is set */ 
        __I2C_TIMEOUT(__I2C_HAL_GET_BTF(pDevInitStruct->Dev), I2C_TIMEOUT_BTF);
        
        /* Generate Stop Condition */
        __I2C_HAL_STOP(pDevInitStruct->Dev);
        
        /* Wait until Busy flag is reset */         
        __I2C_TIMEOUT(!(__I2C_HAL_GET_BUSY(pDevInitStruct->Dev)), I2C_TIMEOUT_BUSY);
        
        /* Disable DMA Channel */                 
        __I2C_HAL_DISABLE_DMATX(pDevInitStruct->Dev);        
        
        /* Disable EVENT Interrupt */
        __I2C_HAL_DISABLE_EVTIT(pDevInitStruct->Dev);
        
        i2c_dbg("\n\rLOG : I2C Device Master TX DMA Disabled");
        
        /* Update State to STATE_READY */
        pDevInitStruct->State = STATE_READY; 
      }
 #endif /* I2C_MASTER_MODE */  
    } 
    /* If Slave Mode selected */
    else
    {
 #ifdef I2C_SLAVE_MODE    	      
      /* Disable DMA Request and Channel */
      __I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->Dev);      
      __I2C_HAL_DISABLE_DMATX(pDevInitStruct->Dev);      
      
      /* Disable EVENT Interrupt */
      __I2C_HAL_DISABLE_EVTIT(pDevInitStruct->Dev);
      
      /* No Stop Condition Generation option bit not selected */   
      if ((pDevInitStruct->wOptions & OPT_I2C_NOSTOP) == 0)
      {
        /* Wait until Busy flag is reset */ 
        __I2C_TIMEOUT(!(__I2C_HAL_GET_BUSY(pDevInitStruct->Dev)), I2C_TIMEOUT_BUSY);
      }
      else
      {
        /* Wait until AF flag is set */ 
        __I2C_TIMEOUT(__I2C_HAL_GET_AF(pDevInitStruct->Dev), I2C_TIMEOUT_BUSY);
        
        /* Clear error flags that can be cleared by writing to SR register */
        __I2C_HAL_CLEAR_ERROR((pDevInitStruct->Dev));        
      }
      
      i2c_dbg("\n\rLOG : I2C Device Slave TX DMA Disabled");
      
      /* Update State to STATE_READY */
      pDevInitStruct->State = STATE_READY; 
 #endif /* I2C_SLAVE_MODE */       
    }    
          
    /* Call TX TC UserCallback */
    I2C_TXTC_UserCallback(pDevInitStruct);     
  }
  /*------------- If HT interrupt ------------*/
  else if ((__I2C_HAL_GET_DMATX_HTIT(pDevInitStruct->Dev)) != 0)
  {         
    i2c_dbg("\n\rLOG : I2C Device TX DMA Half Transfer ");
    
    /* Call DMA TX HT UserCallback */
    I2C_DMATXHT_UserCallback(pDevInitStruct);
  }  
  /*------------- If TE interrupt ------------*/
  else if ((__I2C_HAL_GET_DMATX_TEIT(pDevInitStruct->Dev)) != 0)
  { 
    i2c_dbg("\n\rERROR : I2C Device TX DMA Transfer Error ");
    
    /* Update State to STATE_ERROR */
    pDevInitStruct->State = STATE_ERROR; 
    
    /* Update remaining number of data */
    pDevInitStruct->pTransferTx->wNumData = __I2C_HAL_DMATX_GET_CNDT(pDevInitStruct->Dev);
    
    /* Call DMA TX TE UserCallback */
    I2C_DMATXTE_UserCallback(pDevInitStruct); 
  }  
  
   /* Clear DMA Interrupt Flag */
    __I2C_HAL_CLEAR_DMATX_IT(pDevInitStruct->Dev);
  
  return PASS;
}

uint32_t I2C_DMA_RX_IRQHandler(InitTypeDef* pDevInitStruct)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  pDevInitStruct->wTimeout = I2C_TIMEOUT_DEFAULT; 
  
  i2c_dbg("\n\r\n\rLOG <I2C_DMA_RX_IRQHandler> : I2C Device RX DMA ");
  
  /*------------- If TC interrupt ------------*/
  if ((__I2C_HAL_GET_DMARX_TCIT(pDevInitStruct->Dev)) != 0)
  {   
    i2c_dbg("\n\rLOG : I2C Device RX Complete");
    
    /* Update remaining number of data */
    pDevInitStruct->pTransferRx->wNumData = 0;
       
    /* Call DMA RX TC UserCallback */
    I2C_DMARXTC_UserCallback(pDevInitStruct);
    
 #ifdef I2C_MASTER_MODE      
    /* If Master Mode selected */
    if ((pDevInitStruct->Mode == MODE_MASTER))
    { 
      /* If DMA Normal model */
      if ((pDevInitStruct->wOptions & OPT_DMARX_CIRCULAR) == 0)
      {         
        /* No Stop Condition Generation option bit not selected */   
        if ((pDevInitStruct->wOptions & OPT_I2C_NOSTOP) == 0)
        {
          /* Generate Stop Condition */
          __I2C_HAL_STOP(pDevInitStruct->Dev);
          
          /* Disable DMA Request and Channel */          
          __I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->Dev);
          
          /* Wait until Busy flag is reset */ 
          __I2C_TIMEOUT(!(__I2C_HAL_GET_BUSY(pDevInitStruct->Dev)), I2C_TIMEOUT_BUSY);
        }
        else
        {
          /* Disable DMA Request */          
          __I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->Dev);          
        } 
        
        /* Disable DMA Channel */
        __I2C_HAL_DISABLE_DMARX(pDevInitStruct->Dev);
        
        /* Disable EVENT Interrupt */
        __I2C_HAL_DISABLE_EVTIT(pDevInitStruct->Dev);
        
        /* Disable DMA automatic NACK generation */
        __I2C_HAL_DISABLE_LAST(pDevInitStruct->Dev);
        
        i2c_dbg("\n\rLOG : I2C Device Master RX DMA Disabled");
        
        /* Update State to STATE_READY */
        pDevInitStruct->State = STATE_READY; 
      }
      /* Call RX TC UserCallback */
      I2C_RXTC_UserCallback(pDevInitStruct);
    }
 #endif /* I2C_MASTER_MODE */ 
  }  
  /*------------- If HT interrupt ------------*/
  else if ((__I2C_HAL_GET_DMARX_HTIT(pDevInitStruct->Dev)) != 0)
  {   
    i2c_dbg("\n\rLOG : I2C Device RX DMA Half Transfer");
    
    /* Call DMA RX HT UserCallback */
    I2C_DMARXHT_UserCallback(pDevInitStruct);
  }  
  /*------------- If TE interrupt ------------*/
  else if ((__I2C_HAL_GET_DMARX_TEIT(pDevInitStruct->Dev)) != 0)
  {   
    i2c_dbg("\n\rERROR : I2C Device RX DMA Transfer Error ");
    
    /* Update State to STATE_ERROR */
    pDevInitStruct->State = STATE_ERROR; 
    
    /* Update remaining number of data */
    pDevInitStruct->pTransferRx->wNumData = __I2C_HAL_DMARX_GET_CNDT(pDevInitStruct->Dev);
    
    /* Call DMA RX TE UserCallback */
    I2C_DMARXTE_UserCallback(pDevInitStruct); 
  }
  
  /* Clear DMA Interrupt Flag */
  __I2C_HAL_CLEAR_DMARX_IT(pDevInitStruct->Dev);
  
  return PASS;
}
#endif /* I2C_DMA_PROGMODEL */
/*================== I2C1_IRQhandler ==================*/

#ifdef USE_I2C1

void I2C1_EV_IRQHandler(void)
{  
 /* Call the Common Event handler function */
 I2C_EV_IRQHandler(&I2C1_DevStructure);
}


/**
  * @brief  This function handles I2C1 Errors interrupt.
  * @param  void. 
  * @retval void. 
  */
void I2C1_ER_IRQHandler(void)
{
  i2c_dbg("\n\r\n\rLOG <I2C1_ER_IRQHandler> : I2C1 Device Error IT ");
  
  /* Call the Common Error handler function */
  I2C_ER_IRQHandler(&I2C1_DevStructure);  
}

 #ifdef USE_I2C_DMA
/**
  * @brief  This function handles I2C1 TX DMA interrupt request.
  * @param  void. 
  * @retval void. 
  */
void I2C1_DMA_TX_IRQHandler(void)
{
  /* Call the Common DMA TX handler function */
   I2C_DMA_TX_IRQHandler(&I2C1_DevStructure);
}


/**
  * @brief  This function handles I2C1 RX DMA interrupt request.
  * @param  void. 
  * @retval void. 
  */
void I2C1_DMA_RX_IRQHandler(void)
{
  /* Call the Common DMA RX handler function */
  I2C_DMA_RX_IRQHandler(&I2C1_DevStructure);
}
 #endif /* USE_I2C_DMA */
#endif /* USE_I2C1 */
#endif
