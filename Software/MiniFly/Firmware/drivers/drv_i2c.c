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
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_START_Handle(struct stm32_i2c_bus *i2c_bus)
{
  #ifdef CPAL_I2C_10BIT_ADDR_MODE  
  /* Declare local variable that contains Address Header */
  uint8_t I2CHeaderAddress = 0x00;
  #endif /* CPAL_I2C_10BIT_ADDR_MODE */

  /* Reinitialize Timeout Value */
   i2c_bus->parent.timeout = I2C_TIMEOUT_DEFAULT;
  
  i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT"); 
  
  i2c_dbg("\n\rLOG : I2C Device Start Acknowledged"); 
  
  /* If 7 bit Addressing Mode selected */
  if (i2c_bus->->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_7bit)
  {        
    i2c_dbg("\n\rLOG : I2C Device 7bit Address");
    
    /* Send Address */
    /* If Master run as receiver */
    if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
    {
      /* Send Slave address with bit0 set for read */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferRx->wAddr1) | I2C_OAR1_ADD0));  
      
      /* Update CPAL_State to CPAL_STATE_BUSY */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_RX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy RX");
    }    
    /* If Master run as Transmitter */
    else
    {
      /* Send Slave address with bit0 reset for write */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr1) & (~I2C_OAR1_ADD0)));        
      
      /* Update CPAL_State to CPAL_STATE_BUSY */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_TX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy TX");
    }
    
    i2c_dbg("\n\rLOG : I2C Device Target Address Sent");
    
    /* Initialize Timeout value */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_ADDR;             
  }  
 #ifdef CPAL_I2C_10BIT_ADDR_MODE  
  /* If 10 bit Addressing Mode selected */
  else
  {  
    i2c_dbg("\n\rLOG : I2C Device 10bit Address");
    								      
    /* If Master run as receiver */
    if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
    {
      /* Calculate RX Header Address  */ 
      I2CHeaderAddress = ((((pDevInitStruct->pCPAL_TransferRx->wAddr1) & 0xFF00) >>7) | 0xF0);
    }    
    /* If Master run as Transmitter */
    else if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
    {
      /* Calculate TX Header Address */ 
      I2CHeaderAddress = ((((pDevInitStruct->pCPAL_TransferTx->wAddr1) & 0xFF00) >>7) | 0xF0); 
    }      
    /* If Master run as Receiver */
    else if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX)
    {
      /* Calculate RX Header Address */ 
      I2CHeaderAddress = ((((pDevInitStruct->pCPAL_TransferRx->wAddr1) & 0xFF00) >>7) | 0xF1);       
    }       
    
     /* Send Header */ 
    __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), I2CHeaderAddress); 
    
    i2c_dbg("\n\rLOG : I2C Device Target Header Sent "); 
    
    /* Initialize Timeout value */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_ADD10;                 
  }   
 #endif /* CPAL_I2C_10BIT_ADDR_MODE */
  
#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)\
 || defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_XL) || defined (STM32F10X_CL)
 #ifdef CPAL_I2C_CLOSECOM_METHOD2  
  if ((pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT) &&(pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) 
           && (pDevInitStruct->pCPAL_TransferRx->wNumData == 2))
  {
    /* Activate POS bit */
    __CPAL_I2C_HAL_ENABLE_POS(pDevInitStruct->CPAL_Dev);
  }
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */
#endif  
  return CPAL_PASS;
}


/**
  * @brief  Handles Master address matched (ADDR) interrupt event. 
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_ADDR_Handle(CPAL_InitTypeDef* pDevInitStruct)
{   
  /* Initialize Timeout value (1 ms for each data to be sent/received) */
  if (pDevInitStruct->CPAL_ProgModel != CPAL_PROGMODEL_DMA)
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;                
  }  
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Tx mode */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + pDevInitStruct->pCPAL_TransferTx->wNumData;
  }  
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX)
  {
    /* Set 1ms timeout for each data transfer in case of DMA Rx mode */ 
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + pDevInitStruct->pCPAL_TransferRx->wNumData;
  }  
  else
  {
    /* Reinitialize Timeout Value to default (no timeout initiated) */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;        
  }
  
  if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) && (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT))
  {       
    /* Switch Programing Mode Enable DMA or IT Buffer */
    I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_RX);
  }
  
  
 #ifdef CPAL_I2C_CLOSECOM_METHOD1
  __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev); 
 #endif /* CPAL_I2C_CLOSECOM_METHOD1 */
  
  /* If CPAL_State is CPAL_STATE_BUSY_RX and receiving one byte */  
  if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) && (pDevInitStruct->pCPAL_TransferRx->wNumData == 1))
  { 
    /* Disable Acknowledge */
    __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
    
 #ifdef CPAL_I2C_CLOSECOM_METHOD2
    
  #ifdef USE_CPAL_CRITICAL_CALLBACK
    /* Call Critical section Callback */
    CPAL_EnterCriticalSection_UserCallback();  
  #endif /* USE_CPAL_CRITICAL_CALLBACK */
    
    /* Clear ADDR Flag by reading SR1 then SR2 */
    __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev); 
    
    /* Program Generation of Stop Condition */
    __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
    
  #ifdef USE_CPAL_CRITICAL_CALLBACK
    /* Call Critical section Callback */
    CPAL_ExitCriticalSection_UserCallback();
  #endif /* USE_CPAL_CRITICAL_CALLBACK */
 #else
    /* Program Generation of Stop Condition */
    __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */
    
  }
 #ifdef CPAL_I2C_CLOSECOM_METHOD2  
  else if ((pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT) &&(pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) 
           && (pDevInitStruct->pCPAL_TransferRx->wNumData == 2))
  {        
    /* Clear ADDR Flag by reading SR1 then SR2 */
    __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev); 
    
    /* Disable Acknowledge */
    __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
  }
  else
  {
    /* Clear ADDR Flag by reading SR1 then SR2 */
    __CPAL_I2C_HAL_CLEAR_ADDR(pDevInitStruct->CPAL_Dev);
  }
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */ 
  
#ifdef CPAL_I2C_10BIT_ADDR_MODE
  /* If CPAL_State is not CPAL_STATE_BUSY */
  if (((pDevInitStruct->CPAL_State & (CPAL_STATE_READY_TX | CPAL_STATE_READY_RX)) != 0) 
      && (pDevInitStruct->pCPAL_I2C_Struct->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_10bit))
  {        
    /* If Master run as receiver */
    if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
    {
      /* Update CPAL_State to CPAL_STATE_BUSY_RX */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_RX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy RX");
      
      /* Generate Repeated start bit  */
      __CPAL_I2C_HAL_START(pDevInitStruct->CPAL_Dev);
      
      /* Initialize Timeout value */
      pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_SB;          
    }
    
    /* If Master run as Transmitter */
    if  (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
    {
      /* Update CPAL_State to CPAL_STATE_BUSY_TX */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_TX; 
      
      i2c_dbg("\n\rLOG : I2C Device Busy TX");
    }
  }
  else if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_NO_MEM_ADDR) == 0)      
#endif /* CPAL_I2C_10BIT_ADDR_MODE */
    
#ifndef CPAL_I2C_10BIT_ADDR_MODE
    if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_NO_MEM_ADDR) == 0)
#endif  /* CPAL_I2C_10BIT_ADDR_MODE */
      
      /* If CPAL_OPT_NO_MEM_ADDR is not enabled */
    {
      /* If CPAL_State is CPAL_STATE_BUSY_TX */  
      if (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX)
      {         
        /* If 8 Bit register mode */
        if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_16BIT_REG) == 0)
        {
          /* Send Register Address */
          __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr2)& 0x00FF)); 
          
          /* Wait until TXE flag is set */ 
          __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE);              
        }
        
#ifdef CPAL_16BIT_REG_OPTION
        /* If 16 Bit register mode */
        else
        {
          /* Send MSB Register Address */
          __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(((pDevInitStruct->pCPAL_TransferTx->wAddr2)& 0xFF00) >>8));  
          
          /* Wait until TXE flag is set */ 
          __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE); 
          
          /* Send LSB Register Address */
          __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr2)& 0x00FF));  
          
          /* Wait until TXE flag is set */ 
          __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_TXE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_TXE); 
        }     
#endif /* CPAL_16BIT_REG_OPTION */
      }  
      
      /* Switch Programing Mode Enable DMA or IT Buffer */
      I2C_Enable_DMA_IT(pDevInitStruct, CPAL_DIRECTION_TXRX);   
    }      
  return CPAL_PASS;
}


 #ifdef CPAL_I2C_10BIT_ADDR_MODE
/**
  * @brief  Handles Master 10bit address matched (ADD10) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_ADD10_Handle(CPAL_InitTypeDef* pDevInitStruct)
{ 
  /* Reinitialize Timeout Value */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;
  
  i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
  
  i2c_dbg("\n\rLOG : I2C Device Header Address Acknowledged");
  
  /* Send Address */
  /* If Master run as receiver */
  if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
  {
    /* Send Slave Address */
    __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(pDevInitStruct->pCPAL_TransferRx->wAddr1));  
  }  
  /* If Master run as Transmitter */
  else if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_TX)
  {
    /* Send Slave Address */
    __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)(pDevInitStruct->pCPAL_TransferTx->wAddr1));        
  }
  
  i2c_dbg("\n\rLOG : I2C Device Target Address Sent");  
  
  /* Initialize Timeout value */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_ADDR; 
  
  return CPAL_PASS;
}
 #endif /* CPAL_I2C_10BIT_ADDR_MODE */


 #ifdef CPAL_I2C_IT_PROGMODEL
/**
  * @brief  Handles Master transmission (TXE) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_TXE_Handle(CPAL_InitTypeDef* pDevInitStruct)
{ 
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
  {                   
    /* If Buffer end */
    if (pDevInitStruct->pCPAL_TransferTx->wNumData != 0)
    {   
      /* Call TX UserCallback */
      CPAL_I2C_TX_UserCallback(pDevInitStruct);
      
      /* Write Byte */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (*(pDevInitStruct->pCPAL_TransferTx->pbBuffer))); 
      
      /* Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferTx->wNumData--;
      
      /* If Buffer end */
      if (pDevInitStruct->pCPAL_TransferTx->wNumData != 0)
      {  
        /* Point to next data */
        pDevInitStruct->pCPAL_TransferTx->pbBuffer++;      
      }
    }    
    else 
    {    
      /* Generate Stop Condition */
      __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
      
      i2c_dbg("\n\rLOG : I2C Device Generates Stop");
      
      i2c_dbg("\n\rLOG : I2C Device TX Complete");
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\rLOG : I2C Device TX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\rLOG : I2C Device TX BUFF IT Disabled");
      
      /* Wait until BTF and TXE flags are reset */ 
      __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_EVENT(pDevInitStruct->CPAL_Dev) & (I2C_SR1_BTF | I2C_SR1_TXE )), CPAL_I2C_TIMEOUT_BUSY);
      
      /* Update CPAL_State to CPAL_STATE_READY */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
      
      /* Call TX Transfer complete Callback */
      CPAL_I2C_TXTC_UserCallback(pDevInitStruct);       
    }        
  }
  
  return CPAL_PASS;
}
 #endif /* CPAL_I2C_IT_PROGMODEL */

 #if defined (CPAL_I2C_IT_PROGMODEL) || defined (CPAL_I2C_DMA_1BYTE_CASE)
/**
  * @brief  Handles Master reception (RXNE flag) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_RXNE_Handle(CPAL_InitTypeDef* pDevInitStruct)
{  
  /* If Interrupt Programming Model selected */
  if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
  {  
 
 #ifdef CPAL_I2C_CLOSECOM_METHOD1
    /* if Two bytes remaining for reception */
    if (pDevInitStruct->pCPAL_TransferRx->wNumData == 2)
    {         
      /* Read Byte */
      *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
      
      /* Disable Acknowledge */
      __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
      
      /* Program Generation of Stop Condition */
      __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->wNumData--; 
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->pbBuffer++;      
    }
     /* if One byte remaining for reception */
    else if (pDevInitStruct->pCPAL_TransferRx->wNumData == 1)
    {
      /* Read Byte */
      *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->wNumData--; 
      
      i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
      
      i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
      
      i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
    }      
 #endif /* CPAL_I2C_CLOSECOM_METHOD1 */
    
 #ifdef CPAL_I2C_CLOSECOM_METHOD2
    /* if less than 3 bytes remaining for reception */ 
    if (pDevInitStruct->pCPAL_TransferRx->wNumData <= 3)
    {  
      /* One byte */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 1)
      {              
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
        
        i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* Two bytes */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 2)
      {           
        /* Disable Buffer interrupt */
        __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF); 
        
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /*Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Reset POS */
        __CPAL_I2C_HAL_DISABLE_POS(pDevInitStruct->CPAL_Dev);
        
        i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
        
        i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
      }
      
      /* 3 Last bytes */
      if (pDevInitStruct->pCPAL_TransferRx->wNumData == 3)
      {
        /* Disable Buffer interrupt */
        __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF); 
        
        /* Program NACK Generation */
        __CPAL_I2C_HAL_DISABLE_ACK(pDevInitStruct->CPAL_Dev);
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);        
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Point to next data and Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
        
        pDevInitStruct->pCPAL_TransferRx->wNumData--; 
        
        /* Wait until RXNE flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_RXNE(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_RXNE); 
        
        /* Read Byte */
        *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
        
        /* Decrement remaining number of data */
        pDevInitStruct->pCPAL_TransferRx->wNumData--;   
        
        i2c_dbg("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        i2c_dbg("\n\rLOG : I2C Device RX Nack Programmed");
        
        i2c_dbg("\n\rLOG : I2C Device RX Stop Programmed");
      }          
    } 
 #endif /* CPAL_I2C_CLOSECOM_METHOD2 */
    
    /* if bytes remaining for reception */ 
    else
    {
      /* Read Byte */
      *(pDevInitStruct->pCPAL_TransferRx->pbBuffer) = __CPAL_I2C_HAL_RECEIVE(pDevInitStruct->CPAL_Dev);
      
      /* Point to next data and Decrement remaining number of data */
      pDevInitStruct->pCPAL_TransferRx->pbBuffer++;
      
      pDevInitStruct->pCPAL_TransferRx->wNumData--; 
      
      /* Call RX UserCallback */
      CPAL_I2C_RX_UserCallback(pDevInitStruct);
    }
    
    /* If All data are received */
    if (pDevInitStruct->pCPAL_TransferRx->wNumData == 0)
    {      
      i2c_dbg("\n\rLOG : I2C Device Nack and Stop Generated ");
      
      i2c_dbg("\n\rLOG : I2C Device RX Complete"); 
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\rLOG : I2C Device RX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\rLOG : I2C Device RX BUFF IT Disabled");
      
      /* Clear BTF Flag */
      __CPAL_I2C_HAL_CLEAR_BTF(pDevInitStruct->CPAL_Dev);   
      
      /* If 1Byte DMA option is selected */
      if ((pDevInitStruct->wCPAL_Options & CPAL_DMA_1BYTE_CASE) != 0)
      {
        /* Clear 1Byte DMA option from wCPAL_Options */
        pDevInitStruct->wCPAL_Options &= ~CPAL_DMA_1BYTE_CASE;
        
        /* Change ProgModel to DMA */
        pDevInitStruct->CPAL_ProgModel = CPAL_PROGMODEL_DMA;
      }
      
      /* Wait until Busy flag is reset */ 
      __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
      
      /* Enable ACK generation and disable POS */
      __CPAL_I2C_HAL_ENABLE_ACK(pDevInitStruct->CPAL_Dev);      
      __CPAL_I2C_HAL_DISABLE_POS(pDevInitStruct->CPAL_Dev);
      
      /* Update CPAL_State to CPAL_STATE_READY */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY;
      
      /* Call RX Transfer complete Callback */
      CPAL_I2C_RXTC_UserCallback(pDevInitStruct);
    }
  }  
  return CPAL_PASS;
}
 #endif /* CPAL_I2C_IT_PROGMODEL || CPAL_I2C_DMA_1BYTE_CASE */
#endif /* CPAL_I2C_MASTER_MODE */ 


/**
  * @brief  This function Configure I2C DMA and Interrupts before starting transfer phase.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @param  Direction : Transfer direction.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_Enable_DMA_IT (CPAL_InitTypeDef* pDevInitStruct, CPAL_DirectionTypeDef Direction)
{
  /* Switch the value of CPAL_ProgModel */
  switch (pDevInitStruct->CPAL_ProgModel)
  { 
    
#if defined (CPAL_I2C_IT_PROGMODEL) || defined (CPAL_I2C_DMA_1BYTE_CASE)
    /*----------------------------------------------------------------------------
    Interrupt mode : if CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT
    ---------------------------------------------------------------------------*/            
  case CPAL_PROGMODEL_INTERRUPT:
   
    /* Enable BUFFER Interrupt*/
    __CPAL_I2C_HAL_ENABLE_BUFIT(pDevInitStruct->CPAL_Dev);
    
    i2c_dbg("\n\rLOG : I2C Device BUFF IT Enabled"); 
    
    return CPAL_PASS;
#endif /* CPAL_I2C_IT_PROGMODEL || CPAL_I2C_DMA_1BYTE_CASE */
    
#ifdef CPAL_I2C_DMA_PROGMODEL
    /*----------------------------------------------------------------------------
    DMA mode : if CPAL_ProgModel = CPAL_PROGMODEL_DMA
    ---------------------------------------------------------------------------*/      
    case CPAL_PROGMODEL_DMA:
    
     /* Disable EVENT Interrupt */
     __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
    
     /* Enable DMA request */
     __CPAL_I2C_HAL_ENABLE_DMAREQ(pDevInitStruct->CPAL_Dev);
    
    /* If a data transmission will be performed */
    if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX) || (Direction == CPAL_DIRECTION_TX))
    {
      /* Configure TX DMA Channels */
      CPAL_I2C_HAL_DMATXConfig(pDevInitStruct->CPAL_Dev, pDevInitStruct->pCPAL_TransferTx, pDevInitStruct->wCPAL_Options);
      
      /* Disable DMA automatic NACK generation */
      __CPAL_I2C_HAL_DISABLE_LAST(pDevInitStruct->CPAL_Dev); 
    
      /* Enable TX DMA Channels */
      __CPAL_I2C_HAL_ENABLE_DMATX(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\rLOG : I2C Device DMA TX Enabled");       
    }    
     /* If a data reception will be performed */
    else if ((pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX) || (Direction == CPAL_DIRECTION_RX))
    {
      /* Configure RX DMA Channels */
      CPAL_I2C_HAL_DMARXConfig(pDevInitStruct->CPAL_Dev, pDevInitStruct->pCPAL_TransferRx, pDevInitStruct->wCPAL_Options);
      
      /* If Master Mode Selected */
      if(pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER )
      {
        /* Enable DMA automatic NACK generation */
        __CPAL_I2C_HAL_ENABLE_LAST(pDevInitStruct->CPAL_Dev);
      }
    
      /* Enable RX DMA Channels */
      __CPAL_I2C_HAL_ENABLE_DMARX(pDevInitStruct->CPAL_Dev);                  
    }
    
    return CPAL_PASS; 
#endif /* CPAL_I2C_DMA_PROGMODEL */
    
    /*----------------------------------------------------------------------------
    Default: return error and exit Write Operation
    ---------------------------------------------------------------------------*/      
  default:
    
    /* Update CPAL_State to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR;
    
    i2c_dbg("\n\rERROR : I2C Device Error"); 
    
    /* exit function */
    return CPAL_FAIL;
  }  
}

/**
  * @brief  Configure NVIC and interrupts used by I2C Device according to 
  *         enabled options
  * @param  Device : I2C Device instance.
  * @param  Options : I2C Transfer Options.
  * @retval None. 
  */
void CPAL_I2C_HAL_ITInit(CPAL_DevTypeDef Device, uint32_t Options)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
   
  /* Configure NVIC priority Group */ 
  CPAL_HAL_NVICInit();
   
  /* Enable the IRQ channel */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  /* Configure NVIC for I2Cx EVT Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = CPAL_I2C_IT_EVT_IRQn [Device] ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_EVT_PREPRIO[Device];
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_EVT_SUBPRIO[Device];
  NVIC_Init(&NVIC_InitStructure);
  
  
  /* If I2C ERR Interrupt Option Bit not selected */ 
  if ((Options & CPAL_OPT_I2C_ERRIT_DISABLE) == 0)    
  {
    /* Configure NVIC for I2Cx ERR Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = CPAL_I2C_IT_ERR_IRQn [Device] ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_ERR_PREPRIO[Device];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_ERR_SUBPRIO[Device];
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable I2C Error Interrupts */
    __CPAL_I2C_HAL_ENABLE_ERRIT(Device);
  }
  
#ifdef USE_I2C_DMA
    /* If one or more DMA TX Interrupt option Bits selected */
  if ((Options & CPAL_OPT_I2C_DMA_TX_IT_MASK) != 0)    
  {      
    /* Configure NVIC for DMA TX channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = CPAL_I2C_DMA_TX_IRQn [Device] ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMATX_PREPRIO[Device];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMATX_SUBPRIO[Device];
    NVIC_Init(&NVIC_InitStructure);
    
    /* If DMA TX TC interrupt Option Bits Selected */
    if ((Options & CPAL_OPT_DMATX_TCIT) != 0)
    {
      /* Enable DMA TX Channel TCIT  */
      __I2C_HAL_ENABLE_DMATX_TCIT(Device);

    }
    
    /* If DMA TX HT interrupt Option Bits Selected */
    if ((Options & CPAL_OPT_DMATX_HTIT) != 0)
    {
      /* Enable DMA TX Channel HTIT  */    
       __I2C_HAL_ENABLE_DMATX_HTIT(Device);
    }
    
    /* If DMA TX TE interrupt Option Bits Selected */
    if ((Options & CPAL_OPT_DMATX_TEIT) != 0)
    {
      /* Enable DMA TX Channel TEIT  */    
       __I2C_HAL_ENABLE_DMATX_TEIT(Device); 
    }
  }
  
  /* If one or more DMA RX interrupt option Bits selected */
  if ((Options & CPAL_OPT_I2C_DMA_RX_IT_MASK) != 0)    
  {
    /* Configure NVIC for DMA RX channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = CPAL_I2C_DMA_RX_IRQn [Device] ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_IT_DMARX_PREPRIO[Device];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_IT_DMARX_SUBPRIO[Device];
    NVIC_Init(&NVIC_InitStructure);
    
    /* If DMA RX TC interrupt Option Bits Selected */
    if ((Options & CPAL_OPT_DMARX_TCIT) != 0)
    {
      /* Enable DMA RX Channel TCIT  */
       __I2C_HAL_ENABLE_DMARX_TCIT(Device);  
    }
    
    /* If DMA RX HT interrupt Option Bits Selected */
    if ((Options & CPAL_OPT_DMARX_HTIT) != 0)
    {
      /* Enable DMA RX Channel HTIT  */    
      __I2C_HAL_ENABLE_DMARX_HTIT(Device);  
    }
    
    /* If DMA RX TE interrupt Option Bits Selected */
    if ((Options & CPAL_OPT_DMARX_TEIT) != 0)
    {
      /* Enable DMA RX Channel TEIT  */
      __I2C_HAL_ENABLE_DMARX_TEIT(Device);   
  }  
  }
#endif /* CPAL_I2C_DMA_PROGMODEL */  
  
}
/*================== I2C_Interrupt_Handler ==================*/

/**
  * @brief  This function handles I2C interrupt request for preparing communication
  *         and for transfer phase in case of using Interrupt Programming Model.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS. 
  */
uint32_t CPAL_I2C_EV_IRQHandler( CPAL_InitTypeDef* pDevInitStruct)
{     
  __IO uint16_t I2CFlagStatus = 0x0000;
  
  /* Read I2C1 Status Registers 1 and 2 */
  I2CFlagStatus = __CPAL_I2C_HAL_GET_EVENT(pDevInitStruct->CPAL_Dev); 
 
#ifdef CPAL_I2C_MASTER_MODE
  /*----------------------------------------------------------------------------------------------*/
  /*---------------------------------- If Master Mode selected ----------------------------------*/
  if (pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER)
  { 
    /*----------------------------------------*/  
    /*------------- If SB event --------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_SB ) != 0)
    {       
      return I2C_MASTER_START_Handle(pDevInitStruct);        
    } 
    
    /*----------------------------------------*/
    /*------------- If ADDR event ------------*/
    if((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_ADDR ) != 0)
    {  
      return I2C_MASTER_ADDR_Handle(pDevInitStruct);              
    }
    
 #ifdef I2C_10BIT_ADDR_MODE
    /*----------------------------------------*/
    /*------------- If ADD10 event *----------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_ADD10) != 0)
    { 
      return I2C_MASTER_ADD10_Handle(pDevInitStruct);  
    }    
 #endif /* CPAL_I2C_10BIT_ADDR_MODE */
    
 #ifdef USE_I2C_IT  
    /*----------------------------------------*/
    /*------------- If TXE event -------------*/
    if (((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_TXE) != 0) && (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX))
    {  
      return I2C_MASTER_TXE_Handle(pDevInitStruct); 
    }
 #endif /* CPAL_I2C_IT_PROGMODEL */
    
 #if defined (CPAL_I2C_IT_PROGMODEL) || defined (CPAL_I2C_DMA_1BYTE_CASE)    
    /*----------------------------------------*/
    /*------------- If RXNE event ------------*/
    if (((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_RXNE) != 0) && (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_RX))
    { 
      return I2C_MASTER_RXNE_Handle(pDevInitStruct); 
    }      
 #endif /* CPAL_I2C_IT_PROGMODEL || CPAL_I2C_DMA_1BYTE_CASE */
  }
#endif /* CPAL_I2C_MASTER_MODE */
 
#ifdef CPAL_I2C_SLAVE_MODE  
  /*----------------------------------------------------------------------------------------------*/
  /*---------------------------------- If Slave Mode selected ------------------------------------*/
  if (pDevInitStruct->CPAL_Mode == CPAL_MODE_SLAVE)
  {  
    /*----------------------------------------*/        
    /*------------- If ADDR event ------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_ADDR ) != 0)
    { 
      return I2C_SLAVE_ADDR_Handle(pDevInitStruct); 
    }    

 #ifdef CPAL_I2C_IT_PROGMODEL    
    /*----------------------------------------*/
    /*------------- If TXE event -------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_TXE) != 0)
    { 
      return I2C_SLAVE_TXE_Handle(pDevInitStruct); 
    }  
    
    /*----------------------------------------*/
    /*------------- If RXNE event ------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_RXNE) != 0)
    { 
      return I2C_SLAVE_RXNE_Handle(pDevInitStruct); 
    }    
 #endif /* CPAL_I2C_IT_PROGMODEL */
    
    /*----------------------------------------*/
    /*------------- If STOPF event ------------*/
    if ((I2CFlagStatus & (uint16_t)CPAL_I2C_EVT_STOPF) != 0)
    { 
      return I2C_SLAVE_STOP_Handle(pDevInitStruct); 
    }
  }
#endif /* CPAL_I2C_SLAVE_MODE */
  
  return RT_EOK;
}


/**
  * @brief  Allows to handle errors occurred during initialization or communication 
  *         in order to recover the correct communication status or call specific 
  *         user functions.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval RT_EOK. 
  */
rt_err_t I2C_ER_IRQHandler(CPAL_InitTypeDef* pDevInitStruct)
{  
  /* If AF detected in Slave mode transmitter */
  if ((pDevInitStruct->CPAL_Mode == CPAL_MODE_SLAVE) && (pDevInitStruct->pCPAL_TransferTx->wNumData == 0) &&
      ((pDevInitStruct->CPAL_State == CPAL_STATE_READY) || (pDevInitStruct->CPAL_State == CPAL_STATE_BUSY_TX)))
  {      
    /* Clear error flags that can be cleared by writing to SR register */
    __CPAL_I2C_HAL_CLEAR_ERROR((pDevInitStruct->CPAL_Dev));  
    
    /* If Interrupt Programming Model */
    if (pDevInitStruct->CPAL_ProgModel == CPAL_PROGMODEL_INTERRUPT)
    {  
#ifdef USE_I2C_IT  
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\rLOG : I2C Device EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\rLOG : I2C Device BUFF IT Disabled"); 
      
      /* Wait until Busy flag is reset */ 
      __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
      
      /* Update CPAL_State to CPAL_STATE_READY */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY;      
#endif /* CPAL_I2C_IT_PROGMODEL */
    }   
  }  
  else
  {
    /* Read Error Register and affect to wCPAL_DevError */
    pDevInitStruct->wCPAL_DevError = __CPAL_I2C_HAL_GET_ERROR(pDevInitStruct->CPAL_Dev);
    
    /* Set Device state to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR;
    
    i2c_dbg("\n\r\n\rERROR <CPAL_I2C_ErrorHandler> : I2C Device Error"); 
    
    /* Clear error flags that can be cleared by writing to SR register */
    __CPAL_I2C_HAL_CLEAR_ERROR((pDevInitStruct->CPAL_Dev)); 
    
    /* If Bus error occurred ---------------------------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_BERR) != 0)
    {      
      i2c_dbg("\n\rERROR : I2C Device BERR"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */
      __CPAL_I2C_HAL_SWRST(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\r I2C Device Software reset"); 
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK
      /* Call Bus Error UserCallback */
      CPAL_I2C_BERR_UserCallback(pDevInitStruct->CPAL_Dev);    
#endif /* USE_MULTIPLE_ERROR_CALLBACK */
    }
    
    /* If Arbitration Loss error occurred --------------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_ARLO) != 0)
    {
      i2c_dbg("\n\rERROR : I2C Device ARLO"); 
      
      /* Generate I2C software reset in order to release SDA and SCL lines */    
      __CPAL_I2C_HAL_SWRST(pDevInitStruct->CPAL_Dev);
      
      i2c_dbg("\n\r I2C Device Software reset"); 
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Arbitration Lost UserCallback */ 
      CPAL_I2C_ARLO_UserCallback(pDevInitStruct->CPAL_Dev);  
#endif /* USE_MULTIPLE_ERROR_CALLBACK */    
    }
    
    /* If Overrun error occurred -----------------------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_OVR) != 0)
    {
      i2c_dbg("\n\rERROR : I2C Device OVR");
      
      /* No I2C software reset is performed here in order to allow user to get back
      the last data received correctly */
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Overrun error UserCallback */
      CPAL_I2C_OVR_UserCallback(pDevInitStruct->CPAL_Dev);
#endif /* USE_MULTIPLE_ERROR_CALLBACK */    
    }
        
    /* If Acknowledge Failure error occurred -----------------------------------*/
    if ((pDevInitStruct->wCPAL_DevError & CPAL_I2C_ERR_AF) != 0)
    {        
      i2c_dbg("\n\rERROR : I2C Device AF"); 
      
      /* No I2C software reset is performed here in order to allow user to recover 
      communication */
      
#ifdef USE_MULTIPLE_ERROR_CALLBACK    
      /* Call Acknowledge Failure UserCallback */
      CPAL_I2C_AF_UserCallback(pDevInitStruct->CPAL_Dev);  
#endif /* USE_MULTIPLE_ERROR_CALLBACK */   
      
    }   
        
    /* USE_SINGLE_ERROR_CALLBACK is defined in cpal_conf.h file */
#if defined(USE_SINGLE_ERROR_CALLBACK)  
    /* Call Error UserCallback */  
    CPAL_I2C_ERR_UserCallback(pDevInitStruct->CPAL_Dev, pDevInitStruct->wCPAL_DevError);
#endif /* USE_SINGLE_ERROR_CALLBACK */
  }
  
  return CPAL_PASS;
}


#ifdef USE_I2C_DMA
/**
  * @brief  Handle I2C DMA TX interrupt request when DMA programming Model is 
  *         used for data transmission. 
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS. 
  */
uint32_t I2C_DMA_TX_IRQHandler(CPAL_InitTypeDef* pDevInitStruct)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT; 
  
  i2c_dbg("\n\r\n\rLOG <CPAL_I2C_DMA_TX_IRQHandler> : I2C Device TX DMA ");
  
  /*------------- If TC interrupt ------------*/
  if((__CPAL_I2C_HAL_GET_DMATX_TCIT(pDevInitStruct->CPAL_Dev)) != 0)
  {  
    i2c_dbg("\n\rLOG : I2C Device TX Complete");
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferTx->wNumData = 0;
    
    /* Call DMA TX TC UserCallback */
    CPAL_I2C_DMATXTC_UserCallback(pDevInitStruct);
    
   
    /* If Master Mode selected */
    if (pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER) 
    {
 #ifdef CPAL_I2C_MASTER_MODE 
      /* If DMA Normal mode */
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_DMATX_CIRCULAR) == 0)
      {  
        /* Disable DMA Request */
        __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev); 
        
        /* Wait until BTF flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_BTF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BTF);
        
        /* Generate Stop Condition */
        __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
        
        /* Wait until Busy flag is reset */         
        __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
        
        /* Disable DMA Channel */                 
        __CPAL_I2C_HAL_DISABLE_DMATX(pDevInitStruct->CPAL_Dev);        
        
        /* Disable EVENT Interrupt */
        __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
        
        i2c_dbg("\n\rLOG : I2C Device Master TX DMA Disabled");
        
        /* Update CPAL_State to CPAL_STATE_READY */
        pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
      }
 #endif /* CPAL_I2C_MASTER_MODE */  
    } 
    /* If Slave Mode selected */
    else
    {
 #ifdef CPAL_I2C_SLAVE_MODE    	      
      /* Disable DMA Request and Channel */
      __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev);      
      __CPAL_I2C_HAL_DISABLE_DMATX(pDevInitStruct->CPAL_Dev);      
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      /* No Stop Condition Generation option bit not selected */   
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP) == 0)
      {
        /* Wait until Busy flag is reset */ 
        __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
      }
      else
      {
        /* Wait until AF flag is set */ 
        __CPAL_I2C_TIMEOUT(__CPAL_I2C_HAL_GET_AF(pDevInitStruct->CPAL_Dev), CPAL_I2C_TIMEOUT_BUSY);
        
        /* Clear error flags that can be cleared by writing to SR register */
        __CPAL_I2C_HAL_CLEAR_ERROR((pDevInitStruct->CPAL_Dev));        
      }
      
      i2c_dbg("\n\rLOG : I2C Device Slave TX DMA Disabled");
      
      /* Update CPAL_State to CPAL_STATE_READY */
      pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
 #endif /* CPAL_I2C_SLAVE_MODE */       
    }    
          
    /* Call TX TC UserCallback */
    CPAL_I2C_TXTC_UserCallback(pDevInitStruct);     
  }
  /*------------- If HT interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMATX_HTIT(pDevInitStruct->CPAL_Dev)) != 0)
  {         
    i2c_dbg("\n\rLOG : I2C Device TX DMA Half Transfer ");
    
    /* Call DMA TX HT UserCallback */
    CPAL_I2C_DMATXHT_UserCallback(pDevInitStruct);
  }  
  /*------------- If TE interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMATX_TEIT(pDevInitStruct->CPAL_Dev)) != 0)
  { 
    i2c_dbg("\n\rERROR : I2C Device TX DMA Transfer Error ");
    
    /* Update CPAL_State to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR; 
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferTx->wNumData = __CPAL_I2C_HAL_DMATX_GET_CNDT(pDevInitStruct->CPAL_Dev);
    
    /* Call DMA TX TE UserCallback */
    CPAL_I2C_DMATXTE_UserCallback(pDevInitStruct); 
  }  
  
   /* Clear DMA Interrupt Flag */
    __CPAL_I2C_HAL_CLEAR_DMATX_IT(pDevInitStruct->CPAL_Dev);
  
  return CPAL_PASS;
}

uint32_t I2C_DMA_RX_IRQHandler(CPAL_InitTypeDef* pDevInitStruct)
{
  /* Reinitialize Timeout Value to default (no timeout initiated) */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT; 
  
  i2c_dbg("\n\r\n\rLOG <CPAL_I2C_DMA_RX_IRQHandler> : I2C Device RX DMA ");
  
  /*------------- If TC interrupt ------------*/
  if ((__CPAL_I2C_HAL_GET_DMARX_TCIT(pDevInitStruct->CPAL_Dev)) != 0)
  {   
    i2c_dbg("\n\rLOG : I2C Device RX Complete");
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferRx->wNumData = 0;
       
    /* Call DMA RX TC UserCallback */
    CPAL_I2C_DMARXTC_UserCallback(pDevInitStruct);
    
 #ifdef CPAL_I2C_MASTER_MODE      
    /* If Master Mode selected */
    if ((pDevInitStruct->CPAL_Mode == CPAL_MODE_MASTER))
    { 
      /* If DMA Normal model */
      if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_DMARX_CIRCULAR) == 0)
      {         
        /* No Stop Condition Generation option bit not selected */   
        if ((pDevInitStruct->wCPAL_Options & CPAL_OPT_I2C_NOSTOP) == 0)
        {
          /* Generate Stop Condition */
          __CPAL_I2C_HAL_STOP(pDevInitStruct->CPAL_Dev);
          
          /* Disable DMA Request and Channel */          
          __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev);
          
          /* Wait until Busy flag is reset */ 
          __CPAL_I2C_TIMEOUT(!(__CPAL_I2C_HAL_GET_BUSY(pDevInitStruct->CPAL_Dev)), CPAL_I2C_TIMEOUT_BUSY);
        }
        else
        {
          /* Disable DMA Request */          
          __CPAL_I2C_HAL_DISABLE_DMAREQ(pDevInitStruct->CPAL_Dev);          
        } 
        
        /* Disable DMA Channel */
        __CPAL_I2C_HAL_DISABLE_DMARX(pDevInitStruct->CPAL_Dev);
        
        /* Disable EVENT Interrupt */
        __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
        
        /* Disable DMA automatic NACK generation */
        __CPAL_I2C_HAL_DISABLE_LAST(pDevInitStruct->CPAL_Dev);
        
        i2c_dbg("\n\rLOG : I2C Device Master RX DMA Disabled");
        
        /* Update CPAL_State to CPAL_STATE_READY */
        pDevInitStruct->CPAL_State = CPAL_STATE_READY; 
      }
      /* Call RX TC UserCallback */
      CPAL_I2C_RXTC_UserCallback(pDevInitStruct);
    }
 #endif /* CPAL_I2C_MASTER_MODE */ 
  }  
  /*------------- If HT interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMARX_HTIT(pDevInitStruct->CPAL_Dev)) != 0)
  {   
    i2c_dbg("\n\rLOG : I2C Device RX DMA Half Transfer");
    
    /* Call DMA RX HT UserCallback */
    CPAL_I2C_DMARXHT_UserCallback(pDevInitStruct);
  }  
  /*------------- If TE interrupt ------------*/
  else if ((__CPAL_I2C_HAL_GET_DMARX_TEIT(pDevInitStruct->CPAL_Dev)) != 0)
  {   
    i2c_dbg("\n\rERROR : I2C Device RX DMA Transfer Error ");
    
    /* Update CPAL_State to CPAL_STATE_ERROR */
    pDevInitStruct->CPAL_State = CPAL_STATE_ERROR; 
    
    /* Update remaining number of data */
    pDevInitStruct->pCPAL_TransferRx->wNumData = __CPAL_I2C_HAL_DMARX_GET_CNDT(pDevInitStruct->CPAL_Dev);
    
    /* Call DMA RX TE UserCallback */
    CPAL_I2C_DMARXTE_UserCallback(pDevInitStruct); 
  }
  
  /* Clear DMA Interrupt Flag */
  __CPAL_I2C_HAL_CLEAR_DMARX_IT(pDevInitStruct->CPAL_Dev);
  
  return CPAL_PASS;
}
#endif /* CPAL_I2C_DMA_PROGMODEL */
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
