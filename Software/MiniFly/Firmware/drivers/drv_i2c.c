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
    rt_size_t len = msg->len;
    rt_uint32_t stat = 0;


    return bytes;
}


static rt_size_t stm32_i2c_send_bytes(I2C_TypeDef *I2Cx, struct rt_i2c_msg *msg)
{
    rt_size_t bytes = 0;
    rt_size_t len = msg->len;
    rt_uint32_t stat = 0;
    /* Make sure start bit is not active */


    return bytes;
}
static void i2c_set_clock(I2C_TypeDef *I2Cx, uint32_t clock)
{

}

static rt_uint32_t i2c_send_addr(LPC_I2C_TypeDef *I2Cx, struct rt_i2c_msg *msg)
{
    rt_uint16_t addr;
    rt_uint16_t flags = msg->flags;
    /* Make sure start bit is not active */
    if (I2Cx->CONSET & I2C_I2CONSET_STA)
    {
        I2Cx->CONCLR = I2C_I2CONCLR_STAC;
    }
    /* Test on the direction to set/reset the read/write bit */
    addr = msg->addr << 1;
    if (flags & RT_I2C_RD)
    {
        /* Set the address bit0 for read */
        addr |= 1;
    }
    I2Cx->CONCLR = I2C_I2CONCLR_SIC;
    /* Send the address */
    I2Cx->DAT = addr & I2C_I2DAT_BITMASK;

    while (!(I2Cx->CONSET & I2C_I2CONSET_SI));

    return (I2Cx->STAT & I2C_STAT_CODE_BITMASK);
}


static rt_size_t stm32_i2c_xfer(struct rt_i2c_bus_device *bus,
                              struct rt_i2c_msg msgs[], rt_uint32_t num)
{
    struct rt_i2c_msg *msg;
    rt_uint32_t i;
    rt_err_t ret = RT_ERROR;
    rt_uint32_t stat = 0;
    struct stm_i2c_bus *stm_i2c = (struct stm_i2c_bus *)bus;
    /*start the i2c bus*/
    stat = lpc_i2c_start(lpc_i2c->I2C);
    if ((I2C_I2STAT_M_TX_RESTART != stat) && (I2C_I2STAT_M_TX_START != stat))
    {
        i2c_dbg("start the i2c bus failed,i2c bus stop!\n");
        goto out;
    }
    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];
        if (!(msg->flags & RT_I2C_NO_START))
        {
            if (i)
            {
                stat = lpc_i2c_start(lpc_i2c->I2C);
                if ((I2C_I2STAT_M_TX_RESTART != stat) && (I2C_I2STAT_M_TX_START != stat))
                {
                    i2c_dbg("restart the i2c bus failed,i2c bus stop!\n");
                    goto out;
                }
            }
            stat = i2c_send_addr(lpc_i2c->I2C, msg);
            if (I2C_I2STAT_M_TX_SLAW_ACK != stat && I2C_I2STAT_M_RX_SLAR_ACK != stat)
            {
                i2c_dbg("send i2c address but no ack,i2c stop!");
                goto out;
            }
        }
        if (msg->flags & RT_I2C_RD)
        {
            ret = lpc_i2c_recv_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("read %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_EIO;
                goto out;
            }
        }
        else
        {
            ret = lpc_i2c_send_bytes(lpc_i2c->I2C, msg);
            if (ret >= 1)
                i2c_dbg("write %d byte%s\n",
                        ret, ret == 1 ? "" : "s");
            if (ret < msg->len)
            {
                if (ret >= 0)
                    ret = -RT_ERROR;
                goto out;
            }
        }
    }
    ret = i;

out:
    i2c_dbg("send stop condition\n");
    lpc_i2c_stop(lpc_i2c->I2C);

    return ret;
}


static const struct rt_i2c_bus_device_ops i2c_ops =
{
    stm32_i2c_xfer,
    RT_NULL,
    RT_NULL
};



/** \brief init and register lpc spi bus.
*
* \param SPI: lpc SPI, e.g: LPC_SSP0,LPC_SSP1,LPC_SSP2.
* \param lpc_spi: lpc spi bus struct.
* \param spi_bus_name: spi bus name, e.g: "spi1"
* \return
*
*/
rt_err_t lpc_i2c_register(I2C_TypeDef *I2Cx,
                          struct stm32_i2c_bus *stm_i2c,
                          const char *spi_bus_name)
{

    return  rt_i2c_bus_device_register(&stm_i2c->parent, spi_bus_name);
}


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
  
  /* I2Cx Common Channel Configuration */
  DMA_InitStructure.DMA_BufferSize = 0xFFFF;
  DMA_InitStructure.DMA_PeripheralInc =  DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  /* Select I2Cx DR Address register as DMA PeripheralBaseAddress */
  CPAL_DMA_InitStructure.DMA_PeripheralBaseAddr = CPAL_I2C_DR [Device];
  
  /* If TX Direction (Transmission) selected */
  if ((Direction & CPAL_DIRECTION_TX) != 0)
  {         
    /* Select Memory to Peripheral transfer direction */
    CPAL_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    
    /* Initialize I2Cx DMA Tx Channel */
    DMA_Init((DMA_Channel_TypeDef*)CPAL_I2C_DMA_TX_Channel[Device], &CPAL_DMA_InitStructure);   
  }
  
  /* If RX Direction (Reception) selected */
  if ((Direction & CPAL_DIRECTION_RX ) != 0)
  {  
    /* Select Peripheral to Memory transfer direction */
    CPAL_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    
    /* Initialize I2Cx DMA Rx Channel */
    DMA_Init((DMA_Channel_TypeDef*)CPAL_I2C_DMA_RX_Channel[Device], &CPAL_DMA_InitStructure);   
  }
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
	

    rt_memset((void *)&lpc_i2c1, 0, sizeof(struct lpc_i2c_bus));
    lpc_i2c1.parent.ops = &i2c_ops;
    lpc_i2c_register(LPC_I2C1, &lpc_i2c1, "i2c1");
}

/*================== CPAL_I2C_Event_Handler ==================*/

#ifdef I2C_MASTER_MODE 
/**
  * @brief  Handles Master Start condition (SB) interrupt event.
  * @param  pDevInitStruct: Pointer to the peripheral configuration structure.
  * @retval CPAL_PASS or CPAL_FAIL. 
  */
static uint32_t I2C_MASTER_START_Handle(CPAL_InitTypeDef* pDevInitStruct)
{
  #ifdef CPAL_I2C_10BIT_ADDR_MODE  
  /* Declare local variable that contains Address Header */
  uint8_t I2CHeaderAddress = 0x00;
  #endif /* CPAL_I2C_10BIT_ADDR_MODE */

  /* Reinitialize Timeout Value */
  pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_DEFAULT;
  
  CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT"); 
  
  CPAL_LOG("\n\rLOG : I2C Device Start Acknowledged"); 
  
  /* If 7 bit Addressing Mode selected */
  if (pDevInitStruct->pCPAL_I2C_Struct->I2C_AcknowledgedAddress == I2C_AcknowledgedAddress_7bit)
  {        
    CPAL_LOG("\n\rLOG : I2C Device 7bit Address");
    
    /* Send Address */
    /* If Master run as receiver */
    if (pDevInitStruct->CPAL_State == CPAL_STATE_READY_RX)
    {
      /* Send Slave address with bit0 set for read */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferRx->wAddr1) | I2C_OAR1_ADD0));  
      
      /* Update CPAL_State to CPAL_STATE_BUSY */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_RX; 
      
      CPAL_LOG("\n\rLOG : I2C Device Busy RX");
    }    
    /* If Master run as Transmitter */
    else
    {
      /* Send Slave address with bit0 reset for write */
      __CPAL_I2C_HAL_SEND((pDevInitStruct->CPAL_Dev), (uint8_t)((pDevInitStruct->pCPAL_TransferTx->wAddr1) & (~I2C_OAR1_ADD0)));        
      
      /* Update CPAL_State to CPAL_STATE_BUSY */
      pDevInitStruct->CPAL_State = CPAL_STATE_BUSY_TX; 
      
      CPAL_LOG("\n\rLOG : I2C Device Busy TX");
    }
    
    CPAL_LOG("\n\rLOG : I2C Device Target Address Sent");
    
    /* Initialize Timeout value */
    pDevInitStruct->wCPAL_Timeout = CPAL_I2C_TIMEOUT_MIN + CPAL_I2C_TIMEOUT_ADDR;             
  }  
 #ifdef CPAL_I2C_10BIT_ADDR_MODE  
  /* If 10 bit Addressing Mode selected */
  else
  {  
    CPAL_LOG("\n\rLOG : I2C Device 10bit Address");
    								      
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
    
    CPAL_LOG("\n\rLOG : I2C Device Target Header Sent "); 
    
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
      
      CPAL_LOG("\n\rLOG : I2C Device Busy RX");
      
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
      
      CPAL_LOG("\n\rLOG : I2C Device Busy TX");
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
  
  CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
  
  CPAL_LOG("\n\rLOG : I2C Device Header Address Acknowledged");
  
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
  
  CPAL_LOG("\n\rLOG : I2C Device Target Address Sent");  
  
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
      
      CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
      
      CPAL_LOG("\n\rLOG : I2C Device Generates Stop");
      
      CPAL_LOG("\n\rLOG : I2C Device TX Complete");
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device TX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device TX BUFF IT Disabled");
      
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
      
      CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
      
      CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
      
      CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
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
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
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
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
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
        
        CPAL_LOG("\n\r\n\rLOG <I2C_EV_IRQHandler> : I2C Device Master IT");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Nack Programmed");
        
        CPAL_LOG("\n\rLOG : I2C Device RX Stop Programmed");
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
      CPAL_LOG("\n\rLOG : I2C Device Nack and Stop Generated ");
      
      CPAL_LOG("\n\rLOG : I2C Device RX Complete"); 
      
      /* Disable EVENT Interrupt */
      __CPAL_I2C_HAL_DISABLE_EVTIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device RX EVT IT Disabled");
      
      /* Disable Buffer interrupt */
      __CPAL_I2C_HAL_DISABLE_BUFIT(pDevInitStruct->CPAL_Dev);
      
      CPAL_LOG("\n\rLOG : I2C Device RX BUFF IT Disabled");
      
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
    
    CPAL_LOG("\n\rLOG : I2C Device BUFF IT Enabled"); 
    
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
      
      CPAL_LOG("\n\rLOG : I2C Device DMA TX Enabled");       
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
    
    CPAL_LOG("\n\rERROR : I2C Device Error"); 
    
    /* exit function */
    return CPAL_FAIL;
  }  
}

#endif
