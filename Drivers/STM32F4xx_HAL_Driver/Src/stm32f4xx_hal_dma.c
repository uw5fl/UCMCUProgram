/**
  ******************************************************************************
  * @file    stm32f4xx_hal_dma.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    06-May-2016
  * @brief   DMA HAL module driver.
  *    
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Direct Memory Access (DMA) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral State and errors functions
  @verbatim     
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
   (#) Enable and configure the peripheral to be connected to the DMA Stream
       (except for internal SRAM/FLASH memories: no initialization is 
       necessary) please refer to Reference manual for connection between peripherals
       and DMA requests .

   (#) For a given Stream, program the required configuration through the following parameters:
       Transfer Direction, Source and Destination data formats, 
       Circular, Normal or peripheral flow control mode, Stream Priority level, 
       Source and Destination Increment mode, FIFO mode and its Threshold (if needed), 
       Burst mode for Source and/or Destination (if needed) using HAL_DMA_Init() function.

     *** Polling mode IO operation ***
     =================================
    [..]
          (+) Use HAL_DMA_Start() to start DMA transfer after the configuration of Source 
              address and destination address and the Length of data to be transferred
          (+) Use HAL_DMA_PollForTransfer() to poll for the end of current transfer, in this  
              case a fixed Timeout can be configured by User depending from his application.

     *** Interrupt mode IO operation ***
     ===================================
    [..]
          (+) Configure the DMA interrupt priority using HAL_NVIC_SetPriority()
          (+) Enable the DMA IRQ handler using HAL_NVIC_EnableIRQ() 
          (+) Use HAL_DMA_Start_IT() to start DMA transfer after the configuration of  
              Source address and destination address and the Length of data to be transferred. In this 
              case the DMA interrupt is configured 
          (+) Use HAL_DMA_IRQHandler() called under DMA_IRQHandler() Interrupt subroutine
          (+) At the end of data transfer HAL_DMA_IRQHandler() function is executed and user can 
              add his own function by customization of function pointer XferCpltCallback and 
              XferErrorCallback (i.e a member of DMA handle structure).
    [..]
     (#) Use HAL_DMA_GetState() function to return the DMA state and HAL_DMA_GetError() in case of error 
         detection.

     (#) Use HAL_DMA_Abort() function to abort the current transfer

     -@-   In Memory-to-Memory transfer mode, Circular mode is not allowed.

     -@-   The FIFO is used mainly to reduce bus usage and to allow data packing/unpacking: it is
           possible to set different Data Sizes for the Peripheral and the Memory (ie. you can set
           Half-Word data size for the peripheral to access its data register and set Word data size
           for the Memory to gain in access time. Each two half words will be packed and written in
           a single access to a Word in the Memory).

     -@-   When FIFO is disabled, it is not allowed to configure different Data Sizes for Source
           and Destination. In this case the Peripheral Data Size will be applied to both Source
           and Destination.

     *** DMA HAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in DMA HAL driver.
       
      (+) __HAL_DMA_ENABLE: Enable the specified DMA Stream.
      (+) __HAL_DMA_DISABLE: Disable the specified DMA Stream.
      (+) __HAL_DMA_GET_FS: Return the current DMA Stream FIFO filled level.
      (+) __HAL_DMA_ENABLE_IT: Enable the specified DMA Stream interrupts.
      (+) __HAL_DMA_DISABLE_IT: Disable the specified DMA Stream interrupts.
      (+) __HAL_DMA_GET_IT_SOURCE: Check whether the specified DMA Stream interrupt has occurred or not. 

     [..]
      (@) You can refer to the DMA HAL driver header file for more useful macros

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup DMA DMA
  * @brief DMA HAL module driver
  * @{
  */

#ifdef HAL_DMA_MODULE_ENABLED

/* Private types -------------------------------------------------------------*/
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup DMA_Private_Constants
 * @{
 */
 #define HAL_TIMEOUT_DMA_ABORT    ((uint32_t)5)  /* 5 ms */
/**
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup DMA_Private_Functions
  * @{
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *hdma);
static HAL_StatusTypeDef DMA_CheckFifoParam(DMA_HandleTypeDef *hdma);

/**
  * @}
  */  
  
/* Exported functions --------------------------------------------------------*/
/** @addtogroup DMA_Exported_Functions
  * @{
  */

/** @addtogroup DMA_Exported_Functions_Group1
  *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Stream source
    and destination addresses, incrementation and data sizes, transfer direction, 
    circular/normal mode selection, memory-to-memory mode selection and Stream priority value.
    [..]
    The HAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */
  
/**
  * @brief  Initializes the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and create the associated handle.
  * @param  hdma: Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma)
{
  uint32_t tmp = 0U;
  uint32_t tickstart = HAL_GetTick();
  DMA_Base_Registers *regs;

  /* Check the DMA peripheral state */
  if(hdma == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_DMA_STREAM_ALL_INSTANCE(hdma->Instance));
  assert_param(IS_DMA_CHANNEL(hdma->Init.Channel));
  assert_param(IS_DMA_DIRECTION(hdma->Init.Direction));
  assert_param(IS_DMA_PERIPHERAL_INC_STATE(hdma->Init.PeriphInc));
  assert_param(IS_DMA_MEMORY_INC_STATE(hdma->Init.MemInc));
  assert_param(IS_DMA_PERIPHERAL_DATA_SIZE(hdma->Init.PeriphDataAlignment));
  assert_param(IS_DMA_MEMORY_DATA_SIZE(hdma->Init.MemDataAlignment));
  assert_param(IS_DMA_MODE(hdma->Init.Mode));
  assert_param(IS_DMA_PRIORITY(hdma->Init.Priority));
  assert_param(IS_DMA_FIFO_MODE_STATE(hdma->Init.FIFOMode));
  /* Check the memory burst, peripheral burst and FIFO threshold parameters only
     when FIFO mode is enabled */
  if(hdma->Init.FIFOMode != DMA_FIFOMODE_DISABLE)
  {
    assert_param(IS_DMA_FIFO_THRESHOLD(hdma->Init.FIFOThreshold));
    assert_param(IS_DMA_MEMORY_BURST(hdma->Init.MemBurst));
    assert_param(IS_DMA_PERIPHERAL_BURST(hdma->Init.PeriphBurst));
  }
  
  /* Allocate lock resource */
  __HAL_UNLOCK(hdma);

  /* Change DMA peripheral state */
  hdma->State = HAL_DMA_STATE_BUSY;
  
  /* Disable the peripheral */
  __HAL_DMA_DISABLE(hdma);
  
  /* Check if the DMA Stream is effectively disabled */
  while((hdma->Instance->CR & DMA_SxCR_EN) != RESET)
  {
    /* Check for the Timeout */
    if((HAL_GetTick() - tickstart ) > HAL_TIMEOUT_DMA_ABORT)
    {
      /* Update error code */
      hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;
      
      /* Change the DMA state */
      hdma->State = HAL_DMA_STATE_TIMEOUT;
      
      return HAL_TIMEOUT;
    }
  }
  
  /* Get the CR register value */
  tmp = hdma->Instance->CR;

  /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits */
  tmp &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                      DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  | \
                      DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   | \
                      DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));

  /* Prepare the DMA Stream configuration */
  tmp |=  hdma->Init.Channel             | hdma->Init.Direction        |
          hdma->Init.PeriphInc           | hdma->Init.MemInc           |
          hdma->Init.PeriphDataAlignment | hdma->Init.MemDataAlignment |
          hdma->Init.Mode                | hdma->Init.Priority;

  /* the Memory burst and peripheral burst are not used when the FIFO is disabled */
  if(hdma->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get memory burst and peripheral burst */
    tmp |=  hdma->Init.MemBurst | hdma->Init.PeriphBurst;
  }
  
  /* Write to DMA Stream CR register */
  hdma->Instance->CR = tmp;  

  /* Get the FCR register value */
  tmp = hdma->Instance->FCR;

  /* Clear Direct mode and FIFO threshold bits */
  tmp &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

  /* Prepare the DMA Stream FIFO configuration */
  tmp |= hdma->Init.FIFOMode;

  /* the FIFO threshold is not used when the FIFO mode is disabled */
  if(hdma->Init.FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get the FIFO threshold */
    tmp |= hdma->Init.FIFOThreshold;
    
    if(DMA_CheckFifoParam(hdma) != HAL_OK)
    {
      /* Update error code */
      hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
      
      /* Change the DMA state */
      hdma->State = HAL_DMA_STATE_READY;
      
      return HAL_ERROR; 
    }
  }
  
  /* Write to DMA Stream FCR */
  hdma->Instance->FCR = tmp;

  /* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
     DMA steam Base Address needed by HAL_DMA_IRQHandler() and HAL_DMA_PollForTransfer() */
  regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(hdma);
  
  /* Clear all interrupt flags */
  regs->IFCR = 0x3FU << hdma->StreamIndex;

  /* Initialize the error code */
  hdma->ErrorCode = HAL_DMA_ERROR_NONE;
                                                                                     
  /* Initialize the DMA state */
  hdma->State = HAL_DMA_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  DeInitializes the DMA peripheral 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma)
{
  DMA_Base_Registers *regs;

  /* Check the DMA peripheral state */
  if(hdma == NULL)
  {
    return HAL_ERROR;
  }
  
  /* Check the DMA peripheral state */
  if(hdma->State == HAL_DMA_STATE_BUSY)
  {
    /* Return error status */
    return HAL_BUSY;
  }

  /* Disable the selected DMA Streamx */
  __HAL_DMA_DISABLE(hdma);

  /* Reset DMA Streamx control register */
  hdma->Instance->CR   = 0U;

  /* Reset DMA Streamx number of data to transfer register */
  hdma->Instance->NDTR = 0U;

  /* Reset DMA Streamx peripheral address register */
  hdma->Instance->PAR  = 0U;

  /* Reset DMA Streamx memory 0 address register */
  hdma->Instance->M0AR = 0U;
  
  /* Reset DMA Streamx memory 1 address register */
  hdma->Instance->M1AR = 0U;
  
  /* Reset DMA Streamx FIFO control register */
  hdma->Instance->FCR  = (uint32_t)0x00000021U;
  
  /* Get DMA steam Base Address */  
  regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(hdma);
  
  /* Clear all interrupt flags at correct offset within the register */
  regs->IFCR = 0x3FU << hdma->StreamIndex;

  /* Initialize the error code */
  hdma->ErrorCode = HAL_DMA_ERROR_NONE;

  /* Initialize the DMA state */
  hdma->State = HAL_DMA_STATE_RESET;

  /* Release Lock */
  __HAL_UNLOCK(hdma);

  return HAL_OK;
}

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group2
  *
@verbatim   
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and 
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request  

@endverbatim
  * @{
  */

/**
  * @brief  Starts the DMA Transfer.
  * @param  hdma      : pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  /* Check the parameters */
  assert_param(IS_DMA_BUFFER_SIZE(DataLength));

  /* Process locked */
  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;
    
    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    
    /* Configure the source, destination address and the data length */
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

    /* Enable the Peripheral */
    __HAL_DMA_ENABLE(hdma);
  }
  else
  {
    /* Process unlocked */
    __HAL_UNLOCK(hdma);
    
    /* Return error status */
    status = HAL_BUSY;
  } 
  return status; 
}

/**
  * @brief  Starts the DMA Transfer with interrupt enabled.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  
  /* Check the parameters */
  assert_param(IS_DMA_BUFFER_SIZE(DataLength));
 
  /* Process locked */
  __HAL_LOCK(hdma);
  
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    /* Change DMA peripheral state */
    hdma->State = HAL_DMA_STATE_BUSY;
    
    /* Initialize the error code */
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;
    
    /* Configure the source, destination address and the data length */
    DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);
    
    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << hdma->StreamIndex;
    
    /* Enable Common interrupts*/
    hdma->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
    hdma->Instance->FCR |= DMA_IT_FE;
    
    if(hdma->XferHalfCpltCallback != NULL)
    {
      hdma->Instance->CR  |= DMA_IT_HT;
    }
    
    /* Enable the Peripheral */
    __HAL_DMA_ENABLE(hdma);
  }
  else
  {
    /* Process unlocked */
    __HAL_UNLOCK(hdma);	  
    
    /* Return error status */
    status = HAL_BUSY;
  }
  
  return status;
}

/**
  * @brief  Aborts the DMA Transfer.
  * @param  hdma  : pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  *                   
  * @note  After disabling a DMA Stream, a check for wait until the DMA Stream is 
  *        effectively disabled is added. If a Stream is disabled 
  *        while a data transfer is ongoing, the current data will be transferred
  *        and the Stream will be effectively disabled only after the transfer of
  *        this single data is finished.  
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  
  uint32_t tickstart = HAL_GetTick();
  
  if(hdma->State != HAL_DMA_STATE_BUSY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;
    
    /* Process Unlocked */
    __HAL_UNLOCK(hdma);
    
    return HAL_ERROR;
  }
  else
  {
    /* Disable all the transfer interrupts */
    hdma->Instance->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
    hdma->Instance->FCR &= ~(DMA_IT_FE);
    
    if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
    {
      hdma->Instance->CR  &= ~(DMA_IT_HT);
    }
    
    /* Disable the stream */
    __HAL_DMA_DISABLE(hdma);
    
    /* Check if the DMA Stream is effectively disabled */
    while((hdma->Instance->CR & DMA_SxCR_EN) != RESET)
    {
      /* Check for the Timeout */
      if((HAL_GetTick() - tickstart ) > HAL_TIMEOUT_DMA_ABORT)
      {
        /* Update error code */
        hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;
        
        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
        
        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_TIMEOUT;
        
        return HAL_TIMEOUT;
      }
    }
    
    /* Clear all interrupt flags at correct offset within the register */
    regs->IFCR = 0x3FU << hdma->StreamIndex;
    
    /* Process Unlocked */
    __HAL_UNLOCK(hdma);
    
    /* Change the DMA state*/
    hdma->State = HAL_DMA_STATE_READY;
  }
  return HAL_OK;
}

/**
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma  : pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
  if(hdma->State != HAL_DMA_STATE_BUSY)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;
    return HAL_ERROR;
  }
  else
  {
    /* Set Abort State  */
    hdma->State = HAL_DMA_STATE_ABORT;
    
    /* Disable the stream */
    __HAL_DMA_DISABLE(hdma);
  }

  return HAL_OK;
}

/**
  * @brief  Polling for transfer complete.
  * @param  hdma:          pointer to a DMA_HandleTypeDef structure that contains
  *                        the configuration information for the specified DMA Stream.
  * @param  CompleteLevel: Specifies the DMA level complete.
  * @note   The polling mode is kept in this version for legacy. it is recommanded to use the IT model instead.
  *         This model could be used for debug purpose.
  * @note   The HAL_DMA_PollForTransfer API cannot be used in circular and double buffering mode (automatic circular mode). 
  * @param  Timeout:       Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout)
{
  HAL_StatusTypeDef status = HAL_OK; 
  uint32_t temp;
  uint32_t tickstart = HAL_GetTick(); 
  uint32_t tmpisr;
  
  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs;
  
  /* Polling mode not supported in circular mode and double buffering mode */
  if ((hdma->Instance->CR & DMA_SxCR_CIRC) != RESET)
  {
    hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
    return HAL_ERROR;
  }
  
  /* Get the level transfer complete flag */
  if(CompleteLevel == HAL_DMA_FULL_TRANSFER)
  {
    /* Transfer Complete flag */
    temp = DMA_FLAG_TCIF0_4 << hdma->StreamIndex;
  }
  else
  {
    /* Half Transfer Complete flag */
    temp = DMA_FLAG_HTIF0_4 << hdma->StreamIndex;
  }
  
  regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;
  tmpisr = regs->ISR;
  
  while((tmpisr & temp) == RESET )
  {
    /* Check for the Timeout (Not applicable in circular mode)*/
    if(Timeout != HAL_MAX_DELAY)
    {
      if((Timeout == 0)||((HAL_GetTick() - tickstart ) > Timeout))
      {
        /* Update error code */
        hdma->ErrorCode = HAL_DMA_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);
        
        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;
        
        return HAL_TIMEOUT;
      }
    }
    
    if((tmpisr & (DMA_FLAG_TEIF0_4 << hdma->StreamIndex)) != RESET)
    {
      /* Update error code */
      hdma->ErrorCode |= HAL_DMA_ERROR_TE;
      
      /* Clear the transfer error flag */
      regs->IFCR = DMA_FLAG_TEIF0_4 << hdma->StreamIndex;
    }
    
    if((tmpisr & (DMA_FLAG_FEIF0_4 << hdma->StreamIndex)) != RESET)
    {
      /* Update error code */
      hdma->ErrorCode |= HAL_DMA_ERROR_FE;
      
      /* Clear the FIFO error flag */
      regs->IFCR = DMA_FLAG_FEIF0_4 << hdma->StreamIndex;
    }
    
    if((tmpisr & (DMA_FLAG_DMEIF0_4 << hdma->StreamIndex)) != RESET)
    {
      /* Update error code */
      hdma->ErrorCode |= HAL_DMA_ERROR_DME;
      
      /* Clear the Direct Mode error flag */
      regs->IFCR = DMA_FLAG_DMEIF0_4 << hdma->StreamIndex;
    }
  }
  
  if(hdma->ErrorCode != HAL_DMA_ERROR_NONE)
  {
    if((hdma->ErrorCode & HAL_DMA_ERROR_TE) != RESET)
    {
      HAL_DMA_Abort(hdma);
    
      /* Clear the half transfer and transfer complete flags */
      regs->IFCR = (DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4) << hdma->StreamIndex;
    
      /* Process Unlocked */
      __HAL_UNLOCK(hdma);

      /* Change the DMA state */
      hdma->State= HAL_DMA_STATE_READY;

      return HAL_ERROR;
   }

   status = HAL_ERROR;
  }
  
  /* Get the level transfer complete flag */
  if(CompleteLevel == HAL_DMA_FULL_TRANSFER)
  {
    /* Clear the half transfer and transfer complete flags */
    regs->IFCR = (DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4) << hdma->StreamIndex;
    
    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    hdma->State = HAL_DMA_STATE_READY;
  }
  else
  {
    /* Clear the half transfer and transfer complete flags */
    regs->IFCR = (DMA_FLAG_HTIF0_4) << hdma->StreamIndex;
  }
  
  return status;
}

/**
  * @brief  Handles DMA interrupt request.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval None
  */
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma)
{
  uint32_t tmpisr;
  __IO uint32_t count = 0;
  uint32_t timeout = SystemCoreClock / 9600;

  /* calculate DMA base and stream number */
  DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  tmpisr = regs->ISR;

  /* Transfer Error Interrupt management ***************************************/
  if ((tmpisr & (DMA_FLAG_TEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TE) != RESET)
    {
      /* Disable the transfer error interrupt */
      hdma->Instance->CR  &= ~(DMA_IT_TE);
      
      /* Clear the transfer error flag */
      regs->IFCR = DMA_FLAG_TEIF0_4 << hdma->StreamIndex;
      
      /* Update error code */
      hdma->ErrorCode |= HAL_DMA_ERROR_TE;
    }
  }
  /* FIFO Error Interrupt management ******************************************/
  if ((tmpisr & (DMA_FLAG_FEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_FE) != RESET)
    {
      /* Clear the FIFO error flag */
      regs->IFCR = DMA_FLAG_FEIF0_4 << hdma->StreamIndex;

      /* Update error code */
      hdma->ErrorCode |= HAL_DMA_ERROR_FE;
    }
  }
  /* Direct Mode Error Interrupt management ***********************************/
  if ((tmpisr & (DMA_FLAG_DMEIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_DME) != RESET)
    {
      /* Clear the direct mode error flag */
      regs->IFCR = DMA_FLAG_DMEIF0_4 << hdma->StreamIndex;

      /* Update error code */
      hdma->ErrorCode |= HAL_DMA_ERROR_DME;
    }
  }
  /* Half Transfer Complete Interrupt management ******************************/
  if ((tmpisr & (DMA_FLAG_HTIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_HT) != RESET)
    {
      /* Clear the half transfer complete flag */
      regs->IFCR = DMA_FLAG_HTIF0_4 << hdma->StreamIndex;
      
      /* Multi_Buffering mode enabled */
      if(((hdma->Instance->CR) & (uint32_t)(DMA_SxCR_DBM)) != RESET)
      {
        /* Current memory buffer used is Memory 0 */
        if((hdma->Instance->CR & DMA_SxCR_CT) == RESET)
        {
          if(hdma->XferHalfCpltCallback != NULL)
          {
            /* Half transfer callback */
            hdma->XferHalfCpltCallback(hdma);
          }
        }
        /* Current memory buffer used is Memory 1 */
        else
        {
          if(hdma->XferM1HalfCpltCallback != NULL)
          {
            /* Half transfer callback */
            hdma->XferM1HalfCpltCallback(hdma);
          }
        }
      }
      else
      {
        /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
        if((hdma->Instance->CR & DMA_SxCR_CIRC) == RESET)
        {
          /* Disable the half transfer interrupt */
          hdma->Instance->CR  &= ~(DMA_IT_HT);
        }
        
        if(hdma->XferHalfCpltCallback != NULL)
        {
          /* Half transfer callback */
          hdma->XferHalfCpltCallback(hdma);
        }
      }
    }
  }
  /* Transfer Complete Interrupt management ***********************************/
  if ((tmpisr & (DMA_FLAG_TCIF0_4 << hdma->StreamIndex)) != RESET)
  {
    if(__HAL_DMA_GET_IT_SOURCE(hdma, DMA_IT_TC) != RESET)
    {
      /* Clear the transfer complete flag */
      regs->IFCR = DMA_FLAG_TCIF0_4 << hdma->StreamIndex;
      
      if(HAL_DMA_STATE_ABORT == hdma->State)
      {
        /* Disable all the transfer interrupts */
        hdma->Instance->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
        hdma->Instance->FCR &= ~(DMA_IT_FE);
        
        if((hdma->XferHalfCpltCallback != NULL) || (hdma->XferM1HalfCpltCallback != NULL))
        {
          hdma->Instance->CR  &= ~(DMA_IT_HT);
        }

        /* Clear all interrupt flags at correct offset within the register */
        regs->IFCR = 0x3FU << hdma->StreamIndex;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        /* Change the DMA state */
        hdma->State = HAL_DMA_STATE_READY;

        if(hdma->XferAbortCallback != NULL)
        {
          hdma->XferAbortCallback(hdma);
        }
        return;
      }

      if(((hdma->Instance->CR) & (uint32_t)(DMA_SxCR_DBM)) != RESET)
      {
        /* Current memory buffer used is Memory 0 */
        if((hdma->Instance->CR & DMA_SxCR_CT) == RESET)
        {
          if(hdma->XferM1CpltCallback != NULL)
          {
            /* Transfer complete Callback for memory1 */
            hdma->XferM1CpltCallback(hdma);
          }
        }
        /* Current memory buffer used is Memory 1 */
        else
        {
          if(hdma->XferCpltCallback != NULL)
          {
            /* Transfer complete Callback for memory0 */
            hdma->XferCpltCallback(hdma);
          }
        }
      }
      /* Disable the transfer complete interrupt if the DMA mode is not CIRCULAR */
      else
      {
        if((hdma->Instance->CR & DMA_SxCR_CIRC) == RESET)
        {
          /* Disable the transfer complete interrupt */
          hdma->Instance->CR  &= ~(DMA_IT_TC);

          /* Process Unlocked */
          __HAL_UNLOCK(hdma);

          /* Change the DMA state */
          hdma->State = HAL_DMA_STATE_READY;
        }

        if(hdma->XferCpltCallback != NULL)
        {
          /* Transfer complete callback */
          hdma->XferCpltCallback(hdma);
        }
      }
    }
  }
  
  /* manage error case */
  if(hdma->ErrorCode != HAL_DMA_ERROR_NONE)
  {
    if((hdma->ErrorCode & HAL_DMA_ERROR_TE) != RESET)
    {
      hdma->State = HAL_DMA_STATE_ABORT;

      /* Disable the stream */
      __HAL_DMA_DISABLE(hdma);

      do
      {
        if (++count > timeout)
        {
          break;
        }
      }
      while((hdma->Instance->CR & DMA_SxCR_EN) != RESET);

      /* Process Unlocked */
      __HAL_UNLOCK(hdma);

      /* Change the DMA state */
      hdma->State = HAL_DMA_STATE_READY;
    }

    if(hdma->XferErrorCallback != NULL)
    {
      /* Transfer error callback */
      hdma->XferErrorCallback(hdma);
    }
  }
}

/**
  * @brief  Register callbacks
  * @param  hdma:                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Stream.
  * @param  CallbackID:           User Callback identifer
  *                               a DMA_HandleTypeDef structure as parameter.
  * @param  pCallback:            pointer to private callbacsk function which has pointer to 
  *                               a DMA_HandleTypeDef structure as parameter.
  * @retval HAL status
  */                      
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma))
{

  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hdma);

  if(HAL_DMA_STATE_READY == hdma->State)
  {
    switch (CallbackID)
    {
    case  HAL_DMA_XFER_CPLT_CB_ID:
      hdma->XferCpltCallback = pCallback;
      break;

    case  HAL_DMA_XFER_HALFCPLT_CB_ID:
      hdma->XferHalfCpltCallback = pCallback;
      break;

    case  HAL_DMA_XFER_M1CPLT_CB_ID:
      hdma->XferM1CpltCallback = pCallback;
      break;

    case  HAL_DMA_XFER_M1HALFCPLT_CB_ID:
      hdma->XferM1HalfCpltCallback = pCallback;
      break;

    case  HAL_DMA_XFER_ERROR_CB_ID:
      hdma->XferErrorCallback = pCallback;
      break;

    case  HAL_DMA_XFER_ABORT_CB_ID:
      hdma->XferAbortCallback = pCallback;
      break;

    default:
      break;
    }
  }
  else
  {
    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hdma);
  
  return status;
}

/**
  * @brief  UnRegister callbacks
  * @param  hdma:                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Stream.
  * @param  CallbackID:           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @retval HAL status
  */              
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  /* Process locked */
  __HAL_LOCK(hdma);
  
  if(HAL_DMA_STATE_READY == hdma->State)
  {
    switch (CallbackID)
    {
    case  HAL_DMA_XFER_CPLT_CB_ID:
      hdma->XferCpltCallback = NULL;
      break;
      
    case  HAL_DMA_XFER_HALFCPLT_CB_ID:
      hdma->XferHalfCpltCallback = NULL;
      break;
      
    case  HAL_DMA_XFER_M1CPLT_CB_ID:
      hdma->XferM1CpltCallback = NULL;
      break;
      
    case  HAL_DMA_XFER_M1HALFCPLT_CB_ID:
      hdma->XferM1HalfCpltCallback = NULL;
      break;
      
    case  HAL_DMA_XFER_ERROR_CB_ID:
      hdma->XferErrorCallback = NULL;
      break;
      
    case  HAL_DMA_XFER_ABORT_CB_ID:
      hdma->XferAbortCallback = NULL;
      break; 
      
    case   HAL_DMA_XFER_ALL_CB_ID:
      hdma->XferCpltCallback = NULL;
      hdma->XferHalfCpltCallback = NULL;
      hdma->XferM1CpltCallback = NULL;
      hdma->XferM1HalfCpltCallback = NULL;
      hdma->XferErrorCallback = NULL;
      hdma->XferAbortCallback = NULL;
      break; 
      
    default:
      status = HAL_ERROR;
      break;
    }
  }
  else
  {
    status = HAL_ERROR;
  }
  
  /* Release Lock */
  __HAL_UNLOCK(hdma);
  
  return status;
}

/**
  * @}
  */

/** @addtogroup DMA_Exported_Functions_Group3
  *
@verbatim
 ===============================================================================
                    ##### State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */

/**
  * @brief  Returns the DMA state.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.
  * @retval HAL state
  */
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma)
{
  return hdma->State;
}

/**
  * @brief  Return the DMA error code
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Stream.
  * @retval DMA Error Code
  */
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma)
{
  return hdma->ErrorCode;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup DMA_Private_Functions
  * @{
  */

/**
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  /* Clear DBM bit */
  hdma->Instance->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /* Configure DMA Stream data length */
  hdma->Instance->NDTR = DataLength;

  /* Peripheral to Memory */
  if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
  {
    /* Configure DMA Stream destination address */
    hdma->Instance->PAR = DstAddress;

    /* Configure DMA Stream source address */
    hdma->Instance->M0AR = SrcAddress;
  }
  /* Memory to Peripheral */
  else
  {
    /* Configure DMA Stream source address */
    hdma->Instance->PAR = SrcAddress;

    /* Configure DMA Stream destination address */
    hdma->Instance->M0AR = DstAddress;
  }
}

/**
  * @brief  Returns the DMA Stream base address depending on stream number
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream. 
  * @retval Stream base address
  */
static uint32_t DMA_CalcBaseAndBitshift(DMA_HandleTypeDef *hdma)
{
  uint32_t stream_number = (((uint32_t)hdma->Instance & 0xFFU) - 16U) / 24U;
  
  /* lookup table for necessary bitshift of flags within status registers */
  static const uint8_t flagBitshiftOffset[8U] = {0U, 6U, 16U, 22U, 0U, 6U, 16U, 22U};
  hdma->StreamIndex = flagBitshiftOffset[stream_number];
  
  if (stream_number > 3U)
  {
    /* return pointer to HISR and HIFCR */
    hdma->StreamBaseAddress = (((uint32_t)hdma->Instance & (uint32_t)(~0x3FFU)) + 4U);
  }
  else
  {
    /* return pointer to LISR and LIFCR */
    hdma->StreamBaseAddress = ((uint32_t)hdma->Instance & (uint32_t)(~0x3FFU));
  }
  
  return hdma->StreamBaseAddress;
}

/**
  * @brief  Checks compatibility between FIFO threshold level and size of the memory burst
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream. 
  * @retval HAL status
  */
static HAL_StatusTypeDef DMA_CheckFifoParam(DMA_HandleTypeDef *hdma)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint32_t tmp = hdma->Init.FIFOThreshold;
  
  /* Memory Data size equal to Byte */
  if(hdma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE)
  {
    switch (tmp)
    {
      case DMA_FIFO_THRESHOLD_1QUARTERFULL:
        if((hdma->Init.MemBurst & DMA_SxCR_MBURST_1) == DMA_SxCR_MBURST_1)
        {
          status = HAL_ERROR;
        }
        break;
      case DMA_FIFO_THRESHOLD_HALFFULL:
        if(hdma->Init.MemBurst == DMA_MBURST_INC16)
        {
          status = HAL_ERROR;
        }
        break;
      case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
        if((hdma->Init.MemBurst & DMA_SxCR_MBURST_1) == DMA_SxCR_MBURST_1)
        {
          status = HAL_ERROR;
        }
        break;
      case DMA_FIFO_THRESHOLD_FULL:
        break;
      default:
        break;
    }
  }
  
  /* Memory Data size equal to Half-Word */
  else if (hdma->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD)
  {
    switch (tmp)
    {
      case DMA_FIFO_THRESHOLD_1QUARTERFULL:
        status = HAL_ERROR;
        break;
      case DMA_FIFO_THRESHOLD_HALFFULL:
        if ((hdma->Init.MemBurst & DMA_SxCR_MBURST_1) == DMA_SxCR_MBURST_1)
        {
          status = HAL_ERROR;
        }
        break;
      case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
        status = HAL_ERROR;
        break;
      case DMA_FIFO_THRESHOLD_FULL:
        if (hdma->Init.MemBurst == DMA_MBURST_INC16)
        {
          status = HAL_ERROR;
        }
        break;   
      default:
        break;
    }
  }
  
  /* Memory Data size equal to Word */
  else
  {
    switch (tmp)
    {
      case DMA_FIFO_THRESHOLD_1QUARTERFULL:
      case DMA_FIFO_THRESHOLD_HALFFULL:
      case DMA_FIFO_THRESHOLD_3QUARTERSFULL:
        status = HAL_ERROR;
        break;
      case DMA_FIFO_THRESHOLD_FULL:
        if ((hdma->Init.MemBurst & DMA_SxCR_MBURST_1) == DMA_SxCR_MBURST_1)
        {
          status = HAL_ERROR;
        }
		break;
      default:
        break;
    }
  } 
  
  return status; 
}

/**
  * @}
  */

#endif /* HAL_DMA_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
