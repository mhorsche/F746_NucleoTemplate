/**
  ******************************************************************************
  * @file    stm32f7xx_hal_crc.c
  * @author  MCD Application Team
  * @brief   CRC HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Cyclic Redundancy Check (CRC) peripheral:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
         (+) Enable CRC AHB clock using __HAL_RCC_CRC_CLK_ENABLE();
         (+) Initialize CRC calculator
             (++) specify generating polynomial (peripheral default or non-default one)
             (++) specify initialization value (peripheral default or non-default one)
             (++) specify input data format
             (++) specify input or output data inversion mode if any
         (+) Use HAL_CRC_Accumulate() function to compute the CRC value of the
             input data buffer starting with the previously computed CRC as
             initialization value
         (+) Use HAL_CRC_Calculate() function to compute the CRC value of the
             input data buffer starting with the defined initialization value
             (default or non-default) to initiate CRC calculation

    ##### Callback registration #####
    ==================================

    [..]
    The compilation define USE_HAL_CRC_REGISTER_CALLBACKS when set to 1
    allows the user to configure dynamically the driver callbacks.

    [..]
    Use Function @ref HAL_CRC_RegisterCallback() to register a user callback.
    Function @ref HAL_CRC_RegisterCallback() allows to register following callbacks:
    (+) MspInitCallback           : CRC MspInit.
    (+) MspDeInitCallback         : CRC MspDeInit.
    This function takes as parameters the HAL peripheral handle, the Callback ID
    and a pointer to the user callback function.

    [..]
    Use function @ref HAL_CRC_UnRegisterCallback() to reset a callback to the default
    weak (surcharged) function.
    @ref HAL_CRC_UnRegisterCallback() takes as parameters the HAL peripheral handle,
    and the Callback ID.
    This function allows to reset following callbacks:
    (+) MspInitCallback           : CRC MspInit.
    (+) MspDeInitCallback         : CRC MspDeInit.

    [..]
    Exception done for MspInit and MspDeInit functions that are respectively
    reset to the legacy weak (surcharged) functions in the @ref HAL_CRC_Init()
    and @ref HAL_CRC_DeInit() only when these callbacks are null (not registered beforehand).
    If not, MspInit or MspDeInit are not null, the @ref HAL_CRC_Init() and @ref HAL_CRC_DeInit()
    keep and use the user MspInit/MspDeInit callbacks (registered beforehand).

    [..]
    Callbacks can be registered/unregistered in HAL_CRC_STATE_READY state only.
    Exception done MspInit/MspDeInit that can be registered/unregistered
    in HAL_CRC_STATE_READY or HAL_CRC_STATE_RESET state, thus registered (user)
    MspInit/DeInit callbacks can be used during the Init/DeInit.
    In that case first register the MspInit/MspDeInit user callbacks
    using @ref HAL_CRC_RegisterCallback() before calling @ref HAL_CRC_DeInit()
    or @ref HAL_CRC_Init() function.

    [..]
    When The compilation define USE_HAL_CRC_REGISTER_CALLBACKS is set to 0 or
    not defined, the callback registration feature is not available
    and weak (surcharged) callbacks are used.

  @endverbatim
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/** @addtogroup STM32F7xx_HAL_Driver
  * @{
  */

/** @defgroup CRC CRC
  * @brief CRC HAL module driver.
  * @{
  */

#ifdef HAL_CRC_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup CRC_Private_Functions CRC Private Functions
  * @{
  */
static uint32_t CRC_Handle_8(CRC_HandleTypeDef *hcrc, uint8_t pBuffer[], uint32_t BufferLength);
static uint32_t CRC_Handle_16(CRC_HandleTypeDef *hcrc, uint16_t pBuffer[], uint32_t BufferLength);
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup CRC_Exported_Functions CRC Exported Functions
  * @{
  */

/** @defgroup CRC_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions.
  *
@verbatim
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize the CRC according to the specified parameters
          in the CRC_InitTypeDef and create the associated handle
      (+) DeInitialize the CRC peripheral
      (+) Initialize the CRC MSP (MCU Specific Package)
      (+) DeInitialize the CRC MSP

@endverbatim
  * @{
  */

/**
  * @brief  Initialize the CRC according to the specified
  *         parameters in the CRC_InitTypeDef and create the associated handle.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */
  if (hcrc == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CRC_ALL_INSTANCE(hcrc->Instance));

#if (USE_HAL_CRC_REGISTER_CALLBACKS == 1)
  if (hcrc->State == HAL_CRC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcrc->Lock = HAL_UNLOCKED;

    if (hcrc->MspInitCallback == NULL)
    {
      hcrc->MspInitCallback = HAL_CRC_MspInit; /* Legacy weak MspInit  */
    }

    /* Init the low level hardware */
    hcrc->MspInitCallback(hcrc);
  }
#else
  if (hcrc->State == HAL_CRC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hcrc->Lock = HAL_UNLOCKED;
    /* Init the low level hardware */
    HAL_CRC_MspInit(hcrc);
  }
#endif /* USE_HAL_CRC_REGISTER_CALLBACKS */

  hcrc->State = HAL_CRC_STATE_BUSY;

  /* check whether or not non-default generating polynomial has been
   * picked up by user */
  assert_param(IS_DEFAULT_POLYNOMIAL(hcrc->Init.DefaultPolynomialUse));
  if (hcrc->Init.DefaultPolynomialUse == DEFAULT_POLYNOMIAL_ENABLE)
  {
    /* initialize peripheral with default generating polynomial */
    WRITE_REG(hcrc->Instance->POL, DEFAULT_CRC32_POLY);
    MODIFY_REG(hcrc->Instance->CR, CRC_CR_POLYSIZE, CRC_POLYLENGTH_32B);
  }
  else
  {
    /* initialize CRC peripheral with generating polynomial defined by user */
    if (HAL_CRCEx_Polynomial_Set(hcrc, hcrc->Init.GeneratingPolynomial, hcrc->Init.CRCLength) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  /* check whether or not non-default CRC initial value has been
   * picked up by user */
  assert_param(IS_DEFAULT_INIT_VALUE(hcrc->Init.DefaultInitValueUse));
  if (hcrc->Init.DefaultInitValueUse == DEFAULT_INIT_VALUE_ENABLE)
  {
    WRITE_REG(hcrc->Instance->INIT, DEFAULT_CRC_INITVALUE);
  }
  else
  {
    WRITE_REG(hcrc->Instance->INIT, hcrc->Init.InitValue);
  }


  /* set input data inversion mode */
  assert_param(IS_CRC_INPUTDATA_INVERSION_MODE(hcrc->Init.InputDataInversionMode));
  MODIFY_REG(hcrc->Instance->CR, CRC_CR_REV_IN, hcrc->Init.InputDataInversionMode);

  /* set output data inversion mode */
  assert_param(IS_CRC_OUTPUTDATA_INVERSION_MODE(hcrc->Init.OutputDataInversionMode));
  MODIFY_REG(hcrc->Instance->CR, CRC_CR_REV_OUT, hcrc->Init.OutputDataInversionMode);

  /* makes sure the input data format (bytes, halfwords or words stream)
   * is properly specified by user */
  assert_param(IS_CRC_INPUTDATA_FORMAT(hcrc->InputDataFormat));

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  DeInitialize the CRC peripheral.
  * @param  hcrc CRC handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc)
{
  /* Check the CRC handle allocation */
  if (hcrc == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_CRC_ALL_INSTANCE(hcrc->Instance));

  /* Check the CRC peripheral state */
  if (hcrc->State == HAL_CRC_STATE_BUSY)
  {
    return HAL_BUSY;
  }

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Reset CRC calculation unit */
  __HAL_CRC_DR_RESET(hcrc);

  /* Reset IDR register content */
  CLEAR_BIT(hcrc->Instance->IDR, CRC_IDR_IDR);

#if (USE_HAL_CRC_REGISTER_CALLBACKS == 1)
  if (hcrc->MspDeInitCallback == NULL)
  {
    hcrc->MspDeInitCallback = HAL_CRC_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware */
  hcrc->MspDeInitCallback(hcrc);
#else
  /* DeInit the low level hardware */
  HAL_CRC_MspDeInit(hcrc);
#endif /* USE_HAL_CRC_REGISTER_CALLBACKS */

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_RESET;

  /* Process unlocked */
  __HAL_UNLOCK(hcrc);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Initializes the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcrc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_CRC_MspInit can be implemented in the user file
   */
}

/**
  * @brief  DeInitialize the CRC MSP.
  * @param  hcrc CRC handle
  * @retval None
  */
__weak void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcrc);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_CRC_MspDeInit can be implemented in the user file
   */
}

#if (USE_HAL_CRC_REGISTER_CALLBACKS == 1)
/**
  * @brief  Register a User CRC Callback
  *         To be used instead of the weak predefined callback
  * @param  hcrc CRC handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_CRC_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_CRC_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_RegisterCallback(CRC_HandleTypeDef *hcrc, HAL_CRC_CallbackIDTypeDef CallbackID, pCRC_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(hcrc);

  if ((HAL_CRC_STATE_READY == hcrc->State) ||
      (HAL_CRC_STATE_RESET == hcrc->State))
  {
    switch (CallbackID)
    {
    case HAL_CRC_MSPINIT_CB_ID :
      hcrc->MspInitCallback = pCallback;
      break;

    case HAL_CRC_MSPDEINIT_CB_ID :
      hcrc->MspDeInitCallback = pCallback;
      break;

    default :
     /* Return error status */
      status =  HAL_ERROR;
      break;
    }
  }
  else
  {
    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hcrc);
  return status;
}

/**
  * @brief  Unregister an CRC Callback
  *         CRC callabck is redirected to the weak predefined callback
  * @param  hcrc CRC handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_CRC_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_CRC_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_CRC_UnRegisterCallback(CRC_HandleTypeDef *hcrc, HAL_CRC_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(hcrc);

  if ((HAL_CRC_STATE_READY == hcrc->State) ||
      (HAL_CRC_STATE_RESET == hcrc->State))
  {
    switch (CallbackID)
    {
    case HAL_CRC_MSPINIT_CB_ID :
      hcrc->MspInitCallback = HAL_CRC_MspInit;              /* Legacy weak MspInit  */
      break;

    case HAL_CRC_MSPDEINIT_CB_ID :
      hcrc->MspDeInitCallback = HAL_CRC_MspDeInit;          /* Legacy weak MspDeInit  */
      break;

    default :
     /* Return error status */
      status =  HAL_ERROR;
      break;
    }
  }
  else
  {
    /* Return error status */
    status =  HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(hcrc);
  return status;
}

#endif /* USE_HAL_CRC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group2 Peripheral Control functions
  *  @brief    management functions.
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) compute the 7, 8, 16 or 32-bit CRC value of an 8, 16 or 32-bit data buffer
          using combination of the previous CRC value and the new one.

       [..]  or

      (+) compute the 7, 8, 16 or 32-bit CRC value of an 8, 16 or 32-bit data buffer
          independently of the previous CRC value.

@endverbatim
  * @{
  */

/**
  * @brief  Compute the 7, 8, 16 or 32-bit CRC value of an 8, 16 or 32-bit data buffer
  *         starting with the previously computed CRC as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer, exact input data format is
  *         provided by hcrc->InputDataFormat.
  * @param  BufferLength input data buffer length (number of bytes if pBuffer
  *         type is * uint8_t, number of half-words if pBuffer type is * uint16_t,
  *         number of words if pBuffer type is * uint32_t).
  * @note  By default, the API expects a uint32_t pointer as input buffer parameter.
  *        Input buffer pointers with other types simply need to be cast in uint32_t
  *        and the API will internally adjust its input data processing based on the
  *        handle field hcrc->InputDataFormat.
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
{
  uint32_t index;      /* CRC input data buffer index */
  uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DR register) */

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  switch (hcrc->InputDataFormat)
  {
    case CRC_INPUTDATA_FORMAT_WORDS:
      /* Enter Data to the CRC calculator */
      for (index = 0U; index < BufferLength; index++)
      {
        hcrc->Instance->DR = pBuffer[index];
      }
      temp = hcrc->Instance->DR;
      break;

    case CRC_INPUTDATA_FORMAT_BYTES:
      temp = CRC_Handle_8(hcrc, (uint8_t *)pBuffer, BufferLength);
      break;

    case CRC_INPUTDATA_FORMAT_HALFWORDS:
      temp = CRC_Handle_16(hcrc, (uint16_t *)(void *)pBuffer, BufferLength);    /* Derogation MisraC2012 R.11.5 */
      break;
    default:
      break;
  }

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return the CRC computed value */
  return temp;
}

/**
  * @brief  Compute the 7, 8, 16 or 32-bit CRC value of an 8, 16 or 32-bit data buffer
  *         starting with hcrc->Instance->INIT as initialization value.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer, exact input data format is
  *         provided by hcrc->InputDataFormat.
  * @param  BufferLength input data buffer length (number of bytes if pBuffer
  *         type is * uint8_t, number of half-words if pBuffer type is * uint16_t,
  *         number of words if pBuffer type is * uint32_t).
  * @note  By default, the API expects a uint32_t pointer as input buffer parameter.
  *        Input buffer pointers with other types simply need to be cast in uint32_t
  *        and the API will internally adjust its input data processing based on the
  *        handle field hcrc->InputDataFormat.
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength)
{
  uint32_t index;      /* CRC input data buffer index */
  uint32_t temp = 0U;  /* CRC output (read from hcrc->Instance->DR register) */

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_BUSY;

  /* Reset CRC Calculation Unit (hcrc->Instance->INIT is
  *  written in hcrc->Instance->DR) */
  __HAL_CRC_DR_RESET(hcrc);

  switch (hcrc->InputDataFormat)
  {
    case CRC_INPUTDATA_FORMAT_WORDS:
      /* Enter 32-bit input data to the CRC calculator */
      for (index = 0U; index < BufferLength; index++)
      {
        hcrc->Instance->DR = pBuffer[index];
      }
      temp = hcrc->Instance->DR;
      break;

    case CRC_INPUTDATA_FORMAT_BYTES:
      /* Specific 8-bit input data handling  */
      temp = CRC_Handle_8(hcrc, (uint8_t *)pBuffer, BufferLength);
      break;

    case CRC_INPUTDATA_FORMAT_HALFWORDS:
      /* Specific 16-bit input data handling  */
      temp = CRC_Handle_16(hcrc, (uint16_t *)(void *)pBuffer, BufferLength);    /* Derogation MisraC2012 R.11.5 */
      break;

    default:
      break;
  }

  /* Change CRC peripheral state */
  hcrc->State = HAL_CRC_STATE_READY;

  /* Return the CRC computed value */
  return temp;
}

/**
  * @}
  */

/** @defgroup CRC_Exported_Functions_Group3 Peripheral State functions
  *  @brief    Peripheral State functions.
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State functions #####
 ===============================================================================
    [..]
    This subsection permits to get in run-time the status of the peripheral.

@endverbatim
  * @{
  */

/**
  * @brief  Return the CRC handle state.
  * @param  hcrc CRC handle
  * @retval HAL state
  */
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc)
{
  /* Return CRC handle state */
  return hcrc->State;
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup CRC_Private_Functions
  * @{
  */

/**
  * @brief  Enter 8-bit input data to the CRC calculator.
  *         Specific data handling to optimize processing time.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer
  * @param  BufferLength input data buffer length
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
static uint32_t CRC_Handle_8(CRC_HandleTypeDef *hcrc, uint8_t pBuffer[], uint32_t BufferLength)
{
  uint32_t i; /* input data buffer index */
  uint16_t data;
  __IO uint16_t *pReg;

  /* Processing time optimization: 4 bytes are entered in a row with a single word write,
   * last bytes must be carefully fed to the CRC calculator to ensure a correct type
   * handling by the peripheral */
  for (i = 0U; i < (BufferLength / 4U); i++)
  {
    hcrc->Instance->DR = ((uint32_t)pBuffer[4U * i] << 24U) | \
                         ((uint32_t)pBuffer[(4U * i) + 1U] << 16U) | \
                         ((uint32_t)pBuffer[(4U * i) + 2U] << 8U)  | \
                         (uint32_t)pBuffer[(4U * i) + 3U];
  }
  /* last bytes specific handling */
  if ((BufferLength % 4U) != 0U)
  {
    if ((BufferLength % 4U) == 1U)
    {
      *(__IO uint8_t *)(__IO void *)(&hcrc->Instance->DR) = pBuffer[4U * i];         /* Derogation MisraC2012 R.11.5 */
    }
    if ((BufferLength % 4U) == 2U)
    {
      data = ((uint16_t)(pBuffer[4U * i]) << 8U) | (uint16_t)pBuffer[(4U * i) + 1U];
      pReg = (__IO uint16_t *)(__IO void *)(&hcrc->Instance->DR);                    /* Derogation MisraC2012 R.11.5 */
      *pReg = data;
    }
    if ((BufferLength % 4U) == 3U)
    {
      data = ((uint16_t)(pBuffer[4U * i]) << 8U) | (uint16_t)pBuffer[(4U * i) + 1U];
      pReg = (__IO uint16_t *)(__IO void *)(&hcrc->Instance->DR);                    /* Derogation MisraC2012 R.11.5 */
      *pReg = data;

      *(__IO uint8_t *)(__IO void *)(&hcrc->Instance->DR) = pBuffer[(4U * i) + 2U];  /* Derogation MisraC2012 R.11.5 */
    }
  }

  /* Return the CRC computed value */
  return hcrc->Instance->DR;
}

/**
  * @brief  Enter 16-bit input data to the CRC calculator.
  *         Specific data handling to optimize processing time.
  * @param  hcrc CRC handle
  * @param  pBuffer pointer to the input data buffer
  * @param  BufferLength input data buffer length
  * @retval uint32_t CRC (returned value LSBs for CRC shorter than 32 bits)
  */
static uint32_t CRC_Handle_16(CRC_HandleTypeDef *hcrc, uint16_t pBuffer[], uint32_t BufferLength)
{
  uint32_t i;  /* input data buffer index */
  __IO uint16_t *pReg;

  /* Processing time optimization: 2 HalfWords are entered in a row with a single word write,
   * in case of odd length, last HalfWord must be carefully fed to the CRC calculator to ensure
   * a correct type handling by the peripheral */
  for (i = 0U; i < (BufferLength / 2U); i++)
  {
    hcrc->Instance->DR = ((uint32_t)pBuffer[2U * i] << 16U) | (uint32_t)pBuffer[(2U * i) + 1U];
  }
  if ((BufferLength % 2U) != 0U)
  {
    pReg = (__IO uint16_t *)(__IO void *)(&hcrc->Instance->DR);                 /* Derogation MisraC2012 R.11.5 */
    *pReg = pBuffer[2U * i];
  }

  /* Return the CRC computed value */
  return hcrc->Instance->DR;
}

/**
  * @}
  */

#endif /* HAL_CRC_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */
