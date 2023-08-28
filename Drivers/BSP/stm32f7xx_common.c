/**
 * @file stm32f7xx_common.c
 * @author horsche (horsche@li.plus)
 * @brief
 * @version
 * @date 2023-08-11
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_common.h"

#ifdef EEPROM_MODULE
#include "Components/eeprom/eeprom.h"
#endif /* EEPROM_MODULE */

#ifdef STLINK_MODULE
#include "Components/stlink/stlink.h"
#endif /* STLINK_MODULE */

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"

/* FreeRTOS+TCP includes. */
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>
#include <FreeRTOS_DHCP.h>
#include "NetworkInterface.h"

/* FreeRTOS tick timer interrupt handler prototype ---------------------------*/
extern void xPortSysTickHandler(void);

/******************************************************************************/
/*                           Global Error Handler                             */
/******************************************************************************/

void vAssertCalled(uint32_t ulLine, const char *pcFile)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Turn on/off LEDs to indicate error handler */
  LED_GREEN_Off();
  LED_BLUE_Off();
  LED_RED_On();

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  };
}
/*-----------------------------------------------------------*/

/******************************************************************************/
/*                Cortex-M7 Processor Exceptions Handlers                     */
/******************************************************************************/

void NMI_Handler(void)
{
}
/*-----------------------------------------------------------*/

void HardFault_Handler(void)
{
  vAssertCalled(__LINE__, __FUNCTION__);
}
/*-----------------------------------------------------------*/

void MemManage_Handler(void)
{
  vAssertCalled(__LINE__, __FUNCTION__);
}
/*-----------------------------------------------------------*/

void BusFault_Handler(void)
{
  vAssertCalled(__LINE__, __FUNCTION__);
}
/*-----------------------------------------------------------*/

void UsageFault_Handler(void)
{
  vAssertCalled(__LINE__, __FUNCTION__);
}
/*-----------------------------------------------------------*/

void DebugMon_Handler(void)
{
  vAssertCalled(__LINE__, __FUNCTION__);
}
/*-----------------------------------------------------------*/

void SysTick_Handler(void)
{
  HAL_IncTick();

#if (INCLUDE_xTaskGetSchedulerState == 1)
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */

    xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
}
/*-----------------------------------------------------------*/

#ifndef vPortSVCHandler
__weak void SVC_Handler(void)
{
}
#endif /* vPortSVCHandler */
/*-----------------------------------------------------------*/

#ifndef xPortPendSVHandler
__weak void PendSV_Handler(void)
{
}
#endif /* xPortPendSVHandler */
/*-----------------------------------------------------------*/

/******************************************************************************/
/*                                 IWDG module                                */
/******************************************************************************/
#ifdef IWDG_MODULE
/**
 * @brief Initializes the IWDG (Independent Watchdog Timer) peripheral and
 *    creates the associated handle.
 * @param[in] hIwdg pointer to a IWDG_HandleTypeDef structure that contains
 *    the configuration information for IWDG.
 */
void MCU_IWDG_Init(IWDG_HandleTypeDef *hIwdg)
{
  // if (HAL_IWDG_GetState(hIwdg) == HAL_IWDG_STATE_RESET)
  {
    /*##-1- Configure the IWDG peripheral ####################################*/
    hIwdg->Instance = IWDG;
    hIwdg->Init.Prescaler = IWDG_PRESCALER_256; /* 32 kHz / 256 = 125 Hz */
    hIwdg->Init.Reload = (5 * 125);             /* Between 0x0000 and 0x0FFF (4065) */
    hIwdg->Init.Window = IWDG_WINDOW_DISABLE;

    if (HAL_IWDG_Init(hIwdg) != HAL_OK)
    {
      vAssertCalled(__LINE__, __FUNCTION__);
    }
  }
}
/*-----------------------------------------------------------*/

/**
 * @brief Refresh the IWDG.
 * @param[in] hIwdg pointer to a IWDG_HandleTypeDef structure that contains
 *    the configuration information for the specified IWDG module.
 */
void MCU_IWDG_Refresh(IWDG_HandleTypeDef *hIwdg)
{
  /* Reload IWDG counter with value defined in the reload register */
  __HAL_IWDG_RELOAD_COUNTER(hIwdg);
}
/*-----------------------------------------------------------*/
#endif

/******************************************************************************/
/*                                 RNG module                                 */
/******************************************************************************/
#ifdef RTOS_RNG_MODULE

static RNG_HandleTypeDef hRandomNumberGenerator;

/**
 * @brief Initializes the RNG MSP.
 * @param[in] hRng pointer to a RNG_HandleTypeDef structure that contains
 *    the configuration information for RNG.
 */
static void RTOS_RNG_MspInit(RNG_HandleTypeDef *hRng)
{
  /*##-1- Enable peripherals Clocks ##########################################*/
  __HAL_RCC_RNG_CLK_ENABLE();
}
/*-----------------------------------------------------------*/

void RTOS_RNG_Init(void)
{
  RNG_HandleTypeDef *hRng = &hRandomNumberGenerator;

  if (HAL_RNG_GetState(hRng) == HAL_RNG_STATE_RESET)
  {
    HAL_RNG_RegisterCallback(hRng, HAL_RNG_MSPINIT_CB_ID, RTOS_RNG_MspInit);

    /*##-1- Configure the RNG peripheral #####################################*/
    hRng->Instance = RTOS_RNG;

    if (HAL_RNG_Init(hRng) != HAL_OK)
    {
      vAssertCalled(__LINE__, __FUNCTION__);
    }
  }
}
/*-----------------------------------------------------------*/

/**
 * @brief DeInitializes the RNG MSP.
 * @param[in] hRng pointer to a RNG_HandleTypeDef structure that contains
 *    the configuration information for RNG.
 */
static void RTOS_RNG_MspDeInit(RNG_HandleTypeDef *hRng)
{
  /*##-1- Disable peripherals Clocks #########################################*/
  __HAL_RCC_RNG_CLK_DISABLE();
}
/*-----------------------------------------------------------*/

void RTOS_RNG_DeInit(void)
{
  RNG_HandleTypeDef *hRng = &hRandomNumberGenerator;

  HAL_RNG_RegisterCallback(hRng, HAL_RNG_MSPDEINIT_CB_ID, RTOS_RNG_MspDeInit);

  /* RNG peripheral */
  HAL_RNG_DeInit(hRng);
}
/*-----------------------------------------------------------*/

HAL_StatusTypeDef RTOS_RNG_GenerateRandomNumber(uint32_t *pulValue)
{
  RNG_HandleTypeDef *hRng = &hRandomNumberGenerator;

  HAL_StatusTypeDef xResult;

  xResult = HAL_RNG_GenerateRandomNumber(hRng, &(hRng->RandomNumber));
  if (xResult == HAL_OK)
  {
    *pulValue = hRng->RandomNumber;
  }
  else
  {
    *pulValue = 0;
  }

  return xResult;
}
/*-----------------------------------------------------------*/

#endif /* RTOS_RNG_MODULE */

/******************************************************************************/
/*                                  TIM module                                */
/******************************************************************************/
#ifdef RTOS_TIM_MODULE

static TIM_HandleTypeDef hTimHighResHandler;
static uint32_t ulTimHighResInterruptCount = 0;

/**
 * @brief This function handles FreeRTOS global TIM interrupt.
 */
void RTOS_TIM_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&hTimHighResHandler);
}
/*-----------------------------------------------------------*/

/**
 * @brief  Period elapsed callback in non-blocking mode
 * @param[in] htim: TIM handle
 */
static void RTOS_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *hTim)
{
  ulTimHighResInterruptCount++;
}
/*-----------------------------------------------------------*/

/**
 * @brief FreeRTOS TIM MSP Initialization
 *         This function configures the hardware resources:
 *            - Peripheral's clock enable
 *            - NVIC configuration
 * @param[in] hTim: TIM handle pointer
 */
static void RTOS_TIM_MspInit(TIM_HandleTypeDef *hTim)
{
  /*##-1- Enable peripherals clock ###########################################*/
  RTOS_TIM_CLK_ENABLE();

  /*##-2- NVIC configuration for DMA transfer complete interrupt #############*/
#ifdef RTOS_TIM_IRQn
  HAL_NVIC_SetPriority(RTOS_TIM_IRQn, RTOS_IRQPriority, 0);
  HAL_NVIC_EnableIRQ(RTOS_TIM_IRQn);
#endif /* RTOS_TIM_IRQn */
}
/*-----------------------------------------------------------*/

void RTOS_TIM_Init(void)
{
  TIM_HandleTypeDef *hTim = &hTimHighResHandler;

  if (HAL_TIM_Base_GetState(hTim) == HAL_TIM_STATE_RESET)
  {
    HAL_TIM_RegisterCallback(hTim, HAL_TIM_BASE_MSPINIT_CB_ID, RTOS_TIM_MspInit);

    /*##-1- Configure the TIM peripheral #####################################*/
    hTim->Instance = RTOS_TIM;                         /* Register base address */
    hTim->Init.Prescaler = (RTOS_TIM_PRESCALER - 1ul); /* Specifies the prescaler value used to divide the TIM clock. */
    hTim->Init.Period = (RTOS_TIM_PERIOD - 1ul);       /* Specifies the period value to be loaded into the active. */
    hTim->Init.CounterMode = TIM_COUNTERMODE_UP;       /* Specifies the counter mode. */
    hTim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* Specifies the clock division. */
    hTim->Init.RepetitionCounter = 0ul;                /* Specifies the repetition counter value. */
    hTim->Channel = RTOS_TIM_ACTIVE_CHANNEL;

    if (HAL_TIM_Base_Init(hTim) != HAL_OK)
    {
      vAssertCalled(__LINE__, __FUNCTION__);
    }

    /* Reset SR because HAL_TIM_Base_Init caused the TIM_FLAG_UPDATE flag to become set */
    __HAL_TIM_CLEAR_FLAG(hTim, TIM_FLAG_UPDATE);

    /* Assign callback function for priod elapse and start timer */
    if (HAL_TIM_RegisterCallback(hTim,
                                 HAL_TIM_PERIOD_ELAPSED_CB_ID,
                                 RTOS_TIM_PeriodElapsedCallback) != HAL_OK)
    {
      vAssertCalled(__LINE__, __FUNCTION__);
    }

    /* Start timer as interrupt mode. */
    if (HAL_TIM_Base_Start_IT(hTim) != HAL_OK)
    {
      vAssertCalled(__LINE__, __FUNCTION__);
    }

    /* Ignore the initial interrupt which sets ulTimHighResInterruptCount = 1. */
    __HAL_TIM_CLEAR_FLAG(hTim, TIM_FLAG_UPDATE);
    ulTimHighResInterruptCount = 0ul;
  }
}
/*-----------------------------------------------------------*/

uint64_t RTOS_TIM_GetValue(void)
{
  uint64_t ullReturn;
  if (hTimHighResHandler.Instance == NULL)
  {
    ullReturn = 1000ull * xTaskGetTickCount();
  }
  else
  {
    uint32_t ulCounts[2];
    uint32_t ulSlowCount;

    for (;;)
    {
      ulCounts[0] = hTimHighResHandler.Instance->CNT;
      ulSlowCount = ulTimHighResInterruptCount;
      ulCounts[1] = hTimHighResHandler.Instance->CNT;
      if (ulCounts[1] >= ulCounts[0])
      {
        /* TIM2_IRQHandler() has not occurred in between. */
        break;
      }
    }
    ullReturn = (uint64_t)ulSlowCount * (uint64_t)(__HAL_TIM_GET_AUTORELOAD(&hTimHighResHandler) + 1) + (uint64_t)(ulCounts[1]);
  }

  return ullReturn;
}
/*-----------------------------------------------------------*/

#endif /* RTOS_TIM_MODULE */

/******************************************************************************/
/*                          IO (LED, PB) OPERATIONS                           */
/* LED to indicate MCU status                                                 */
/******************************************************************************/
#ifdef GPIO_MODULE

void LED_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*##-1- Enable Digital Output Pins #########################################*/
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /*##-1a- Configure LED1 (Green) output #####################################*/
#ifdef LED1_GPIO_PIN
  LED1_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LED1_GPIO_PIN;
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET);
#endif /* LED1_GPIO_PIN */

  /*##-1b- Configure LED2 (Blue) output ######################################*/
#ifdef LED2_GPIO_PIN
  LED2_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LED2_GPIO_PIN;
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_RESET);
#endif /* LED2_GPIO_PIN */

  /*##-1c- Configure LED3 (Red) output #######################################*/
#ifdef LED3_GPIO_PIN
  LED3_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = LED3_GPIO_PIN;
  HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_RESET);
#endif /* LED3_GPIO_PIN */

  /*##-2- Enable Digital Input Pins ##########################################*/

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /*##-2a- Configure User Push Button ########################################*/
#ifdef USER_BUTTON_GPIO_PIN
  USER_BUTTON_GPIO_CLK_ENABLE();
  GPIO_InitStruct.Pin = USER_BUTTON_GPIO_PIN;
#ifdef USER_BUTTON_EXTI_IRQn /* MODE_EXTI */
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
#else  /* MODE_GPIO */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
#endif /* USER_BUTTON_EXTI_IRQn */
  HAL_GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStruct);

/* Enable and set Button EXTI Interrupt to the lowest priority */
#ifdef USER_BUTTON_EXTI_IRQn
  HAL_NVIC_SetPriority((IRQn_Type)(USER_BUTTON_EXTI_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(USER_BUTTON_EXTI_IRQn));
#endif /* USER_BUTTON_EXTI_IRQn */
#endif /* USER_BUTTON_GPIO_PIN */
}
/*-----------------------------------------------------------*/

void MCU_GPIO_DeInit(void)
{
  /*##-1- Disable Digital Output Pins ########################################*/

  /*##-1a- Configure LED1 (Green) output #####################################*/
#ifdef LED1_GPIO_PIN
  LED1_GPIO_CLK_DISABLE();
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_DeInit(LED1_GPIO_PORT, LED1_GPIO_PIN);
#endif /* LED1_GPIO_PIN */

  /*##-1b- Configure LED2 (Yellow) output ####################################*/
#ifdef LED2_GPIO_PIN
  LED2_GPIO_CLK_DISABLE();
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_DeInit(LED2_GPIO_PORT, LED2_GPIO_PIN);
#endif /* LED2_GPIO_PIN */

  /*##-1c- Configure LED3 (Red) output #######################################*/
#ifdef LED3_GPIO_PIN
  LED3_GPIO_CLK_DISABLE();
  HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_DeInit(LED3_GPIO_PORT, LED3_GPIO_PIN);
#endif /* LED3_GPIO_PIN */

  /*##-2- Disable Digital Input Pins #########################################*/
#ifdef USER_BUTTON_GPIO_PIN
#ifdef USER_BUTTON_EXTI_IRQn
  HAL_NVIC_DisableIRQ((IRQn_Type)(USER_BUTTON_EXTI_IRQn));
#endif /* USER_BUTTON_EXTI_IRQn */
  HAL_GPIO_DeInit(USER_BUTTON_GPIO_PORT, USER_BUTTON_GPIO_PIN);
#endif /* USER_BUTTON_GPIO_PIN */
}
/*-----------------------------------------------------------*/

#endif /* GPIO_MODULE */

// /******************************************************************************/
// /*                              ST-Link MODULE                                */
// /* Virtual-COM Port for console logging                                       */
// /******************************************************************************/
// #ifdef STLINK_MODULE

// #ifdef STLK_UART_IRQHandler
// /**
//  * @brief This function handles DMA Rx interrupt request.
//  */
// void STLK_UART_IRQHandler(void)
// {
//   // LogDebug(("STLK_UART_IRQHandler"));
//   HAL_UART_IRQHandler(&(pxSTLinkContext->hUart));
// }
// /*-----------------------------------------------------------*/
// #endif /* STLK_UART_IRQHandler */

// #ifdef STLK_UART_TX_DMA_IRQHandler
// /**
//  * @brief This function handles DMA Tx interrupt request.
//  */
// void STLK_UART_TX_DMA_IRQHandler(void)
// {
//   // LogDebug(("STLK_UART_TX_DMA_IRQHandler"));
//   HAL_DMA_IRQHandler(pxSTLinkContext->hUart.hdmatx);
// }
// /*-----------------------------------------------------------*/
// #endif /* STLK_UART_TX_DMA_IRQHandler */

// #ifdef STLK_UART_RX_DMA_IRQHandler
// /**
//  * @brief This function handles DMA Rx interrupt request.
//  */
// void STLK_UART_RX_DMA_IRQHandler(void)
// {
//   // LogDebug(("STLK_UART_RX_DMA_IRQHandler"));
//   HAL_DMA_IRQHandler(pxSTLinkContext->hUart.hdmarx);
// }
// /*-----------------------------------------------------------*/
// #endif /* STLK_UART_RX_DMA_IRQHandler */

// /**
//  * @brief Register UART transfer buffer complete (half complete) callback
//  *    functions used by the ST-LINK driver.
//  * @param[out] hUart: UART handle
//  * @param[in] pCallback: pointer to private callback function which has pointer
//  *    to a UART_HandleTypeDef structure as parameter.
//  */
// void STLINK_UART_RegisterCallback(UART_HandleTypeDef *hUart, void (*pCallback)(UART_HandleTypeDef *_hUart))
// {
//   if (HAL_UART_GetState(hUart) != HAL_UART_STATE_BUSY)
//   {
//     UART_InitCallbacksToDefault(hUart);
//     // HAL_UART_RegisterCallback(hUart, HAL_UART_RX_HALFCOMPLETE_CB_ID, pCallback);
//     // HAL_UART_RegisterCallback(hUart, HAL_UART_RX_COMPLETE_CB_ID, pCallback);
//     HAL_UART_RegisterCallback(hUart, HAL_UART_TX_HALFCOMPLETE_CB_ID, pCallback);
//     HAL_UART_RegisterCallback(hUart, HAL_UART_TX_COMPLETE_CB_ID, pCallback);
//   }
// }
// /*-----------------------------------------------------------*/

// /**
//  * @brief UART MSP Initialization
//  *         This function configures the hardware resources:
//  *            - Peripheral's clock enable
//  *            - Peripheral's GPIO Configuration
//  * @param[in] hUart: UART handle pointer
//  */
// static void STLINK_UART_MspInit(UART_HandleTypeDef *hUart)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
//   RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = {0};

//   /*##-1- Enable peripherals and GPIO Clocks #################################*/
//   /* Enable GPIO TX/RX clock */
//   STLK_UART_TX_GPIO_CLK_ENABLE();
//   STLK_UART_RX_GPIO_CLK_ENABLE();

//   /* Select SysClk as source of USART clocks */
//   RCC_PeriphClkInit.PeriphClockSelection = STLK_UART_RCC_PERIPHCLK;
//   RCC_PeriphClkInit.Usart3ClockSelection = STLK_UART_RCC_CLKSOURCE;
//   HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

//   /* Enable USARTx clock */
//   STLK_UART_CLK_ENABLE();

//   /*##-2- Configure peripheral GPIO ##########################################*/
//   /* UART TX GPIO pin configuration  */
//   GPIO_InitStruct.Pin = STLK_UART_TX_PIN;
//   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//   GPIO_InitStruct.Pull = GPIO_PULLUP;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//   GPIO_InitStruct.Alternate = STLK_UART_TX_AF;

//   HAL_GPIO_Init(STLK_UART_TX_GPIO_PORT, &GPIO_InitStruct);

//   /* UART RX GPIO pin configuration  */
//   GPIO_InitStruct.Pin = STLK_UART_RX_PIN;
//   GPIO_InitStruct.Alternate = STLK_UART_RX_AF;

//   HAL_GPIO_Init(STLK_UART_RX_GPIO_PORT, &GPIO_InitStruct);

//   /*##-3- Configure the DMA ##################################################*/
// #ifdef STLK_UART_TX_DMA_STREAM
//   /* Enable DMA clock */
//   STLK_UART_TX_DMA_CLK_ENABLE();

//   /* Configure the DMA handler for transmission process */
//   DMA_HandleTypeDef *hdma_tx = hUart->hdmatx;
//   hdma_tx->Instance = STLK_UART_TX_DMA_STREAM;
//   hdma_tx->Init.Channel = STLK_UART_TX_DMA_CHANNEL;
//   hdma_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//   hdma_tx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//   hdma_tx->Init.Mode = DMA_NORMAL;
//   hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;

//   hdma_tx->Init.MemInc = DMA_MINC_ENABLE; /* Dynamic Memory: 1 x uint8_t */
//   hdma_tx->Init.MemBurst = DMA_MBURST_INC4;
//   hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;

//   hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE; /* Static Peripheral: 1 x uint8_t */
//   hdma_tx->Init.PeriphBurst = DMA_PBURST_INC4;
//   hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;

//   hdma_tx->Init.Priority = DMA_PRIORITY_LOW;

//   /* Init the DMA */
//   if (HAL_DMA_Init(hdma_tx) != HAL_OK)
//   {
//     vAssertCalled(__LINE__, __FUNCTION__);
//   }

//   /* Associate the initialized DMA handle to the UART handle */
//   // __HAL_LINKDMA(hUart, hdmatx, *hdma_tx);
//   hdma_tx->Parent = hUart;
//   // HAL_DMA_RegisterCallback(&hdma_tx, HAL_DMA_XFER_CPLT_CB_ID, HAL_DMA_TransferCpltCallback);
//   // HAL_DMA_RegisterCallback(&hdma_tx, HAL_DMA_XFER_ERROR_CB_ID, HAL_DMA_ErrorCallback);
// #endif

// #ifdef STLK_UART_RX_DMA_STREAM
//   /* Enable DMA clock */
//   STLK_UART_RX_DMA_CLK_ENABLE();

//   /* Configure the DMA handler for reception process */
//   DMA_HandleTypeDef *hdma_rx = hUart->hdmarx;
//   hdma_rx->Instance = STLK_UART_RX_DMA_STREAM;
//   hdma_rx->Init.Channel = STLK_UART_RX_DMA_CHANNEL;
//   hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
//   hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
//   hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
//   hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//   hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//   hdma_rx->Init.Mode = DMA_NORMAL;
//   hdma_rx->Init.Priority = DMA_PRIORITY_HIGH;
//   hdma_rx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//   hdma_rx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//   hdma_rx->Init.MemBurst = DMA_MBURST_INC4;
//   hdma_rx->Init.PeriphBurst = DMA_PBURST_INC4;

//   /* Init the DMA */
//   if (HAL_DMA_Init(hdma_rx) != HAL_OK)
//   {
//     vAssertCalled(__LINE__, __FUNCTION__);
//   }

//   /* Associate the initialized DMA handle to the the UART handle */
//   hdma_rx->Parent = hUart;
//   // __HAL_LINKDMA(hUart, hdmarx, hdma_rx);
//   // HAL_DMA_RegisterCallback(&hdma_rx, HAL_DMA_XFER_CPLT_CB_ID, HAL_DMA_TransferCpltCallback);
//   // HAL_DMA_RegisterCallback(&hdma_rx, HAL_DMA_XFER_ERROR_CB_ID, HAL_DMA_ErrorCallback);
// #endif

//   /*##-4- Configure the NVIC for USART and DMA ###############################*/
// #ifdef STLK_UART_TX_DMA_IRQn
//   /* NVIC configuration for DMA transfer complete interrupt (UARTx_TX) */
//   HAL_NVIC_SetPriority(STLK_UART_TX_DMA_IRQn, STLK_IRQPriority, 2);
//   HAL_NVIC_EnableIRQ(STLK_UART_TX_DMA_IRQn);
// #endif
// #ifdef STLK_UART_RX_DMA_IRQn
//   /* NVIC configuration for DMA transfer complete interrupt (UARTx_RX) */
//   HAL_NVIC_SetPriority(STLK_UART_RX_DMA_IRQn, STLK_IRQPriority, 1);
//   HAL_NVIC_EnableIRQ(STLK_UART_RX_DMA_IRQn);
// #endif
// #ifdef STLK_UART_IRQn
//   /* NVIC configuration for global UART interrupt */
//   HAL_NVIC_SetPriority(STLK_UART_IRQn, STLK_IRQPriority, 0);
//   HAL_NVIC_EnableIRQ(STLK_UART_IRQn);
// #endif
// }
// /*-----------------------------------------------------------*/

// /**
//  * @brief Initializes peripherals used by the UART ST-Link VCOM driver.
//  *
//  * @param[out] hUart UART handle
//  * @param[in] hDmaRx RxDMA handle
//  * @param[in] hDmaTx TxDMA handle
//  */
// void STLINK_UART_Init(UART_HandleTypeDef *hUart,
//                       DMA_HandleTypeDef *hDmaRx,
//                       DMA_HandleTypeDef *hDmaTx)
// {
//   if (HAL_UART_GetState(hUart) == HAL_UART_STATE_RESET)
//   {
//     HAL_UART_RegisterCallback(hUart, HAL_UART_MSPINIT_CB_ID, STLINK_UART_MspInit);

//     /* Put the USART peripheral in the Asynchronous mode (UART Mode):
//      *     Word Length = 8 Bits
//      *     Stop Bit    = One Stop bit
//      *     Parity      = None
//      *     BaudRate    = 115200 baud
//      *     Hardware flow control disabled (RTS and CTS signals) */
//     hUart->Instance = STLK_UART;
//     hUart->Init.BaudRate = 115200;
//     hUart->Init.WordLength = UART_WORDLENGTH_8B;
//     hUart->Init.StopBits = UART_STOPBITS_1;
//     hUart->Init.Parity = UART_PARITY_NONE;

//     hUart->Init.Mode = UART_MODE_TX_RX;
//     hUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
//     hUart->Init.OverSampling = UART_OVERSAMPLING_16;

//     hUart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//     hUart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

//     /* Assign SPI DMAs */
//     hUart->hdmarx = hDmaRx;
//     hUart->hdmatx = hDmaTx;

//     /* Init the UART */
//     if (HAL_UART_Init(hUart) != HAL_OK)
//     {
//       vAssertCalled(__LINE__, __FUNCTION__);
//     }
//   }
// }
// /*-----------------------------------------------------------*/

// /**
//  * @brief UART MSP De-Initialization
//  *         This function frees the hardware resources:
//  *           - Disable the Peripheral's clock
//  *           - Revert GPIO and NVIC configuration to their default state
//  * @param[in] huart: UART handle pointer
//  */
// void STLINK_UART_MspDeInit(UART_HandleTypeDef *huart)
// {
//   /*##-1- Reset peripherals ##################################################*/
//   STLK_UART_FORCE_RESET();
//   STLK_UART_RELEASE_RESET();

//   /*##-2- Disable peripherals and GPIO Clocks #################################*/
//   HAL_GPIO_DeInit(STLK_UART_TX_GPIO_PORT, STLK_UART_TX_PIN);
//   HAL_GPIO_DeInit(STLK_UART_RX_GPIO_PORT, STLK_UART_RX_PIN);

//   /*##-3- Disable the DMA Streams ############################################*/
// #ifdef STLK_UART_RX_DMA_STREAM
//   // STLK_UART_RX_DMA_CLK_ENABLE();
//   /* De-Initialize the DMA associated to reception process */
//   if (huart->hdmarx != NULL)
//   {
//     HAL_DMA_DeInit(huart->hdmarx);
//   }
// #endif
// #ifdef STLK_UART_RX_TX_STREAM
//   // STLK_UART_TX_DMA_CLK_ENABLE();
//   /* De-Initialize the DMA associated to transmission process */
//   if (huart->hdmatx != NULL)
//   {
//     HAL_DMA_DeInit(huart->hdmatx);
//   }
// #endif

//   /*##-4- Disable the NVIC for USART and DMA #################################*/
// #ifdef STLK_UART_TX_DMA_IRQn
//   HAL_NVIC_DisableIRQ(STLK_UART_TX_DMA_IRQn);
// #endif
// #ifdef STLK_UART_RX_DMA_IRQn
//   HAL_NVIC_DisableIRQ(STLK_UART_RX_DMA_IRQn);
// #endif
// #ifdef STLK_UART_IRQn
//   HAL_NVIC_DisableIRQ(STLK_UART_IRQn);
// #endif
// }

// /**
//  * @brief DeInitializes peripherals used by the UART ST-Link VCOM driver.
//  *
//  * @param[in] hUart UART handle
//  */
// void STLINK_UART_DeInit(UART_HandleTypeDef *hUart)
// {
//   HAL_UART_RegisterCallback(hUart, HAL_UART_MSPDEINIT_CB_ID, STLINK_UART_MspDeInit);

//   /* DeInit the UART */
//   if (HAL_UART_DeInit(hUart) != HAL_OK)
//   {
//     vAssertCalled(__LINE__, __FUNCTION__);
//   }
// }
// /*-----------------------------------------------------------*/

// /**
//  * @brief Write data to ST-Link VCOM USART.
//  *
//  * @param[in] hUart UART handle
//  * @param[in] pBuffer: Pointer to data buffer
//  * @param[in] BufferSize: Amount of data to be sent
//  * @retval HAL status
//  */
// HAL_StatusTypeDef STLINK_UART_SendData(UART_HandleTypeDef *hUart, uint8_t *pBuffer, size_t BufferSize)
// {
//   HAL_StatusTypeDef status = HAL_OK;

//   status = HAL_UART_Transmit(hUart, (uint8_t *)pBuffer, BufferSize, 1000);
//   if (status != HAL_OK)
//   {
//     /* De-initialize the USART communication bus */
//     if (HAL_UART_DeInit(hUart) != HAL_OK)
//     {
//       vAssertCalled(__LINE__, __FUNCTION__);
//     }

//     /* Re-Initialize the USART communication bus */
//     if (HAL_UART_Init(hUart) != HAL_OK)
//     {
//       vAssertCalled(__LINE__, __FUNCTION__);
//     }
//   }
//   return status;
// }
// /*-----------------------------------------------------------*/

// /**
//  * @brief Write data to ST-Link VCOM USART.
//  *
//  * @param[in] hUart UART handle
//  * @param[in] pBuffer: Pointer to data buffer
//  * @param[in] BufferSize: Amount of data to be sent
//  * @retval HAL status
//  */
// HAL_StatusTypeDef STLINK_UART_SendData_DMA(UART_HandleTypeDef *hUart, uint8_t *pBuffer, size_t BufferSize)
// {
//   HAL_StatusTypeDef status = HAL_OK;

//   status = HAL_UART_Transmit_DMA(hUart, (uint8_t *)pBuffer, BufferSize);
//   if (status != HAL_OK)
//   {
//     /* Re-Initiaize the UART Bus */
//     UARTx_Error(hUart);
//   }
//   return status;
// }
// /*-----------------------------------------------------------*/

// /**
//  * @brief Write data to ST-Link VCOM USART.
//  *
//  * @param[in] hUart UART handle
//  * @param[in] pBuffer: Pointer to data buffer
//  * @param[in] BufferSize: Amount of data to be sent
//  * @retval HAL UART status
//  */
// HAL_UART_StateTypeDef STLINK_UART_GetState(UART_HandleTypeDef *hUart)
// {
//   return (HAL_UART_GetState(hUart));
// }
// /*-----------------------------------------------------------*/

// #endif /* STLINK_MODULE */

/******************************************************************************/
/*                               LAN8742 module                               */
/* 0/100 Base-T/TX Ethernet Transceiver with Cable Diagnostics                */
/* https://www.microchip.com/en-us/product/LAN8742                            */
/******************************************************************************/
#ifdef LAN8742A_MODULE

#ifdef LAN8742_nINTSEL_IRQn
/**
 * @brief This function handles external line interrupt request.
 */
void LAN8742_nINTSEL_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if (__HAL_GPIO_EXTI_GET_IT(LAN8742_nINTSEL_PIN) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(LAN8742_nINTSEL_PIN);
    // HAL_GPIO_EXTI_Callback(GPIO_Pin);
    HAL_ETH_EXTI_Callback();
  }
}
#endif /* LAN8742_nINTSEL_IRQn */

/**
 * @brief Initializes the ETH MSP.
 * @param[in] heth pointer to a ETH_HandleTypeDef structure that contains
 *         the configuration information for ETHERNET module
 */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* ETH clock enable (MAC, MACTX, MACRX) */
  __HAL_RCC_ETH_CLK_ENABLE();

#ifdef LAN8742_nINTSEL_PIN
  /* LAN8742A MAC interrupt used on PHY_LED2 */
  LAN8742_nINTSEL_GPIO_CLK_ENABLE();
#endif /* LAN8742_nINTSEL_PIN */

  /* LAN8742A RMII interface */
  RMII_REF_CLK_GPIO_CLK_ENABLE();
  RMII_MDIO_GPIO_CLK_ENABLE();
  RMII_MDC_GPIO_CLK_ENABLE();
  RMII_CRS_DV_GPIO_CLK_ENABLE();
  RMII_RXD0_GPIO_CLK_ENABLE();
  RMII_RXD1_GPIO_CLK_ENABLE();
#ifdef RMII_MII_RXER_PIN
  RMII_MII_RXER_GPIO_CLK_ENABLE();
#endif /* RMII_MII_RXER_PIN */
  RMII_TX_EN_GPIO_CLK_ENABLE();
  RMII_TXD0_GPIO_CLK_ENABLE();
  RMII_TXD1_GPIO_CLK_ENABLE();

  /*##-2- Configure peripheral GPIOs #########################################*/
#ifdef LAN8742_nINTSEL_PIN
  /* LAN8742A MAC interrupt used on PHY_LED2 */
  GPIO_InitStruct.Pin = LAN8742_nINTSEL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LAN8742_nINTSEL_PORT, &GPIO_InitStruct);
#endif /* LAN8742_nINTSEL_PIN */

  /* LAN8742A MAC RMII signals */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;

  GPIO_InitStruct.Pin = RMII_REF_CLK_PIN;
  HAL_GPIO_Init(RMII_REF_CLK_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RMII_MDIO_PIN;
  HAL_GPIO_Init(RMII_MDIO_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RMII_MDC_PIN;
  HAL_GPIO_Init(RMII_MDC_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RMII_CRS_DV_PIN;
  HAL_GPIO_Init(RMII_CRS_DV_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RMII_RXD0_PIN;
  HAL_GPIO_Init(RMII_RXD0_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RMII_RXD1_PIN;
  HAL_GPIO_Init(RMII_RXD1_GPIO_PORT, &GPIO_InitStruct);

#ifdef RMII_MII_RXER_PIN
  GPIO_InitStruct.Pin = RMII_MII_RXER_PIN;
  HAL_GPIO_Init(RMII_MII_RXER_GPIO_PORT, &GPIO_InitStruct);
#endif /* RMII_MII_RXER_PIN */

  GPIO_InitStruct.Pin = RMII_TX_EN_PIN;
  HAL_GPIO_Init(RMII_TX_EN_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RMII_TXD0_PIN;
  HAL_GPIO_Init(RMII_TXD0_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RMII_TXD1_PIN;
  HAL_GPIO_Init(RMII_TXD1_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the NVIC for ETH and GPIO (nINTSEL) ######################*/

  /* Enable the Ethernet global Interrupt */
  HAL_NVIC_SetPriority(ETH_IRQn, ETH_IRQPriority, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);

#ifdef LAN8742_WKUP_IRQn
  HAL_NVIC_SetPriority(LAN8742_WKUP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LAN8742_WKUP_IRQn);
#endif /* LAN8742_WKUP_IRQn */

#ifdef LAN8742_nINTSEL_IRQn
  /* Configure GPIO (nINTSEL) interrupt */
  HAL_NVIC_SetPriority(LAN8742_nINTSEL_IRQn, ETH_IRQPriority, 1);
  HAL_NVIC_EnableIRQ(LAN8742_nINTSEL_IRQn);
#endif /* LAN8742_nINTSEL_IRQn */
}
/*-----------------------------------------------------------*/

/**
 * @brief DeInitializes ETH MSP.
 * @param[in] heth pointer to a ETH_HandleTypeDef structure that contains
 *         the configuration information for ETHERNET module
 */
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
  /*##-1- Enable peripherals clock, keep GPIO clock enabled ##################*/
  __HAL_RCC_ETH_CLK_DISABLE();

  /*##-2- DeInitialize peripheral GPIOs ######################################*/
#ifdef LAN8742_nINTSEL_PIN
  /* LAN8742A MAC interrupt used on PHY_LED2 */
  HAL_GPIO_DeInit(LAN8742_nINTSEL_PORT, LAN8742_nINTSEL_PIN);
#endif /* LAN8742_nINTSEL_PIN */

  /* LAN8742A RMII interface */
  HAL_GPIO_DeInit(RMII_REF_CLK_GPIO_PORT, RMII_REF_CLK_PIN);
  HAL_GPIO_DeInit(RMII_MDIO_GPIO_PORT, RMII_MDIO_PIN);
  HAL_GPIO_DeInit(RMII_MDC_GPIO_PORT, RMII_MDC_PIN);
  HAL_GPIO_DeInit(RMII_CRS_DV_GPIO_PORT, RMII_CRS_DV_PIN);
  HAL_GPIO_DeInit(RMII_RXD0_GPIO_PORT, RMII_RXD0_PIN);
  HAL_GPIO_DeInit(RMII_RXD1_GPIO_PORT, RMII_RXD1_PIN);
#ifdef RMII_MII_RXER_PIN
  HAL_GPIO_DeInit(RMII_MII_RXER_GPIO_PORT, RMII_MII_RXER_PIN);
#endif /* RMII_MII_RXER_PIN */
  HAL_GPIO_DeInit(RMII_TX_EN_GPIO_PORT, RMII_TX_EN_PIN);
  HAL_GPIO_DeInit(RMII_TXD0_GPIO_PORT, RMII_TXD0_PIN);
  HAL_GPIO_DeInit(RMII_TXD1_GPIO_PORT, RMII_TXD1_PIN);

  /*##-2- Disable the NVIC for ETH and GPIO (nINTSEL) ########################*/

  /* Enable the Ethernet global Interrupt */
  HAL_NVIC_DisableIRQ(ETH_IRQn);
#ifdef LAN8742_WKUP_IRQn
  HAL_NVIC_DisableIRQ(LAN8742_WKUP_IRQn);
#endif /* LAN8742_WKUP_IRQn */
#ifdef LAN8742_nINTSEL_IRQn
  /* Configure GPIO (nINTSEL) interrupt */
  HAL_NVIC_DisableIRQ(LAN8742_nINTSEL_IRQn);
#endif /* LAN8742_nINTSEL_IRQn */
}
/*-----------------------------------------------------------*/

#endif /* LAN8742A_MODULE */

/********************************** END OF FILE *******************************/
