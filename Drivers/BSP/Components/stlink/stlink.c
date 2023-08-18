/**
 * @file stlink.c
 * @author horsche (horsche@li.plus)
 * @brief
 * @version
 * @date 2023-08-11
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "Components/stlink/stlink.h"
// #include "cmsis_os.h"

/* Standard includes. */
#include <string.h> /* memset */

/* Private typedefs ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief  Callback function for receive buffer complete (half complete). This
 *    function sends notification to task handle.
 *
 * @note The process counter must not be lower than half of buffer size,
 *    otherwise the this function indicates a race condition. Therefore, the
 *    user callback function must increse process counter as soon the data in
 *    receive buffer are processed successfully.
 *
 * @param[in] hDma Pointer to DMA handler.
 */
static void prvXferCpltCallback(UART_HandleTypeDef *hUart);

/* Private variables ---------------------------------------------------------*/

/**
 * @brief STLINK context.
 */
STLINKContext_t *pxSTLinkContext;

/* Private defines -----------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

const char *pucSTLinkStringStatus(const STLINKStatus_t xStatus)
{
  const char *str = NULL;

  switch (xStatus)
  {
  case STLINKFail:
    str = "STLINKFail";
    break;

  case STLINKSuccess:
    str = "STLINKSuccess";
    break;

  case STLINKBusy:
    str = "STLINKBusy";
    break;

  case STLINKBadParameter:
    str = "STLINKBadParameter";
    break;

  default:
    str = "STLINKInvalidStatus";
    break;
  }

  return str;
}
/*-----------------------------------------------------------*/

const char *pucSTLinkStringState(const STLINKState_t xState)
{
  const char *str = NULL;

  switch (xState)
  {

  case STLINKStateReset:
    str = "STLINKStateReset";
    break;

  case STLINKStateReady:
    str = "STLINKStateReady";
    break;

  case STLINKStateBusy:
    str = "STLINKStateBusy";
    break;

  case STLINKStateError:
    str = "STLINKStateError";
    break;

  case STLINKStateAbort:
    str = "STLINKStateAbort";
    break;

  default:
    str = "STLINKInvalidState";
    break;
  }

  return str;
}
/*-----------------------------------------------------------*/

/**
 * @brief UART MSP Initialization
 *         This function configures the hardware resources:
 *            - Peripheral's clock enable
 *            - Peripheral's GPIO Configuration
 * @param[in] hUart: UART handle pointer
 */
static void STLINK_UART_MspInit(UART_HandleTypeDef *hUart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = {0};

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  STLK_UART_TX_GPIO_CLK_ENABLE();
  STLK_UART_RX_GPIO_CLK_ENABLE();

  /* Select SysClk as source of USART clocks */
  RCC_PeriphClkInit.PeriphClockSelection = STLK_UART_RCC_PERIPHCLK;
  RCC_PeriphClkInit.Usart3ClockSelection = STLK_UART_RCC_CLKSOURCE;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  /* Enable USARTx clock */
  STLK_UART_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin = STLK_UART_TX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = STLK_UART_TX_AF;

  HAL_GPIO_Init(STLK_UART_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = STLK_UART_RX_PIN;
  GPIO_InitStruct.Alternate = STLK_UART_RX_AF;

  HAL_GPIO_Init(STLK_UART_RX_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the DMA ##################################################*/
#ifdef STLK_UART_TX_DMA_STREAM
  /* Enable DMA clock */
  STLK_UART_TX_DMA_CLK_ENABLE();

  /* Configure the DMA handler for transmission process */
  DMA_HandleTypeDef *hdma_tx = hUart->hdmatx;
  hdma_tx->Instance = STLK_UART_TX_DMA_STREAM;
  hdma_tx->Init.Channel = STLK_UART_TX_DMA_CHANNEL;
  hdma_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx->Init.Mode = DMA_NORMAL;
  hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;

  hdma_tx->Init.MemInc = DMA_MINC_ENABLE; /* Dynamic Memory: 1 x uint8_t */
  hdma_tx->Init.MemBurst = DMA_MBURST_INC4;
  hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;

  hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE; /* Static Peripheral: 1 x uint8_t */
  hdma_tx->Init.PeriphBurst = DMA_PBURST_INC4;
  hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;

  hdma_tx->Init.Priority = DMA_PRIORITY_LOW;

  /* Init the DMA */
  if (HAL_DMA_Init(hdma_tx) != HAL_OK)
  {
    vAssertCalled(__LINE__, __FUNCTION__);
  }

  /* Associate the initialized DMA handle to the UART handle */
  // __HAL_LINKDMA(hUart, hdmatx, *hdma_tx);
  hdma_tx->Parent = hUart;
  // HAL_DMA_RegisterCallback(&hdma_tx, HAL_DMA_XFER_CPLT_CB_ID, HAL_DMA_TransferCpltCallback);
  // HAL_DMA_RegisterCallback(&hdma_tx, HAL_DMA_XFER_ERROR_CB_ID, HAL_DMA_ErrorCallback);
#endif

#ifdef STLK_UART_RX_DMA_STREAM
  /* Enable DMA clock */
  STLK_UART_RX_DMA_CLK_ENABLE();

  /* Configure the DMA handler for reception process */
  DMA_HandleTypeDef *hdma_rx = hUart->hdmarx;
  hdma_rx->Instance = STLK_UART_RX_DMA_STREAM;
  hdma_rx->Init.Channel = STLK_UART_RX_DMA_CHANNEL;
  hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
  hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_rx->Init.Mode = DMA_NORMAL;
  hdma_rx->Init.Priority = DMA_PRIORITY_HIGH;
  hdma_rx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_rx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_rx->Init.MemBurst = DMA_MBURST_INC4;
  hdma_rx->Init.PeriphBurst = DMA_PBURST_INC4;

  /* Init the DMA */
  if (HAL_DMA_Init(hdma_rx) != HAL_OK)
  {
    vAssertCalled(__LINE__, __FUNCTION__);
  }

  /* Associate the initialized DMA handle to the the UART handle */
  hdma_rx->Parent = hUart;
  // __HAL_LINKDMA(hUart, hdmarx, hdma_rx);
  // HAL_DMA_RegisterCallback(&hdma_rx, HAL_DMA_XFER_CPLT_CB_ID, HAL_DMA_TransferCpltCallback);
  // HAL_DMA_RegisterCallback(&hdma_rx, HAL_DMA_XFER_ERROR_CB_ID, HAL_DMA_ErrorCallback);
#endif

  /*##-4- Configure the NVIC for USART and DMA ###############################*/
#ifdef STLK_UART_TX_DMA_IRQn
  /* NVIC configuration for DMA transfer complete interrupt (UARTx_TX) */
  HAL_NVIC_SetPriority(STLK_UART_TX_DMA_IRQn, STLK_IRQPriority, 2);
  HAL_NVIC_EnableIRQ(STLK_UART_TX_DMA_IRQn);
#endif
#ifdef STLK_UART_RX_DMA_IRQn
  /* NVIC configuration for DMA transfer complete interrupt (UARTx_RX) */
  HAL_NVIC_SetPriority(STLK_UART_RX_DMA_IRQn, STLK_IRQPriority, 1);
  HAL_NVIC_EnableIRQ(STLK_UART_RX_DMA_IRQn);
#endif
#ifdef STLK_UART_IRQn
  /* NVIC configuration for global UART interrupt */
  HAL_NVIC_SetPriority(STLK_UART_IRQn, STLK_IRQPriority, 0);
  HAL_NVIC_EnableIRQ(STLK_UART_IRQn);
#endif
}
/*-----------------------------------------------------------*/

STLINKStatus_t xSTLinkInit(STLINKContext_t *pxContext, void *pvTaskHandle)
{
  STLINKStatus_t xStatus = STLINKSuccess;

  /* Validate arguments. */
  if ((pxContext == NULL) || (pvTaskHandle == NULL))
  {
    LogError(("Argument cannot be NULL: pxContext=%p / pvTaskHandle=%p",
              (void *)pxContext, (void *)pvTaskHandle));
    xStatus = STLINKBadParameter;
  }
  else
  {
    /* Assign context to global context pointer */
    pxSTLinkContext = pxContext;
  }

  if (xStatus == STLINKSuccess)
  {
    (void)memset(pxContext, 0x00, sizeof(STLINKContext_t));

    /* UART Initialization */
    if (HAL_UART_GetState(&(pxContext->hUart)) == HAL_UART_STATE_RESET)
    {
      HAL_UART_RegisterCallback(&(pxContext->hUart), HAL_UART_MSPINIT_CB_ID, STLINK_UART_MspInit);

      /* Put the USART peripheral in the Asynchronous mode (UART Mode):
       *     Word Length = 8 Bits
       *     Stop Bit    = One Stop bit
       *     Parity      = None
       *     BaudRate    = 115200 baud
       *     Hardware flow control disabled (RTS and CTS signals) */
      pxContext->hUart.Instance = STLK_UART;
      pxContext->hUart.Init.BaudRate = 115200;
      pxContext->hUart.Init.WordLength = UART_WORDLENGTH_8B;
      pxContext->hUart.Init.StopBits = UART_STOPBITS_1;
      pxContext->hUart.Init.Parity = UART_PARITY_NONE;

      pxContext->hUart.Init.Mode = UART_MODE_TX_RX;
      pxContext->hUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      pxContext->hUart.Init.OverSampling = UART_OVERSAMPLING_16;
      
      pxContext->hUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
      pxContext->hUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

      /* Assign SPI DMAs */
      pxContext->hUart.hdmarx = &(pxContext->hDmaRx);
      pxContext->hUart.hdmatx = &(pxContext->hDmaTx);

      /* Init the UART */
      if (HAL_UART_Init(&(pxContext->hUart)) != HAL_OK)
      {
        vAssertCalled(__LINE__, __FUNCTION__);
      }
    }

    LogDebug(("Peripheral initialized: hUart=%p",
              &(pxContext->hUart)));

    /* Set interrupt functions. */
    if (HAL_UART_GetState(&(pxContext->hUart)) != HAL_UART_STATE_BUSY)
    {
      UART_InitCallbacksToDefault(&(pxContext->hUart));
      // HAL_UART_RegisterCallback(&(pxContext->hUart), HAL_UART_RX_HALFCOMPLETE_CB_ID, pCallback);
      // HAL_UART_RegisterCallback(&(pxContext->hUart), HAL_UART_RX_COMPLETE_CB_ID, pCallback);
      HAL_UART_RegisterCallback(&(pxContext->hUart), HAL_UART_TX_HALFCOMPLETE_CB_ID, prvXferCpltCallback);
      HAL_UART_RegisterCallback(&(pxContext->hUart), HAL_UART_TX_COMPLETE_CB_ID, prvXferCpltCallback);
    }

    /* Set task handle for interrupt notification. */
    pxContext->pvTaskHandle = pvTaskHandle;
  }

  if (xStatus == STLINKSuccess)
  {
    /* Set state ready */
    pxContext->xState = STLINKStateReady;
  }

  return xStatus;
}
/*-----------------------------------------------------------*/

STLINKStatus_t xSTLinkDeInit(STLINKContext_t *pxContext)
{
  /* UART Deinitialization */
  STLINK_UART_DeInit(&(pxContext->hUart));

  return STLINKSuccess;
}
/*-----------------------------------------------------------*/

STLINKStatus_t xSTLinkIsReady(STLINKContext_t *pxContext)
{
  HAL_UART_StateTypeDef xStatus;
  xStatus = HAL_UART_GetState(&(pxContext->hUart));
  if (xStatus != HAL_UART_STATE_READY)
  {
    return STLINKBusy;
  }
  return STLINKSuccess;
}
/*-----------------------------------------------------------*/

STLINKStatus_t xSTLinkSend(STLINKContext_t *pxContext, uint8_t *pucBuffer, size_t xReceivedBytes)
{
  HAL_StatusTypeDef xStatus = HAL_OK;
  xStatus = HAL_UART_Transmit(&(pxContext->hUart), pucBuffer, xReceivedBytes, 1000);

  // if (STLINK_UART_SendData_DMA(&(pxSTLinkContext->hUart), pucBuffer, xReceivedBytes) != HAL_OK)
  if (xStatus != HAL_OK)
  {
    pxContext->xState = STLINKStateError;
    return STLINKFail;
  }
  pxContext->xState = STLINKStateBusy;

  return STLINKSuccess;
}
/*-----------------------------------------------------------*/

static void prvXferCpltCallback(UART_HandleTypeDef *hUart)
{
  // pxSTLinkContext->xState = STLINKStateReady;

  // BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // /* Notify calculation task that SPI receive completed. */
  // if (pxSTLinkContext->pvTaskHandle != NULL)
  // {
  //   /* Notify the task that the transmission is complete. */
  //   vTaskNotifyGiveFromISR(pxSTLinkContext->pvTaskHandle,
  //                          &xHigherPriorityTaskWoken);
  //   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  // }
}
/*-----------------------------------------------------------*/

/********************************* END OF FILE ********************************/
