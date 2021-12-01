/**
 * @file logging.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
// #include <stdio.h> /* use printf.c for embedded systems https://github.com/mpaland/printf */
#include "printf.h"
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <ctype.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"
#include "message_buffer.h"

/* FreeRTOS+TCP includes. */
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

/* Logging includes. */
#include "logging.h"
#include "logging_levels.h"

/* Private define ------------------------------------------------------------*/
/* Used to dimension the array used to hold the messages.  The available space
 * will actually be one less than this, so 999. */
#define loggingSTORAGE_SIZE_BYTES (1024)
#define loggingMAX_MESSAGE_SIZE (128)

/* A block time of zero simply means don't block. */
#define dlDONT_BLOCK 0

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart; /* global UART (ST-Link VCOM) handler */

/* Global message buffers handle */
MessageBufferHandle_t xMessageBuffer;

/* The variable used to hold the message buffer structure. */
StaticMessageBuffer_t xMessageBufferStruct;

/* Defines the memory that will actually hold the messages within the message
 * buffer.  Should be one more than the value passed in the xBufferSizeBytes
 * parameter. */
static uint8_t ucStorageBuffer[loggingSTORAGE_SIZE_BYTES];

/* Counter for successfully submitted and dropped messages */
static uint32_t ulDropCount;
static uint32_t ulUDPTxSuccessCount;
static uint32_t ulUDPTxErrorCount;

/* Stores the selected logging targets passed in as parameters to the
 * vLoggingInit() function. */
BaseType_t xUARTLoggingUsed = pdTRUE, xSWOLoggingUsed = pdFALSE, xUDPLoggingUsed = pdFALSE;

/* The UDP socket and address on/to which print messages are sent. */
Socket_t xPrintSocket = FREERTOS_INVALID_SOCKET;
struct freertos_sockaddr xPrintUDPAddress;

osThreadId_t xLoggerTaskHandle;
const osThreadAttr_t xLoggerTaskAttributes =
    {
        .name = "logger",
        .stack_size = (configMINIMAL_STACK_SIZE * 5),
        .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/*
 * The logging thread that performs the actual writing of messages.
 */
static void prvLoggingThread(void *pvParam);

/*
 * Creates the socket to which UDP messages are sent.  This function is not
 * called directly to prevent the print socket being created from within the IP
 * task - which could result in a deadlock.  Instead the function call is
 * deferred to run in the RTOS daemon task - hence it prototype.
 */
static void prvCreatePrintSocket(void *pvParameter1, uint32_t ulParameter2);

/*
 * Write a messages to stdout, either with or without a time-stamp.
 * The logging thread will finally call printf() and fflush().
 */
static void prvLoggingPrintf(const char *pcFormat, va_list xArgs);

/* User code -----------------------------------------------------------------*/
void vLoggingInit(BaseType_t xLogToUART,
                  BaseType_t xLogToSWO,
                  BaseType_t xLogToUDP,
                  uint32_t ulRemoteIPAddress, uint16_t usRemotePort)
{
  /* Can only be called before the scheduler has started. */
  configASSERT(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED);

#if ((ipconfigHAS_DEBUG_PRINTF == 1) || (ipconfigHAS_PRINTF == 1))
  {
    /* Record which output methods are to be used. */
    xUARTLoggingUsed = xLogToUART;
    xSWOLoggingUsed = xLogToSWO;
    xUDPLoggingUsed = xLogToUDP;

    /* If UART logging is used then initialise peripheral. */
    if (xUARTLoggingUsed != pdFALSE)
    {
      /* Put the USART peripheral in the Asynchronous mode (UART Mode):
       * - Word Length = 8 Bits
       * - Stop Bit    = One Stop bit
       * - Parity      = None
       * - BaudRate    = 115200 baud
       * - Hardware flow control disabled (RTS and CTS signals) */
      huart.Instance = STLK_USART;
      huart.Init.BaudRate = 115200;
      huart.Init.WordLength = UART_WORDLENGTH_8B;
      huart.Init.StopBits = UART_STOPBITS_1;
      huart.Init.Parity = UART_PARITY_NONE;
      huart.Init.Mode = UART_MODE_TX_RX;
      huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      huart.Init.OverSampling = UART_OVERSAMPLING_16;
      huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
      huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
      if (HAL_UART_Init(&huart) != HAL_OK)
      {
        Error_Handler();
      }
    }

    /* If SWO logging is used then initialise peripheral. */
    if (xSWOLoggingUsed != pdFALSE)
    {
      /* Noting to initialise here. */
    }

    /* If UDP logging is used then store the address to which the log data
     * will be sent - but don't create the socket yet because the network is
     * not initialised. */
    if (xUDPLoggingUsed != pdFALSE)
    {
      /* Set the address to which the print messages are sent. */
      xPrintUDPAddress.sin_port = FreeRTOS_htons(usRemotePort);
      xPrintUDPAddress.sin_addr = ulRemoteIPAddress;
    }

    /* As neither the pucMessageBufferStorageArea or pxStaticMessageBuffer
     * parameters were NULL, xMessageBuffer will not be NULL, and can be used to
     * reference the created message buffer in other message buffer API calls. */
    xMessageBuffer = xMessageBufferCreateStatic(sizeof(ucStorageBuffer),
                                                ucStorageBuffer,
                                                &xMessageBufferStruct);
    if (xMessageBuffer == NULL)
    {
      /* Message Queue object not created, handle failure */
      Error_Handler();
    }

    if (xLoggerTaskHandle == NULL)
    {
      /* Create the logging thread. */
      xLoggerTaskHandle = osThreadNew(prvLoggingThread, NULL, &xLoggerTaskAttributes);
    }
  }
#else  /* if ( ( ipconfigHAS_DEBUG_PRINTF == 1 ) || ( ipconfigHAS_PRINTF == 1 ) ) */
  {
    /* FreeRTOSIPConfig is set such that no print messages will be output.
     * Avoid compiler warnings about unused parameters. */
    (void)xLogToUART;
    (void)xLogToSWO;
    (void)xLogToUDP;
    (void)ulRemoteIPAddress;
    (void)usRemotePort;
  }
#endif /* ( ipconfigHAS_DEBUG_PRINTF == 1 ) || ( ipconfigHAS_PRINTF == 1 )  */
}
/*-----------------------------------------------------------*/

void vLoggingPrintf(const char *pcFormat, ...)
{
  va_list xArgs;

  va_start(xArgs, pcFormat);
  prvLoggingPrintf(pcFormat, xArgs);
  va_end(xArgs);
}
/*-----------------------------------------------------------*/

static void prvCreatePrintSocket(void *pvParameter1, uint32_t ulParameter2)
{
  static const TickType_t xSendTimeOut = pdMS_TO_TICKS(0);
  Socket_t xSocket;

  /* The function prototype is that of a deferred function, but the parameters
   * are not actually used. */
  (void)pvParameter1;
  (void)ulParameter2;

  xSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

  if (xSocket != FREERTOS_INVALID_SOCKET)
  {
    /* FreeRTOS+TCP decides which port to bind to. */
    FreeRTOS_setsockopt(xSocket, 0, FREERTOS_SO_SNDTIMEO, &xSendTimeOut, sizeof(xSendTimeOut));
    FreeRTOS_bind(xSocket, NULL, 0);

    /* Now the socket is bound it can be assigned to the print socket. */
    xPrintSocket = xSocket;
  }
}
/*-----------------------------------------------------------*/

static void prvLoggingPrintf(const char *pcFormat, va_list xArgs)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE; /* Initialised to pdFALSE. */
  char ucTxBuffer[loggingMAX_MESSAGE_SIZE];

  /* Parse logging message using printf (this function blows) */
  size_t xTransmitBytes = vsnprintf(ucTxBuffer, sizeof(ucTxBuffer), pcFormat, xArgs);

  /* Attempt to send the string to the message buffer. */
  size_t xBytesSent = xMessageBufferSendFromISR(xMessageBuffer, (void *)&ucTxBuffer, xTransmitBytes, &xHigherPriorityTaskWoken);
  if (xBytesSent != xTransmitBytes)
  {
    /* The string could not be added to the message buffer because there was
     * not enough free space in the buffer. */
    ulDropCount++;
  }

  /* If xHigherPriorityTaskWoken was set to pdTRUE inside
   * xMessageBufferSendFromISR() then a task that has a priority above the
   * priority of the currently executing task was unblocked and a context
   * switch should be performed to ensure the ISR returns to the unblocked
   * task.  In most FreeRTOS ports this is done by simply passing
   * xHigherPriorityTaskWoken into taskYIELD_FROM_ISR(), which will test the
   * variables value, and perform the context switch if necessary.  Check the
   * documentation for the port in use for port specific instructions. */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/

static void prvLoggingThread(void *pvParameters)
{
  char ucRxBuffer[loggingMAX_MESSAGE_SIZE];
  size_t xReceivedBytes;
  const TickType_t xBlockTime = pdMS_TO_TICKS(20);

  /* Infinite loop */
  for (;;)
  {
    /* Receive the next message from the message buffer.  Wait in the Blocked
     * state (so not using any CPU processing time) for a maximum of 100ms for
     * a message to become available. */
    xReceivedBytes = xMessageBufferReceive(xMessageBuffer, (void *)ucRxBuffer, sizeof(ucRxBuffer), xBlockTime);

    if (xReceivedBytes > 0)
    {
      /* A ucRxBuffer contains a message that is xReceivedBytes long.  Process
       * the message here... */

      /* Enable red LED to indicate data transfer */
      BSP_LED_On(LED_RED);

      /* Send message via UART */
      if (xUARTLoggingUsed != pdFALSE)
      {
        HAL_UART_Transmit(&huart, (uint8_t *)&ucRxBuffer, xReceivedBytes, 100);
      }

      /* Send message via SWO */
      if (xSWOLoggingUsed != pdFALSE)
      {
        for (int DataIdx = 0; DataIdx < xReceivedBytes; DataIdx++)
        {
          ITM_SendChar(ucRxBuffer[DataIdx]);
        }
      }

      /* If the message is to be logged to a UDP port then it can be sent directly
       * because it only uses FreeRTOS function. */
      if (xUDPLoggingUsed != pdFALSE)
      {
        if ((xPrintSocket == FREERTOS_INVALID_SOCKET) && (FreeRTOS_IsNetworkUp() != pdFALSE))
        {
          /* Create and bind the socket to which print messages are sent.  The
           * xTimerPendFunctionCall() function is used even though this is
           * not an interrupt because this function is called from the IP task
           * and the IP task cannot itself wait for a socket to bind.  The
           * parameters to prvCreatePrintSocket() are not required so set to
           * NULL or 0. */
          xTimerPendFunctionCall(prvCreatePrintSocket, NULL, 0, dlDONT_BLOCK);
        }

        if (xPrintSocket != FREERTOS_INVALID_SOCKET)
        {
#if (ipconfigZERO_COPY_TX_DRIVER != 0)
          {
            uint8_t *pucBuffer;
            /* This RTOS task is going to send using the zero copy interface.  The
             * data being sent is therefore written directly into a buffer that is
             * passed into, rather than copied into, the FreeRTOS_sendto()
             * function.
             * First obtain a buffer of adequate length from the TCP/IP stack into which
             * the string will be written. */
            pucBuffer = FreeRTOS_GetUDPPayloadBuffer(xReceivedBytes, portMAX_DELAY);

            /* Check a buffer was obtained. */
            if (pucBuffer != 0)
            {
              /* Create the string that is sent. */
              memcpy((uint8_t *)pucBuffer, ucRxBuffer, xReceivedBytes);
              // memset(pucBuffer, 0x00, xStringLength);
              // sprintf(pucBuffer, "%s%lurn", ucStringToSend, ulCount);

              /* Pass the buffer into the send function.  ulFlags has the
               * FREERTOS_ZERO_COPY bit set so the TCP/IP stack will take control of the
               * buffer rather than copy data out of the buffer. */
              BaseType_t lReturned = FreeRTOS_sendto(xPrintSocket, (void *)pucBuffer, xReceivedBytes, FREERTOS_ZERO_COPY, &xPrintUDPAddress, sizeof(xPrintUDPAddress));
              if (lReturned == 0)
              {
                /* The send operation failed. Adjust TxError counter. */
                ulUDPTxErrorCount++;

                /* The send operation failed, so this RTOS task is still responsible
                 * for the buffer obtained from the TCP/IP stack.  To ensure the buffer
                 * is not lost it must either be used again, or, as in this case,
                 * returned to the TCP/IP stack using FreeRTOS_ReleaseUDPPayloadBuffer().
                 * pucBuffer can be safely re-used after this call. */
                FreeRTOS_ReleaseUDPPayloadBuffer((void *)pucBuffer);
              }
              else
              {
                /* Adjust TxSuccess counter */
                ulUDPTxSuccessCount++;

                /* The send was successful so the TCP/IP stack is now managing the
                 * buffer pointed to by pucBuffer, and the TCP/IP stack will
                 * return the buffer once it has been sent.  pucBuffer can
                 * be safely re-used. */
              }
            }
          }
#else
          {
            /* Zero copy interface is disabled. Use ipconfigZERO_COPY_TX_DRIVER
             * to allow zero copy access.
             * Send the buffer with ulFlags set to 0, so the FREERTOS_ZERO_COPY bit
             * is clear. */
            BaseType_t lReturned = FreeRTOS_sendto(xPrintSocket, ucRxBuffer, xReceivedBytes, 0, &xPrintUDPAddress, sizeof(xPrintUDPAddress));
            if (lReturned != xReceivedBytes)
            {
              /* The send operation failed. Adjust TxError counter. */
              ulUDPTxErrorCount++;
            }
            else
            {
              /* Adjust TxSuccess counter */
              ulUDPTxSuccessCount++;
            }
          }
#endif
        }
      }

      /* Disable red LED after data transfer */
      BSP_LED_Off(LED_RED);
    }
  }
}
/*-----------------------------------------------------------*/

/***************************** END OF FILE ************************************/
