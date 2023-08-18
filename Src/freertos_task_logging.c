/**
 * @file logging.c
 * @author horsche (horsche@li.plus)
 * @brief
 * @version 0.1
 * @date 2021-11-26
 *
 * @copyright Copyright (c) 2021
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Components/stlink/stlink.h"

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
#include "freertos_task_logging.h"
#include "logging_levels.h"

/* Private define ------------------------------------------------------------*/

/**
 * @brief Task name listed in task manager.
 */
#define loggerTASK_NAME "LOG"

/**
 * @brief Stack size needed for prvTaskLed(), a bit of a guess.
 *    Need ~500 byte due to printf (logging)
 */
#define loggerTASK_STACK_SIZE (5 * configMINIMAL_STACK_SIZE)

/**
 * @brief The priority of prvTaskLed().
 */
#define loggerTASK_PRIORITY 8

/* Used to dimension the array used to hold the messages.  The available space
 * will actually be one less than this. */
#define loggingSTORAGE_SIZE_BYTES (2048)
#define loggingMAX_MESSAGE_SIZE (256)

/* A block time of zero simply means don't block. */
#define dlDONT_BLOCK 0

/* Private variables ---------------------------------------------------------*/
TaskHandle_t xLoggerTaskHandle;

static STLINKContext_t xSTLinkContext;

/* Global message buffers handle */
static MessageBufferHandle_t xMessageBuffer;

static char ucStaticTxBuffer[loggingMAX_MESSAGE_SIZE];
static char ucStaticRxBuffer[loggingMAX_MESSAGE_SIZE];

/* Counter for successfully submitted and dropped messages */
static uint32_t ulDropCount;
static uint32_t ulUDPTxSuccessCount;
static uint32_t ulUDPTxErrorCount;

/* Stores the selected logging targets passed in as parameters to the
 * vLoggingInit() function. */
BaseType_t xUARTLoggingUsed = pdTRUE, xSWOLoggingUsed = pdFALSE, xUDPLoggingUsed = pdTRUE;

/* The UDP socket and address on/to which print messages are sent. */
Socket_t xPrintSocket = FREERTOS_INVALID_SOCKET;
struct freertos_sockaddr xPrintUDPAddress;

/* Private function prototypes -----------------------------------------------*/
/*
 * The logging thread that performs the actual writing of messages.
 */
static void prvTaskLogging(void *pvParam);

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

    /* Creates a new message buffer using dynamically allocated memory. */
    xMessageBuffer = xMessageBufferCreate(loggingSTORAGE_SIZE_BYTES);
    if (xMessageBuffer == NULL)
    {
      /* Message Queue object not created, handle failure */
      vAssertCalled(__LINE__, __FUNCTION__);
    }

    if (xLoggerTaskHandle == NULL)
    {
      /* Create the logging thread. */
      xTaskCreate(prvTaskLogging,
                  loggerTASK_NAME,
                  loggerTASK_STACK_SIZE,
                  NULL,
                  loggerTASK_PRIORITY,
                  &xLoggerTaskHandle);
    }

    /* If UART logging is used then initialise peripheral. */
    if (xUARTLoggingUsed != pdFALSE)
    {
      /* Initialize UART ST-Link VCOM driver. */
      if (xSTLinkInit(&xSTLinkContext, (void *)xLoggerTaskHandle) != STLINKSuccess)
      {
        /* Disable UART logging since initializing ST-Link failed. */
        xUARTLoggingUsed = pdFALSE;
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

void vSetUDPAddress(const uint8_t ucRemoteIPAddress[ipIP_ADDRESS_LENGTH_BYTES],
                    const uint16_t usRemotePort)
{
  /* Enable UDP logging in any case. */
  xUDPLoggingUsed = pdTRUE;

  /* Set the address to which the print messages are sent. */
  xPrintUDPAddress.sin_addr = FreeRTOS_inet_addr_quick(ucRemoteIPAddress[0], ucRemoteIPAddress[1], ucRemoteIPAddress[2], ucRemoteIPAddress[3]);
  xPrintUDPAddress.sin_port = FreeRTOS_htons(usRemotePort);
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
  UBaseType_t uxSavedInterruptStatus;

  if (xMessageBuffer == NULL)
  {
    /* Message buffer not initialized yet. */
    ulDropCount++;
    return;
  }

  /* Parse logging message using printf (this function blows) */
  size_t xTransmitBytes = vsnprintf(ucStaticTxBuffer, sizeof(ucStaticTxBuffer), pcFormat, xArgs);

  /* Call taskENTER_CRITICAL_FROM_ISR() to create a critical section, saving the
   * returned value into a local stack variable. */
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  /* Attempt to send the string to the message buffer. */
  size_t xBytesSent = xMessageBufferSendFromISR(xMessageBuffer,
                                                (void *)&ucStaticTxBuffer,
                                                FreeRTOS_min_uint32(xTransmitBytes, sizeof(ucStaticTxBuffer)),
                                                &xHigherPriorityTaskWoken);

  /* The operation that required the critical section is complete so exit the
   * critical section.  Assuming interrupts were enabled on entry to this ISR,
   * the value saved in uxSavedInterruptStatus will result in interrupts being
   * re-enabled. */
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

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
  // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/

static void prvTaskLogging(void *pvParameters)
{
  size_t xReceivedBytes;
  uint32_t ulSTLinkReady = 1;
  const TickType_t xBlockTime = pdMS_TO_TICKS(10);

  /* Infinite loop */
  for (;;)
  {
    // /* Wait for previous message to be send completely. */
    // if (xSTLinkIsReady(pxSTLinkContext) != STLINKSuccess)
    // {
    //   /* Wait to be notified that the transmission is complete.  Note
    //    * the first parameter is pdTRUE, which has the effect of clearing
    //    * the task's notification value back to 0, making the notification
    //    * value act like a binary (rather than a counting) semaphore.  */
    //   ulSTLinkReady = ulTaskNotifyTake(pdTRUE, xBlockTime);
    // }
    // else
    // {
    //   /* ST-Link is not busy, no need to wait for transfer complete */
    //   ulSTLinkReady = 1;
    // }

    /* Receive the next message from the message buffer.  Wait in the Blocked
     * state (so not using any CPU processing time) for a maximum of 100ms for
     * a message to become available. */
    xReceivedBytes = xMessageBufferReceive(xMessageBuffer,
                                           (void *)ucStaticRxBuffer,
                                           sizeof(ucStaticRxBuffer),
                                           xBlockTime);

    if (xReceivedBytes > 0)
    {
      /* A ucStaticRxBuffer contains a message that is xReceivedBytes long.  Process
       * the message here... */

      /* Enable red LED to indicate data transfer */
      LED_RED_On();

      /* Send message via UART */
      if (xUARTLoggingUsed != pdFALSE && ulSTLinkReady == 1)
      {
        xSTLinkSend(pxSTLinkContext, (uint8_t *)&ucStaticRxBuffer, xReceivedBytes);
      }

      /* Send message via SWO */
      if (xSWOLoggingUsed != pdFALSE)
      {
        for (int DataIdx = 0; DataIdx < xReceivedBytes; DataIdx++)
        {
          ITM_SendChar(ucStaticRxBuffer[DataIdx]);
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
              memcpy((uint8_t *)pucBuffer, ucStaticRxBuffer, xReceivedBytes);
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
            BaseType_t lReturned = FreeRTOS_sendto(xPrintSocket, ucStaticRxBuffer, xReceivedBytes, 0, &xPrintUDPAddress, sizeof(xPrintUDPAddress));
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
      LED_RED_Off();
    }
  }
}
/*-----------------------------------------------------------*/

/***************************** END OF FILE ************************************/
