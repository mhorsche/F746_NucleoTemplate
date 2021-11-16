/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <ctype.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"
#include "stream_buffer.h"

/* FreeRTOS+TCP includes. */
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

/* Logging includes. */
#include "logging.h"
#include "logging_levels.h"

/* Private define ------------------------------------------------------------*/
/* Dimensions the arrays into which print messages are created. */
#define dlMAX_PRINT_STRING_LENGTH 255

/* The size of the stream buffer used to pass messages from FreeRTOS tasks to
 * the thread that is responsible for making any calls that are necessary for the 
 * selected logging method. */
#define dSTREAM_BUFFER_LENGTH_BYTES ((size_t)100)
#define dSTREAM_BUFFER_TRIGGER_LEVEL_10 ((BaseType_t)10)
#define LOG_MESSAGE_COUNT 16

/* A block time of zero simply means don't block. */
#define dlDONT_BLOCK 0

/* Message object structure */
typedef struct
{
  size_t len;
  char str[dSTREAM_BUFFER_LENGTH_BYTES];
} msgqueue_obj_t;

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart; /* global UART (ST-Link VCOM) handler */

osMessageQueueId_t mid_MsgQueue = NULL;

osThreadId_t tid_logger;
const osThreadAttr_t LoggerTask_attributes =
    {
        .name = "logger",
        .stack_size = configMINIMAL_STACK_SIZE * 5,
        .priority = (osPriority_t)osPriorityLow,
};

/* Stores the selected logging targets passed in as parameters to the
 * vLoggingInit() function. */
BaseType_t xUARTLoggingUsed = pdTRUE,
           xSWOLoggingUsed = pdFALSE,
           xUDPLoggingUsed = pdFALSE;

/* The stream buffer that is used to send data from an interrupt to the task. */
// static StreamBufferHandle_t xStreamBuffer = NULL;

/* The UDP socket and address on/to which print messages are sent. */
Socket_t xPrintSocket = FREERTOS_INVALID_SOCKET;
struct freertos_sockaddr xPrintUDPAddress;

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
      /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
      /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit    = One Stop bit
      - Parity      = None
      - BaudRate    = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
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

    /* If UDP logging is used then store the address to which the log data
     * will be sent - but don't create the socket yet because the network is
     * not initialised. */
    if (xUDPLoggingUsed != pdFALSE)
    {
      /* Set the address to which the print messages are sent. */
      xPrintUDPAddress.sin_port = FreeRTOS_htons(usRemotePort);
      xPrintUDPAddress.sin_addr = ulRemoteIPAddress;
    }

    /* If a disk file or stdout are to be used then Win32 system calls will
     * have to be made.  Such system calls cannot be made from FreeRTOS tasks
     * so create a stream buffer to pass the messages to a Win32 thread, then
     * create the thread itself, along with a Win32 event that can be used to
     * unblock the thread. */
    if ((xUARTLoggingUsed != pdFALSE) || (xSWOLoggingUsed != pdFALSE))
    {
      /* Create the message queue */
      mid_MsgQueue = osMessageQueueNew(LOG_MESSAGE_COUNT, sizeof(msgqueue_obj_t), NULL);
      if (mid_MsgQueue == NULL)
      {
        /* Message Queue object not created, handle failure */
        Error_Handler();
      }

      /* Create the stream buffer that sends data from the interrupt to the
       * task, and create the task. */
      // xStreamBuffer = xStreamBufferCreate(/* The buffer length in bytes. */
      //                                     dSTREAM_BUFFER_LENGTH_BYTES,
      //                                     /* The stream buffer's trigger level. */
      //                                     dSTREAM_BUFFER_TRIGGER_LEVEL_10);

      //   /* Create the buffer. */
      //   xLogStreamBuffer = (StreamBuffer_t *)malloc(
      //       sizeof(*xLogStreamBuffer) -
      //       sizeof(xLogStreamBuffer->ucArray) +
      //       dlLOGGING_STREAM_BUFFER_SIZE + 1);
      //   configASSERT(xLogStreamBuffer);
      //   memset(
      //       xLogStreamBuffer,
      //       '\0',
      //       sizeof(*xLogStreamBuffer) - sizeof(xLogStreamBuffer->ucArray));
      //   xLogStreamBuffer->LENGTH = dlLOGGING_STREAM_BUFFER_SIZE + 1;

      /* Create the logging thread */
      tid_logger = osThreadNew(prvLoggingThread, NULL, &LoggerTask_attributes);
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
  osStatus_t status;
  msgqueue_obj_t msg;

  memset((uint8_t *)msg.str, 0x00, sizeof(msg.str));
  size_t len = vsnprintf(msg.str, sizeof(msg.str), pcFormat, xArgs);
  msg.len = len;

  /* Append message to message queue */
  status = osMessageQueuePut(mid_MsgQueue, &msg, 0U, 0U);
  if (status != osOK)
  {
    /* osErrorTimeout: the message could not be put into the queue in the given time (wait-timed semantics).
     * osErrorResource: not enough space in the queue (try semantics).
     * osErrorParameter: parameter mq_id is NULL or invalid, non-zero timeout specified in an ISR. */
  }

  /* Send the string to the stream buffer. */
  // xStreamBufferSendFromISR(xStreamBuffer,
  //                          (const void *)(buf),
  //                          len,
  //                          NULL);

  return;
}
/*-----------------------------------------------------------*/

static void prvLoggingThread(void *pvParameters)
{
  osStatus_t status;
  msgqueue_obj_t msg;
  char buf[dSTREAM_BUFFER_LENGTH_BYTES];

  /* Infinite loop */
  for (;;)
  {
    /* wait for message */
    status = osMessageQueueGet(mid_MsgQueue, &msg, NULL, 100U);
    if (status == osOK)
    {
      /* Enable red LED to indicate data transfer */
      BSP_LED_On(LED_RED);

      /* Send message via UART */
      if (xUARTLoggingUsed != pdFALSE)
      {
        HAL_UART_Transmit(&huart, (uint8_t *)&msg.str, msg.len, 1000);
      }

      /* Send message via SWO */
      if (xSWOLoggingUsed != pdFALSE)
      {
        for (int DataIdx = 0; DataIdx < msg.len; DataIdx++)
        {
          ITM_SendChar(msg.str[DataIdx]);
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
           * and the	IP task cannot itself wait for a socket to bind.  The
           * parameters to prvCreatePrintSocket() are not required so set to
           * NULL or 0. */
          xTimerPendFunctionCall(prvCreatePrintSocket, NULL, 0, dlDONT_BLOCK);
        }

        if (xPrintSocket != FREERTOS_INVALID_SOCKET)
        {
          // uint8_t *pucBuffer;
          // /* This RTOS task is going to send using the zero copy interface.  The
          //  * data being sent is therefore written directly into a buffer that is
          //  * passed into, rather than copied into, the FreeRTOS_sendto()
          //  * function.
          //  * First obtain a buffer of adequate length from the TCP/IP stack into which
          //  * the string will be written. */
          // pucBuffer = FreeRTOS_GetUDPPayloadBuffer(msg.len, portMAX_DELAY);

          // /* Check a buffer was obtained. */
          // if (pucBuffer != 0)
          // {
          //   /* Create the string that is sent. */
          //   memcpy((uint8_t *)pucBuffer, msg.str, msg.len);
          //   // memset(pucBuffer, 0x00, xStringLength);
          //   // sprintf(pucBuffer, "%s%lurn", ucStringToSend, ulCount);

          //   /* Pass the buffer into the send function.  ulFlags has the
          //    * FREERTOS_ZERO_COPY bit set so the TCP/IP stack will take control of the
          //    * buffer rather than copy data out of the buffer. */
          //   BaseType_t lReturned = FreeRTOS_sendto(xPrintSocket,
          //                                          (void *)pucBuffer,
          //                                          msg.len,
          //                                          FREERTOS_ZERO_COPY,
          //                                          &xPrintUDPAddress, sizeof(xPrintUDPAddress));

          //   if (lReturned == 0)
          //   {
          //     /* The send operation failed, so this RTOS task is still responsible
          //      * for the buffer obtained from the TCP/IP stack.  To ensure the buffer
          //      * is not lost it must either be used again, or, as in this case,
          //      * returned to the TCP/IP stack using FreeRTOS_ReleaseUDPPayloadBuffer().
          //      * pucBuffer can be safely re-used after this call. */
          //     FreeRTOS_ReleaseUDPPayloadBuffer((void *)pucBuffer);
          //   }
          //   else
          //   {
          //     /* The send was successful so the TCP/IP stack is now managing the
          //      * buffer pointed to by pucBuffer, and the TCP/IP stack will
          //      * return the buffer once it has been sent.  pucBuffer can
          //      * be safely re-used. */
          //   }
          // }

          FreeRTOS_sendto(xPrintSocket, msg.str, msg.len, 0, &xPrintUDPAddress, sizeof(xPrintUDPAddress));
        }
      }

      /* Disable red LED after data transfer */
      BSP_LED_Off(LED_RED);
    }
  }
}
/*-----------------------------------------------------------*/

/***************************** END OF FILE ************************************/
