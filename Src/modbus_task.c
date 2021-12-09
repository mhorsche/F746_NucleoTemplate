/**
 * @file modbus_task.c
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
#include "cmsis_os.h"
#include "modbus_task.h"

/* Standard includes. */
#include <string.h>
#include <math.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

/* MODBUS library includes. */
#include "core_modbus.h"

/* Private define ------------------------------------------------------------*/

#ifndef ipconfigMODBUS_HAS_TCP
/* A TCP server socket will be created. */
#define ipconfigMODBUS_HAS_TCP (1)
#endif

#ifndef ipconfigMODBUS_HAS_UDP
/* A UDP server socket will be created. */
#define ipconfigMODBUS_HAS_UDP (0)
#endif

#ifndef ipconfigMODBUS_TCP_PORT
/* Put the TCP server at this port number: */
/* 502 seems to be the standard TCP server port number. */
#define ipconfigMODBUS_TCP_PORT (502)
#endif

#ifndef ipconfigMODBUS_UDP_PORT
/* Put the UDP server at this port number: */
/* 502 seems to be the standard UDP server port number. */
#define ipconfigMODBUS_UDP_PORT (502)
#endif

#ifndef ipconfigMODBUS_BACKLOG_NUMBER
/* A new socket is created for each new connection the backlog value 
 * puts a limit on the number of simultaneously connected clients. */
#define ipconfigMODBUS_BACKLOG_NUMBER (4)
#endif

#ifndef ipconfigMODBUS_TCP_TIMEOUT_MS
#define ipconfigMODBUS_TCP_TIMEOUT_MS (2000)
#endif

#ifndef ipconfigMODBUS_LOOP_BLOCKING_TIME_MS
/* Let the mainloop wake-up so now and then. */
#define ipconfigMODBUS_LOOP_BLOCKING_TIME_MS 5000UL
#endif

#ifndef ipconfigMODBUS_USE_ZERO_COPY
#define ipconfigMODBUS_USE_ZERO_COPY (0)
#endif

#ifndef ipconfigMODBUS_TX_BUFSIZE
#define ipconfigMODBUS_TX_BUFSIZE (1 * ipconfigTCP_MSS) /* Units of bytes. */
#endif
#ifndef ipconfigMODBUS_TX_WINSIZE
#define ipconfigMODBUS_TX_WINSIZE (1) /* Size in units of MSS */
#endif
#ifndef ipconfigMODBUS_RX_BUFSIZE
#define ipconfigMODBUS_RX_BUFSIZE (1 * ipconfigTCP_MSS) /* Units of bytes. */
#endif
#ifndef ipconfigMODBUS_RX_WINSIZE
#define ipconfigMODBUS_RX_WINSIZE (1) /* Size in units of MSS */
#endif

#define ipconfigMODBUS_RECV_BUFFER_SIZE (256)

/* FTP and HTTP servers execute in the TCP server work task. */
#define modbusSERVER_TASK_PRIORITY (tskIDLE_PRIORITY + 3)
#define modbusSERVER_STACK_SIZE (2048)

/* Private macros ------------------------------------------------------------*/

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/* Private typedef -----------------------------------------------------------*/

/**
 * Modbus function codes
 */
typedef enum ModbusFunctionType
{
  MODBUS_FCK_READ_COILS = 0x01,                       /** Read Coils */
  MODBUS_FCK_READ_DISCRETE_INPUTS = 0x02,             /** Read Discrete Inputs */
  MODBUS_FCK_READ_HOLDING_REGISTERS = 0x03,           /** Read Holding Registers */
  MODBUS_FCK_READ_INPUT_REGISTERS = 0x04,             /** Read Input Registers */
  MODBUS_FCK_WRITE_SINGLE_COIL = 0x05,                /** Write Single Coil */
  MODBUS_FCK_WRITE_SINGLE_REGISTER = 0x06,            /** Write Single Register */
  MODBUS_FCK_READ_EXCEPTION_STATE = 0x07,             /** Read Exception Status (Serial Line only) */
  MODBUS_FCK_DIAGNOSTICS = 0x08,                      /** Diagnostics (Serial Line only) */
  MODBUS_FCK_GET_COMM_EVENT_COUNTER = 0x0B,           /** Get Comm Event Counter (Serial Line only) */
  MODBUS_FCK_GET_COMM_EVENT_LOG = 0x0C,               /** Get Comm Event Log (Serial Line only) */
  MODBUS_FCK_WRITE_MULTIPLE_COILS = 0x0F,             /** Write Multiple Coils */
  MODBUS_FCK_WRITE_MULTIPLE_REGISTERS = 0x10,         /** Write Multiple registers */
  MODBUS_FCK_REPORT_SERVER_ID = 0x11,                 /** Report Server ID (Serial Line only) */
  MODBUS_FCK_READ_FILE_RECORD = 0x14,                 /** Read File Record */
  MODBUS_FCK_WRITE_FILE_RECORD = 0x15,                /** Write File Record */
  MODBUS_FCK_MASK_WRITE_REGISTER = 0x16,              /** Mask Write Register */
  MODBUS_FCK_READ_WRITE_MULTIPLE_REGISTERS = 0x17,    /** Read/Write Multiple registers */
  MODBUS_FCK_READ_FIFO_QUEUE = 0x18,                  /** Read FIFO Queue */
  MODBUS_FCK_ENCAPSULATED_INTERFACE_TRANSPORT = 0x2B, /** Encapsulated Interface Transport */
  MODBUS_FCK_CAN_OPEN_PDU = 0x0D,                     /** CANopen General Reference Request and Response */
  MODBUS_FCK_READ_DEVICE_IDENTIFICATION = 0x0E        /** Read Device Identification */
} ModbusFunctionType_t;

/**
 * Modbus sub-function codes
 */
typedef enum ModbusSubFunctionType
{
  MODBUS_SF_RETURN_QUERY_DATA = 0x00,                    /** Return Query Data */
  MODBUS_SF_RESTART_COMMUNICATION = 0x01,                /** Restart Communications Option */
  MODBUS_SF_RETURN_DIAGNOSTIC_REGISTER = 0x02,           /** Return Diagnostic Register */
  MODBUS_SF_CHANGE_DELIMITER = 0x03,                     /** Change ASCII Input Delimiter */
  MODBUS_SF_FORCE_LISTEN_ONLY_MODE = 0x04,               /** Force Listen Only Mode */
  MODBUS_SF_CLEAR_COUNTERS = 0x0A,                       /** Clear Counters and Diagnostic Register */
  MODBUS_SF_RETURN_BUS_MESSAGE_COUNT = 0x0B,             /** Return Bus Message Count */
  MODBUS_SF_RETURN_BUS_COMMUNICATION_ERROR_COUNT = 0x0C, /** Return Bus Communication Error Count */
  MODBUS_SF_RETURN_BUS_EXCEPTION_ERROR_COUNT = 0x0D,     /** Return Bus Exception Error Count */
  MODBUS_SF_RETURN_SERVER_MESSAGE_COUNT = 0x0E,          /** Return Server Message Count */
  MODBUS_SF_RETURN_SERVER_NO_RESPONSE_COUNT = 0x0F,      /** Return Server No Response Count */
  MODBUS_SF_RETURN_SERVER_NAK_COUNT = 0x10,              /** Return Server NAK Count */
  MODBUS_SF_RETURN_SERVER_BUSY_COUNT = 0x11,             /** Return Server Busy Count */
  MODBUS_SF_RETURN_BUS_CHARACTER_OVERRUN_COUNT = 0x12,   /** Return Bus Character Overrun Count */
  MODBUS_SF_CLEAR_OVERRUN_COUNTER_FLAG = 0x14            /** Clear Overrun Counter and Flag */
} ModbusSubFunctionType_t;

typedef struct
{
  /** TCP Socket handle */
  Socket_t xServerSocket;

  /** Counters and diagnostic */
  ModbusCounter_t counter;
  uint16_t diagnostic;

  /** Modbus Rx/Tx buffer */
  ModbusPacketInfo_t xBuffer;

#if (ipconfigUSE_IPv6 != 0)
  struct freertos_sockaddr6 xRemoteAddr;
#else
  struct freertos_sockaddr xRemoteAddr;
#endif
  uint32_t ulRecvCount;        /* Total received bytes */
  struct xLIST_ITEM xListItem; /* With this item the client will be bound to a List_t. */

} TcpClient_t;

/* Private variables ---------------------------------------------------------*/
ModbusServer_t xModbusServer;

static List_t xTCPClientList;
static SocketSet_t xSocketSet;

static SemaphoreHandle_t xSocketSemaphore;

/**
 * @brief Modbus task handle to avoid task getting started multiple times.
 */
osThreadId_t xModbusTaskHandle;
const osThreadAttr_t xModbusTaskAttributes =
    {
        .name = "modbus",
        .stack_size = modbusSERVER_STACK_SIZE,
        .priority = (osPriority_t)tskIDLE_PRIORITY,
};

/* Private function prototypes -----------------------------------------------*/

static void prvModbusTask(void *pvParameters);

static void prvModbusServerWork(Socket_t xSocket);

static void prvModbusTCPClose(TcpClient_t *pxClient);

static int prvModbusTCPSend(TcpClient_t *pxClient);

static void prvModbusTCPWork(TcpClient_t *pxClient);

#if (ipconfigUSE_CALLBACKS != 0) && (ipconfigMODBUS_HAS_UDP != 0)
static BaseType_t prvOnUdpReceive(Socket_t xSocket, void *pvData, size_t xLength,
                                  const struct freertos_sockaddr *pxFrom, const struct freertos_sockaddr *pxDest);
#endif

#if (ipconfigMODBUS_HAS_TCP != 0)
static void prvCreateTCPServerSocket(Socket_t *xTCPServerSocket);
#endif

#if (ipconfigMODBUS_HAS_UDP != 0)
static Socket_t prvCreateUDPServerSocket(void);
#endif

static ModbusStatus_t prvModbusParseRequest(TcpClient_t *pxClient, ModbusPacketInfo_t *pxRxBuffer, BaseType_t xRecvResult);

static ModbusException_t prvModbusHandleRequest(TcpClient_t *pxClient, ModbusPacketInfo_t *pxRxBuffer, BaseType_t xRecvResult);

/* As for now, still defined in 'FreeRTOS-Plus-TCP\FreeRTOS_TCP_WIN.c' : */
extern void vListInsertGeneric(List_t *const pxList, ListItem_t *const pxNewListItem, MiniListItem_t *const pxWhere);
static portINLINE void vListInsertFifo(List_t *const pxList, ListItem_t *const pxNewListItem)
{
  vListInsertGeneric(pxList, pxNewListItem, &pxList->xListEnd);
}

/* String Functions ----------------------------------------------------------*/

/**
 * Function code value to string
 *
 * @param  {ucFunctionCode} Function code @see enum modbus_funktion_type
 * @return {string} Function code message text string
 */
static const char *pucModbusFunctionType(ModbusFunctionType_t ucFunctionCode)
{
  switch (ucFunctionCode)
  {
  case MODBUS_FCK_READ_COILS:
    return "Read Coils";
  case MODBUS_FCK_READ_DISCRETE_INPUTS:
    return "Read Discrete Inputs";
  case MODBUS_FCK_READ_HOLDING_REGISTERS:
    return "Read Holding Registers";
  case MODBUS_FCK_READ_INPUT_REGISTERS:
    return "Read Input Registers";
  case MODBUS_FCK_WRITE_SINGLE_COIL:
    return "Write Single Coil";
  case MODBUS_FCK_WRITE_SINGLE_REGISTER:
    return "Write Single Register";
  case MODBUS_FCK_READ_EXCEPTION_STATE:
    return "Read Exception Status (Serial Line only)";
  case MODBUS_FCK_DIAGNOSTICS:
    return "Diagnostics (Serial Line only)";
  case MODBUS_FCK_GET_COMM_EVENT_COUNTER:
    return "Get Comm Event Counter (Serial Line only)";
  case MODBUS_FCK_GET_COMM_EVENT_LOG:
    return "Get Comm Event Log (Serial Line only)";
  case MODBUS_FCK_WRITE_MULTIPLE_COILS:
    return "Write Multiple Coils";
  case MODBUS_FCK_WRITE_MULTIPLE_REGISTERS:
    return "Write Multiple registers";
  case MODBUS_FCK_REPORT_SERVER_ID:
    return "Report Server ID (Serial Line only)";
  case MODBUS_FCK_READ_FILE_RECORD:
    return "Read File Record";
  case MODBUS_FCK_WRITE_FILE_RECORD:
    return "Write File Record";
  case MODBUS_FCK_MASK_WRITE_REGISTER:
    return "Mask Write Register";
  case MODBUS_FCK_READ_WRITE_MULTIPLE_REGISTERS:
    return "Read/Write Multiple registers";
  case MODBUS_FCK_READ_FIFO_QUEUE:
    return "Read FIFO Queue";
  case MODBUS_FCK_ENCAPSULATED_INTERFACE_TRANSPORT:
    return "Encapsulated Interface Transport";
  case MODBUS_FCK_CAN_OPEN_PDU:
    return "CANopen General Reference Request and Response";
  case MODBUS_FCK_READ_DEVICE_IDENTIFICATION:
    return "Read Device Identification";
  }
  return "Invalid";
}

/**
 * Exception code value to string
 *
 * @param  {exception_code} Exception code @see enum ModbusException_t
 * @return {string} Exception code message text string
 */
static const char *pucModbusException(ModbusException_t exception_code)
{
  switch (exception_code)
  {
  case MODBUS_EXC_NO_EXCEPTION:
    return "NO_EXCEPTION";
  case MODBUS_EXC_ILLEGAL_FUNCTION:
    return "ILLEGAL_FUNCTION";
  case MODBUS_EXC_ILLEGAL_DATA_ADDRESS:
    return "ILLEGAL_DATA_ADDRESS";
  case MODBUS_EXC_ILLEGAL_DATA_VALUE:
    return "ILLEGAL_DATA_VALUE";
  case MODBUS_EXC_SLAVE_DEVICE_FAILURE:
    return "SLAVE_DEVICE_FAILURE";
  case MODBUS_EXC_ACKNOWLEDGE:
    return "ACKNOWLEDGE";
  case MODBUS_EXC_SLAVE_DEVICE_BUSY:
    return "SLAVE_DEVICE_BUSY";
  case MODBUS_EXC_MEMORY_PARITY_ERROR:
    return "MEMORY_PARITY_ERROR";
  case MODBUS_EXC_GATEWAY_PATH_UNAVAILABLE:
    return "GATEWAY_PATH_UNAVAILABLE";
  case MODBUS_EXC_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND:
    return "GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND";
  }
  return "Invalid";
}

/**
 * Connection state type value to string
 *
 * @param {status} Status code @see enum ModbusStatus_t
 * @return {string} Message type text string
 */
const char *pucModbusStatus(ModbusStatus_t status)
{
  const char *str = NULL;

  switch (status)
  {
  case MODBUS_STATE_SUCCESS:
    str = "Completed successfully";
    break;

    // case MODBUS_STATE_DISCONNECTED:
    //   str = "Disconnected";
    //   break;

    // case MODBUS_STATE_ACCEPTED:
    //   str = "Accepted";
    //   break;

  case MODBUS_STATE_BAD_REQUEST:
    str = "Invalid packet was received from the client";
    break;

  case MODBUS_STATE_REFUSED_MBAP:
    str = "Refused Modbus Application Protocol header";
    break;

    // case MODBUS_STATE_REFUSED_SERVER:
    //   str = "Refused server";
    //   break;

    // case MODBUS_STATE_REFUSED_USERNAME_PASS:
    //   str = "Refused user credentials";
    //   break;

    // case MODBUS_STATE_REFUSED_NOT_AUTHORIZED:
    //   str = "Refused not authorized";
    //   break;

    // case MODBUS_STATE_LISTENING:
    //   str = "Server listening on connections";
    //   break;

    // case MODBUS_STATE_ERROR:
    //   str = "TCP Error occured";
    //   break;

    // case MODBUS_STATE_SERVER_SHUTDOWN:
    //   str = "Server shutdown";
    //   break;

    // case MODBUS_STATE_RETRIES:
    //   str = "Retries";
    //   break;

    // case MODBUS_STATE_TIMEOUT:
    //   str = "Timeout";
    //   break;

    // case MODBUS_STATE_OUT_OF_MEMORY:
    //   str = "Out of memory";
    //   break;

  default:
    str = "Invalid Modbus Status code";
    break;
  }

  return str;
}

/* Private user code ---------------------------------------------------------*/

void vModbusInstall(ModbusServerConfig_t xModbusServerConfig)
{
  xModbusServer.server_id = xModbusServerConfig.server_id;
  xModbusServer.server_name = xModbusServerConfig.server_name;

  if (xModbusTaskHandle == NULL)
  {
    /* Open new TCP Receive task to handle Modbus messages */
    xModbusTaskHandle = osThreadNew(prvModbusTask, NULL, &xModbusTaskAttributes);
  }
}
/*-----------------------------------------------------------*/

static void prvModbusTask(void *pvParameters)
{
  /* Remove compiler warning about unused parameter. */
  (void)pvParameters;

  xSocketSet = FreeRTOS_CreateSocketSet();

  vListInitialise(&xTCPClientList);

  /* Wait until the network is up before creating the servers.  The notification
   * is given from the network event hook 'vApplicationIPNetworkEventHook'.
   *    xTaskNotifyGive(xModbusTaskHandle);
   */
  // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  /* The priority of this task can be raised now the disk has been
   * initialised. */
  vTaskPrioritySet(NULL, modbusSERVER_TASK_PRIORITY);

  xSocketSemaphore = xSemaphoreCreateBinary();

#if (ipconfigMODBUS_HAS_TCP != 0)
  Socket_t xTCPServerSocket = NULL;
  {
    prvCreateTCPServerSocket(&xTCPServerSocket);
  }
#endif /* ipconfigMODBUS_HAS_TCP */

#if (ipconfigMODBUS_HAS_UDP != 0)
  Socket_t xUDPServerSocket = NULL;
  {
    xUDPServerSocket = prvCreateUDPServerSocket();
    (void)xUDPServerSocket;
  }
#endif /* ipconfigMODBUS_HAS_UDP */

  for (;;)
  {
    BaseType_t xResult;
    const TickType_t xBlockingTime = pdMS_TO_TICKS(ipconfigMODBUS_LOOP_BLOCKING_TIME_MS);

    xSemaphoreTake(xSocketSemaphore, xBlockingTime);

    /* Wait at most 5 seconds. */
    xResult = FreeRTOS_select(xSocketSet, xBlockingTime);
#if (ipconfigMODBUS_HAS_TCP != 0)
    if (xResult != 0)
    {
      const MiniListItem_t *pxEnd;
      const ListItem_t *pxIterator;

      pxEnd = (const MiniListItem_t *)listGET_END_MARKER(&xTCPClientList);

      prvModbusServerWork(xTCPServerSocket);

      /* Check all TCP clients: */
      for (pxIterator = (const ListItem_t *)listGET_NEXT(pxEnd);
           pxIterator != (const ListItem_t *)pxEnd;)
      {
        TcpClient_t *pxClient;

        pxClient = (TcpClient_t *)listGET_LIST_ITEM_OWNER(pxIterator);

        /* Let the iterator point to the next element before the current element
         * removes itself from the list. */
        pxIterator = (const ListItem_t *)listGET_NEXT(pxIterator);

        prvModbusTCPWork(pxClient);
      }
    }
#endif /* ipconfigMODBUS_HAS_TCP */

#if (ipconfigMODBUS_HAS_UDP != 0)
#if (ipconfigUSE_CALLBACKS == 0)
    if (xResult != 0)
    {
      prvModbusUDPWork(xUDPServerSocket);
    }
    else
#endif /* ipconfigUSE_CALLBACKS */
    {
      if (ulUDPRecvCountSeen != ulUDPRecvCount)
      {
        /* The amount is still changing, do not show it yet. */
        ulUDPRecvCountSeen = ulUDPRecvCount;
      }
      else if (ulUDPRecvCountShown != ulUDPRecvCount)
      {
        uint32_t ulNewBytes = ulUDPRecvCount - ulUDPRecvCountShown;
        uint32_t ulMB = (ulNewBytes + HUNDREDTH_MB / 2) / HUNDREDTH_MB;

        LogInfo(("UDP received %lu + %lu (%lu.%02lu MB) = %lu",
                 ulUDPRecvCountShown,
                 ulNewBytes,
                 ulMB / 100,
                 ulMB % 100,
                 ulUDPRecvCount));
        ulUDPRecvCountShown = ulUDPRecvCount;
      }
    }
#endif /* ipconfigMODBUS_HAS_UDP */
  }
}
/*-----------------------------------------------------------*/

static void prvModbusServerWork(Socket_t xSocket)
{
  struct freertos_sockaddr xAddress;
  socklen_t xSocketLength;
  Socket_t xNexSocket;

  /* Accept new client connection */
  xNexSocket = FreeRTOS_accept(xSocket, &xAddress, &xSocketLength);
  if ((xNexSocket != NULL) && (xNexSocket != FREERTOS_INVALID_SOCKET))
  {
    char pucBuffer[16];
    TcpClient_t *pxClient;

    pxClient = (TcpClient_t *)pvPortMalloc(sizeof(*pxClient));
    memset(pxClient, '\0', sizeof(*pxClient));

    pxClient->xServerSocket = xNexSocket;

    listSET_LIST_ITEM_OWNER(&(pxClient->xListItem), (void *)pxClient);
#if (ipconfigUSE_IPv6 != 0)
    FreeRTOS_GetRemoteAddress(xNexSocket, (struct freertos_sockaddr6 *)&pxClient->xRemoteAddr);
#else
    FreeRTOS_GetRemoteAddress(xNexSocket, (struct freertos_sockaddr *)&pxClient->xRemoteAddr);
    FreeRTOS_inet_ntoa(pxClient->xRemoteAddr.sin_addr, pucBuffer);

    LogInfo(("Received a connection from %s:%u",
             pucBuffer,
             FreeRTOS_ntohs(pxClient->xRemoteAddr.sin_port)));
#endif

    FreeRTOS_FD_SET(xNexSocket, xSocketSet, eSELECT_READ | eSELECT_EXCEPT); // eSELECT_READ, eSELECT_ALL

    vListInsertFifo(&xTCPClientList, &(pxClient->xListItem));
  }
}
/*-----------------------------------------------------------*/

static void prvModbusTCPClose(TcpClient_t *pxClient)
{
  /* Remove server socket from the socket set. */
  if (pxClient->xServerSocket != NULL)
  {
    char pucBuffer[16];

#if (ipconfigUSE_IPv6 == 0)
    FreeRTOS_inet_ntoa(pxClient->xRemoteAddr.sin_addr, pucBuffer);
    LogInfo(("Closing server socket %s:%u after %lu bytes",
             pucBuffer,
             FreeRTOS_ntohs(pxClient->xRemoteAddr.sin_port),
             pxClient->ulRecvCount));
#endif
    FreeRTOS_FD_CLR(pxClient->xServerSocket, xSocketSet, eSELECT_ALL);
    FreeRTOS_closesocket(pxClient->xServerSocket);
    pxClient->xServerSocket = NULL;
  }

  /* Remove client socket from the socket set. */
  {
    if (pxClient->xServerSocket == NULL)
    {
      /* Remove this socket from the list. */
      uxListRemove(&(pxClient->xListItem));
      vPortFree((void *)pxClient);
    }
  }
}
/*-----------------------------------------------------------*/

static int prvModbusTCPSend(TcpClient_t *pxClient)
{
  BaseType_t xResult = 0;
  ModbusPacketInfo_t *pcWriteBuffer = &(pxClient->xBuffer);

  do
  {
    size_t uxMaxSpace = (size_t)FreeRTOS_tx_space(pxClient->xServerSocket);
    size_t uxSize = (size_t)FreeRTOS_min_uint32(uxMaxSpace, (int32_t)sizeof(pxClient->xBuffer));

    uxSize = FreeRTOS_min_uint32(uxSize, pcWriteBuffer->xMBAP.usLength + 6); /* xMBAP.usLength contains UID (1 byte), thats why MBAP length is set to 6 byte only */
    if (uxSize <= 0)
    {
      break;
    }

    /* Parse Modbus Application Protocol header (MBAP) - 7 Byte */
    (*((uint16_t *)pcWriteBuffer + 0)) = FreeRTOS_htons(pcWriteBuffer->xMBAP.usTID);    /* Transaction Identifier - 2 Byte */
    (*((uint16_t *)pcWriteBuffer + 1)) = FreeRTOS_htons(pcWriteBuffer->xMBAP.usPID);    /* Protocol Identifier - 2 Byte */
    (*((uint16_t *)pcWriteBuffer + 2)) = FreeRTOS_htons(pcWriteBuffer->xMBAP.usLength); /* Number of bytes - 2 Byte */

    // (*((uint8_t *)pcWriteBuffer + 6)) = (uint8_t)pcWriteBuffer->xMBAP.ucUnitIdentifier; /* Unit Identifier - 1 Byte */

    /* Parse Modbus Protocol Data Unit (PDU) - up to 253 Byte */
    // *((uint8_t *)pcWriteBuffer + 7) = (uint8_t)pcWriteBuffer->xPDU.ucFunctionCode; /* Modbus Function Code - 1 Byte */
    // *((uint8_t *)pcWriteBuffer + 8) = pxClient->xBuffer.xPDU.ucData;               /* Protocol Data - up to 252 Byte */

#if 0
    static char ucRxBufferString[ipconfigMODBUS_RECV_BUFFER_SIZE];
    memset(ucRxBufferString, '\0', sizeof(ucRxBufferString));
    for (uint16_t i = 0; i < uxSize; i++)
    {
      /* 0x00 0x01 0x00 0x00 0x00 0x06 0x01 0x03 0x00 0x06 0x00 0x05 */
      snprintf(ucRxBufferString, sizeof(ucRxBufferString), "%s 0x%02X", ucRxBufferString, (uint8_t)(*((uint8_t *)pcWriteBuffer + i)));
    }
    LogInfo(("TCP[ port %d ] send %d bytes:%s",
             FreeRTOS_ntohs(pxClient->xRemoteAddr.sin_port),
             (int)uxSize,
             ucRxBufferString));
#endif

    xResult = FreeRTOS_send(pxClient->xServerSocket, (const void *)pcWriteBuffer, uxSize, 0);
    if (xResult < 0)
    {
      break;
    }
    pxClient->xBuffer.xMBAP.usLength -= uxSize;
    if (pxClient->xBuffer.xMBAP.usLength == 0ul)
    {
      /* All data have been sent. No longer interested in eSELECT_WRITE events. */
      FreeRTOS_FD_CLR(pxClient->xServerSocket, xSocketSet, eSELECT_WRITE);
    }
    if (uxSize > 0)
    {
      xResult += uxSize;
    }
  } while (0);

  return xResult;
}
/*-----------------------------------------------------------*/

static void prvModbusTCPWork(TcpClient_t *pxClient)
{
  BaseType_t xRecvResult;
  ModbusPacketInfo_t *pxBuffer = &(pxClient->xBuffer);

  if (pxClient->xServerSocket == NULL)
  {
    prvModbusTCPClose(pxClient);
    return;
  }

  for (;;)
  {
#if (ipconfigMODBUS_USE_ZERO_COPY != 0)
    {
      const BaseType_t xRecvSize = 0x10000;
      xRecvResult = FreeRTOS_recv(pxClient->xServerSocket, /* The socket being received from. */
                                  &pcRecvBuffer,           /* The buffer into which the received data will be written. */
                                  xRecvSize,               /* Any size is OK here. */
                                  FREERTOS_ZERO_COPY);
    }
#else
    {
      const BaseType_t xRecvSize = sizeof(pxClient->xBuffer);
      xRecvResult = FreeRTOS_recv(pxClient->xServerSocket,      /* The socket being received from. */
                                  (void *)&(pxClient->xBuffer), /* The buffer into which the received data will be written. */
                                  xRecvSize,                    /* The size of the buffer provided to receive the data. */
                                  0);
    }
#endif

    if (xRecvResult <= 0)
    {
      break;
    }
    pxClient->ulRecvCount += xRecvResult;

    /* Parse Modbus request */
    ModbusStatus_t status = prvModbusParseRequest(pxClient, pxBuffer, xRecvResult);
    if (status != MODBUS_STATE_SUCCESS)
    {
      LogWarn(("Parsing incoming message failed (%s / %d)",
               pucModbusStatus(status), status));
      /* Adjust Communication Error Count */
      pxClient->counter.crc_error++;
    }

    /* Whole message received, perform request, response data will be set in pxClient->xBuffer */
    ModbusException_t res = prvModbusHandleRequest(pxClient, pxBuffer, xRecvResult);
    if (res != MODBUS_EXC_NO_EXCEPTION)
    {
      /* Bus Exception Error Count and Server Busy Count  */
      pxClient->counter.exception_error++;
      if (res == MODBUS_EXC_ACKNOWLEDGE || res == MODBUS_EXC_SLAVE_DEVICE_BUSY)
      {
        pxClient->counter.server_busy++;
      }

      /* Modbus exception response:
       *   the objective is to provide to the client relevant information concerning the
       *   error detected during the processing
       *   the exception function code = the request function code + 0x80
       *   an exception code is provided to indicate the reason of the error
       *   response length is always 3 Bytes (Unit Identifier + Function code | 0x80 + Exception code) */
      LogWarn(("Modbus request '%s' exception: %s (%d)",
               pucModbusFunctionType((uint8_t)(*((uint8_t *)pxBuffer + 7))),
               pucModbusException(res), res));

      pxClient->xBuffer.xMBAP.usLength = 3;      /* Number of bytes (incl. UID) - 2 Byte */
      *((uint8_t *)pxBuffer + 7) |= 0x80;        /* Modbus Function Code - 1 Byte */
      *((uint8_t *)pxBuffer + 8) = (uint8_t)res; /* Exception code - 1 Byte */
    }
    else
    {
      /* Server Message Count */
      pxClient->counter.server_message++;

      /* Positive Modbus response:
       *   the response function code = the request function code
       *   response data depends on function code */
      LogDebug(("Request '%s' successful (response len %d)",
                pucModbusFunctionType((uint8_t)(*((uint8_t *)pxBuffer + 7))),
                pxClient->xBuffer.xMBAP.usLength));
    }

    // /* Return modbus response */
    // if (pxClient->response_len == 0)
    // {
    //   /* Adjust Server No Response Count */
    //   pxClient->counter.server_no_response++;
    // }
    // else
    // {
    //   err = modbus_response(client, pxClient->conn);
    xRecvResult = prvModbusTCPSend(pxClient);
    //   if (err != ERR_OK)
    //   {
    //     LWIP_DEBUGF(MODBUS_DEBUG_WARN, ("modbus_tcp_recv_cb: Sending response failed (%s / %d)",
    //                                     lwip_strerr(err), err));
    //   }
    // }

    // /* If keep alive functionality is used */
    // if (pxClient->timeout != 0)
    // {
    //   /* Reset server alive watchdog */
    //   pxClient->watchdog = 0;
    // }

    // /* Notify upper layer of modbus request */
    // if (pxClient->request_cb != NULL)
    // {
    //   pxClient->request_cb(client, pxClient->client_arg);
    // }

#if (ipconfigMODBUS_USE_ZERO_COPY != 0)
    {
      FreeRTOS_recv(pxClient->xServerSocket, /* The socket being received from. */
                    NULL,                    /* The buffer into which the received data will be written. */
                    xRecvResult,             /* This is important now. */
                    0);
    }
#endif
  } /* for( ;; ) */

  // if ((xRecvResult == 0) && (pxClient->bits.bIsControl == pdFALSE_UNSIGNED) && (pxClient->bits.bReverse != pdFALSE_UNSIGNED))
  // {
  //   xRecvResult = prvModbusTCPSend(pxClient);
  // }
  if ((xRecvResult < 0) && (xRecvResult != -pdFREERTOS_ERRNO_EAGAIN))
  {
    prvModbusTCPClose(pxClient);
  }
}
/*-----------------------------------------------------------*/

#if (ipconfigUSE_CALLBACKS != 0) && (ipconfigMODBUS_HAS_UDP != 0)
static BaseType_t prvOnUdpReceive(Socket_t xSocket, void *pvData, size_t xLength,
                                  const struct freertos_sockaddr *pxFrom, const struct freertos_sockaddr *pxDest)
{
  (void)pvData;
  (void)pxFrom;

  ulUDPRecvCount += xLength;
#if (ipconfigMODBUS_DOES_ECHO_UDP != 0)
  {
    FreeRTOS_sendto(xSocket, (const void *)pvData, xLength, 0, pxFrom, sizeof(*pxFrom));
  }
#else /* ipconfigMODBUS_DOES_ECHO_UDP */
  {
    (void)xSocket;
  }
#endif
  /* Tell the driver not to store the RX data */
  return 1;
}
#endif /* ipconfigUSE_CALLBACKS != 0 */
/*-----------------------------------------------------------*/

#if (ipconfigMODBUS_HAS_TCP != 0)
static void prvCreateTCPServerSocket(Socket_t *xTCPServerSocket)
{
  BaseType_t xBindResult, xListenResult;
  struct freertos_sockaddr xModbusServerAddress;
  TickType_t xNoTimeOut = 0;
  // TickType_t xTimeOut = pdMS_TO_TICKS(ipconfigMODBUS_TCP_TIMEOUT_MS);
  BaseType_t xRxTxBufferLength = MODBUS_MAX_ADU_BUFFER_LEN;

  if (*xTCPServerSocket != NULL)
  {
    LogInfo(("TCP server %p  already listening on port %u",
             *xTCPServerSocket, ipconfigMODBUS_TCP_PORT));
    return;
  }

  *xTCPServerSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);
  configASSERT((*xTCPServerSocket != FREERTOS_INVALID_SOCKET) && (*xTCPServerSocket != NULL));

  /* Bind the socket to the port that the client task will send to, then
   * listen for incoming connections. */
  memset(&xModbusServerAddress, '\0', sizeof xModbusServerAddress);
  xModbusServerAddress.sin_addr = FreeRTOS_GetIPAddress();
  xModbusServerAddress.sin_port = FreeRTOS_htons(ipconfigMODBUS_TCP_PORT);

  /* Set the receive time out. Set a time out so accept() will just wait for a connection. 
   * Note that any TCP child connections will inherit this reception time-out. */
  FreeRTOS_setsockopt(*xTCPServerSocket, 0, FREERTOS_SO_RCVTIMEO, (void *)&xNoTimeOut, sizeof(xNoTimeOut));
  FreeRTOS_setsockopt(*xTCPServerSocket, 0, FREERTOS_SO_RCVBUF, (void *)&xRxTxBufferLength, sizeof(xRxTxBufferLength));

  FreeRTOS_setsockopt(*xTCPServerSocket, 0, FREERTOS_SO_SNDTIMEO, (void *)&xNoTimeOut, sizeof(xNoTimeOut));
  FreeRTOS_setsockopt(*xTCPServerSocket, 0, FREERTOS_SO_SNDBUF, (void *)&xRxTxBufferLength, sizeof(xRxTxBufferLength));

  /** The TCP/IP RTOS task will then give to the semaphore on any of these events:
   *    Arrival of new data
   *    After delivering data, when new transmission buffer space becomes available
   *    An outgoing TCP connection has succeeded
   *    A new client has connected to a TCP socket
   *    A TCP connection was closed or reset
   * @note: If a socket has a reference to a semaphore then the semaphore must not be deleted!  To 
   * remove the semaphore call FreeRTOS_setsockopt() again, but this time with a NULL semaphore. */
  FreeRTOS_setsockopt(*xTCPServerSocket, 0, FREERTOS_SO_SET_SEMAPHORE, (void *)&xSocketSemaphore, sizeof(xSocketSemaphore));

/* Set the window and buffer sizes. */
#if (ipconfigUSE_TCP_WIN == 1)
  {
    WinProperties_t xWinProperties;

    memset(&xWinProperties, '\0', sizeof xWinProperties);

    /* Fill in the buffer and window sizes that will be used by the socket. */
    xWinProperties.lTxBufSize = ipconfigMODBUS_TX_BUFSIZE; /* Units of bytes. */
    xWinProperties.lTxWinSize = ipconfigMODBUS_TX_WINSIZE; /* Size in units of MSS */
    xWinProperties.lRxBufSize = ipconfigMODBUS_RX_BUFSIZE; /* Units of bytes. */
    xWinProperties.lRxWinSize = ipconfigMODBUS_RX_WINSIZE; /* Size in units of MSS */

    FreeRTOS_setsockopt(*xTCPServerSocket, 0, FREERTOS_SO_WIN_PROPERTIES, (void *)&xWinProperties, sizeof(xWinProperties));
  }
#endif /* ipconfigUSE_TCP_WIN */

  xBindResult = FreeRTOS_bind(*xTCPServerSocket, &xModbusServerAddress, sizeof(xModbusServerAddress));
  xListenResult = FreeRTOS_listen(*xTCPServerSocket, ipconfigMODBUS_BACKLOG_NUMBER); /* e.g. 10, maximum number of connected TCP clients */

  FreeRTOS_FD_SET(*xTCPServerSocket, xSocketSet, eSELECT_READ);

  LogInfo(("Created TCP server socket %p bind port %u (bind %ld / listen %ld)",
           *xTCPServerSocket, ipconfigMODBUS_TCP_PORT, xBindResult, xListenResult));

  return /* xTCPServerSocket */;
}
#endif /* ipconfigMODBUS_HAS_TCP */
/*-----------------------------------------------------------*/

#if (ipconfigMODBUS_HAS_UDP != 0)
static Socket_t prvCreateUDPServerSocket(Socket_t *xUDPServerSocket)
{
  if (*xUDPServerSocket != NULL)
  {
    LogInfo(("UDP server %p  already listening on port %u",
             *xUDPServerSocket, ipconfigMODBUS_UDP_PORT));
    return;
  }
}
#endif /* ipconfigMODBUS_HAS_UDP */
/*-----------------------------------------------------------*/

/**
 * Modbus incoming message parser:
 *
 * (*1) Modbus TCP/IP Application Data Unit
 * (*2) Modbus Application Protocol header (MBAP)
 * (*3) Modbus Protocol Data Unit (PDU)
 * Remark: the different fields are encoded in Big-endian.
 *
 * |<----------------------- Modbus TCP/IP ADU(*1) ----------------------->|
 * |<---------- MBAP Header (*2) ------------>|<-------- PDU (*3) -------->|
 * +-----------+-----------+-----------+------+------+---------------------+
 * |    TID    |    PID    |  Length   | UID  | Code |        Data         |
 * | 0x00 0x05 | 0x00 0x00 | 0x00 0x06 | 0x01 | 0x03 | 0x00 0x02 0x00 0x01 |
 * +-----------+-----------+-----------+------+------+---------------------+
 *
 * TCP_TID     = 0 (Transaction Identifier - 2 Byte)
 * TCP_PID     = 2 (Protocol Identifier - 2 Byte)
 * TCP_LEN     = 4 (Number of bytes - 2 Byte)
 * TCP_UID     = 6 (Unit Identifier - 1 Byte)
 * MODBUS_FUNC = 7 (Modbus Function Code - 1 Byte)
 * MODBUS_DATA = 8 (Modbus Data - Length depending on function code)
 *
 * Exemplarily a valid modbus message to read holding register address 0x02
 * echo -n -e '\x00\x01\x00\x00\x00\x06\x03\x00\x02\x00\x01' | nc IP_ADDR PORT
 *
 *      0x01 = Remote UID is 1
 *      0x03 = Function Code is Read Holding Registers
 *    0x0002 = Starting Address (2 Bytes)
 *    0x0001 = Quantity of Registers (2 Bytes)
 * 
 */
static ModbusStatus_t prvModbusParseRequest(TcpClient_t *pxClient, ModbusPacketInfo_t *pxRxBuffer, BaseType_t xRecvResult)
{
#if 0
  static char ucRxBufferString[ipconfigMODBUS_RECV_BUFFER_SIZE];
  memset(ucRxBufferString, '\0', sizeof(ucRxBufferString));
  for (uint16_t i = 0; i < xRecvResult; i++)
  {
    /* 0x00 0x01 0x00 0x00 0x00 0x06 0x01 0x03 0x00 0x06 0x00 0x05 */
    snprintf(ucRxBufferString, sizeof(ucRxBufferString), "%s 0x%02X", ucRxBufferString, (uint8_t)(*((uint8_t *)pxRxBuffer + i)));
  }
  LogInfo(("TCP[ port %d ] recv %d bytes:%s",
           FreeRTOS_ntohs(pxClient->xRemoteAddr.sin_port),
           (int)xRecvResult,
           ucRxBufferString));
#endif

  /* Check Modbus request length */
  if (xRecvResult < 7)
  {
    LogWarn(("Invalid Modbus request (length %d)",
             xRecvResult));
    return MODBUS_STATE_BAD_REQUEST;
  }

  /* Parse Modbus Application Protocol header (MBAP) - 7 Byte */
  pxClient->xBuffer.xMBAP.usTID = FreeRTOS_ntohs((uint16_t)(*((uint16_t *)pxRxBuffer + 0)));    /* Transaction Identifier - 2 Byte */
  pxClient->xBuffer.xMBAP.usPID = FreeRTOS_ntohs((uint16_t)(*((uint16_t *)pxRxBuffer + 1)));    /* Protocol Identifier - 2 Byte */
  pxClient->xBuffer.xMBAP.usLength = FreeRTOS_ntohs((uint16_t)(*((uint16_t *)pxRxBuffer + 2))); /* Number of bytes - 2 Byte */

  // pxClient->xBuffer.xMBAP.ucUnitIdentifier = (uint8_t)(*((uint8_t *)pxRxBuffer + 6));           /* Unit Identifier - 1 Byte */

  /* Parse Modbus Protocol Data Unit (PDU) - up to 253 Byte */
  // pxClient->xBuffer.xPDU.ucFunctionCode = (uint8_t)(*((uint8_t *)pxRxBuffer + 7)); /* Modbus Function Code - 1 Byte */
  // pxClient->xBuffer.xPDU.ucData = (uint8_t *)((uint8_t *)pxRxBuffer + 8);          /* Protocol Data - up to 252 Byte */

  /* Protocol Identifier (PID) must be 0 for Modbus protocol */
  if (pxClient->xBuffer.xMBAP.usPID != 0)
  {
    LogWarn(("Invalid MBAP, PID must be 0: tid %d / pid %d / len %d / uid %d",
             pxClient->xBuffer.xMBAP.usTID,
             pxClient->xBuffer.xMBAP.usPID,
             pxClient->xBuffer.xMBAP.usLength,
             pxClient->xBuffer.xMBAP.ucUnitIdentifier));
    return MODBUS_STATE_REFUSED_MBAP;
  }

  /* Cross check with MBAP length */
  if ((uint16_t)(xRecvResult - 7 /* MBAP size */) != pxClient->xBuffer.xMBAP.usLength - 1)
  {
    LogWarn(("Length does not match (MBAP %d / xRecvResult %d) ",
             pxClient->xBuffer.xMBAP.usLength,
             xRecvResult));
    return MODBUS_STATE_BAD_REQUEST;
  }

  return MODBUS_STATE_SUCCESS;
}
/*-----------------------------------------------------------*/

/**
 * Complete Modbus message received
 *
 * @param client Modbus client
 * @param length length received part
 * @param remaining_length Remaining length of complete message
 */
static ModbusException_t prvModbusHandleRequest(TcpClient_t *pxClient, ModbusPacketInfo_t *pxRxBuffer, BaseType_t xRecvResult)
{
  /* Parse Modbus Protocol Data Unit (PDU) - up to 253 Byte */
  uint8_t ucFunctionCode = (uint8_t)(*((uint8_t *)pxRxBuffer + 7)); /* Modbus Function Code - 1 Byte */
  uint8_t *pcRxData = (uint8_t *)((uint8_t *)pxRxBuffer + 8);       /* Protocol Data - up to 252 Byte */
  uint8_t *pcTxData = pcRxData;

  uint16_t *pusResponseLength = &(pxClient->xBuffer.xMBAP.usLength);
  uint16_t usRequestLength = (pxClient->xBuffer.xMBAP.usLength - 1);

  LogInfo(("Function %s (0x%02X) with %d byte",
           pucModbusFunctionType(ucFunctionCode),
           ucFunctionCode,
           usRequestLength));

  /* Check for valid function code */
  switch (ucFunctionCode)
  {
  case MODBUS_FCK_READ_COILS:
    /** (0x01) Read Coils
     * This function code is used to read from 1 to 2000 contiguous state of coils in a remote
     * device. The Request PDU specifies the starting address, i.e. the address of the first coil
     * specified, and the number of coils. In the PDU Coils are addressed starting at zero. Therefore
     * coils numbered 1-16 are addressed as 0-15.
     * The coils in the response message are packed as one coil per bit of the data field. Status is
     * indicated as 1= ON and 0= OFF. The LSB of the first data byte contains the output addressed
     * in the query. The other coils follow toward the high order end of this byte, and from low order
     * to high order in subsequent bytes.
     * If the returned output quantity is not a multiple of eight, the remaining bits in the final data
     * byte will be padded with zeros (toward the high order end of the byte). The Byte Count field
     * specifies the quantity of complete bytes of data.
     * Request:
     *   Function code      1 Byte  0x01
     *   Starting Address   2 Bytes 0x0000 to 0xFFFF
     *   Quantity of coils  2 Bytes 1 to 2000 (0x7D0)
     * Response:
     *   Function code      1 Byte  0x01
     *   Byte count         1 Byte  N*
     *   Quantity of coils  n Byte  n = N or N+1
     *   *N = Quantity of Inputs / 8 if the remainder is different of 0 => N = N+1
     * Error:
     *   Error code         1 Byte  0x81
     *   Exception code     1 Byte  01 or 02 or 03 or 04
     */
    // if (xModbusServer.read_bits_0x01_cb != NULL)
    // if (xModbusReadCoilsHook_0x01 != NULL)
    {
      /* Read bits callback assigned */
      if (usRequestLength == 5)
      {
        /* Starting Address (Byte 0:1), Quantity of coils (Byte 2:3) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t len = (pcRxData[2] << 8) | pcRxData[3];
        /* Check quantity, must be between 1 (0x0001) and 2000 (0x07D0) */
        if (1 <= len && len <= MODBUS_MAX_COIL_LENGTH)
        {
          /* Check address within range (65535) */
          if ((addr + len) <= 0xFFFF)
          {
            /* Byte count = Quantity of Inputs / 8 if the remainder is different of 0 => N = N+1 */
            uint8_t count = (uint8_t)(ceil((float)len / (float)8));

            /* Build response */
            pcTxData[0] = count;
            *pusResponseLength = count + 2 + 1; /* Number of bytes (incl. UID) - 2 Byte */
            // return xModbusServer.read_bits_0x01_cb(pcTxData + 1, addr, len, xModbusServer.callback_arg);
            return xModbusReadCoilsHook_0x01(pcTxData + 1, addr, len, xModbusServer.callback_arg);
          }
          else
          {
            /* Invalid address number */
            return MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
          }
        }
        else
        {
          /* Invalid quantity number */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with Modbus requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_READ_DISCRETE_INPUTS:
    /** (0x02) Read Discrete Inputs
     * This function code is used to read from 1 to 2000 contiguous state of discrete inputs in a
     * remote device. The Request PDU specifies the starting address, i.e. the address of the first
     * input specified, and the number of inputs. In the PDU Discrete Inputs are addressed starting
     * at zero. Therefore Discrete inputs numbered 1-16 are addressed as 0-15.
     * The discrete inputs in the response message are packed as one input per bit of the data field.
     * Status is indicated as 1 = ON; 0 = OFF. The LSB of the first data byte contains the input
     * addressed in the query. The other inputs follow toward the high order end of this byte, and
     * from low order to high order in subsequent bytes.
     * If the returned input quantity is not a multiple of eight, the remaining bits in the final d ata byte
     * will be padded with zeros (toward the high order end of the byte). The Byte Count field
     * specifies the quantity of complete bytes of data.
     * Request:
     *   Function code      1 Byte  0x02
     *   Starting Address   2 Bytes 0x0000 to 0xFFFF
     *   Quantity of Inputs 2 Bytes 1 to 2000 (0x7D0)
     * Response:
     *   Function code      1 Byte  0x02
     *   Byte count         1 Byte  N*
     *   Input Status       n Byte  n = N or N+1
     *   *N = Quantity of Inputs / 8 if the remainder is different of 0 => N = N+1
     * Error:
     *   Error code         1 Byte  0x82
     *   Exception code     1 Byte  01 or 02 or 03 or 04
     */
    // if (xModbusServer.read_input_bits_0x02_cb != NULL)
    // if (xModbusReadInputBitsHook_0x02 != NULL)
    {
      /* Read input bits callback assigned */
      if (usRequestLength == 5)
      {
        /* Starting Address (Byte 0:1), Quantity of Inputs (Byte 2:3) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t len = (pcRxData[2] << 8) | pcRxData[3];
        /* Check quantity, must be between 1 (0x0001) and 2000 (0x07D0) */
        if (1 <= len && len <= MODBUS_MAX_COIL_LENGTH)
        {
          /* Check address within range (65535) */
          if ((addr + len) <= 0xFFFF)
          {
            /* Byte count = Quantity of Inputs / 8 if the remainder is different of 0 => N = N+1 */
            uint8_t count = (uint8_t)(ceil((float)len / (float)8));

            /* Build response */
            pcTxData[0] = count;
            *pusResponseLength = count + 2 + 1; /* Number of bytes (incl. UID) - 2 Byte */
            // return xModbusServer.read_input_bits_0x02_cb(pcTxData + 1, addr, len, xModbusServer.callback_arg);
            return xModbusReadInputBitsHook_0x02(pcTxData + 1, addr, len, xModbusServer.callback_arg);
          }
          else
          {
            /* Invalid address number */
            return MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
          }
        }
        else
        {
          /* Invalid quantity number */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with Modbus requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_READ_HOLDING_REGISTERS:
    /** (0x03) Read Holding Registers
     * This function code is used to read the contents of a contiguous block of holding registers in a
     * remote device. The Request PDU specifies the starting register address and the number of
     * registers. In the PDU Registers are addressed starting at zero. Therefore registers numbered
     * 1-16 are addressed as 0-15.
     * The register data in the response message are packed as two bytes per register, with the
     * binary contents right justified within each byte. For each register, the first byte contains the
     * high order bits and the second contains the low order bits.
     * Request:
     *   Function code          1 Byte  0x03
     *   Starting Address       2 Bytes 0x0000 to 0xFFFF
     *   Quantity of Registers  2 Bytes 1 to 125 (0x7D)
     * Response:
     *   Function code          1 Byte  0x03
     *   Byte count             1 Byte  2 x N
     *   Register value         2N Byte
     * Error:
     *   Error code             1 Byte  0x83
     *   Exception code         1 Byte  01 or 02 or 03 or 04
     */
    // if (xModbusServer.read_registers_0x03_cb != NULL)
    // if (xModbusReadHoldregsHook_0x03 != NULL)
    {
      /* Read registers callback assigned */
      if (usRequestLength == 5)
      {
        /* Starting Address (Byte 0:1), Quantity of Registers (Byte 2:3) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t len = (pcRxData[2] << 8) | pcRxData[3];
        /* Check quantity, must be between 1 (0x01) and 125 (0x7D) */
        if (1 <= len && len <= MODBUS_MAX_REGISTER_LENGTH)
        {
          /* Check address within range (65535) */
          if ((addr + len) <= 0xFFFF)
          {
            /* Build response */
            pcTxData[0] = len * 2;
            *pusResponseLength = (len * 2) + 2 + 1; /* Number of bytes (incl. UID) - 2 Byte */
            // return xModbusServer.read_registers_0x03_cb((uint16_t *)(pcTxData + 1), addr, len, xModbusServer.callback_arg);
            return xModbusReadHoldregsHook_0x03((uint16_t *)(pcTxData + 1), addr, len, xModbusServer.callback_arg);
          }
          else
          {
            /* Invalid address number */
            return MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
          }
        }
        else
        {
          /* Invalid quantity number */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with Modbus requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_READ_INPUT_REGISTERS:
    /** (0x04) Read Input Registers
     * This function code is used to read from 1 to 125 contiguous input registers in a remote device.
     * The Request PDU specifies the starting register address and the number of registers. In the
     * PDU Registers are addressed starting at zero. Therefore input registers numbered 1-16 are
     * addressed as 0-15.
     * The register data in the response message are packed as two bytes per register, with the
     * binary contents right justified within each byte. For each register, the first byte contains the
     * high order bits and the second contains the low order bits.
     * Request:
     *   Function code                1 Byte  0x04
     *   Starting Address             2 Bytes 0x0000 to 0xFFFF
     *   Quantity of Input Registers  2 Bytes 1 to 125 (0x7D)
     * Response:
     *   Function code                1 Byte  0x04
     *   Byte count                   1 Byte  2 x N
     *   Input Registers              2N Byte
     * Error:
     *   Error code                   1 Byte  0x84
     *   Exception code               1 Byte  01 or 02 or 03 or 04
     */
    // if (xModbusServer.read_input_registers_0x04_cb != NULL)
    // if (xModbusReadInputRegistersHook_0x04 != NULL)
    {
      /* Read input registers callback assigned */
      if (usRequestLength == 5)
      {
        /* Starting Address (Byte 0:1), Quantity of Input Registers (Byte 2:3) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t len = (pcRxData[2] << 8) | pcRxData[3];
        /* Check quantity, must be between 1 (0x01) and 125 (0x7D) */
        if (1 <= len && len <= MODBUS_MAX_REGISTER_LENGTH)
        {
          /* Check address within range (65535) */
          if ((addr + len) <= 0xFFFF)
          {
            /* Build response */
            pcTxData[0] = len * 2;
            *pusResponseLength = (len * 2) + 2 + 1; /* Number of bytes (incl. UID) - 2 Byte */
            // return xModbusServer.read_input_registers_0x04_cb((uint16_t *)(pcTxData + 1), addr, len, xModbusServer.callback_arg);
            return xModbusReadInputRegistersHook_0x04((uint16_t *)(pcTxData + 1), addr, len, xModbusServer.callback_arg);
          }
          else
          {
            /* Invalid address number */
            return MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
          }
        }
        else
        {
          /* Invalid quantity number */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with Modbus requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_WRITE_SINGLE_COIL:
    /** (0x05) Write Single Coil
     * This function code is used to write a single output to either ON or OFF in a remote device.
     * The requested ON/OFF state is specified by a constant in the request data field. A value of FF
     * 00 hex requests the output to be ON. A value of 00 00 requests it to be OFF. All other values
     * are illegal and will not affect the output.
     * The Request PDU specifies the address of the coil to be forced. Coils are addressed starting
     * at zero. Therefore coil numbered 1 is addressed as 0. The requested ON/OFF state is
     * specified by a constant in the Coil Value field. A value of 0xFF00 requests the coil to be ON.
     * A value of 0x0000 requests the coil to be off. All other values are illegal and will not affect the
     * coil.
     * The normal response is an echo of the request, returned after the coil state has been written.
     * Request:
     *   Function code     1 Byte  0x05
     *   Output Address    2 Bytes 0x0000 to 0xFFFF
     *   Output Value      2 Bytes 0x0000 or 0xFF00
     * Response:
     *   Function code     1 Byte  0x05
     *   Output Address    2 Bytes 0x0000 to 0xFFFF
     *   Output Value      2 Bytes 0x0000 or 0xFF00
     * Error:
     *   Error code        1 Byte  0x85
     *   Exception code    1 Byte  01 or 02 or 03 or 04
     */
    // if (xModbusServer.write_bit_0x05_cb != NULL)
    // if (xModbusWriteBitHook_0x05 != NULL)
    {
      /* Write bits callback assigned */
      if (usRequestLength == 5)
      {
        /* Output Address (Byte 0:1), Output Value (Byte 2:3) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t val = (pcRxData[2] << 8) | pcRxData[3];
        /* Check value, must be OFF (0x0000) or ON (0xFF00) */
        if (val == 0x0000 || val == 0xFF00)
        {
          /* The normal response is an echo of the request */
          // memcpy(&(pcTxData), &(pcRxData), *pusResponseLength - 1);
          /* Adjust output value to match with callback function for (0x0F) Write Multiple Coils */
          pcRxData[1] = (val == 0xFF00 ? 1 : 0);
          // return xModbusServer.write_bit_0x05_cb(pcRxData + 2, addr, 1, xModbusServer.callback_arg);
          return xModbusWriteBitHook_0x05(pcRxData + 2, addr, 1, xModbusServer.callback_arg);
        }
        else
        {
          /* Invalid quantity number */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with Modbus requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_WRITE_SINGLE_REGISTER:
    /** (0x06) Write Single Register
     * This function code is used to write a single holding register in a remote device.
     * The Request PDU specifies the address of the register to be written. Registers are addressed
     * starting at zero. Therefore register numbered 1 is addressed as 0.
     * The normal response is an echo of the request, returned after the register contents have been
     * written.
     * Request:
     *   Function code     1 Byte   0x06
     *   Register Address  2 Bytes  0x0000 to 0xFFFF
     *   Register Value    2 Bytes  0x0000 to 0xFFFF
     * Response:
     *   Function code     1 Byte   0x06
     *   Register Address  2 Bytes  0x0000 to 0xFFFF
     *   Register Value    2 Bytes  0x0000 to 0xFFFF
     * Error:
     *   Error code        1 Byte   0x86
     *   Exception code    1 Byte   01 or 02 or 03 or 04
     */
    // if (xModbusServer.write_register_0x06_cb != NULL)
    // if (xModbusWriteHoldregHook_0x06 != NULL)
    {
      /* Write registers callback assigned */
      if (usRequestLength == 5)
      {
        /* Register Address (Byte 0:1), Register Value (Byte 2:3) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t val = (pcRxData[2] << 8) | pcRxData[3];
        /* Check value, must be between 0x0000 and 0xFFFF, useless since its all uint16 values, but it is in the Modbus specification on page 20 */
        if (0x0000 <= val && val <= 0xFFFF)
        {
          /* The normal response is an echo of the request */
          // memcpy(&(pcTxData), &(pcRxData), *pusResponseLength - 1);
          // return xModbusServer.write_register_0x06_cb((uint16_t *)(pcRxData + 2), addr, 1, xModbusServer.callback_arg);
          return xModbusWriteHoldregHook_0x06((uint16_t *)(pcRxData + 2), addr, 1, xModbusServer.callback_arg);
        }
        else
        {
          /* Invalid quantity number */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with Modbus requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_READ_EXCEPTION_STATE:
    /** (0x07) Read Exception Status (Serial Line only)
     * This function code is used to read the contents of eight Exception Status outputs in a remote
     * device.
     * The function provides a simple method for accessing this information, because the Exception
     * Output references are known (no output reference is needed in the function).
     * The normal response contains the state of the eight Exception Status outputs. The outputs
     * are packed into one data byte, with one bit per output. The state of the lowest output
     * reference is contained in the least significant bit of the byte.
     * The contents of the eight Exception Status outputs are device specific.
     * Request:
     *   Function code     1 Byte   0x07
     * Response:
     *   Function code     1 Byte   0x07
     *   Output Data       1 Byte   0x00 to 0xFF
     * Error:
     *   Error code        1 Byte   0x87
     *   Exception code    1 Byte   01 or 04
     */
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;
  case MODBUS_FCK_DIAGNOSTICS:
    /** (0x08) Diagnostics (Serial Line only)
     * Modbus function code 08 provides a series of tests for checking the communication system
     * between a client device and a server, or for checking various internal error conditions within a
     * server.
     * The function uses a twobyte sub-function code field in the query to define the type of test to
     * be performed. The server echoes both the function code and sub-function code in a normal
     * response. Some of the diagnostics cause data to be returned from the remote device in the
     * data field of a normal response.
     * In general, issuing a diagnostic function to a remote device does not affect the running of the
     * user program in the remote device. User logic, like discrete and registers, is not accessed by
     * the diagnostics. Certain functions can optionally reset error counters in the remote device.
     * A server device can, however, be forced into Listen Only Mode in which it will monitor the
     * messages on the communications system but not respond to them. This can affect the
     * outcome of your application program if it depends upon any further exchange of data with the
     * remote device. Generally, the mode is forced to remove a malfunctioning remote device from
     * the communications system.
     * Request:
     *   Function code     1 Byte   0x08
     *   Sub-function      2 Bytes
     *   Data              2N Bytes
     * Response:
     *   Function code     1 Byte   0x08
     *   Sub-function      2 Bytes
     *   Data              2N Bytes
     * Error:
     *   Error code        1 Byte   0x88
     *   Exception code    1 Byte   01 or 03 or 04
     */
    /* Check minimum message length */
    if (usRequestLength >= 5)
    {
      /* Subfunction code (Byte 0:1), Data (Byte 2:3) */
      uint16_t subfct = (pcRxData[0] << 8) | pcRxData[1];
      uint16_t data = (pcRxData[2] << 8) | pcRxData[3];

      /* Switch Subfunction code */
      switch (subfct)
      {
      case MODBUS_SF_RETURN_QUERY_DATA:
        /** (0x00) Return Query Data
         * The data passed in the request data field is to be returned (looped back) in the response. The
         * entire response message should be identical to the request.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 00           Any                     Echo Request Data
         */
        /* Request data field is to be returned (looped back) */
        memcpy(&(pcTxData), &(pcRxData), *pusResponseLength - 1);
        return MODBUS_EXC_NO_EXCEPTION;
      case MODBUS_SF_RESTART_COMMUNICATION:
        /** (0x01) Restart Communications Option
         * The remote device serial line port must be initialized and restarted, and all of its
         * communications event counters are cleared. If the port is currently in Listen Only Mode, no
         * response is returned. This function is the only one that brings the port out of Lis ten Only
         * Mode. If the port is not currently in Listen Only Mode, a normal response is returned. This
         * occurs before the restart is executed.
         * When the remote device receives the request, it attempts a restart and executes its power up
         * confidence tests. Successful completion of the tests will bring the port online.
         * A request data field contents of FF 00 hex causes the ports Communications Event Log to be
         * cleared also. Contents of 00 00 leave the log as it was prior to the restart.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 01           00 00                   Echo Request Data
         *   00 01           FF 00                   Echo Request Data
         */
        return MODBUS_EXC_ILLEGAL_FUNCTION;
      case MODBUS_SF_RETURN_DIAGNOSTIC_REGISTER:
        /** (0x02) Return Diagnostic Register
         * The contents of the remote devices 16bit diagnostic register are returned in the response.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 02           00 00                   Diagnostic Register Contents
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Return diagnostic register */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->diagnostic), 2);

          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      case MODBUS_SF_CHANGE_DELIMITER:
        /** (0x03) Change ASCII Input Delimiter
         * The character CHAR passed in the request data field becomes the end of message delimiter
         * for future messages (replacing the default LF character). This function is useful in cases of a
         * Line Feed is not required at the end of ASCII messages.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 03           CHAR 00                 Echo Request Data
         */
        return MODBUS_EXC_ILLEGAL_FUNCTION;
      case MODBUS_SF_FORCE_LISTEN_ONLY_MODE:
        /** (0x04) Force Listen Only Mode
         * Forces the addressed remote device to its Listen Only Mode for Modbus communications.
         * This isolates it from the other devices on the network, allowing them to continue
         * communicating without interruption from the addressed remote device. No response is
         * returned.
         * When the remote device enters its Listen Only Mode, all active communication controls are
         * turned off. The Ready watchdog timer is allowed to expire, locking the controls off. While the
         * device is in this mode, any Modbus messages addressed to it or broadcast are monitored,
         * but no actions will be taken and no responses will be sent.
         * The only function that will be processed after the mode is entered will be the Restart
         * Communications Option function (function code 8, sub-function 1).
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 04           00 00                   No Response Returned
         */
        return MODBUS_EXC_ILLEGAL_FUNCTION;
      case MODBUS_SF_CLEAR_COUNTERS:
        /** (0x0A) Clear Counters and Diagnostic Register
         * The goal is to clear all counters and the diagnostic register. Counters are also cleared upon
         * powerup.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 0A           00 00                   Echo Request Data
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Clear all counters and the diagnostic register */
          pxClient->counter.message = 0;
          pxClient->counter.crc_error = 0;
          pxClient->counter.exception_error = 0;
          pxClient->counter.server_message = 0;
          pxClient->counter.server_no_response = 0;
          pxClient->counter.server_nak = 0;
          pxClient->counter.server_busy = 0;
          pxClient->counter.bus_character_overrun = 0;
          pxClient->diagnostic = 0;
          /* Echo Request Data */
          memcpy(&(pcTxData), &(pcRxData), *pusResponseLength - 1);

          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      case MODBUS_SF_RETURN_BUS_MESSAGE_COUNT:
        /** (0x0B) Return Bus Message Count
         * The response data field returns the quantity of messages that the remote device has detected
         * on the communications system since its last restart, clear counters operation, or power up.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 0B           00 00                   Total Message Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.message), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_RETURN_BUS_COMMUNICATION_ERROR_COUNT:
        /** (0x0C) Return Bus Communication Error Count
         * The response data field returns the quantity of CRC errors encountered by the remote device
         * since its last restart, clear counters operation, or powerup.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 0C           00 00                   CRC Error Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.crc_error), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_RETURN_BUS_EXCEPTION_ERROR_COUNT:
        /** (0x0D) Return Bus Exception Error Count
         * The response data field returns the quantity of Modbus exception responses returned by the
         * remote device since its last restart, clear counters operation, or powerup.
         * Exception responses are described and listed in section 7 .
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 0D           00 00                   Exception Error Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.exception_error), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_RETURN_SERVER_MESSAGE_COUNT:
        /** (0x0E) Return Server Message Count
         * The response data field returns the quantity of messages addressed to the remote device, or
         * broadcast, that the remote device has processed since its last restart, clear counters
         * operation, or powerup.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 0E           00 00                   Server Message Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.server_message), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_RETURN_SERVER_NO_RESPONSE_COUNT:
        /** (0x0F) Return Server No Response Count
         * The response data field returns the quantity of messages addressed to the remote device for
         * which it has returned no response (neither a normal response nor an exception response),
         * since its last restart, clear counters operation, or powerup.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 0F           00 00                   Server No Response Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.server_no_response), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_RETURN_SERVER_NAK_COUNT:
        /** (0x10) Return Server NAK Count
         * The response data field returns the quantity of messages addressed to the remote device for
         * which it returned a Negative Acknowledge (NAK) exception response, since its last restart,
         * clear counters operation, or powerup.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 10           00 00                   Server NAK Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.server_nak), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_RETURN_SERVER_BUSY_COUNT:
        /** (0x11) Return Server Busy Count
         * The response data field returns the quantity of messages addressed to the remote device for
         * which it returned a Server Device Busy exception response, since its last restart, clear
         * counters operation, or powerup.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 11           00 00                   Server Device Busy Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.server_busy), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_RETURN_BUS_CHARACTER_OVERRUN_COUNT:
        /** (0x12) Return Bus Character Overrun Count
         * The response data field returns the quantity of messages addressed to the remote device that
         * it could not handle due to a character overrun condition, since its last restart, clear counters
         * operation, or powerup. A character overrun is caused by data characters arriving at the port
         * faster than they can be stored, or by the loss of a character due to a hardware malfunction.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 12           00 00                   Server Character Overrun Count
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Echo Request Data, counter is equal to (subfunction code - 0x0B) */
          pcTxData[0] = subfct;
          memcpy(&(pcTxData[2]), &(pxClient->counter.bus_character_overrun), 2);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
        break;
      case MODBUS_SF_CLEAR_OVERRUN_COUNTER_FLAG:
        /** (0x14) Clear Overrun Counter and Flag
         * Clears the overrun error counter and reset the error flag.
         *   Sub-function    Data Field (Request)    Data Field (Response)
         *   00 14           00 00                   Echo Request Data
         */
        if (usRequestLength == 5 && data == 0x0000)
        {
          /* Clears the overrun error counter and reset the error flag */
          pxClient->counter.bus_character_overrun = 0;
          /* Echo Request Data */
          // memcpy(&(pcTxData), &(pcRxData), *pusResponseLength - 1);
          return MODBUS_EXC_NO_EXCEPTION;
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      /* Subfunction code no supported */
      return MODBUS_EXC_ILLEGAL_FUNCTION;
    }
    else
    {
      /* Message length does not match with MB requirements */
      return MODBUS_EXC_ILLEGAL_DATA_VALUE;
    }
    break;
  case MODBUS_FCK_GET_COMM_EVENT_COUNTER:
    /** (0x0B) Get Comm Event Counter (Serial Line only)
     * This function code is used to get a state word and an event count from the remote device's
     * communication event counter.
     * By fetching the current count before and after a series of messages, a client can determine
     * whether the messages were handled normally by the remote device.
     * The device’s event counter is incremented once for each successful message completion. It is
     * not incremented for exception responses, poll commands, or fetch event counter commands.
     * The event counter can be reset by means of the Diagnostics function (code 08), with a subfunction of Restart Communications Option (code 00 01) or Clear Counters and Diagnostic
     * Register (code 00 0A).
     * The normal response contains a two–byte state word, and a two–byte event count. The
     * state word will be all ones (FF FF hex) if a previously–issued program command is still being
     * processed by the remote device (a busy condition exists). Otherwise, the state word will be
     * all zeros.
     * Request:
     *   Function code     1 Byte  0x0B
     * Response:
     *   Function code     1 Byte  0x0B
     *   Status            2 Bytes 0x0000 to 0xFFFF
     *   Event Count       2 Bytes 0x0000 to 0xFFFF
     * Error:
     *   Error code        1 Byte  0x8B
     *   Exception code    1 Byte  01 or 04
     */
    break;
  case MODBUS_FCK_GET_COMM_EVENT_LOG:
    /** (0x0C) Get Comm Event Log (Serial Line only)
     * This function code is used to get a state word, event count, message count, and a field of
     * event bytes from the remote device.
     * The state word and event counts are identical to that returned by the Get Communications
     * Event Counter function (11, 0B hex).
     * The message counter contains the quantity of messages processed by the remote device
     * since its last restart, clear counters operation, or power–up. This count is identical to that
     * returned by the Diagnostic function (code 08), sub-function Return Bus Message Count (code
     * 11, 0B hex).
     * The event bytes field contains 0-64 bytes, with each byte corresponding to the state of one
     * Modbus send or receive operation for the remote device. The remote device enters the
     * events into the field in chronological order. Byte 0 is the most recent event. Each new byte
     * flushes the oldest byte from the field.
     * Request:
     *   Function code     1 Byte  0x0C
     * Response:
     *   Function code     1 Byte 0x0C
     *   Byte Count        1 Byte N*
     *   Status            2 Bytes 0x0000 to 0xFFFF
     *   Event Count       2 Bytes 0x0000 to 0xFFFF
     *   Message Count     2 Bytes 0x0000 to 0xFFFF
     *   Events            (N-6) x 1 Byte
     *   *N = Quantity of Events + 3 x 2 Bytes, (Length of Status, Event Count and Message Count)
     * Error:
     *   Error code        1 Byte  0x8C
     *   Exception code    1 Byte  01 or 04
     */
    break;
  case MODBUS_FCK_WRITE_MULTIPLE_COILS:
    /** (0x0F) Write Multiple Coils
     * This function code is used to force each coil in a sequence of coils to either ON or OFF in a
     * remote device. The Request PDU specifies the coil references to be forced. Coils are
     * addressed starting at zero. Therefore coil numbered 1 is addressed as 0.
     * The requested ON/OFF states are specified by contents of the request data field. A logical ' 1'
     * in a bit position of the field requests the corresponding output to be ON. A logical '0' requests
     * it to be OFF.
     * The normal response returns the function code, starting address, and quantity of coils forced.
     * Request:
     *   Function code       1 Byte 0x0F
     *   Starting Address    2 Bytes 0x0000 to 0xFFFF
     *   Quantity of Outputs 2 Bytes 0x0001 to 0x07B0
     *   Byte Count          1 Byte N*
     *   Outputs Value       N* x 1 Byte
     *   *N = Quantity of Inputs / 8 if the remainder is different of 0 => N = N+1
     * Response:
     *   Function code       1 Byte  0x0F
     *   Starting Address    2 Bytes 0x0000 to 0xFFFF
     *   Quantity of Outputs 2 Bytes 0x0001 to 0x07B0
     * Error:
     *   Error code          1 Byte  0x8F
     *   Exception code      1 Byte  01 or 02 or 03 or 04
     */
    // if (xModbusServer.write_bits_0x0F_cb != NULL)
    // if (xModbusWriteBitsHook_0x0F != NULL)
    {
      /* Write bits callback assigned */
      if (usRequestLength >= 7)
      {
        /* Output Address (Byte 0:1), Quantity of Outputs (Byte 2:3), Byte Count (Byte 4) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t len = (pcRxData[2] << 8) | pcRxData[3];
        uint8_t bytes = pcRxData[4];
        /* Check quantity, must be between 1 (0x0001) and 2000 (0x07D0) */
        if (1 <= len && len <= MODBUS_MAX_COIL_LENGTH && bytes == (uint8_t)(ceil((float)len / (float)8)))
        {
          /* Check address within range (65535) */
          if ((addr + len) <= 0xFFFF)
          {
            /* Build response */
            *pusResponseLength = 5 + 1; /* Number of bytes (incl. UID) - 2 Byte */
            // return xModbusServer.write_bits_0x0F_cb(pcRxData + 5, addr, len, xModbusServer.callback_arg);
            return xModbusWriteBitsHook_0x0F(pcRxData + 5, addr, len, xModbusServer.callback_arg);
          }
          else
          {
            /* Invalid address number */
            return MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
          }
        }
        else
        {
          /* Invalid quantity number */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with Modbus requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_WRITE_MULTIPLE_REGISTERS:
    /** (0x10) Write Multiple registers
     * This function code is used to write a block of contiguous registers (1 to 123 registers) in a
     * remote device.
     * The requested written values are specified in the request data field. Data is packed as two
     * bytes per register.
     * The normal response returns the function code, starting address, and quantity of registers
     * written.
     * Request:
     *   Function code           1 Byte       0x10
     *   Starting Address        2 Bytes      0x0000 to 0xFFFF
     *   Quantity of Registers   2 Bytes      0x0001 to 0x007B
     *   Byte Count              1 Byte       2 x N*
     *   Registers Value         N x 2 Bytes  Value
     * Response:
     *   Function code           1 Byte       0x10
     *   Starting Address        2 Bytes      0x0000 to 0xFFFF
     *   Quantity of Registers   2 Bytes      1 to 123 (0x7B)
     * Error:
     *   Function  code          1 Byte       0x90
     *   Exception code          1 Byte       01 or 02 or 03 or 04
     */
    // if (xModbusServer.write_registers_0x10_cb != NULL)
    // if (xModbusWriteHoldregsHook_0x10 != NULL)
    {
      /* Write registers callback assigned */
      if (usRequestLength >= 5)
      {
        /* Starting Address (Byte 0:1), Quantity of Registers (Byte 2:3) */
        uint16_t addr = (pcRxData[0] << 8) | pcRxData[1];
        uint16_t len = (pcRxData[2] << 8) | pcRxData[3];
        uint8_t bytes = pcRxData[4];
        /* Check quantity, must be between 1 (0x01) and 123 (0x7B), , Byte Count == Quantity of Registers x 2 */
        if (1 <= len && len <= (MODBUS_MAX_REGISTER_LENGTH - 2) && bytes == len * 2)
        {
          /* Check address within range (65535) */
          if ((addr + len) <= 0xFFFF)
          {
            /* The normal response is an echo of the request */
            *pusResponseLength = 5 + 1; /* Number of bytes (incl. UID) - 2 Byte */
            // return xModbusServer.write_registers_0x10_cb((uint16_t *)(pcRxData + 5), addr, len, xModbusServer.callback_arg);
            return xModbusWriteHoldregsHook_0x10((uint16_t *)(pcRxData + 5), addr, len, xModbusServer.callback_arg);
          }
          else
          {
            /* Invalid address number */
            return MODBUS_EXC_ILLEGAL_DATA_ADDRESS;
          }
        }
        else
        {
          /* Invalid value */
          return MODBUS_EXC_ILLEGAL_DATA_VALUE;
        }
      }
      else
      {
        /* Message length does not match with MB requirements */
        return MODBUS_EXC_ILLEGAL_DATA_VALUE;
      }
    }
    break;
  case MODBUS_FCK_REPORT_SERVER_ID:
    /** (0x11) Report Server ID (Serial Line only)
     * This function code is used to read the description of the type, the current state, and other
     * information specific to a remote device.
     * The format of a normal response is shown in the following example. The data contents are
     * specific to each type of device.
     * Request:
     *   Function code     1 Byte   0x11
     * Response:
     *   Function code     1 Byte   0x11
     *   Byte Count        1 Byte
     *   Server ID
     *   Run Indicator     1 Byte   0x00 = OFF, 0xFF = ON
     *   Additional Data
     * Error:
     *   Function  code    1 Byte   0x91
     *   Exception code    1 Byte   01 or 04
     */
    if (usRequestLength == 1)
    {
      /* Generate server name with time/date */
      char str[128] = {0};
      snprintf(str, sizeof(str), "%s (%s %s)", xModbusServer.server_name, __DATE__, __TIME__);
      /* Build response */
      pcTxData[0] = strlen(str) + 2;
      pcTxData[1] = xModbusServer.server_id;
      pcTxData[2] = 0xFF; /* Run indicator (0x00 = OFF, 0xFF = ON) */
      strcpy((char *)&(pcTxData[3]), str);
      *pusResponseLength = strlen(str) + 4 + 1; /* Number of bytes (incl. UID) - 2 Byte */

      return MODBUS_EXC_NO_EXCEPTION;
    }
    else
    {
      /* Message length does not match with MB requirements */
      return MODBUS_EXC_ILLEGAL_DATA_VALUE;
    }
    break;
  case MODBUS_FCK_READ_FILE_RECORD:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  case MODBUS_FCK_WRITE_FILE_RECORD:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  case MODBUS_FCK_MASK_WRITE_REGISTER:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  case MODBUS_FCK_READ_WRITE_MULTIPLE_REGISTERS:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  case MODBUS_FCK_READ_FIFO_QUEUE:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  case MODBUS_FCK_ENCAPSULATED_INTERFACE_TRANSPORT:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  case MODBUS_FCK_CAN_OPEN_PDU:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  case MODBUS_FCK_READ_DEVICE_IDENTIFICATION:
    return MODBUS_EXC_ILLEGAL_FUNCTION;
    break;

  default:
    /* Check for user defined function codes */
    if ((65 <= ucFunctionCode && ucFunctionCode <= 72) || (100 <= ucFunctionCode && ucFunctionCode <= 110))
    {
      // if (xModbusServer.default_request_cb != NULL)
      {
        /* Response must contain function code */
        // return xModbusServer.default_request_cb(ucFunctionCode, pcRxData, usRequestLength, pcTxData, pusResponseLength, xModbusServer.callback_arg);
        return xModbusUserFunctionHook_0xXX(ucFunctionCode, pcRxData, usRequestLength, pcTxData, pusResponseLength, xModbusServer.callback_arg);
      }
    }
    break;
  }
  /* Unsupported/illegal function code */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function code 0x01 (read coils).
 *
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        written to (bits state)
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of bits (1 to 2000)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusReadCoilsHook_0x01(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d",
            addr, addr, len));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusReadCoilsHook_0x01 function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function code 0x02 (read discrete inputs).
 *
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        written to (bits state)
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of bits (1 to 2000)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusReadInputBitsHook_0x02(uint8_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d",
            addr, addr, len));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusReadInputBitsHook_0x02 function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function code 0x03 (read holding registers).
 * To ensure correct endianness (big-endian), use the function Modbus_htons/
 * Modbus_htonl to convert host to network.
 *
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        writte to (register values)
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of registers (1 to 125)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusReadHoldregsHook_0x03(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d",
            addr, addr, len));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusReadHoldregsHook_0x03 function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function code 0x04 (read input registers).
 * To ensure correct endianness (big-endian), use the function Modbus_htons/
 * Modbus_htonl to convert host to network.
 *
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        writte to (register values)
 * @param {addr} Requested starting address (0x0000 to 0xFFFF)
 * @param {len} Requested quantity of registers (1 to 125)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusReadInputRegistersHook_0x04(uint16_t *tx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d",
            addr, addr, len));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusReadInputRegistersHook_0x04 function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function codes 0x05 (write single coil). The
 * requested ON/OFF state is specified by a constant in the request data field. 
 * A value of FF 00 hex requests the output to be ON. A value of 00 00 requests 
 * it to be OFF. All other values are illegal and will not affect the output.
 *
 * @param {rx_buffer} Modbus client receive buffer with requested write sequence
 * @param {addr} Starting address (0x0000 to 0xFFFF)
 * @param {len} Quantity of bits (1)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusWriteBitHook_0x05(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d / rx_buffer 0x%04X ...",
            addr, addr, len, rx_buffer[0]));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusWriteBitHook_0x05 function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function codes 0x06 (write single register).
 * The write sequence is given as uint16_t value. To ensure correct endianness
 * (big-endian), use the function Modbus_ntohs/Modbus_ntohl to convert network
 * to host.
 *
 * @param {rx_buffer} Modbus client receive buffer with requested write sequence
 * @param {addr} Starting address (0x0000 to 0xFFFF)
 * @param {len} Quantity of holdregs (1)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusWriteHoldregHook_0x06(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d / rx_buffer 0x%04X 0x%04X ...",
            addr, addr, len, rx_buffer[0], rx_buffer[1]));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusWriteHoldregHook_0x06 function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function codes 0x0F (write multiple coils). The 
 * write sequence is given as bit stream. Example of a request to write a series 
 * of 10 bits starting at address 20:
 *
 * rx_buffer:  | 1  1  0  0  1  1  0  1  | X  X  X  X  X  X  0  1  |
 * bit address:| 27 26 25 24 23 22 21 20 | –  –  –  –  –  –  29 28 |
 *
 * The first byte transmitted (CD hex) addresses outputs 27-20, with the least
 * significant bit addressing the lowest output (20) in this set.
 * The next byte transmitted (01 hex) addresses outputs 29-28, with the least
 * significant bit addressing the lowest output (28) in this set.
 *
 * @param {rx_buffer} Modbus client receive buffer with requested write sequence
 * @param {addr} Starting address (0x0000 to 0xFFFF)
 * @param {len} Quantity of bits (1 to 2000)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusWriteBitsHook_0x0F(const uint8_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d / rx_buffer 0x%04X ...",
            addr, addr, len, rx_buffer[0]));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusWriteBitsHook_0x0F function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for requested function code 0x10 (write multiple registers).
 * The write sequence is given as uint16_t values for both function codes. To
 * ensure correct endianness (big-endian), use the function Modbus_ntohs/
 * Modbus_ntohl to convert network to host. Example of a request to write two
 * registers starting at 2 to 00 0A and 01 02 hex:
 *
 * rx_buffer:        | 0x00 0x0A | 0x01 0x02 |
 * register address: |     2     |     3     |
 *
 * @param {rx_buffer} Modbus client receive buffer with requested write sequence
 * @param {addr} Starting address (0x0000 to 0xFFFF)
 * @param {len} Quantity of holdregs (1 to 125)
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusWriteHoldregsHook_0x10(const uint16_t *rx_buffer, const uint16_t addr, const uint16_t len, void *arg)
{
  LogDebug(("addr %d (0x%04X) / len %d / rx_buffer 0x%04X 0x%04X ...",
            addr, addr, len, rx_buffer[0], rx_buffer[1]));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusWriteHoldregsHook_0x10 function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/**
 * Callback function for all user defined function codes (65 to 72/100 to 110).
 * Therefore, the tx and rx_buffers are given to perform read/write requests.
 *
 * @param {ucFunctionCode} Requested user function code (65 to 72/100 to 110)
 * @param {rx_buffer} Modbus client receive buffer with requested write sequence
 * @param {request_len} Requested length in bytes
 * @param {tx_buffer} Modbus client transfer buffer where the response must be
 *        writte to
 * @param {request_len} Response length in bytes
 * @param {arg} Additional *callback_arg to pass to the callback function
 *        @see modbus_server_config_t
 *
 * @return {exception} Modbus exception codes @see ModbusException_t
 */
__weak ModbusException_t
xModbusUserFunctionHook_0xXX(const uint8_t ucFunctionCode, const uint8_t *rx_buffer, const uint16_t usRequestLength, uint8_t *tx_buffer, uint16_t *response_len, void *arg)
{
  LogDebug(("function code 0x%02X / usRequestLength %d",
            ucFunctionCode, usRequestLength));

  /* This is a __weak placeholder function. Functionallity must be implemented 
   * by user defined xModbusUserFunctionHook_0xXX function */
  return MODBUS_EXC_ILLEGAL_FUNCTION;
}
/*-----------------------------------------------------------*/

/***************************** END OF FILE ************************************/
