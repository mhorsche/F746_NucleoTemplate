/**
 * @file stlink.h
 * @author horsche (horsche@li.plus)
 * @brief
 * @version 0.1
 * @date 2022-01-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __STLINK_H
#define __STLINK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_common.h"

/* Logging -------------------------------------------------------------------*/
/** @brief Include logging header files and define logging macros in the
 *         following order:
 *           1. Include the header file "logging_levels.h".
 *           2. Define the LIBRARY_LOG_NAME and LIBRARY_LOG_LEVEL macros.
 *           3. Include the header file "logging_stack.h".
 */
#include "logging_levels.h"

/* Logging configuration for the MQTT library. */
#ifndef LIBRARY_LOG_NAME
#define LIBRARY_LOG_NAME "STLK"
#endif

#ifndef LIBRARY_LOG_LEVEL
#define LIBRARY_LOG_LEVEL LOG_WARN
#endif

extern void vLoggingPrintf(const char *pcFormatString,
                           ...);
#ifndef SdkLog
#define SdkLog(message) vLoggingPrintf message
#endif

#include "logging_stack.h"

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "stream_buffer.h"

/* Exported defines ----------------------------------------------------------*/

#ifndef stlinkDMA_RXBUF_SIZE
/**
 * @brief Peripheral-to-memory buffer sizes.
 * @note The size of the receive buffer must not be greater than 32768 due to
 *    maximum DMA size (uint16_t). So sizeof(int16_t) * stlinkDMA_RXBUF_SIZE
 *    must be smaller than 65536.
 */
#define stlinkDMA_RXBUF_SIZE (8192U)
#endif

/**
 * @note The size of the receive buffer must not be greater than 32768 due to
 *    maximum DMA size (uint16_t):
 *      sizeof(int16_t) * stlinkDMA_RXBUF_SIZE < 65536
 *
 */
#if stlinkDMA_RXBUF_SIZE > 32768U
#error "Receive buffer size to high, stlinkDMA_RXBUF_SIZE must be smaller than 32768!"
#endif

/* Exported types ------------------------------------------------------------*/
struct STLinkContext;

/**
 * @brief Return codes from STLINK functions.
 */
typedef enum
{
  STLINKFail = 0,     /**< @brief . */
  STLINKSuccess,      /**< @brief . */
  STLINKBusy,         /**< @brief . */
  STLINKBadParameter, /**< @brief . */

} STLINKStatus_t;

/**
 * @brief Values indicating STLINK state definition.
 */
typedef enum STLINKState
{
  STLINKStateReset = 0, /*!< */
  STLINKStateReady,     /*!< */
  STLINKStateBusy,      /*!< */
  STLINKStateError,     /*!< */
  STLINKStateAbort      /*!< */
} STLINKState_t;

/**
 * @brief STLINK interrupt notifications.
 */
typedef enum STLINKNotify
{
  STLINKNotifyStop = 0x01,          /** @brief Stop STLINK. */
  STLINKNotifyRxBufferCplt = 0x02,  /** @brief Receive buffer (half) complete callback. */
  STLINKNotifyRaceCondition = 0x04, /** @brief Race condition detected. */
  STLINKNotifyTrigger = 0x08,       /** @brief Trigger notification. */
  STLINKNotifyOverFlow = 0x10,      /** @brief Over/Under Flow notification. */
  STLINKNotifyTransferCplt = 0x20,  /** @brief Memory-to-Memory DMA tansfer complete callback. */
  STLINKNotifyMQTTConnected = 0x80, /** @brief Notification if MQTT is initialized. */
} STLINKNotify_t;

/**
 * @brief A struct representing an STLINK connection.
 */
typedef struct STLINKContext
{
  /**
   * @brief Peripheral state.
   */
  STLINKState_t xState;

  /**
   * @brief Task handle (TaskHandle_t) for notifications.
   */
  void *pvTaskHandle;

  /**
   * @brief STLINK peripheral handler.
   */
  UART_HandleTypeDef hUart; /* VCOM UART handler */

  DMA_HandleTypeDef hDmaTx; /* UART transfer DMA handler */
  DMA_HandleTypeDef hDmaRx; /* UART receive DMA handler */

} STLINKContext_t;

/* Exported variables --------------------------------------------------------*/

/**
 * @brief STLINK context.
 */
extern STLINKContext_t *pxSTLinkContext;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Error code to string conversion for STLINK statuses.
 *
 * @param[in] status The status to convert to a string.
 *
 * @return The string representation of the status.
 */
const char *pucSTLinkStringStatus(const STLINKStatus_t status);

/**
 * @brief STLINK states to string conversion.
 *
 * @param[in] state The state to convert to a string.
 *
 * @return The string representation of the state.
 */
const char *pucSTLinkStringState(const STLINKState_t state);

/**
 * @brief  Initializes peripherals used by the SPI STLINK driver.
 *
 * @param[in] pxContext The context to initialize.
 *
 * @return STLINKSuccess (1) if operation is correctly performed, else return value
 *    different from STLINKSuccess (1)
 */
STLINKStatus_t xSTLinkInit(STLINKContext_t *pxContext, void *pvTaskHandle);

/**
 * @brief  DeInitializes the STLINK.
 *
 * @return STLINKSuccess (1) if operation is correctly performed, else return value
 *    different from STLINKSuccess (1)
 */
STLINKStatus_t xSTLinkDeInit(STLINKContext_t *pxContext);

STLINKStatus_t xSTLinkIsReady(STLINKContext_t *pxContext);

STLINKStatus_t xSTLinkSend(STLINKContext_t *pxContext, uint8_t *pucBuffer, size_t xReceivedBytes);

#endif /* __STLINK_H */

/********************************* END OF FILE ********************************/