/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef __LOGGING_H
#define __LOGGING_H

/******************************************************************************/
/*                          Initialize Logging Stack                          */
/*                     DO NOT CHANGE the following order                      */
/* Include header files and define logging macros in the following order:     */
/* 1. Include the header file "logging_levels.h".                             */
/* 2. Define the LIBRARY_LOG_NAME and LIBRARY_LOG_LEVEL macros.               */
/* 3. Include the header file "logging_stack.h".                              */
/******************************************************************************/

/* 1. Include the header file "logging_levels.h". */
// #include "logging_levels.h"

/* 2. Define the LIBRARY_LOG_NAME and LIBRARY_LOG_LEVEL macros. */
// #ifndef LIBRARY_LOG_NAME
// #define LIBRARY_LOG_NAME "LOG"
// #endif

// #ifndef LIBRARY_LOG_LEVEL
// #define LIBRARY_LOG_LEVEL LOG_INFO
// #endif

// #ifndef ipconfigHAS_DEBUG_PRINTF
// #define ipconfigHAS_DEBUG_PRINTF 1
// #endif

/* 3. Include the header file "logging_stack.h". */
// #include "logging_stack.h"

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* FreeRTOS includes. */
#include <FreeRTOS.h> /* BaseType_t */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief Initialize a logging system that can be used from FreeRTOS tasks.  Do not
 * call printf() directly while the scheduler is running.
 *
 * Set xLogToUART, xLogToSWO and xLogToUDP to either pdTRUE or pdFALSE to
 * lot to UART, SWO and a UDP port respectively.
 *
 * If xLogToUDP is pdTRUE then ulRemoteIPAddress and usRemotePort must be set
 * to the IP address and port number to which UDP log messages will be sent.
 */
void vLoggingInit(BaseType_t xLogToUART,
                  BaseType_t xLogToSWO,
                  BaseType_t xLogToUDP,
                  uint32_t ulRemoteIPAddress,
                  uint16_t usRemotePort);

/**
 * @brief Set ulRemoteIPAddress and usRemotePort must be set to the IP address and port
 * number to which UDP log messages will be sent.
 */
void vSetUDPAddress(uint32_t ulRemoteIPAddress,
                    const uint16_t usRemotePort);

/**
 * @brief Printf like logging interface to log messages from FreeRTOS
 * tasks.
 *
 * Depending on the configuration made through vLoggingInit(),
 * this function will print to UART, SWO or transmit over a UDP port.
 *
 * @param[in] pcFormat The format string of the log message.
 * @param[in] ... The variadic list of parameters for the format
 * specifiers in the @p pcFormat.
 */
void vLoggingPrintf(const char *pcFormat,
                    ...);

#endif /* __LOGGING_H */
