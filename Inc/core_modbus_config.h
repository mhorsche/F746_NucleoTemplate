/**
 * @file core_modbus_config.h
 * @author horsche (horsche@li.plus)
 * @brief 
 * @version 0.1
 * @date 2021-11-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef CORE_MODBUS_CONFIG_H
#define CORE_MODBUS_CONFIG_H

/**************************************************/
/******* DO NOT CHANGE the following order ********/
/**************************************************/

/* Include logging header files and define logging macros in the following order:
 * 1. Include the header file "logging_levels.h".
 * 2. Define the LIBRARY_LOG_NAME and LIBRARY_LOG_LEVEL macros depending on
 * the logging configuration for MODBUS.
 * 3. Include the header file "logging_stack.h", if logging is enabled for MODBUS.
 */

#include "logging_levels.h"

/* Logging configuration for the MODBUS library. */
#ifndef LIBRARY_LOG_NAME
#define LIBRARY_LOG_NAME "MODBUS"
#endif

#ifndef LIBRARY_LOG_LEVEL
#define LIBRARY_LOG_LEVEL LOG_DEBUG
#endif

/* Prototype for the function used to print to console on Windows simulator
 * of FreeRTOS.
 * The function prints to the console before the network is connected;
 * then a UDP port after the network has connected. */
extern void vLoggingPrintf(const char *pcFormatString,
                           ...);

/* Map the SdkLog macro to the logging function to enable logging
 * on Windows simulator. */
#ifndef SdkLog
#define SdkLog(message) vLoggingPrintf message
#endif

#include "logging_stack.h"
/************ End of logging configuration ****************/

#endif /* ifndef CORE_MODBUS_CONFIG_H */
