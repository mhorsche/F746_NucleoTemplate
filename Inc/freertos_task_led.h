/**
 * @file freertos_task_led.h
 * @author horsche (horsche@li.plus)
 * @brief
 * @version
 * @date 2023-08-11
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_TASK_LED_H
#define __FREERTOS_TASK_LED_H

/******************************************************************************/
/*                          Initialize Logging Stack                          */
/*                     DO NOT CHANGE the following order                      */
/* Include header files and define logging macros in the following order:     */
/* 1. Include the header file "logging_levels.h".                             */
/* 2. Define the LIBRARY_LOG_NAME and LIBRARY_LOG_LEVEL macros.               */
/* 3. Include the header file "logging_stack.h".                              */
/******************************************************************************/

/* 1. Include the header file "logging_levels.h". */
#include "logging_levels.h"

/* 2. Define the LIBRARY_LOG_NAME and LIBRARY_LOG_LEVEL macros. */
#ifndef LIBRARY_LOG_NAME
#define LIBRARY_LOG_NAME "LED"
#endif

#ifndef LIBRARY_LOG_LEVEL
#define LIBRARY_LOG_LEVEL LOG_INFO
#endif

/* 3. Include the header file "logging_stack.h". */
#include "logging_stack.h"

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Initialize a simple LED task.
 */
void vLEDTaskInstall(void);

#endif /* __FREERTOS_TASK_LED_H */

/***************************** END OF FILE ************************************/
