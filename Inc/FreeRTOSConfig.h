/*
 * FreeRTOS Kernel V10.2.1
 * Portion Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Portion Copyright (C) 2019 StMicroelectronics, Inc.  All Rights Reserved.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/
// #define configENABLE_FPU 1
// #define configENABLE_MPU 0

#define configUSE_PREEMPTION 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_QUEUE_SETS 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 1
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configMAX_PRIORITIES (56)
#define configMINIMAL_STACK_SIZE ((unsigned short)130)
#define configTOTAL_HEAP_SIZE ((size_t)(46 * 1024))
#define configMAX_TASK_NAME_LEN (10)
#define configUSE_TRACE_FACILITY 1
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configUSE_MUTEXES 1
#define configQUEUE_REGISTRY_SIZE 8
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_APPLICATION_TASK_TAG 0
#define configUSE_COUNTING_SEMAPHORES 1

/* Allow static allocation for xStreamBufferGenericCreateStatic. */
#define configSUPPORT_STATIC_ALLOCATION 0

/* Run time stats gathering definitions. */
#define configGENERATE_RUN_TIME_STATS 1

/* Set configUSE_STATS_FORMATTING_FUNCTIONS to 2 to include the stats formatting
 * functions but without including stdio.h here. */
#define configUSE_STATS_FORMATTING_FUNCTIONS 2
#define configRECORD_STACK_HIGH_ADDRESS 1

/* The index within the target task's array of notification values to which the
 * notification is to be sent. uxIndexToNotify must be less than
 * configTASK_NOTIFICATION_ARRAY_ENTRIES. xTaskNotifyFromISR() does not have
 * this parameter and always sends notifications to index 0. */
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 2

/* Defaults to size_t for backward compatibility, but can be changed
   if lengths will always be less than the number of bytes in a size_t. */
#define configMESSAGE_BUFFER_LENGTH_TYPE size_t

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 1
#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES (2)

/* Software timer definitions. */
#define configUSE_TIMERS 1
#define configTIMER_SERVICE_TASK_NAME "TIMER"
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH 5
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2)

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskCleanUpResources 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_eTaskGetState 1
#define INCLUDE_xTimerPendFunctionCall 1

#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xQueueGetMutexHolder 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
// #define USE_FREERTOS_HEAP_5
// #define configAPPLICATION_ALLOCATED_HEAP 1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4 /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 4

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define xPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the
timebase source is SysTick, to prevent overwriting SysTick_Handler defined within
STM32Cube HAL */
// #define xPortSysTickHandler SysTick_Handler

/* Definitions needed when configGENERATE_RUN_TIME_STATS is on */
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS vStartHighResolutionTimer
#define portGET_RUN_TIME_COUNTER_VALUE ullGetMicrosecondTime

/* Defaults to uint32_t for backward compatibility, but can be changed to
 * uint64_t if run time counter is of higher accuracy (eg. usec). */
#define configRUN_TIME_COUNTER_TYPE uint64_t

/* Exported functions --------------------------------------------------------*/

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)

/* Library includes. */
#include "stm32f7xx_hal.h"

extern uint32_t SystemCoreClock;

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
extern void vAssertCalled(uint32_t ulLine, const char *pcFile);
#define configASSERT(x) \
  if ((x) == 0)         \
  vAssertCalled(__LINE__, __FILE__)

#endif /*  defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__) */

/**
 * @brief Used to define multiple heap regions for use by heap_5.c. This
 *    function must be called before any calls to pvPortMalloc() - not creating
 *    a task, queue, semaphore, mutex, software timer, event group, etc. will
 *    result in pvPortMalloc being called.
 *
 * @see freertos.c
 */
void vHeapInit(void);

/**
 * @brief  Initializes TIM peripherals used by the FreeRTOS driver.
 */
void vStartHighResolutionTimer(void);

/**
 * @brief  Return high resolution (us) timer value for task runtime statistics.
 *
 * @retval (uint64_t) Timer value in microseconds.
 */
uint64_t ullGetMicrosecondTime(void);

/**
 * @brief Helper functions for simple runtime measurements. Use tic/toc to start
 *    and stop runtime and print status message.
 *
 * @see freertos.c
 */

void vTic(const char *function, uint32_t line);
#define tic() vTic(__FUNCTION__, __LINE__)
void vToc(const char *function, uint32_t line);
#define toc() vToc(__FUNCTION__, __LINE__)

/**
 * @brief Initialize and start Random Number Generator used for sequence number
 *    by TCP/IP stack.
 *
 * @see freertos.c
 */
void vRandomNumberGeneratorInit(void);

/**
 * @brief  Use by the pseudo random number generator.
 *
 * @retval (uint32_t) Random number using STM32_HAL random number generator.
 */
uint32_t uxRand(void);

/**
 * @brief Callback that provides the inputs necessary to
 *    generate a randomized TCP Initial Sequence Number per RFC 6528.
 *
 * @see freertos.c
 *
 * @param[in] ulSourceAddress:
 * @param[in] usSourcePort:
 * @param[in] ulDestinationAddress:
 * @param[in] usDestinationPort:
 *
 * @return (uint32_t) Random number using STM32_HAL random number generator.
 */
uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort); /* @see freertos.c */

/**
 * @brief This xApplicationGetRandomNumber() will set *pulNumber to a random
 *    number, and return pdTRUE. When the random number generator is broken,
 *    it shall return pdFALSE.
 *
 * @see freertos.c
 *
 * @param[out] pulValue: pointer to value which should be set.
 * @retval pdTRUE on success, otherwise pdFALSE and pulValue will be set to 0.
 */
int32_t xApplicationGetRandomNumber(uint32_t *pulValue); /* @see freertos.c */

/**
 * @brief Show task manager.
 *
 * @see freertos.c
 *
 * @param[in] usDoClear: clear/reset stats and set new reference time to
 *    current timestamp.
 */
void vShowTaskTable(uint16_t usDoClear);

#endif /* FREERTOS_CONFIG_H */

/********************************** END OF FILE *******************************/