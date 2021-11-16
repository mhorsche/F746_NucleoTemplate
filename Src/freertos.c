/*
 * freertos.c
 *
 *  Created on: Nov 12, 2021
 *      Author: horsche
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"   /* RTOS task related API prototypes. */
#include "queue.h"  /* RTOS queue related API prototypes. */
#include "timers.h" /* Software timer related API prototypes. */
#include "semphr.h" /* Semaphore related API prototypes. */

/* FreeRTOS+TCP includes. */
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>
#include <FreeRTOS_DHCP.h>
#include "NetworkInterface.h"

/* Utilities includes. */
#include "logging.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim13;
volatile unsigned long ulHighFrequencyTimerTicks;

/* Private function prototypes -----------------------------------------------*/
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
// void vApplicationStackOverflowHook(TaskHandle_t pxTask, signed char *pcTaskName);

/* Private user code ---------------------------------------------------------*/
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
  ulHighFrequencyTimerTicks = 0;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 1079;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start_IT(&htim13);
}

__weak unsigned long getRunTimeCounterValue(void)
{
  return ulHighFrequencyTimerTicks;
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
  // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint32_t ulCount = 0;

  /* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
    1 in FreeRTOSConfig.h.

    "Give" the semaphore on every 500th tick interrupt. */
  ulCount++;
  if (ulCount >= 500UL)
  {
    /* This function is called from an interrupt context (the RTOS tick
        interrupt),    so only ISR safe API functions can be used (those that end
        in "FromISR()".

        xHigherPriorityTaskWoken was initialised to pdFALSE, and will be set to
        pdTRUE by xSemaphoreGiveFromISR() if giving the semaphore unblocked a
        task that has equal or higher priority than the interrupted task.
        NOTE: A semaphore is used for example purposes.  In a real application it
        might be preferable to use a direct to task notification,
        which will be faster and use less RAM. */
    // xSemaphoreGiveFromISR(xEventSemaphore, &xHigherPriorityTaskWoken);
    ulCount = 0UL;
  }

  /* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
    normally be performed before leaving the interrupt (because during the
    execution of the interrupt a task of equal or higher priority than the
    running task was unblocked).  The syntax required to context switch from
    an interrupt is port dependent, so check the documentation of the port you
    are using.

    In this case, the function is running in the context of the tick interrupt,
    which will automatically check for the higher priority task to run anyway,
    so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
  /* The malloc failed hook is enabled by setting
    configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

    Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  for (;;)
    ;
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)pcTaskName;
  (void)xTask;

  LogError(("vApplicationStackOverflowHook %s", pcTaskName));

  /* Run time stack overflow checking is performed if
    configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected.  pxCurrentTCB can be
    inspected in the debugger if the task name passed into this function is
    corrupt. */
  for (;;)
    ;
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
  volatile size_t xFreeStackSpace;

  /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
    FreeRTOSConfig.h.

    This function is called on each cycle of the idle task.  In this case it
    does nothing useful, other than report the amount of FreeRTOS heap that
    remains unallocated. */
  xFreeStackSpace = xPortGetFreeHeapSize();

  if (xFreeStackSpace > 100)
  {
    /* By now, the kernel has allocated everything it is going to, so
        if there is a lot of heap remaining unallocated then
        the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
        reduced accordingly. */
  }
}
/*-----------------------------------------------------------*/

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
  static BaseType_t xTasksAlreadyCreated = pdFALSE;
  uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
  char cBuffer[16];

  /* Both eNetworkUp and eNetworkDown events can be processed here. */
  if (eNetworkEvent == eNetworkUp)
  {
    /* Create the tasks that use the TCP/IP stack if they have not already
        been created. */
    if (xTasksAlreadyCreated == pdFALSE)
    {
      /* For convenience, tasks that use FreeRTOS+TCP can be created here
       * to ensure they are not created before the network is usable. */
      xTasksAlreadyCreated = pdTRUE;
    }
    /* The network is up and configured.  Print out the configuration,
     * which may have been obtained from a DHCP server. */
    FreeRTOS_GetAddressConfiguration(&ulIPAddress, &ulNetMask,
                                     &ulGatewayAddress, &ulDNSServerAddress);

    /* Convert the IP address to a string then print it out. */
    FreeRTOS_inet_ntoa(ulIPAddress, cBuffer);
    LogInfo(("IP Address: %s\r\n", cBuffer));

    /* Convert the net mask to a string then print it out. */
    FreeRTOS_inet_ntoa(ulNetMask, cBuffer);
    LogInfo(("Subnet Mask: %s\r\n", cBuffer));

    /* Convert the IP address of the gateway to a string then print it out. */
    FreeRTOS_inet_ntoa(ulGatewayAddress, cBuffer);
    LogInfo(("Gateway IP Address: %s\r\n", cBuffer));

    /* Convert the IP address of the DNS server to a string then print it out. */
    FreeRTOS_inet_ntoa(ulDNSServerAddress, cBuffer);
    LogInfo(("DNS server IP Address: %s\r\n", cBuffer));
  }
  else
  {
    LogInfo(("Ethernet is DOWN\r\n"));
  }
}
/*-----------------------------------------------------------*/
/**
 * @brief Callback that provides the inputs necessary to 
 * generate a randomized TCP Initial Sequence Number per 
 * RFC 6528.  In this case just a psuedo random number is used
 * so THIS IS NOT RECOMMENDED FOR PRODUCTION SYSTEMS.
 */
uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress, uint16_t usSourcePort, uint32_t ulDestinationAddress, uint16_t usDestinationPort)
{
  return HAL_RNG_GetRandomNumber(&hrng);
}
/*-----------------------------------------------------------*/

BaseType_t xApplicationGetRandomNumber(uint32_t *pulValue)
{
  HAL_StatusTypeDef xResult;
  BaseType_t xReturn;
  uint32_t ulValue;

  xResult = HAL_RNG_GenerateRandomNumber(&hrng, &ulValue);
  if (xResult == HAL_OK)
  {
    xReturn = pdPASS;
    *pulValue = ulValue;
  }
  else
  {
    xReturn = pdFAIL;
  }
  return xReturn;
}
/*-----------------------------------------------------------*/

#if (ipconfigSUPPORT_OUTGOING_PINGS == 1)
void vApplicationPingReplyHook(ePingReplyStatus_t eStatus, uint16_t usIdentifier)
{
  LogInfo(("Received ping ID %04X\n", usIdentifier));
}
#endif
/*-----------------------------------------------------------*/

/********************************** END OF FILE *******************************/
