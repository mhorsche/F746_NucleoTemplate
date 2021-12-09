/**
 * @file freertos.c
 * @author horsche (horsche@li.plus)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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

/* MQTT library includes. */
#include "core_mqtt.h"

/* Application include. */
#include "mqtt_task.h"
#include "iperf_task.h"
#include "modbus_task.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

static const uint32_t ulReloadCount = 10000000ul;
static const uint32_t ulPrescale = 108ul; /* Timer running at 108MHz / 108 = 1MHz */

uint32_t ulInterruptCount = 0;
uint32_t ulTimer2Flags;

static volatile uint64_t ullHiresTime;

static volatile uint32_t ulHeapSize;
static volatile uint8_t *pucHeapStart;

#if defined(__IAR_SYSTEMS_ICC__)
uint8_t heapMemory[100000];
#else
extern uint8_t __bss_end__, _estack, _Min_Stack_Size;
#endif

/* Private define ------------------------------------------------------------*/
#if defined(__IAR_SYSTEMS_ICC__)
#define HEAP_START heapMemory[0]
#define HEAP_END heapMemory[sizeof heapMemory]
#else
#define HEAP_START __bss_end__
#define HEAP_END _estack
#endif

/* Private function prototypes -----------------------------------------------*/
void vStartHighResolutionTimer(void);
uint64_t ullGetMicrosecondTime(void);

/* Private user code ---------------------------------------------------------*/

/* Timer2 initialization function */
void vStartHighResolutionTimer(void)
{
  htim2.Instance = TIM2;                             /* Register base address */
  htim2.Init.Prescaler = (ulPrescale - 1ul);         /* Specifies the prescaler value used to divide the TIM clock. */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;       /* Specifies the counter mode. */
  htim2.Init.Period = (ulReloadCount - 1ul);         /* Specifies the period value to be loaded into the active. */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; /* Specifies the clock division. */
  htim2.Init.RepetitionCounter = 0ul;                /* Specifies the repetition counter value. */
  htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_1;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_Base_Start_IT(&htim2);

  /* Ignore the initial interrupt which sets ulInterruptCount = 1. */
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  ulInterruptCount = 0ul;
}
/*-----------------------------------------------------------*/

/**
 * @brief Return high resolution (us) timer value for task runtime statistics.
 * 
 * @return uint64_t
 */
uint64_t ullGetMicrosecondTime(void)
{
  uint64_t ullReturn;
  if (htim2.Instance == NULL)
  {
    ullReturn = 1000ull * xTaskGetTickCount();
  }
  else
  {
    uint32_t ulCounts[2];
    uint32_t ulSlowCount;

    for (;;)
    {
      ulCounts[0] = htim2.Instance->CNT;
      ulSlowCount = ulInterruptCount;
      ulCounts[1] = htim2.Instance->CNT;
      if (ulCounts[1] >= ulCounts[0])
      {
        /* TIM2_IRQHandler() has not occurred in between. */
        break;
      }
    }
    ullReturn = (uint64_t)ulSlowCount * ulReloadCount + ulCounts[1];
  }

  return ullReturn;
}
/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
  // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint32_t ulCount = 0;

  /* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
   * 1 in FreeRTOSConfig.h.
   * "Give" the semaphore on every 500th tick interrupt. */
  ulCount++;
  if (ulCount >= 500UL)
  {
    /* This function is called from an interrupt context (the RTOS tick
     * interrupt),    so only ISR safe API functions can be used (those that end
     * in "FromISR()".
     * xHigherPriorityTaskWoken was initialised to pdFALSE, and will be set to
     * pdTRUE by xSemaphoreGiveFromISR() if giving the semaphore unblocked a
     * task that has equal or higher priority than the interrupted task.
     * NOTE: A semaphore is used for example purposes.  In a real application it
     * might be preferable to use a direct to task notification,
     * which will be faster and use less RAM. */
    // xSemaphoreGiveFromISR(xEventSemaphore, &xHigherPriorityTaskWoken);
    ulCount = 0UL;
  }

  /* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
   * normally be performed before leaving the interrupt (because during the
   * execution of the interrupt a task of equal or higher priority than the
   * running task was unblocked).  The syntax required to context switch from
   * an interrupt is port dependent, so check the documentation of the port you
   * are using.
   * In this case, the function is running in the context of the tick interrupt,
   * which will automatically check for the higher priority task to run anyway,
   * so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
  LogError(("vApplicationMallocFailedHook"));

  /* The malloc failed hook is enabled by setting
   * configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.
   * Called if a call to pvPortMalloc() fails because there is insufficient
   * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
   * internally by FreeRTOS API functions that create tasks, queues, software
   * timers, and semaphores.  The size of the FreeRTOS heap is set by the
   * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  Error_Handler();
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)pcTaskName;
  (void)xTask;

  LogError(("vApplicationStackOverflowHook %s", pcTaskName));

  /* Run time stack overflow checking is performed if
   * configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
   * function is called if a stack overflow is detected.  pxCurrentTCB can be
   * inspected in the debugger if the task name passed into this function is
   * corrupt. */
  Error_Handler();
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
  volatile size_t xFreeStackSpace;

  /* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
   * FreeRTOSConfig.h.
   * This function is called on each cycle of the idle task.  In this case it
   * does nothing useful, other than report the amount of FreeRTOS heap that
   * remains unallocated. */
  xFreeStackSpace = xPortGetFreeHeapSize();

  if (xFreeStackSpace > 100)
  {
    /* By now, the kernel has allocated everything it is going to, so
     * if there is a lot of heap remaining unallocated then
     * the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
     * reduced accordingly. */
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
     * been created. */
    if (xTasksAlreadyCreated == pdFALSE)
    {
      /* For convenience, tasks that use FreeRTOS+TCP can be created here
       * to ensure they are not created before the network is usable. */
      vIPerfInstall();

      ModbusServerConfig_t xModbusServerConfig;
      vModbusInstall(xModbusServerConfig);

      /* Otherwise use notification */
      xTaskNotifyGive(xMQTTTaskHandle);

      xTasksAlreadyCreated = pdTRUE;
    }
    /* The network is up and configured.  Print out the configuration,
     * which may have been obtained from a DHCP server. */
    FreeRTOS_GetAddressConfiguration(&ulIPAddress, &ulNetMask,
                                     &ulGatewayAddress, &ulDNSServerAddress);

    /* Convert the IP address to a string then print it out. */
    FreeRTOS_inet_ntoa(ulIPAddress, cBuffer);
    LogInfo(("IP Address: %s", cBuffer));

    /* Convert the net mask to a string then print it out. */
    FreeRTOS_inet_ntoa(ulNetMask, cBuffer);
    LogInfo(("Subnet Mask: %s", cBuffer));

    /* Convert the IP address of the gateway to a string then print it out. */
    FreeRTOS_inet_ntoa(ulGatewayAddress, cBuffer);
    LogInfo(("Gateway IP Address: %s", cBuffer));

    /* Convert the IP address of the DNS server to a string then print it out. */
    FreeRTOS_inet_ntoa(ulDNSServerAddress, cBuffer);
    LogInfo(("DNS server IP Address: %s", cBuffer));
  }
  else
  {
    LogInfo(("Ethernet is DOWN"));
  }
}
/*-----------------------------------------------------------*/

/**
 * @brief Callback that provides the inputs necessary to 
 * generate a randomized TCP Initial Sequence Number per 
 * RFC 6528.  In this case just a psuedo random number is used
 * so THIS IS NOT RECOMMENDED FOR PRODUCTION SYSTEMS.
 * 
 * @return uint32_t random number using STM32_HAL random number generator
 */
uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress, uint16_t usSourcePort, uint32_t ulDestinationAddress, uint16_t usDestinationPort)
{
  HAL_StatusTypeDef xResult;
  uint32_t ulValue;

  xResult = HAL_RNG_GenerateRandomNumber(&xRNG, &(xRNG.RandomNumber));
  if (xResult == HAL_OK)
  {
    ulValue = xRNG.RandomNumber;
  }
  else
  {
    ulValue = 0;
  }
  return ulValue;
}
/*-----------------------------------------------------------*/

BaseType_t xApplicationGetRandomNumber(uint32_t *pulValue)
{
  HAL_StatusTypeDef xResult;
  BaseType_t xReturn;

  xResult = HAL_RNG_GenerateRandomNumber(&xRNG, &(xRNG.RandomNumber));
  if (xResult == HAL_OK)
  {
    xReturn = pdPASS;
    *pulValue = xRNG.RandomNumber;
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
  LogInfo(("Ping status %d ID = %04X", eStatus, usIdentifier));
}
#endif
/*-----------------------------------------------------------*/

#if (ipconfigUSE_DHCP_HOOK == 1)
eDHCPCallbackAnswer_t xApplicationDHCPHook(eDHCPCallbackPhase_t eDHCPPhase,
                                           uint32_t ulIPAddress)
{
  eDHCPCallbackAnswer_t eReturn;
  uint32_t ulStaticIPAddress, ulStaticNetMask;

  /* This hook is called in a couple of places during the DHCP process, as
   * identified by the eDHCPPhase parameter. */
  switch (eDHCPPhase)
  {
  case eDHCPPhasePreDiscover:
    /* A DHCP discovery is about to be sent out.  eDHCPContinue is
     * returned to allow the discovery to go out.
     * If eDHCPUseDefaults had been returned instead then the DHCP process
     * would be stopped and the statically configured IP address would be
     * used.
     * If eDHCPStopNoChanges had been returned instead then the DHCP
     * process would be stopped and whatever the current network
     * configuration was would continue to be used. */
    eReturn = eDHCPContinue;
    break;

  case eDHCPPhasePreRequest:
    /* An offer has been received from the DHCP server, and the offered
     * IP address is passed in the ulIPAddress parameter.  Convert the
     * offered and statically allocated IP addresses to 32-bit values. */
    ulStaticIPAddress = FreeRTOS_inet_addr_quick(ipconfigIP_ADDR0, ipconfigIP_ADDR1, ipconfigIP_ADDR2, ipconfigIP_ADDR3);
    ulStaticNetMask = FreeRTOS_inet_addr_quick(ipconfigNET_MASK0, ipconfigNET_MASK1, ipconfigNET_MASK2, ipconfigNET_MASK3);

    /* Mask the IP addresses to leave just the sub-domain octets. */
    ulStaticIPAddress &= ulStaticNetMask;
    ulIPAddress &= ulStaticNetMask;

    /* Are the sub-domains the same? */
    if (ulStaticIPAddress == ulIPAddress)
    {
      /* The sub-domains match, so the default IP address can be
       * used.  The DHCP process is stopped at this point. */
      eReturn = eDHCPUseDefaults;
    }
    else
    {
      /* The sub-domains don't match, so continue with the DHCP
       * process so the offered IP address is used. */
      eReturn = eDHCPContinue;
    }
    break;

  default:
    /* Cannot be reached, but set eReturn to prevent compiler warnings
     * where compilers are disposed to generating one. */
    eReturn = eDHCPContinue;
    break;
  }

  return eReturn;
}
#endif
/*-----------------------------------------------------------*/

#if (ipconfigUSE_LLMNR != 0) || (ipconfigUSE_NBNS != 0) || (ipconfigDHCP_REGISTER_HOSTNAME == 1)
const char *pcApplicationHostnameHook(void)
{
  /* Assign the host name to this network node.  This function will
   * be called during the DHCP: the machine will be registered with an IP
   * address plus this name. */
  return mainHOST_NAME;
}
#endif

/*-----------------------------------------------------------*/

BaseType_t xApplicationDNSQueryHook(const char *pcName)
{
  BaseType_t rc = strcasecmp(mainHOST_NAME, pcName) == 0 || strcasecmp(mainDEVICE_NICK_NAME, pcName) == 0;

  LogInfo(("Comp '%s' with '%s'/'%s' %ld\n", pcName, mainHOST_NAME, mainDEVICE_NICK_NAME, rc));

  return rc;
}
/*-----------------------------------------------------------*/

void vHeapInit(void)
{
  uint32_t ulStackSize = (uint32_t) & (_Min_Stack_Size);

  pucHeapStart = (uint8_t *)((((uint32_t)&HEAP_START) + 7) & ~0x07ul);

  ulHeapSize = (uint32_t)(&HEAP_END - &HEAP_START);
  ulHeapSize &= ~0x07ul;
  ulHeapSize -= ulStackSize;

  HeapRegion_t xHeapRegions[] = {
      {(unsigned char *)pucHeapStart, ulHeapSize},
      {NULL, 0}};

  vPortDefineHeapRegions(xHeapRegions);
}
/*-----------------------------------------------------------*/

void vShowTaskTable(BaseType_t xDoClear)
{
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  uint64_t ullTotalRunTime;
  uint32_t ulStatsAsPermille;
  uint32_t ulStackSize;

  /* Take a snapshot of the number of tasks in case it changes while this
   * function is executing. */
  uxArraySize = uxTaskGetNumberOfTasks();

  /* Allocate a TaskStatus_t structure for each task.  An array could be
   * allocated statically at compile time. */
  pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

  LogInfo(("Task name    Prio    Stack     Time(uS)     Perc"));

  if (pxTaskStatusArray != NULL)
  {
    /* Generate raw status information about each task. */
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);

    ullTotalRunTime = ullGetMicrosecondTime() - ullHiresTime;

    /* For percentage calculations. */
    ullTotalRunTime /= 1000UL;

    /* Avoid divide by zero errors. */
    if (ullTotalRunTime > 0ull)
    {
      /* For each populated position in the pxTaskStatusArray array,
       * format the raw data as human readable ASCII data */
      for (x = 0; x < uxArraySize; x++)
      {
        /* What percentage of the total run time has the task used?
         * This will always be rounded down to the nearest integer.
         * ulTotalRunTimeDiv100 has already been divided by 100. */
        ulStatsAsPermille = pxTaskStatusArray[x].ulRunTimeCounter / ullTotalRunTime;

        LogInfo(("%-14.14s %2lu %8u %12lu  %4lu.%lu %%",
                 pxTaskStatusArray[x].pcTaskName,
                 pxTaskStatusArray[x].uxCurrentPriority,
                 pxTaskStatusArray[x].usStackHighWaterMark,
                 pxTaskStatusArray[x].ulRunTimeCounter,
                 ulStatsAsPermille / 10,
                 ulStatsAsPermille % 10));
      }
    }

    /* The array is no longer needed, free the memory it consumes. */
    vPortFree(pxTaskStatusArray);
  }
  ulStackSize = (uint32_t) & (_Min_Stack_Size);
  LogInfo(("Heap: min/cur/max: %lu %lu %lu stack %lu\r\n",
           (uint32_t)xPortGetMinimumEverFreeHeapSize(),
           (uint32_t)xPortGetFreeHeapSize(),
           (uint32_t)ulHeapSize,
           (uint32_t)ulStackSize));
  if (xDoClear != pdFALSE)
  {
    ullHiresTime = ullGetMicrosecondTime();
  }
}
/*-----------------------------------------------------------*/

/********************************** END OF FILE *******************************/
