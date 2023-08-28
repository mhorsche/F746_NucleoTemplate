/**
 * @file freertos.c
 * @author horsche (horsche@li.plus)
 * @brief
 * @version
 * @date 2023-08-11
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

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
#include "freertos_task_logging.h"

/* MQTT library includes. */
// #include "core_mqtt.h"

/* Application include. */
#include "freertos_task_iperf.h"
// #include "freertos_task_mqtt.h"
// #include "freertos_task_modbus.h"

/* Private variables ---------------------------------------------------------*/
static uint64_t ullTic = 0;
static uint64_t ullToc = 0;

static uint64_t ullHiresTime = 0;

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

/* Function prototypes -------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/******************************************************************************/
/*                          Public/Exported Functions                         */
/******************************************************************************/

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

void vStartHighResolutionTimer(void)
{
  /* Initialize and start FreeRTOS timer. */
  RTOS_TIM_Init();
}
/*-----------------------------------------------------------*/

uint64_t ullGetMicrosecondTime(void)
{
  uint64_t ullReturn;
  ullReturn = RTOS_TIM_GetValue();

  return ullReturn;
}
/*-----------------------------------------------------------*/

void vTic(const char *function, uint32_t line)
{
  FreeRTOS_printf(("%s:%d tic: %lld usec\r\n", function, line, ullGetMicrosecondTime()));
  ullTic = ullGetMicrosecondTime();
}
/*-----------------------------------------------------------*/

void vToc(const char *function, uint32_t line)
{
  ullToc = ullGetMicrosecondTime();
  FreeRTOS_printf(("%s:%d toc: %lld usec (diff %lld)\r\n", function, line, ullToc, (ullToc - ullTic)));
}
/*-----------------------------------------------------------*/

void vRandomNumberGeneratorInit(void)
{
  /* Initialize and start RNG. */
  RTOS_RNG_Init();
}
/*-----------------------------------------------------------*/

int32_t xApplicationGetRandomNumber(uint32_t *pulValue)
{
  HAL_StatusTypeDef xReturn;

  /* Use RNG module to generate random number. */
  xReturn = RTOS_RNG_GenerateRandomNumber(pulValue);

  return (xReturn == HAL_OK ? pdPASS : pdFAIL);
}
/*-----------------------------------------------------------*/

uint32_t uxRand(void)
{
  BaseType_t xReturn;
  uint32_t ulValue;

  /* Use RNG module to generate random number. */
  xReturn = xApplicationGetRandomNumber(&ulValue);
  if (xReturn == pdFAIL)
  {
    /* Failed, ulValue will be zero */
    FreeRTOS_printf(("Random number failed, mapped value is zero.\r\n"));
  }
  return ulValue;
}
/*-----------------------------------------------------------*/

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort)
{
  return uxRand();
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
  FreeRTOS_printf(("vApplicationMallocFailedHook\r\n"));

  /* The malloc failed hook is enabled by setting
   * configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.
   * Called if a call to pvPortMalloc() fails because there is insufficient
   * free memory available in the FreeRTOS heap.  pvPortMalloc() is called
   * internally by FreeRTOS API functions that create tasks, queues, software
   * timers, and semaphores.  The size of the FreeRTOS heap is set by the
   * configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  vAssertCalled(__LINE__, __FUNCTION__);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)pcTaskName;
  (void)xTask;

  FreeRTOS_printf(("vApplicationStackOverflowHook %s\r\n", pcTaskName));

  /* Run time stack overflow checking is performed if
   * configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
   * function is called if a stack overflow is detected.  pxCurrentTCB can be
   * inspected in the debugger if the task name passed into this function is
   * corrupt. */
  vAssertCalled(__LINE__, __FUNCTION__);
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
      // vIPerfInstall();

      // ModbusServerConfig_t xModbusServerConfig;
      // vModbusInstall(xModbusServerConfig);

      // /* Otherwise use notification */
      // xTaskNotifyGive(xMQTTTaskHandle);

      xTasksAlreadyCreated = pdTRUE;
    }
    /* The network is up and configured.  Print out the configuration,
     * which may have been obtained from a DHCP server. */
    FreeRTOS_GetAddressConfiguration(&ulIPAddress, &ulNetMask,
                                     &ulGatewayAddress, &ulDNSServerAddress);

    /* Convert the IP address to a string then print it out. */
    FreeRTOS_inet_ntoa(ulIPAddress, cBuffer);
    FreeRTOS_printf(("IP Address: %s\r\n", cBuffer));

    /* Convert the net mask to a string then print it out. */
    FreeRTOS_inet_ntoa(ulNetMask, cBuffer);
    FreeRTOS_printf(("Subnet Mask: %s\r\n", cBuffer));

    /* Convert the IP address of the gateway to a string then print it out. */
    FreeRTOS_inet_ntoa(ulGatewayAddress, cBuffer);
    FreeRTOS_printf(("Gateway IP Address: %s\r\n", cBuffer));

    /* Convert the IP address of the DNS server to a string then print it out. */
    FreeRTOS_inet_ntoa(ulDNSServerAddress, cBuffer);
    FreeRTOS_printf(("DNS server IP Address: %s\r\n", cBuffer));
  }
  else
  {
    FreeRTOS_printf(("Ethernet is DOWN\r\n"));
  }
}
/*-----------------------------------------------------------*/

#if (ipconfigSUPPORT_OUTGOING_PINGS == 1)
void vApplicationPingReplyHook(ePingReplyStatus_t eStatus, uint16_t usIdentifier)
{
  FreeRTOS_printf(("Ping status %d ID = %04X\r\n", eStatus, usIdentifier));
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

  FreeRTOS_printf(("Comp '%s' with '%s'/'%s' %ld\r\n", pcName, mainHOST_NAME, mainDEVICE_NICK_NAME, rc));

  return rc;
}
/*-----------------------------------------------------------*/

void vShowTaskTable(uint16_t usDoClear)
{
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  uint64_t ullTotalRunTimeDiv100;

  /* Take a snapshot of the number of tasks in case it changes while this
   * function is executing. */
  uxArraySize = uxTaskGetNumberOfTasks();

  /* Allocate a TaskStatus_t structure for each task.  An array could be
   * allocated statically at compile time. */
  pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

  FreeRTOS_printf(("\r\n+----+---------------+------+-------+-----------------+-------+\r\n"));
  FreeRTOS_printf(("| no | Task name     | Prio | Stack |        Time(uS) |  Perc |\r\n"));
  FreeRTOS_printf(("+----+---------------+------+-------+-----------------+-------+\r\n"));

  if (pxTaskStatusArray != NULL)
  {
    /* Generate raw status information about each task. */
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);

    ullTotalRunTimeDiv100 = (ullGetMicrosecondTime() - ullHiresTime) / 100;

    /* Avoid divide by zero errors. */
    if (ullTotalRunTimeDiv100 > 0ull)
    {
      /* For each populated position in the pxTaskStatusArray array,
       * format the raw data as human readable ASCII data */
      for (x = 0; x < uxArraySize; x++)
      {
        FreeRTOS_printf(("| %2d | %-13.13s | %4lu | %5u | %15" PRIu64 " | %5.2f |\r\n",
                         x,
                         pxTaskStatusArray[x].pcTaskName,
                         pxTaskStatusArray[x].uxCurrentPriority,
                         pxTaskStatusArray[x].usStackHighWaterMark,
                         pxTaskStatusArray[x].ulRunTimeCounter,
                         (float)(pxTaskStatusArray[x].ulRunTimeCounter) / (float)(ullTotalRunTimeDiv100)));
      }
    }

    /* The array is no longer needed, free the memory it consumes. */
    vPortFree(pxTaskStatusArray);
  }
  FreeRTOS_printf(("+----+---------------+------+-------+-----------------+-------+\r\n"));
  FreeRTOS_printf(("| Heap: min %6lu / free %6lu / total %6lu / stack %5lu |\r\n",
                   (uint32_t)xPortGetMinimumEverFreeHeapSize(),
                   (uint32_t)xPortGetFreeHeapSize(),
                   (uint32_t)ulHeapSize,
                   (uint32_t) & (_Min_Stack_Size)));

  FreeRTOS_printf(("| Network Buffers: min %3u / free %3u / total %3u             |\r\n",
                   (uint16_t)uxGetMinimumFreeNetworkBuffers(),
                   (uint16_t)uxGetNumberOfFreeNetworkBuffers(),
                   ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS));
  FreeRTOS_printf(("+-------------------------------------------------------------+\r\n\r\n"));

  if (usDoClear != pdFALSE)
  {
    ullHiresTime = ullGetMicrosecondTime();
  }
}
/*-----------------------------------------------------------*/

/********************************** END OF FILE *******************************/
