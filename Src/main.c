/**
 * @file main.c
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

/* Standard includes. */
#include <time.h>

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>
#include <FreeRTOS_DHCP.h>
#include "NetworkInterface.h"

/* Utilities includes. */
#include "freertos_task_logging.h"
#include "freertos_task_led.h"
// #include "freertos_mqtt_task.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* The MAC address array is not declared const as the MAC address will
 * normally be read from an EEPROM and not hard coded (in real deployed
 * applications) .*/
static uint8_t ucMACAddress[6] = {ipconfigMAC_ADDR0, ipconfigMAC_ADDR1, ipconfigMAC_ADDR2, 0 /* ipconfigMAC_ADDR3 */, 0 /* ipconfigMAC_ADDR4 */, 0 /* ipconfigMAC_ADDR5 */};

/* Define the network addressing.  These parameters will be used if either
 * ipconfigUDE_DHCP is 0 or if ipconfigUSE_DHCP is 1 but DHCP auto configuration
 * failed. */
static const uint8_t ucIPAddress[4] = {ipconfigIP_ADDR0, ipconfigIP_ADDR1, ipconfigIP_ADDR2, ipconfigIP_ADDR3};
static const uint8_t ucNetMask[4] = {ipconfigNET_MASK0, ipconfigNET_MASK1, ipconfigNET_MASK2, ipconfigNET_MASK3};
static const uint8_t ucGatewayAddress[4] = {ipconfigGATEWAY_ADDR0, ipconfigGATEWAY_ADDR1, ipconfigGATEWAY_ADDR2, ipconfigGATEWAY_ADDR3};
static const uint8_t ucDNSServerAddress[4] = {ipconfigDNS_SERVER_ADDR0, ipconfigDNS_SERVER_ADDR1, ipconfigDNS_SERVER_ADDR2, ipconfigDNS_SERVER_ADDR3};

/* Set the following constant to pdTRUE to log using the method indicated by the
 * name of the constant, or pdFALSE to not log using the method indicated by the
 * name of the constant.  Options include to UART out (xLogToUART), to SWO
 * (xLogToSWO), and to a UDP port (xLogToUDP).  If xLogToUDP is set to pdTRUE
 * then UDP messages are sent to the IP address configured as the echo server
 * address (see the configECHO_SERVER_ADDR0 definitions in FreeRTOSConfig.h) and
 * the port number set by configPRINT_PORT in FreeRTOSConfig.h. */
const BaseType_t xLogToUART = pdTRUE, xLogToSWO = pdFALSE, xLogToUDP = pdFALSE;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief System Clock is configured as follow:
 *    System Clock source            = PLL (HSE)
 *    SYSCLK(Hz)                     = 216000000
 *    HCLK(Hz)                       = 216000000
 *    AHB Prescaler                  = 1
 *    APB1 Prescaler                 = 4
 *    APB2 Prescaler                 = 2
 *    HSE Frequency(Hz)              = 8000000
 *    PLL_M                          = 8
 *    PLL_N                          = 432
 *    PLL_P                          = 2
 *    PLL_Q                          = 9
 *    VDD(V)                         = 3.3
 *    Main regulator output voltage  = Scale1 mode
 *    Flash Latency(WS)              = 7
 */
static void SystemClock_Config(void);

/**
 * @brief CPU L1-Cache configuration.
 */
static void Cache_Config(void);

/**
 * @brief Configure the MPU attributes.
 */
static void MPU_Config(void);

/**
 * @brief Configure Debug Mode to stop all timers.
 */
static void DebugMode_Config(void);

/******************************************************************************/
/*                          Public/Exported Functions                         */
/******************************************************************************/

int main(void)
{
  /* Configure the MPU attributes */
  MPU_Config();

  /* Enable the CPU Cache */
  Cache_Config();

  /* STM32F7xx HAL library initialization:
   *   - Configure the Flash ART accelerator
   *   - Systick timer is configured by default as source of time base, but user
   *     can eventually implement his proper time base source (a general purpose
   *     timer for example or other time source), keeping in mind that Time base
   *     duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
   *     handled in milliseconds basis.
   *   - Set NVIC Group Priority to 4
   *   - Low Level Initialization
   */
  HAL_Init();

  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Configure debug mode to stop all timers */
  DebugMode_Config();

  /* Initialise a heap in SRAM */
  vHeapInit();

  /* Initialize and start RNG. */
  vRandomNumberGeneratorInit();

  /* Initialize logging for libraries that depend on it. */
  uint32_t ulLoggingIPAddress = 0;
  ulLoggingIPAddress = FreeRTOS_inet_addr_quick(ipconfigLOGGING_SERVER_ADDR0, ipconfigLOGGING_SERVER_ADDR1, ipconfigLOGGING_SERVER_ADDR2, ipconfigLOGGING_SERVER_ADDR3);
  vLoggingInit(xLogToUART, xLogToSWO, xLogToUDP, ulLoggingIPAddress, ipconfigLOGGING_SERVER_PORT);

  FreeRTOS_printf(("\r\n#### Template %s: %s %s ####\r\n(DevID: 0x%X / RevID: 0x%X / %d kByte)\r\n\r\n",
                   FW_VERSION, __DATE__, __TIME__,
                   (uint16_t)stm32f7xxGET_SIGNATURE(),
                   (uint16_t)stm32f7xxGET_REVISION(),
                   (uint16_t)stm32f7xxGET_FLASH_SIZE()));

  /* Initialise the RTOS's TCP/IP stack.  The tasks that use the network
   * are created in the vApplicationIPNetworkEventHook() hook function
   * below.  The hook function is called when the network connects. */
  FreeRTOS_IPInit(ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress);

  /* Definition and creation of FreeRTOS Threads (Tasks) */
  vLEDTaskInstall();

  /* Start the tasks and timer running. */
  vTaskStartScheduler();

  /* If all is well, the scheduler will now be running, and the following
  line will never be reached.  If the following line does execute, then
  there was insufficient FreeRTOS heap memory available for the Idle and/or
  timer tasks to be created.  See the memory management section on the
  FreeRTOS web site for more details on the FreeRTOS heap
  http://www.freertos.org/a00111.html. */
  while (1)
  {
  }
}
/*-----------------------------------------------------------*/

/******************************************************************************/
/*                         Private/Static Functions                           */
/******************************************************************************/

static void SystemClock_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure LSE Drive Capability */
  HAL_PWR_EnableBkUpAccess();

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  // RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  // RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  // RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  // RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  // RCC_OscInitStruct.PLL.PLLM = 8;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;

  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if ((ret = HAL_RCC_OscConfig(&RCC_OscInitStruct)) != HAL_OK)
  {
    vAssertCalled(__LINE__, __FUNCTION__);
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  if ((ret = HAL_PWREx_EnableOverDrive()) != HAL_OK)
  {
    vAssertCalled(__LINE__, __FUNCTION__);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
   * clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if ((ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7)) != HAL_OK)
  {
    vAssertCalled(__LINE__, __FUNCTION__);
  }

  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}
/*-----------------------------------------------------------*/

static void Cache_Config(void)
{
  /* Invalidates Instruction and Data Cache */
  SCB_InvalidateICache();
  SCB_InvalidateDCache();

  /* Enable/Disable I-Cache */
  SCB_DisableICache();
  // SCB_EnableICache();

  /* Enable/Disable D-Cache */
  SCB_DisableDCache();
  // SCB_EnableDCache();
}
/*-----------------------------------------------------------*/

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  LL_MPU_Disable();

  extern int __ETH_DESCRIPTORS_START, __ETH_BUFFERS_START;

  LL_MPU_ConfigRegion(
      LL_MPU_REGION_NUMBER0,
      0x0,
      (uint32_t)&__ETH_DESCRIPTORS_START,
      LL_MPU_REGION_SIZE_1KB | LL_MPU_TEX_LEVEL0 | LL_MPU_REGION_FULL_ACCESS |
          LL_MPU_INSTRUCTION_ACCESS_DISABLE | LL_MPU_ACCESS_NOT_SHAREABLE | LL_MPU_ACCESS_NOT_CACHEABLE |
          LL_MPU_ACCESS_BUFFERABLE);
  LL_MPU_EnableRegion(LL_MPU_REGION_NUMBER0);

  LL_MPU_ConfigRegion(
      LL_MPU_REGION_NUMBER1,
      0x0,
      (uint32_t)&__ETH_BUFFERS_START,
      LL_MPU_REGION_SIZE_64KB | LL_MPU_TEX_LEVEL1 | LL_MPU_REGION_FULL_ACCESS |
          LL_MPU_INSTRUCTION_ACCESS_DISABLE | LL_MPU_ACCESS_NOT_SHAREABLE | LL_MPU_ACCESS_NOT_CACHEABLE |
          LL_MPU_ACCESS_NOT_BUFFERABLE);
  LL_MPU_EnableRegion(LL_MPU_REGION_NUMBER1);

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}
/*-----------------------------------------------------------*/

static void DebugMode_Config(void)
{
  /* Configure debug mode on APB2 */
  //  __HAL_RCC_DBGMCU_CLK_ENABLE ();

  /* Freeze watchdog during debug mode */
  __HAL_DBGMCU_FREEZE_WWDG();
  __HAL_DBGMCU_FREEZE_IWDG();

  /* Freeze ALL timer during debug mode */
  __HAL_DBGMCU_FREEZE_RTC();
  __HAL_DBGMCU_FREEZE_TIM1();
  __HAL_DBGMCU_FREEZE_TIM2();
  __HAL_DBGMCU_FREEZE_TIM3();
  __HAL_DBGMCU_FREEZE_TIM4();
  __HAL_DBGMCU_FREEZE_TIM5();
  __HAL_DBGMCU_FREEZE_TIM6();
  __HAL_DBGMCU_FREEZE_TIM7();
  __HAL_DBGMCU_FREEZE_TIM8();
  __HAL_DBGMCU_FREEZE_TIM9();
  __HAL_DBGMCU_FREEZE_TIM10();
  __HAL_DBGMCU_FREEZE_TIM11();
  __HAL_DBGMCU_FREEZE_TIM12();
  __HAL_DBGMCU_FREEZE_TIM13();
  __HAL_DBGMCU_FREEZE_TIM14();
}
/*-----------------------------------------------------------*/

/********************************** END OF FILE *******************************/
