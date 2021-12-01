/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

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
#include "logging.h"

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static UBaseType_t ulNextRand;
RNG_HandleTypeDef xRNG;

osThreadId_t xLedTaskHandle;
const osThreadAttr_t xLedTaskAttributes =
    {
        .name = "led",
        .stack_size = (configMINIMAL_STACK_SIZE * 7), /* Need ~500 byte due to printf (logging) */
        .priority = (osPriority_t)osPriorityBelowNormal1,
};

/* The MAC address array is not declared const as the MAC address will
normally be read from an EEPROM and not hard coded (in real deployed
applications).*/
static uint8_t ucMACAddress[6] = {ipconfigMAC_ADDR0, ipconfigMAC_ADDR1, ipconfigMAC_ADDR2, ipconfigMAC_ADDR3, ipconfigMAC_ADDR4, ipconfigMAC_ADDR5};

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
const BaseType_t xLogToUART = pdTRUE, xLogToSWO = pdFALSE, xLogToUDP = pdTRUE;

/* Private function prototypes -----------------------------------------------*/
extern void vShowTaskTable(BaseType_t xDoClear); /* @see freertos.c */
extern void vHeapInit(void);                     /* @see freertos.c */

static void prvLedTask(void *pvParameters);

static void SystemClock_Config(void);
static void DebugMode_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  // /* Enable I-Cache */
  // SCB_EnableICache();

  // /* Enable D-Cache */
  // SCB_EnableDCache();

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
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // https://www.freertos.org/RTOS-Cortex-M3-M4.html

  /* Configure the system clock to 216 MHz */
  SystemClock_Config();

  /* Configure debug mode to stop all timers */
  DebugMode_Config();

  /* Enable GPIO Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure IO in output push-pull mode to drive external LEDs */
  BSP_LED_Init(LED_RED);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);

  vHeapInit();

  /* Initialize logging for libraries that depend on it. */
  uint32_t ulLoggingIPAddress = 0;
  ulLoggingIPAddress = FreeRTOS_inet_addr_quick(ipconfigECHO_SERVER_ADDR0, ipconfigECHO_SERVER_ADDR1, ipconfigECHO_SERVER_ADDR2, ipconfigECHO_SERVER_ADDR3);
  vLoggingInit(xLogToUART, xLogToSWO, xLogToUDP, ulLoggingIPAddress, ipconfigPRINT_PORT);
  FreeRTOS_printf(("\r\n\r\n\r\n"));

  /* Initialise the RTOS's TCP/IP stack.  The tasks that use the network
   * are created in the vApplicationIPNetworkEventHook() hook function
   * below.  The hook function is called when the network connects. */
  xRNG.Instance = RNG;
  if (HAL_RNG_Init(&xRNG) != HAL_OK)
  {
    Error_Handler();
  }
  FreeRTOS_IPInit(ucIPAddress, ucNetMask, ucGatewayAddress, ucDNSServerAddress, ucMACAddress);

  /* Definition and creation of FreeRTOS Threads (Tasks) */
  xLedTaskHandle = osThreadNew(prvLedTask, NULL, &xLedTaskAttributes);

  /* Init and start scheduler */
  osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {
  };
}
/*-----------------------------------------------------------*/

/**
 * @brief  Use by the pseudo random number generator.
 */
UBaseType_t uxRand(void)
{
  const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

  /* Utility function to generate a pseudo random number. */

  ulNextRand = (ulMultiplier * ulNextRand) + ulIncrement;
  return ((int)(ulNextRand >> 16UL) & 0x7fffUL);
}
/*-----------------------------------------------------------*/

static void prvLedTask(void *pvParameters)
{
  uint32_t cnt = 0;

  for (;;)
  {
    /* Show task manager every 10 iterations */
    if (cnt++ % 10 == 9)
    {
      // vShowTaskTable(pdFALSE);
    }

    /* Toggle blue LED and write command via SWO/SWV */
    BSP_LED_Toggle(LED_GREEN);
    osDelay(1000);
  }
}
/*-----------------------------------------------------------*/

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  /* Turn on/off LEDs to indicate error handler */
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_On(LED_RED);

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  };
}
/*-----------------------------------------------------------*/

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow : 
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 216000000
 *            HCLK(Hz)                       = 216000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 432
 *            PLL_P                          = 2
 *            PLL_Q                          = 9
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 7
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the OverDrive to reach the 216 MHz Frequency */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  // RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
  // PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  // PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  // if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK)
  //   {
  //     Error_Handler ();
  //   }
}
/*-----------------------------------------------------------*/

/**
 * @brief  Configure Debug Mode to stop all timers.
 * @param  None
 * @retval None
 */
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

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
/*-----------------------------------------------------------*/
#endif

/**
  * @}
  */

/**
  * @}
  */
/********************************** END OF FILE *******************************/
