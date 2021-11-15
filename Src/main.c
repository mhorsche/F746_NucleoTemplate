/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32F7xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
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
/* Set the following constant to pdTRUE to log using the method indicated by the
 * name of the constant, or pdFALSE to not log using the method indicated by the
 * name of the constant.  Options include to UART out (xLogToUART), to SWO 
 * (xLogToSWO), and to a UDP port (xLogToUDP).  If xLogToUDP is set to pdTRUE
 * then UDP messages are sent to the IP address configured as the echo server
 * address (see the configECHO_SERVER_ADDR0 definitions in FreeRTOSConfig.h) and
 * the port number set by configPRINT_PORT in FreeRTOSConfig.h. */
const BaseType_t xLogToUART = pdTRUE, xLogToSWO = pdTRUE, xLogToUDP = pdFALSE;

osThreadId_t tid_startup;
const osThreadAttr_t StartupTask_attributes =
    {
        .name = "start",
        .stack_size = configMINIMAL_STACK_SIZE * 2,
        .priority = (osPriority_t)osPriorityBelowNormal,
};
osThreadId_t tid_led;
const osThreadAttr_t LedTask_attributes =
    {
        .name = "led",
        .stack_size = configMINIMAL_STACK_SIZE * 2,
        .priority = (osPriority_t)osPriorityBelowNormal1,
};

/* Private function prototypes -----------------------------------------------*/
static void prvStartupTask(void *pvParameters);
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
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();

  /* This sample code shows how to use GPIO HAL API to toggle GPIOA-GPIO_PIN_5 IO
    in an infinite loop. It is possible to connect a LED between GPIOA-GPIO_PIN_5
    output and ground via a 330ohm resistor to see this external LED blink.
    Otherwise an oscilloscope can be used to see the output GPIO signal */

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); //https://www.freertos.org/RTOS-Cortex-M3-M4.html

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

  /* Initialize logging for libraries that depend on it. */
  uint32_t ulLoggingIPAddress = 0;
  // ulLoggingIPAddress = FreeRTOS_inet_addr_quick(
  //     configECHO_SERVER_ADDR0,
  //     configECHO_SERVER_ADDR1,
  //     configECHO_SERVER_ADDR2,
  //     configECHO_SERVER_ADDR3 );
  vLoggingInit(xLogToUART, xLogToSWO, xLogToUDP, ulLoggingIPAddress, configPRINT_PORT);

  /* Definition and creation of FreeRTOS Threads (Tasks) */
  tid_startup = osThreadNew(prvStartupTask, NULL, &StartupTask_attributes);
  tid_led = osThreadNew(prvLedTask, NULL, &LedTask_attributes);

  /* Init and start scheduler */
  osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {
  };
}

static void
prvStartupTask(void *pvParameters)
{
  uint32_t cnt = 0;

  for (;;)
  {
    LogInfo(("## prvStartupTask %d Loop ##\r\n", cnt));

    /* Toggle green LED and write command via SWO/SWV */
    BSP_LED_Toggle(LED_GREEN);
    osDelay(100);

    /* Increment loop counter */
    cnt++;
  }
}

static void
prvLedTask(void *pvParameters)
{
  uint32_t cnt = 0;

  for (;;)
  {
    LogInfo(("** prvLedTask %d Loop **\r\n", cnt));

    /* Toggle blue LED and write command via SWO/SWV */
    BSP_LED_Toggle(LED_BLUE);
    osDelay(5);

    /* Increment loop counter */
    cnt++;
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  /* Turn N/A on */
  BSP_LED_On(LED_RED);

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  };
}

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
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
