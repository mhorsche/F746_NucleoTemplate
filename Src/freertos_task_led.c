/**
 * @file freertos_task_led.c
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
#include "freertos_task_led.h"

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include "task.h"

/* Private define ------------------------------------------------------------*/
/**
 * @brief Task name listed in task manager.
 */
#define ledTASK_NAME "LED"

/**
 * @brief Stack size needed for prvTaskLed(), a bit of a guess.
 *    Need ~500 byte due to printf (logging)
 */
#define ledTASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 7)

/**
 * @brief The priority of prvTaskLed().
 */
#define ledTASK_PRIORITY 17

/* Variables -----------------------------------------------------------------*/
TaskHandle_t xLedTaskHandle;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief The task used to blink LED and show task manager.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation.
 *    Not used in this example.
 */
static void prvTaskLed(void *pvParameters);

/******************************************************************************/
/*                          Public/Exported Functions                         */
/******************************************************************************/

void vLEDTaskInstall(void)
{
  if (xLedTaskHandle == NULL)
  {
    /* Configure LED1, LED2, LED3 and PushButton */
    LED_GPIO_Init();

    /* Start LED task toggling green LED every second. */
    xTaskCreate(prvTaskLed,
                ledTASK_NAME,
                ledTASK_STACK_SIZE,
                NULL,
                ledTASK_PRIORITY,
                &xLedTaskHandle);
  }
}
/*-----------------------------------------------------------*/

/******************************************************************************/
/*                         Private/Static Functions                           */
/******************************************************************************/

static void prvTaskLed(void *pvParameters)
{
  // static uint64_t runtime;
  uint32_t cnt = 0;

  /* Remove compiler warnings about unused parameters. */
  (void)pvParameters;

  for (;;)
  {
    /* Toggle green LED */
    LED_GREEN_Toggle();

    /* Show task manager every 5 iterations */
    if ((cnt++ % 5) == 0)
    {
      vShowTaskTable(pdFALSE);
    }

    /* Delay task for 1000 msec. */
    vTaskDelay(1000);
  }
}
/*-----------------------------------------------------------*/

/********************************** END OF FILE *******************************/
