/**
 * @file stm32f7xx_common.h
 * @author horsche (horsche@li.plus)
 * @brief
 * @version
 * @date 2023-08-11
 *
 * @copyright Copyright (c) 2023
 */

#ifndef __STM32F7XX_COMMON_H
#define __STM32F7XX_COMMON_H

#ifndef USE_STM32F7XX_NUCLEO_144
#define USE_STM32F7XX_NUCLEO_144
#endif

/* Includes ------------------------------------------------------------------*/
#ifndef FW_VERSION
#define FW_VERSION "v0.0.0"
#endif

#if defined(USE_STM32F7XX_NUCLEO_144)

#include "stm32f7xx_nucleo_144.h"

#else
#error "Please select the target board used in your application first (USE_F746_NUCLEO, ...)"
#endif

/* Exported Structs ----------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Exported Macros -----------------------------------------------------------*/

/**
 * @brief Read unique chip ID, see Reference Manual page 1646
 */
#define stm32f7xxGET_UNIQUE_BYTE(x) ((x >= 0 && x < 12) ? (*(uint8_t *)(0x1FF0F420 + (x))) : 0)
/**
 * @brief Get STM32Fxxx device signature
 * @note  Defined as macro to get maximal response time
 *          https://github.com/MaJerle/stm32fxxx-hal-libraries/
 * @param  None
 *
 * @retval Device signature, bits 11:0 are valid, 15:12 are always 0
 *          - 0x0413: STM32F405xx/07xx and STM32F415xx/17xx)
 *          - 0x0419: STM32F42xxx and STM32F43xxx
 *          - 0x0423: STM32F401xB/C
 *          - 0x0433: STM32F401xD/E
 *          - 0x0431: STM32F411xC/E
 *          - 0x0421: STM32F446xx
 *          - 0x0449: STM32F7x6xx
 *          - 0x0444: STM32F03xxx
 *          - 0x0445: STM32F04xxx
 *          - 0x0440: STM32F05xxx
 *          - 0x0448: STM32F07xxx
 *          - 0x0442: STM32F09xxx
 */
#define stm32f7xxGET_SIGNATURE() (DBGMCU->IDCODE & 0x00000FFF)
/**
 * @brief  Gets STM32Fxxx device revision
 * @param  None
 * @retval Device revision value
 *          - 0x1000: Revision A
 *          - 0x1001: Revision Z
 *          - 0x1003: Revision Y
 *          - 0x1007: Revision 1
 *          - 0x2001: Revision 3
 */
#define stm32f7xxGET_REVISION() ((DBGMCU->IDCODE >> 16) & 0x0000FFFF)

/**
 * @brief  Gets STM32Fxxx device package
 * @note   This is not available for all packages, somewhere HardFault may occur
 * @param  None
 * @retval Device revision value
 *          - 0b01xx: LQFP208 and TFBGA216 package
 *          - 0b0011: LQFP176 and UFBGA176 package
 *          - 0b0010: WLCSP143 and LQFP144 package
 *          - 0b0001: LQFP100 package
 */
#define stm32f7xxGET_PACKAGE() (((*(__IO uint16_t *)(PACKAGE_BASE)) & 0x0700) >> 8)

/**
 * @brief  Gets STM32Fxxx device flash size in kilo bytes
 * @param  None
 * @retval Flash size in kilo bytes
 *            - Device has stored value of flash size in kB
 */
#define stm32f7xxGET_FLASH_SIZE() (*(__IO uint16_t *)(FLASHSIZE_BASE))

/******************************************************************************/
/*                           Global Error Handler                             */
/******************************************************************************/

/**
 * @brief This function is executed in case of error occurrence.
 *    Normal function code is "vAssertCalled(__LINE__, __FILE__)" or
 *    "vAssertCalled(__LINE__, __FUNCTION__)".
 *
 * @param[in] line: error line source number
 * @param[in] file: pointer to the source file or function name
 */
void vAssertCalled(uint32_t ulLine, const char *pcFile);

/******************************************************************************/
/*                Cortex-M7 Processor Exceptions Handlers                     */
/******************************************************************************/

/**
 * @brief This function handles NMI exception.
 */
void NMI_Handler(void);

/**
 * @brief This function handles Hard Fault exception.
 */
void HardFault_Handler(void);

/**
 * @brief This function handles Memory Manage exception.
 */
void MemManage_Handler(void);

/**
 * @brief This function handles Bus Fault exception.
 */
void BusFault_Handler(void);

/**
 * @brief This function handles Usage Fault exception.
 */
void UsageFault_Handler(void);

/**
 * @brief This function handles Debug Monitor exception.
 */
void DebugMon_Handler(void);

/**
 * @brief This function handles SysTick Handler.
 */
void SysTick_Handler(void);

#ifndef vPortSVCHandler
/**
 * @brief This function handles SVCall exception.
 */
void SVC_Handler(void);
#endif

#ifndef xPortPendSVHandler
/**
 * @brief This function handles PendSVC exception.
 */
void PendSV_Handler(void);
#endif /* xPortPendSVHandler */

/******************************************************************************/
/*                          IO (LED, PB) OPERATIONS                           */
/******************************************************************************/
#ifdef GPIO_MODULE

/**
 * @brief Configures LED/Push Button GPIOs.
 */
void LED_GPIO_Init(void);

/**
 * @brief DeInit LED GPIOs.
 * @note LED DeInit does not disable the GPIO clock nor disable the Mfx
 */
void LED_GPIO_DeInit(void);

/**
 * @brief LED shortcuts using low level functions
 */
#ifdef LED1_GPIO_PIN
#define LED_GREEN_On() LL_GPIO_SetOutputPin(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED_GREEN_Off() LL_GPIO_ResetOutputPin(LED1_GPIO_PORT, LED1_GPIO_PIN)
#define LED_GREEN_Toggle() LL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN)
#endif /* LED1_GPIO_PIN */

#ifdef LED2_GPIO_PIN
#define LED_BLUE_On() LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED_BLUE_Off() LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_GPIO_PIN)
#define LED_BLUE_Toggle() LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN)
#endif /* LED2_GPIO_PIN */

#ifdef LED3_GPIO_PIN
#define LED_RED_On() LL_GPIO_SetOutputPin(LED3_GPIO_PORT, LED3_GPIO_PIN)
#define LED_RED_Off() LL_GPIO_ResetOutputPin(LED3_GPIO_PORT, LED3_GPIO_PIN)
#define LED_RED_Toggle() LL_GPIO_TogglePin(LED3_GPIO_PORT, LED3_GPIO_PIN)
#endif /* LED3_GPIO_PIN */

/**
 * @brief PushButton shortcuts using low level functions
 */
#ifdef USER_BUTTON_GPIO_PIN
#define USER_BUTTON_GetState() LL_GPIO_IsInputPinSet(USER_BUTTON_GPIO_PORT, USER_BUTTON_GPIO_PIN)
#endif /* LED3_GPIO_PIN */

#endif /* GPIO_MODULE */
/******************************************************************************/
/*                          FreeRTOS Timer module                             */
/******************************************************************************/
#ifdef RTOS_TIM_MODULE

/**
 * @brief Initializes TIM peripherals used by the FreeRTOS driver. Register TIM
 *    period elapsed callback function and start TIM in interrupt mode.
 */
void RTOS_TIM_Init(void);
void RTOS_TIM_DeInit(void);

uint64_t RTOS_TIM_GetValue(void);

#endif /* RTOS_TIM_MODULE */

/******************************************************************************/
/*              FreeRTOS RNG (random number generator) module                 */
/******************************************************************************/
#ifdef RTOS_RNG_MODULE

/**
 * @brief Initializes the RNG (Random Number Generator) peripheral and creates
 *    the associated handle.
 */
void RTOS_RNG_Init(void);

/**
 * @brief Initializes the RNG peripheral and creates the associated handle.
 */
void RTOS_RNG_DeInit(void);

/**
 * @brief Generates a 32-bit random number in interrupt mode.
 *
 * @param[out] pulValue: pointer to value which should be set.
 * @retval HAL status
 */
HAL_StatusTypeDef RTOS_RNG_GenerateRandomNumber(uint32_t *pulValue);

#endif /* RTOS_RNG_MODULE */

/******************************************************************************/
/*                        Independent Watchdog (IWDG)                         */
/******************************************************************************/
#ifdef IWDG_MODULE

void MCU_IWDG_Init(void);
void MCU_IWDG_DeInit(void);

void MCU_IWDG_Refresh(void);

#endif /* IWDG_MODULE */

#endif /* __STM32F7XX_COMMON_H */

/********************************** END OF FILE *******************************/
