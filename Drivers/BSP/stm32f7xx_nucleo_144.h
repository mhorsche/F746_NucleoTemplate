/**
 * @file stm32f7xx_nucleo_144.h
 * @author horsche (horsche@li.plus)
 * @brief
 * @version
 * @date 2023-08-11
 *
 * @copyright Copyright (c) 2023
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F7XX_NUCLEO_144_H
#define __STM32F7XX_NUCLEO_144_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

#define RTOS_TIM_MODULE
#define RTOS_RNG_MODULE

#define GPIO_MODULE
#define STLINK_MODULE
#define LAN8742A_MODULE

// #define IWDG_MODULE

/**
 * @brief MCU peripheral interrupt priorities.
 *
 * @note The highest interrupt priority that can be used by any interrupt
 *    service routine that makes calls to interrupt safe FreeRTOS API functions.
 *    DO NOT CALL INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT
 *    HAS A HIGHER PRIORITY THAN THIS!
 *    Higher priorities are lower numeric values!
 */
#define RTOS_IRQPriority (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 7)
#define ETH_IRQPriority (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 8)

#define STLK_IRQPriority (configLIBRARY_LOWEST_INTERRUPT_PRIORITY)

/******************************************************************************/
/*                          FreeRTOS Timer module                             */
/******************************************************************************/
#ifdef RTOS_TIM_MODULE

#ifndef HAL_TIM_MODULE_ENABLED
#error "Enable HAL_TIM_MODULE_ENABLED for RTOS TIM driver!"
#else /* !HAL_TIM_MODULE_ENABLED */

#define RTOS_TIM_PRESCALER (108ul)   /* Timer running at 108MHz / 108 = 1MHz */
#define RTOS_TIM_PERIOD (10000000ul) /* Every 10 sec */

#define RTOS_TIM TIM2 /* Use a 32bit timer */
#define RTOS_TIM_CHANNEL TIM_CHANNEL_1
#define RTOS_TIM_ACTIVE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_1
#define RTOS_TIM_CLK_ENABLE() __HAL_RCC_TIM2_CLK_ENABLE()

#define RTOS_TIM_IRQn TIM2_IRQn
#define RTOS_TIM_IRQHandler TIM2_IRQHandler
#define RTOS_TIM_FORCE_RESET() __HAL_RCC_TIM2_FORCE_RESET()
#define RTOS_TIM_RELEASE_RESET() __HAL_RCC_TIM2_RELEASE_RESET()

#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* RTOS_TIM_MODULE */

/******************************************************************************/
/*              FreeRTOS RNG (random number generator) module                 */
/******************************************************************************/
#ifdef RTOS_RNG_MODULE
#ifndef HAL_RNG_MODULE_ENABLED
#error "Enable HAL_RNG_MODULE_ENABLED for RTOS RNG driver!"
#else /* !HAL_RNG_MODULE_ENABLED */

#define RTOS_RNG RNG

#endif /* HAL_RNG_MODULE_ENABLED */

#endif /* RTOS_RNG_MODULE */

/******************************************************************************/
/*                          IO (LED, PB) OPERATIONS                           */
/******************************************************************************/
#ifdef GPIO_MODULE

#ifndef HAL_GPIO_MODULE_ENABLED
#error "Enable HAL_GPIO_MODULE_ENABLED for GPIO driver!"
#else /* !HAL_GPIO_MODULE_ENABLED */

/**
 * @brief  LED GPIO Configuration
 *          PB0 ------> LED1 (green)
 *          PB7 ------> LED2 (blue)
 *         PB17 ------> LED3 (red)
 *
 *         PC11 <------ Push Button
 */

/* LED1 = Green */
#define LED1_GPIO_PIN GPIO_PIN_0
#define LED1_GPIO_PORT GPIOB
#define LED1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()

/* LED2 = Blue */
#define LED2_GPIO_PIN GPIO_PIN_7
#define LED2_GPIO_PORT GPIOB
#define LED2_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()

/* LED3 = Red */
#define LED3_GPIO_PIN GPIO_PIN_14
#define LED3_GPIO_PORT GPIOB
#define LED3_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()

/* Push Button */
#define USER_BUTTON_GPIO_PIN GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()

#define USER_BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define USER_BUTTON_EXTI_LINE USER_BUTTON_GPIO_PIN

#endif /* HAL_GPIO_MODULE_ENABLED */

#endif /* GPIO_MODULE */

/******************************************************************************/
/*                        Independent Watchdog (IWDG)                         */
/******************************************************************************/
#ifdef IWDG_MODULE
#ifndef HAL_IWDG_MODULE_ENABLED
#error "Enable HAL_IWDG_MODULE_ENABLED for IWDG driver!"
#else /* !HAL_IWDG_MODULE_ENABLED */

#endif /* HAL_IWDG_MODULE_ENABLED */

#endif /* IWDG_MODULE */

/******************************************************************************/
/*                              ST-LINK module                                */
/* Virtual-COM port for debugging and status messages                         */
/******************************************************************************/
#ifdef STLINK_MODULE

#ifndef HAL_UART_MODULE_ENABLED
#error "Enable UART_MODULE_ENABLED for ST-LINK driver!"
#else /* !HAL_UART_MODULE_ENABLED */

/**
 * @brief  USART3 GPIO Configuration
 *    PD8 ------> USART3_TX
 *    PD9 <------ USART3_RX
 */

/* Definition for USART Pins */
#define STLK_UART_TX_PIN GPIO_PIN_9
#define STLK_UART_TX_GPIO_PORT GPIOD
#define STLK_UART_TX_AF GPIO_AF7_USART3
#define STLK_UART_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

#define STLK_UART_RX_PIN GPIO_PIN_8
#define STLK_UART_RX_GPIO_PORT GPIOD
#define STLK_UART_RX_AF GPIO_AF7_USART3
#define STLK_UART_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()

/* Definition for USART clock resources */
#define STLK_UART USART3
#define STLK_UART_BAUDRATE 115200
#define STLK_UART_CLK_ENABLE() __HAL_RCC_USART3_CLK_ENABLE()
#define STLK_UART_RCC_PERIPHCLK RCC_PERIPHCLK_USART3
#define STLK_UART_RCC_CLKSOURCE RCC_USART3CLKSOURCE_PCLK1

#define STLK_UART_FORCE_RESET() __USART3_FORCE_RESET()
#define STLK_UART_RELEASE_RESET() __USART3_RELEASE_RESET()

  /* Definition for USART's DMA */
  // #define STLK_UART_RX_DMA_STREAM DMA2_Stream5
  // #define STLK_UART_RX_DMA_CHANNEL DMA_CHANNEL_4 /* USART1_RX */
  // #define STLK_UART_RX_DMA_CLK_ENABLE() __HAL_RCC_DMA2_CLK_ENABLE()
  // #define STLK_UART_RX_DMA_FORCE_RESET() __HAL_RCC_DMA2_FORCE_RESET()
  // #define STLK_UART_RX_DMA_RELEASE_RESET() __HAL_RCC_DMA2_RELEASE_RESET()

// #define STLK_UART_TX_DMA_STREAM DMA1_Stream4
// #define STLK_UART_TX_DMA_CHANNEL DMA_CHANNEL_7 /* USART3_TX */
// #define STLK_UART_TX_DMA_CLK_ENABLE() __HAL_RCC_DMA1_CLK_ENABLE()
// #define STLK_UART_TX_DMA_FORCE_RESET() __HAL_RCC_DMA1_FORCE_RESET()
// #define STLK_UART_TX_DMA_RELEASE_RESET() __HAL_RCC_DMA1_RELEASE_RESET()

/* ST-Link USART is running in interrupt mode */
#ifdef STLK_IRQPriority

/* Definition for UART's NVIC */
#define STLK_UART_IRQn USART3_IRQn
#define STLK_UART_IRQHandler USART3_IRQHandler

#ifdef STLK_UART_IRQHandler
  void STLK_UART_IRQHandler(void);
#endif /* STLK_UART_IRQHandler */

  // #define STLK_UART_RX_DMA_IRQn DMA1_Stream1_IRQn
  // #define STLK_UART_RX_DMA_IRQHandler DMA1_Stream1_IRQHandler

#ifdef STLK_UART_RX_DMA_IRQHandler
  void STLK_UART_RX_DMA_IRQHandler(void);
#endif /* STLK_UART_RX_DMA_IRQHandler */

  // #define STLK_UART_TX_DMA_IRQn DMA1_Stream4_IRQn
  // #define STLK_UART_TX_DMA_IRQHandler DMA1_Stream4_IRQHandler

#ifdef STLK_UART_TX_DMA_IRQHandler
  void STLK_UART_TX_DMA_IRQHandler(void);
#endif /* STLK_UART_TX_DMA_IRQHandler */

#endif /* STLK_IRQPriority */

  void STLINK_UART_Init(UART_HandleTypeDef *hUart,
                        DMA_HandleTypeDef *hDmaRx,
                        DMA_HandleTypeDef *hDmaTx);
  void STLINK_UART_DeInit(UART_HandleTypeDef *hUart);

  void STLINK_UART_RegisterCallback(UART_HandleTypeDef *hUart,
                                    void (*pCallback)(UART_HandleTypeDef *_hUart));

  HAL_UART_StateTypeDef STLINK_UART_GetState(UART_HandleTypeDef *hUart);
  HAL_StatusTypeDef STLINK_UART_SendData(UART_HandleTypeDef *hUart, uint8_t *pBuffer, size_t BufferSize);
  HAL_StatusTypeDef STLINK_UART_SendData_DMA(UART_HandleTypeDef *hUart, uint8_t *pBuffer, size_t BufferSize);

#endif /* HAL_UART_MODULE_ENABLED */

#endif /* STLINK_MODULE */

/******************************************************************************/
/*                               LAN8742 module                               */
/* 0/100 Base-T/TX Ethernet Transceiver with Cable Diagnostics                */
/* https://www.microchip.com/en-us/product/LAN8742                            */
/******************************************************************************/
#ifdef LAN8742A_MODULE

#ifndef HAL_ETH_MODULE_ENABLED
#error "Enable ETH_MODULE_ENABLED for Ethernet driver!"
#else /* !HAL_ETH_MODULE_ENABLED */

  /**
   * @brief  RMII LAN8742A GPIO Configuration
   *    PA1 <------- RMII_REF_CLK
   *    PA2 <------> RMII_MDIO
   *    PC1 -------> RMII_MDC
   *    PA7 <------- RMII_MII_CRS_DV
   *    PC4 <------- RMII_MII_RXD0
   *    PC5 <------- RMII_MII_RXD1
   *    PG2 <------- RMII_MII_RXER (fixed to ground)
   *    PG11 ------> RMII_MII_TX_EN
   *    PG13 ------> RMII_MII_TXD0
   *    PB13 ------> RMII_MII_TXD1
   */

#define RMII_REF_CLK_PIN GPIO_PIN_1
#define RMII_REF_CLK_GPIO_PORT GPIOA
#define RMII_REF_CLK_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define RMII_REF_CLK_GPIO_CLK_DISABLE() __HAL_RCC_GPIOD_CLK_DISABLE()

#define RMII_MDIO_PIN GPIO_PIN_2
#define RMII_MDIO_GPIO_PORT GPIOA
#define RMII_MDIO_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define RMII_MDIO_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()

#define RMII_MDC_PIN GPIO_PIN_1
#define RMII_MDC_GPIO_PORT GPIOC
#define RMII_MDC_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define RMII_MDC_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()

#define RMII_CRS_DV_PIN GPIO_PIN_7
#define RMII_CRS_DV_GPIO_PORT GPIOA
#define RMII_CRS_DV_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define RMII_CRS_DV_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()

#define RMII_RXD0_PIN GPIO_PIN_4
#define RMII_RXD0_GPIO_PORT GPIOC
#define RMII_RXD0_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define RMII_RXD0_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()

#define RMII_RXD1_PIN GPIO_PIN_5
#define RMII_RXD1_GPIO_PORT GPIOC
#define RMII_RXD1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define RMII_RXD1_GPIO_CLK_DISABLE() __HAL_RCC_GPIOC_CLK_DISABLE()

#define RMII_MII_RXER_PIN GPIO_PIN_2
#define RMII_MII_RXER_GPIO_PORT GPIOG
#define RMII_MII_RXER_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE()
#define RMII_MII_RXER_GPIO_CLK_DISABLE() __HAL_RCC_GPIOG_CLK_DISABLE()

#define RMII_TX_EN_PIN GPIO_PIN_11
#define RMII_TX_EN_GPIO_PORT GPIOG
#define RMII_TX_EN_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE()
#define RMII_TX_EN_GPIO_CLK_DISABLE() __HAL_RCC_GPIOG_CLK_DISABLE()

#define RMII_TXD0_PIN GPIO_PIN_13
#define RMII_TXD0_GPIO_PORT GPIOG
#define RMII_TXD0_GPIO_CLK_ENABLE() __HAL_RCC_GPIOG_CLK_ENABLE()
#define RMII_TXD0_GPIO_CLK_DISABLE() __HAL_RCC_GPIOG_CLK_DISABLE()

#define RMII_TXD1_PIN GPIO_PIN_13
#define RMII_TXD1_GPIO_PORT GPIOB
#define RMII_TXD1_GPIO_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define RMII_TXD1_GPIO_CLK_DISABLE() __HAL_RCC_GPIOB_CLK_DISABLE()

#endif /* HAL_ETH_MODULE_ENABLED */

#endif /* LAN8742A_MODULE */

#endif /* __STM32F7XX_NUCLEO_144_H */

/********************************** END OF FILE *******************************/
