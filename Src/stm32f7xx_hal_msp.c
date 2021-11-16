/**
  ******************************************************************************
  * @file    UART/UART_Printf/Src/stm32f7xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* FreeRTOS includes. */
#include <FreeRTOS.h>

/* Utilities includes. */
#include <logging.h>

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @defgroup HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
 * @brief  Initializes the Global MSP.
 * @param  None
 * @retval None
 */
void HAL_MspInit(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

/**
 * @brief  DeInitializes the Global MSP.
 * @param  None  
 * @retval None
 */
void HAL_MspDeInit(void)
{
}

/**
 * @brief  TIM MSP Initialization
 *         This function configures the hardware resources used in this example:
 *            - Peripheral's clock enable
 *            - Peripheral's GPIO Configuration
 * @param  htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
  if (htim_base->Instance == TIM13)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();

    /* TIM13 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  }
}

/**
 * @brief  TIM MSP Initialization
 *         This function freeze the hardware resources used in this example
 * @param  htim: TIM handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
  if (htim_base->Instance == TIM13)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM13_CLK_DISABLE();

    /* TIM13 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  }
}

/**
 * @brief  UART MSP Initialization
 *         This function configures the hardware resources used in this example:
 *            - Peripheral's clock enable
 *            - Peripheral's GPIO Configuration
 * @param  huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit = {0};

  if (huart->Instance == STLK_USART)
  {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    STLK_TX_GPIO_CLK_ENABLE();
    STLK_RX_GPIO_CLK_ENABLE();

    /* Select SysClk as source of USART1 clocks */
    RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    RCC_PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

    /* Enable USARTx clock */
    STLK_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin = STLK_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = STLK_TX_AF;

    HAL_GPIO_Init(STLK_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = STLK_RX_PIN;
    GPIO_InitStruct.Alternate = STLK_RX_AF;

    HAL_GPIO_Init(STLK_RX_GPIO_PORT, &GPIO_InitStruct);
  }
}

/**
 * @brief  UART MSP De-Initialization
 *         This function frees the hardware resources used in this example:
 *           - Disable the Peripheral's clock
 *           - Revert GPIO and NVIC configuration to their default state
 * @param  huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == STLK_USART)
  {
    /*##-1- Reset peripherals ##################################################*/
    STLK_FORCE_RESET();
    STLK_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks #################################*/
    /* Configure UART Tx as alternate function  */
    HAL_GPIO_DeInit(STLK_TX_GPIO_PORT, STLK_TX_PIN);
    /* Configure UART Rx as alternate function  */
    HAL_GPIO_DeInit(STLK_RX_GPIO_PORT, STLK_RX_PIN);
  }
}

/**
 * @brief  Initializes the RNG MSP.
 * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
 *                the configuration information for RNG.
 * @retval None
 */
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
  if (hrng->Instance == RNG)
  {
    /* RNG clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
  }
}

/**
 * @brief  DeInitializes the RNG MSP.
 * @param  hrng pointer to a RNG_HandleTypeDef structure that contains
 *                the configuration information for RNG.
 * @retval None
 */
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng)
{

  if (hrng->Instance == RNG)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RNG_CLK_DISABLE();
  }
}

/**
 * @brief  Initializes the ETH MSP.
 * @param  heth pointer to a ETH_HandleTypeDef structure that contains
 *         the configuration information for ETHERNET module
 * @retval None
 */
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (heth->Instance == ETH)
  {
    /* ETH clock enable */
    __HAL_RCC_ETH_CLK_ENABLE();

    RMII_REF_CLK_GPIO_CLK_ENABLE();
    RMII_MDIO_GPIO_CLK_ENABLE();
    RMII_MDC_GPIO_CLK_ENABLE();
    RMII_CRS_DV_GPIO_CLK_ENABLE();
    RMII_RXD0_GPIO_CLK_ENABLE();
    RMII_RXD1_GPIO_CLK_ENABLE();
    RMII_MII_RXER_GPIO_CLK_ENABLE();
    RMII_TX_EN_GPIO_CLK_ENABLE();
    RMII_TXD0_GPIO_CLK_ENABLE();
    RMII_TXD1_GPIO_CLK_ENABLE();

    /* ETH GPIO Configuration */
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;

    GPIO_InitStruct.Pin = RMII_REF_CLK_PIN;
    HAL_GPIO_Init(RMII_REF_CLK_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_MDIO_PIN;
    HAL_GPIO_Init(RMII_MDIO_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_MDC_PIN;
    HAL_GPIO_Init(RMII_MDC_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_CRS_DV_PIN;
    HAL_GPIO_Init(RMII_CRS_DV_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_RXD0_PIN;
    HAL_GPIO_Init(RMII_RXD0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_RXD1_PIN;
    HAL_GPIO_Init(RMII_RXD1_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_MII_RXER_PIN;
    HAL_GPIO_Init(RMII_MII_RXER_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_TX_EN_PIN;
    HAL_GPIO_Init(RMII_TX_EN_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_TXD0_PIN;
    HAL_GPIO_Init(RMII_TXD0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RMII_TXD1_PIN;
    HAL_GPIO_Init(RMII_TXD1_GPIO_PORT, &GPIO_InitStruct);

    /* Enable the Ethernet global Interrupt */
    HAL_NVIC_SetPriority(ETH_IRQn, 5, 0); // @todo check prio 0x7?
    HAL_NVIC_EnableIRQ(ETH_IRQn);

    // HAL_NVIC_SetPriority(ETH_WKUP_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(ETH_WKUP_IRQn);
  }
}

/**
 * @brief  DeInitializes ETH MSP.
 * @param  heth pointer to a ETH_HandleTypeDef structure that contains
 *         the configuration information for ETHERNET module
 * @retval None
 */
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth)
{
  if (heth->Instance == ETH)
  {
    /* Peripheral clock disable */
    __HAL_RCC_ETH_CLK_DISABLE();

    /* ETH GPIO Configuration */
    HAL_GPIO_DeInit(RMII_REF_CLK_GPIO_PORT, RMII_REF_CLK_PIN);
    HAL_GPIO_DeInit(RMII_MDIO_GPIO_PORT, RMII_MDIO_PIN);
    HAL_GPIO_DeInit(RMII_MDC_GPIO_PORT, RMII_MDC_PIN);
    HAL_GPIO_DeInit(RMII_CRS_DV_GPIO_PORT, RMII_CRS_DV_PIN);
    HAL_GPIO_DeInit(RMII_RXD0_GPIO_PORT, RMII_RXD0_PIN);
    HAL_GPIO_DeInit(RMII_RXD1_GPIO_PORT, RMII_RXD1_PIN);
    HAL_GPIO_DeInit(RMII_MII_RXER_GPIO_PORT, RMII_MII_RXER_PIN);
    HAL_GPIO_DeInit(RMII_TX_EN_GPIO_PORT, RMII_TX_EN_PIN);
    HAL_GPIO_DeInit(RMII_TXD0_GPIO_PORT, RMII_TXD0_PIN);
    HAL_GPIO_DeInit(RMII_TXD1_GPIO_PORT, RMII_TXD1_PIN);

    /* ETH interrupt Deinit */
    HAL_NVIC_DisableIRQ(ETH_IRQn);
    // HAL_NVIC_DisableIRQ(ETH_WKUP_IRQn);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
