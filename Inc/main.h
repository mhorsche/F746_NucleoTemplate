/**
 * @file main.h
 * @author horsche (horsche@li.plus)
 * @brief 
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_nucleo_144.h"

/* Exported types ------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern RNG_HandleTypeDef xRNG;

/* Exported macro ------------------------------------------------------------*/

/* Read unique chip ID, see Reference Manual page 1646 */
#define GET_UNIQUE_BYTE(x) ((x >= 0 && x < 12) ? (*(uint8_t *)(0x1FF0F420 + (x))) : 0)

/* Exported functions --------------------------------------------------------*/
void Error_Handler(void);
#ifdef DEBUG
int _write(int file, char *ptr, int len);
#endif

#endif /* __MAIN_H */

/********************************** END OF FILE *******************************/