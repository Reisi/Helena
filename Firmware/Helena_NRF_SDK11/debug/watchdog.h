/**
  ******************************************************************************
  * @file    watchdog.h
  * @author  Thomas Reisnecker
  * @brief
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef WATCHDOG_H_INCLUDED
#define WATCHDOG_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
ret_code_t wdg_Init(void);

void wdg_Feed(void);

#endif // WATCHDOG_H_INCLUDED

/**END OF FILE*****************************************************************/
