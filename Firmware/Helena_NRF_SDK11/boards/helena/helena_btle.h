/**
  ******************************************************************************
  * @file    helena_btle.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef KD2_BTLE_H_INCLUDED
#define KD2_BTLE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "board.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
ret_code_t hln_btle_Init(bool isImuAvailable);

#endif // KD2_BTLE_H_INCLUDED

/**END OF FILE*****************************************************************/
