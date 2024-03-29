/**
  ******************************************************************************
  * @file    ?.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    ?
  * @brief   ?
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FASTMATH_H
#define FASTMATH_H

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
q15_t arm_asin_q15(q15_t y);
q15_t arm_acos_q15(q15_t y);
q15_t arm_atan2_q15(int16_t y, int16_t x);

#endif /* FASTMATH_H */

/**END OF FILE*****************************************************************/
