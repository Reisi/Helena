/**
  ******************************************************************************
  * @file    template_hmi.h
  * @author  Thomas Reisnecker
  * @brief   Header for template hmi module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEMPLATE_HMI_H_INCLUDED
#define TEMPLATE_HMI_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hmi module
 *
 * @return NRF_SUCCESS or a propagated error from but_DebounceInit
 */
ret_code_t template_Hmi_Init();

/** @brief The execute function of the hmi module
 */
void template_Hmi_Execute(void);

#endif // TEMPLATE_HMI_H_INCLUDED

/**END OF FILE*****************************************************************/

