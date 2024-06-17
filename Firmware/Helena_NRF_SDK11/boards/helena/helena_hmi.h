/**
  ******************************************************************************
  * @file    helena_hmi.h
  * @author  Thomas Reisnecker
  * @brief   Header for helena hmi module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HLN_HMI_H_INCLUDED
#define HLN_HMI_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "helena_board.h"

/* Exported constants --------------------------------------------------------*/
#define BUTTON_NOT_USED         0xFFFFFFFF

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/// TODO: temporary until board detect module implemented
typedef struct
{
    uint32_t button;
    uint32_t com;
    uint32_t ledRed;
    uint32_t ledBlue;
} hln_brdPins_t;

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hmi module
 *
 * @return NRF_SUCCESS or a propagated error from but_DebounceInit
 */
ret_code_t hln_hmi_Init(hln_brd_t const* pPins, bool secondaryButton);

/** @brief The execute function of the hmi module
 */
void hln_hmi_Execute(void);

#endif // HLN_HMI_H_INCLUDED

/**END OF FILE*****************************************************************/

