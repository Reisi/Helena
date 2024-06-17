/**
  ******************************************************************************
  * @file    hmi.h
  * @author  Thomas Reisnecker
  * @brief   ?
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HMI_H_INCLUDED
#define HMI_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "nrf_sdm.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    HMI_LED_ADVERTISING = 0,
    HMI_LED_CONNECTED,
    HMI_LED_UPDATING
} hmi_leds_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void hmi_Init(void);

void hmi_GetClockConfig(nrf_clock_lf_cfg_t* pConfig);

bool hmi_IsDfuButtonPressed(void);

void hmi_SetLed(hmi_leds_t led, bool state);

#endif /* HMI_H_INCLUDED */

/**END OF FILE*****************************************************************/
