/**
  ******************************************************************************
  * @file    hmi.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    15/10/26
  * @brief   Header for hmi.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef _HMI_H_
#define _HMI_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
    hmi_BUTTONNOPRESS = 0, hmi_BUTTONPRESS, hmi_BUTTONSHORT, hmi_BUTTONLONG, hmi_BUTTONULTRALONG
} hmi_ButtonEnum;

typedef enum {
    hmi_LEDRED = 0, hmi_LEDBLUE
} hmi_LedEnum;

typedef enum {
    hmi_LEDOFF = 0, hmi_LEDON, hmi_LEDTOGGLE
} hmi_LedStateEnum;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void hmi_Init(void);
hmi_ButtonEnum hmi_Debounce(void);
void hmi_SetLed(hmi_LedEnum, hmi_LedStateEnum);

#endif /*_HMI_H_*/

/**END OF FILE*****************************************************************/
