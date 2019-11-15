/**
  ******************************************************************************
  * @file    hmi.h
  * @author  Thomas Reisnecker
  * @brief   Header for hmi.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef _HMI_H_
#define _HMI_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
    HMI_BUTTONNOPRESS = 0,
    HMI_BUTTONPRESS,
    HMI_BUTTONSHORT,
    HMI_BUTTONLONG,
    HMI_BUTTONULTRALONG
} hmi_buttonState_t;

typedef enum {
    HMI_LEDRED = 0,
    HMI_LEDBLUE
} hmi_ledType_t;

typedef enum {
    HMI_LEDOFF = 0,
    HMI_LEDON,
    HMI_LEDTOGGLE
} hmi_ledState_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hmi module
 */
void hmi_Init(void);

/** @brief debounce function, this function should be called in the main loop
 *
 * @return button press states of the internal button
 */
hmi_buttonState_t hmi_Debounce(void);

/** @brief function to control the leds
 *
 * @param[in] led   led type to control
 * @param[in] state the new state
 */
void hmi_SetLed(hmi_ledType_t led, hmi_ledState_t state);

#endif /*_HMI_H_*/

/**END OF FILE*****************************************************************/
