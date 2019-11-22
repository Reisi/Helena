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
typedef enum
{
    HMI_BT_INTERNAL = 0,
    HMI_BT_RMT_SELECT,
    HMI_BT_RMT_NEXT,
    HMI_BT_RMT_PREV,
    HMI_BT_RMT_UP,
    HMI_BT_RMT_DOWN,
    HMI_BT_CNT
} hmi_buttonType_t;

typedef enum
{
    HMI_BS_NOPRESS = 0,
    HMI_BS_PRESS,
    HMI_BS_SHORT,
    HMI_BS_LONG,
    HMI_BS_ULTRALONG
} hmi_buttonState_t;

typedef enum
{
    HMI_LEDRED = 0,
    HMI_LEDBLUE
} hmi_ledType_t;

typedef enum
{
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
 * @param[out]  pButtonReport   A pointer to an hmi_buttonState_t array with at
 *                              least HMI_BT_CNT members
 * @param[in]   countOf         count of pButtonReport
 * @return      NRF_SUCCESS
 *              NRF_ERROR_NULL
 *              NRF_ERROR_NO_MEM
 */
uint32_t hmi_Debounce(hmi_buttonState_t * pButtonReport, uint16_t countOf);

/** @brief function to report a remote button state
 *
 * @param[in]   type    type of button
 * @param[in]   state   state of button
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INVALID_PARAM
 *
 * @note        The reported button State will be included in the next call of
 *              hmi_Debounce().
 */
uint32_t hmi_ReportButton(hmi_buttonType_t type, hmi_buttonState_t state);

/** @brief function to control the leds
 *
 * @param[in] led   led type to control
 * @param[in] state the new state
 */
void hmi_SetLed(hmi_ledType_t led, hmi_ledState_t state);

#endif /*_HMI_H_*/

/**END OF FILE*****************************************************************/
