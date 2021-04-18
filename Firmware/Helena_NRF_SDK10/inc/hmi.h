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
#include "btle.h"

/* Exported types ------------------------------------------------------------*/
/*typedef enum
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
} hmi_buttonState_t;*/

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

typedef enum
{
    HMI_EVT_INTERNAL_SHORT = 0,
    HMI_EVT_INTERNAL_LONG,
    HMI_EVT_INTERNAL_HOLD,
    HMI_EVT_XIAOMI_PRI_SHORT,
    HMI_EVT_XIAOMI_PRI_LONG,
    HMI_EVT_XIAOMI_PRI_HOLD,
    HMI_EVT_XIAOMI_SEC_SHORT,
    HMI_EVT_XIAOMI_SEC_LONG,
    HMI_EVT_XIAOMI_SEC_HOLD,
    HMI_EVT_R51_PLAYPAUSE,
    HMI_EVT_R51_VOLUP,
    HMI_EVT_R51_VOLDOWN,
    HMI_EVT_R51_NEXTTRACK,
    HMI_EVT_R51_PREVTRACK,
    HMI_EVT_R51_MODE
} hmi_eventType_t;

/**@brief     Event handler type.
 *
 * @details   This is the type of the event handler that should be provided by
 *            the application of this module in order to receive events.
 * @note      The event handler is always call from main context.
 *
 * @param[in] event the type of event
 */
typedef void (* hmi_eventHandler_t)(hmi_eventType_t event);

typedef struct
{
    hmi_eventHandler_t eventHandler;
} hmi_init_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hmi module
 *
 * @param[in] pInit the init structure
 * @return NRF_SUCCESS
 *         NRF_ERROR_NULL if init structure is null
 */
uint32_t hmi_Init(hmi_init_t const* pInit);

/** @brief The execute function of the hmi module
 *
 * @details This function handles the debouncing of the internal button and the
 *          evaluation of short or long clicks or hold buttons
 *
 * @note   call this function on a regular basis in the main loop.
 */
void hmi_Execute(void);

/** @brief Function to report events from the ble hid server
 *
 * @param[in] hidEvent the event received from the ble module
 */
uint32_t hmi_ReportHidEvent(btle_hidEventType_t hidEvent);

/** @brief debounce function, this function should be called in the main loop
 *
 * @param[out]  pButtonReport   A pointer to an hmi_buttonState_t array with at
 *                              least HMI_BT_CNT members
 * @param[in]   countOf         count of pButtonReport
 * @return      NRF_SUCCESS
 *              NRF_ERROR_NULL
 *              NRF_ERROR_NO_MEM
 */
//uint32_t hmi_Debounce(hmi_buttonState_t * pButtonReport, uint16_t countOf);

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
//uint32_t hmi_ReportButton(hmi_buttonType_t type, hmi_buttonState_t state);

/** @brief function to control the leds
 *
 * @param[in] led   led type to control
 * @param[in] state the new state
 */
void hmi_SetLed(hmi_ledType_t led, hmi_ledState_t state);

#endif /*_HMI_H_*/

/**END OF FILE*****************************************************************/
