/**
  ******************************************************************************
  * @file    com_message_handling.h
  * @author  Thomas Reisnecker
  * @brief   This module filters the messages on the single wire communication
  *          line and only relays relevant messages like:
  *          - mode change requests
  *          - device reset requests
  *          - status message requests
  *          It also processes the messages of taillights and batteries and
  *          buffers the taillight power and battery SOC
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COM_MESSAGE_HANDLING_H_INCLUDED
#define COM_MESSAGE_HANDLING_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "comreloaded.h"

/* Exported types ------------------------------------------------------------*/
/// TODO: change battery SOC from percent in q7_9_t to q16_t or q8_t?
typedef uint16_t q7_9_t;    // used for battery SOC in %
typedef uint16_t q3_13_t;   // used for taillight power in W

typedef enum
{
    CMH_EVENT_MODE_REQUEST,     // mode request message received, change mode and send status message!
    CMH_EVENT_RESET_REQUEST,    // reset request received, reset device!
    CMH_EVENT_STATUS_REQUEST,   // status request received, send status message!
} cmh_eventType_t;

typedef struct
{
    cmh_eventType_t type;
    uint8_t mode;               // the mode to change to for event @ref CMH_EVENT_MODE_REQUEST
} cmh_event_t;

typedef void (*cmh_eventHandler_t)(cmh_event_t const* pEvent);

typedef struct
{
    cmh_eventHandler_t handler;
    uint32_t rxPin;     // the pin that should be used
    uint32_t txPin;     // use COM_PIN_NOT_USED for passive mode
} cmh_init_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the com message handling module
 *
 * @param[in] pInit  the initialization structure
 * @return NRF_SUCCESS
 *         NRF_ERROR_NULL if pInit or handler is NULL
 *         NRF_ERROR_INVALID_PARAM if no rxPin defined
 *         or the propagated error code of the com_Init() function
 *
 * @note if the single wire communication is not used, set both rx and tx pins
 *       to COM_PIN_BOT_USED
 */
ret_code_t cmh_Init(cmh_init_t const* pInit);

/** @brief function to relay a mode change to other devices on the com line
 *
 * @param[in] mode  the mode to relay
 * @return NRF_SUCCESS
 *         NRF_ERROR_INVALID_STATE if not initialized
 *         NRF_ERROR_NOT_SUPPORTED if no txPin defined
 *         or the propagated error code of the com_Put() function
 */
ret_code_t cmh_RelayMode(uint8_t mode);

/// TODO: status message
//ret_code_t cmh_SendStatusMessage

/** @brief function to get the output power of connected taillight
 * @note if data is to old this function will send a remote transfer request
 *
 * @param[out] pPower  the last known output power
 * @return NRF_SUCCESS
 *         NRF_ERROR_NULL if pPower is null
 *         NRF_ERROR_INVALID_STATE if the app_timer is inactive (no timeout calculation possible)
 *         NRF_ERROR_NOT_FOUND if no taillight available
 *         NRF_ERROR_TIMEOUT if last known data is older than 30 sec
 */
ret_code_t cmh_GetTaillightPower(q3_13_t* pPower);

/** @brief function to get the state of charge of connected batteries
 * @note if data is to old this function will send a remote transfer request
 *
 * @param[out] pSOC  the last known state of charge
 * @return NRF_SUCCESS
 *         NRF_ERROR_NULL if pSOC is null
 *         NRF_ERROR_INVALID_STATE if the app_timer is inactive (no timeout calculation possible)
 *         NRF_ERROR_NOT_FOUND if no battery available
 *         NRF_ERROR_TIMEOUT if last known data is older than 30 sec
 */
ret_code_t cmh_GetBatterySOC(q7_9_t* pSOC);

#endif // COM_MESSAGE_HANDLING_H_INCLUDED

/**END OF FILE*****************************************************************/

