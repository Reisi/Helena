/**
  ******************************************************************************
  * @file    com.h
  * @author  Thomas Reisnecker
  * @brief   Single wire communication protocol to communicate with battery and
  *          rear light.
  *          Transfering messages is done withing a timer interrupt handler. Due
  *          to the fact that the softdevice might interrupt the code execution
  *          on higher interrupt levels, error can occur. To avoid errors while
  *          transmitting, radio notifications are used and no message is sent
  *          while radio is active.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COM_H_INCLUDED
#define COM_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    COM_MSG_STATUS       = 0,   // status message
    COM_MSG_RESETREQ     = 1,   // reset request
    COM_MSG_ACKRETRREQ   = 2,   // acknowledge request message
    COM_MSG_ACKRETURN    = 3,   // acknowledge return message
    COM_MSG_STATUSRETREQ = 4,   // status return request message
    COM_MSG_REMTRANSREQ  = 15   // remote transfer request
} com_messageType_t;

typedef struct
{
    uint8_t id;                 // identifier
    com_messageType_t type;     // message type
    uint8_t len;                // message lenght
    uint8_t data[15];           // message data
} com_message_t;

typedef enum
{
    COM_EVT_MESSAGE_RECEIVED,   // a new message is available
    COM_EVT_CRC_ERROR,          // message received, but crc does not match
} com_eventType_t;

typedef struct
{
    com_eventType_t type;
    com_message_t const* pMessage;  // data for @ref COM_EVT_MESSAGE_RECEIVED
} com_event_t;

typedef void (*com_eventHandler_t)(com_event_t const* pEvt);

typedef struct
{
    com_eventHandler_t pHandler;    // the handler to report received messages, must not be NULL
    uint32_t txPin;     // pin for transmitting, COM_PIN_NOT_USED for passive mode
    uint32_t rxPin;     // pin for receiving (rx and tx pin can be the same pin)
} com_init_t;

/* Exported constants --------------------------------------------------------*/
#define COM_PIN_NOT_USED    0xFFFFFFFF

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the com module
 *
 * @param[in] pInit the initialization structure
 * @return NRF_SUCCESS
 *         NRF_ERROR_NULL if pInit or pHandler is NULL
 *         NRF_ERROR_INVALID_PARAMS if rxPin is not set
 *         or an propagated initialization error code from the used peripherals
 *
 * @note if the com module is not used, set rx and tx pins to COM_PIN_NOT_USED
 */
ret_code_t com_Init(com_init_t const* pInit);

/** @brief function to send a message
 *
 * @param[in] pMsg the message to send
 * @return NRF_SUCCESS (also, if initialized with rx and tx pins set to COM_PIN_NOT_USED)
 *         NRF_ERROR_INVALID_STATE if not initialized
 *         NRF_ERROR_NOT_SUPPORTED if no tx pin specified
 *         NRF_ERROR_NULL if pMsg is null
 *         NRF_ERROR_NO_MEM if buffer is full
 */
ret_code_t com_Put(com_message_t const* pMsg);

void com_execute(void);

#endif // COM_H_INCLUDED

/**END OF FILE*****************************************************************/

