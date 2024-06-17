/**
  ******************************************************************************
  * @file    hmi.h
  * @author  Thomas Reisnecker
  * @brief   Header for hmi module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HMI_H_INCLUDED
#define HMI_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "data_storage.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    HMI_LT_RED = 0,
    HMI_LT_GREEN,
    HMI_LT_BLUE,
    HMI_LT_CNT
} hmi_ledType_t;

typedef enum
{
    HMI_LS_OFF = 0,
    HMI_LS_BLINKFAST,
    HMI_LS_BLINKSLOW,
    HMI_LS_ON
} hmi_ledState_t;

typedef enum
{
    HMI_EVT_MODEOFF,        // go to off mode
    HMI_EVT_MODEPREF,       // go to preferred mode
    HMI_EVT_MODETEMP,       // go to temporary mode
    HMI_EVT_MODESOS,        // go to SOS mode
    HMI_EVT_MODENUMBER,     // go to mode number
    HMI_EVT_NEXTMODE,       // go to next mode
    HMI_EVT_PREVMODE,       // go to previous mode
    HMI_EVT_NEXTGROUP,      // go to next group
    HMI_EVT_PREVGROUP,      // go to previous group
    HMI_EVT_SEARCHREMOTE,   // start procedure to search for remote devices
    HMI_EVT_DELETEBONDS,    // start procedure to delete bonds
    HMI_EVT_FACTORYRESET    // start procedure to restore to factory setting
} hmi_evtType_t;

typedef struct
{
    uint16_t connHandle;    // the originator event, CONN_HANDLE_INVALID for wired com
    uint8_t modeNumber;     // the desired mode number
} hmi_modeReq_t;

typedef union
{
    hmi_modeReq_t      mode;          // params for event HMI_EVT_MODENUMBER
    ds_reportHandler_t resultHandler; // params for event HMI_EVT_DELETEBONDS and HMI_EVT_FACTORYRESET
} hmi_evtParams_t;

typedef struct
{
    hmi_evtType_t type;     // the type of event
    hmi_evtParams_t params;
} hmi_evt_t;

typedef uint32_t (* hmi_eventHandler_t)(hmi_evt_t const* pEvent);

typedef struct
{
    hmi_eventHandler_t eventHandler;
} hmi_init_t;

typedef hmi_evtType_t hmi_evtRequest_t;

/* Exported constants --------------------------------------------------------*/
#define HMI_LED_NA  0xFFFFFFFF

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hmi module
 *
 * @param[in] init the initialization structure
 *
 * @return NRF_SUCCESS
 * @return NRF_ERROR_NULL
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

/** @brief function to control the leds
 *
 * @param[in] led   led type to control
 * @param[in] state the new state
 */
void hmi_SetLed(hmi_ledType_t led, hmi_ledState_t state);

/** @brief function to request an event
 *
 * @param[in] request
 * @param[in] mode    the parameters for HMI_EVT_MODENUMBER, use NULL otherwise
 * @return NRF_SUCCESS
 * @return NRF_ERROR_INVALID_STATE
 */
uint32_t hmi_RequestEvent(hmi_evtRequest_t request, hmi_evtParams_t* pParams);

#endif // HMI_H_INCLUDED

/**END OF FILE*****************************************************************/

