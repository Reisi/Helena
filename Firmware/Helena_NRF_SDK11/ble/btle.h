/**
  ******************************************************************************
  * @file    btle.h
  * @author  Thomas Reisnecker
  * @brief   bluetooth module header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BTLE_H_INCLUDED
#define BTLE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "section_vars.h"
#include "ble.h"
#include "ble_hps.h"
#include "ble_dis.h"
#include "version.h"
#include "app_error.h"

/* section variable -----------------------------------------------------------*/
typedef uint32_t (*btle_serviceAdd_t)(void * pContext);
typedef void (*btle_onEvent_t)(void * pContext, ble_evt_t * pBleEvt);

typedef struct
{
    btle_serviceAdd_t  serviceAdd;  /**< Function to add board specific services. */
    btle_onEvent_t     eventHandler;
    void             * pContext;    /**< context for board specific services */
} btle_service_t;

NRF_SECTION_VARS_REGISTER_SYMBOLS(btle_service_t, btle_services);

#define BTLE_SERVICE_REGISTER(_name, _init, _onEvent, _context)  \
    NRF_SECTION_VARS_ADD(btle_services, btle_service_t _name) =  \
    {                                                            \
        .serviceAdd = _init,                                     \
        .eventHandler = _onEvent,                                \
        .pContext = _context                                     \
    }

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    BTLE_EVT_NAME_CHANGE                // event indicating that the device name was changed
} btle_evt_type_t;

typedef struct
{
    btle_evt_type_t type;
    union
    {
        char const* pNewDeviceName;     // the new device name for BTLE_EVT_NAME_CHANGE event
    };
} btle_event_t;

typedef void (*btle_eventHandler_t)(btle_event_t const * pEvt);

typedef enum
{
    BTLE_ADV_TYPE_ALWAYS_OPEN,          // advertising is always open for unknown devices
    BTLE_ADV_TYPE_STARTUP,              // connections from unknown devices only possible at start
    BTLE_ADV_TYPE_AFTER_SEARCH          // connections from unknown devices only possible after search window
} btle_advType_t;

typedef struct
{
    char const* pDevicename;            // the name of the device
    char const* pManufacturer;
    char const* pModelNumber;
    char const* pBoardHWVersion;
    uint16_t deviceAppearance;          // the used appearance
} btle_info_t;

typedef struct
{
    btle_advType_t       advType;       // the type of advertising, that should be used
    uint16_t             maxDeviceNameLength;       // the maximum supported device name length, 0 is name change is not supported, max value is BLE_GAP_DEVNAME_DEFAULT_LEN (31 bytes)
    btle_info_t const*   pInfo;         // the information included into the GAP and Device Information Service
    btle_eventHandler_t  eventHandler;  // the event handler for btle events
    ble_hps_init_t*      pHpsInit;      // the initialization structure for the Helen Project Service
} btle_init_t;

typedef struct
{
    uint16_t connHandle;
    uint8_t  mode;
} btle_modeRelay_t;

typedef struct
{
    uint8_t mode;           // current mode
    uint16_t outputPower;   // output power in mW, will be included if > 0
    int8_t temperature;     // temperature in °C, will be included if value is in range -40..85
    uint16_t inputVoltage;  // input voltage in mV, will be included if > 0
} btle_hpsMeasurement_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the bluetooth module
 *
 * @param[in] pInit
 * @return NRF_SUCCESS
 */
ret_code_t btle_Init(btle_init_t const* pInit);

/** @brief function to relay a mode change to connected lights
 *
 * @note use this function to relay a mode change to connected devices. Use
 *       BLE_CONN_HANDLE_ALL to relay to all connected devices (when mode
 *       change was initiated from this device). If the mode change was already
 *       relayed from another device use the connection Handle of this device
 *       to prevent a command back to the originator.
 * @param[in] pRelay  the source and the mode to relay
 * @return NRF_ERROR_NOT_FOUND if no device connected, otherwise NRF_SUCCESS
 */
ret_code_t btle_RelayMode(btle_modeRelay_t const* pRelay);

/** @brief Function for updating light information. If a device is connected and
 *         light control service notifications are enabled a notification
 *         message will be sent.
 * @note call this function about once per second with actual values
 *
 * @param[in]   pData   light data to be sent
 * @return      NRF_SUCCESS, NRF_ERROR_INVALID_STATE if no notifications are to
 *              send or the propagated error of ble_lcs_light_measurement_send()
 */
ret_code_t btle_ReportHpsMeasurements(btle_hpsMeasurement_t const* pData);

#endif /* BTLE_H_INCLUDED */

/**END OF FILE*****************************************************************/
