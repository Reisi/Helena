/**
  ******************************************************************************
  * @file    link_management.h
  * @author  Thomas Reisnecker
  * @brief   link management module
  *          This module takes care about the link management. It handles
  *          advertising, scanning, connecting and disconnecting.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LINK_MANAGEMENT_H_INCLUDED
#define LINK_MANAGEMENT_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "data_storage.h"
#include "remote.h"
#include "btle.h"
#include "softdevice_handler.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    LM_ADV_OFF = 0,                 // advertising off
    LM_ADV_SLOW,                    // slow advertising, no timeout, not bonded devices will be rejected
    LM_ADV_FAST,                    // fast advertising, timeout 30s, not bonded devices will be rejected (automatic restart with BTLE_ADV_SLOW)
    LM_ADV_OPEN,                    // fast advertising, timeout 30s, open for all connections (automatic restart with BTLE_ADV_SLOW)
} lm_advState_t;

typedef enum
{
    LM_SCAN_OFF = 0,                // scanning is off
    LM_SCAN_LOW_POWER,              // low power scanning for known devices
    LM_SCAN_LOW_LATENCY,            // low latency scanning for known devices
    LM_SCAN_SEARCH,                 // low latency scanning for new devices, after 30s or connection automatically returns to previous mode
} lm_scanState_t;

typedef enum
{
    LM_EVT_ADV_MODE_CHANGED,        // the advertising mode has changed
    LM_EVT_SCAN_MODE_CHANGED,       // the scan mode has changed
    LM_EVT_CONNECTED,               // a connection has been established
    LM_EVT_DISCONNECTED,            // device disconnected
    LM_EVT_DELETED,                 // the event as response to lm_DeleteBonds()
} lm_evtType_t;

typedef struct
{
    uint16_t            connHandle; // the connection handle
    uint8_t             role;       // BLE_GAP_ROLE_CENTRAL or BLE_GAP_ROLE_PERIPH
    rem_driver_t const* pRemDriver; // in case of a remote control, this is the preselected driver, otherwise NULL
} lm_connEvt_t;

typedef struct
{
    lm_evtType_t       type;
    union
    {
        lm_advState_t  newAdvState; // used for LM_EVT_ADV_MODE_CHANGED
        lm_scanState_t newScanState;// used for LM_EVT_SCAN_MODE_CHANGED
        lm_connEvt_t   conn;        // used for LM_EVT_CONNECTED and LM_EVT_DISCONNECTED
        uint32_t       errCode;     // used for LM_EVT_DELETED, containing the result of the delete request
    } evt;
} lm_evt_t;

typedef void (*lm_eventHandler_t)(lm_evt_t const* pEvt);

typedef struct
{
    lm_eventHandler_t eventHandler;     // the event handler
    btle_advType_t    advType;      // the type of advertising that should be used
    uint8_t           uuidType;     // the ble type of the light control and helen project service
} lm_init_t;

typedef enum
{
    LM_EXP_OFF = 0,                 // scanning and advertising is off
    LM_EXP_LOW_POWER,               // advertising enabled (only slow mode), scanning in low power mode, whitelist is used
    LM_EXP_LOW_LATENCY,             // advertising enabled, scanning in low latency mode, whitelist is used
    LM_EXP_SEARCHING,               // scanning in low latency mode without whitelist for 30s, restart with previous mode.
                                    // If advType is BTLE_ADV_TYPE_AFTER_SEARCH, a 30s open advertising is started after scanning
} lm_exposureMode_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function for initializing of the link management module
 *
 * @param[in]  pInit
 * @param[out] pBleEventHandler
 * @return NRF_SUCCESS, NRF_ERROR_NULL or relayed erros
 */
ret_code_t lm_Init(lm_init_t const* pInit, ble_evt_handler_t* pBleEventHandler);

/** @brief function to set the desired exposure mode
 *
 * @param[in] mode
 * @return NRF_SUCCESS or an propagated errors from scanning or advertising
 */
ret_code_t lm_SetExposureMode(lm_exposureMode_t mode);

/** @brief functions to get the actual advertising and scanning states
 */
lm_advState_t lm_GetAdvertisingState(void);
lm_scanState_t lm_GetScanningState(void);

/** @brief functions to get the current number of connected devices
 */
uint8_t lm_GetPeriphCnt(void);
uint8_t lm_GetCentralCnt(void);

/** @brief function to get the preferred connection parameters for GAP
 */
ble_gap_conn_params_t const* lm_getPreferredConnParams(void);

/** @brief function to initiate the disconnection of all devices
 */
void lm_disconnectAll(void);

/** @brief function to disconnect and ignore remote controls
 *
 * @param[in] enable true to disconnect and ignore, false to reconnect
 */
void lm_diconnectAndIgnoreRemote(bool enable);

/** @brief function to initialize the deletion of all bonds
 *
 * @param[in] pHandler the handler that is called after delete operation
 * @return NRF_SUCCESS or the error of pm_peers_delete
 */
ret_code_t lm_DeleteBonds(ds_reportHandler_t pHandler);


#endif // LINK_MANAGEMENT_H_INCLUDED

/**END OF FILE*****************************************************************/
