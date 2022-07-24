/**
  ******************************************************************************
  * @file    btle.c
  * @author  Thomas Reisnecker
  * @brief   helenas bluetooth module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "btle.h"
#include "ble.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "sdk_errors.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "app_timer.h"
#include "main.h"
#include "ble_dis.h"
#include "ble_cgw.h"
#include "ble_lcs.h"
#include "ble_hids_c.h"
#include "ble_hids_c_hardcoded.h"
#include "ble_lcs_c.h"
#include "ble_advertising.h"
#include "ble_db_discovery.h"
#include "ble_scanning.h"
#include "peer_manager.h"
#include "id_manager.h"
#include "fstorage.h"
#include "fds.h"
#include "version.h"

#ifdef BTDEBUG
#include "ble_nus.h"
#include "debug.h"
#include "SEGGER_RTT.h"
#endif

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    CENTRAL_NONE = 0,               // no device connected
    CENTRAL_LIGHT,                  // light device
    CENTRAL_XIAOMI,                 // Xiaomi Yi Remote
    CENTRAL_R51,                    // R51 remote
    CENTRAL_AUVISO                  // Auviso remote
} centralDevice_t;

typedef struct
{
    btle_eventHandler_t handler;    // event handler given at initialization
    btle_scanMode_t scanConfig;     // current scan configuration
    ble_scan_mode_t scanMode;       // current scan mode
    centralDevice_t centralDevice;  // current device type in central connection
    bool allowPeripheralBonding;    // indicating if the bonding for peripheral is allowed
    uint16_t connHandleCentral;     // connection handle related to central connection
    uint16_t connHandlePeriph;      // connection handle related to peripheral connection
} state_t;

/* Private macros ------------------------------------------------------------*/
#define COUNT_OF(x)                     (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/
#define NUM_OF_PERIPH_CONNECTIONS       1
#define NUM_OF_CENTRAL_CONNECTIONS      1
#define NUM_OF_CONNECTIONS              (NUM_OF_PERIPH_CONNECTIONS + NUM_OF_CENTRAL_CONNECTIONS)

#define UUID16_SIZE                     2
#define UUID128_SIZE                    16

#ifdef HELENA
#define DEVICE_NAME                     "Helena"
#elif defined BILLY
#define DEVICE_NAME                     "Billina"
#endif // BILLY
#define DEVICE_APPEARANCE               BLE_APPEARANCE_GENERIC_CYCLING

#define LCS_NOTIFY_PERIOD               APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

#define LOG(...)                        //SEGGER_RTT_printf(0, __VA_ARGS__)

/* Private variables ---------------------------------------------------------*/
static const ble_gap_sec_params_t secParamBond =
{
    .bond              = true,
    .mitm              = false,
    .io_caps           = BLE_GAP_IO_CAPS_NONE,
    .oob               = false,
    .min_key_size      = 7,
    .max_key_size      = 16,
    .kdist_periph.enc  = 1,
    .kdist_periph.id   = 1,
    .kdist_central.enc = 1,
    .kdist_central.id  = 1,
};

static ble_cgw_t cgwGattsData;                  // gatt server handles for the com gateway service
BLE_LCS_DEF(lcsGattsData, NUM_OF_CONNECTIONS);  // gatt server handles for the light control service

#if BTDEBUG
static ble_nus_t nusGattsData;                  // gatt server handles for the nordic uart service
#endif // BTDEBUG

static ble_db_discovery_t discDatabase;         // database for service discovery
static ble_hids_c_t hidsGattcData;              // gatt client handles for the hid service
BLE_LCS_C_DEF(lcsGattcData, NUM_OF_CONNECTIONS);// gatt client handles for the lcs service

static state_t state;                           // current state

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @brief Application event handler for central events from the ble stack
 */
static void onCentralEvt(ble_evt_t * pBleEvt)
{
    btle_event_t evt;
    uint32_t errCode;

    switch (pBleEvt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        state.connHandleCentral = pBleEvt->evt.gap_evt.conn_handle;

        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_CONNECTED;
        evt.connHandle = pBleEvt->evt.gap_evt.conn_handle;
        state.handler(&evt);

        // secure link if device is not bonded yet, but not for R51 remotes,
        // these remotes won't let helena reconnect after bonding
        pm_link_status_t linkStatus;
        (void)pm_link_status_get(state.connHandleCentral, &linkStatus);
        if (state.centralDevice == CENTRAL_R51)
        {
            // check if this is a new R51 remote, if yes store device address
            pm_peer_id_t idDummy;
            (void)pm_peer_id_get(pBleEvt->evt.gap_evt.conn_handle, &idDummy); // always returns NRF_SUCCESS
            if (idDummy == PM_PEER_ID_INVALID)
            {
                pm_store_token_t dummyToken;
                static pm_peer_data_bonding_t bondingData;
                uint32_t errCode;

                memset(&bondingData, 0, sizeof(bondingData));
                bondingData.own_role = BLE_GAP_ROLE_CENTRAL;
                errCode = im_ble_addr_get(pBleEvt->evt.gap_evt.conn_handle, &bondingData.peer_id.id_addr_info);
                APP_ERROR_CHECK(errCode);
                errCode = pm_peer_new(&bondingData, &idDummy, &dummyToken);
                if (errCode == NRF_ERROR_NO_MEM)
                    errCode = fds_gc();     /// TODO: try to store again
                APP_ERROR_CHECK(errCode);
            }
        }
        else if (linkStatus.connected && !linkStatus.encrypted)
        {
            if (linkStatus.bonded)
                errCode = pm_link_secure(state.connHandleCentral, false);// for non R51 devices secure link
            else
                errCode = pm_link_secure(state.connHandleCentral, true);
            if (errCode == NRF_ERROR_NO_MEM)                            // run garbage collection, peer manager will try to save again
                errCode = fds_gc();
            APP_ERROR_CHECK(errCode);
        }

        // start database discovery
        if (discDatabase.discovery_in_progress == false)
        {
            memset(&discDatabase, 0, sizeof(discDatabase));
            errCode = ble_db_discovery_start(&discDatabase, state.connHandleCentral);
            APP_ERROR_CHECK(errCode); /// TODO: sometimes NRF_ERROR_BUSY at startup, maybe reconnection and discovery was aborted previously?
        }

        // activate notifications here, and not in the database discovery event handler (not available with hardcoded devices)
        if (state.centralDevice == CENTRAL_XIAOMI || state.centralDevice == CENTRAL_R51 || state.centralDevice == CENTRAL_AUVISO)
        {
            errCode = ble_hids_c_report_notif_enable(&hidsGattcData, 2, true);
            APP_ERROR_CHECK(errCode);
            hidsGattcData.conn_handle = pBleEvt->evt.gap_evt.conn_handle;
        }
        break;

    case BLE_GAP_EVT_TIMEOUT:
        if (pBleEvt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
        {
            // if connection failed central device has to be reset
            if (state.centralDevice == CENTRAL_XIAOMI || state.centralDevice == CENTRAL_R51 || state.centralDevice == CENTRAL_AUVISO)
                hidsGattcData.conn_handle = BLE_CONN_HANDLE_INVALID;
            state.centralDevice = CENTRAL_NONE;
            /// TODO: restart scanning
        }
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_DISCONNECTED;
        evt.connHandle = pBleEvt->evt.gap_evt.conn_handle;
        state.handler(&evt);

        state.connHandleCentral = BLE_CONN_HANDLE_INVALID;
        state.centralDevice = CENTRAL_NONE;
        break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    {
        ble_gap_conn_params_t newParams = pBleEvt->evt.gap_evt.params.conn_param_update_request.conn_params;

        // R51 remotes wanna have connection intervals from 250 to 500 msec, thats to lazy!!
        if (state.centralDevice == CENTRAL_R51 && newParams.min_conn_interval > MSEC_TO_UNITS(100, UNIT_1_25_MS))
        {
            newParams.min_conn_interval = MSEC_TO_UNITS(50, UNIT_1_25_MS);
            newParams.max_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS);
        }

        errCode = sd_ble_gap_conn_param_update(pBleEvt->evt.gap_evt.conn_handle, &newParams);
        APP_ERROR_CHECK(errCode);

    }   break;

    default:
        break;
    }
}

/** @brief Application event handler for peripheral events form the ble stack.
 */
static void onPeriphEvt(ble_evt_t * pBleEvt)
{
    btle_event_t evt;

    switch (pBleEvt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        state.connHandlePeriph = pBleEvt->evt.gap_evt.conn_handle;

        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_PERIPH_CONNECTED;
        evt.connHandle = pBleEvt->evt.gap_evt.conn_handle;
        state.handler(&evt);

        /// TODO: maybe bonding request?

        // start database discovery
        if (discDatabase.discovery_in_progress == false)
        {
            memset(&discDatabase, 0, sizeof(discDatabase));
            uint32_t errCode = ble_db_discovery_start(&discDatabase, state.connHandlePeriph);
            APP_ERROR_CHECK(errCode);
        }
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_PERIPH_DISCONNECTED;
        evt.connHandle = pBleEvt->evt.gap_evt.conn_handle;
        state.handler(&evt);

        state.connHandlePeriph = BLE_CONN_HANDLE_INVALID;
        break;

    default:
        break;
    }
}

/** @brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 */
static void bleEvtDispatch(ble_evt_t * pBleEvt)
{
    LOG("[BTLE]: evt 0x%02x\r\n", pBleEvt->header.evt_id);

    ble_conn_state_on_ble_evt(pBleEvt);
    uint8_t role = ble_conn_state_role(pBleEvt->evt.gap_evt.conn_handle);

    // not possible to reconnect to R51 remotes after bonding, but they won't
    // disconnect if bonding is not performed, so let's just ignore the
    // request until I have a better idea.
    if (role == BLE_GAP_ROLE_CENTRAL && state.centralDevice == CENTRAL_R51 && pBleEvt->header.evt_id == BLE_GAP_EVT_SEC_REQUEST)
    {

    }
    // if a bonding request is received in peripheral connection, the security
    // parameters have to be changed if bonding is not allowed.
    else if (role == BLE_GAP_ROLE_PERIPH && pBleEvt->header.evt_id == BLE_GAP_EVT_SEC_PARAMS_REQUEST && !state.allowPeripheralBonding)
    {
        APP_ERROR_CHECK(pm_sec_params_set(NULL));
        pm_ble_evt_handler(pBleEvt);
        ble_gap_sec_params_t secParams;
        secParams = secParamBond;
        APP_ERROR_CHECK(pm_sec_params_set(&secParams));
    }
    else
        pm_ble_evt_handler(pBleEvt);

    // database discovery and light control service (client and server) are used in both directions
    ble_db_discovery_on_ble_evt(&discDatabase, pBleEvt);
    ble_lcs_on_ble_evt(&lcsGattsData, pBleEvt);
    ble_lcs_c_on_ble_evt(&lcsGattcData, pBleEvt);

    if (role == BLE_GAP_ROLE_PERIPH || role == BLE_GAP_ROLE_INVALID)
    {
        ble_conn_params_on_ble_evt(pBleEvt);
        ble_advertising_on_ble_evt(pBleEvt);
        ble_cgw_on_ble_evt(&cgwGattsData, pBleEvt); // the com gateway is only used in peripheral connections
#ifdef BTDEBUG
        ble_nus_on_ble_evt(&nusGattsData, pBleEvt);
#endif
        onPeriphEvt(pBleEvt);
    }
    if (role == BLE_GAP_ROLE_CENTRAL || role == BLE_GAP_ROLE_INVALID)
    {
        ble_scanning_on_ble_evt(pBleEvt);
        ble_hids_c_on_ble_evt(&hidsGattcData, pBleEvt); // the hid server client is only used in central connections
        onCentralEvt(pBleEvt);
    }
}

/**@brief Function for dispatching a system event to interested modules.
 */
static void sysEvtDispatch(uint32_t sysEvt)
{
    ble_advertising_on_sys_evt(sysEvt);
    fs_sys_event_handler(sysEvt);
    com_OnSysEvt(sysEvt);
#ifdef BTDEBUG
    debug_OnSysEvent(sysEvt);
#endif
}

static pm_peer_id_t nextCentralPeerIdGet(pm_peer_id_t peerId)
{
    uint32_t errCode;
    pm_peer_data_bonding_t bondData;
    pm_peer_data_t peerData;
    peerData.length_words = PM_BONDING_DATA_N_WORDS();
    peerData.data.p_bonding_data = &bondData;
    peerData.data_type = PM_PEER_DATA_ID_BONDING;

    while (1)
    {
        peerId = pm_next_peer_id_get(peerId);
        if (peerId == PM_PEER_ID_INVALID)
            return peerId;


        errCode = pm_peer_data_get(peerId, PM_PEER_DATA_ID_BONDING, &peerData);
        if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
            APP_ERROR_HANDLER(errCode);

        if (peerData.data.p_bonding_data->own_role == BLE_GAP_ROLE_CENTRAL)
            return peerId;
    }
}

static void deleteOtherCentrals(pm_peer_id_t centralToKeep)
{
    pm_peer_id_t peerId = nextCentralPeerIdGet(PM_PEER_ID_INVALID);
    while (peerId != PM_PEER_ID_INVALID)
    {
        if (peerId != centralToKeep)
        {
            uint16_t connHandle;
            uint32_t errCode;
            // disconnect before delete
            errCode = pm_conn_handle_get(peerId, &connHandle);
            APP_ERROR_CHECK(errCode);
            if (connHandle != BLE_CONN_HANDLE_INVALID)
            {
                errCode = sd_ble_gap_disconnect(connHandle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                if (errCode != NRF_SUCCESS && errCode != BLE_ERROR_INVALID_CONN_HANDLE)
                    APP_ERROR_HANDLER(errCode);        //^- already disconnected but not received event yet
            }

            pm_peer_delete(peerId);
        }
        peerId = nextCentralPeerIdGet(peerId);
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pmEvtHandler(pm_evt_t const * pEvt)
{
    uint32_t errCode;

    switch(pEvt->evt_id)
    {
    case PM_EVT_LINK_SECURE_FAILED:
        // In some cases, when securing fails, it can be restarted directly. Sometimes it can
        // be restarted, but only after changing some Security Parameters. Sometimes, it cannot
        // be restarted until the link is disconnected and reconnected. Sometimes it is
        // impossible, to secure the link, or the peer device does not support it. How to
        // handle this error is highly application dependent.
        if (pEvt->params.link_secure_failed_evt.error.error_type == PM_ERROR_TYPE_PM_SEC_ERROR)
        {
            switch (pEvt->params.link_secure_failed_evt.error.error.pm_sec_error)
            {
                case PM_SEC_ERROR_CODE_PIN_OR_KEY_MISSING:
                    // Reject if bonding is not allowed
                    if (!state.allowPeripheralBonding)
                        break;
                    // Rebond if one party has lost its keys.
                    errCode = pm_link_secure(pEvt->conn_handle, true);
                    if (errCode != NRF_ERROR_INVALID_STATE)
                    {
                        APP_ERROR_CHECK(errCode);
                    }
                    break;

                default:
                    break;
            }
        }
        else if (pEvt->params.link_secure_failed_evt.error.error_type == PM_ERROR_TYPE_SEC_STATUS)
        {
            switch (pEvt->params.link_secure_failed_evt.error.error.sec_status)
            {
                default:
                    break;
            }
        }
        break;

    case PM_EVT_STORAGE_FULL:
        // Run garbage collection on the flash.
        errCode = fds_gc();
        if (errCode != NRF_ERROR_BUSY)
        {
            APP_ERROR_CHECK(errCode);
        }
        break;

    case PM_EVT_ERROR_UNEXPECTED:
        // A likely fatal error occurred. Assert.
        APP_ERROR_CHECK(pEvt->params.error_unexpected_evt.error);
        break;

    case PM_EVT_PEER_DATA_UPDATE_FAILED:
        APP_ERROR_CHECK_BOOL(false);
        break;

    case PM_EVT_ERROR_LOCAL_DB_CACHE_APPLY:
        // The local database has likely changed, send service changed indications.
        pm_local_database_has_changed();
        break;

    case PM_EVT_BONDED_PEER_CONNECTED:
        LOG("[BTLE]: Peer connected \r\n");
        break;
    case PM_EVT_LINK_SECURED:
        LOG("[BTLE]: link secured %d\r\n", pEvt->params.link_secured_evt.procedure);
        break;
    case PM_EVT_PEER_DATA_UPDATED:
    case PM_EVT_LOCAL_DB_CACHE_APPLIED:
    case PM_EVT_SERVICE_CHANGED_INDICATION_SENT:
    default:
        break;
    }
}

/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peerManagerInit(bool eraseBonds)
{
    uint32_t errCode;

    errCode = pm_init();
    APP_ERROR_CHECK(errCode);

    if (eraseBonds)
    {
        pm_peer_delete_all();
    }

    ble_gap_sec_params_t secParam = secParamBond;
    errCode = pm_sec_params_set(&secParam);
    APP_ERROR_CHECK(errCode);

    errCode = pm_register(pmEvtHandler);
    APP_ERROR_CHECK(errCode);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gapParamsInit(void)
{
    uint32_t errCode;

    ble_gap_conn_sec_mode_t secMode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode);
    errCode = sd_ble_gap_device_name_set(&secMode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(errCode);

    errCode = sd_ble_gap_appearance_set(DEVICE_APPEARANCE);
    APP_ERROR_CHECK(errCode);

    ble_gap_conn_params_t gapConnParams;
    memset(&gapConnParams, 0, sizeof(ble_gap_conn_params_t));
    gapConnParams.min_conn_interval = MSEC_TO_UNITS(50, UNIT_1_25_MS);
    gapConnParams.max_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS);
    gapConnParams.slave_latency     = 0;
    gapConnParams.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS);

    errCode = sd_ble_gap_ppcp_set(&gapConnParams);
    APP_ERROR_CHECK(errCode);
}

/** @brief error handler for connection parameter module
 */
static void connParamsErrorHandler(uint32_t errCode)
{
    APP_ERROR_HANDLER(errCode);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void connParamsInit(void)
{
    uint32_t errCode;
    ble_conn_params_init_t connParamInit;

    memset(&connParamInit, 0, sizeof(connParamInit));

    connParamInit.p_conn_params                  = NULL;    // use default connection parameter
    connParamInit.first_conn_params_update_delay = APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER);
    connParamInit.next_conn_params_update_delay  = APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER);
    connParamInit.max_conn_params_update_count   = 3;
    connParamInit.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;//cgwGattsData.rx_handles.cccd_handle;
    connParamInit.disconnect_on_fail             = true;
    connParamInit.evt_handler                    = NULL;
    connParamInit.error_handler                  = connParamsErrorHandler;

    errCode = ble_conn_params_init(&connParamInit);
    APP_ERROR_CHECK(errCode);
}

/** @brief handler for incomming messages of the com bluetooth gateway service
 */
static void cgwDataHandler(ble_cgw_t * pCgw, com_MessageStruct * pMessageRx)
{
    (void)com_Put(pMessageRx);
}

/** @brief Event handler for the light control service
 */
static void lcsCpEventHandler(ble_lcs_ctrlpt_t * pLcsCtrlpt,
                                                   ble_lcs_ctrlpt_evt_t * pEvt)
{
    if (state.handler == NULL)
        return;

    btle_event_t evt;
    evt.evt = BTLE_EVT_LCS_CTRL_POINT;
    evt.connHandle = pEvt->conn_handle;

    switch (pEvt->evt_type)
    {
    case BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_MODE_CNT;
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_GRP_CNFG:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_GROUP_CNT;
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_MODE_CNFG:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_MODE_CONFIG;
        evt.lcscpEventParams.modeConfigStart = pEvt->p_params->mode_list_start;
        break;
    case BLE_LCS_CTRLPT_EVT_SET_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_SET_MODE;
        evt.lcscpEventParams.modeToSet = pEvt->p_params->set_mode;
        break;
    case BLE_LCS_CTRLPT_EVT_CNFG_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_CONFIG_MODE;
        evt.lcscpEventParams.modeToConfig.modeNumber = pEvt->p_params->mode_config.mode_number_start;
        evt.lcscpEventParams.modeToConfig.listEntries = pEvt->p_params->mode_config.mode_entries;
#ifdef HELENA
        evt.lcscpEventParams.modeToConfig.pConfig = (btle_LcsModeConfig_t*)pEvt->p_params->mode_config.config_hlmt;
#elif defined BILLY
        evt.lcscpEventParams.modeToConfig.pConfig = (btle_LcsModeConfig_t*)pEvt->p_params->mode_config.config_bk;
#endif // defined
        break;
    case BLE_LCS_CTRLPT_EVT_CNFG_GROUP:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_CONFIG_GROUP;
        evt.lcscpEventParams.groupConfig.numOfModes = pEvt->p_params->group_config.number_of_modes;
        evt.lcscpEventParams.groupConfig.pNumOfModesPerGroup = pEvt->p_params->group_config.p_list_of_modes_per_group;
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_LED_CNFG:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_LED_CONFIG;
        break;
    case BLE_LCS_CTRLPT_EVT_CHK_LED_CNFG:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_CHECK_LED_CONFIG;
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_SENS_OFF:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_SENS_OFFSET;
        break;
    case BLE_LCS_CTRLPT_EVT_CALIB_SENS_OFF:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_CALIB_SENS_OFFSET;
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_LIMITS:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_LIMITS;
        break;
    case BLE_LCS_CTRLPT_EVT_SET_LIMITS:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_SET_LIMITS;
#ifdef HELENA
        evt.lcscpEventParams.currentLimits.floodInPercent = pEvt->p_params->current_limits.flood;
        evt.lcscpEventParams.currentLimits.spotInPercent = pEvt->p_params->current_limits.spot;
#elif defined BILLY
        evt.lcscpEventParams.currentLimits.mainBeamInPercent = pEvt->p_params->current_limits.main_beam;
        evt.lcscpEventParams.currentLimits.highBeamInPercent = pEvt->p_params->current_limits.high_beam;
#endif // defined
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_PREF_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_PREF_MODE;
        break;
    case BLE_LCS_CTRLPT_EVT_SET_PREF_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_SET_PREF_MODE;
        evt.lcscpEventParams.prefMode = pEvt->p_params->pref_mode;
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_TEMP_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_TEMP_MODE;
        break;
    case BLE_LCS_CTRLPT_EVT_SET_TEMP_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_SET_TEMP_MODE;
        evt.lcscpEventParams.prefMode = pEvt->p_params->temp_mode;
        break;
    default:
        return;
    }
    state.handler(&evt);
}

/** @brief Error handler for the light control service
 */
static void lcsErrorHandler(uint32_t errCode)
{
    if (errCode != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        APP_ERROR_HANDLER(errCode);
    }
}

#ifdef BTDEBUG
static void nusEventHandler(ble_nus_t *pNus, uint8_t *pData, uint16_t length)
{
    debug_OnNusEvt(pData, length);
}
#endif // BTDEBUG

/**@brief Function for initializing services that will be used by the application.
 */
static void servicesInit(char const* pDrvRev, btle_lcsFeature_t* pFeature)
{
    uint32_t errCode;

    //initialize device information service
    ble_dis_init_t disInit;
    char versionString[48];
    uint_fast8_t i = 0;
    if (VERSION_MAJOR / 10)
        versionString[i++] = (VERSION_MAJOR / 10) + '0';
    versionString[i++] = (VERSION_MAJOR % 10) + '0';
    versionString[i++] = '.';
    if (VERSION_MINOR / 10)
        versionString[i++] = (VERSION_MINOR / 10) + '0';
    versionString[i++] = (VERSION_MINOR % 10) + '0';
    versionString[i++] = '.';
    if (VERSION_PATCH / 10)
        versionString[i++] = (VERSION_PATCH / 10) + '0';
    versionString[i++] = (VERSION_PATCH % 10) + '0';
#ifdef DEBUG
    versionString[i++] = '-';
    strcpy(&versionString[i], VERSION_LEVEL);
    i = strlen(versionString);
    versionString[i++] = '-';
    itoa(VERSION_BUILD, &versionString[i], 10);
#endif //DEBUG
    if (pDrvRev != NULL)
    {
        i = strlen(versionString);
        strcat(&versionString[i], " | ");
        strcat(&versionString[i+3], pDrvRev);
    }
    memset(&disInit, 0, sizeof(ble_dis_init_t));
    ble_srv_ascii_to_utf8(&disInit.fw_rev_str, versionString);
    ble_srv_ascii_to_utf8(&disInit.hw_rev_str, (char*)pBoardConfig->hwRevStr);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&disInit.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&disInit.dis_attr_md.write_perm);
    errCode = ble_dis_init(&disInit);
    APP_ERROR_CHECK(errCode);

    //initialize com bluetooth gateway service
    ble_cgw_init_t cgwInit;
    memset(&cgwInit, 0, sizeof(ble_cgw_init_t));
    cgwInit.data_handler = cgwDataHandler;
    errCode = ble_cgw_init(&cgwGattsData, &cgwInit);
    APP_ERROR_CHECK(errCode);

    //initialize light control service
    ble_lcs_init_t lcsInit;
    memset(&lcsInit, 0, sizeof(ble_lcs_init_t));
    lcsInit.evt_handler = NULL;
    if (state.handler != NULL)                      // control point can not be
        lcsInit.cp_evt_handler = lcsCpEventHandler; // included without event handler
    lcsInit.error_handler = lcsErrorHandler;
#ifdef HELENA
    lcsInit.feature.light_type = BLE_LCS_LT_HELMET_LIGHT;
    lcsInit.feature.hlmt_features.flood_supported = pFeature->floodSupported;
    lcsInit.feature.hlmt_features.spot_supported = pFeature->spotSupported;
    lcsInit.feature.hlmt_features.pitch_comp_supported = pFeature->pitchSupported;
    lcsInit.feature.hlmt_features.driver_cloning_supported = pFeature->cloneSupported;
    lcsInit.feature.hlmt_features.external_taillight_supported = pFeature->taillightSupported;
    lcsInit.feature.hlmt_features.external_brakelight_supported = pFeature->brakelightSupported;
    lcsInit.feature.stp_features.sensor_calibration_supported = pFeature->pitchSupported;
#elif defined BILLY
    lcsInit.feature.light_type = BLE_LCS_LT_BIKE_LIGHT;
    lcsInit.feature.bk_features.main_beam_supported = pFeature->mainBeamSupported;
    lcsInit.feature.bk_features.extended_main_beam_supported = pFeature->extMainBeamSupported;
    lcsInit.feature.bk_features.high_beam_supported = pFeature->highBeamSupported;
    lcsInit.feature.bk_features.daylight_supported = pFeature->daylightSupported;
    lcsInit.feature.bk_features.external_taillight_supported = pFeature->taillightSupported;
    lcsInit.feature.bk_features.external_brakelight_supported = pFeature->brakelightSupported;
    lcsInit.feature.stp_features.sensor_calibration_supported = pFeature->brakelightSupported;
#endif // defined
    lcsInit.feature.cfg_features.mode_change_supported = 1;
    lcsInit.feature.cfg_features.mode_config_supported = 1;
    lcsInit.feature.cfg_features.mode_grouping_supported = 1;
    lcsInit.feature.cfg_features.preferred_mode_supported = 1;
    lcsInit.feature.cfg_features.temporary_mode_supported = 1;
    lcsInit.feature.stp_features.led_config_check_supported = 1;
    lcsInit.feature.stp_features.current_limitation_supported = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lf_attr_md.read_perm);
#ifdef DEBUG    // disable encryption for control point in debug mode
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lcp_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lcp_attr_md.write_perm);
#else
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&lcsInit.lcs_lcp_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&lcsInit.lcs_lcp_attr_md.write_perm);
#endif // DEBUG
    errCode = ble_lcs_init(&lcsGattsData, NUM_OF_CONNECTIONS, &lcsInit);
    APP_ERROR_CHECK(errCode);

#ifdef BTDEBUG
    ble_nus_init_t nusInit;
    memset(&nusInit, 0, sizeof(nusInit));
    nusInit.data_handler = &nusEventHandler;
    errCode = ble_nus_init(&nusGattsData, &nusInit);
    APP_ERROR_CHECK(errCode);
#endif // BTDEBUG
}

/** @brief Error handler for the advertising module
 */
void advertisingErrorHandler(uint32_t errCode)
{
    APP_ERROR_HANDLER(errCode);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertisingInit(void)
{
    uint32_t errCode;
    ble_uuid_t advUuid[2];
    ble_advdata_t advData, scanRsp;
    ble_adv_modes_config_t options;

    advUuid[0].uuid = BLE_UUID_LCS_SERVICE;
    advUuid[0].type = lcsGattsData.uuid_type;
    memset(&advData, 0, sizeof(ble_advdata_t));
    advData.name_type                     = BLE_ADVDATA_FULL_NAME;
    advData.include_appearance            = false;
    advData.flags                         = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advData.uuids_more_available.uuid_cnt = 1;
    advData.uuids_more_available.p_uuids  = &advUuid[0];

    advUuid[1].uuid = BLE_UUID_CGW_SERVICE;
    advUuid[1].type = cgwGattsData.uuid_type;
    memset(&scanRsp, 0, sizeof(ble_advdata_t));
    scanRsp.uuids_more_available.uuid_cnt = 1;
    scanRsp.uuids_more_available.p_uuids  = &advUuid[1];

    memset(&options, 0, sizeof(ble_adv_modes_config_t));
    options.ble_adv_fast_enabled    = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval   = MSEC_TO_UNITS(40, UNIT_0_625_MS);
    options.ble_adv_fast_timeout    = 180;
    options.ble_adv_slow_enabled    = BLE_ADV_SLOW_ENABLED;
    options.ble_adv_slow_interval   = MSEC_TO_UNITS(1000, UNIT_0_625_MS);
    options.ble_adv_slow_timeout    = 0;

    errCode = ble_advertising_init(&advData, &scanRsp, &options, NULL, advertisingErrorHandler);
    APP_ERROR_CHECK(errCode);

    errCode = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(errCode);
}

/** @brief Function to initialize the discovery module
 */
static void discoveryInit()
{
    uint32_t errCode;

    errCode = ble_db_discovery_init();
    APP_ERROR_CHECK(errCode);
}

/** @brief Event handler to process report from the YiaomiYi Remote Control
 * @note This remote sends an event with the physical state of the buttons
 *       every time the state changes, so it can be processed like a normal
 *       button (that is already debounced)
 *
 * @param[in] connHandle  the connection handle this event relates to
 * @param[in] buttonState the state of the buttons
 */
static void hidsCXiaomiEventHandler(ble_hids_c_t * pBleHidsC, ble_hids_c_evt_t * pEvt)
{
#define VOLUME_UP           (1<<6)
#define VOLUME_DOWN         (1<<7)
#define BUTTONPRESS_LONG    APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)

    if (pEvt->evt_type != BLE_HIDS_C_EVT_REPORT_NOTIFICATION || pEvt->params.report.index != 2)
        return;

    if (state.handler == NULL)
        return;

    btle_event_t evt;
    evt.connHandle = pBleHidsC->conn_handle;
    evt.evt = BTLE_EVT_HID;

    static uint8_t lastState;
    uint8_t buttonChange, buttonState = pEvt->params.report.p_report[0];

    buttonChange = lastState ^ buttonState;

    if (buttonChange & VOLUME_UP)
    {
        evt.subEvt.hid = buttonState & VOLUME_UP ? BTLE_EVT_HID_XIA_MAIN_PRESSED : BTLE_EVT_HID_XIA_MAIN_RELEASED;
        state.handler(&evt);
    }
    if (buttonChange & VOLUME_DOWN)
    {
        evt.subEvt.hid = buttonState & VOLUME_DOWN ? BTLE_EVT_HID_XIA_SEC_PRESSED : BTLE_EVT_HID_XIA_SEC_RELEASED;
        state.handler(&evt);
    }

    lastState = buttonState;

#undef VOLUME_UP
#undef VOLUME_DOWN
#undef BUTTONPRESS_LONG
}

/** @brief Event handler to process report from the R51 Remote Control
 * @note This remote sends an event with the usage type, so no button handling is nescessary
 *
 * @param[in] connHandle  the connection handle this event relates to
 * @param[in] usage the state of the buttons
 */
static void hidsCR51EventHandler(ble_hids_c_t * pBleHidsC, ble_hids_c_evt_t * pEvt)
{
#define USAGE_CONSUMER_CONTROL  0x01    // sent by click on the mode button
#define USAGE_PLAY_PAUSE        0xCD    // sent by click on the play/pause button
#define USAGE_VOLUME_INCREMENT  0xE9    // sent by single click (and when keep pressed) on the volume up button
#define USAGE_VOLUME_DECREMENT  0xEA    // sent by single click (and when keep pressed) on the volume down button
#define USAGE_SCAN_NEXT_TRACK   0xB5    // sent by double click on the volume up button
#define USAGE_SCAN_PREV_TRACK   0xB6    // sent by double click on the volume down button

    if (pEvt->evt_type != BLE_HIDS_C_EVT_REPORT_NOTIFICATION || pEvt->params.report.index != 2)
        return;

    btle_event_t evt;

    evt.connHandle = pBleHidsC->conn_handle;
    evt.evt = BTLE_EVT_HID;

    switch (pEvt->params.report.p_report[0])
    {
    case USAGE_PLAY_PAUSE:
        evt.subEvt.hid = BTLE_EVT_HID_R51_PLAYPAUSE;
        break;
    case USAGE_VOLUME_INCREMENT:
        evt.subEvt.hid = BTLE_EVT_HID_R51_VOL_UP;
        break;
    case USAGE_VOLUME_DECREMENT:
        evt.subEvt.hid = BTLE_EVT_HID_R51_VOL_DOWN;
        break;
    case USAGE_CONSUMER_CONTROL:
        evt.subEvt.hid = BTLE_EVT_HID_R51_CC;
        break;
    case USAGE_SCAN_NEXT_TRACK:
        evt.subEvt.hid = BTLE_EVT_HID_R51_NEXT_TRACK;
        break;
    case USAGE_SCAN_PREV_TRACK:
        evt.subEvt.hid = BTLE_EVT_HID_R51_PREV_TRACK;
        break;
    default:
        return; // no event for this cases
    }

    state.handler(&evt);

#undef USAGE_CONSUMER_CONTROL
#undef USAGE_PLAY_PAUSE
#undef USAGE_VOLUME_INCREMENT
#undef USAGE_VOLUME_DECREMENT
#undef USAGE_SCAN_NEXT_TRACK
#undef USAGE_SCAN_PREV_TRACK
}

/** @brief Event handler to process report from the Auviso Remote Control
 * @note This remote sends an event with the usage type, so no button handling is nescessary
 *
 * @param[in] connHandle  the connection handle this event relates to
 * @param[in] usage the state of the buttons
 */
static void hidsCAuvisoEventHandler(ble_hids_c_t * pBleHidsC, ble_hids_c_evt_t * pEvt)
{
#define USAGE_PLAY_PAUSE        0xCD    // sent by click on the play/pause button
#define USAGE_VOLUME_INCREMENT  0xE9    // sent by single click on the volume up button
#define USAGE_VOLUME_DECREMENT  0xEA    // sent by single click on the volume down button
#define USAGE_SCAN_NEXT_TRACK   0xB5    // sent by single click on the next track button
#define USAGE_SCAN_PREV_TRACK   0xB6    // sent by single click on the previous track button

    if (pEvt->evt_type != BLE_HIDS_C_EVT_REPORT_NOTIFICATION || pEvt->params.report.index != 2)
        return;

    btle_event_t evt;

    evt.connHandle = pBleHidsC->conn_handle;
    evt.evt = BTLE_EVT_HID;

    switch (pEvt->params.report.p_report[0])
    {
    case USAGE_PLAY_PAUSE:
        evt.subEvt.hid = BTLE_EVT_HID_AUV_PLAYPAUSE;
        break;
    case USAGE_VOLUME_INCREMENT:
        evt.subEvt.hid = BTLE_EVT_HID_AUV_VOL_UP;
        break;
    case USAGE_VOLUME_DECREMENT:
        evt.subEvt.hid = BTLE_EVT_HID_AUV_VOL_DOWN;
        break;
    case USAGE_SCAN_NEXT_TRACK:
        evt.subEvt.hid = BTLE_EVT_HID_AUV_NEXT_TRACK;
        break;
    case USAGE_SCAN_PREV_TRACK:
        evt.subEvt.hid = BTLE_EVT_HID_AUV_PREV_TRACK;
        break;
    default:
        return; // no event for this cases
    }

    state.handler(&evt);

#undef USAGE_PLAY_PAUSE
#undef USAGE_VOLUME_INCREMENT
#undef USAGE_VOLUME_DECREMENT
#undef USAGE_SCAN_NEXT_TRACK
#undef USAGE_SCAN_PREV_TRACK
}

void lcsCEventHandler(ble_lcs_c_t * pBleLcsC, ble_lcs_c_evt_t * pEvt)
{
    switch (pEvt->evt_type)
    {
    case BLE_LCS_C_EVT_DISCOVERY_COMPLETE:
        //ble_lcs_c_lcm_notif_enable(pBleLcsC, pEvt->p_server->conn_handle, true);
        ble_lcs_c_cp_indic_enable(pBleLcsC, pEvt->p_server->conn_handle, true);
        break;

    case BLE_LCS_C_EVT_MEASUREMENT_NOTIFY:
        if (state.handler)
        {
            btle_event_t evt = { .lcsmEventParams.temperature = INT8_MAX, .lcsmEventParams.pitch = INT8_MAX};
            evt.evt = BTLE_EVT_LCS;
            evt.subEvt.lcs = BTLE_EVT_LCS_MEAS_RECEIVED;
            evt.connHandle = pEvt->p_server->conn_handle;
#ifdef HELENA
            evt.lcsmEventParams.mode.setup.flood             = pEvt->data.measurement.hlmt.mode.setup.flood;
            evt.lcsmEventParams.mode.setup.spot              = pEvt->data.measurement.hlmt.mode.setup.spot;
            evt.lcsmEventParams.mode.setup.pitchCompensation = pEvt->data.measurement.hlmt.mode.setup.pitchCompensation;
            evt.lcsmEventParams.mode.setup.cloned            = pEvt->data.measurement.hlmt.mode.setup.cloned;
            evt.lcsmEventParams.mode.setup.taillight         = pEvt->data.measurement.hlmt.mode.setup.taillight;
            evt.lcsmEventParams.mode.setup.brakelight        = pEvt->data.measurement.hlmt.mode.setup.brakelight;
            if (pEvt->data.measurement.flags.hlmt.intensity_present)
            {
                if (pEvt->data.measurement.hlmt.mode.setup.pitchCompensation)
                    evt.lcsmEventParams.mode.illuminanceInLux = pEvt->data.measurement.hlmt.mode.intensity;
                else
                    evt.lcsmEventParams.mode.intensityInPercent = pEvt->data.measurement.hlmt.mode.intensity;
            }
            if (pEvt->data.measurement.flags.hlmt.flood_status_present)
            {
                evt.lcsmEventParams.statusFlood.overcurrent    = pEvt->data.measurement.hlmt.flood_status.overcurrent;
                evt.lcsmEventParams.statusFlood.inputVoltage   = pEvt->data.measurement.hlmt.flood_status.voltage;
                evt.lcsmEventParams.statusFlood.temperature    = pEvt->data.measurement.hlmt.flood_status.temperature;
                evt.lcsmEventParams.statusFlood.dutyCycleLimit = pEvt->data.measurement.hlmt.flood_status.dutycycle;
            }
            if (pEvt->data.measurement.flags.hlmt.spot_status_present)
            {
                evt.lcsmEventParams.statusSpot.overcurrent    = pEvt->data.measurement.hlmt.spot_status.overcurrent;
                evt.lcsmEventParams.statusSpot.inputVoltage   = pEvt->data.measurement.hlmt.spot_status.voltage;
                evt.lcsmEventParams.statusSpot.temperature    = pEvt->data.measurement.hlmt.spot_status.temperature;
                evt.lcsmEventParams.statusSpot.dutyCycleLimit = pEvt->data.measurement.hlmt.spot_status.dutycycle;
            }
            if (pEvt->data.measurement.flags.hlmt.flood_power_present)
                evt.lcsmEventParams.powerFlood = pEvt->data.measurement.hlmt.flood_power;
            if (pEvt->data.measurement.flags.hlmt.spot_power_present)
                evt.lcsmEventParams.powerSpot = pEvt->data.measurement.hlmt.spot_power;
            if (pEvt->data.measurement.flags.hlmt.temperature_present)
                evt.lcsmEventParams.temperature = pEvt->data.measurement.temperature;
            if (pEvt->data.measurement.flags.hlmt.input_voltage_present)
                evt.lcsmEventParams.inputVoltage = pEvt->data.measurement.input_voltage;
            if (pEvt->data.measurement.flags.hlmt.pitch_present)
                evt.lcsmEventParams.pitch = pEvt->data.measurement.pitch;
            if (pEvt->data.measurement.flags.hlmt.battery_soc_present)
                evt.lcsmEventParams.batterySoc = pEvt->data.measurement.battery_soc;
            if (pEvt->data.measurement.flags.hlmt.taillight_power_present)
                evt.lcsmEventParams.powerTaillight = pEvt->data.measurement.taillight_power;
#elif defined BILLY
            evt.lcsmEventParams.mode.setup.mainBeam          = pEvt->data.measurement.bk.mode.setup.main_beam;
            evt.lcsmEventParams.mode.setup.extendedMainBeam  = pEvt->data.measurement.bk.mode.setup.extended_main_beam;
            evt.lcsmEventParams.mode.setup.highBeam          = pEvt->data.measurement.bk.mode.setup.high_beam;
            evt.lcsmEventParams.mode.setup.daylight          = pEvt->data.measurement.bk.mode.setup.daylight;
            evt.lcsmEventParams.mode.setup.taillight         = pEvt->data.measurement.bk.mode.setup.taillight;
            evt.lcsmEventParams.mode.setup.brakelight        = pEvt->data.measurement.bk.mode.setup.brakelight;
            if (pEvt->data.measurement.flags.bk.intensity_present)
            {
                evt.lcsmEventParams.mode.mainBeamIntensityInPercent = pEvt->data.measurement.bk.mode.main_beam_intensity;
                evt.lcsmEventParams.mode.highBeamIntensityInPercent = pEvt->data.measurement.bk.mode.high_beam_intensity;
            }
            if (pEvt->data.measurement.flags.bk.main_beam_status_present)
            {
                evt.lcsmEventParams.statusMainBeam.overcurrent    = pEvt->data.measurement.bk.main_beam_status.overcurrent;
                evt.lcsmEventParams.statusMainBeam.inputVoltage   = pEvt->data.measurement.bk.main_beam_status.voltage;
                evt.lcsmEventParams.statusMainBeam.temperature    = pEvt->data.measurement.bk.main_beam_status.temperature;
                evt.lcsmEventParams.statusMainBeam.dutyCycleLimit = pEvt->data.measurement.bk.main_beam_status.dutycycle;
            }
            if (pEvt->data.measurement.flags.bk.high_beam_status_present)
            {
                evt.lcsmEventParams.statusHighBeam.overcurrent    = pEvt->data.measurement.bk.high_beam_status.overcurrent;
                evt.lcsmEventParams.statusHighBeam.inputVoltage   = pEvt->data.measurement.bk.high_beam_status.voltage;
                evt.lcsmEventParams.statusHighBeam.temperature    = pEvt->data.measurement.bk.high_beam_status.temperature;
                evt.lcsmEventParams.statusHighBeam.dutyCycleLimit = pEvt->data.measurement.bk.high_beam_status.dutycycle;
            }
            if (pEvt->data.measurement.flags.bk.main_beam_power_present)
                evt.lcsmEventParams.powerMainBeam = pEvt->data.measurement.bk.main_beam_power;
            if (pEvt->data.measurement.flags.bk.high_beam_power_present)
                evt.lcsmEventParams.powerHighBeam = pEvt->data.measurement.bk.high_beam_power;
            if (pEvt->data.measurement.flags.bk.temperature_present)
                evt.lcsmEventParams.temperature = pEvt->data.measurement.temperature;
            if (pEvt->data.measurement.flags.bk.input_voltage_present)
                evt.lcsmEventParams.inputVoltage = pEvt->data.measurement.input_voltage;
            if (pEvt->data.measurement.flags.bk.pitch_present)
                evt.lcsmEventParams.pitch = pEvt->data.measurement.pitch;
            if (pEvt->data.measurement.flags.bk.battery_soc_present)
                evt.lcsmEventParams.batterySoc = pEvt->data.measurement.battery_soc;
            if (pEvt->data.measurement.flags.bk.taillight_power_present)
                evt.lcsmEventParams.powerTaillight = pEvt->data.measurement.taillight_power;
#endif // BILLY

            state.handler(&evt);
        }
        break;

    case BLE_LCS_C_EVT_CONTROL_POINT_INDIC:
        break;

    default:
        break;
    }
}

void lcsCErrorHandler(ble_lcs_c_t * pBleLcsC, ble_lcs_c_error_evt_t * pEvt)
{
    switch (pEvt->evt_type)
    {
    case BLE_LCS_C_ERROR_EVT_LCM_NOTIFY:
    case BLE_LCS_C_ERROR_EVT_LCCP_IND:
    case BLE_LCS_C_ERROR_EVT_LCCP_WRITE:
        if (pEvt->data.gatt_status != BLE_GATT_STATUS_ATTERR_CPS_PROC_ALR_IN_PROG &&  // if a control point operation is already in progress
            pEvt->data.gatt_status != BLE_GATT_STATUS_ATTERR_INSUF_AUTHENTICATION)    // if device is not bonded
            APP_ERROR_CHECK(pEvt->data.gatt_status);
        break;
    default:
        break;
    }
}

/** @brief Function to initialize the GATT Collector services
 */
static void serviceCollectorInit()
{
    uint32_t errCode;

    // no hids initiation here, this is done when connecting to a remote

    ble_lcs_c_init_t lcsCInit;
    lcsCInit.evt_handler = lcsCEventHandler;
    lcsCInit.error_handler = lcsCErrorHandler;
    errCode = ble_lcs_c_init(&lcsGattcData, COUNT_OF(lcsGattcData_server_data), &lcsCInit);
    APP_ERROR_CHECK(errCode);
}

/** @brief Error handler for the scanning module
 */
static void scanningErrorHandler(uint32_t errCode)
{
    APP_ERROR_CHECK(errCode);
}

/** @brief Function to parse an advertising report and check if it is a Light Control device
 *
 * @param[in] pAdvData Advertising report to parse
 * @return    CENTRAL_LIGHT if a Light Control service uuid was found, otherwise CENTRAL_NONE
 */
static centralDevice_t isLcsDevice(const ble_gap_evt_adv_report_t * pAdvData)
{
    uint_fast8_t index = 0;

    while (index < pAdvData->dlen)
    {
        uint8_t length = pAdvData->data[index];
        uint8_t type   = pAdvData->data[index+1];

        if (type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE ||
            type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
        {
            index += 2;
            for (uint_fast8_t i = 0; i < length/UUID128_SIZE; i++)
            {
                ble_uuid128_t lcsServiceUuid;
                memcpy(&lcsServiceUuid, &ble_lcs_c_base_uuid, sizeof(lcsServiceUuid));
                lcsServiceUuid.uuid128[13] = (BLE_UUID_LCS_SERVICE >> 8) & 0xFF;
                lcsServiceUuid.uuid128[12] = BLE_UUID_LCS_SERVICE & 0xFF;
                if (memcmp(lcsServiceUuid.uuid128, &pAdvData->data[index + i * UUID128_SIZE], sizeof(lcsServiceUuid.uuid128)) == 0)
                    return CENTRAL_LIGHT;
            }
            return CENTRAL_NONE;
        }
        index += length + 1;
    }
    return CENTRAL_NONE;
}


/** @brief Scanning module event handler
 *
 * @details   This handler processes the advertising report delivered from the
 *            scanning module and prepares a whitelist when requested.
 *
 * @param[in] pScanEvt  Scan event structure
 */
static void scanningEventHandler(const ble_scan_evt_t * const pScanEvt)
{
    btle_event_t evt;
    uint32_t errCode;

    evt.evt = BTLE_EVT_CONNECTION;
    evt.connHandle = BLE_CONN_HANDLE_INVALID;   // no connection for scanning events

    switch (pScanEvt->ble_scan_event)
    {
    case BLE_SCAN_EVT_IDLE:
    case BLE_SCAN_EVT_PAUSE:
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_SEARCH_OFF;
        state.scanMode = BLE_SCAN_MODE_IDLE;
        break;
    case BLE_SCAN_EVT_FAST:
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_SEARCH_FOR_NEW;
        state.scanMode = BLE_SCAN_MODE_FAST;
        break;
    case BLE_SCAN_EVT_FAST_WHITELIST:
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_SEARCH_LOWLATENCY;
        state.scanMode = BLE_SCAN_MODE_FAST;
        break;
    case BLE_SCAN_EVT_SLOW:
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_SEARCH_FOR_NEW;
        state.scanMode = BLE_SCAN_MODE_SLOW;
        errCode = ble_scanning_start(BLE_SCAN_MODE_SLOW);
        APP_ERROR_CHECK(errCode);
        break;
    case BLE_SCAN_EVT_SLOW_WHITELIST:
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_SEARCH_LOWPOWER;
        state.scanMode = BLE_SCAN_MODE_SLOW;
        if (state.scanConfig == BTLE_SCAN_MODE_LOW_LATENCY)
        {
            errCode = ble_scanning_start(BLE_SCAN_MODE_FAST);
            APP_ERROR_CHECK(errCode);
        }
        break;
    case BLE_SCAN_EVT_ADV_REPORT_RECEIVED:
    {
        // check if device is compatible
        centralDevice_t deviceType;

        ble_hids_c_init_t hidsCInit;
        ble_hids_c_hc_device_t hidDevice = ble_hids_c_hc_is_device(pScanEvt->p_ble_adv_report);

        switch (hidDevice)
        {
        case BLE_HIDS_C_HC_DEVICE_XIAOMI:
            deviceType = CENTRAL_XIAOMI;
            hidsCInit.evt_handler = hidsCXiaomiEventHandler;
            break;
        case BLE_HIDS_C_HC_DEVICE_R51:
            deviceType = CENTRAL_R51;
            hidsCInit.evt_handler = hidsCR51EventHandler;
            break;
        case BLE_HIDS_C_HC_DEVICE_AUVISO:
            deviceType = CENTRAL_AUVISO;
            hidsCInit.evt_handler = hidsCAuvisoEventHandler;
            break;
        default:
            deviceType = isLcsDevice(pScanEvt->p_ble_adv_report);
            break;
        }

        if (deviceType != CENTRAL_NONE)
        {
            static const ble_gap_scan_params_t scanParams =
            {
                .active      = 0,
                .selective   = 0,
                .p_whitelist = NULL,
                .interval    = MSEC_TO_UNITS(60, UNIT_0_625_MS),
                .window      = MSEC_TO_UNITS(30, UNIT_0_625_MS),
                .timeout     = 30,
            };
            static const ble_gap_conn_params_t connParams =
            {
                MSEC_TO_UNITS(10, UNIT_1_25_MS),
                MSEC_TO_UNITS(50, UNIT_1_25_MS),
                0,
                MSEC_TO_UNITS(4000, UNIT_10_MS)
            };
            // stop scanning
            errCode = ble_scanning_start(BLE_SCAN_MODE_IDLE);
            APP_ERROR_CHECK(errCode);
            // and connect to device
            errCode = sd_ble_gap_connect(&pScanEvt->p_ble_adv_report->peer_addr, &scanParams, &connParams);
            APP_ERROR_CHECK(errCode);

            state.centralDevice = deviceType;   /// TODO: reset to CENTRAL_NONE if connection attempt fails

            if (hidDevice != BLE_HIDS_C_HC_DEVICE_UNKNOWN)
            {
                errCode = ble_hids_c_hc_init(&hidsGattcData, &hidsCInit, hidDevice);
                APP_ERROR_CHECK(errCode);
            }
        }
    }   return;//break;
    case BLE_SCAN_EVT_WHITELIST_REQUEST:
    {
        ble_gap_irk_t * irks[1];
        ble_gap_addr_t * addrs[1];
        pm_peer_id_t peerIds[1];
        uint_fast8_t peerIdsCnt = 0;
        ble_gap_whitelist_t whitelist = {.pp_irks = irks, .pp_addrs = addrs};
        pm_peer_id_t peerId = nextCentralPeerIdGet(PM_PEER_ID_INVALID);
        while(peerId != PM_PEER_ID_INVALID && peerIdsCnt < 1)
        {
            peerIds[peerIdsCnt++] = peerId;
            peerId = nextCentralPeerIdGet(peerId);
        }
        errCode = pm_wlist_create(peerIds, peerIdsCnt, &whitelist);
        APP_ERROR_CHECK(errCode);
        errCode = ble_scanning_whitelist_reply(&whitelist);
        APP_ERROR_CHECK(errCode);
    }   return;//break;
    default:
        return;//break;
    }

    // relay scanning event to application
    if (state.handler != NULL)
        state.handler(&evt);
}

/** @brief Function to initialize scanning module
 *
 * @details This function initialize the scanning module and starts scanning.
 */
static void scanningInit()
{
    uint32_t errCode;
    ble_scan_modes_config_t options = {0};

    options.ble_scan_active_scanning    = true;
    options.ble_scan_whitelist_enabled  = true;
    options.ble_scan_fast_enabled       = true;
    options.ble_scan_fast_interval      = MSEC_TO_UNITS(60, UNIT_0_625_MS);
    options.ble_scan_fast_window        = MSEC_TO_UNITS(30, UNIT_0_625_MS);
    options.ble_scan_fast_timeout       = 180;
    options.ble_scan_slow_enabled       = true;
    options.ble_scan_slow_interval      = MSEC_TO_UNITS(1280, UNIT_0_625_MS);
    options.ble_scan_slow_window        = MSEC_TO_UNITS(11.25, UNIT_0_625_MS);
    options.ble_scan_slow_timeout       = 0;

    errCode = ble_scanning_init(&options, scanningEventHandler, scanningErrorHandler);
    APP_ERROR_CHECK(errCode);

    errCode = ble_scanning_start(BLE_SCAN_MODE_FAST);
    APP_ERROR_CHECK(errCode);
}

/* Public functions ----------------------------------------------------------*/
void btle_StackInit()
{
    uint32_t errCode;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, NULL);

    // Enable BLE stack.
    ble_enable_params_t bleEnableParams;
    memset(&bleEnableParams, 0, sizeof(ble_enable_params_t));
    bleEnableParams.gatts_enable_params.attr_tab_size   = 0x390; //BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    bleEnableParams.gatts_enable_params.service_changed = true;
    errCode = sd_ble_enable(&bleEnableParams);
    APP_ERROR_CHECK(errCode);

    // Register with the SoftDevice handler module for BLE events.
    errCode = softdevice_ble_evt_handler_set(bleEvtDispatch);
    APP_ERROR_CHECK(errCode);

    // Register with the SoftDevice handler module for System events.
    errCode = softdevice_sys_evt_handler_set(sysEvtDispatch);
    APP_ERROR_CHECK(errCode);

    // enable workaround for Rev 2 ics
    ble_opt_t cpuBlockingEnabled;
    cpuBlockingEnabled.common_opt.radio_cpu_mutex.enable = 1;
    errCode = sd_ble_opt_set(BLE_COMMON_OPT_RADIO_CPU_MUTEX, &cpuBlockingEnabled);
    APP_ERROR_CHECK(errCode);

    // set maximum output power
    errCode = sd_ble_gap_tx_power_set(4);
    APP_ERROR_CHECK(errCode);
}

void btle_Init(bool deleteBonds, char const* pDrvRev, btle_lcsFeature_t* pFeature, btle_eventHandler_t pEvtHandler)
{
    state.handler = pEvtHandler;
    state.connHandleCentral = BLE_CONN_HANDLE_INVALID;
    state.connHandlePeriph = BLE_CONN_HANDLE_INVALID;

    peerManagerInit(deleteBonds);
    gapParamsInit();
    connParamsInit();
    servicesInit(pDrvRev, pFeature);
    advertisingInit();
    discoveryInit();
    serviceCollectorInit();
    scanningInit();
}

void btle_SetScanConfig(btle_scanMode_t newScanConfig)
{
    uint32_t errCode;

    state.scanConfig = newScanConfig;
    if (state.scanConfig == BTLE_SCAN_MODE_LOW_LATENCY && state.scanMode == BLE_SCAN_MODE_SLOW)
    {
        errCode = ble_scanning_start(BLE_SCAN_MODE_FAST);
        APP_ERROR_CHECK(errCode);
    }
    if (state.scanConfig == BTLE_SCAN_MODE_LOW_POWER && state.scanMode == BLE_SCAN_MODE_FAST)
    {
        errCode = ble_scanning_start(BLE_SCAN_MODE_SLOW);
        APP_ERROR_CHECK(errCode);
    }
}

uint32_t btle_ComGatewayCheck(const com_MessageStruct * pMessageIn)
{
    if (pMessageIn == NULL)
        return NRF_ERROR_NULL;

    if (cgwGattsData.is_notification_enabled == false)
        return NRF_ERROR_INVALID_STATE;

    if ((pMessageIn->Identifier == 0x64 || pMessageIn->Identifier == 0x6C || pMessageIn->Identifier == 0x26) &&
        ((pMessageIn->Control & 0xF0) == 0))
        return ble_cgw_send_message(&cgwGattsData, pMessageIn);
    return NRF_SUCCESS;
}

uint32_t btle_UpdateLcsMeasurements(const btle_lcsMeasurement_t * pData)
{
    static uint32_t lastTimeSent;
    ble_lcs_lm_t notifyData;
    uint32_t timestamp, errCode = NRF_SUCCESS;
    bool isEnabled = false;

    if (pData == NULL)
        return NRF_ERROR_NULL;

    for (uint_fast8_t i = 0; i < COUNT_OF(lcsGattsData_client_data); i++)
    {
        if (lcsGattsData_client_data[i].conn_handle != BLE_CONN_HANDLE_INVALID &&
            lcsGattsData_client_data[i].is_lm_notfy_enabled == true)
        {
            isEnabled = true;
            break;
        }
    }

    if (!isEnabled)
        return NRF_ERROR_INVALID_STATE;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, lastTimeSent, &timestamp);

    if (timestamp < LCS_NOTIFY_PERIOD)
        return NRF_SUCCESS;

    memset(&notifyData, 0, sizeof(ble_lcs_lm_t));
#ifdef HELENA
    notifyData.light_type = BLE_LCS_LT_HELMET_LIGHT;
    notifyData.hlmt.mode.setup.flood = pData->mode.setup.flood ? 1 : 0;
    notifyData.hlmt.mode.setup.spot = pData->mode.setup.spot ? 1 : 0;
    notifyData.hlmt.mode.setup.pitchCompensation = pData->mode.setup.pitchCompensation ? 1 : 0;
    notifyData.hlmt.mode.setup.cloned = pData->mode.setup.cloned ? 1 : 0;
    notifyData.hlmt.mode.setup.taillight = pData->mode.setup.taillight ? 1 : 0;
    notifyData.hlmt.mode.setup.brakelight = pData->mode.setup.brakelight ? 1 : 0;
    if (pData->mode.setup.flood || pData->mode.setup.spot)
    {
        notifyData.flags.hlmt.intensity_present = 1;
        notifyData.hlmt.mode.intensity = pData->mode.intensityInPercent;  // don't care if pitch compensated or not, variables share same location
    }
    if (pData->mode.setup.flood)
    {
        notifyData.flags.hlmt.flood_power_present = 1;
        notifyData.flags.hlmt.flood_status_present = 1;
        notifyData.hlmt.flood_power = pData->powerFlood;
        notifyData.hlmt.flood_status.overcurrent = pData->statusFlood.overcurrent ? 1 : 0;
        notifyData.hlmt.flood_status.voltage = pData->statusFlood.inputVoltage ? 1 : 0;
        notifyData.hlmt.flood_status.temperature = pData->statusFlood.temperature ? 1 : 0;
        notifyData.hlmt.flood_status.dutycycle = pData->statusFlood.dutyCycleLimit ? 1 : 0;
    }
    if (pData->mode.setup.spot)
    {
        notifyData.flags.hlmt.spot_power_present = 1;
        notifyData.flags.hlmt.spot_status_present = 1;
        notifyData.hlmt.spot_power = pData->powerSpot;
        notifyData.hlmt.spot_status.overcurrent = pData->statusSpot.overcurrent ? 1 : 0;
        notifyData.hlmt.spot_status.voltage = pData->statusSpot.inputVoltage ? 1 : 0;
        notifyData.hlmt.spot_status.temperature = pData->statusSpot.temperature ? 1 : 0;
        notifyData.hlmt.spot_status.dutycycle = pData->statusSpot.dutyCycleLimit ? 1 : 0;
    }
    if (pData->temperature >= -40 && pData->temperature <= 85)
    {
        notifyData.flags.hlmt.temperature_present = 1;
        notifyData.temperature = pData->temperature;
    }
    if (pData->inputVoltage != 0)
    {
        notifyData.flags.hlmt.input_voltage_present = 1;
        notifyData.input_voltage = pData->inputVoltage;;
    }
    if (pData->pitch >= -90 && pData->pitch <= 90)
    {
        notifyData.flags.hlmt.pitch_present = 1;
        notifyData.pitch = pData->pitch;
    }
    if (pData->batterySoc <= 100)
    {
        notifyData.flags.hlmt.battery_soc_present = 1;
        notifyData.battery_soc = pData->batterySoc;
    }
    if (pData->powerTaillight > 0)
    {
        notifyData.flags.hlmt.taillight_power_present = 1;
        notifyData.taillight_power = pData->powerTaillight;
    }
#elif defined BILLY
    notifyData.light_type = BLE_LCS_LT_BIKE_LIGHT;
    notifyData.bk.mode.setup.main_beam = pData->mode.setup.mainBeam ? 1 : 0;
    notifyData.bk.mode.setup.extended_main_beam = pData->mode.setup.extendedMainBeam ? 1 : 0;
    notifyData.bk.mode.setup.high_beam = pData->mode.setup.highBeam ? 1 : 0;
    notifyData.bk.mode.setup.daylight = pData->mode.setup.daylight ? 1 : 0;
    notifyData.bk.mode.setup.taillight = pData->mode.setup.taillight ? 1 : 0;
    notifyData.bk.mode.setup.brakelight = pData->mode.setup.brakelight ? 1 : 0;
    if (pData->mode.setup.mainBeam || pData->mode.setup.extendedMainBeam || pData->mode.setup.highBeam)
    {
        notifyData.flags.bk.intensity_present = 1;
        notifyData.bk.mode.main_beam_intensity = pData->mode.mainBeamIntensityInPercent;
        notifyData.bk.mode.high_beam_intensity = pData->mode.highBeamIntensityInPercent;
    }
    if (pData->mode.setup.mainBeam || pData->mode.setup.extendedMainBeam)
    {
        notifyData.flags.bk.main_beam_power_present        = 1;
        notifyData.flags.bk.main_beam_status_present       = 1;
        notifyData.bk.main_beam_power              = pData->powerMainBeam;
        notifyData.bk.main_beam_status.overcurrent = pData->statusMainBeam.overcurrent ? 1 : 0;
        notifyData.bk.main_beam_status.voltage     = pData->statusMainBeam.inputVoltage ? 1 : 0;
        notifyData.bk.main_beam_status.temperature = pData->statusMainBeam.temperature ? 1 : 0;
        notifyData.bk.main_beam_status.dutycycle   = pData->statusMainBeam.dutyCycleLimit ? 1 : 0;
    }
    if (pData->mode.setup.highBeam)
    {
        notifyData.flags.bk.high_beam_power_present   = 1;
        notifyData.flags.bk.high_beam_status_present  = 1;
        notifyData.bk.high_beam_power              = pData->powerHighBeam;
        notifyData.bk.high_beam_status.overcurrent = pData->statusHighBeam.overcurrent ? 1 : 0;
        notifyData.bk.high_beam_status.voltage     = pData->statusHighBeam.inputVoltage ? 1 : 0;
        notifyData.bk.high_beam_status.temperature = pData->statusHighBeam.temperature ? 1 : 0;
        notifyData.bk.high_beam_status.dutycycle   = pData->statusHighBeam.dutyCycleLimit ? 1 : 0;
    }
    if (pData->temperature >= -40 && pData->temperature <= 85)
    {
        notifyData.flags.bk.temperature_present = 1;
        notifyData.temperature = pData->temperature;
    }
    if (pData->inputVoltage != 0)
    {
        notifyData.flags.bk.input_voltage_present = 1;
        notifyData.input_voltage = pData->inputVoltage;;
    }
    if (pData->pitch >= -90 && pData->pitch <= 90)
    {
        notifyData.flags.bk.pitch_present = 1;
        notifyData.pitch = pData->pitch;
    }
    if (pData->batterySoc <= 100)
    {
        notifyData.flags.hlmt.battery_soc_present = 1;
        notifyData.battery_soc = pData->batterySoc;
    }
    if (pData->powerTaillight > 0)
    {
        notifyData.flags.hlmt.taillight_power_present = 1;
        notifyData.taillight_power = pData->powerTaillight;
    }
#endif // defined


    (void)app_timer_cnt_get(&lastTimeSent);

    for (uint_fast8_t i = 0; i < COUNT_OF(lcsGattsData_client_data); i++)
    {
        if (lcsGattsData_client_data[i].conn_handle != BLE_CONN_HANDLE_INVALID &&
            lcsGattsData_client_data[i].is_lm_notfy_enabled == true)
        {
            if (errCode == NRF_SUCCESS)
                errCode = ble_lcs_light_measurement_send(&lcsGattsData, lcsGattsData_client_data[i].conn_handle, &notifyData);
            else
                (void)ble_lcs_light_measurement_send(&lcsGattsData, lcsGattsData_client_data[i].conn_handle, &notifyData);
        }
    }

    return errCode;
}

uint32_t btle_SendEventResponse(const btle_LcscpEventResponse_t *pRsp, uint16_t connHandle)
{
    ble_lcs_ctrlpt_rsp_t rsp;

    if (pRsp == NULL)
        return NRF_ERROR_NULL;

    rsp.opcode = pRsp->evt - BTLE_EVT_LCSCP_REQ_MODE_CNT + BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT;
    rsp.status = pRsp->retCode;

    switch (pRsp->evt)
    {
    case BTLE_EVT_LCSCP_REQ_MODE_CNT:
        rsp.params.mode_cnt = pRsp->responseParams.modeCnt;
        break;
    case BTLE_EVT_LCSCP_REQ_GROUP_CNT:
        rsp.params.group_config.num_of_groups = pRsp->responseParams.groupCfg.numOfModes;
        rsp.params.group_config.p_list_of_modes_per_group = pRsp->responseParams.groupCfg.pNumOfModesPerGroup;
        break;
    case BTLE_EVT_LCSCP_REQ_MODE_CONFIG:
        rsp.params.mode_config_list.p_list_hlmt = (ble_lcs_hlmt_mode_t*)pRsp->responseParams.modeList.pList;
        rsp.params.mode_config_list.num_of_entries = pRsp->responseParams.modeList.listEntries;
        break;
    case BTLE_EVT_LCSCP_REQ_LED_CONFIG:
    case BTLE_EVT_LCSCP_CHECK_LED_CONFIG:
#ifdef HELENA
        rsp.params.led_config.cnt_flood = pRsp->responseParams.ledConfig.floodCnt;
        rsp.params.led_config.cnt_spot = pRsp->responseParams.ledConfig.spotCnt;
#elif defined BILLY
        rsp.params.led_config.cnt_main_beam = pRsp->responseParams.ledConfig.mainBeamCnt;
        rsp.params.led_config.cnt_high_beam = pRsp->responseParams.ledConfig.highBeamCnt;
#endif // defined
        break;
    case BTLE_EVT_LCSCP_REQ_SENS_OFFSET:
    case BTLE_EVT_LCSCP_CALIB_SENS_OFFSET:
        rsp.params.sens_offset.x = pRsp->responseParams.sensOffset.x;
        rsp.params.sens_offset.y = pRsp->responseParams.sensOffset.y;
        rsp.params.sens_offset.z = pRsp->responseParams.sensOffset.z;
        break;
    case BTLE_EVT_LCSCP_REQ_LIMITS:
#ifdef HELENA
        rsp.params.current_limits.flood = pRsp->responseParams.currentLimits.floodInPercent;
        rsp.params.current_limits.spot = pRsp->responseParams.currentLimits.spotInPercent;
#elif defined BILLY
        rsp.params.current_limits.main_beam = pRsp->responseParams.currentLimits.mainBeamInPercent;
        rsp.params.current_limits.high_beam = pRsp->responseParams.currentLimits.highBeamInPercent;
#endif // BILLY
        break;
    case BTLE_EVT_LCSCP_REQ_PREF_MODE:
        rsp.params.pref_mode = pRsp->responseParams.prefMode;
        break;
    case BTLE_EVT_LCSCP_REQ_TEMP_MODE:
        rsp.params.pref_mode = pRsp->responseParams.tempMode;
        break;
    default:
        break;
    }

    return ble_lcs_ctrlpt_mode_resp(&lcsGattsData.ctrl_pt, connHandle, &rsp);
}

uint32_t btle_DeleteCentralBonds()
{
    uint32_t errCode;

    errCode = ble_scanning_pause(true);
    if (errCode != NRF_SUCCESS)
    {
        APP_ERROR_CHECK(errCode);
        return errCode;
    }

    if (state.connHandleCentral != BLE_CONN_HANDLE_INVALID)
    {
        errCode = sd_ble_gap_disconnect(state.connHandleCentral, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (errCode != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(errCode);
            return errCode;
        }
    }

    deleteOtherCentrals(PM_PEER_ID_INVALID);

    errCode = ble_scanning_pause(false);
    if (errCode != NRF_SUCCESS)
    {
        APP_ERROR_CHECK(errCode);
        return errCode;
    }

    return ble_scanning_start(BLE_SCAN_MODE_IDLE);
}

uint32_t btle_SearchRemoteDevice()
{
    if (state.connHandleCentral != BLE_CONN_HANDLE_INVALID)
    {
        uint32_t errCode;
        errCode = sd_ble_gap_disconnect(state.connHandleCentral, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(errCode);
    }
    return ble_scanning_start_without_whitelist(BLE_SCAN_MODE_FAST);
}

uint32_t btle_AllowPeripheralBonding(bool allow)
{
    if (state.allowPeripheralBonding == allow)
        return NRF_ERROR_INVALID_STATE;
    state.allowPeripheralBonding = allow;
    return NRF_SUCCESS;
}

uint32_t btle_SetMode(uint8_t mode, uint16_t connHandle)
{
    /// TODO: check if indication of previous change has been received, if not schedule command

    ble_lcs_c_cp_write_t cmd;
    uint32_t errCode = NRF_SUCCESS;

    cmd.command = BLE_LCS_C_CP_CMD_SET_MODE;
    cmd.params.mode_to_set = mode;

    if (connHandle == BTLE_CONN_HANDLE_INVALID)
        errCode = NRF_ERROR_INVALID_PARAM;
    else if (connHandle == BTLE_CONN_HANDLE_ALL)
    {
        for (uint_fast8_t i = 0; i < COUNT_OF(lcsGattcData_server_data); i++)
        {
            if (lcsGattcData_server_data[i].conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                if (errCode == NRF_SUCCESS)
                    errCode = ble_lcs_c_cp_write(&lcsGattcData, lcsGattcData_server_data[i].conn_handle, &cmd);
                else
                    (void)ble_lcs_c_cp_write(&lcsGattcData, lcsGattcData_server_data[i].conn_handle, &cmd);
            }
        }
    }
    else
        errCode = ble_lcs_c_cp_write(&lcsGattcData, connHandle, &cmd);

    return errCode;
}

#ifdef BTDEBUG
uint32_t btle_SendNusString(uint8_t* pData, uint16_t len)
{
    return ble_nus_string_send(&nusGattsData, pData, len);
}
#endif

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEAF, line_num, p_file_name);
}

/**END OF FILE*****************************************************************/



