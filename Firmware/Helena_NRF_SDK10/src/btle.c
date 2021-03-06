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
#include "ble_lcs_c.h"
#include "ble_advertising.h"
#include "ble_db_discovery.h"
#include "ble_scanning.h"
#include "peer_manager.h"
#include "fstorage.h"
#include "fds.h"
#include "version.h"

#ifdef HELENA_DEBUG_FIELD_TESTING
#include "debug.h"
#endif

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    btle_eventHandler_t handler;    // event handler given at initialization
    btle_scanMode_t scanConfig;     // current scan configuration
    ble_scan_mode_t scanMode;       // current scan mode
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

#define DEVICE_NAME                     "Helena"
#define DEVICE_APPEARANCE               BLE_APPEARANCE_GENERIC_CYCLING

#define LCS_NOTIFY_PERIOD               APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

/* Private variables ---------------------------------------------------------*/
static ble_cgw_t cgwGattsData;                  // gatt server handles for the com gateway service
BLE_LCS_DEF(lcsGattsData, NUM_OF_CONNECTIONS);  // gatt server handles for the light control service

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

        // start database discovery
        if (discDatabase.discovery_in_progress == false)
        {
            memset(&discDatabase, 0, sizeof(discDatabase));
            errCode = ble_db_discovery_start(&discDatabase, state.connHandleCentral);
            APP_ERROR_CHECK(errCode);
        }

        // secure link if device is not bonded yet
        pm_link_status_t linkStatus;
        (void)pm_link_status_get(state.connHandleCentral, &linkStatus);
        if (linkStatus.connected && !linkStatus.bonded && !linkStatus.encrypted)
        {
            errCode = pm_link_secure(state.connHandleCentral, true);
            APP_ERROR_CHECK(errCode);
        }
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_DISCONNECTED;
        evt.connHandle = pBleEvt->evt.gap_evt.conn_handle;
        state.handler(&evt);

        state.connHandleCentral = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        errCode = sd_ble_gap_conn_param_update(pBleEvt->evt.gap_evt.conn_handle, &pBleEvt->evt.gap_evt.params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(errCode);
        break;

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
    ble_conn_state_on_ble_evt(pBleEvt);
    pm_ble_evt_handler(pBleEvt);

    uint8_t role = ble_conn_state_role(pBleEvt->evt.gap_evt.conn_handle);

    // database discovery and light control service (client and server) are used in both directions
    ble_db_discovery_on_ble_evt(&discDatabase, pBleEvt);
    ble_lcs_on_ble_evt(&lcsGattsData, pBleEvt);
    ble_lcs_c_on_ble_evt(&lcsGattcData, pBleEvt);

    if (role == BLE_GAP_ROLE_PERIPH || role == BLE_GAP_ROLE_INVALID)
    {
        ble_conn_params_on_ble_evt(pBleEvt);
        ble_advertising_on_ble_evt(pBleEvt);
        ble_cgw_on_ble_evt(&cgwGattsData, pBleEvt); // the com gateway is only used in peripheral connections
#ifdef HELENA_DEBUG_FIELD_TESTING
        debug_OnBleEvt(pBleEvt);
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
#ifdef HELENA_DEBUG_FIELD_TESTING
    debug_OnSysEvent(sysEvt);
#endif
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
    case PM_EVT_LINK_SECURED:
        break;

    case PM_EVT_LINK_SECURE_FAILED:
        /** In some cases, when securing fails, it can be restarted directly. Sometimes it can
         *  be restarted, but only after changing some Security Parameters. Sometimes, it cannot
         *  be restarted until the link is disconnected and reconnected. Sometimes it is
         *  impossible, to secure the link, or the peer device does not support it. How to
         *  handle this error is highly application dependent. */
        if (pEvt->params.link_secure_failed_evt.error.error_type == PM_ERROR_TYPE_PM_SEC_ERROR)
        {
            switch (pEvt->params.link_secure_failed_evt.error.error.pm_sec_error)
            {
                case PM_SEC_ERROR_CODE_PIN_OR_KEY_MISSING:
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

    //case PM_EVT_PEER_DATA_UPDATED:
    //    APP_ERROR_CHECK_BOOL(false);
    //    break;

    case PM_EVT_PEER_DATA_UPDATE_FAILED:
        APP_ERROR_CHECK_BOOL(false);
        break;

    case PM_EVT_ERROR_LOCAL_DB_CACHE_APPLY:
        // The local database has likely changed, send service changed indications.
        pm_local_database_has_changed();
        break;

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

    ble_gap_sec_params_t secParam;
    memset(&secParam, 0, sizeof(ble_gap_sec_params_t));

    // just works Security parameters to be used for all security procedures.
    secParam.bond              = true;
    secParam.mitm              = false;
    secParam.io_caps           = BLE_GAP_IO_CAPS_NONE;
    secParam.oob               = false;
    secParam.min_key_size      = 7;
    secParam.max_key_size      = 16;
    secParam.kdist_periph.enc  = 1;
    secParam.kdist_periph.id   = 0;
    secParam.kdist_central.enc = 1;
    secParam.kdist_central.id  = 0;

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
    APP_ERROR_CHECK(com_Put(pMessageRx));
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
        evt.lcscpEventParams.modeToConfig.pConfig = (btle_LcsModeConfig_t*)pEvt->p_params->mode_config.config;
        break;
    case BLE_LCS_CTRLPT_EVT_CNFG_GROUP:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_CONFIG_GROUP;
        evt.lcscpEventParams.groupConfig = pEvt->p_params->group_config;
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
        evt.lcscpEventParams.currentLimits.floodInPercent = pEvt->p_params->current_limits.flood;
        evt.lcscpEventParams.currentLimits.spotInPercent = pEvt->p_params->current_limits.spot;
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_PREF_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_PREF_MODE;
        break;
    case BLE_LCS_CTRLPT_EVT_SET_PREF_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_SET_PREF_MODE;
        evt.lcscpEventParams.prefMode = pEvt->p_params->pref_mode;
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
    versionString[i++] = '-';
    strcpy(&versionString[i], VERSION_LEVEL);
    i = strlen(versionString);
    versionString[i++] = '-';
    itoa(VERSION_BUILD, &versionString[i], 10);
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
    lcsInit.features.flood_supported = pFeature->floodSupported;
    lcsInit.features.spot_supported = pFeature->spotSupported;
    lcsInit.features.pitch_comp_supported = pFeature->pitchSupported;
    lcsInit.features.mode_change_supported = 1;
    lcsInit.features.mode_config_supported = 1;
    lcsInit.features.mode_grouping_supported = 1;
    lcsInit.features.led_config_check_supported = 1;
    lcsInit.features.sensor_calibration_supported = 1;
    lcsInit.features.current_limitation_supported = 1;
    lcsInit.features.preferred_mode_supported = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lf_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lcp_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lcp_attr_md.write_perm);
    errCode = ble_lcs_init(&lcsGattsData, NUM_OF_CONNECTIONS, &lcsInit);
    APP_ERROR_CHECK(errCode);
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

/** @brief Event handler for the hid service collector
 *
 * @details Until now this is just a provisionally event handler to work with
 *          the Xiaomi Yi Remote.
 *
 * @param[in] pBleHidsC GATT collector database for the hid service
 * @param[in] pEvt      received event
 */
static void hidsCEventHandler(ble_hids_c_t * pBleHidsC, ble_hids_c_evt_t * pEvt)
{
#define VOLUME_UP   (1<<6)
#define VOLUME_DOWN (1<<7)

#define BUTTONPRESS_LONG   APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)

    uint32_t errCode;
    static uint32_t lastPressVolUp, lastPressVolDown;

    switch (pEvt->evt_type)
    {
    case BLE_HIDS_C_EVT_DISCOVERY_COMPLETE:
        errCode = ble_hids_c_report_notif_enable(pBleHidsC, 2, true);
        APP_ERROR_CHECK(errCode);
        break;
    case BLE_HIDS_C_EVT_REPORT_MAP_READ_RESP:
        break;
    case BLE_HIDS_C_EVT_REPORT_NOTIFICATION:
        if (pEvt->params.report.index == 2)
        {
            if (pEvt->params.report.p_report[0] & VOLUME_UP)
                (void)app_timer_cnt_get(&lastPressVolUp);
            if (pEvt->params.report.p_report[0] & VOLUME_DOWN)
                (void)app_timer_cnt_get(&lastPressVolDown);
            if (pEvt->params.report.p_report[0] == 0)
            {
                uint32_t timestamp;
                btle_event_t evt;
                evt.connHandle = pBleHidsC->conn_handle;
                (void)app_timer_cnt_get(&timestamp);
                if (lastPressVolUp != 0)
                {
                    (void)app_timer_cnt_diff_compute(timestamp, lastPressVolUp, &timestamp);
                    lastPressVolUp = 0;
                    evt.evt = BTLE_EVT_HID;
                    if (timestamp > BUTTONPRESS_LONG)
                        evt.subEvt.hid = BTLE_EVT_HID_VOL_UP_LONG;
                    else
                        evt.subEvt.hid = BTLE_EVT_HID_VOL_UP_SHORT;
                    state.handler(&evt);
                }
                if (lastPressVolDown != 0)
                {
                    (void)app_timer_cnt_diff_compute(timestamp, lastPressVolDown, &timestamp);
                    lastPressVolDown = 0;
                    evt.evt = BTLE_EVT_HID;
                    if (timestamp > BUTTONPRESS_LONG)
                        evt.subEvt.hid = BTLE_EVT_HID_VOL_DOWN_LONG;
                    else
                        evt.subEvt.hid = BTLE_EVT_HID_VOL_DOWN_SHORT;
                    state.handler(&evt);
                }
            }
        }
        break;
    default:
        break;
    }
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

            evt.lcsmEventParams.mode.setup.flood             = pEvt->data.measurement.mode.setup.flood;
            evt.lcsmEventParams.mode.setup.spot              = pEvt->data.measurement.mode.setup.spot;
            evt.lcsmEventParams.mode.setup.pitchCompensation = pEvt->data.measurement.mode.setup.pitchCompensation;
            evt.lcsmEventParams.mode.setup.cloned            = pEvt->data.measurement.mode.setup.cloned;
            evt.lcsmEventParams.mode.setup.taillight         = pEvt->data.measurement.mode.setup.taillight;
            evt.lcsmEventParams.mode.setup.brakelight        = pEvt->data.measurement.mode.setup.brakelight;

            if (pEvt->data.measurement.flags.intensity_present)
            {
                if (pEvt->data.measurement.mode.setup.pitchCompensation)
                    evt.lcsmEventParams.mode.illuminanceInLux = pEvt->data.measurement.mode.intensity;
                else
                    evt.lcsmEventParams.mode.intensityInPercent = pEvt->data.measurement.mode.intensity;
            }
            if (pEvt->data.measurement.flags.flood_status_present)
            {
                evt.lcsmEventParams.statusFlood.overcurrent    = pEvt->data.measurement.flood_status.overcurrent;
                evt.lcsmEventParams.statusFlood.inputVoltage   = pEvt->data.measurement.flood_status.voltage;
                evt.lcsmEventParams.statusFlood.temperature    = pEvt->data.measurement.flood_status.temperature;
                evt.lcsmEventParams.statusFlood.dutyCycleLimit = pEvt->data.measurement.flood_status.dutycycle;
            }
            if (pEvt->data.measurement.flags.spot_status_present)
            {
                evt.lcsmEventParams.statusSpot.overcurrent    = pEvt->data.measurement.spot_status.overcurrent;
                evt.lcsmEventParams.statusSpot.inputVoltage   = pEvt->data.measurement.spot_status.voltage;
                evt.lcsmEventParams.statusSpot.temperature    = pEvt->data.measurement.spot_status.temperature;
                evt.lcsmEventParams.statusSpot.dutyCycleLimit = pEvt->data.measurement.spot_status.dutycycle;
            }
            if (pEvt->data.measurement.flags.flood_power_present)
                evt.lcsmEventParams.powerFlood = pEvt->data.measurement.flood_power;
            if (pEvt->data.measurement.flags.spot_power_present)
                evt.lcsmEventParams.powerSpot = pEvt->data.measurement.spot_power;
            if (pEvt->data.measurement.flags.temperature_present)
                evt.lcsmEventParams.temperature = pEvt->data.measurement.temperature;
            if (pEvt->data.measurement.flags.input_voltage_present)
                evt.lcsmEventParams.inputVoltage = pEvt->data.measurement.input_voltage;
            if (pEvt->data.measurement.flags.pitch_present)
                evt.lcsmEventParams.pitch = pEvt->data.measurement.pitch;

            state.handler(&evt);
        }
        break;

    case BLE_LCS_C_EVT_CONTROL_POINT_INDIC:
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

    ble_hids_c_init_t hidsCInit;
    hidsCInit.evt_handler = hidsCEventHandler;
    errCode = ble_hids_c_init(&hidsGattcData, &hidsCInit);
    APP_ERROR_CHECK(errCode);

    ble_lcs_c_init_t lcsCInit;
    lcsCInit.evt_handler = lcsCEventHandler;
    errCode = ble_lcs_c_init(&lcsGattcData, COUNT_OF(lcsGattcData_server_data), &lcsCInit);
    APP_ERROR_CHECK(errCode);
}

/** @brief Error handler for the scanning module
 */
static void scanningErrorHandler(uint32_t errCode)
{
    APP_ERROR_CHECK(errCode);
}

/** @brief Function to parse an advertising report and check if it is an HID device
 *
 * @param[in] pAdvData Advertising report to parse
 * @return    NRF_SUCCESS if a hid service uuid was found, otherwise NRF_ERROR_NOT_FOUND
 */
static uint32_t isHidDevice(const ble_gap_evt_adv_report_t * pAdvData)
{
    uint_fast8_t index = 0;

    while (index < pAdvData->dlen)
    {
        uint8_t length = pAdvData->data[index];
        uint8_t type   = pAdvData->data[index+1];

        if (type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE ||
            type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
        {
            index += 2;
            for (uint_fast8_t i = 0; i < length/UUID16_SIZE; i++)
            {
                uint16_t uuid;
                uuid = uint16_decode(&pAdvData->data[index + i * UUID16_SIZE]);
                if (uuid == BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE)
                    return NRF_SUCCESS;
            }
            return NRF_ERROR_NOT_FOUND;
        }
        index += length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/** @brief Function to parse an advertising report and check if it is a Light Control device
 *
 * @param[in] pAdvData Advertising report to parse
 * @return    NRF_SUCCESS if a Light Control service uuid was found, otherwise NRF_ERROR_NOT_FOUND
 */
static uint32_t isLcsDevice(const ble_gap_evt_adv_report_t * pAdvData)
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
                    return NRF_SUCCESS;
            }
            return NRF_ERROR_NOT_FOUND;
        }
        index += length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
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
    uint32_t errCode;

    switch (pScanEvt->ble_scan_event)
    {
    case BLE_SCAN_EVT_IDLE:
    case BLE_SCAN_EVT_PAUSE:
        state.scanMode = BLE_SCAN_MODE_IDLE;
        break;
    case BLE_SCAN_EVT_FAST:
    case BLE_SCAN_EVT_FAST_WHITELIST:
        state.scanMode = BLE_SCAN_MODE_FAST;
        break;
    case BLE_SCAN_EVT_SLOW:
    case BLE_SCAN_EVT_SLOW_WHITELIST:
        state.scanMode = BLE_SCAN_MODE_SLOW;
        if (state.scanConfig == BTLE_SCAN_MODE_LOW_LATENCY)
        {
            errCode = ble_scanning_start(BLE_SCAN_MODE_FAST);
            APP_ERROR_CHECK(errCode);
        }
        break;
    case BLE_SCAN_EVT_ADV_REPORT_RECEIVED:
    {
        bool isHid = isHidDevice(pScanEvt->p_ble_adv_report) == NRF_SUCCESS;
        bool isLcs = isLcsDevice(pScanEvt->p_ble_adv_report) == NRF_SUCCESS;
        // check if the device is advertising as hid device
        if (isHid || isLcs)
        {
            static const ble_gap_scan_params_t scanParams =
            {
                .active      = 0,
                .selective   = 0,
                .p_whitelist = NULL,
                .interval    = MSEC_TO_UNITS(22.5, UNIT_0_625_MS),
                .window      = MSEC_TO_UNITS(11.25, UNIT_0_625_MS),
                .timeout     = 180,
            };
            static const ble_gap_conn_params_t connParams =
            {
                MSEC_TO_UNITS(50, UNIT_1_25_MS),
                MSEC_TO_UNITS(100, UNIT_1_25_MS),
                0,
                MSEC_TO_UNITS(4000, UNIT_10_MS)
            };
            // stop scanning
            errCode = ble_scanning_start(BLE_SCAN_MODE_IDLE);
            APP_ERROR_CHECK(errCode);
            // and connect to device
            errCode = sd_ble_gap_connect(&pScanEvt->p_ble_adv_report->peer_addr, &scanParams, &connParams);
            APP_ERROR_CHECK(errCode);
        }
    }   break;
    case BLE_SCAN_EVT_WHITELIST_REQUEST:
    {
        ble_gap_irk_t * irks[1];
        ble_gap_addr_t * addrs[1];
        pm_peer_id_t peerIds[1];
        uint_fast8_t peerIdsCnt = 0;
        ble_gap_whitelist_t whitelist = {.pp_irks = irks, .pp_addrs = addrs};
        pm_peer_id_t peerId = pm_next_peer_id_get(PM_PEER_ID_INVALID);
        while(peerId != PM_PEER_ID_INVALID && peerIdsCnt < 1)
        {
            pm_peer_data_bonding_t bondData;
            pm_peer_data_t peerData;
            peerData.length_words = PM_BONDING_DATA_N_WORDS();
            peerData.data.p_bonding_data = &bondData;
            peerData.data_type = PM_PEER_DATA_ID_BONDING;
            errCode = pm_peer_data_get(peerId, PM_PEER_DATA_ID_BONDING, &peerData);
            if (errCode != NRF_SUCCESS)
            {
                APP_ERROR_CHECK(errCode);
            }
            else if (peerData.data.p_bonding_data->own_role == BLE_GAP_ROLE_CENTRAL)
                peerIds[peerIdsCnt++] = peerId;
            peerId = pm_next_peer_id_get(peerId);
        }
        errCode = pm_wlist_create(peerIds, peerIdsCnt, &whitelist);
        APP_ERROR_CHECK(errCode);
        errCode = ble_scanning_whitelist_reply(&whitelist);
        APP_ERROR_CHECK(errCode);
    }   break;
    default:
        break;
    }
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
    options.ble_scan_fast_interval      = MSEC_TO_UNITS(22.5, UNIT_0_625_MS);
    options.ble_scan_fast_window        = MSEC_TO_UNITS(11.25, UNIT_0_625_MS);
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
    bleEnableParams.gatts_enable_params.attr_tab_size   = 0x400; //BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
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
    notifyData.mode.setup.flood = pData->mode.setup.flood ? 1 : 0;
    notifyData.mode.setup.spot = pData->mode.setup.spot ? 1 : 0;
    notifyData.mode.setup.pitchCompensation = pData->mode.setup.pitchCompensation ? 1 : 0;
    notifyData.mode.setup.cloned = pData->mode.setup.cloned ? 1 : 0;
    notifyData.mode.setup.taillight = pData->mode.setup.taillight ? 1 : 0;
    notifyData.mode.setup.brakelight = pData->mode.setup.brakelight ? 1 : 0;
    if (pData->mode.setup.flood || pData->mode.setup.spot)
    {
        notifyData.flags.intensity_present = 1;
        notifyData.mode.intensity = pData->mode.intensityInPercent;  // don't care if pitch compensated or not, variables share same location
    }
    if (pData->mode.setup.flood)
    {
        notifyData.flags.flood_power_present = 1;
        notifyData.flags.flood_status_present = 1;
        notifyData.flood_power = pData->powerFlood;
        notifyData.flood_status.overcurrent = pData->statusFlood.overcurrent ? 1 : 0;
        notifyData.flood_status.voltage = pData->statusFlood.inputVoltage ? 1 : 0;
        notifyData.flood_status.temperature = pData->statusFlood.temperature ? 1 : 0;
        notifyData.flood_status.dutycycle = pData->statusFlood.dutyCycleLimit ? 1 : 0;
    }
    if (pData->mode.setup.spot)
    {
        notifyData.flags.spot_power_present = 1;
        notifyData.flags.spot_status_present = 1;
        notifyData.spot_power = pData->powerSpot;
        notifyData.spot_status.overcurrent = pData->statusSpot.overcurrent ? 1 : 0;
        notifyData.spot_status.voltage = pData->statusSpot.inputVoltage ? 1 : 0;
        notifyData.spot_status.temperature = pData->statusSpot.temperature ? 1 : 0;
        notifyData.spot_status.dutycycle = pData->statusSpot.dutyCycleLimit ? 1 : 0;
    }
    if (pData->temperature >= -40 && pData->temperature <= 85)
    {
        notifyData.flags.temperature_present = 1;
        notifyData.temperature = pData->temperature;
    }
    if (pData->inputVoltage != 0)
    {
        notifyData.flags.input_voltage_present = 1;
        notifyData.input_voltage = pData->inputVoltage;;
    }
    if (pData->pitch >= -90 && pData->pitch <= 90)
    {
        notifyData.flags.pitch_present = 1;
        notifyData.pitch = pData->pitch;
    }

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
        rsp.params.group_config = pRsp->responseParams.groupCnt;
        break;
    case BTLE_EVT_LCSCP_REQ_MODE_CONFIG:
        rsp.params.mode_config_list.p_list = (ble_lcs_light_mode_t*)pRsp->responseParams.modeList.pList;
        rsp.params.mode_config_list.num_of_entries = pRsp->responseParams.modeList.listEntries;
        break;
    case BTLE_EVT_LCSCP_REQ_LED_CONFIG:
    case BTLE_EVT_LCSCP_CHECK_LED_CONFIG:
        rsp.params.led_config.cnt_flood = pRsp->responseParams.ledConfig.floodCnt;
        rsp.params.led_config.cnt_spot = pRsp->responseParams.ledConfig.spotCnt;
        break;
    case BTLE_EVT_LCSCP_REQ_SENS_OFFSET:
    case BTLE_EVT_LCSCP_CALIB_SENS_OFFSET:
        rsp.params.sens_offset.x = pRsp->responseParams.sensOffset.x;
        rsp.params.sens_offset.y = pRsp->responseParams.sensOffset.y;
        rsp.params.sens_offset.z = pRsp->responseParams.sensOffset.z;
        break;
    case BTLE_EVT_LCSCP_REQ_LIMITS:
        rsp.params.current_limits.flood = pRsp->responseParams.currentLimits.floodInPercent;
        rsp.params.current_limits.spot = pRsp->responseParams.currentLimits.spotInPercent;
        break;
    case BTLE_EVT_LCSCP_REQ_PREF_MODE:
        rsp.params.pref_mode = pRsp->responseParams.prefMode;
        break;
    default:
        break;
    }

    return ble_lcs_ctrlpt_mode_resp(&lcsGattsData.ctrl_pt, connHandle, &rsp);
}

uint32_t btle_DeleteBonds()
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

    if (state.connHandlePeriph != BLE_CONN_HANDLE_INVALID)
    {
        errCode = sd_ble_gap_disconnect(state.connHandlePeriph, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (errCode != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(errCode);
            return errCode;
        }
    }

    pm_peer_delete_all();

    errCode = ble_scanning_pause(false);
    if (errCode != NRF_SUCCESS)
    {
        APP_ERROR_CHECK(errCode);
        return errCode;
    }

    return ble_scanning_start(BLE_SCAN_MODE_IDLE);
}

uint32_t btle_SearchForRemote()
{
    return ble_scanning_start_without_whitelist(BLE_SCAN_MODE_FAST);
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

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEAF, line_num, p_file_name);
}

/**END OF FILE*****************************************************************/



