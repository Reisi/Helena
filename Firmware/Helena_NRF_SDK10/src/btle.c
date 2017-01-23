/**
  ******************************************************************************
  * @file    btle.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/08
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
    uint8_t     * pData;
    uint16_t      dataLen;
} advDataStruct;

/* Private macros ------------------------------------------------------------*/
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/* Private defines -----------------------------------------------------------*/
#define UUID16_SIZE                     2

#define DEVICE_NAME                     "Helena"
#define DEVICE_APPEARANCE               BLE_APPEARANCE_GENERIC_CYCLING
#define MANUFACTURER_NAME               "insert_name_here"

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define ADV_INTERVAL_FAST               MSEC_TO_UNITS(40, UNIT_0_625_MS)
#define ADV_INTERVAL_SLOW               MSEC_TO_UNITS(1280, UNIT_0_625_MS)
#define ADV_TIMEOUT_IN_SECONDS          180

#define SCAN_INTERVAL_FAST              MSEC_TO_UNITS(22.5, UNIT_0_625_MS)
#define SCAN_WINDOW_FAST                MSEC_TO_UNITS(11.25, UNIT_0_625_MS)
#define SCAN_TIMEOUT_FAST               180
#define SCAN_INTERVAL_SLOW              MSEC_TO_UNITS(1280, UNIT_0_625_MS)
#define SCAN_WINDOW_SLOW                MSEC_TO_UNITS(11.25, UNIT_0_625_MS)
#define SCAN_TIMEOUT_SLOW               0

#define SEC_PARAM_BOND                  1                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                      /**< Maximum encryption key size. */

#define LCS_NOTIFY_PERIOD               APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

/* Private variables ---------------------------------------------------------*/
static ble_cgw_t cgwGattsData;          /**< gatt server handles for the com gateway service */
static ble_lcs_t lcsGattsData;          /**< gatt server handles for the light control service  */

static ble_db_discovery_t discDatabase; /**< database for service discovery */
static ble_hids_c_t hidsGattcData;      /**< gatt client handles for the hid service */

static btle_EventHandler pEventHandler; /**< ble event handler */

static btle_ScanModeEnum scanConfig;    /**< actual requested scan mode configuration */
static ble_scan_mode_t scanMode;        /**< actual Scan mode */

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @brief Application event handler for central events from the ble stack
 */
static void onCentralEvt(ble_evt_t * pBleEvt)
{
    btle_EventStruct evt;
    uint32_t errCode;

    switch (pBleEvt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_CONNECTED;
        pEventHandler(&evt);

        // start database discovery
        memset(&discDatabase, 0, sizeof(discDatabase));
        errCode = ble_db_discovery_start(&discDatabase, pBleEvt->evt.gap_evt.conn_handle);
        APP_ERROR_CHECK(errCode);

        // enable bonding if device is not bonded yet
        pm_peer_id_t peerId;
        if (pm_peer_id_get(pBleEvt->evt.gap_evt.conn_handle, &peerId) == NRF_ERROR_NOT_FOUND)
        {
            errCode = pm_link_secure(pBleEvt->evt.gap_evt.conn_handle, true);
            APP_ERROR_CHECK(errCode);
        }
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_CENTRAL_DISCONNECTED;
        pEventHandler(&evt);
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
    btle_EventStruct evt;

    switch (pBleEvt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_PERIPH_CONNECTED;
        pEventHandler(&evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        evt.evt = BTLE_EVT_CONNECTION;
        evt.subEvt.conn = BTLE_EVT_CONN_PERIPH_DISCONNECTED;
        pEventHandler(&evt);
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

    if (role == BLE_GAP_ROLE_PERIPH || role == BLE_GAP_ROLE_INVALID)
    {
        ble_conn_params_on_ble_evt(pBleEvt);
        ble_advertising_on_ble_evt(pBleEvt);
        ble_cgw_on_ble_evt(&cgwGattsData, pBleEvt);
        ble_lcs_on_ble_evt(&lcsGattsData, pBleEvt);
#ifdef HELENA_DEBUG_FIELD_TESTING
        debug_OnBleEvt(pBleEvt);
#endif
        onPeriphEvt(pBleEvt);
    }
    if (role == BLE_GAP_ROLE_CENTRAL || role == BLE_GAP_ROLE_INVALID)
    {
        ble_scanning_on_ble_evt(pBleEvt);
        ble_db_discovery_on_ble_evt(&discDatabase, pBleEvt);
        ble_hids_c_on_ble_evt(&hidsGattcData, pBleEvt);
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

/** @brief Function for initialization of the ble stack
 */
static void bleStackInit()
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

    // Security parameters to be used for all security procedures.
    secParam.bond              = SEC_PARAM_BOND;
    secParam.mitm              = SEC_PARAM_MITM;
    secParam.io_caps           = SEC_PARAM_IO_CAPABILITIES;
    secParam.oob               = SEC_PARAM_OOB;
    secParam.min_key_size      = SEC_PARAM_MIN_KEY_SIZE;
    secParam.max_key_size      = SEC_PARAM_MAX_KEY_SIZE;
    secParam.kdist_periph.enc  = 1;
    secParam.kdist_periph.id   = 1;
    secParam.kdist_central.enc = 1;
    secParam.kdist_central.id  = 1;

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
    gapConnParams.min_conn_interval = MIN_CONN_INTERVAL;
    gapConnParams.max_conn_interval = MAX_CONN_INTERVAL;
    gapConnParams.slave_latency     = SLAVE_LATENCY;
    gapConnParams.conn_sup_timeout  = CONN_SUP_TIMEOUT;

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
    connParamInit.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    connParamInit.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    connParamInit.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    connParamInit.start_on_notify_cccd_handle    = cgwGattsData.rx_handles.cccd_handle;
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
    SEGGER_RTT_printf(0, "%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", pMessageRx->Identifier,
                                                                             pMessageRx->Control,
                                                                             pMessageRx->Data[0],
                                                                             pMessageRx->Data[1],
                                                                             pMessageRx->Data[2],
                                                                             pMessageRx->Data[3],
                                                                             pMessageRx->Data[4],
                                                                             pMessageRx->Data[5],
                                                                             pMessageRx->Data[6]);
    APP_ERROR_CHECK(com_Put(pMessageRx));
}

/** @brief Event handler for the light control service
 */
static void lcsCpEventHandler(ble_lcs_ctrlpt_t * pLcsCtrlpt,
                                                   ble_lcs_ctrlpt_evt_t * pEvt)
{
    btle_EventStruct evt;
    evt.evt = BTLE_EVT_LCS_CTRL_POINT;

    switch (pEvt->evt_type)
    {
    case BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_MODE_CNT;
        pEventHandler(&evt);
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_GRP_CNFG:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_GROUP_CNT;
        pEventHandler(&evt);
        break;
    case BLE_LCS_CTRLPT_EVT_REQ_MODE_CNFG:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_REQ_MODE_CONFIG;
        evt.lcscpEventParams.modeConfigStart = pEvt->p_params->mode_list_start;
        pEventHandler(&evt);
        break;
    case BLE_LCS_CTRLPT_EVT_SET_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_SET_MODE;
        evt.lcscpEventParams.modeToSet = pEvt->p_params->set_mode;
        pEventHandler(&evt);
        break;
    case BLE_LCS_CTRLPT_EVT_CNFG_MODE:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_CONFIG_MODE;
        evt.lcscpEventParams.modeToConfig.modeNumber = pEvt->p_params->mode_config.mode_number_start;
        evt.lcscpEventParams.modeToConfig.listEntries = pEvt->p_params->mode_config.mode_entries;
        evt.lcscpEventParams.modeToConfig.pConfig = (btle_LightModeConfig*)pEvt->p_params->mode_config.config;
        pEventHandler(&evt);
        break;
    case BLE_LCS_CTRLPT_EVT_CNFG_GROUP:
        evt.subEvt.lcscp = BTLE_EVT_LCSCP_CONFIG_GROUP;
        evt.lcscpEventParams.groupConfig = pEvt->p_params->group_config;
        pEventHandler(&evt);
        break;
    default:
        break;
    }
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
static void servicesInit(void)
{
    uint32_t errCode;

    //initialize device information service
    ble_dis_init_t disInit;
    char versionString[24];
    uint_fast8_t i = 0;
    versionString[i++] = VERSION_MAJOR + '0';
    versionString[i++] = '.';
    if (VERSION_MINOR < 10)
        versionString[i++] = VERSION_MINOR + '0';
    else
        versionString[i++] = VERSION_MINOR - 10 + 'A';
    versionString[i++] = '.';
    if (VERSION_PATCH < 10)
        versionString[i++] = VERSION_PATCH + '0';
    else
        versionString[i++] = VERSION_PATCH - 10 + 'A';
    versionString[i++] = '-';
    strcpy(&versionString[i], VERSION_LEVEL);
    i = strlen(versionString);
    versionString[i++] = '-';
    itoa(VERSION_BUILD, &versionString[i], 10);
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
    if (pEventHandler != NULL)                      // control point can not be
        lcsInit.cp_evt_handler = lcsCpEventHandler; // included without event handler
    lcsInit.error_handler = lcsErrorHandler;
    lcsInit.features.flood_supported = 1;
    lcsInit.features.spot_supported = 1;
    lcsInit.features.pitch_comp_supported = 1;
    lcsInit.features.mode_change_supported = 1;
    lcsInit.features.mode_config_supported = 1;
    lcsInit.features.mode_grouping_supported = 1;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lf_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lcp_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&lcsInit.lcs_lcp_attr_md.write_perm);
    errCode = ble_lcs_init(&lcsGattsData, &lcsInit);
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
    ble_uuid_t advUuids[1];
    ble_advdata_t advData, scanRsp;
    ble_adv_modes_config_t options;

    advUuids[0].uuid = BLE_UUID_CGW_SERVICE;
    advUuids[0].type = cgwGattsData.uuid_type;

    memset(&advData, 0, sizeof(ble_advdata_t));
    advData.name_type               = BLE_ADVDATA_FULL_NAME;
    advData.include_appearance      = true;
    advData.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&scanRsp, 0, sizeof(ble_advdata_t));
    scanRsp.uuids_complete.uuid_cnt = sizeof(advUuids) / sizeof(advUuids[0]);
    scanRsp.uuids_complete.p_uuids  = advUuids;

    memset(&options, 0, sizeof(ble_adv_modes_config_t));
    options.ble_adv_fast_enabled    = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval   = ADV_INTERVAL_FAST;
    options.ble_adv_fast_timeout    = ADV_TIMEOUT_IN_SECONDS;
    options.ble_adv_slow_enabled    = BLE_ADV_SLOW_ENABLED;
    options.ble_adv_slow_interval   = ADV_INTERVAL_SLOW;
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
                btle_EventStruct evt;
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
                    pEventHandler(&evt);
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
                    pEventHandler(&evt);
                }
            }
        }
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
}

/** @brief Error handler for the scanning module
 */
static void scanningErrorHandler(uint32_t errCode)
{
    APP_ERROR_CHECK(errCode);
}

/** @brief Function to parse an advertising report
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
                UUID16_EXTRACT(&uuid, &pAdvData->data[index + i * UUID16_SIZE]);
                if (uuid == BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE)
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
        scanMode = BLE_SCAN_MODE_IDLE;
        break;
    case BLE_SCAN_EVT_FAST:
    case BLE_SCAN_EVT_FAST_WHITELIST:
        scanMode = BLE_SCAN_MODE_FAST;
        break;
    case BLE_SCAN_EVT_SLOW:
    case BLE_SCAN_EVT_SLOW_WHITELIST:
        scanMode = BLE_SCAN_MODE_SLOW;
        if (scanConfig == BTLE_SCAN_MODE_LOW_LATENCY)
        {
            errCode = ble_scanning_start(BLE_SCAN_MODE_FAST);
            APP_ERROR_CHECK(errCode);
        }
        break;
    case BLE_SCAN_EVT_ADV_REPORT_RECEIVED:
        // check if the device is advertising as hid device
        if (isHidDevice(pScanEvt->p_ble_adv_report) == NRF_SUCCESS)
        {
            static const ble_gap_scan_params_t scanParams =
            {
                .active      = 0,
                .selective   = 0,
                .p_whitelist = NULL,
                .interval    = SCAN_INTERVAL_FAST,
                .window      = SCAN_WINDOW_FAST,
                .timeout     = SCAN_TIMEOUT_FAST,
            };
            static const ble_gap_conn_params_t connParams =
            {
                MSEC_TO_UNITS(50, UNIT_1_25_MS),
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
        }
        break;
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
    options.ble_scan_fast_interval      = SCAN_INTERVAL_FAST;
    options.ble_scan_fast_timeout       = SCAN_TIMEOUT_FAST;
    options.ble_scan_fast_window        = SCAN_WINDOW_FAST;
    options.ble_scan_slow_enabled       = true;
    options.ble_scan_slow_interval      = SCAN_INTERVAL_SLOW;
    options.ble_scan_slow_timeout       = SCAN_TIMEOUT_SLOW;
    options.ble_scan_slow_window        = SCAN_WINDOW_SLOW;

    errCode = ble_scanning_init(&options, scanningEventHandler, scanningErrorHandler);
    APP_ERROR_CHECK(errCode);

    errCode = ble_scanning_start(BLE_SCAN_MODE_FAST);
    APP_ERROR_CHECK(errCode);
}

/* Public functions ----------------------------------------------------------*/
void btle_Init(bool deleteBonds, btle_EventHandler pEvtHandler)
{
    pEventHandler = pEvtHandler;

    bleStackInit();
    peerManagerInit(deleteBonds);
    gapParamsInit();
    connParamsInit();
    servicesInit();
    advertisingInit();
    discoveryInit();
    serviceCollectorInit();
    scanningInit();
}

void btle_SetScanConfig(btle_ScanModeEnum newScanConfig)
{
    uint32_t errCode;

    scanConfig = newScanConfig;
    if (scanConfig == BTLE_SCAN_MODE_LOW_LATENCY && scanMode == BLE_SCAN_MODE_SLOW)
    {
        errCode = ble_scanning_start(BLE_SCAN_MODE_FAST);
        APP_ERROR_CHECK(errCode);
    }
    if (scanConfig == BTLE_SCAN_MODE_LOW_POWER && scanMode == BLE_SCAN_MODE_FAST)
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

uint32_t btle_UpdateLightMeasurements(const btle_lightDataStruct * pData)
{
    static uint32_t lastTimeSent;
    ble_lcs_lm_t notifyData;
    union
    {
        ble_lcs_lm_status_flags_t out;
        btle_lightStatusFlags     in;
    } statusFlags;
    uint32_t timestamp;

    (void)app_timer_cnt_get(&timestamp);
    (void)app_timer_cnt_diff_compute(timestamp, lastTimeSent, &timestamp);

    if (timestamp < LCS_NOTIFY_PERIOD ||
        lcsGattsData.conn_handle == BLE_CONN_HANDLE_INVALID ||
        lcsGattsData.is_lm_notfy_enabled == false)
        return NRF_SUCCESS;

    if (pData == NULL)
        return NRF_ERROR_NULL;

    if (lcsGattsData.conn_handle == BLE_CONN_HANDLE_INVALID || lcsGattsData.is_lm_notfy_enabled == false)
        return NRF_ERROR_INVALID_STATE;

    memset(&notifyData, 0, sizeof(ble_lcs_lm_t));
    notifyData.mode.type = pData->mode;
    if (pData->mode != BTLE_LIGHT_MODE_OFF)
    {
        notifyData.flags.intensity_present = 1;
        notifyData.mode.intensity = pData->intensity;
    }
    if (pData->mode == BTLE_LIGHT_MODE_FLOOD || pData->mode == BTLE_LIGHT_MODE_FLOOD_AND_SPOT ||
        pData->mode == BTLE_LIGHT_MODE_FLOOD_PITCH_COMPENSATED || pData->mode == BTLE_LIGHT_MODE_FLOOD_AND_SPOT_PITCH_COMPENSATED ||
        pData->mode == BTLE_LIGHT_MODE_FLOOD_CLONED || pData->mode == BTLE_LIGHT_MODE_FLOOD_PITCH_COMPENSATED_CLONED)
    {
        notifyData.flags.flood_power_present = 1;
        notifyData.flags.flood_status_present = 1;
        notifyData.flood_power = pData->powerFlood;
        statusFlags.in = pData->statusFlood;
        notifyData.flood_status = statusFlags.out;
    }
    if (pData->mode == BTLE_LIGHT_MODE_SPOT || pData->mode == BTLE_LIGHT_MODE_FLOOD_AND_SPOT ||
        pData->mode == BTLE_LIGHT_MODE_SPOT_PITCH_COMPENSATED || pData->mode == BTLE_LIGHT_MODE_FLOOD_AND_SPOT_PITCH_COMPENSATED ||
        pData->mode == BTLE_LIGHT_MODE_SPOT_CLONED || pData->mode == BTLE_LIGHT_MODE_SPOT_PITCH_COMPENSATED_CLONED)
    {
        notifyData.flags.spot_power_present = 1;
        notifyData.flags.spot_status_present = 1;
        notifyData.spot_power = pData->powerSpot;
        statusFlags.in = pData->statusSpot;
        notifyData.spot_status = statusFlags.out;
    }
    if (pData->temperature != 0)
    {
        notifyData.flags.temperature_present = 1;
        notifyData.temperature = pData->temperature;
    }
    if (pData->inputVoltage != 0)
    {
        notifyData.flags.input_voltage_present = 1;
        notifyData.input_voltage = pData->inputVoltage;;
    }
    if (pData->mode == BTLE_LIGHT_MODE_FLOOD_PITCH_COMPENSATED || pData->mode == BTLE_LIGHT_MODE_SPOT_PITCH_COMPENSATED ||
        pData->mode == BTLE_LIGHT_MODE_FLOOD_PITCH_COMPENSATED_CLONED || pData->mode == BTLE_LIGHT_MODE_SPOT_PITCH_COMPENSATED_CLONED ||
        pData->mode == BTLE_LIGHT_MODE_FLOOD_AND_SPOT_PITCH_COMPENSATED)
    {
        notifyData.flags.pitch_present = 1;
        notifyData.pitch = pData->pitch;
    }

    (void)app_timer_cnt_get(&lastTimeSent);

    return ble_lcs_light_measurement_send(&lcsGattsData, &notifyData);
}

uint32_t btle_SendEventResponse(const btle_LcscpEventResponseStruct *pRsp)
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
    default:
        break;
    }

    return ble_lcs_ctrlpt_mode_resp(&lcsGattsData.ctrl_pt, &rsp);
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

    if (lcsGattsData.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        errCode = sd_ble_gap_disconnect(lcsGattsData.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (errCode != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(errCode);
            return errCode;
        }
    }

    if (hidsGattcData.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        errCode = sd_ble_gap_disconnect(hidsGattcData.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
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

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEAF, line_num, p_file_name);
}

/**END OF FILE*****************************************************************/



