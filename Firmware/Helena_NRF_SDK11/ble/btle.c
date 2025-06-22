/**
  ******************************************************************************
  * @file    btle.c
  * @author  Thomas Reisnecker
  * @brief   ricos bluetooth module
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define BTLE_LOG_ENABLED

#ifdef BTLE_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BTLE_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "btle.h"
#include "main.h"
#include "softdevice_handler.h"
#include <string.h>
#include "link_management.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "ble_dis.h"
#include "ble_hps.h"
#include "ble_hps_c.h"
#include "ble_hps_discovery.h"
#include "ble_hids_c.h"
#include "ble_gatt_queue.h"
#include "dfu_app_handler_pm.h"
#include "board_config.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))
#define BTLE_SERVICE_CNT()      NRF_SECTION_VARS_COUNT(btle_service_t, btle_services)
#define BTLE_SERVICE_GET(cnt)   NRF_SECTION_VARS_GET(cnt, btle_service_t, btle_services)

/* Private defines -----------------------------------------------------------*/
#define TOTAL_LINK_COUNT        (PERIPHERAL_LINK_COUNT + CENTRAL_LINK_COUNT)

#define CENTRAL_SEC_COUNT       CENTRAL_LINK_COUNT

#define DFU_REVISION            0x0001

/* Private variables ---------------------------------------------------------*/
static ble_dfu_t              dfuGattS;         // server data for device firmware update service
BLE_HPS_DEF(hpsGattS, TOTAL_LINK_COUNT);        // server data for helen project service
static ble_hps_evt_handler_t  pHpsServerHandler;// the application hps event handler

BLE_HPS_C_DEF(hpsGattC, TOTAL_LINK_COUNT);      // helen project service client
BLE_HPS_DISC_DEF(hpsDisc, TOTAL_LINK_COUNT);    // the hps discovery database
static ble_hids_c_t           hidsGattC;        // hid service client
static ble_gq_inst_t          gattQueueInst;

static ble_evt_handler_t      linkManagerBleEventHandler;
static btle_eventHandler_t    btleEventHandler;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void bleEventDispatch(ble_evt_t* pBleEvt)
{
    if (linkManagerBleEventHandler != NULL)
        linkManagerBleEventHandler(pBleEvt);

    ble_hps_on_ble_evt(&hpsGattS, pBleEvt);
    ble_dfu_on_ble_evt(&dfuGattS, pBleEvt);

    ble_gq_on_ble_evt(&gattQueueInst);
    ble_hps_c_on_ble_evt(&hpsGattC, pBleEvt);
    ble_hps_discovery_on_ble_evt(&hpsDisc, pBleEvt);
    ble_hids_c_on_ble_evt(&hidsGattC, pBleEvt);

    for (uint_fast8_t i = 0; i < BTLE_SERVICE_CNT(); i++)
    {
        btle_service_t const* pService = BTLE_SERVICE_GET(i);
        if (pService->eventHandler != NULL)
            pService->eventHandler(pService->pContext, pBleEvt);
    }

    // check if there was a write operation to the device name characteristic and report the new name
    if (btleEventHandler != NULL && pBleEvt->header.evt_id == BLE_GATTS_EVT_WRITE)
    {
        if (pBleEvt->evt.gatts_evt.params.write.uuid.uuid == BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME)
        {
            uint16_t len = pBleEvt->evt.gatts_evt.params.write.len;
            char newName[len + 1];

            memcpy(newName, pBleEvt->evt.gatts_evt.params.write.data, len);
            newName[len] = 0;

            btle_event_t evt;
            evt.type = BTLE_EVT_NAME_CHANGE;
            evt.pNewDeviceName = newName;

            btleEventHandler(&evt);
        }
    }
}

static ret_code_t bleStackInit()
{
    ret_code_t errCode;
    ble_enable_params_t enableParams;

    memset(&enableParams, 0, sizeof(enableParams));
    enableParams.common_enable_params.vs_uuid_count   = LONG_UUID_COUNT;
    enableParams.gap_enable_params.periph_conn_count  = PERIPHERAL_LINK_COUNT;
    enableParams.gap_enable_params.central_conn_count = CENTRAL_LINK_COUNT;
    enableParams.gap_enable_params.central_sec_count  = CENTRAL_SEC_COUNT;
    enableParams.gatts_enable_params.attr_tab_size    = ATTR_TAB_SIZE;
    enableParams.gatts_enable_params.service_changed  = 1;

    errCode = softdevice_enable(&enableParams);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = softdevice_ble_evt_handler_set(bleEventDispatch);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = sd_ble_gap_tx_power_set(4);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static ret_code_t gapParamsInit(uint16_t deviceAppearance, char const* pDeviceName, uint16_t maxDeviceNameLenght)
{
    uint32_t errCode;
    ble_gap_conn_params_t connParams = *lm_getPreferredConnParams();
    ble_gap_conn_sec_mode_t secMode;

    if (maxDeviceNameLenght > MAX_NAME_LENGTH)
        return NRF_ERROR_INVALID_LENGTH;
    else if (maxDeviceNameLenght)
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&secMode);
    else
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&secMode);

    errCode = sd_ble_gap_device_name_set(&secMode, (uint8_t const*)pDeviceName, strlen(pDeviceName));
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = sd_ble_gap_appearance_set(deviceAppearance);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = sd_ble_gap_ppcp_set(&connParams);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static bool hpsServerHandler(ble_hps_evt_t const * pEvt)
{
    static bool indicEnabled;

    // check if control point indications have been en/disbaled
    if (ble_conn_state_role(pEvt->p_client->conn_handle) == BLE_GAP_ROLE_PERIPH)
    {
        switch (pEvt->evt_type)
        {
        case BLE_HPS_EVT_CP_INDICATION_ENABLED:
            if (!indicEnabled)
            {
                LOG_INFO("[btle]: remote disabled");
                lm_diconnectAndIgnoreRemote(true);
                indicEnabled = true;
            }
            break;
        case BLE_HPS_EVT_CP_INDICATION_DISABLED:
            if (indicEnabled)
            {
                LOG_INFO("[btle]: remote reenabled");
                lm_diconnectAndIgnoreRemote(false);
                indicEnabled = false;
            }
            break;
        default:
            break;
        }
    }

    // relay event to main
    if (pHpsServerHandler != NULL)
        return pHpsServerHandler(pEvt);
    else
        return false;
}

static void hpsServerErrorHandler(uint32_t errCode)
{
    /// TODO: sd_ble_gatts_hvx returns NRF_ERROR_INVALID_STATE when initiating factory reset through hps, idk why?
    LOG_ERROR("[btle]: hps error %d", errCode);
}

static void resetPrepare(uint16_t connHandle)
{

}

static ret_code_t gattServerInit(ble_hps_init_t* pHpsInit, btle_info_t const* pInfo)
{
    ret_code_t errCode;
    ble_dis_init_t disInit = {{0}};
    ble_dfu_init_t dfusInit = {0};

    // initialize the device information service
    if (pInfo->pManufacturer != NULL)
        ble_srv_ascii_to_utf8(&disInit.manufact_name_str, (char*)pInfo->pManufacturer);
    if (pInfo->pModelNumber != NULL)
        ble_srv_ascii_to_utf8(&disInit.model_num_str, (char*)pInfo->pModelNumber);
    if (pInfo->pBoardHWVersion != NULL)
        ble_srv_ascii_to_utf8(&disInit.hw_rev_str, (char*)pInfo->pBoardHWVersion);
    ble_srv_ascii_to_utf8(&disInit.fw_rev_str, VERSION_STRING);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&disInit.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&disInit.dis_attr_md.write_perm);
    errCode = ble_dis_init(&disInit);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[btle]: dis init error %d\r\n", errCode);
        return errCode;
    }

    // initialize the helen project service
    if (pHpsInit != NULL)
    {
#pragma message ( "debug configuration set to just works temporary" )
#ifdef DEBUG_EXT
        pHpsInit->modes_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
        pHpsInit->cp_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
#else
        pHpsInit->modes_wr_sec = SEC_JUST_WORKS;
        pHpsInit->cp_wr_sec = SEC_JUST_WORKS;
#endif // DEBUG_EXT
        pHpsServerHandler = pHpsInit->evt_handler;  // save handler, we need to listen to events here, too
        pHpsInit->evt_handler = hpsServerHandler;
        pHpsInit->error_handler = hpsServerErrorHandler;
        errCode = ble_hps_init(&hpsGattS, ARRAY_SIZE(hpsGattS_client_data), pHpsInit);
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[btle]: hps init error %d\r\n", errCode);
            return errCode;
        }
    }

    // initialize the device firmware update service
    dfusInit.evt_handler = dfu_app_on_dfu_evt;
    dfusInit.revision = DFU_REVISION;
    errCode = ble_dfu_init(&dfuGattS, &dfusInit);
    if (errCode != NRF_SUCCESS)
        return errCode;

    dfu_app_reset_prepare_set(resetPrepare);

    // initialize additional services
    for (uint_fast8_t i = 0; i < BTLE_SERVICE_CNT(); i++)
    {
        btle_service_t const* pService = BTLE_SERVICE_GET(i);
        if (pService->serviceAdd != NULL)
        {
            ret_code_t errCode = pService->serviceAdd(pService->pContext);
            if (errCode != NRF_SUCCESS)
            {
                LOG_ERROR("[btle]: custom service init error %d", errCode);
                return errCode;
            }
        }
    }

    return NRF_SUCCESS;
}

static void gqErrorHandler(uint32_t errCode)
{
    LOG_ERROR("[btle]: gatt queue error %d", errCode);
}

static void hpsClientErrorHandler(uint32_t errCode)
{
    LOG_ERROR("[btle]: hps client error %d", errCode);
}

static void hpsDiscoveryEvtHandler(ble_hps_discovery_evt_t const* pEvt)
{
    if (pEvt->type == BLE_HPS_DISCOVERY_COMPLETE)
    {
        ret_code_t errCode;

        LOG_INFO("[btle]: hps detected");

        // assign handles
        errCode = ble_hps_c_handles_assign(&hpsGattC, pEvt->conn_handle, pEvt->p_db);
        APP_ERROR_CHECK(errCode);

        // enable indications to be able to relay mode changes
        errCode = ble_hps_c_cp_indic_enable(&hpsGattC, pEvt->conn_handle, true);
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[btle]: hps indication enable error %d", errCode);
            APP_ERROR_HANDLER(errCode);
        }
    }
}

static void hpsDiscoveryErrorHandler(uint32_t errCode)
{
    LOG_ERROR("[btle]: hps discovery error %d", errCode);
}

static ret_code_t gattClientInit()
{
    ret_code_t errCode;

    // initialize the gatt queue module
    errCode = ble_gq_init(&gattQueueInst, gqErrorHandler);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[btle]: gatt queue init error %d", errCode);
        return errCode;
    }

    // initialize the helen project service client
    ble_hps_c_init_t hpsInit = {0};
    hpsInit.error_handler = hpsClientErrorHandler;
    hpsInit.evt_handler = NULL;
    hpsInit.p_gq_inst = &gattQueueInst;
    errCode = ble_hps_c_init(&hpsGattC, ARRAY_SIZE(hpsGattC_server_data), &hpsInit);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[btle]: helen project service client init error %d", errCode);
        return errCode;
    }

    // initialize the helen project service discovery
    ble_hps_discovery_init_t hpsDiscInit = {0};
    hpsDiscInit.evt_handler = hpsDiscoveryEvtHandler;
    hpsDiscInit.error_handler = hpsDiscoveryErrorHandler;
    hpsDiscInit.uuid_type = hpsGattC.uuid_type;
    errCode = ble_hps_discovery_init(&hpsDisc, sizeof(hpsDisc.p_conn_handles), &hpsDiscInit);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[btle]: helen discovery init error %d", errCode);
        return errCode;
    }

    // initialize the hid service client module
    ble_hids_c_init_t hidsInit = {0};
    hidsInit.p_gq_inst = &gattQueueInst;
    errCode = ble_hids_c_init(&hidsGattC, &hidsInit);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NO_MEM)
    {
        LOG_ERROR("[btle]: hid service client init error %d", errCode);
        return errCode;
    }

    return NRF_SUCCESS;
}

static ret_code_t gattInit(ble_hps_init_t* pHpsInit, btle_info_t const* pInfo)
{
    ret_code_t errCode;

    errCode = gattServerInit(pHpsInit, pInfo);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gattClientInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static void onConnected(lm_connEvt_t const* pConn)
{
    uint32_t errCode;

    // if device is a remote control, assign handles directly
    if (pConn->pRemDriver != NULL)
    {
        LOG_INFO("[btle]: remote control connected");
        hidsGattC.evt_handler = pConn->pRemDriver->evt_handler;
        ble_hids_c_db_t const* pHandles = pConn->pRemDriver->get_handles();
        errCode = ble_hids_c_handles_assign(&hidsGattC, pConn->connHandle, pHandles);
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[btle]: hid handle assign error %d", errCode);
        }
        errCode = pConn->pRemDriver->notif_enable(&hidsGattC, true);
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[btle]: hid notification enable error %d", errCode);
        }
    }
    // otherwise start database discovery
    else
    {
        errCode = ble_hps_discovery_start(&hpsDisc, pConn->connHandle);
        if (errCode != NRF_SUCCESS)
        {
            LOG_INFO("[btle]: database discovery start error %d", errCode);
        }
    }
}

static void lmEventHandler(lm_evt_t const* pEvt)
{
    switch (pEvt->type)
    {
    case LM_EVT_CONNECTED:
        onConnected(&pEvt->evt.conn);
        break;

    case LM_EVT_ADV_MODE_CHANGED:
    case LM_EVT_SCAN_MODE_CHANGED:
    case LM_EVT_DISCONNECTED:
    case LM_EVT_DELETED:
    default:
        break;
    }

}

static void relayMode(uint16_t connHandle, btle_modeRelay_t const* pRelay)
{
    uint32_t errCode;
    ble_hps_c_cp_write_t hpsCmd;

    // don't relay mode to source handle
    if (pRelay->connHandle == connHandle)
        return;

    hpsCmd.command = BLE_HPS_C_CP_CMD_SET_MODE;
    hpsCmd.params.mode_to_set = pRelay->mode;

    errCode = ble_hps_c_cp_write(&hpsGattC, connHandle, &hpsCmd);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
    {
        LOG_ERROR("[btle]: error %d relaying mode", errCode);
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t btle_Init(btle_init_t const* pInit)
{
    ret_code_t errCode;

    if (pInit == NULL || pInit->eventHandler == NULL)
        return NRF_ERROR_NULL;

    btleEventHandler = pInit->eventHandler;

    bool firmwareUpdated = NRF_POWER->GPREGRET == BOOTLOADER_DFU_END;
    if (firmwareUpdated)    /// TODO: send service changed indication?
        NRF_POWER->GPREGRET = 0;

    errCode = bleStackInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gapParamsInit(BLE_APPEARANCE_GENERIC_CYCLING, pInit->pInfo->pDevicename, pInit->maxDeviceNameLength);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = gattInit(pInit->pHpsInit, pInit->pInfo);
    if (errCode != NRF_SUCCESS)
        return errCode;

    lm_init_t lmInit = {0};
    lmInit.advType = pInit->advType;
    lmInit.uuidType = hpsGattS.uuid_type;
    lmInit.eventHandler = lmEventHandler;
    errCode = lm_Init(&lmInit, &linkManagerBleEventHandler);
        return errCode;

    return NRF_SUCCESS;
}

ret_code_t btle_RelayMode(btle_modeRelay_t const* pRelay)
{
    sdk_mapped_flags_key_list_t connHandles;
    connHandles = ble_conn_state_conn_handles();
    for (uint32_t i = 0; i < connHandles.len; i++)
    {
        relayMode(connHandles.flag_keys[i], pRelay);
    }

    return connHandles.len ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;
}

ret_code_t btle_ReportHpsMeasurements(btle_hpsMeasurement_t const* pData)
{
    if (pData == NULL)
        return NRF_ERROR_NULL;

    ret_code_t errCode = NRF_SUCCESS;
    ble_hps_m_t message;
    memset(&message, 0, sizeof(message));

    message.mode = pData->mode;
    if (pData->outputPower != 0)
    {
        message.flags.output_power_present = 1;
        message.output_power = pData->outputPower;
    }
    if (pData->temperature >= -40 && pData->temperature <= 85)
    {
        message.flags.temperature_present = 1;
        message.temperature = pData->temperature;
    }
    if (pData->inputVoltage != 0)
    {
        message.flags.input_voltage_present = 1;
        message.input_voltage = pData->inputVoltage;;
    }

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(hpsGattS_client_data); i++)
    {
        if (hpsGattS_client_data[i].conn_handle == BLE_CONN_HANDLE_INVALID ||
            hpsGattS_client_data[i].is_m_notfy_enabled == false)
            continue;

        if (errCode == NRF_SUCCESS)
            errCode = ble_hps_measurement_send(&hpsGattS, hpsGattS_client_data[i].conn_handle, &message);
        else
            (void)ble_hps_measurement_send(&hpsGattS, hpsGattS_client_data[i].conn_handle, &message);
    }

    return errCode;
}

/**END OF FILE*****************************************************************/
