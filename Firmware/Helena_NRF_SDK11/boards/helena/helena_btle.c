/**
  ******************************************************************************
  * @file    KD2_service.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define BRD_LOG_ENABLED

#ifdef BRD_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BRD_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "ble_KD2.h"
#include "helena.h"
#include "btle.h"
#include "helena_btle.h"
#include "ble_conn_state.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)   (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/
static uint32_t initService(void* pContext);
static void onBleEvent(void * pContext, ble_evt_t * pBleEvt);

/* Private variables ---------------------------------------------------------*/
static bool isImuPresent;
BLE_KD2_DEF(KD2Gatts, 1);   // for memory reasons only one client is implemented
BTLE_SERVICE_REGISTER(KD2Service, initService, onBleEvent, NULL);
static ble_KD2_cp_op_code_t pendingOpCode;

/* Private read only variables -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void resultHandler(ret_code_t errCode)
{
    ble_KD2_cp_rsp_t rsp;

    if (pendingOpCode == 0 || KD2Gatts.p_client->conn_handle == BLE_CONN_HANDLE_INVALID)
        return; // something went wrong

    rsp.opcode = pendingOpCode;
    rsp.status = errCode == NRF_SUCCESS ? BLE_KD2_CP_RSP_VAL_SUCCESS : BLE_KD2_CP_RSP_VAL_FAILED;

    errCode = ble_KD2_cp_response(&KD2Gatts, KD2Gatts.p_client->conn_handle, &rsp);   /// TODO: try again in case of error?
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending response in result handler", errCode);

    pendingOpCode = 0;
}

static void onChannelSetupRequest(uint16_t connHandle, uint8_t channel)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;
    hln_channelSetup_t channelSetup;
    int32_t conv;

    rsp.opcode = BLE_KD2_CP_OP_REQ_CHN_CONFIG;

    errCode = hln_GetChannelSetup(&channelSetup, channel);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.channel_config.channel = channel;
        conv = channelSetup.outputPower * 1000l;
        conv = (conv + 512) >> 10;
        rsp.params.channel_config.config.output_power = conv;
        conv = channelSetup.outputLimit * 100l;
        conv = (conv + (1<<15)) >> 16;
        rsp.params.channel_config.config.output_limit = conv;
        if (channelSetup.optic.type >= PRG_TYPE_CNT)
            rsp.params.channel_config.config.optic_type = BLE_KD2_OPTIC_NA;
        else
            rsp.params.channel_config.config.optic_type = channelSetup.optic.type - PRG_TYPE_15 + BLE_KD2_OPTIC_15D;
        conv = channelSetup.optic.offset * 36000l;
        conv = conv >= 0 ? conv + 16384 : conv - 16384; // rounding away from zero
        rsp.params.channel_config.config.optic_offset = conv / 32768;
    }
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        LOG_ERROR("[BRD]: error %d getting channel setup", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending channel setup response", errCode);
}

static void onChannelSetupSet(uint16_t connHandle, uint8_t channel, ble_KD2_channel_config_t const* pChannelSetup)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;
    hln_channelSetup_t channelSetup;
    int32_t conv;

    rsp.opcode = BLE_KD2_CP_OP_SET_CHN_CONFIG;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    if (pendingOpCode == 0)
    {
        conv = (int32_t)pChannelSetup->output_power << 10;
        channelSetup.outputPower = (conv + 500) / 1000;
        conv = (int32_t)pChannelSetup->output_limit << 16;
        channelSetup.outputLimit = (conv + 50) / 100;
        if (pChannelSetup->optic_type == BLE_KD2_OPTIC_NA)
            channelSetup.optic.type = PRG_TYPE_NA;
        else
            channelSetup.optic.type = pChannelSetup->optic_type - BLE_KD2_OPTIC_15D + PRG_TYPE_15;
        conv = pChannelSetup->optic_offset * 32768l;
        conv = conv >= 0 ? conv + 180 : conv - 180; // rounding away from zero
        channelSetup.optic.offset = conv / 36000;

        errCode = hln_SetChannelSetup(&channelSetup, channel, resultHandler);
        if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_SUCCESS)
        {
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending channel setup response", errCode);
}

static void onComPinRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;
    hln_comPinMode_t comPinMode;

    rsp.opcode = BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG;

    errCode = hln_GetComPinMode(&comPinMode);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.com_pin = comPinMode;
    }
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        LOG_ERROR("[BRD]: error %d getting com pin mode", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending com pin response", errCode);
}

static void onComPinSet(uint16_t connHandle, ble_KD2_com_pin_t comPinMode)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_SET_COM_PIN_CONFIG;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    if (pendingOpCode == 0)
    {
        errCode = hln_SetComPinMode(comPinMode, resultHandler);
        if (errCode == NRF_ERROR_INVALID_PARAM)
            rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
        else if (errCode == NRF_ERROR_NOT_SUPPORTED)
            rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;
        else if (errCode == NRF_SUCCESS)
        {
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending com pin response", errCode);
}

static void onIntCompRequest(uint16_t connHandle)
{
    // not enabled, should not happen

    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_REQ_INT_COMP;
    rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending internal compensation response", errCode);
}

static void onIntCompSet(uint16_t connHandle, ble_KD2_int_comp_t const *pComp)
{
    // should not happen

    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_SET_INT_COMP;
    rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending internal compensation response", errCode);
}

static void onExtCompRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;
    hbd_calibData_t comp;

    rsp.opcode = BLE_KD2_CP_OP_REQ_EXT_COMP;

    errCode = hln_GetCompensation(&comp, 0);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.ext_comp.temperature_offset = comp.temperatureOffset;
        rsp.params.ext_comp.left_current_gain = comp.gainLeft;
        rsp.params.ext_comp.right_current_gain = comp.gainRight;
    }
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        LOG_ERROR("[BRD]: error %d getting external compensation", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending external compensation response", errCode);
}

static void onExtCompSet(uint16_t connHandle, ble_KD2_ext_comp_t const *pComp)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;
    hbd_calibData_t comp;

    rsp.opcode = BLE_KD2_CP_OP_SET_EXT_COMP;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    comp.temperatureOffset = pComp->temperature_offset;
    comp.gainLeft = pComp->left_current_gain;
    comp.gainRight = pComp->right_current_gain;

    errCode = hln_SetCompensation(&comp, 0, resultHandler);
    if (errCode == NRF_ERROR_INVALID_PARAM)
        rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;
    else if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        return;
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending external compensation response", errCode);
}

static void onImuStateRequest(uint16_t connHandle)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;
    bool isCalibrated;

    rsp.opcode = BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE;

    errCode = hln_IsImuCalibrated(&isCalibrated);
    if (errCode == NRF_SUCCESS)
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_SUCCESS;
        rsp.params.imu_calib_state = isCalibrated;
    }
    else if (errCode == NRF_ERROR_NOT_SUPPORTED)
        rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;
    else
    {
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;
        LOG_ERROR("[BRD]: error %d imu calib state", errCode);
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending com pin response", errCode);
}

static void onImuCalibStart(uint16_t connHandle)
{
    ret_code_t errCode;
    ble_KD2_cp_rsp_t rsp;

    rsp.opcode = BLE_KD2_CP_OP_START_IMU_CALIB;
    rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

    if (pendingOpCode == 0)
    {
        errCode = hln_CalibrateImu(resultHandler);
        if (errCode == NRF_ERROR_NOT_SUPPORTED)
            rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;
        else if (errCode == NRF_SUCCESS)
        {
            pendingOpCode = rsp.opcode;
            return; // response is sent in result handler
        }
    }

    errCode = ble_KD2_cp_response(&KD2Gatts, connHandle, &rsp);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[BRD]: error %d sending imu calib start response", errCode);
}

static void eventHandler(ble_KD2_evt_t const *pEvt)
{
    if (pEvt->evt_type != BLE_KD2_EVT_CP_EVT)
        return;

    if (pendingOpCode != 0)
    {
        ble_KD2_cp_rsp_t rsp;
        ret_code_t errCode;

        rsp.opcode = BLE_KD2_CP_OP_SET_EXT_COMP;
        rsp.status = BLE_KD2_CP_RSP_VAL_FAILED;

        errCode = ble_KD2_cp_response(&KD2Gatts, pEvt->p_client->conn_handle, &rsp);
        if (errCode != NRF_SUCCESS)
            LOG_ERROR("[BRD]: error %d sending response", errCode);
    }

    /// TODO: check is already something is ongoing and ic conn handle match

    switch (pEvt->cp_evt)
    {
    case BLE_KD2_CP_OP_REQ_CHN_CONFIG:
        onChannelSetupRequest(pEvt->p_client->conn_handle, pEvt->p_params->channel_config.channel);
        break;

    case BLE_KD2_CP_OP_SET_CHN_CONFIG:
        onChannelSetupSet(pEvt->p_client->conn_handle, pEvt->p_params->channel_config.channel, &pEvt->p_params->channel_config.config);
        break;

    case BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG:
        onComPinRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_SET_COM_PIN_CONFIG:
        onComPinSet(pEvt->p_client->conn_handle, pEvt->p_params->com_pin);
        break;

    case BLE_KD2_CP_OP_REQ_INT_COMP:
        onIntCompRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_SET_INT_COMP:
        onIntCompSet(pEvt->p_client->conn_handle, &pEvt->p_params->int_comp);
        break;

    case BLE_KD2_CP_OP_REQ_EXT_COMP:
        onExtCompRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_SET_EXT_COMP:
        onExtCompSet(pEvt->p_client->conn_handle, &pEvt->p_params->ext_comp);
        break;

    case BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE:
        onImuStateRequest(pEvt->p_client->conn_handle);
        break;

    case BLE_KD2_CP_OP_START_IMU_CALIB:
        onImuCalibStart(pEvt->p_client->conn_handle);
        break;

    default:
        break;
    }
}

static void errorHandler(uint32_t errCode)
{
    LOG_ERROR("[brd]: KD2 service error %d", errCode);
}

static uint32_t initService(void* pContext)
{
    ble_KD2_init_t kd2Init = {0};

    kd2Init.feature.config.channel_config_supported = 1;
    kd2Init.feature.config.com_pin_mode_supported   = 1;
    kd2Init.feature.config.imu_calib_supported      = isImuPresent ? 1 : 0;
    kd2Init.feature.channel.adaptive_supported      = isImuPresent ? 1 : 0;

#ifdef DEBUG_EXT
    // compensation only supported in debug mode
    kd2Init.feature.config.internal_comp_supported  = 0;
    kd2Init.feature.config.external_comp_supported  = 1;

#pragma message ( "debug configuration set to just works temporary" )
    kd2Init.KD2_cp_wr_sec      = SEC_JUST_WORKS;//SEC_OPEN;
    kd2Init.KD2_cp_cccd_wr_sec = SEC_JUST_WORKS;//SEC_OPEN;
#else
    kd2Init.KD2_cp_wr_sec      = SEC_JUST_WORKS;
    kd2Init.KD2_cp_cccd_wr_sec = SEC_JUST_WORKS;
#endif // DEBUG_EXT
    kd2Init.KD2_f_rd_sec       = SEC_OPEN;

    kd2Init.error_handler = errorHandler;
    kd2Init.evt_handler   = eventHandler;

    return ble_KD2_init(&KD2Gatts, ARRAY_SIZE(KD2Gatts_client_data), &kd2Init);
}

static void onBleEvent(void* pContext, ble_evt_t* pBleEvt)
{
    /// TODO: does this (conn_handle) work for all events?
    if (ble_conn_state_role(pBleEvt->evt.gap_evt.conn_handle) == BLE_GAP_ROLE_PERIPH)
        ble_KD2_on_ble_evt(&KD2Gatts, pBleEvt);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t hln_btle_Init(bool isImuAvailable)
{
    isImuPresent = isImuAvailable;

    // ble isn't initialized yet, impossible to add service here
    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/

