/**
  ******************************************************************************
  * @file    ble_lcs_ctrlpt.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/10
  * @brief   Light Control Service Control Point implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_lcs.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define BLE_UUID_LCS_LCP_CHARACTERISTIC 0x0104   /**< The UUID of the light Control Point characteristic */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    ble_lcs_ctrlpt_op_code_t   opcode;
    ble_lcs_ctrlpt_write_val_t params;
} ble_lcs_ctrlpt_val_t;

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**@brief sends a control point indication.
 *
 * @param[in]   p_lcs_ctrlpt      LC Ctrlpt structure.
 */
static void lcs_ctrlpt_resp_send(ble_lcs_ctrlpt_t * p_lcs_ctrlpt)
{
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    uint32_t               err_code;

    if ((p_lcs_ctrlpt->procedure_status == BLE_LCS_CTRLPT_RSP_CODE_IND_PENDING))
    {
        hvx_len = p_lcs_ctrlpt->response.len;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_lcs_ctrlpt->lc_ctrlpt_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = p_lcs_ctrlpt->response.encoded_rsp;

        err_code = sd_ble_gatts_hvx(p_lcs_ctrlpt->conn_handle, &hvx_params);

        // Error handling
        if ((err_code == NRF_SUCCESS) && (hvx_len != p_lcs_ctrlpt->response.len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        switch (err_code)
        {
            case NRF_SUCCESS:
                p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_RSP_CODE_IND_CONFIRM_PENDING;
                // Wait for HVC event
                break;

            case BLE_ERROR_NO_TX_BUFFERS:
                // Wait for TX_COMPLETE event to retry transmission
                p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_RSP_CODE_IND_PENDING;
                break;

            default:
                // Report error to application
                p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_PROC_STATUS_FREE;
                if (p_lcs_ctrlpt->error_handler != NULL)
                {
                    p_lcs_ctrlpt->error_handler(err_code);
                }
                break;
        }
    }
}

/**@brief check if the cccd is configured
 *
 * @param[in]   p_lcs_ctrlpt      SC Ctrlpt structure.
 * @return  true if the sc_control point's cccd is correctly configured, false otherwise.
 */
static bool is_cccd_configured(ble_lcs_ctrlpt_t * p_lcs_ctrlpt)
{
    uint32_t err_code;
    uint8_t  cccd_value_buf[BLE_CCCD_VALUE_LEN];
    bool     is_cccp_indic_enabled = false;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = BLE_CCCD_VALUE_LEN;
    gatts_value.offset  = 0;
    gatts_value.p_value = cccd_value_buf;

    err_code = sd_ble_gatts_value_get(p_lcs_ctrlpt->conn_handle,
                                      p_lcs_ctrlpt->lc_ctrlpt_handles.cccd_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application
        if (p_lcs_ctrlpt->error_handler != NULL)
        {
            p_lcs_ctrlpt->error_handler(err_code);
        }
    }

    is_cccp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);

    return is_cccp_indic_enabled;
}

/**@brief Decode an incoming control point write.
 *
 * @param[in]    rcvd_val       received write value
 * @param[in]    len            value length
 * @param[out]   decoded_ctrlpt decoded control point structure
 */
static uint32_t lcs_ctrlpt_decode(uint8_t             * p_rcvd_val,
                                 uint8_t               len,
                                 ble_lcs_ctrlpt_val_t * p_write_val)
{
    uint8_t pos = 0;

    if (len < BLE_LCS_CTRLPT_MIN_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    p_write_val->opcode = (ble_lcs_ctrlpt_op_code_t)p_rcvd_val[pos++];

    switch (p_write_val->opcode)
    {
        case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT:
            break;

        case BLE_LCS_CTRLPT_OP_CODE_REQ_GRP_CNFG:
            break;

        case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNFG:
            p_write_val->params.mode_list_start = p_rcvd_val[pos];
            break;

        case BLE_LCS_CTRLPT_OP_CODE_SET_MODE:
            p_write_val->params.set_mode = p_rcvd_val[pos];
            break;

        case BLE_LCS_CTRLPT_OP_CODE_CNFG_MODE:
            if (len - 2 > BLE_LCS_CTRLPT_MAX_NUM_OF_MODES * 2)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            p_write_val->params.mode_config.mode_number_start = p_rcvd_val[pos++];
            p_write_val->params.mode_config.mode_entries = 0;
            for (uint_fast8_t i = 0; pos < len; i++)
            {
                p_write_val->params.mode_config.config[i].type = p_rcvd_val[pos++];
                p_write_val->params.mode_config.config[i].intensity = p_rcvd_val[pos++];
                p_write_val->params.mode_config.mode_entries++;
            }
            break;

        case BLE_LCS_CTRLPT_OP_CODE_CNFG_GROUP:
            p_write_val->params.group_config = p_rcvd_val[pos];
            break;

        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    return NRF_SUCCESS;
}

/**@brief encode a control point response indication.
 *
 * @param[in]   p_lcs_ctrlpt      LC Ctrlpt structure.
 * @param[in]   p_ctrlpt_rsp  structure containing response data to be encoded
 * @param[out]  p_data        pointer where data needs to be written
 * @return                    size of encoded data
 */
static int ctrlpt_rsp_encode(ble_lcs_ctrlpt_t      * p_lcs_ctrlpt,
                             const ble_lcs_ctrlpt_rsp_t  * p_ctrlpt_rsp,
                             uint8_t               * p_data)
{
    uint8_t len = 0;

    p_data[len++] = BLE_LCS_CTRLPT_OP_CODE_RESPONSE;
    p_data[len++] = p_ctrlpt_rsp->opcode;
    p_data[len++] = p_ctrlpt_rsp->status;

    if (p_ctrlpt_rsp->status == BLE_LCS_CTRLPT_RSP_CODE_SUCCESS)
    {
        switch (p_ctrlpt_rsp->opcode)
        {
            case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT:
                p_data[len++] = p_ctrlpt_rsp->params.mode_cnt;
                break;
            case BLE_LCS_CTRLPT_OP_CODE_REQ_GRP_CNFG:
                p_data[len++] = p_ctrlpt_rsp->params.group_config;
                break;
            case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNFG:
                for (uint8_t i = 0; i < p_ctrlpt_rsp->params.mode_config_list.num_of_entries; i++)
                {
                    p_data[len++] = p_ctrlpt_rsp->params.mode_config_list.p_list[i].type;
                    p_data[len++] = p_ctrlpt_rsp->params.mode_config_list.p_list[i].intensity;
                }
                break;

            default:
                // No implementation needed.
                break;
        }
    }
    return len;
}

static bool is_feature_supported(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_lcs_ctrlpt_op_code_t op_code)
{
    bool supported = false;

    switch (op_code)
    {
        case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT:
           supported = true;
           break;

        case BLE_LCS_CTRLPT_OP_CODE_REQ_GRP_CNFG:
            if (p_lcs_ctrlpt->supported_features.mode_grouping_supported == 1)
            {
                supported = true;
            }
            break;

        case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNFG:
            supported = true;
            break;

        case BLE_LCS_CTRLPT_OP_CODE_SET_MODE:
            if (p_lcs_ctrlpt->supported_features.mode_change_supported == 1)
            {
                supported = true;
            }
            break;

        case BLE_LCS_CTRLPT_OP_CODE_CNFG_MODE:
            if (p_lcs_ctrlpt->supported_features.mode_config_supported == 1)
            {
                supported = true;
            }
            break;

        case BLE_LCS_CTRLPT_OP_CODE_CNFG_GROUP:
            if (p_lcs_ctrlpt->supported_features.mode_grouping_supported == 1)
            {
                supported = true;
            }
            break;

        default:
            break;
    }

    return supported;
}

/**@brief Handle a write event to the Speed and Cadence Control Point.
 *
 * @param[in]   p_lcs_ctrlpt      LC Ctrlpt structure.
 * @param[in]   p_evt_write      WRITE event to be handled.
 */
static void on_ctrlpt_write(ble_lcs_ctrlpt_t       * p_lcs_ctrlpt,
                            ble_gatts_evt_write_t * p_evt_write)
{
    //ble_sc_ctrlpt_val_t                   rcvd_ctrlpt =
    //{ BLE_SCPT_RESPONSE_CODE , 0, BLE_SENSOR_LOCATION_OTHER };

    //ble_sc_ctrlpt_rsp_t                   rsp;
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
    //ble_sc_ctrlpt_evt_t                   evt;

    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    //auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;

    if (is_cccd_configured(p_lcs_ctrlpt))
    {
        if (p_lcs_ctrlpt->procedure_status == BLE_LCS_CTRLPT_PROC_STATUS_FREE)
        {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
        }
        else
        {
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_CPS_PROC_ALR_IN_PROG;
        }
    }
    else
    {
        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_CPS_CCCD_CONFIG_ERROR;
    }

    err_code = sd_ble_gatts_rw_authorize_reply(p_lcs_ctrlpt->conn_handle, &auth_reply);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application.
        if (p_lcs_ctrlpt->error_handler != NULL)
        {
            p_lcs_ctrlpt->error_handler(err_code);
        }
    }

    if (auth_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS)
    {
        return;
    }

    ble_lcs_ctrlpt_val_t rcvd_ctrlpt;
    err_code = lcs_ctrlpt_decode(p_evt_write->data, p_evt_write->len, &rcvd_ctrlpt);

    if (err_code != NRF_SUCCESS)
    {
        // Report error to application.
        if (p_lcs_ctrlpt->error_handler != NULL)
        {
            p_lcs_ctrlpt->error_handler(err_code);
        }
        return;
    }

    p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_PROC_IN_PROGRESS;

    if (p_lcs_ctrlpt->evt_handler != NULL && is_feature_supported(p_lcs_ctrlpt, rcvd_ctrlpt.opcode))
    {
        ble_lcs_ctrlpt_evt_t evt;

        switch (rcvd_ctrlpt.opcode)
        {
            case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNT:
                evt.evt_type = BLE_LCS_CTRLPT_EVT_REQ_MODE_CNT;
                evt.p_params = &rcvd_ctrlpt.params;
                p_lcs_ctrlpt->evt_handler(p_lcs_ctrlpt, &evt);
                break;

            case BLE_LCS_CTRLPT_OP_CODE_REQ_GRP_CNFG:
                evt.evt_type = BLE_LCS_CTRLPT_EVT_REQ_GRP_CNFG;
                evt.p_params = &rcvd_ctrlpt.params;
                p_lcs_ctrlpt->evt_handler(p_lcs_ctrlpt, &evt);
                break;

            case BLE_LCS_CTRLPT_OP_CODE_REQ_MODE_CNFG:
                evt.evt_type = BLE_LCS_CTRLPT_EVT_REQ_MODE_CNFG;
                evt.p_params = &rcvd_ctrlpt.params;
                p_lcs_ctrlpt->evt_handler(p_lcs_ctrlpt, &evt);
                break;

            case BLE_LCS_CTRLPT_OP_CODE_SET_MODE:
                evt.evt_type = BLE_LCS_CTRLPT_EVT_SET_MODE;
                evt.p_params = &rcvd_ctrlpt.params;
                p_lcs_ctrlpt->evt_handler(p_lcs_ctrlpt, &evt);
                break;

            case BLE_LCS_CTRLPT_OP_CODE_CNFG_MODE:
                evt.evt_type = BLE_LCS_CTRLPT_EVT_CNFG_MODE;
                evt.p_params = &rcvd_ctrlpt.params;
                p_lcs_ctrlpt->evt_handler(p_lcs_ctrlpt, &evt);
                break;

            case BLE_LCS_CTRLPT_OP_CODE_CNFG_GROUP:
                evt.evt_type = BLE_LCS_CTRLPT_EVT_CNFG_GROUP;
                evt.p_params = &rcvd_ctrlpt.params;
                p_lcs_ctrlpt->evt_handler(p_lcs_ctrlpt, &evt);
                break;

            default:
                break;
        }
    }
    else
    {
        ble_lcs_ctrlpt_rsp_t rsp;
        rsp.status = BLE_LCS_CTRLPT_RSP_CODE_NOT_SUPPORTED;
        rsp.opcode = rcvd_ctrlpt.opcode;
        err_code = ble_lcs_ctrlpt_mode_resp(p_lcs_ctrlpt, &rsp);
        if (err_code != NRF_SUCCESS)
        {
            // Report error to application.
            if (p_lcs_ctrlpt->error_handler != NULL)
            {
                p_lcs_ctrlpt->error_handler(err_code);
            }
        }
    }
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_lcs_ctrlpt  LC Ctrlpt structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_evt_t * p_ble_evt)
{
    p_lcs_ctrlpt->conn_handle      = p_ble_evt->evt.gap_evt.conn_handle;
    p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_PROC_STATUS_FREE;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_lcs_ctrlpt  LC Ctrlpt structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_evt_t * p_ble_evt)
{
    (void)p_ble_evt;
    p_lcs_ctrlpt->conn_handle      = BLE_CONN_HANDLE_INVALID;
    p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_PROC_STATUS_FREE;
}

/**@brief Authorize WRITE request event handler.
 *
 * @details Handles WRITE events from the BLE stack.
 *
 * @param[in]   p_lcs_ctrlpt LC Ctrlpt structure.
 * @param[in]   p_gatts_evt  GATTS Event received from the BLE stack.
 *
 */
static void on_rw_authorize_request(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_gatts_evt_t * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t * p_auth_req = &p_gatts_evt->params.authorize_request;
    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if (   (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_PREP_WRITE_REQ)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW)
            && (p_gatts_evt->params.authorize_request.request.write.op
                != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL)
           )
        {
            if (p_auth_req->request.write.handle == p_lcs_ctrlpt->lc_ctrlpt_handles.value_handle)
            {
                on_ctrlpt_write(p_lcs_ctrlpt, &p_auth_req->request.write);
            }
        }
    }
}

/**@brief Function for handling the BLE_GATTS_EVT_HVC event.
 *
 * @param[in]   p_lcs_ctrlpt  LC Ctrlpt structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_sc_hvc_confirm(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_evt_t * p_ble_evt)
{
    if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_lcs_ctrlpt->lc_ctrlpt_handles.value_handle)
    {
        if (p_lcs_ctrlpt->procedure_status == BLE_LCS_CTRLPT_RSP_CODE_IND_CONFIRM_PENDING)
        {
            p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_PROC_STATUS_FREE;
        }
    }
}

/**@brief Tx Complete event handler.
 *
 * @details Tx Complete event handler.
 *          Handles WRITE events from the BLE stack and if an indication was pending try sending it
 *          again.
 *
 * @param[in]   p_lcs_ctrlpt  LC Ctrlpt structure.
 */
static void on_tx_complete(ble_lcs_ctrlpt_t * p_lcs_ctrlpt)
{
    if (p_lcs_ctrlpt->procedure_status == BLE_LCS_CTRLPT_RSP_CODE_IND_PENDING)
    {
        lcs_ctrlpt_resp_send(p_lcs_ctrlpt);
    }
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_lcs_ctrlpt_init(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_lcs_ctrlpt_init_t * p_lcs_ctrlpt_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    p_lcs_ctrlpt->uuid_type          = p_lcs_ctrlpt_init->uuid_type;
    p_lcs_ctrlpt->supported_features = p_lcs_ctrlpt_init->supported_features;
    p_lcs_ctrlpt->service_handle     = p_lcs_ctrlpt_init->service_handle;
    p_lcs_ctrlpt->conn_handle        = BLE_CONN_HANDLE_INVALID;
    p_lcs_ctrlpt->evt_handler        = p_lcs_ctrlpt_init->evt_handler;
    p_lcs_ctrlpt->procedure_status   = BLE_LCS_CTRLPT_PROC_STATUS_FREE;
    p_lcs_ctrlpt->error_handler      = p_lcs_ctrlpt_init->error_handler;

    memset(&cccd_md, 0, sizeof(ble_gatts_attr_md_t));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_lcs_ctrlpt_init->lc_ctrlpt_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));
    char_md.char_props.indicate = 1;
    char_md.char_props.write    = 1;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;

    ble_uuid.type = p_lcs_ctrlpt->uuid_type;
    ble_uuid.uuid = BLE_UUID_LCS_LCP_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(ble_gatts_attr_md_t));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    attr_md.write_perm = p_lcs_ctrlpt_init->lc_ctrlpt_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 1;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(ble_gatts_attr_t));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 0;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_LCS_CTRLPT_MAX_LEN;
    attr_char_value.p_value   = 0;

    return sd_ble_gatts_characteristic_add(p_lcs_ctrlpt->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lcs_ctrlpt->lc_ctrlpt_handles);
}

void ble_lcs_ctrlpt_on_ble_evt(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lcs_ctrlpt, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_lcs_ctrlpt, p_ble_evt);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            on_rw_authorize_request(p_lcs_ctrlpt, &p_ble_evt->evt.gatts_evt);
            break;

        case BLE_GATTS_EVT_HVC:
            on_sc_hvc_confirm(p_lcs_ctrlpt, p_ble_evt);
            break;

        case BLE_EVT_TX_COMPLETE:
            on_tx_complete(p_lcs_ctrlpt);
            break;

        default:
            break;
    }
}

uint32_t ble_lcs_ctrlpt_mode_resp(ble_lcs_ctrlpt_t * p_lcs_ctrlpt, const ble_lcs_ctrlpt_rsp_t * p_rsps)
{
    if (p_lcs_ctrlpt == NULL || p_rsps == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_lcs_ctrlpt->procedure_status != BLE_LCS_CTRLPT_PROC_IN_PROGRESS)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_lcs_ctrlpt->procedure_status = BLE_LCS_CTRLPT_RSP_CODE_IND_PENDING;

    p_lcs_ctrlpt->response.len = ctrlpt_rsp_encode(p_lcs_ctrlpt, p_rsps,
                                                  p_lcs_ctrlpt->response.encoded_rsp);

    lcs_ctrlpt_resp_send(p_lcs_ctrlpt);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/



