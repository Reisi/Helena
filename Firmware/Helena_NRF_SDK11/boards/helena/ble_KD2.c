/**
  ******************************************************************************
  * @file    ble_KD2.c
  * @author  Thomas Reisnecker
  * @brief   KD2 Control Service implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_hps.h"
#include "ble_KD2.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define BLE_UUID_KD2_SERVICE            0x0501  /**< The UUID of the KD2 feature characteristic */
#define BLE_UUID_KD2_F_CHARACTERISTIC   0x0502  /**< The UUID of the KD2 feature characteristic */
#define BLE_UUID_KD2_CP_CHARACTERISTIC  0x0503  /**< The UUID of the KD2 control point characteristic */

#define KD2_CFG_F_CHANNEL_CONFIG_MASK   (1 << 0)
#define KD2_CFG_F_COM_PIN_MASK          (1 << 1)
#define KD2_CFG_F_INT_COMP_MASK         (1 << 2)
#define KD2_CFG_F_EXT_COMP_MASK         (1 << 3)
#define KD2_CFG_F_IMU_CALIB_MASK        (1 << 4)

#define KD2_CHN_F_ADAPTIVE_MASK         (1 << 0)

#define SIZE_OF_CONFIG                  7
#define SIZE_OF_COMPIN                  1
#define SIZE_OF_INT_COMP                12
#define SIZE_OF_EXT_COMP                4

/* Private macros -----------------------------------------------------------*/
#define VERIFY_LEN(actual, target)      \
do                                      \
{                                       \
    if (target != actual)               \
    {                                   \
        return NRF_ERROR_INVALID_PARAM; \
    }                                   \
} while (0)

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    ble_KD2_cp_op_code_t opcode;
    ble_KD2_cp_params_t  params;
} cp_val_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t m_num_of_clients;
security_req_t f_rd_sec, cp_wr_sec, cp_cccd_wr_sec;

/* Private functions ---------------------------------------------------------*/
static uint32_t feature_char_add(ble_KD2_t * p_KD2, security_req_t rd_sec)
{
    ble_add_char_params_t add_char_params;
    uint8_t config_flags = 0;
    uint8_t channel_flags = 0;
    uint8_t init_value_encoded[2];

    if (p_KD2->feature.config.channel_config_supported)
    {
        config_flags |= KD2_CFG_F_CHANNEL_CONFIG_MASK;
    }
    if (p_KD2->feature.config.com_pin_mode_supported)
    {
        config_flags |= KD2_CFG_F_COM_PIN_MASK;
    }
    if (p_KD2->feature.config.internal_comp_supported)
    {
        config_flags |= KD2_CFG_F_INT_COMP_MASK;
    }
    if (p_KD2->feature.config.external_comp_supported)
    {
        config_flags |= KD2_CFG_F_EXT_COMP_MASK;
    }
    if (p_KD2->feature.config.imu_calib_supported)
    {
        config_flags |= KD2_CFG_F_IMU_CALIB_MASK;
    }
    init_value_encoded[0] = config_flags;

    if (p_KD2->feature.channel.adaptive_supported)
    {
        channel_flags |= KD2_CHN_F_ADAPTIVE_MASK;
    }
    init_value_encoded[1] = channel_flags;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_KD2_F_CHARACTERISTIC;
    add_char_params.uuid_type       = p_KD2->uuid_type;
    add_char_params.init_len        = sizeof(init_value_encoded);
    add_char_params.max_len         = sizeof(init_value_encoded);
    add_char_params.p_init_value    = init_value_encoded;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = rd_sec;

    return characteristic_add(p_KD2->service_handle, &add_char_params, &p_KD2->f_handles);
}

static uint32_t control_point_char_add(ble_KD2_t * p_KD2, security_req_t wr_sec, security_req_t cccd_wr_sec)
{
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_UUID_KD2_CP_CHARACTERISTIC;
    add_char_params.uuid_type           = p_KD2->uuid_type;
    add_char_params.max_len             = BLE_KD2_CP_MAX_LEN;
    add_char_params.is_var_len          = true;
    add_char_params.char_props.indicate = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.is_defered_write    = true;
    add_char_params.write_access        = wr_sec;
    add_char_params.cccd_write_access   = cccd_wr_sec;

    return characteristic_add(p_KD2->service_handle, &add_char_params, &p_KD2->cp_handles);
}

static ble_KD2_client_spec_t * get_client_data_by_conn_handle(uint16_t conn_handle, ble_KD2_client_spec_t * p_context)
{
    uint_fast8_t index = m_num_of_clients;

    while (index--)
    {
        if (p_context[index].conn_handle == conn_handle)
        {
            return &p_context[index];
        }
    }

    return NULL;
}

static bool is_indication_enabled(ble_KD2_t const * p_KD2, uint16_t conn_handle)
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

    err_code = sd_ble_gatts_value_get(conn_handle, p_KD2->cp_handles.cccd_handle, &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application
        if (p_KD2->error_handler != NULL)
        {
            p_KD2->error_handler(err_code);
        }
        return false;
    }

    is_cccp_indic_enabled = ble_srv_is_indication_enabled(cccd_value_buf);

    return is_cccp_indic_enabled;
}

static void on_connect(ble_KD2_t const * p_KD2, ble_evt_t const * p_ble_evt)
{
    ble_KD2_evt_t           evt;
    ble_KD2_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(BLE_CONN_HANDLE_INVALID, p_KD2->p_client);
    if (p_client == NULL)
    {
        if (p_KD2->error_handler != NULL)
        {
            p_KD2->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }
    p_client->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    if (p_KD2->evt_handler)
    {
        return;
    }

    // check the hosts CCCD value to inform the application if it has to send indications
   if (is_indication_enabled(p_KD2, p_ble_evt->evt.gap_evt.conn_handle))
    {
        p_client->is_KD2cp_indic_enabled = true;

        memset(&evt, 0, sizeof(ble_KD2_evt_t));
        evt.evt_type    = BLE_KD2_EVT_CP_INDICATION_ENABLED;
        evt.p_KD2     = p_KD2;
        evt.p_client    = p_client;

        p_KD2->evt_handler(&evt);
    }
}

static void on_disconnect(ble_KD2_t * p_KD2, ble_evt_t const * p_ble_evt)
{
    ble_KD2_evt_t evt;
    ble_KD2_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(p_ble_evt->evt.gap_evt.conn_handle, p_KD2->p_client);
    if (p_client == NULL)
    {
        return;
    }

    if (p_client->is_KD2cp_indic_enabled)
    {
        p_client->is_KD2cp_indic_enabled = false;

        if (p_KD2->evt_handler != NULL)
        {
            memset(&evt, 0, sizeof(ble_KD2_evt_t));
            evt.evt_type    = BLE_KD2_EVT_CP_INDICATION_DISABLED;
            evt.p_KD2     = p_KD2;
            evt.p_client    = p_client;

            p_KD2->evt_handler(&evt);
        }
    }

    p_client->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_client->procedure_status = BLE_KD2_CP_PROC_STATUS_FREE;
}

static void on_cp_cccd_write(ble_KD2_t * p_KD2, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    ble_KD2_evt_t evt;
    ble_KD2_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_KD2->p_client);
    if (p_client == NULL)
    {
        if (p_KD2->error_handler != NULL)
        {
            p_KD2->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_evt_write->len == 2)
    {
        // CCCD written, update indication state
        memset(&evt, 0, sizeof(ble_KD2_evt_t));
        evt.p_KD2     = p_KD2;
        evt.p_client    = p_client;


        if (ble_srv_is_indication_enabled(p_evt_write->data))
        {
            p_client->is_KD2cp_indic_enabled = true;
            evt.evt_type = BLE_KD2_EVT_CP_INDICATION_ENABLED;
        }
        else
        {
            p_client->is_KD2cp_indic_enabled = false;
            evt.evt_type = BLE_KD2_EVT_CP_INDICATION_DISABLED;
        }

        if (p_KD2->evt_handler != NULL)
        {
            p_KD2->evt_handler(&evt);
        }
    }
}

static void on_write(ble_KD2_t * p_KD2, ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_gatts_evt->params.write;

    if (p_evt_write->handle == p_KD2->cp_handles.cccd_handle)
    {
        on_cp_cccd_write(p_KD2, p_evt_write, p_gatts_evt->conn_handle);
    }
}

uint32_t cp_decode(uint8_t const   * p_rcvd_val,
                   uint8_t           len,
                   ble_KD2_t const * p_KD2,
                   cp_val_t        * p_val)
{
   uint8_t pos = 0;

   if (len < BLE_KD2_CP_MIN_LEN)
   {
       return NRF_ERROR_INVALID_PARAM;
   }

   p_val->opcode = (ble_KD2_cp_op_code_t)p_rcvd_val[pos++];

   switch (p_val->opcode)
   {
    case BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG:
    case BLE_KD2_CP_OP_REQ_INT_COMP:
    case BLE_KD2_CP_OP_REQ_EXT_COMP:
    case BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE:
    case BLE_KD2_CP_OP_START_IMU_CALIB:
        VERIFY_LEN(len, 1);
        return NRF_SUCCESS;

    case BLE_KD2_CP_OP_REQ_CHN_CONFIG:
        VERIFY_LEN(len, 2);
        p_val->params.channel_config.channel = p_rcvd_val[pos++];
        return NRF_SUCCESS;

    case BLE_KD2_CP_OP_SET_CHN_CONFIG:
        VERIFY_LEN(len, 1 + SIZE_OF_CONFIG);
        p_val->params.channel_config.channel             = p_rcvd_val[pos++];
        p_val->params.channel_config.config.output_power = uint16_decode(&p_rcvd_val[pos]);
        pos += 2;
        p_val->params.channel_config.config.output_limit = p_rcvd_val[pos++];
        p_val->params.channel_config.config.optic_type   = p_rcvd_val[pos++];
        p_val->params.channel_config.config.optic_offset = uint16_decode(&p_rcvd_val[pos]);
        pos += 2;
        return NRF_SUCCESS;

    case BLE_KD2_CP_OP_SET_COM_PIN_CONFIG:
        VERIFY_LEN(len, 1 + SIZE_OF_COMPIN);
        p_val->params.com_pin = p_rcvd_val[pos];
        return NRF_SUCCESS;

    case BLE_KD2_CP_OP_SET_INT_COMP:
        VERIFY_LEN(len, 1 + SIZE_OF_INT_COMP);
        p_val->params.int_comp.voltage_gain       = uint16_decode(&p_rcvd_val[pos]);
        p_val->params.int_comp.voltage_offset     = uint16_decode(&p_rcvd_val[pos + 2]);
        p_val->params.int_comp.current_gain       = uint16_decode(&p_rcvd_val[pos + 4]);
        p_val->params.int_comp.current_offset     = uint16_decode(&p_rcvd_val[pos + 6]);
        p_val->params.int_comp.temperature_gain   = uint16_decode(&p_rcvd_val[pos + 8]);
        p_val->params.int_comp.temperature_offset = uint16_decode(&p_rcvd_val[pos + 10]);
        return NRF_SUCCESS;

    case BLE_KD2_CP_OP_SET_EXT_COMP:
        VERIFY_LEN(len, 1 + SIZE_OF_EXT_COMP);
        p_val->params.ext_comp.temperature_offset = uint16_decode(&p_rcvd_val[pos]);
        p_val->params.ext_comp.left_current_gain  = p_rcvd_val[pos + 2];
        p_val->params.ext_comp.right_current_gain = p_rcvd_val[pos + 3];
        return NRF_SUCCESS;

    default:
        return NRF_ERROR_INVALID_PARAM;
   }
}

static bool is_feature_supported(ble_KD2_f_t const * p_feature, ble_KD2_cp_op_code_t opcode)
{
    switch (opcode)
    {
    case BLE_KD2_CP_OP_REQ_CHN_CONFIG:
    case BLE_KD2_CP_OP_SET_CHN_CONFIG:
        return p_feature->config.channel_config_supported;

    case BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG:
    case BLE_KD2_CP_OP_SET_COM_PIN_CONFIG:
        return p_feature->config.com_pin_mode_supported;

    case BLE_KD2_CP_OP_REQ_INT_COMP:
    case BLE_KD2_CP_OP_SET_INT_COMP:
        return p_feature->config.internal_comp_supported;

    case BLE_KD2_CP_OP_REQ_EXT_COMP:
    case BLE_KD2_CP_OP_SET_EXT_COMP:
        return p_feature->config.external_comp_supported;

    case BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE:
    case BLE_KD2_CP_OP_START_IMU_CALIB:
        return p_feature->config.imu_calib_supported;

    default:
        return false;
   }
}

static void on_cp_write(ble_KD2_t * p_KD2, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
    ble_KD2_client_spec_t             * p_client;
    ble_KD2_evt_t                       evt;
    cp_val_t                              decoded;

    p_client = get_client_data_by_conn_handle(conn_handle, p_KD2->p_client);
    if (p_client == NULL)
    {
        if (p_KD2->error_handler != NULL)
        {
            p_KD2->error_handler(NRF_ERROR_NOT_FOUND);
        }
    }

    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.offset      = 0;
    auth_reply.params.write.len         = 0;
    auth_reply.params.write.p_data      = NULL;
    auth_reply.params.write.update      = 1;

    if (p_client == NULL)
    {
        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_UNKNOWN;
    }
    else if (is_indication_enabled(p_KD2, conn_handle))
    {
        if (p_client->procedure_status == BLE_KD2_CP_PROC_STATUS_FREE)
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

    err_code = sd_ble_gatts_rw_authorize_reply(conn_handle, &auth_reply);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application.
        if (p_KD2->error_handler != NULL)
        {
            p_KD2->error_handler(err_code);
        }
        return;
    }

    if (auth_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS || p_client == NULL)
    {
        return;
    }

    p_client->procedure_status = BLE_KD2_CP_PROC_IN_PROGRESS;

    err_code = cp_decode(p_evt_write->data, p_evt_write->len, p_KD2, &decoded);
    if (err_code != NRF_SUCCESS)
    {
        ble_KD2_cp_rsp_t rsp;
        rsp.opcode = decoded.opcode;
        rsp.status = BLE_KD2_CP_RSP_VAL_INVALID;
        err_code = ble_KD2_cp_response(p_KD2, conn_handle, &rsp);
        if (err_code != NRF_SUCCESS && p_KD2->error_handler)
        {
            p_KD2->error_handler(err_code);
        }
    }
    else if (p_KD2->evt_handler && is_feature_supported(&p_KD2->feature, decoded.opcode))
    {
        evt.evt_type = BLE_KD2_EVT_CP_EVT;
        evt.p_KD2 = p_KD2;
        evt.p_client = p_client;
        evt.cp_evt = decoded.opcode;
        evt.p_params = &decoded.params;

        p_KD2->evt_handler(&evt);
    }
    else
    {
        ble_KD2_cp_rsp_t rsp;
        rsp.opcode = decoded.opcode;
        rsp.status = BLE_KD2_CP_RSP_VAL_NOT_SUPPORTED;
        err_code = ble_KD2_cp_response(p_KD2, conn_handle, &rsp);
        if (err_code != NRF_SUCCESS && p_KD2->error_handler)
        {
            p_KD2->error_handler(err_code);
        }
    }
}

static void on_rw_authorize_request(ble_KD2_t * p_KD2, ble_gatts_evt_t const * p_gatts_evt)
{
    ble_gatts_evt_rw_authorize_request_t const * p_auth_req = &p_gatts_evt->params.authorize_request;

    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if ((p_auth_req->request.write.op != BLE_GATTS_OP_PREP_WRITE_REQ) &&
            (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) &&
            (p_auth_req->request.write.op != BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
        {
            if (p_auth_req->request.write.handle == p_KD2->cp_handles.value_handle)
            {
                on_cp_write(p_KD2, &p_auth_req->request.write, p_gatts_evt->conn_handle);
            }
        }
    }
}

static uint8_t cp_encode(ble_KD2_cp_rsp_t const* p_rsp, ble_KD2_t const * p_KD2, uint8_t * p_data)
{
    uint8_t len = 0;

    p_data[len++] = BLE_KD2_CP_OP_RSP_CODE;
    p_data[len++] = p_rsp->opcode;
    p_data[len++] = p_rsp->status;

    if (p_rsp->status == BLE_KD2_CP_RSP_VAL_SUCCESS)
    {
        switch (p_rsp->opcode)
        {
            case BLE_KD2_CP_OP_REQ_CHN_CONFIG:
                p_data[len++] = p_rsp->params.channel_config.channel;
                uint16_encode(p_rsp->params.channel_config.config.output_power, &p_data[len]);
                len += 2;
                p_data[len++] = p_rsp->params.channel_config.config.output_limit;
                p_data[len++] = p_rsp->params.channel_config.config.optic_type;
                uint16_encode(p_rsp->params.channel_config.config.optic_offset, &p_data[len]);
                len += 2;
                break;

            case BLE_KD2_CP_OP_REQ_COM_PIN_CONFIG:
                p_data[len++] = p_rsp->params.com_pin;
                break;

            case BLE_KD2_CP_OP_REQ_INT_COMP:
                uint16_encode(p_rsp->params.int_comp.voltage_gain, &p_data[len]);
                len += 2;
                uint16_encode(p_rsp->params.int_comp.voltage_offset, &p_data[len]);
                len += 2;
                uint16_encode(p_rsp->params.int_comp.current_gain, &p_data[len]);
                len += 2;
                uint16_encode(p_rsp->params.int_comp.current_offset, &p_data[len]);
                len += 2;
                uint16_encode(p_rsp->params.int_comp.temperature_gain, &p_data[len]);
                len += 2;
                uint16_encode(p_rsp->params.int_comp.temperature_offset, &p_data[len]);
                len += 2;
                break;

            case BLE_KD2_CP_OP_REQ_EXT_COMP:
                uint16_encode(p_rsp->params.ext_comp.temperature_offset, &p_data[len]);
                len += 2;
                p_data[len++] = p_rsp->params.ext_comp.left_current_gain;
                p_data[len++] = p_rsp->params.ext_comp.right_current_gain;
                break;

            case BLE_KD2_CP_OP_REQ_IMU_CALIB_STATE:
                p_data[len++] = p_rsp->params.imu_calib_state ? 1 : 0;
                break;

            default:
                break;
        }
    }

    return len;
}

static void cp_rsp_send(ble_KD2_t const * p_KD2, uint16_t conn_handle)
{
    uint32_t                  err_code;
    uint16_t                  hvx_len;
    ble_gatts_hvx_params_t    hvx_params;
    ble_KD2_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_KD2->p_client);
    if (p_client == NULL)
    {
        if (p_KD2->error_handler != NULL)
        {
            p_KD2->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if ((p_client->procedure_status == BLE_KD2_CP_RSP_CODE_IND_PENDING))
    {
        hvx_len = p_client->response.len;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_KD2->cp_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = p_client->response.encoded_rsp;

        err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);

        // Error handling
        if ((err_code == NRF_SUCCESS) && (hvx_len != p_client->response.len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        switch (err_code)
        {
            case NRF_SUCCESS:
                p_client->procedure_status = BLE_KD2_CP_RSP_CODE_IND_CONFIRM_PENDING;
                // Wait for HVC event
                break;

            case NRF_ERROR_RESOURCES:
                // Wait for TX_COMPLETE event to retry transmission
                p_client->procedure_status = BLE_KD2_CP_RSP_CODE_IND_PENDING;
                break;

            default:
                // Report error to application
                p_client->procedure_status = BLE_KD2_CP_PROC_STATUS_FREE;
                if (p_KD2->error_handler != NULL)
                {
                    p_KD2->error_handler(err_code);
                }
                break;
        }
    }
}

static void on_tx_complete(ble_KD2_t const * p_KD2)
{
    uint8_t i = m_num_of_clients;

    while (i--)
    {
        if (p_KD2->p_client[i].procedure_status == BLE_KD2_CP_RSP_CODE_IND_PENDING)
        {
            cp_rsp_send(p_KD2, p_KD2->p_client[i].conn_handle);
        }
    }
}

static void on_sc_hvc_confirm(ble_KD2_t * p_KD2, ble_evt_t const * p_ble_evt)
{
    ble_KD2_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(p_ble_evt->evt.gatts_evt.conn_handle, p_KD2->p_client);
    if (p_client == NULL)
    {
        if (p_KD2->error_handler != NULL)
        {
            p_KD2->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_KD2->cp_handles.value_handle)
    {
        if (p_client->procedure_status == BLE_KD2_CP_RSP_CODE_IND_CONFIRM_PENDING)
        {
            p_client->procedure_status = BLE_KD2_CP_PROC_STATUS_FREE;
        }
    }
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_KD2_init(ble_KD2_t * p_KD2, uint8_t max_clients, ble_KD2_init_t const * p_KD2_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t KD2_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};

    if (p_KD2 == NULL || p_KD2_init == NULL || p_KD2_init->evt_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (max_clients == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_num_of_clients = max_clients;
    f_rd_sec         = p_KD2_init->KD2_f_rd_sec;
    cp_wr_sec        = p_KD2_init->KD2_cp_wr_sec;
    cp_cccd_wr_sec   = p_KD2_init->KD2_cp_cccd_wr_sec;

    // Initialize service structure
    p_KD2->evt_handler    = p_KD2_init->evt_handler;
    p_KD2->error_handler  = p_KD2_init->error_handler;
    p_KD2->feature        = p_KD2_init->feature;

    while (max_clients--)
    {
        p_KD2->p_client[max_clients].conn_handle      = BLE_CONN_HANDLE_INVALID;
        p_KD2->p_client[max_clients].procedure_status = BLE_KD2_CP_PROC_STATUS_FREE;
    }

    // add custom base uuid
    err_code = sd_ble_uuid_vs_add(&KD2_base_uuid, &p_KD2->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add service
    ble_uuid.type = p_KD2->uuid_type;
    ble_uuid.uuid = BLE_UUID_KD2_SERVICE;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_KD2->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add light feature characteristic
    err_code = feature_char_add(p_KD2, f_rd_sec);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add control control point characteristic
    err_code = control_point_char_add(p_KD2, cp_wr_sec, cp_cccd_wr_sec);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

void ble_KD2_on_ble_evt(void * p_context, ble_evt_t * p_ble_evt)
{
    if (p_context == NULL || p_ble_evt == NULL)
    {
        return;
    }

    ble_KD2_t * p_KD2 = (ble_KD2_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_KD2, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_KD2, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_KD2, &p_ble_evt->evt.gatts_evt);
        break;

    case BLE_GATTS_EVT_HVC:
        on_sc_hvc_confirm(p_KD2, p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_rw_authorize_request(p_KD2, &p_ble_evt->evt.gatts_evt);
        break;

    case BLE_EVT_TX_COMPLETE:
        on_tx_complete(p_KD2);
        break;

    default:
        break;
    }
}

uint32_t ble_KD2_cp_response(ble_KD2_t * p_KD2, uint16_t conn_handle, ble_KD2_cp_rsp_t const * p_rsp)
{
    ble_KD2_client_spec_t * p_client;

    if (p_KD2 == NULL || p_rsp == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_client = get_client_data_by_conn_handle(conn_handle, p_KD2->p_client);
    if (p_client == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (p_client->procedure_status != BLE_KD2_CP_PROC_IN_PROGRESS)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_client->procedure_status = BLE_KD2_CP_RSP_CODE_IND_PENDING;

    p_client->response.len = cp_encode(p_rsp, p_KD2, p_client->response.encoded_rsp);

    cp_rsp_send(p_KD2, conn_handle);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/



