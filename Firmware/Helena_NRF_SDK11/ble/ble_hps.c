/**
  ******************************************************************************
  * @file    ble_hps.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @brief   Helen Project Service implementation
  ******************************************************************************
  */

  /* logger configuration ------------------------------------------------------*/
//#define HPS_LOG_ENABLED

#ifdef HPS_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // HPS_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "ble_hps.h"
#include "ble_srv_common.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define BLE_UUID_HPS_MEAS_CHARACTERISTIC 0x0302  /**< The UUID of the measurement characteristic */
#define BLE_UUID_HPS_FEAT_CHARACTERISTIC 0x0303  /**< The UUID of the feature characteristic */
#define BLE_UUID_HPS_MODE_CHARACTERISTIC 0x0304  /**< The UUID of the modes characteristic */
#define BLE_UUID_HPS_CTPT_CHARACTERISTIC 0x0305  /**< The UUID of the control point characteristic */

#define BLE_HPS_M_MIN_CHAR_LEN           3       /**< minimum length of the measurement characteristic */
#define BLE_HPS_M_MAX_CHAR_LEN           8       /**< maximum length of the measurement characteristic */

#define BLE_HPS_F_MIN_CHAR_LEN           4       /**< minimum length of the feature characteristic */

#define BLE_HPS_CP_MIN_CHAR_LEN          1       /**< minimum length of the control point characteristic */
#define BLE_HPS_CP_MAX_CHAR_LEN          20      /**< maximum length of the control point characteristic */

#define BLE_HPS_MD_MAX_WRITE_LEN         20      /**< maximum length of mode characteristics if no queued writes buffer is provided */

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)   (sizeof(x)/sizeof(x[0]))

#define VERIFY_LEN(actual, target)      \
do                                      \
{                                       \
    if (target != actual)               \
    {                                   \
        return NRF_ERROR_INVALID_PARAM; \
    }                                   \
} while (0)

/* Private types -------------------------------------------------------------*/
typedef struct
{
    uint16_t conn_handle;
    uint16_t len;
} qwr_status_t;

/* Private variables ---------------------------------------------------------*/
static uint8_t  m_num_of_clients;
static qwr_status_t qwr_status = {.conn_handle = BLE_CONN_HANDLE_INVALID};

/* Private functions ---------------------------------------------------------*/
static ble_hps_client_spec_t * get_client_data_by_conn_handle(uint16_t conn_handle, ble_hps_client_spec_t * p_context)
{
    uint_fast8_t index = m_num_of_clients;

    while (index)
    {
        if (p_context[index - 1].conn_handle == conn_handle)
        {
            return &p_context[index - 1];
        }
        index--;
    }

    return NULL;
}

static uint32_t measurement_char_add(ble_hps_t * p_hps)
{
    ble_add_char_params_t add_char_params;
    uint8_t               init_value[BLE_HPS_M_MIN_CHAR_LEN] = {0};

    init_value[2] = 255;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = BLE_UUID_HPS_MEAS_CHARACTERISTIC;
    add_char_params.uuid_type         = p_hps->uuid_type;
    add_char_params.init_len          = BLE_HPS_M_MIN_CHAR_LEN;
    add_char_params.max_len           = BLE_HPS_M_MAX_CHAR_LEN;
    add_char_params.is_var_len        = true;
    add_char_params.p_init_value      = init_value;
    add_char_params.char_props.notify = 1;
    add_char_params.cccd_write_access = SEC_OPEN;

    return characteristic_add(p_hps->service_handle, &add_char_params, &p_hps->m_handles);
}

static uint32_t feature_valid_check(ble_hps_feature_init_t const * p_feature)
{
    if (p_feature == NULL || (p_feature->channel_count && p_feature->p_channel_size == NULL))
    {
        return NRF_ERROR_NULL;
    }

    if (p_feature->mode_count == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    for (uint8_t i = 0; i < p_feature->channel_count; i++)
    {
        if ((p_feature->p_channel_size[i].channel_bitsize == 0) ||
            (p_feature->p_channel_size[i].sp_ftr_bitsize > 15) ||
            (p_feature->p_channel_size[i].sp_ftr_bitsize >= p_feature->p_channel_size[i].channel_bitsize) ||
            (p_feature->p_channel_size[i].channel_description > 15))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }

    return NRF_SUCCESS;
}

static uint32_t feature_char_add(ble_hps_t * p_hps, ble_hps_init_t const * p_hps_init)
{
    ble_add_char_params_t    add_char_params = {0};
    uint16_t                 channel_size, len = 0;
    uint8_t                  init_value[BLE_HPS_F_MIN_CHAR_LEN + p_hps_init->features.channel_count * sizeof(uint16_t)];
    ble_gatts_char_handles_t f_handles;
    union
    {
        ble_hps_f_flags_t raw;
        uint16_t          encoded;
    } flags;

    p_hps->features.mode_count    = p_hps_init->features.mode_count;
    p_hps->features.channel_count = p_hps_init->features.channel_count;
    p_hps->features.flags         = p_hps_init->features.flags;

    init_value[len++] = p_hps_init->features.mode_count;
    init_value[len++] = p_hps_init->features.channel_count;
    for (uint8_t i = 0; i < p_hps_init->features.channel_count; i++)
    {
        channel_size  = p_hps_init->features.p_channel_size[i].channel_bitsize;
        channel_size |= p_hps_init->features.p_channel_size[i].sp_ftr_bitsize << 8;
        channel_size |= p_hps_init->features.p_channel_size[i].channel_description << 12;
        len += uint16_encode(channel_size, &init_value[len]);
    }
    flags.raw = p_hps_init->features.flags;
    len += uint16_encode(flags.encoded, &init_value[len]);

    add_char_params.uuid              = BLE_UUID_HPS_FEAT_CHARACTERISTIC;
    add_char_params.uuid_type         = p_hps->uuid_type;
    add_char_params.init_len          = ARRAY_SIZE(init_value);
    add_char_params.max_len           = ARRAY_SIZE(init_value);
    add_char_params.p_init_value      = init_value;
    add_char_params.char_props.read   = 1;
    add_char_params.read_access       = SEC_OPEN;

    return characteristic_add(p_hps->service_handle, &add_char_params, &f_handles);
}

static uint32_t modes_char_add(ble_hps_t * p_hps, ble_hps_init_t const * p_hps_init)
{
    uint32_t err_code;
    ble_add_char_params_t add_char_params = {0};

    add_char_params.uuid              = BLE_UUID_HPS_MODE_CHARACTERISTIC;
    add_char_params.uuid_type         = p_hps->uuid_type;
    add_char_params.init_len          = p_hps_init->modes.total_size;
    add_char_params.max_len           = p_hps_init->modes.total_size;
    add_char_params.p_init_value      = (uint8_t*)p_hps_init->modes.p_mode_config;
    add_char_params.is_value_user     = true;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.write  = 1;
    add_char_params.is_defered_write  = true;
    add_char_params.read_access       = SEC_OPEN;
    add_char_params.write_access      = p_hps_init->modes_wr_sec;

    err_code = characteristic_add(p_hps->service_handle, &add_char_params, &p_hps->modes_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    p_hps->modes_len = p_hps_init->modes.total_size;

    return err_code;
}

static uint32_t cp_char_add(ble_hps_t * p_hps, ble_hps_init_t const * p_hps_init)
{
    ble_add_char_params_t add_char_params = {0};

    add_char_params.uuid                = BLE_UUID_HPS_CTPT_CHARACTERISTIC;
    add_char_params.uuid_type           = p_hps->uuid_type;
    add_char_params.max_len             = BLE_HPS_CP_MAX_CHAR_LEN;
    add_char_params.is_var_len          = true;
    add_char_params.char_props.indicate = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.is_defered_write    = true;
    add_char_params.write_access        = p_hps_init->cp_wr_sec;
    add_char_params.cccd_write_access   = p_hps_init->cp_wr_sec;

    return characteristic_add(p_hps->service_handle, &add_char_params, &p_hps->cp_handles);
}

static void send_event(ble_hps_evt_type_t type, ble_hps_t const * p_hps, ble_hps_client_spec_t const * p_client)
{
    ble_hps_evt_t evt = {0};

    evt.evt_type = type;
    evt.p_hps = p_hps;
    evt.p_client = p_client;

    if (p_hps->evt_handler != NULL)
    {
        (void)p_hps->evt_handler(&evt);
    }
}

static void on_connect(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    uint32_t                err_code;
    ble_hps_client_spec_t * p_client;
    ble_gatts_value_t       gatts_val;
    uint8_t                 cccd_value[2];

    p_client = get_client_data_by_conn_handle(BLE_CONN_HANDLE_INVALID, p_hps->p_client);
    if (p_client == NULL)
    {
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    p_client->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;

    // check the hosts measurement CCCD value to inform the application if it has to send notifications
    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_hps->m_handles.cccd_handle,
                                      &gatts_val);
    if (err_code == NRF_SUCCESS && ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        p_client->is_m_notfy_enabled = true;
        send_event(BLE_HPS_EVT_M_NOTIVICATION_ENABLED, p_hps, p_client);
    }

    // check the hosts control point CCCD value to inform the application if it has to send indications
    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_hps->cp_handles.cccd_handle,
                                      &gatts_val);
    if (err_code == NRF_SUCCESS && ble_srv_is_indication_enabled(gatts_val.p_value))
    {
        p_client->is_cp_indic_enabled = true;
        send_event(BLE_HPS_EVT_CP_INDICATION_ENABLED, p_hps, p_client);
    }
}

static void on_disconnect(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    ble_hps_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(p_ble_evt->evt.gap_evt.conn_handle, p_hps->p_client);
    if (p_client == NULL)
    {
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    LOG_INFO("[hps]: client disconnected");

    if (p_client->is_m_notfy_enabled)
    {
        p_client->is_m_notfy_enabled = false;
        send_event(BLE_HPS_EVT_M_NOTIVICATION_DISABLED, p_hps, p_client);
    }

    if (p_client->is_cp_indic_enabled)
    {
        p_client->is_cp_indic_enabled = false;
        send_event(BLE_HPS_EVT_CP_INDICATION_DISABLED, p_hps, p_client);
    }

    p_client->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_client->procedure_status = BLE_HPS_CP_PROC_STATUS_FREE;

    if (p_ble_evt->evt.gap_evt.conn_handle == qwr_status.conn_handle)
    {
        p_hps->p_qwr_buf->inUse = false;
        qwr_status.conn_handle = BLE_CONN_HANDLE_INVALID;
        qwr_status.len = 0;
    }
}

static void on_m_cccd_write(ble_hps_t * p_hps, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    ble_hps_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_hps->p_client);
    if (p_client == NULL)
    {
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
       if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_client->is_m_notfy_enabled = true;
            send_event(BLE_HPS_EVT_M_NOTIVICATION_ENABLED, p_hps, p_client);
        }
        else
        {
            p_client->is_m_notfy_enabled = false;
            send_event(BLE_HPS_EVT_M_NOTIVICATION_DISABLED, p_hps, p_client);
        }
    }
}

static void on_cp_cccd_write(ble_hps_t * p_hps, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    ble_hps_client_spec_t * p_client;

    LOG_INFO("[hps]: cccd write %d", conn_handle);

    p_client = get_client_data_by_conn_handle(conn_handle, p_hps->p_client);
    if (p_client == NULL)
    {
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_evt_write->len == 2)
    {
        // CCCD written, update indication state
        if (ble_srv_is_indication_enabled(p_evt_write->data))
        {
            p_client->is_cp_indic_enabled = true;
            LOG_INFO("[hps]: cccd enabled");
            send_event(BLE_HPS_EVT_CP_INDICATION_ENABLED, p_hps, p_client);
        }
        else
        {
            p_client->is_cp_indic_enabled = false;
            LOG_INFO("[hps]: cccd disabled");
            send_event(BLE_HPS_EVT_CP_INDICATION_DISABLED, p_hps, p_client);
        }
    }
}

static void on_write(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_hps->m_handles.cccd_handle)
    {
        on_m_cccd_write(p_hps, p_evt_write, p_ble_evt->evt.gatts_evt.conn_handle);
    }
    else if (p_evt_write->handle == p_hps->cp_handles.cccd_handle)
    {
        on_cp_cccd_write(p_hps, p_evt_write, p_ble_evt->evt.gatts_evt.conn_handle);
    }
}

static uint16_t m_encode(ble_hps_m_t const * p_data, uint8_t * p_buffer)
{
    uint16_t len = 0;

    union
    {
        ble_hps_m_flags_t raw;
        uint8_t           encoded;
    } flags;

    p_buffer[len++] = 0;

    flags.raw = p_data->flags;
    p_buffer[len++] = flags.encoded;

    p_buffer[len++] = p_data->mode;

    if (flags.raw.output_power_present)
    {
        len += uint16_encode(p_data->output_power, &p_buffer[len]);
    }

    if (flags.raw.temperature_present)
    {
        p_buffer[len++] = p_data->temperature;
    }

    if (flags.raw.input_voltage_present)
    {
        len += uint16_encode(p_data->input_voltage, &p_buffer[len]);
    }

    return len;
}

static void on_mem_request(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    uint32_t err_code;

    if (p_ble_evt->evt.common_evt.params.user_mem_request.type == BLE_USER_MEM_TYPE_GATTS_QUEUED_WRITES)
    {
        LOG_INFO("[hps]: qwr mem request");

        if (qwr_status.conn_handle == BLE_CONN_HANDLE_INVALID && p_hps->p_qwr_buf->inUse == false)
        {
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.common_evt.conn_handle, NULL);
            if (err_code == NRF_SUCCESS)
            {
                qwr_status.conn_handle = p_ble_evt->evt.common_evt.conn_handle;
                p_hps->p_qwr_buf->inUse = true;
            }
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }

        if (err_code != NRF_SUCCESS && p_hps->error_handler != NULL)
        {
            p_hps->error_handler(err_code);
        }
    }
}

/*static void on_mem_release(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    if (p_ble_evt->evt.common_evt.params.user_mem_release.type == BLE_USER_MEM_TYPE_GATTS_QUEUED_WRITES)
    {
        LOG_INFO("[hps]: qwr mem release");

        p_hps->p_qwr_buf->inUse = false;
        qwr_status.conn_handle = BLE_CONN_HANDLE_INVALID;
        qwr_status.len = 0;
    }
}*/

static bool cp_decode(uint8_t const* p_data, uint16_t len,
                      ble_hps_cp_t * p_decoded, ble_hps_cp_rsp_ind_t * p_response)
{
    uint8_t i = 0;

    p_decoded->opcode = p_data[i++];

    p_response->len = 0;
    p_response->encoded_rsp[p_response->len++] = BLE_HPS_CP_RESPONSE_CODE;
    p_response->encoded_rsp[p_response->len++] = p_decoded->opcode;

    switch (p_decoded->opcode)
    {
    case BLE_HPS_CP_SET_MODE:
        if (len != 2)
        {
            p_response->encoded_rsp[p_response->len++] = BLE_HPS_CP_RSP_VAL_INVALID;
            return false;
        }
        else
        {
            p_response->encoded_rsp[p_response->len++] = BLE_HPS_CP_RSP_VAL_SUCCESS;
            p_decoded->mode_to_set = p_data[i];
            return true;
        }

    case BLE_HPS_CP_SET_MODE_OVERRIDE:
        p_response->encoded_rsp[p_response->len++] = BLE_HPS_CP_RSP_VAL_SUCCESS;
        p_decoded->channel_config.p_config = &p_data[i];
        p_decoded->channel_config.size = len - 1;
        return true;

    case BLE_HPS_CP_REQ_MODE:
    case BLE_HPS_CP_REQ_SEARCH:
    case BLE_HPS_CP_REQ_FACTORY_RESET:
        if (len != 1)
        {
            p_response->encoded_rsp[p_response->len++] = BLE_HPS_CP_RSP_VAL_INVALID;
            return false;
        }
        else
        {
            p_response->encoded_rsp[p_response->len++] = BLE_HPS_CP_RSP_VAL_SUCCESS;
            return true;
        }

    default:
        p_response->encoded_rsp[p_response->len++] = BLE_HPS_CP_RSP_VAL_INVALID;
        return false;
    }
}

static void cp_rsp_send(ble_hps_t const * p_hps, ble_hps_client_spec_t * p_client)
{
    uint32_t                  err_code;
    uint16_t                  hvx_len;
    ble_gatts_hvx_params_t    hvx_params;

    if ((p_client->procedure_status == BLE_HPS_CP_RSP_CODE_IND_PENDING))
    {
        hvx_len = p_client->pending_response.len;
        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_hps->cp_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = p_client->pending_response.encoded_rsp;

        err_code = sd_ble_gatts_hvx(p_client->conn_handle, &hvx_params);

        // Error handling
        if ((err_code == NRF_SUCCESS) && (hvx_len != p_client->pending_response.len))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }

        switch (err_code)
        {
            case NRF_SUCCESS:
                p_client->procedure_status = BLE_HPS_CP_RSP_CODE_IND_CONFIRM_PENDING;
                // Wait for HVC event
                break;

            case NRF_ERROR_RESOURCES:
                // Wait for TX_COMPLETE event to retry transmission
                p_client->procedure_status = BLE_HPS_CP_RSP_CODE_IND_PENDING;
                break;

            default:
                // Report error to application
                p_client->procedure_status = BLE_HPS_CP_PROC_STATUS_FREE;
                if (p_hps->error_handler != NULL)
                {
                    p_hps->error_handler(err_code);
                }
                break;
        }
    }
}

static void on_cp_write(ble_hps_t * p_hps, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;
    ble_hps_evt_t                         evt;
    ble_hps_client_spec_t               * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_hps->p_client);
    if (p_client == NULL)
    {
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(NRF_ERROR_NOT_FOUND);
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
    else if (p_client->is_cp_indic_enabled)
    {
        if (p_client->procedure_status == BLE_HPS_CP_PROC_STATUS_FREE)
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
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(err_code);
        }
        return;
    }

    if (auth_reply.params.write.gatt_status != BLE_GATT_STATUS_SUCCESS || p_client == NULL)
    {
        return;
    }

    p_client->procedure_status = BLE_HPS_CP_RSP_CODE_IND_PENDING;

    if(cp_decode(p_evt_write->data, p_evt_write->len, &evt.params.ctrl_pt, &p_client->pending_response) &&
       p_hps->evt_handler != NULL)
    {
        // send event
        evt.evt_type = BLE_HPS_EVT_CP_WRITE;
        evt.p_hps    = p_hps;
        evt.p_client = p_client;
        if(!p_hps->evt_handler(&evt))
        {
            p_client->pending_response.encoded_rsp[2] = BLE_HPS_CP_RSP_VAL_FAILED;
        }
    }

    cp_rsp_send(p_hps, p_client);
}

/*uint32_t qwr_value_get(ble_user_mem_block_t const * p_qwr,
                       uint16_t               attr_handle,
                       uint8_t              * p_mem,
                       uint16_t             * p_len)
{
    if (p_qwr == NULL || p_mem == NULL || p_len == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint16_t i          = 0;
    uint16_t handle     = BLE_GATT_HANDLE_INVALID;
    uint16_t val_len    = 0;
    uint16_t val_offset = 0;
    uint32_t cur_len    = 0;

    do
    {
        handle = uint16_decode(&(p_qwr->p_mem[i]));

        if (handle == BLE_GATT_HANDLE_INVALID)
        {
            break;
        }

        i         += sizeof(uint16_t);
        val_offset = uint16_decode(&(p_qwr->p_mem[i]));
        i         += sizeof(uint16_t);
        val_len    = uint16_decode(&(p_qwr->p_mem[i]));
        i         += sizeof(uint16_t);

        if (handle == attr_handle)
        {
            cur_len = val_offset + val_len;
            if (cur_len <= *p_len)
            {
                memcpy((p_mem + val_offset), &(p_qwr->p_mem[i]), val_len);
            }
            else
            {
                return NRF_ERROR_NO_MEM;
            }
        }

        i += val_len;
    }
    while (i < p_qwr->len);

    *p_len = cur_len;
    return NRF_SUCCESS;
}*/

static void on_md_write(ble_hps_t * p_hps, ble_gatts_evt_write_t const * p_evt_write, uint16_t conn_handle)
{
    uint32_t                              err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply = {0};
    ble_hps_evt_t                         evt;
    ble_hps_client_spec_t               * p_client;

    auth_reply.type                     = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
    auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_WRITE_NOT_PERMITTED;

    LOG_INFO("[hps]: mode write request %d received", p_evt_write->op);

    p_client = get_client_data_by_conn_handle(conn_handle, p_hps->p_client);
    if (p_client == NULL)
    {
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(NRF_ERROR_NOT_FOUND);
        }

        auth_reply.params.write.gatt_status = BLE_GATT_STATUS_ATTERR_UNLIKELY_ERROR;
    }
    else if (p_hps->evt_handler)
    {
        uint8_t const* p_data = p_evt_write->data;
        uint16_t len          = p_evt_write->len;
        uint8_t* p_buffer     = p_hps->p_qwr_buf->p_block->p_mem;

        switch (p_evt_write->op)
        {
        case BLE_GATTS_OP_WRITE_REQ:
            evt.evt_type = BLE_HPS_EVT_MODE_CONFIG_CHANGED;
            evt.p_hps = p_hps;
            evt.p_client = p_client;
            evt.params.modes.p_mode_config = (ble_hps_mode_config_t const*)p_data;
            evt.params.modes.p_channel_config = (void const*)&p_data[p_hps->features.mode_count * sizeof(ble_hps_mode_config_t)];
            evt.params.modes.total_size = len;

            auth_reply.params.write.gatt_status = p_hps->evt_handler(&evt) ? BLE_GATT_STATUS_SUCCESS : BLE_HPS_GATT_STATUS_ATTERR_INVALID_PARAM;
            auth_reply.params.write.update = 1;
            break;

        case BLE_GATTS_OP_PREP_WRITE_REQ:
            if (qwr_status.conn_handle == conn_handle &&
                (p_evt_write->offset + p_evt_write->len) <= p_hps->p_qwr_buf->p_block->len)
            {
                memcpy(&p_buffer[p_evt_write->offset], p_evt_write->data, p_evt_write->len);
                qwr_status.len += p_evt_write->len;
                auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
                auth_reply.params.write.offset = p_evt_write->offset;
                auth_reply.params.write.len = p_evt_write->len;
                auth_reply.params.write.p_data = p_evt_write->data;
                auth_reply.params.write.update = 1;
            }
            break;

        case BLE_GATTS_OP_EXEC_WRITE_REQ_NOW:
            evt.evt_type = BLE_HPS_EVT_MODE_CONFIG_CHANGED;
            evt.p_hps = p_hps;
            evt.p_client = p_client;
            evt.params.modes.p_mode_config = (ble_hps_mode_config_t const*)p_buffer;
            evt.params.modes.p_channel_config = (void const*)&p_buffer[p_hps->features.mode_count * sizeof(ble_hps_mode_config_t)];
            evt.params.modes.total_size = qwr_status.len;

            auth_reply.params.write.gatt_status = p_hps->evt_handler(&evt) ? BLE_GATT_STATUS_SUCCESS : BLE_HPS_GATT_STATUS_ATTERR_INVALID_PARAM;

            qwr_status.conn_handle = BLE_CONN_HANDLE_INVALID;
            qwr_status.len = 0;
            p_hps->p_qwr_buf->inUse = false;
            break;

        case BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL:
            auth_reply.params.write.gatt_status = BLE_GATT_STATUS_SUCCESS;
            qwr_status.conn_handle = BLE_CONN_HANDLE_INVALID;
            qwr_status.len = 0;
            p_hps->p_qwr_buf->inUse = false;
            break;

        default:
            break;

        }
    }

    err_code = sd_ble_gatts_rw_authorize_reply(conn_handle, &auth_reply);
    if (err_code != NRF_SUCCESS)
    {
        // Report error to application.
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(err_code);
        }
    }
}

static void on_rw_authorize_request(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_rw_authorize_request_t const * p_auth_req = &p_ble_evt->evt.gatts_evt.params.authorize_request;

    if (p_auth_req->type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
    {
        if ((p_auth_req->request.write.handle == p_hps->cp_handles.value_handle) &&
            ((p_auth_req->request.write.op == BLE_GATTS_OP_WRITE_REQ) ||
             (p_auth_req->request.write.op == BLE_GATTS_OP_WRITE_CMD) ||
             (p_auth_req->request.write.op == BLE_GATTS_OP_SIGN_WRITE_CMD)))
        {
            on_cp_write(p_hps, &p_auth_req->request.write, p_ble_evt->evt.gatts_evt.conn_handle);
        }

        if ((p_auth_req->request.write.handle == p_hps->modes_handles.value_handle) ||
            (p_auth_req->request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL) ||
            (p_auth_req->request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW))
        {
            on_md_write(p_hps, &p_auth_req->request.write, p_ble_evt->evt.gatts_evt.conn_handle);
        }
    }
}

static void on_tx_complete(ble_hps_t const * p_hps)
{
    uint8_t i = m_num_of_clients;

    while (i--)
    {
        if (p_hps->p_client[i].procedure_status == BLE_HPS_CP_RSP_CODE_IND_PENDING)
        {
            cp_rsp_send(p_hps, &p_hps->p_client[i]);
        }
    }
}

static void on_sc_hvc_confirm(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    ble_hps_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(p_ble_evt->evt.gatts_evt.conn_handle, p_hps->p_client);
    if (p_client == NULL)
    {
        if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(NRF_ERROR_NOT_FOUND);
        }
        return;
    }

    if (p_ble_evt->evt.gatts_evt.params.hvc.handle == p_hps->cp_handles.value_handle)
    {
        if (p_client->procedure_status == BLE_HPS_CP_RSP_CODE_IND_CONFIRM_PENDING)
        {
            p_client->procedure_status = BLE_HPS_CP_PROC_STATUS_FREE;
        }
    }
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_hps_init(ble_hps_t * p_hps, uint8_t max_clients, const ble_hps_init_t * p_hps_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t hps_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};

    if (p_hps == NULL || p_hps_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    err_code = feature_valid_check(&p_hps_init->features);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (max_clients == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_num_of_clients = max_clients;

    if (p_hps_init->modes.total_size > BLE_HPS_MD_MAX_WRITE_LEN)
    {
        if (p_hps_init->p_qwr_buf == NULL || p_hps_init->p_qwr_buf->p_block == NULL)
        {
            return NRF_ERROR_NULL;
        }
        if (p_hps_init->p_qwr_buf->p_block->len < BLE_HPS_MIN_BUF_SIZE(p_hps_init->modes.total_size))
        {
            return NRF_ERROR_NO_MEM;
        }
    }

    // Initialize service structure
    p_hps->p_qwr_buf           = p_hps_init->p_qwr_buf;
    p_hps->evt_handler         = p_hps_init->evt_handler;
    p_hps->error_handler       = p_hps_init->error_handler;

    while (max_clients--)
    {
        p_hps->p_client[max_clients].conn_handle = BLE_CONN_HANDLE_INVALID;
    }

    // add custom base uuid
    err_code = sd_ble_uuid_vs_add(&hps_base_uuid, &p_hps->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add service
    ble_uuid.type = p_hps->uuid_type;
    ble_uuid.uuid = BLE_UUID_HPS_SERVICE;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_hps->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add measurement characteristic
    err_code = measurement_char_add(p_hps);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add feature characteristic
    err_code = feature_char_add(p_hps, p_hps_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add modes characteristic
    err_code = modes_char_add(p_hps, p_hps_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add control point characteristic
    err_code = cp_char_add(p_hps, p_hps_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

void ble_hps_on_ble_evt(ble_hps_t * p_hps, ble_evt_t const * p_ble_evt)
{
    if (p_hps == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_hps, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_hps, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_hps, p_ble_evt);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        on_rw_authorize_request(p_hps, p_ble_evt);
        break;

    case BLE_GATTS_EVT_HVC:
        on_sc_hvc_confirm(p_hps, p_ble_evt);
        break;

    case BLE_EVT_TX_COMPLETE:
        on_tx_complete(p_hps);
        break;

    case BLE_EVT_USER_MEM_REQUEST:
        on_mem_request(p_hps, p_ble_evt);
        break;

    /*case BLE_EVT_USER_MEM_RELEASE:
        on_mem_release(p_hps, p_ble_evt);
        break;*/

    default:
        break;
    }
}

/*uint16_t ble_hps_on_qwr_evt(ble_hps_t const * p_hps, nrf_ble_qwr_t * p_qwr, nrf_ble_qwr_evt_t * p_evt)
{
    uint32_t err_code;
    uint8_t buffer[p_hps->modes_len];
    uint16_t len = sizeof(buffer);
    ble_hps_evt_t evt;
    uint16_t ret_code = BLE_GATT_STATUS_SUCCESS;

    if (p_hps->evt_handler != NULL && p_evt->attr_handle == p_hps->modes_handles.value_handle)
    {
        err_code = nrf_ble_qwr_value_get(p_qwr, p_evt->attr_handle, buffer, &len);
        if (err_code == NRF_SUCCESS)
        {
            evt.evt_type = BLE_HPS_EVT_MODE_CONFIG_CHANGED;
            evt.p_hps = p_hps;
            evt.p_client = get_client_data_by_conn_handle(p_qwr->conn_handle, p_hps->p_client);
            evt.params.modes.p_mode_config = (ble_hps_mode_config_t const*)buffer;
            evt.params.modes.p_channel_config = (void const*)&buffer[p_hps->features.mode_count * sizeof(ble_hps_mode_config_t)];
            evt.params.modes.total_size = len;

            ret_code = p_hps->evt_handler(&evt) ? BLE_GATT_STATUS_SUCCESS : BLE_HPS_GATT_STATUS_ATTERR_INVALID_PARAM;
        }
        else if (p_hps->error_handler != NULL)
        {
            p_hps->error_handler(err_code);
        }
    }

    return ret_code;
}*/

uint32_t ble_hps_measurement_send(ble_hps_t const * p_hps, uint16_t conn_handle, ble_hps_m_t const * p_data)
{
    if (p_hps == NULL || p_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t               err_code;
    uint8_t                encoded_data[BLE_HPS_M_MAX_CHAR_LEN];
    uint16_t               len;
    uint16_t               hvx_len;
    ble_gatts_hvx_params_t hvx_params;

    len     = m_encode(p_data, encoded_data);
    hvx_len = len;

    memset(&hvx_params, 0, sizeof(ble_gatts_hvx_params_t));
    hvx_params.handle = p_hps->m_handles.value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len  = &hvx_len;
    hvx_params.p_data = encoded_data;

    err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
    if (err_code == NRF_SUCCESS && hvx_len != len)
    {
        err_code = NRF_ERROR_DATA_SIZE;
    }

    return err_code;
}

/**END OF FILE*****************************************************************/



