/**
  ******************************************************************************
  * @file    ble_lcs.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/08
  * @brief   Light Control Service implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_lcs.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define BLE_UUID_LCS_LM_CHARACTERISTIC      0x0102  /**< The UUID of the light measurement characteristic */
#define BLE_UUID_LCS_LF_CHARACTERISTIC      0x0103  /**< The UUID of the light feature characteristic */

#define BLE_LCS_LM_MIN_CHAR_LEN             3       /**< minimum length of the light measurement characteristic */
#define BLE_LCS_LM_MAX_CHAR_LEN             14      /**< maximum length of the light measurement characteristic */

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t m_num_of_clients;

/* Private functions ---------------------------------------------------------*/
static uint32_t light_measurement_char_add(ble_lcs_t * p_lcs, const ble_lcs_init_t * p_lcs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    ble_uuid_t          ble_uuid;
    uint8_t             init_value[BLE_LCS_LM_MIN_CHAR_LEN];
    /*union
    {
        ble_lcs_light_setup_t raw;
        uint8_t               encoded;
    } setup;*/

    memset(&cccd_md, 0, sizeof(ble_gatts_attr_md_t));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_lcs_init->lcs_lm_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lcs->uuid_type;
    ble_uuid.uuid = BLE_UUID_LCS_LM_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(ble_gatts_attr_md_t));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(init_value, 0, sizeof(init_value));
    //setup.raw = p_lcs_init->initial_mode.setup;
    init_value[BLE_LCS_LM_MIN_CHAR_LEN - 1] = 0; //setup.encoded;

    memset(&attr_char_value, 0, sizeof(ble_gatts_attr_t));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = BLE_LCS_LM_MIN_CHAR_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_LCS_LM_MAX_CHAR_LEN;
    attr_char_value.p_value   = init_value;

    return sd_ble_gatts_characteristic_add(p_lcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lcs->lm_handles);
}

static uint32_t light_feature_char_add(ble_lcs_t * p_lcs, const ble_lcs_init_t * p_lcs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    ble_uuid_t          ble_uuid;
    uint8_t init_value_encoded[4];

    init_value_encoded[0] = (uint8_t)p_lcs_init->feature.light_type;
    union
    {
        ble_lcs_lf_cfg_t raw;
        uint8_t          encoded;
    } init_value_cfg = {.raw = p_lcs_init->feature.cfg_features};
    init_value_encoded[1] = init_value_cfg.encoded;
    union
    {
        ble_lcs_lf_stp_t raw;
        uint8_t          encoded;
    } init_value_stp = {.raw = p_lcs_init->feature.stp_features};
    init_value_encoded[2] = init_value_stp.encoded;
    union
    {
        ble_lcs_lf_hlmt_t hlmt;
        ble_lcs_lf_bk_t   bk;
        //ble_lcs_lf_tl_t   tl;
        uint8_t           encoded;
    } init_value_lght;
    switch (p_lcs_init->feature.light_type)
    {
    case BLE_LCS_LT_HELMET_LIGHT:
        init_value_lght.hlmt = p_lcs_init->feature.hlmt_features;
        break;
    case BLE_LCS_LT_BIKE_LIGHT:
        init_value_lght.bk   = p_lcs_init->feature.bk_features;
        break;
    /*case BLE_LCS_LT_TL_LIGHT:
        init_value_lght.tl   = p_lcs_init->tl_features;
        break;*/
    default:
        init_value_lght.encoded = 0;
        break;
    }
    init_value_encoded[3] = init_value_lght.encoded;

    memset(&char_md, 0, sizeof(ble_gatts_char_md_t));
    char_md.char_props.read   = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_lcs->uuid_type;
    ble_uuid.uuid = BLE_UUID_LCS_LF_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(ble_gatts_attr_md_t));
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.read_perm = p_lcs_init->lcs_lf_attr_md.read_perm;
    attr_md.vloc      = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth   = 0;
    attr_md.wr_auth   = 0;
    attr_md.vlen      = 0;

    memset(&attr_char_value, 0, sizeof(ble_gatts_attr_t));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(init_value_encoded);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(init_value_encoded);
    attr_char_value.p_value   = init_value_encoded;

    return sd_ble_gatts_characteristic_add(p_lcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lcs->lf_handles);
}

static ble_lcs_client_spec_t * get_client_data_by_conn_handle(uint16_t conn_handle, ble_lcs_client_spec_t * p_context)
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

static void on_connect(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt)
{
    uint32_t                err_code;
    ble_lcs_evt_t           evt;
    ble_gatts_value_t       gatts_val;
    uint8_t                 cccd_value[2];
    ble_lcs_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(BLE_CONN_HANDLE_INVALID, p_lcs->p_client);
    if (p_client == NULL)
    {
        return; /// TODO: add error handler and report NRF_ERROR_NO_MEM
    }
    p_client->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    // check the hosts CCCD value to inform the application if it has to send notifications
    memset(&gatts_val, 0, sizeof(ble_gatts_value_t));
    gatts_val.p_value = cccd_value;
    gatts_val.len     = sizeof(cccd_value);
    gatts_val.offset  = 0;
    err_code = sd_ble_gatts_value_get(p_ble_evt->evt.gap_evt.conn_handle,
                                      p_lcs->lm_handles.cccd_handle,
                                      &gatts_val);
    if (err_code == NRF_SUCCESS &&
        p_lcs->evt_handler != NULL &&
        ble_srv_is_notification_enabled(gatts_val.p_value))
    {
        p_client->is_lm_notfy_enabled = true;

        memset(&evt, 0, sizeof(ble_lcs_evt_t));
        evt.evt_type    = BLE_LCS_EVT_LM_NOTIVICATION_ENABLED;
        evt.p_lcs       = p_lcs;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_client    = p_client;

        p_lcs->evt_handler(&evt);
    }
}

static void on_disconnect(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt)
{
    ble_lcs_evt_t evt;
    ble_lcs_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(p_ble_evt->evt.gap_evt.conn_handle, p_lcs->p_client);
    if (p_client == NULL)
    {
        return; /// TODO: add error handler and report NRF_ERROR_NOT_FOUND
    }

    p_client->is_lm_notfy_enabled = false;

    if (p_lcs->evt_handler != NULL)
    {
        memset(&evt, 0, sizeof(ble_lcs_evt_t));
        evt.evt_type    = BLE_LCS_EVT_LM_NOTIVICATION_DISABLED;
        evt.p_lcs       = p_lcs;
        evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        evt.p_client    = p_client;

        p_lcs->evt_handler(&evt);
    }

    p_client->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_lm_cccd_write(ble_lcs_t * p_lcs, ble_gatts_evt_write_t * p_evt_write, uint16_t conn_handle)
{
    ble_lcs_evt_t evt;
    ble_lcs_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_lcs->p_client);
    if (p_client == NULL)
    {
        return; /// TODO: add error handler and report NRF_ERROR_NOT_FOUND
    }

    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        memset(&evt, 0, sizeof(ble_lcs_evt_t));
        evt.p_lcs       = p_lcs;
        evt.conn_handle = conn_handle;
        evt.p_client    = p_client;


        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_client->is_lm_notfy_enabled = true;
            evt.evt_type = BLE_LCS_EVT_LM_NOTIVICATION_ENABLED;
        }
        else
        {
            p_client->is_lm_notfy_enabled = false;
            evt.evt_type = BLE_LCS_EVT_LM_NOTIVICATION_DISABLED;
        }

        if (p_lcs->evt_handler != NULL)
        {
            p_lcs->evt_handler(&evt);
        }
    }
}

static void on_write(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_lcs->lm_handles.cccd_handle)
    {
        on_lm_cccd_write(p_lcs, p_evt_write, p_ble_evt->evt.gatts_evt.conn_handle);
    }
}

static uint8_t lm_encode(const ble_lcs_lm_t * p_lcs_lm, const ble_lcs_lf_t * p_features, uint8_t * p_encoded_buffer)
{
    uint8_t len = 0;

    union
    {
        ble_lcs_lm_status_flags_t raw;
        uint8_t                   encoded;
    } status_flags;

    union
    {
        ble_lcs_lm_flags_t raw;
        uint16_t           encoded;
    } flags;

    union
    {
        ble_lcs_hlmt_setup_t  hlmt;
        ble_lcs_bk_setup_t    bk;
        //ble_lcs_tl_setup_t    tl;
        uint8_t               encoded;
    } setup;

    p_encoded_buffer[len++] = (uint8_t)p_lcs_lm->light_type;

    switch (p_lcs_lm->light_type)
    {
    case BLE_LCS_LT_HELMET_LIGHT:
        // encode flags
        flags.raw = p_lcs_lm->flags;
        /*if (p_features->hlmt_features.flood_supported == 0)
        {
            flags.raw.flood_status_present = 0;
            flags.raw.flood_power_present = 0;
        }
        if (p_features->hlmt_features.spot_supported == 0)
        {
            flags.raw.spot_status_present = 0;
            flags.raw.spot_power_present = 0;
        }
        if (p_features->hlmt_features.pitch_comp_supported == 0)
        {
            flags.raw.pitch_present = 0;
        }*/
        len += uint16_encode(flags.encoded & BLE_LCS_LM_FLAGS_MASK, &p_encoded_buffer[len]);

        // encode setup
        setup.hlmt = p_lcs_lm->hlmt.mode.setup;
        p_encoded_buffer[len++] = setup.encoded;

        // encode intensity
        if (flags.raw.hlmt.intensity_present)
        {
            p_encoded_buffer[len++] = p_lcs_lm->hlmt.mode.intensity;
        }

        // encode flood status
        if (flags.raw.hlmt.flood_status_present)
        {
            status_flags.raw = p_lcs_lm->hlmt.flood_status;
            p_encoded_buffer[len++] = status_flags.encoded & BLE_LCS_LM_STATUS_FLAGS_MASK;
        }

        // encode spot status
        if (flags.raw.hlmt.spot_status_present)
        {
            status_flags.raw = p_lcs_lm->hlmt.spot_status;
            p_encoded_buffer[len++] = status_flags.encoded & BLE_LCS_LM_STATUS_FLAGS_MASK;
        }

        // encode flood output power
        if (flags.raw.hlmt.flood_power_present)
        {
            len += uint16_encode(p_lcs_lm->hlmt.flood_power, &p_encoded_buffer[len]);
        }

        // encode spot output power
        if (flags.raw.hlmt.spot_power_present)
        {
            len += uint16_encode(p_lcs_lm->hlmt.spot_power, &p_encoded_buffer[len]);
        }

        // encode temperature
        if (flags.raw.hlmt.temperature_present)
        {
            p_encoded_buffer[len++] = p_lcs_lm->temperature;
        }

        // encode input voltage
        if (flags.raw.hlmt.input_voltage_present)
        {
            len += uint16_encode(p_lcs_lm->input_voltage, &p_encoded_buffer[len]);
        }

        // encode pitch
        if (flags.raw.hlmt.pitch_present)
        {
            p_encoded_buffer[len++] = p_lcs_lm->pitch;
        }

        break;
    case BLE_LCS_LT_BIKE_LIGHT:
        // encode flags
        flags.raw = p_lcs_lm->flags;
        /*if (p_features->bk_features.main_beam_supported == 0 &&
            p_features->bk_features.extended_main_beam_supported == 0)
        {
            flags.raw.main_beam_status_present = 0;
            flags.raw.main_beam_power_present = 0;
        }
        if (p_features->bk_features.high_beam_supported == 0)
        {
            flags.raw.high_beam_status_present = 0;
            flags.raw.high_beam_power_present = 0;
        }*/
        len += uint16_encode(flags.encoded & BLE_LCS_LM_FLAGS_MASK, &p_encoded_buffer[len]);

        // encode setup
        setup.bk = p_lcs_lm->bk.mode.setup;
        p_encoded_buffer[len++] = setup.encoded;

        // encode intensity
        if (flags.raw.bk.intensity_present)
        {
            p_encoded_buffer[len++] = p_lcs_lm->bk.mode.main_beam_intensity;
            p_encoded_buffer[len++] = p_lcs_lm->bk.mode.high_beam_intensity;
        }

        // encode main beam status
        if (flags.raw.bk.main_beam_status_present)
        {
            status_flags.raw = p_lcs_lm->bk.main_beam_status;
            p_encoded_buffer[len++] = status_flags.encoded & BLE_LCS_LM_STATUS_FLAGS_MASK;
        }

        // encode high beam status
        if (flags.raw.bk.high_beam_status_present)
        {
            status_flags.raw = p_lcs_lm->bk.high_beam_status;
            p_encoded_buffer[len++] = status_flags.encoded & BLE_LCS_LM_STATUS_FLAGS_MASK;
        }

        // encode main beam output power
        if (flags.raw.bk.main_beam_power_present)
        {
            len += uint16_encode(p_lcs_lm->bk.main_beam_power, &p_encoded_buffer[len]);
        }

        // encode high beam output power
        if (flags.raw.bk.high_beam_power_present)
        {
            len += uint16_encode(p_lcs_lm->bk.high_beam_power, &p_encoded_buffer[len]);
        }

        // encode temperature
        if (flags.raw.bk.temperature_present)
        {
            p_encoded_buffer[len++] = p_lcs_lm->temperature;
        }

        // encode input voltage
        if (flags.raw.bk.input_voltage_present)
        {
            len += uint16_encode(p_lcs_lm->input_voltage, &p_encoded_buffer[len]);
        }

        // encode inclination
        if (flags.raw.bk.pitch_present)
        {
            p_encoded_buffer[len++] = p_lcs_lm->pitch;
        }
        break;
    default:
        break;
    }

    return len;
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_lcs_init(ble_lcs_t * p_lcs, uint8_t max_clients, const ble_lcs_init_t * p_lcs_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t lcs_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};

    if (p_lcs == NULL || p_lcs_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (max_clients == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_num_of_clients = max_clients;

    // Initialize service structure
    p_lcs->evt_handler = p_lcs_init->evt_handler;
    p_lcs->feature     = p_lcs_init->feature;

    while (max_clients)
    {
        p_lcs->p_client[max_clients - 1].conn_handle = BLE_CONN_HANDLE_INVALID;
        max_clients--;
    }

    // add custom base uuid
    err_code = sd_ble_uuid_vs_add(&lcs_base_uuid, &p_lcs->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add service
    ble_uuid.type = p_lcs->uuid_type;
    ble_uuid.uuid = BLE_UUID_LCS_SERVICE;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lcs->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add light measurement characteristic
    err_code = light_measurement_char_add(p_lcs, p_lcs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add light feature characteristic
    err_code = light_feature_char_add(p_lcs, p_lcs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // add light control control point characteristic
    if (p_lcs_init->cp_evt_handler == NULL)
    {
        return NRF_SUCCESS;
    }
    ble_lcs_ctrlpt_init_t ctrlpt_init;
    memset(&ctrlpt_init, 0, sizeof(ble_lcs_ctrlpt_init_t));
    ctrlpt_init.uuid_type          = p_lcs->uuid_type;
    ctrlpt_init.lc_ctrlpt_attr_md  = p_lcs_init->lcs_lcp_attr_md;
    ctrlpt_init.feature            = p_lcs_init->feature;
    ctrlpt_init.service_handle     = p_lcs->service_handle;
    ctrlpt_init.evt_handler        = p_lcs_init->cp_evt_handler;
    ctrlpt_init.error_handler      = p_lcs_init->error_handler;

    err_code = ble_lcs_ctrlpt_init(&p_lcs->ctrl_pt, m_num_of_clients, &ctrlpt_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

void ble_lcs_on_ble_evt(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt)
{
    if (p_lcs == NULL || p_ble_evt == NULL)
    {
        return;
    }

    ble_lcs_ctrlpt_on_ble_evt(&p_lcs->ctrl_pt, p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_lcs, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_lcs, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_lcs, p_ble_evt);
        break;

    default:
        break;
    }
}

uint32_t ble_lcs_light_measurement_send(const ble_lcs_t * p_lcs, uint16_t conn_handle, const ble_lcs_lm_t * p_lcs_lm)
{
    uint32_t err_code;
    ble_lcs_client_spec_t * p_client;

    p_client = get_client_data_by_conn_handle(conn_handle, p_lcs->p_client);
    if (p_client == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    // send data if connected and notifying
    if (p_client->is_lm_notfy_enabled)
    {
        uint8_t                encoded_data[BLE_LCS_LM_MAX_CHAR_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = lm_encode(p_lcs_lm, &p_lcs->feature, encoded_data);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(ble_gatts_hvx_params_t));
        hvx_params.handle = p_lcs->lm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_data;

        err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
        if (err_code == NRF_SUCCESS && hvx_len != len)
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}

/**END OF FILE*****************************************************************/



