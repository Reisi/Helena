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
#define BLE_UUID_LCS_LM_CHARACTERISTIC 0x0102   /**< The UUID of the light measurement characteristic */
#define BLE_UUID_LCS_LF_CHARACTERISTIC 0x0103   /**< The UUID of the light feature characteristic */

#define BLR_LCS_LM_MIN_CHAR_LEN            3    /**< minimum length of the light measurement characteristic */
#define BLR_LCS_LM_MAX_CHAR_LEN            14   /**< maximum length of the light measurement characteristic */

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static uint32_t light_measurement_char_add(ble_lcs_t * p_lcs, const ble_lcs_init_t * p_lcs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    ble_uuid_t          ble_uuid;
    uint8_t             init_value[BLR_LCS_LM_MIN_CHAR_LEN] = {0, 0, p_lcs_init->initial_mode.type};

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

    memset(&attr_char_value, 0, sizeof(ble_gatts_attr_t));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = BLR_LCS_LM_MIN_CHAR_LEN;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLR_LCS_LM_MAX_CHAR_LEN;
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
    union
    {
        ble_lcs_lf_t raw;
        uint8_t      encoded;
    } init_value = {.raw = p_lcs_init->features};

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
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 1;
    attr_char_value.p_value   = &init_value.encoded;

    return sd_ble_gatts_characteristic_add(p_lcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_lcs->lf_handles);
}

static void on_connect(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt)
{
    p_lcs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt)
{
    (void)p_ble_evt;
    p_lcs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_lm_cccd_write(ble_lcs_t * p_lcs, ble_gatts_evt_write_t * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        ble_lcs_evt_t evt;

        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_lcs->is_lm_notfy_enabled = true;
            evt.evt_type = BLE_LCS_EVT_LM_NOTIVICATION_ENABLED;
        }
        else
        {
            p_lcs->is_lm_notfy_enabled = false;
            evt.evt_type = BLE_LCS_EVT_LM_NOTIVICATION_DISABLED;
        }

        if (p_lcs->evt_handler != NULL)
        {
            p_lcs->evt_handler(p_lcs, &evt);
        }
    }
}

static void on_write(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_lcs->lm_handles.cccd_handle)
    {
        on_lm_cccd_write(p_lcs, p_evt_write);
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

    // encode flags
    flags.raw = p_lcs_lm->flags;
    if (p_features->flood_supported == 0)
    {
        flags.raw.flood_status_present = 0;
        flags.raw.flood_power_present = 0;
    }
    if (p_features->spot_supported == 0)
    {
        flags.raw.spot_status_present = 0;
        flags.raw.spot_power_present = 0;
    }
    if (p_features->pitch_comp_supported == 0)
    {
        flags.raw.pitch_present = 0;
    }
    len += uint16_encode(flags.encoded, &p_encoded_buffer[len]);

    // encode mode
    p_encoded_buffer[len++] = p_lcs_lm->mode.type;

    // encode intensity
    if (flags.raw.intensity_present)
    {
        p_encoded_buffer[len++] = p_lcs_lm->mode.intensity;
    }

    // encode flood status
    if (flags.raw.flood_status_present)
    {
        status_flags.encoded = 0;
        status_flags.raw = p_lcs_lm->flood_status;
        p_encoded_buffer[len++] = status_flags.encoded;
    }

    // encode spot status
    if (flags.raw.spot_status_present)
    {
        status_flags.encoded = 0;
        status_flags.raw = p_lcs_lm->flood_status;
        p_encoded_buffer[len++] = status_flags.encoded;
    }

    // encode flood output power
    if (flags.raw.flood_power_present)
    {
        len += uint16_encode(p_lcs_lm->flood_power, &p_encoded_buffer[len]);
    }

    // encode spot output power
    if (flags.raw.spot_power_present)
    {
        len += uint16_encode(p_lcs_lm->spot_power, &p_encoded_buffer[len]);
    }

    // encode temperature
    if (flags.raw.temperature_present)
    {
        p_encoded_buffer[len++] = p_lcs_lm->temperature;
    }

    // encode input voltage
    if (flags.raw.input_voltage_present)
    {
        len += uint16_encode(p_lcs_lm->input_voltage, &p_encoded_buffer[len]);
    }

    // encode pitch
    if (flags.raw.pitch_present)
    {
        p_encoded_buffer[len++] = p_lcs_lm->pitch;
    }

    return len;
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_lcs_init(ble_lcs_t * p_lcs, const ble_lcs_init_t * p_lcs_init)
{
    uint32_t err_code;
    ble_uuid_t ble_uuid;
    ble_uuid128_t lcs_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};

    if (p_lcs == NULL || p_lcs_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Initialize service structure
    p_lcs->evt_handler = p_lcs_init->evt_handler;
    p_lcs->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_lcs->features    = p_lcs_init->features;

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
    ctrlpt_init.supported_features = p_lcs_init->features;
    ctrlpt_init.service_handle     = p_lcs->service_handle;
    ctrlpt_init.evt_handler        = p_lcs_init->cp_evt_handler;
    ctrlpt_init.error_handler      = p_lcs_init->error_handler;

    return ble_lcs_ctrlpt_init(&p_lcs->ctrl_pt, &ctrlpt_init);

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

uint32_t ble_lcs_light_measurement_send(const ble_lcs_t * p_lcs, const ble_lcs_lm_t * p_lcs_lm)
{
    uint32_t err_code;

    // send data if connected and notifying
    if (p_lcs->conn_handle != BLE_CONN_HANDLE_INVALID && p_lcs->is_lm_notfy_enabled)
    {
        uint8_t                encoded_data[BLR_LCS_LM_MAX_CHAR_LEN];
        uint16_t               len;
        uint16_t               hvx_len;
        ble_gatts_hvx_params_t hvx_params;

        len     = lm_encode(p_lcs_lm, &p_lcs->features, encoded_data);
        hvx_len = len;

        memset(&hvx_params, 0, sizeof(ble_gatts_hvx_params_t));
        hvx_params.handle = p_lcs->lm_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len  = &hvx_len;
        hvx_params.p_data = encoded_data;

        err_code = sd_ble_gatts_hvx(p_lcs->conn_handle, &hvx_params);
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



