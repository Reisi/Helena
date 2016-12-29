#include "ble_cgw.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include <string.h>

/**@brief     Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_cgw     Com Gateway Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_cgw_t * p_cgw, ble_evt_t * p_ble_evt)
{
    p_cgw->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief     Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110
 *            SoftDevice.
 *
 * @param[in] p_cgw     Com Gateway Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_cgw_t * p_cgw, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cgw->conn_handle = BLE_CONN_HANDLE_INVALID;
    p_cgw->is_notification_enabled = false;
}


/**@brief     Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_cgw     Com Gateway Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_cgw_t * p_cgw, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (
        (p_evt_write->handle == p_cgw->rx_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_cgw->is_notification_enabled = true;
        }
        else
        {
            p_cgw->is_notification_enabled = false;
        }
    }
    else if (
             (p_evt_write->handle == p_cgw->tx_handles.value_handle)
             &&
             (p_cgw->data_handler != NULL)
            )
    {
        p_cgw->data_handler(p_cgw, (com_MessageStruct*)p_evt_write->data);
    }
    else
    {
        // Do Nothing. This event is not relevant to this service.
    }
}


/**@brief       Function for adding RX characteristic.
 *
 * @param[in]   p_cgw        Com Gateway Service structure.
 * @param[in]   p_cgw_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rx_char_add(ble_cgw_t * p_cgw, const ble_cgw_init_t * p_cgw_init)
{
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type             = p_cgw->uuid_type;
    ble_uuid.uuid             = BLE_UUID_CGW_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc              = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth           = 0;
    attr_md.wr_auth           = 0;
    attr_md.vlen              = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(com_MessageStruct);

    return sd_ble_gatts_characteristic_add(p_cgw->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cgw->rx_handles);
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */

}


/**@brief       Function for adding TX characteristic.
 *
 * @param[in]   p_cgw        Com Gateway Service structure.
 * @param[in]   p_cgw_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t tx_char_add(ble_cgw_t * p_cgw, const ble_cgw_init_t * p_cgw_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write            = 1;
    char_md.char_props.write_wo_resp    = 1;
    char_md.p_char_user_desc            = NULL;
    char_md.p_char_pf                   = NULL;
    char_md.p_user_desc_md              = NULL;
    char_md.p_cccd_md                   = NULL;
    char_md.p_sccd_md                   = NULL;

    ble_uuid.type                       = p_cgw->uuid_type;
    ble_uuid.uuid                       = BLE_UUID_CGW_TX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth                     = 0;
    attr_md.wr_auth                     = 0;
    attr_md.vlen                        = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid              = &ble_uuid;
    attr_char_value.p_attr_md           = &attr_md;
    attr_char_value.init_len            = 1;
    attr_char_value.init_offs           = 0;
    attr_char_value.max_len             = sizeof(com_MessageStruct);

    return sd_ble_gatts_characteristic_add(p_cgw->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cgw->tx_handles);
}


void ble_cgw_on_ble_evt(ble_cgw_t * p_cgw, ble_evt_t * p_ble_evt)
{
    if ((p_cgw == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cgw, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cgw, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cgw, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_cgw_init(ble_cgw_t * p_cgw, const ble_cgw_init_t * p_cgw_init)
{
    uint32_t        err_code;
    ble_uuid_t      ble_uuid;
    ble_uuid128_t   cgw_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};

    if ((p_cgw == NULL) || (p_cgw_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    // Initialize service structure.
    p_cgw->conn_handle              = BLE_CONN_HANDLE_INVALID;
    p_cgw->data_handler             = p_cgw_init->data_handler;
    p_cgw->is_notification_enabled  = false;


    /**@snippet [Adding proprietary Service to S110 SoftDevice] */

    // Add custom base UUID.
    err_code = sd_ble_uuid_vs_add(&cgw_base_uuid, &p_cgw->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_cgw->uuid_type;
    ble_uuid.uuid = BLE_UUID_CGW_SERVICE;

    // Add service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_cgw->service_handle);
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add RX Characteristic.
    err_code = rx_char_add(p_cgw, p_cgw_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add TX Characteristic.
    err_code = tx_char_add(p_cgw, p_cgw_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_cgw_send_message(ble_cgw_t * p_cgw, const com_MessageStruct * p_message_tx)
{
    ble_gatts_hvx_params_t hvx_params;
    uint16_t len;

    if (p_cgw == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if ((p_cgw->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_cgw->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));
    len = (p_message_tx->Control & 0x0F) + 2;

    hvx_params.handle = p_cgw->rx_handles.value_handle;
    hvx_params.p_data = (uint8_t*)p_message_tx;
    hvx_params.p_len  = &len;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_cgw->conn_handle, &hvx_params);
}
