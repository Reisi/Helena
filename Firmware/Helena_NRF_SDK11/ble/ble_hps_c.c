#include <string.h>
#include "ble_hps_c.h"
#include "nordic_common.h"
#include "app_error.h"
#include "nrf_log.h"

#define BLE_HPS_C_CP_CHAR_MIN_IND_LEN   3       /**< minimum length of the control point characteristic */


static ble_uuid128_t const ble_hps_c_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};
static uint8_t       m_num_of_servers;              /**< Number of server specific data structures in the current instance */

/** @brief Function to get server related handles
 *
 * @param[in] conn_handle the connection handle as provided by the softdevice
 * @param[in] p_context   a pointer to the first element of the server context array
 * @return    NULL if no data for this connection handle available, otherwise the server specific data
 */
static ble_hps_c_server_spec_t * get_server_data_by_conn_handle(uint16_t conn_handle, ble_hps_c_server_spec_t * p_context)
{
    uint_fast8_t index = m_num_of_servers;

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

/**@brief    Function for handling disconnect events
 *
 * @param[in] p_ble_hps_c  Pointer to the Helen Project Service Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_disconnect(ble_hps_c_t * p_ble_hps_c, const ble_evt_t * p_ble_evt)
{
    ble_hps_c_server_spec_t * p_server;

    p_server = get_server_data_by_conn_handle(p_ble_evt->evt.gap_evt.conn_handle, p_ble_hps_c->p_server);
    if (p_server == NULL)
    {
        return; // device this event is related to has no light control service
    }

    p_server->conn_handle           = BLE_CONN_HANDLE_INVALID;
    p_server->hps_db.cp_handle      = BLE_GATT_HANDLE_INVALID;
    p_server->hps_db.cp_cccd_handle = BLE_GATT_HANDLE_INVALID;
}

/** @brief function to decode the Control Point Characteristic Indication Data
 *
 * @param[in]  p_data raw data as provided by the softdevice
 * @param[in]  size   data size
 * @param[out] p_cp   Control Point Structure
 * @return     true if a valid response received, otherwise false
 */
static bool decode_control_point(uint8_t const * p_data, uint16_t size, ble_hps_c_cp_rsp_t * p_cp)
{
    if (*p_data++ != BLE_HPS_C_CP_CMD_RESPONSE || size < BLE_HPS_C_CP_CHAR_MIN_IND_LEN)
    {
        return false;
    }

    p_cp->command = *p_data++;
    p_cp->response_value = *p_data++;

    return true;
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @param[in] p_ble_hps_c  Pointer to the Helen Project Service Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_hvx(ble_hps_c_t * p_ble_hps_c, const ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    uint16_t data_size;
    ble_hps_c_evt_t evt = {0};
    ble_hps_c_server_spec_t * p_server;

    p_server = get_server_data_by_conn_handle(p_ble_evt->evt.gattc_evt.conn_handle, p_ble_hps_c->p_server);
    if (p_server == NULL)
    {
        return; // device this event is related to has no light control service
    }

    // Check if this is a Control Point Indication
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_server->hps_db.cp_handle)
    {
        // send confirmation
        err_code = sd_ble_gattc_hv_confirm(p_server->conn_handle, p_server->hps_db.cp_handle);
        if (err_code != NRF_SUCCESS)
        {
            /// TODO: ??
        }

        data_size = p_ble_evt->evt.gattc_evt.params.hvx.len - 3;
        evt.p_server = p_server;
        evt.evt_type = BLE_HPS_C_EVT_CONTROL_POINT_INDIC;
        if (p_ble_hps_c->evt_handler &&
            decode_control_point(p_ble_evt->evt.gattc_evt.params.hvx.data,
                                 data_size, &evt.data.control_point))
        {
            p_ble_hps_c->evt_handler(p_ble_hps_c, &evt);
        }
    }
}

/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_hps_c  Pointer to the Helen Project Service Client Structure.
 * @param[in] p_ble_evt    Pointer to the SoftDevice event.
 */
static void on_write_rsp(ble_hps_c_t * p_ble_hps_c, const ble_evt_t * p_ble_evt)
{
    ble_hps_c_server_spec_t * p_server;

    p_server = get_server_data_by_conn_handle(p_ble_evt->evt.gattc_evt.conn_handle, p_ble_hps_c->p_server);
    if (p_server == NULL)
    {
        return; // device this event is related to has no helen control service
    }

    // check write status for relevant handles
    if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS &&
        p_ble_hps_c->error_handler != NULL)
    {
        p_ble_hps_c->error_handler(p_ble_evt->evt.gattc_evt.gatt_status);
    }
}

/**@brief Function for creating a TX message for writing a CCCD.
 */
static uint32_t cccd_configure(ble_hps_c_t * p_ble_hps_c, uint16_t conn_handle, uint16_t cccd_handle, uint16_t cccd_val)
{
    ble_gq_message_t msg;

    memset(&msg, 0, sizeof(ble_gq_message_t));
    msg.conn_handle                         = conn_handle;
    msg.type                                = BLE_GQ_WRITE_REQ;
    msg.req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    msg.req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    msg.req.write_req.gattc_params.handle   = cccd_handle;
    msg.req.write_req.gattc_params.len      = BLE_CCCD_VALUE_LEN;
    msg.req.write_req.gattc_params.p_value  = msg.req.write_req.gattc_value;
    msg.req.write_req.gattc_params.offset   = 0;
    msg.req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;

    return ble_gq_message_add(p_ble_hps_c->p_gq_inst, &msg);
}

static uint32_t encode_control_point(const ble_hps_c_cp_write_t* p_command, uint8_t * p_data, uint16_t * p_len)
{
    *p_len = 0;

    *p_data++ = p_command->command;
    *p_len = 1;

    switch (p_command->command)
    {
    case BLE_HPS_C_CP_CMD_SET_MODE:
        *p_data = p_command->params.mode_to_set;
        *p_len += 1;
        return NRF_SUCCESS;

    case BLE_HPS_C_CP_CMD_REQ_MODE:
    case BLE_HPS_C_CP_CMD_REQ_SEARCH:
    case BLE_HPS_C_CP_CMD_REQ_F_RESET:
        return NRF_ERROR_NOT_SUPPORTED;

    default:
        return NRF_ERROR_INVALID_PARAM;
    }
}



uint32_t ble_hps_c_init(ble_hps_c_t * p_ble_hps_c, uint8_t num_of_servers, ble_hps_c_init_t * p_ble_hps_c_init)
{
    uint32_t        err_code;

    if (p_ble_hps_c == NULL || p_ble_hps_c_init == NULL || p_ble_hps_c_init->p_gq_inst == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (num_of_servers == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    ble_uuid_t hps_uuid;

    err_code = sd_ble_uuid_vs_add(&ble_hps_c_base_uuid, &hps_uuid.type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    hps_uuid.uuid = BLE_UUID_HPS_SERVICE;

    p_ble_hps_c->uuid_type        = hps_uuid.type;
    p_ble_hps_c->evt_handler      = p_ble_hps_c_init->evt_handler;
    p_ble_hps_c->error_handler    = p_ble_hps_c_init->error_handler;
    p_ble_hps_c->p_gq_inst        = p_ble_hps_c_init->p_gq_inst;

    m_num_of_servers = num_of_servers;
    while (num_of_servers)
    {
        p_ble_hps_c->p_server[num_of_servers - 1].conn_handle             = BLE_CONN_HANDLE_INVALID;
        p_ble_hps_c->p_server[num_of_servers - 1].hps_db.cp_handle      = BLE_GATT_HANDLE_INVALID;
        p_ble_hps_c->p_server[num_of_servers - 1].hps_db.cp_cccd_handle = BLE_GATT_HANDLE_INVALID;
        num_of_servers--;
    }

    return NRF_SUCCESS;
}

void ble_hps_c_on_ble_evt(ble_hps_c_t * p_ble_hps_c, ble_evt_t const * p_ble_evt)
{
    if (p_ble_hps_c == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            // nothing to do, server specific data structure will be filled in db discovery handler
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_hps_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_hps_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_hps_c, p_ble_evt);
            break;

        default:
            break;
    }
}

uint32_t ble_hps_c_cp_write(ble_hps_c_t * p_ble_hps_c, uint16_t conn_handle, const ble_hps_c_cp_write_t* p_command)
{
    uint32_t err_code;
    ble_hps_c_server_spec_t * p_server;

    if (p_ble_hps_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_server = get_server_data_by_conn_handle(conn_handle, p_ble_hps_c->p_server);
    if (p_server == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    ble_gq_message_t msg;

    memset(&msg, 0, sizeof(ble_gq_message_t));
    msg.conn_handle                         = conn_handle;
    msg.type                                = BLE_GQ_WRITE_REQ;
    msg.req.write_req.gattc_params.handle   = p_server->hps_db.cp_handle;
    msg.req.write_req.gattc_params.p_value  = msg.req.write_req.gattc_value;
    msg.req.write_req.gattc_params.offset   = 0;
    msg.req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;

    err_code = encode_control_point(p_command, msg.req.write_req.gattc_value, &msg.req.write_req.gattc_params.len);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return ble_gq_message_add(p_ble_hps_c->p_gq_inst, &msg);
}

uint32_t ble_hps_c_cp_indic_enable(ble_hps_c_t * p_ble_hps_c, uint16_t conn_handle, bool enable)
{
    uint16_t cccd_val;
    ble_hps_c_server_spec_t * p_server;

    if (p_ble_hps_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_server = get_server_data_by_conn_handle(conn_handle, p_ble_hps_c->p_server);
    if (p_server == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (enable)
    {
        cccd_val = BLE_GATT_HVX_INDICATION;
    }
    else
    {
        cccd_val = 0;
    }

    return cccd_configure(p_ble_hps_c, conn_handle, p_server->hps_db.cp_cccd_handle, cccd_val);
}

uint32_t ble_hps_c_handles_assign(ble_hps_c_t *    p_ble_hps_c,
                                  uint16_t         conn_handle,
                                  ble_hps_c_db_t const * p_peer_handles)
{
    if (p_ble_hps_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ble_hps_c_server_spec_t * p_server;

    // check if there is already an array assigned for this connection
    p_server = get_server_data_by_conn_handle(conn_handle, p_ble_hps_c->p_server);
    if (p_server == NULL)
    {
        // get empty array
        p_server = get_server_data_by_conn_handle(BLE_CONN_HANDLE_INVALID, p_ble_hps_c->p_server);
        if (p_server == NULL)
        {
            return NRF_ERROR_NOT_FOUND;
        }
        p_server->conn_handle = conn_handle;
    }

    if (p_peer_handles != NULL)
    {
        p_server->hps_db = *p_peer_handles;
    }

    return NRF_SUCCESS;
}
