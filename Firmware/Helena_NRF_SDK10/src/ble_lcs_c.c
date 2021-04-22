#include <string.h>
#include "ble_lcs_c.h"
#include "ble_db_discovery.h"
#include "nordic_common.h"
#include "app_error.h"



#define BLE_UUID_LCS_LM_CHARACTERISTIC      0x0102  /**< The UUID of the light measurement characteristic */
#define BLE_UUID_LCS_LF_CHARACTERISTIC      0x0103  /**< The UUID of the light feature characteristic */
#define BLE_UUID_LCS_LCP_CHARACTERISTIC     0x0104  /**< The UUID of the light Control Point characteristic */

#define TX_BUFFER_MASK       0x03                   /**< TX Buffer mask, must be a mask of contiguous zeroes, followed by contiguous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE       (TX_BUFFER_MASK + 1)   /**< Size of the send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH  5 // 20                  /**< The maximum size of a control point cmd message, limited for due to lack of memory */
#define MAX_MODE_LIST_LENGTH  (WRITE_MESSAGE_LENGTH-2) /**< The maximum size of a mode list that can be sent */



typedef enum
{
    READ_REQ,      /**< Type identifying that this tx_message is a read request. */
    WRITE_REQ      /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                       /**< The GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding the data that will be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    tx_request_t type;         /**< Type of message. (read or write). */
    union
    {
        uint16_t       read_handle;  /**< Read request handle. */
        write_params_t write_req;    /**< Write request message. */
    } req;
} tx_message_t;



ble_uuid128_t        ble_lcs_c_base_uuid = {{0x1B, 0xC5, 0xD5, 0xA5, 0x02, 0x00, 0x0e, 0x84, 0xe4, 0x11, 0x7d, 0xed, 0x00, 0x00, 0x77, 0x4f}};
static ble_lcs_c_t * mp_ble_lcs_c;                  /**< Pointer to the current instance of the Light Control client module */
static uint8_t       m_num_of_servers;              /**< Number of server specific data structures in the current instance */
static tx_message_t  m_tx_buffer[TX_BUFFER_SIZE];   /**< Transmit buffer for the messages that will be transmitted to the central. */
static uint32_t      m_tx_insert_index = 0;         /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t      m_tx_index        = 0;         /**< Current index in the transmit buffer containing the next message to be transmitted. */



/** @brief Function to get server related handles
 *
 * @param[in] conn_handle the connection handle as provided by the softdevice
 * @param[in] p_context   a pointer to the first element of the server context array
 * @return    NULL if no data for this connection handle available, otherwise the server specific data
 */
static ble_lcs_c_server_spec_t * get_server_data_by_conn_handle(uint16_t conn_handle, ble_lcs_c_server_spec_t * p_context)
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

/**@brief     Function for handling events from the database discovery module.
 *
 * @details   This function will handle an event from the database discovery module, and determine
 *            if it relates to the discovery of Light Control service at the peer. If so, it will
 *            call the application's event handler indicating that the Light Control service has been
 *            discovered at the peer. It also populate the event with the service related
 *            information before providing it to the application.
 *
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
 static void db_discovery_evt_handler(ble_db_discovery_evt_t * p_evt)
{
    // Check if Ligth Control Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.p_discovered_db->srv_uuid.uuid == BLE_UUID_LCS_SERVICE &&
        p_evt->params.p_discovered_db->srv_uuid.type == mp_ble_lcs_c->uuid_type)
    {
        ble_lcs_c_server_spec_t * p_server;

        p_server = get_server_data_by_conn_handle(BLE_CONN_HANDLE_INVALID, mp_ble_lcs_c->p_server);
        if (p_server == NULL)
        {
            return;
        }

        p_server->conn_handle = p_evt->conn_handle;

        // Find the Light Control Measurement characteristic
        for (uint_fast8_t i = 0; i < p_evt->params.p_discovered_db->char_count; i++)
        {
            if (p_evt->params.p_discovered_db->charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_LCS_LM_CHARACTERISTIC)
            {
                // Found HID information characteristic. Store handle and break
                p_server->lcm_handle =
                    p_evt->params.p_discovered_db->charateristics[i].characteristic.handle_value;
                p_server->lcm_cccd_handle =
                    p_evt->params.p_discovered_db->charateristics[i].cccd_handle;
                break;
            }
        }
        // Find the Light Control Feature characteristic
        for (uint_fast8_t i = 0; i < p_evt->params.p_discovered_db->char_count; i++)
        {
            if (p_evt->params.p_discovered_db->charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_LCS_LF_CHARACTERISTIC)
            {
                // Found report map characteristic. Store handle and break
                p_server->lcf_handle =
                    p_evt->params.p_discovered_db->charateristics[i].characteristic.handle_value;
                break;
            }
        }
        // Find the Light Control Control Point characteristic
        for (uint_fast8_t i = 0; i < p_evt->params.p_discovered_db->char_count; i++)
        {
            if (p_evt->params.p_discovered_db->charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_LCS_LCP_CHARACTERISTIC)
            {
                // Found HID information characteristic. Store handle and break
                p_server->lccp_handle =
                    p_evt->params.p_discovered_db->charateristics[i].characteristic.handle_value;
                p_server->lccp_cccd_handle =
                    p_evt->params.p_discovered_db->charateristics[i].cccd_handle;
                break;
            }
        }

        if (mp_ble_lcs_c->evt_handler != NULL)
        {
            ble_lcs_c_evt_t evt;
            evt.evt_type = BLE_LCS_C_EVT_DISCOVERY_COMPLETE;
            evt.p_server = p_server;
            mp_ble_lcs_c->evt_handler(mp_ble_lcs_c, &evt);
        }
    }
}/**@brief    Function for handling disconnect events
 *
 * @param[in] p_ble_lcs_c  Pointer to the Light Control Service Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_disconnect(ble_lcs_c_t * p_ble_lcs_c, const ble_evt_t * p_ble_evt)
{
    ble_lcs_c_server_spec_t * p_server;

    p_server = get_server_data_by_conn_handle(p_ble_evt->evt.gap_evt.conn_handle, mp_ble_lcs_c->p_server);
    if (p_server == NULL)
    {
        return; // device this event is related to has no light control service
    }

    p_server->conn_handle      = BLE_CONN_HANDLE_INVALID;
    p_server->lcm_handle       = BLE_GATT_HANDLE_INVALID;
    p_server->lcm_cccd_handle  = BLE_GATT_HANDLE_INVALID;
    p_server->lcf_handle       = BLE_GATT_HANDLE_INVALID;
    p_server->lccp_handle      = BLE_GATT_HANDLE_INVALID;
    p_server->lccp_cccd_handle = BLE_GATT_HANDLE_INVALID;
}

/** @brief function to decode the Light Control Measurement Characteristic Data
 *
 * @param[in]  p_data raw data as provided by the softdevice
 * @param[out] p_meas Light Control Measurement Structure
 */
static void decode_measurement(uint8_t const * p_data, ble_lcs_c_lm_t * p_meas)
{
    union
    {
        ble_lcs_c_lm_flags_t decoded;
        uint16_t             encoded;
    } flags;

    union
    {
        ble_lcs_c_lm_status_flags_t decoded;
        uint8_t                     encoded;
    } status_flags;

    union
    {
        ble_lcs_c_hlmt_setup_t  decoded_hlmt;
        ble_lcs_c_bk_setup_t    decoded_bk;
        uint8_t                 encoded;
    } setup;

    p_meas->light_type = *p_data++;

    flags.encoded = uint16_decode(p_data);
    p_meas->flags = flags.decoded;
    p_data += 2;

    setup.encoded = *p_data++;

    switch (p_meas->light_type)
    {
    case BLE_LCS_C_LT_HELMET_LIGHT:
        p_meas->hlmt.mode.setup = setup.decoded_hlmt;
        if (flags.decoded.hlmt.intensity_present)
        {
            p_meas->hlmt.mode.intensity = *p_data++;
        }
        if (flags.decoded.hlmt.flood_status_present)
        {
            status_flags.encoded = *p_data++;
            p_meas->hlmt.flood_status = status_flags.decoded;
        }
        if (flags.decoded.hlmt.spot_status_present)
        {
            status_flags.encoded = *p_data++;
            p_meas->hlmt.spot_status = status_flags.decoded;
        }
        if (flags.decoded.hlmt.flood_power_present)
        {
            p_meas->hlmt.flood_power = uint16_decode(p_data);
            p_data += 2;
        }
        if (flags.decoded.hlmt.spot_power_present)
        {
            p_meas->hlmt.spot_power = uint16_decode(p_data);
            p_data += 2;
        }
        if (flags.decoded.hlmt.temperature_present)
        {
            p_meas->temperature = *p_data++;
        }
        if (flags.decoded.hlmt.input_voltage_present)
        {
            p_meas->input_voltage = uint16_decode(p_data);
            p_data += 2;
        }
        if (flags.decoded.hlmt.pitch_present)
        {
            p_meas->pitch = *p_data;
        }
        if (flags.decoded.hlmt.battery_soc_present)
        {
            p_meas->battery_soc = *p_data;
        }
        if (flags.decoded.hlmt.taillight_power_present)
        {
            p_meas->taillight_power = uint16_decode(p_data);
            p_data += 2;
        }
        break;
    case BLE_LCS_C_LT_BIKE_LIGHT:
        p_meas->bk.mode.setup = setup.decoded_bk;
        if (flags.decoded.bk.intensity_present)
        {
            p_meas->bk.mode.main_beam_intensity = *p_data++;
            p_meas->bk.mode.high_beam_intensity = *p_data++;
        }
        if (flags.decoded.bk.main_beam_status_present)
        {
            status_flags.encoded = *p_data++;
            p_meas->bk.main_beam_status = status_flags.decoded;
        }
        if (flags.decoded.bk.high_beam_status_present)
        {
            status_flags.encoded = *p_data++;
            p_meas->bk.high_beam_status = status_flags.decoded;
        }
        if (flags.decoded.bk.main_beam_power_present)
        {
            p_meas->bk.high_beam_power = uint16_decode(p_data);
            p_data += 2;
        }
        if (flags.decoded.bk.high_beam_power_present)
        {
            p_meas->bk.high_beam_power = uint16_decode(p_data);
            p_data += 2;
        }
        if (flags.decoded.bk.temperature_present)
        {
            p_meas->temperature = *p_data++;
        }
        if (flags.decoded.bk.input_voltage_present)
        {
            p_meas->input_voltage = uint16_decode(p_data);
            p_data += 2;
        }
        if (flags.decoded.bk.pitch_present)
        {
            p_meas->pitch = *p_data;
        }
        if (flags.decoded.bk.battery_soc_present)
        {
            p_meas->battery_soc = *p_data;
        }
        if (flags.decoded.bk.taillight_power_present)
        {
            p_meas->taillight_power = uint16_decode(p_data);
            p_data += 2;
        }
        break;
    case BLE_LCS_C_LT_TAIL_LIGHT:
    default:
        break;
    }
}

/** @brief function to decode the Light Control Measurement Characteristic Data
 *
 * @param[in]  p_data raw data as provided by the softdevice
 * @param[in]  size   data size
 * @param[out] p_cp   Light Control Control Point Structure
 * @return     true if a valid response received, otherwise false
 */
static bool decode_control_point(uint8_t const * p_data, uint16_t size, ble_lcs_c_cp_rsp_t * p_cp)
{
    if (*p_data++ != BLE_LCS_C_CP_CMD_RESPONSE)
    {
        return false;
    }

    p_cp->command = *p_data++;
    p_cp->response_value = *p_data++;

    if (p_cp->response_value != BLE_LCS_C_CP_RV_SUCCESS)
    {
        return true;
    }

    switch (p_cp->command)
    {
    case BLE_LCS_C_CP_CMD_REQ_MODE_CNT:
        p_cp->params.mode_cnt = *p_data;
        return true;
    case BLE_LCS_C_CP_CMD_REQ_GRP_CNFG:
        p_cp->params.group_cnt = *p_data;
        return true;
    case BLE_LCS_C_CP_CMD_REQ_MODE_CNFG:
        p_cp->params.mode_cfg.num_of_bytes = size - 3;
        p_cp->params.mode_cfg.pModes = (void const*)p_data;   // <- working?
        return true;
    case BLE_LCS_C_CP_CMD_REQ_LED_CNFG:
    case BLE_LCS_C_CP_CMD_CHK_LED_CNFG:
        p_cp->params.led_cfg.flood_cnt = *p_data++;
        p_cp->params.led_cfg.spot_cnt = *p_data;
        return true;
    case BLE_LCS_C_CP_CMD_REQ_SENS_OFF:
    case BLE_LCS_C_CP_CMD_CALIB_SENS_OFF:
        p_cp->params.sens_off.x = uint16_decode(p_data);
        p_data += 2;
        p_cp->params.sens_off.y = uint16_decode(p_data);
        p_data += 2;
        p_cp->params.sens_off.z = uint16_decode(p_data);
        return true;
    case BLE_LCS_C_CP_CMD_REQ_LIMITS:
        p_cp->params.curr_limit.flood = *p_data++;
        p_cp->params.curr_limit.spot = *p_data;
        return true;
    default:
        return true;
    }
}

/** @brief function to decode the Light Control Feature Characteristic Data
 *
 * @param[in]  p_data          raw data as provided by the softdevice
 * @param[out] p_feature Light Control Feature Structure
 */
static void decode_feature(uint8_t const * p_data, ble_lcs_c_lf_t * p_feature)
{
    union
    {
        ble_lcs_c_lf_t decoded;
        uint16_t       encoded;
    } feature;

    feature.encoded = uint16_decode(p_data);
    *p_feature = feature.decoded;
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will handle the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the hid report from the peer. If
 *            so, this function will relay the data to the application.
 *
 * @param[in] p_ble_lcs_c  Pointer to the Light Control Service Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_hvx(ble_lcs_c_t * p_ble_lcs_c, const ble_evt_t * p_ble_evt)
{
    ble_lcs_c_evt_t evt;
    ble_lcs_c_server_spec_t * p_server;

    p_server = get_server_data_by_conn_handle(p_ble_evt->evt.gattc_evt.conn_handle, mp_ble_lcs_c->p_server);
    if (p_server == NULL)
    {
        return; // device this event is related to has no light control service
    }

    memset(&evt, 0, sizeof(evt));
    evt.p_server = p_server;

    // Check if this is a Light Control Measurement notification
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_server->lcm_handle)
    {
        evt.evt_type = BLE_LCS_C_EVT_MEASUREMENT_NOTIFY;
        decode_measurement(p_ble_evt->evt.gattc_evt.params.hvx.data,
                           &evt.data.measurement);
        if (p_ble_lcs_c->evt_handler)
        {
            p_ble_lcs_c->evt_handler(p_ble_lcs_c, &evt);
        }
    }
    // Check if this is a Light Control Control Point Indication
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_server->lccp_handle)
    {
        uint32_t err_code;
        uint16_t data_size;

        // send confirmation
        err_code = sd_ble_gattc_hv_confirm(p_server->conn_handle, p_server->lccp_handle);
        if (err_code != NRF_SUCCESS)
        {
            /// TODO: ??
        }

        data_size = p_ble_evt->evt.gattc_evt.params.hvx.len - 3;
        evt.evt_type = BLE_LCS_C_EVT_CONTROL_POINT_INDIC;
        if (p_ble_lcs_c->evt_handler &&
            decode_control_point(p_ble_evt->evt.gattc_evt.params.hvx.data,
                                 data_size, &evt.data.control_point))
        {
            p_ble_lcs_c->evt_handler(p_ble_lcs_c, &evt);
        }
    }
}

/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        uint32_t err_code;

        if (m_tx_buffer[m_tx_index].type == READ_REQ)
        {
            err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                         m_tx_buffer[m_tx_index].req.read_handle,
                                         0);
        }
        else
        {
            err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                          &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
        }
        if (err_code == NRF_SUCCESS)
        {
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
    }
}

/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_lcs_c  Pointer to the Light Control Service Client Structure.
 * @param[in] p_ble_evt    Pointer to the SoftDevice event.
 */
static void on_write_rsp(ble_lcs_c_t * p_ble_lcs_c, const ble_evt_t * p_ble_evt)
{
    (void)p_ble_evt;

    ble_lcs_c_server_spec_t * p_server;

    p_server = get_server_data_by_conn_handle(p_ble_evt->evt.gattc_evt.conn_handle, mp_ble_lcs_c->p_server);
    if (p_server == NULL)
    {
        return; // device this event is related to has no light control service
    }

    // check write status for relevant handles
    if (p_ble_evt->evt.gattc_evt.gatt_status != BLE_GATT_STATUS_SUCCESS &&
        mp_ble_lcs_c->error_handler != NULL)
    {
        ble_lcs_c_error_evt_t evt;

        evt.p_server = p_server;
        evt.data.gatt_status = p_ble_evt->evt.gattc_evt.gatt_status;

        if (p_ble_evt->evt.gattc_evt.params.write_rsp.handle == p_server->lcm_cccd_handle)
        {
            evt.evt_type = BLE_LCS_C_ERROR_EVT_LCM_NOTIFY;
        }
        else if (p_ble_evt->evt.gattc_evt.params.write_rsp.handle == p_server->lccp_handle)
        {
            evt.evt_type = BLE_LCS_C_ERROR_EVT_LCCP_WRITE;
        }
        else if (p_ble_evt->evt.gattc_evt.params.write_rsp.handle == p_server->lccp_cccd_handle)
        {
            evt.evt_type = BLE_LCS_C_ERROR_EVT_LCCP_IND;
        }
        else // should not happen
        {
            return;
        }

        p_ble_lcs_c->error_handler(p_ble_lcs_c, &evt);
    }
}

/**@brief     Function for handling read response events.
 *
 * @details   This function will validate the read response and raise the appropriate
 *            event to the application.
 *
 * @param[in] p_ble_lcs_c   Pointer to the Light Control Service Client Structure.
 * @param[in] p_ble_evt     Pointer to the SoftDevice event.
 */
static void on_read_rsp(ble_lcs_c_t * p_ble_lcs_c, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_read_rsp_t * p_response;
    ble_lcs_c_evt_t evt;
    ble_lcs_c_server_spec_t * p_server;

    p_server = get_server_data_by_conn_handle(p_ble_evt->evt.gattc_evt.conn_handle, mp_ble_lcs_c->p_server);
    if (p_server == NULL)
    {
        return; // device this event is related to has no light control service
    }

    p_response = &p_ble_evt->evt.gattc_evt.params.read_rsp;

    // Check if this is a read response of the Light Control Feature characteristic
    if (p_response->handle == p_server->lcf_handle)
    {
        memset(&evt, 0, sizeof(evt));
        evt.evt_type = BLE_LCS_C_EVT_FEATURE_READ_RESP;
        evt.p_server = p_server;
        decode_feature(p_response->data, &evt.data.feature);
        if (p_ble_lcs_c->evt_handler != NULL)
        {
            p_ble_lcs_c->evt_handler(p_ble_lcs_c, &evt);
        }
    }
}

/**@brief Function for creating a TX message for writing a CCCD.
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t handle_cccd, uint16_t cccd_val)
{
    tx_message_t * p_msg;
    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    if (m_tx_insert_index == m_tx_index)
    {
        m_tx_insert_index--;
        m_tx_insert_index &= TX_BUFFER_MASK;

        return NRF_ERROR_NO_MEM;
    }

    p_msg->req.write_req.gattc_params.handle   = handle_cccd;
    p_msg->req.write_req.gattc_params.len      = BLE_CCCD_VALUE_LEN;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

static uint32_t encode_control_point(const ble_lcs_c_cp_write_t* p_command, uint8_t * p_data, uint16_t * p_len)
{
    *p_len = 0;

    *p_data++ = p_command->command;
    *p_len = 1;

    switch (p_command->command)
    {
    case BLE_LCS_C_CP_CMD_SET_MODE:
        *p_data = p_command->params.mode_to_set;
        *p_len += 1;
        return NRF_SUCCESS;
    case BLE_LCS_C_CP_CMD_CNFG_GROUP:
        *p_data = p_command->params.group_cnt;
        *p_len += 1;
        return NRF_SUCCESS;
    case BLE_LCS_C_CP_CMD_REQ_MODE_CNFG:
        *p_data = p_command->params.start_mode;
        *p_len += 1;
        return NRF_SUCCESS;
    case BLE_LCS_C_CP_CMD_CNFG_MODE:
        if (p_command->params.mode_cfg.num_of_bytes > MAX_MODE_LIST_LENGTH)
        {
            *p_len = 0;
            return NRF_ERROR_NO_MEM;
        }
        *p_data++ = p_command->params.mode_cfg.start;
        *p_len += 1;
        for (uint_fast8_t i = 0; i < p_command->params.mode_cfg.num_of_bytes; i++)
        {
            uint8_t* pModes = (uint8_t*)p_command->params.mode_cfg.pModes;
            *p_data++ = pModes[i];
            *p_len += 1;
        }
        return NRF_SUCCESS;
    case BLE_LCS_C_CP_CMD_SET_LIMITS:
        *p_data++ = p_command->params.curr_limit.flood;
        *p_data++ = p_command->params.curr_limit.spot;
        *p_len += 2;
        return NRF_SUCCESS;
    default:
        return NRF_ERROR_INVALID_PARAM;
    }
}



uint32_t ble_lcs_c_init(ble_lcs_c_t * p_ble_lcs_c, uint8_t num_of_servers, ble_lcs_c_init_t * p_ble_lcs_c_init)
{
    uint32_t        err_code;

    if ((p_ble_lcs_c == NULL) || (p_ble_lcs_c_init == NULL))
    {
        return NRF_ERROR_NULL;
    }

    if (num_of_servers == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    ble_uuid_t lcs_uuid;

    err_code = sd_ble_uuid_vs_add(&ble_lcs_c_base_uuid, &lcs_uuid.type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    lcs_uuid.uuid = BLE_UUID_LCS_SERVICE;

    p_ble_lcs_c->uuid_type        = lcs_uuid.type;
    p_ble_lcs_c->evt_handler      = p_ble_lcs_c_init->evt_handler;
    p_ble_lcs_c->error_handler    = p_ble_lcs_c_init->error_handler;

    m_num_of_servers = num_of_servers;
    while (num_of_servers)
    {
        p_ble_lcs_c->p_server[num_of_servers - 1].conn_handle      = BLE_CONN_HANDLE_INVALID;
        p_ble_lcs_c->p_server[num_of_servers - 1].lcm_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_lcs_c->p_server[num_of_servers - 1].lcm_cccd_handle  = BLE_GATT_HANDLE_INVALID;
        p_ble_lcs_c->p_server[num_of_servers - 1].lcf_handle       = BLE_GATT_HANDLE_INVALID;
        p_ble_lcs_c->p_server[num_of_servers - 1].lccp_handle      = BLE_GATT_HANDLE_INVALID;
        p_ble_lcs_c->p_server[num_of_servers - 1].lccp_cccd_handle = BLE_GATT_HANDLE_INVALID;
        num_of_servers--;
    }

    mp_ble_lcs_c = p_ble_lcs_c;

    return ble_db_discovery_evt_register(&lcs_uuid,
                                         db_discovery_evt_handler);
}

void ble_lcs_c_on_ble_evt(ble_lcs_c_t * p_ble_lcs_c, ble_evt_t const * p_ble_evt)
{
    if ((p_ble_lcs_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            // nothing to do, server specific data structure will be filled in db discovery handler
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_lcs_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_lcs_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_lcs_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_rsp(p_ble_lcs_c, p_ble_evt);
            break;

        default:
            break;
    }

    tx_buffer_process();
}

uint32_t ble_lcs_c_lcm_notif_enable(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle, bool enable)
{
    uint16_t cccd_val;
    ble_lcs_c_server_spec_t * p_server;

    if (p_ble_lcs_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_server = get_server_data_by_conn_handle(conn_handle, mp_ble_lcs_c->p_server);
    if (p_server == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    if (enable)
    {
        cccd_val = BLE_GATT_HVX_NOTIFICATION;
    }
    else
    {
        cccd_val = 0;
    }

    return cccd_configure(conn_handle, p_server->lcm_cccd_handle, cccd_val);
}

uint32_t ble_lcs_c_feature_read(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle)
{
    tx_message_t * p_msg;
    ble_lcs_c_server_spec_t * p_server;

    if (p_ble_lcs_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_server = get_server_data_by_conn_handle(conn_handle, mp_ble_lcs_c->p_server);
    if (p_server == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    p_msg                = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index   &= TX_BUFFER_MASK;

    if (m_tx_insert_index == m_tx_index)
    {
        m_tx_insert_index--;
        m_tx_insert_index &= TX_BUFFER_MASK;

        return NRF_ERROR_NO_MEM;
    }

    p_msg->req.read_handle = p_server->lcf_handle;
    p_msg->conn_handle     = conn_handle;
    p_msg->type            = READ_REQ;

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_lcs_c_cp_write(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle, const ble_lcs_c_cp_write_t* p_command)
{
    uint32_t err_code;
    ble_lcs_c_server_spec_t * p_server;
    tx_message_t * p_msg;

    if (p_ble_lcs_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_server = get_server_data_by_conn_handle(conn_handle, mp_ble_lcs_c->p_server);
    if (p_server == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    p_msg                = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index   &= TX_BUFFER_MASK;

    if (m_tx_insert_index == m_tx_index)
    {
        m_tx_insert_index--;
        m_tx_insert_index &= TX_BUFFER_MASK;

        return NRF_ERROR_NO_MEM;
    }

    p_msg->req.write_req.gattc_params.handle   = p_server->lccp_handle;
    p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
    p_msg->req.write_req.gattc_params.offset   = 0;
    p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
    p_msg->conn_handle                         = conn_handle;
    p_msg->type                                = WRITE_REQ;

    err_code = encode_control_point(p_command, p_msg->req.write_req.gattc_value, &p_msg->req.write_req.gattc_params.len);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    tx_buffer_process();
    return NRF_SUCCESS;
}

uint32_t ble_lcs_c_cp_indic_enable(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle, bool enable)
{
    uint16_t cccd_val;
    ble_lcs_c_server_spec_t * p_server;

    if (p_ble_lcs_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_server = get_server_data_by_conn_handle(conn_handle, mp_ble_lcs_c->p_server);
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

    return cccd_configure(conn_handle, p_server->lccp_cccd_handle, cccd_val);
}
