/**
  ******************************************************************************
  * @file    ble_hids_c.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/12/03
  * @brief   HID over GATT Service Client module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sdk_common.h"
#include <string.h>
#include "ble_hids_c.h"
#include "nrf_error.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "app_error.h"

/* Private defines -----------------------------------------------------------*/
#define HID_INFO_FLAG_REMOTE_WAKE   (1<<0)
#define HID_INFO_FLAG_NORM_CONN     (1<<1)

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**@brief     Function for handling disconnect events
 *
 * @param[in] p_ble_hids_c Pointer to the HID Service Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_disconnect(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    // check if this disconnect event belongs to our connection
    if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_hids_c->conn_handle)
    {
        p_ble_hids_c->conn_handle = BLE_CONN_HANDLE_INVALID;
        //p_ble_hids_c->peer_hids_db.hid_info_handle = BLE_GATT_HANDLE_INVALID;
        //p_ble_hids_c->peer_hids_db.report_map_handle = BLE_GATT_HANDLE_INVALID;
        for (uint_fast8_t i = 0; i < NUM_OF_REPORT_HANDLES; i++)
        {
            p_ble_hids_c->peer_hids_db.report_handles[i].report_handle = BLE_GATT_HANDLE_INVALID;
            p_ble_hids_c->peer_hids_db.report_handles[i].cccd_handle = BLE_GATT_HANDLE_INVALID;
            //p_ble_hids_c->peer_hids_db.report_handles[i].report_reference_handle = BLE_GATT_HANDLE_INVALID;
        }
    }
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will handle the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the hid report from the peer. If
 *            so, this function will relay the data to the application.
 *
 * @param[in] p_ble_hids_c Pointer to the HID Service Client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event received.
 */
static void on_hvx(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    // Check if this notification is a report notification.
    if (p_ble_evt->evt.gattc_evt.conn_handle != p_ble_hids_c->conn_handle)
    {
        return;
    }

    for (uint_fast8_t i = 0; i < NUM_OF_REPORT_HANDLES; i++)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_hids_c->peer_hids_db.report_handles[i].report_handle)
        {
            ble_hids_c_evt_t evt;
            evt.evt_type = BLE_HIDS_C_EVT_REPORT_NOTIFICATION;
            evt.params.report.index = i;
            evt.params.report.p_report = p_ble_evt->evt.gattc_evt.params.hvx.data;
            evt.params.report.len = p_ble_evt->evt.gattc_evt.params.hvx.len;
            if (p_ble_hids_c->evt_handler != NULL)
            {
                p_ble_hids_c->evt_handler(p_ble_hids_c, &evt);
            }
            break;
        }
    }
}

/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_hids_c Pointer to the HID Service Client Structure.
 * @param[in] p_ble_evt    Pointer to the SoftDevice event.
 */
static void on_write_rsp(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    (void)p_ble_hids_c;
    (void)p_ble_evt;

    if (p_ble_hids_c->conn_handle == p_ble_evt->evt.gattc_evt.conn_handle)
    {
        // check write status for relevant handles
        for (uint_fast8_t i = 0; i < NUM_OF_REPORT_HANDLES; i++)
        {
            if (p_ble_evt->evt.gattc_evt.gatt_status != NRF_SUCCESS &&
                p_ble_evt->evt.gattc_evt.error_handle == p_ble_hids_c->peer_hids_db.report_handles[i].cccd_handle)
            {
                APP_ERROR_CHECK(p_ble_evt->evt.gattc_evt.gatt_status);
            }
        }
    }
}

/**@brief     Function for handling read response events.
 *
 * @details   This function will validate the read response and raise the appropriate
 *            event to the application.
 *
 * @param[in] p_bas_c   Pointer to the HID Service Client Structure.
 * @param[in] p_ble_evt Pointer to the SoftDevice event.
 */
/*static void on_read_rsp(ble_hids_c_t * p_ble_hids_c, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_read_rsp_t * p_response;
    ble_hids_c_evt_t evt;

    p_response = &p_ble_evt->evt.gattc_evt.params.read_rsp;

    if (p_ble_hids_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    // Check if this is a read response of the HID information characteristic
    if (p_response->handle == p_ble_hids_c->peer_hids_db.hid_info_handle)
    {
        memset(&evt.params.hid_info, 0, sizeof(ble_hids_c_hid_info_t));
        evt.evt_type = BLE_HIDS_C_EVT_HID_INFO_READ_RESP;
        evt.params.hid_info.bcd_hid = uint16_decode(p_response->data);
        evt.params.hid_info.country_code = p_response->data[2];
        if (p_response->data[3] & HID_INFO_FLAG_REMOTE_WAKE)
        {
            evt.params.hid_info.remote_wake = 1;
        }
        if (p_response->data[3] & HID_INFO_FLAG_NORM_CONN)
        {
            evt.params.hid_info.normally_connectable = 1;
        }
        if (p_ble_hids_c->evt_handler != NULL)
        {
            p_ble_hids_c->evt_handler(p_ble_hids_c, &evt);
        }
    }

    // Check if this is a read response of the Report map characteristic
    if (p_response->handle == p_ble_hids_c->peer_hids_db.report_map_handle)
    {
        evt.evt_type = BLE_HIDS_C_EVT_REPORT_MAP_READ_RESP;
        evt.params.report_map.p_report_map = p_response->data;
        evt.params.report_map.len          = p_response->len;
        if (p_ble_hids_c->evt_handler != NULL)
        {
            p_ble_hids_c->evt_handler(p_ble_hids_c, &evt);
        }
    }

    // Check if this is a read response of a Report characteristic
    for (uint_fast8_t i = 0; i < NUM_OF_REPORT_HANDLES; i++)
    {
        if (p_response->handle == p_ble_hids_c->peer_hids_db.report_handles[i].report_handle)
        {
            evt.evt_type = BLE_HIDS_C_EVT_REPORT_READ_RESP;
            evt.params.report.index = i;
            evt.params.report.p_report = p_response->data;
            evt.params.report.len      = p_response->len;
            if (p_ble_hids_c->evt_handler != NULL)
            {
                p_ble_hids_c->evt_handler(p_ble_hids_c, &evt);
            }
        }
    }

    // Check if this is a read response of a Report reference characteristic
    for (uint_fast8_t i = 0; i < NUM_OF_REPORT_HANDLES; i++)
    {
        if (p_response->handle == p_ble_hids_c->peer_hids_db.report_handles[i].report_reference_handle)
        {
            evt.evt_type = BLE_HIDS_C_EVT_REPORT_REFERENCE_READ_RESP;
            evt.params.report_ref.index       = i;
            evt.params.report_ref.report_id   = p_response->data[0];
            evt.params.report_ref.report_type = p_response->data[1];
            if (p_ble_hids_c->evt_handler != NULL)
            {
                p_ble_hids_c->evt_handler(p_ble_hids_c, &evt);
            }
        }
    }
}*/

/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(ble_hids_c_t * p_ble_hids_c, uint16_t cccd_handle, bool enable)
{
    ble_gq_message_t msg;
    uint16_t cccd_val   = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    memset(&msg, 0, sizeof(ble_gq_message_t));
    msg.conn_handle                         = p_ble_hids_c->conn_handle;
    msg.type                                = BLE_GQ_WRITE_REQ;
    msg.req.write_req.gattc_value[0]        = LSB_16(cccd_val);
    msg.req.write_req.gattc_value[1]        = MSB_16(cccd_val);
    msg.req.write_req.gattc_params.handle   = cccd_handle;
    msg.req.write_req.gattc_params.len      = BLE_CCCD_VALUE_LEN;
    msg.req.write_req.gattc_params.p_value  = msg.req.write_req.gattc_value;
    msg.req.write_req.gattc_params.offset   = 0;
    msg.req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;

    return ble_gq_message_add(p_ble_hids_c->p_gq_inst, &msg);
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_hids_c_init(ble_hids_c_t * p_ble_hids_c, ble_hids_c_init_t * p_ble_hids_c_init)
{
    if (p_ble_hids_c == NULL || p_ble_hids_c_init == NULL || p_ble_hids_c_init->p_gq_inst == NULL)
    {
        return NRF_ERROR_NULL;
    }

    //ble_uuid_t hids_uuid;

    //hids_uuid.type = BLE_UUID_TYPE_BLE;
    //hids_uuid.uuid = BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE;

    p_ble_hids_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
    //p_ble_hids_c->peer_hids_db.hid_info_handle   = BLE_GATT_HANDLE_INVALID;
    //p_ble_hids_c->peer_hids_db.report_map_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_hids_c->error_handler                  = p_ble_hids_c_init->error_handler;
    p_ble_hids_c->evt_handler                    = p_ble_hids_c_init->evt_handler;
    p_ble_hids_c->p_gq_inst                      = p_ble_hids_c_init->p_gq_inst;
    for (uint_fast8_t i = 0; i < NUM_OF_REPORT_HANDLES; i++)
    {
        p_ble_hids_c->peer_hids_db.report_handles[i].report_handle           = BLE_GATT_HANDLE_INVALID;
        p_ble_hids_c->peer_hids_db.report_handles[i].cccd_handle             = BLE_GATT_HANDLE_INVALID;
        //p_ble_hids_c->peer_hids_db.report_handles[i].report_reference_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS; //ble_db_discovery_evt_register(&hids_uuid);
}

/*void ble_hids_c_on_db_disc_evt(ble_hids_c_t * p_ble_hids_c, ble_db_discovery_evt_t const * p_evt)
{
    // Check if HID Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_BLE)
    {
        ble_hids_c_evt_t evt;
        evt.evt_type    = BLE_HIDS_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

        NRF_LOG_DEBUG("HID Service discovered at peer.");

        // Find the HID information characteristic
        for (uint_fast8_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_HID_INFORMATION_CHAR)
            {
                // Found HID information characteristic. Store handle and break
                evt.params.hid_db.hid_info_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                break;
            }
        }
        // Find the report map characteristic
        for (uint_fast8_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_REPORT_MAP_CHAR)
            {
                // Found report map characteristic. Store handle and break
                evt.params.hid_db.report_map_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                break;
            }
        }
        // Find the report characteristic
        for (uint_fast8_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                BLE_UUID_REPORT_CHAR)
            {
                // Found report characteristic. Store in first empty array member
                for (uint_fast8_t j = 0; j < NUM_OF_REPORT_HANDLES; j++)
                {
                    if (evt.params.hid_db.report_handles[j].report_handle == BLE_GATT_HANDLE_INVALID)
                    {
                        evt.params.hid_db.report_handles[j].report_handle =
                            p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                        evt.params.hid_db.report_handles[j].cccd_handle =
                            p_evt->params.discovered_db.charateristics[i].cccd_handle;
                        // report reference is not supported
                        //evt.params.hid_db.report_handles[j].report_reference_handle =
                        //    p_evt->params.discovered_db.charateristics[i].report_reference_handle;
                        break;
                    }
                }
            }
        }

        if (p_ble_hids_c->evt_handler != NULL)
        {
            p_ble_hids_c->evt_handler(p_ble_hids_c, &evt);
        }
    }
    else if ((p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND) ||
             (p_evt->evt_type == BLE_DB_DISCOVERY_ERROR))
    {
        NRF_LOG_DEBUG("HID Service discovery failure at peer. ");
    }
    else
    {
        // Do nothing.
    }
}*/

void ble_hids_c_on_ble_evt(ble_hids_c_t * p_ble_hids_c, ble_evt_t const * p_ble_evt)
{
    if ((p_ble_hids_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            // nothing to do, connection handle will be assigned in db discovery handler
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_ble_hids_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_hids_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_rsp(p_ble_hids_c, p_ble_evt);
            break;

        /*case BLE_GATTC_EVT_READ_RSP:
            on_read_rsp(p_ble_hids_c, p_ble_evt);
            break;*/

        default:
            break;
    }
}

/*uint32_t ble_hids_c_hid_info_read(ble_hids_c_t * p_ble_hids_c)
{
    if (p_ble_hids_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_ble_hids_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gq_message_t msg;

    memset(&msg, 0, sizeof(ble_gq_message_t));
    msg.conn_handle     = p_ble_hids_c->conn_handle;
    msg.type            = BLE_GQ_READ_REQ;
    msg.req.read_handle = p_ble_hids_c->peer_hids_db.hid_info_handle;

    return ble_gq_message_add(p_ble_hids_c->p_gq_inst, &msg);
}

uint32_t ble_hids_c_report_map_read(ble_hids_c_t * p_ble_hids_c)
{
    if (p_ble_hids_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_ble_hids_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    ble_gq_message_t msg;

    memset(&msg, 0, sizeof(ble_gq_message_t));
    msg.conn_handle     = p_ble_hids_c->conn_handle;
    msg.type            = BLE_GQ_READ_REQ;
    msg.req.read_handle = p_ble_hids_c->peer_hids_db.report_map_handle;

    return ble_gq_message_add(p_ble_hids_c->p_gq_inst, &msg);
}

uint32_t ble_hids_c_report_read(ble_hids_c_t * p_ble_hids_c, uint8_t index)
{
    if (p_ble_hids_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_ble_hids_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (index >= NUM_OF_REPORT_HANDLES)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    ble_gq_message_t msg;

    memset(&msg, 0, sizeof(ble_gq_message_t));
    msg.conn_handle     = p_ble_hids_c->conn_handle;
    msg.type            = BLE_GQ_READ_REQ;
    msg.req.read_handle = p_ble_hids_c->peer_hids_db.report_handles[index].report_handle;

    return ble_gq_message_add(p_ble_hids_c->p_gq_inst, &msg);
}

uint32_t ble_hids_c_report_reference_read(ble_hids_c_t * p_ble_hids_c, uint8_t index)
{
    if (p_ble_hids_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_ble_hids_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (index >= NUM_OF_REPORT_HANDLES)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    ble_gq_message_t msg;

    memset(&msg, 0, sizeof(ble_gq_message_t));
    msg.conn_handle     = p_ble_hids_c->conn_handle;
    msg.type            = BLE_GQ_READ_REQ;
    msg.req.read_handle = p_ble_hids_c->peer_hids_db.report_handles[index].report_reference_handle;

    return ble_gq_message_add(p_ble_hids_c->p_gq_inst, &msg);
}*/

uint32_t ble_hids_c_report_notif_enable(ble_hids_c_t * p_ble_hids_c, uint8_t index, bool enable)
{
    if (p_ble_hids_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (index >= NUM_OF_REPORT_HANDLES)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    return cccd_configure(p_ble_hids_c, p_ble_hids_c->peer_hids_db.report_handles[index].cccd_handle, enable);
}

uint32_t ble_hids_c_handles_assign(ble_hids_c_t *         p_ble_hids_c,
                                  uint16_t                conn_handle,
                                  ble_hids_c_db_t const * p_peer_handles)
{
    if (p_ble_hids_c == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_ble_hids_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_hids_c->peer_hids_db = *p_peer_handles;
    }

    //return nrf_ble_gq_conn_handle_register(p_ble_hids_c->p_gatt_queue, conn_handle);
    return NRF_SUCCESS;
}

//#endif // NRF_MODULE_ENABLED(BLE_HIDS_C)
/**END OF FILE*****************************************************************/



