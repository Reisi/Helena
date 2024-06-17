/**
  ******************************************************************************
  * @file    ble_hps_discovery.c
  * @author  Thomas Reisnecker
  * @brief   module to discover the helen project service at a peer
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ble_hps_discovery.h"
#include "string.h"

/* Private defines -----------------------------------------------------------*/
#define SRV_DISC_START_HANDLE  0x0001

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t m_num_of_clients;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static void send_error(ble_hps_discovery_db_t const * p_hps_discovery, ret_code_t err_code)
{
    if (p_hps_discovery->error_handler == NULL)
    {
        return;
    }

    p_hps_discovery->error_handler(err_code);
}

static void remove_conn_handle(uint16_t conn_handle, uint16_t * p_conn_handles, uint8_t num_of_conn_handles)
{
    uint8_t i;

    // go to index of requested conn handle
    for (i = 0; i < num_of_conn_handles; i++)
    {
        if (p_conn_handles[i] == conn_handle)
            break;
    }

    // in case of conn handle not found
    if (i == num_of_conn_handles)
        return;

    // shift all handles after requested conn handles one position up
    for (; i < num_of_conn_handles; i++)
    {
        p_conn_handles[i] = i == num_of_conn_handles - 1 ? BLE_CONN_HANDLE_INVALID : p_conn_handles[i + 1];
    }
}

static ret_code_t start_next_discovery(uint8_t uuid_type, uint16_t * p_conn_handles, uint8_t num_of_conn_handles)
{
    ret_code_t err_code, return_code = NRF_SUCCESS;
    ble_uuid_t hps_uuid;

    hps_uuid.type = uuid_type;
    hps_uuid.uuid = BLE_UUID_HPS_SERVICE;

    while (p_conn_handles[0] != BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gattc_primary_services_discover(p_conn_handles[0], SRV_DISC_START_HANDLE, &hps_uuid);

        if (err_code == NRF_SUCCESS)
        {
            break;
        }

        return_code = err_code;
        remove_conn_handle(p_conn_handles[0], p_conn_handles, num_of_conn_handles);
    }

    return return_code;
}

static void on_finished(ble_hps_discovery_db_t * p_hps_discovery)
{
    ret_code_t err_code;

    p_hps_discovery->hps_db.cp_handle      = BLE_GATT_HANDLE_INVALID;
    p_hps_discovery->hps_db.cp_cccd_handle = BLE_GATT_HANDLE_INVALID;

    remove_conn_handle(p_hps_discovery->p_conn_handles[0], p_hps_discovery->p_conn_handles, m_num_of_clients);

    err_code = start_next_discovery(p_hps_discovery->uuid_type, p_hps_discovery->p_conn_handles, m_num_of_clients);
    if (err_code != NRF_SUCCESS)
    {
        send_error(p_hps_discovery, err_code);
    }
}

static void send_event(ble_hps_discovery_db_t const * p_hps_discovery,
                       ble_hps_discovery_evt_type_t type,
                       uint16_t conn_handle,
                       ble_hps_c_db_t const * p_db)
{
    if (p_hps_discovery->evt_handler == NULL)
    {
        return;
    }

    ble_hps_discovery_evt_t evt;
    evt.type = type;
    evt.conn_handle = conn_handle;
    evt.p_db = p_db;
    p_hps_discovery->evt_handler(&evt);
}

static void on_primary_srv_discovery_rsp(ble_hps_discovery_db_t * const p_hps_discovery,
                                         ble_gattc_evt_t const * p_ble_gattc_evt)
{
    if (p_ble_gattc_evt->conn_handle != p_hps_discovery->p_conn_handles[0])
    {
        return;
    }

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        send_event(p_hps_discovery, BLE_HPS_DISCOVERY_SRV_NOT_FOUND, p_ble_gattc_evt->conn_handle, NULL);
        on_finished(p_hps_discovery);
        return;
    }

    p_hps_discovery->handle_range = p_ble_gattc_evt->params.prim_srvc_disc_rsp.services[0].handle_range;

    ret_code_t err_code;
    err_code = sd_ble_gattc_characteristics_discover(p_ble_gattc_evt->conn_handle,
                                                     &p_hps_discovery->handle_range);

    if (err_code != NRF_SUCCESS)
    {
        send_error(p_hps_discovery, err_code);
        on_finished(p_hps_discovery);
    }
}

static void on_characteristic_discovery_rsp(ble_hps_discovery_db_t * p_hps_discovery,
                                            ble_gattc_evt_t const *    p_ble_gattc_evt)
{
    ret_code_t err_code;

    if (p_ble_gattc_evt->conn_handle != p_hps_discovery->p_conn_handles[0])
    {
        return;
    }

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        send_event(p_hps_discovery, BLE_HPS_DISCOVERY_SRV_NOT_FOUND, p_ble_gattc_evt->conn_handle, NULL);
        on_finished(p_hps_discovery);
        return;
    }

    for (uint32_t i = 0; i < p_ble_gattc_evt->params.char_disc_rsp.count; i++)
    {
        ble_gattc_char_t const * p_char = p_ble_gattc_evt->params.char_disc_rsp.chars;

        p_hps_discovery->handle_range.start_handle = p_char[i].handle_value + 1;

        if (p_char[i].uuid.uuid == BLE_UUID_HPS_CP_CHARACTERISTIC &&
            p_char[i].uuid.type == p_hps_discovery->uuid_type)
        {
            p_hps_discovery->hps_db.cp_handle = p_char[i].handle_value;

            err_code = sd_ble_gattc_descriptors_discover(p_ble_gattc_evt->conn_handle,
                                                     &p_hps_discovery->handle_range);

            if (err_code != NRF_SUCCESS)
            {
                send_error(p_hps_discovery, err_code);
                on_finished(p_hps_discovery);
            }

            return;
        }
    }

    err_code = sd_ble_gattc_characteristics_discover(p_ble_gattc_evt->conn_handle,
                                                     &p_hps_discovery->handle_range);

    if (err_code != NRF_SUCCESS)
    {
        send_error(p_hps_discovery, err_code);
        on_finished(p_hps_discovery);
    }
}

static void on_descriptor_discovery_rsp(ble_hps_discovery_db_t * p_hps_discovery,
                                            ble_gattc_evt_t const *    p_ble_gattc_evt)
{
    ret_code_t err_code;

    if (p_ble_gattc_evt->conn_handle != p_hps_discovery->p_conn_handles[0])
    {
        return;
    }

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        send_event(p_hps_discovery, BLE_HPS_DISCOVERY_SRV_NOT_FOUND, p_ble_gattc_evt->conn_handle, NULL);
        on_finished(p_hps_discovery);

        return;
    }

    for (uint32_t i = 0; i < p_ble_gattc_evt->params.desc_disc_rsp.count; i++)
    {
        ble_gattc_desc_t const * p_desc = p_ble_gattc_evt->params.desc_disc_rsp.descs;

        p_hps_discovery->handle_range.start_handle = p_desc[i].handle + 1;

        if (p_desc[i].uuid.uuid == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG)
        {
            p_hps_discovery->hps_db.cp_cccd_handle = p_desc[i].handle;

            send_event(p_hps_discovery, BLE_HPS_DISCOVERY_COMPLETE, p_ble_gattc_evt->conn_handle, &p_hps_discovery->hps_db);
            on_finished(p_hps_discovery);

            return;
        }
    }

    err_code = sd_ble_gattc_descriptors_discover(p_ble_gattc_evt->conn_handle,
                                                     &p_hps_discovery->handle_range);

    if (err_code != NRF_SUCCESS)
    {
        send_error(p_hps_discovery, err_code);
        on_finished(p_hps_discovery);
    }
}

static void on_disconnect(ble_hps_discovery_db_t * p_hps_discovery, ble_gap_evt_t const * p_gap_evt)
{
    if (p_gap_evt->conn_handle == p_hps_discovery->p_conn_handles[0])
    {
        on_finished(p_hps_discovery);
    }
    else
    {
        remove_conn_handle(p_gap_evt->conn_handle, p_hps_discovery->p_conn_handles, m_num_of_clients);
    }
}

/* Public functions ----------------------------------------------------------*/
ret_code_t ble_hps_discovery_init(ble_hps_discovery_db_t * p_hps_discovery, uint8_t max_clients, ble_hps_discovery_init_t const * p_init)
{
    if (p_hps_discovery == NULL || p_init == NULL || p_init->evt_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_hps_discovery->evt_handler = p_init->evt_handler;
    p_hps_discovery->error_handler = p_init->error_handler;
    p_hps_discovery->uuid_type = p_init->uuid_type;
    m_num_of_clients = max_clients;

    p_hps_discovery->hps_db.cp_handle      = BLE_GATT_HANDLE_INVALID;
    p_hps_discovery->hps_db.cp_cccd_handle = BLE_GATT_HANDLE_INVALID;

    for (uint8_t i = 0; i < max_clients; i++)
    {
        p_hps_discovery->p_conn_handles[i] = BLE_CONN_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}

ret_code_t ble_hps_discovery_start(ble_hps_discovery_db_t * p_hps_discovery, uint16_t conn_handle)
{
    if (p_hps_discovery == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint8_t i;
    for (i = 0; i < m_num_of_clients; i++)
    {
        if (p_hps_discovery->p_conn_handles[i] == BLE_CONN_HANDLE_INVALID)
        {
            p_hps_discovery->p_conn_handles[i] = conn_handle;
            break;
        }
    }

    if (i == m_num_of_clients)  // queue is full
    {
        return NRF_ERROR_BUSY;
    }

    if (i > 0)                  // another discovery already in progress, nut successfully queued
    {
        return NRF_SUCCESS;
    }

    return start_next_discovery(p_hps_discovery->uuid_type, p_hps_discovery->p_conn_handles, m_num_of_clients);
}

void ble_hps_discovery_on_ble_evt(ble_hps_discovery_db_t * p_hps_discovery, ble_evt_t const * p_ble_evt)
{
    if (p_hps_discovery == NULL || p_ble_evt == NULL)
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
            on_primary_srv_discovery_rsp(p_hps_discovery, &(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            on_characteristic_discovery_rsp(p_hps_discovery, &(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            on_descriptor_discovery_rsp(p_hps_discovery, &(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_hps_discovery, &(p_ble_evt->evt.gap_evt));
            break;

        default:
            break;
    }
}

/**END OF FILE*****************************************************************/
