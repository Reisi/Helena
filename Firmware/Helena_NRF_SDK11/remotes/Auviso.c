/**
  ******************************************************************************
  * @file    Auviso.h
  * @author  Thomas Reisnecker
  * @brief   Pearl/Auviso Remote Control driver
  *
  * @note this is a project specific driver for the Pearl/Auviso Remote control.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "remote.h"
#include "hmi.h"
#include "string.h"
#include "app_error.h"

//#include "KD2_curr.h"

/* Defines -------------------------------------------------------------------*/
#define DEVICE_NAME             "SmartRemote"
#define UUID16_SIZE             2       /**< Size of 16 bit UUID. */
#define USAGE_PLAY_PAUSE        0xCD    // sent by click on the play/pause button
#define USAGE_VOLUME_INCREMENT  0xE9    // sent by single click on the volume up button
#define USAGE_VOLUME_DECREMENT  0xEA    // sent by single click on the volume down button
#define USAGE_SCAN_NEXT_TRACK   0xB5    // sent by single click on the next track button
#define USAGE_SCAN_PREV_TRACK   0xB6    // sent by single click on the previous track button

/* Read only variables -------------------------------------------------------*/
static const ble_hids_c_db_t device_handles =
{
    //.hid_info_handle             = 0x0011,
    //.report_map_handle           = 0x0026,
    .report_handles              =
    {
        /*{
            .report_handle           = 0x0013,
            .cccd_handle             = 0,
            .report_reference_handle = 0,
        },
        {
            .report_handle           = 0x001b,
            .cccd_handle             = 0,
            .report_reference_handle = 0,
        },*/
        {
            .report_handle           = 0x0017,
            .cccd_handle             = 0x0018,
            //.report_reference_handle = 0,
        }
    }
};

/* Exported functions --------------------------------------------------------*/
bool rem_auviso_is_device(uint8_t const* p_data, uint16_t len)
{
    uint_fast8_t index = 0;

    bool match_uuid = false, match_name = false;

    while (index < len)
    {
        uint8_t length = p_data[index];
        uint8_t type   = p_data[index+1];

        // check if the device advertises as a HID device
        if (type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE ||
            type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
        {
            for (uint_fast8_t i = 0; i < length/UUID16_SIZE; i++)
            {
                uint16_t uuid;
                uuid = uint16_decode(&p_data[index + 2 + i * UUID16_SIZE]);
                if (uuid == BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE)
                    match_uuid = true;
            }
        }
        // check if the device name matches
        else if (type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME ||
                 type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME)
        {
            if (strncmp(DEVICE_NAME, (char*)&p_data[index + 2], strlen(DEVICE_NAME)) == 0)
                match_name = true;
        }

        index += length + 1;
    }
    return match_uuid && match_name;
}

ble_hids_c_db_t const* rem_auviso_get_handles(void)
{
    return &device_handles;
}

uint32_t rem_auviso_notif_enable(ble_hids_c_t * p_ble_hids_c, bool enable)
{
    return ble_hids_c_report_notif_enable(p_ble_hids_c, 0, enable);
}

void rem_auviso_on_hids_c_evt(ble_hids_c_t * p_ble_hids_c, ble_hids_c_evt_t * p_evt)
{
    (void)p_ble_hids_c;

    if (p_evt->evt_type != BLE_HIDS_C_EVT_REPORT_NOTIFICATION || p_evt->params.report.index != 0)
    {
        return;
    }

    uint32_t err_code;

    switch(p_evt->params.report.p_report[0])
    {
    case USAGE_PLAY_PAUSE:
        err_code = hmi_RequestEvent(HMI_EVT_MODEPREF, 0);
        APP_ERROR_CHECK(err_code);
        break;
    case USAGE_VOLUME_INCREMENT:
        err_code = hmi_RequestEvent(HMI_EVT_NEXTGROUP, 0);
        APP_ERROR_CHECK(err_code);
        break;
    case USAGE_VOLUME_DECREMENT:
        err_code = hmi_RequestEvent(HMI_EVT_PREVGROUP, 0);
        APP_ERROR_CHECK(err_code);
        break;
    case USAGE_SCAN_NEXT_TRACK:
        err_code = hmi_RequestEvent(HMI_EVT_NEXTMODE, 0);
        APP_ERROR_CHECK(err_code);
        break;
    case USAGE_SCAN_PREV_TRACK:
        err_code = hmi_RequestEvent(HMI_EVT_PREVMODE, 0);
        APP_ERROR_CHECK(err_code);
        break;
    default:
        return; // no event for this cases
    }
}

/* section variable ----------------------------------------------------------*/
REMOTE_REGISTER(const rem_driver_t rem_auviso) =
{
    .is_device = rem_auviso_is_device,
    .get_handles = rem_auviso_get_handles,
    .notif_enable = rem_auviso_notif_enable,
    .evt_handler = rem_auviso_on_hids_c_evt,
};

/**END OF FILE*****************************************************************/
