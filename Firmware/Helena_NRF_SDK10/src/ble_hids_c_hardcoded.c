/**
  ******************************************************************************
  * @file    ble_hids_c_hardcoded.c
  * @author  Thomas Reisnecker
  * @brief   HID over GATT Service Client module extension for hardcoded devices
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ble_hids_c_hardcoded.h"
#include "nrf_error.h"
#include "ble_srv_common.h"
#include "nordic_common.h"

/* Private defines -----------------------------------------------------------*/
#define UUID16_SIZE 2

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const ble_hids_c_t device_handels[BLE_HIDS_C_HC_DEVICE_CNT - 1] =
{
    {
        .hid_info_handle             = 0x001F,
        .report_map_handle           = 0x0023,
        {
            {
                .report_handle           = 0x002C,
                .cccd_handle             = 0x002E,
                .report_reference_handle = 0,
            },
            {
                .report_handle           = 0x0030,
                .cccd_handle             = 0,
                .report_reference_handle = 0,
            },
            {
                .report_handle           = 0x0033,
                .cccd_handle             = 0x0035,
                .report_reference_handle = 0,
            }
        }
    },
    {
        .hid_info_handle             = 0x000D,
        .report_map_handle           = 0x0011,
        {
            {
                .report_handle           = 0x001A,
                .cccd_handle             = 0x001C,
                .report_reference_handle = 0,
            },
            {
                .report_handle           = 0x001E,
                .cccd_handle             = 0x0020,
                .report_reference_handle = 0,
            },
            {
                .report_handle           = 0x0022,
                .cccd_handle             = 0x0024,
                .report_reference_handle = 0,
            }
        }
    },
    {
        .hid_info_handle             = 0x0011,
        .report_map_handle           = 0x0026,
        {
            {
                .report_handle           = 0x0013,
                .cccd_handle             = 0,
                .report_reference_handle = 0,
            },
            {
                .report_handle           = 0x001b,
                .cccd_handle             = 0,
                .report_reference_handle = 0,
            },
            {
                .report_handle           = 0x0017,
                .cccd_handle             = 0x0018,
                .report_reference_handle = 0,
            }
        }
    }
};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static bool is_valid_device_type(ble_hids_c_hc_device_t device_type)
{
    if ((device_type - BLE_HIDS_C_HC_DEVICE_XIAOMI) < (BLE_HIDS_C_HC_DEVICE_CNT - BLE_HIDS_C_HC_DEVICE_XIAOMI))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* Public functions ----------------------------------------------------------*/
uint32_t ble_hids_c_hc_init(ble_hids_c_t * p_ble_hids_c_hc, ble_hids_c_init_t * p_ble_hids_c_hc_init, ble_hids_c_hc_device_t device_type)
{
    if (p_ble_hids_c_hc == NULL || p_ble_hids_c_hc_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (!is_valid_device_type(device_type))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(p_ble_hids_c_hc, &device_handels[device_type - BLE_HIDS_C_HC_DEVICE_XIAOMI], sizeof(ble_hids_c_t));

    p_ble_hids_c_hc->evt_handler = p_ble_hids_c_hc_init->evt_handler;

    return NRF_SUCCESS;
}

ble_hids_c_hc_device_t ble_hids_c_hc_is_device(const ble_gap_evt_adv_report_t * p_adv_data)
{
    uint_fast8_t index = 0;

    bool is_hid = false;
    ble_hids_c_hc_device_t device_type = BLE_HIDS_C_HC_DEVICE_UNKNOWN;

    while (index < p_adv_data->dlen)
    {
        uint8_t length = p_adv_data->data[index];
        uint8_t type   = p_adv_data->data[index+1];

        if (type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE ||
            type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
        {
            for (uint_fast8_t i = 0; i < length/UUID16_SIZE; i++)
            {
                uint16_t uuid;
                uuid = uint16_decode(&p_adv_data->data[index + 2 + i * UUID16_SIZE]);
                if (uuid == BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE)
                    is_hid = true;
            }
        }
        else if (type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME ||
                 type == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME)
        {
            if (strncmp("XiaoYi_RC", (char*)&p_adv_data->data[index + 2], strlen("XiaoYi")) == 0)
                device_type = BLE_HIDS_C_HC_DEVICE_XIAOMI;
            else if (strncmp("R51", (char*)&p_adv_data->data[index + 2], strlen("R51")) == 0)
                device_type = BLE_HIDS_C_HC_DEVICE_R51;
            else if (strncmp("SmartRemote", (char*)&p_adv_data->data[index + 2], strlen("SmartRemote")) == 0)
                device_type = BLE_HIDS_C_HC_DEVICE_AUVISO;
        }

        index += length + 1;
    }
    return is_hid ? device_type : BLE_HIDS_C_HC_DEVICE_UNKNOWN;
}

/**END OF FILE*****************************************************************/
