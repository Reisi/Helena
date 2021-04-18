/**
  ******************************************************************************
  * @file    ble_hids_c_hardcoded.h
  * @author  Thomas Reisnecker
  * @brief   HID over GATT Service Client module
  *
  * @details This is a special hardcoded extension of the HID over GATT mosule,
  *          that uses hardcoded data instead of database discovery. For now
  *          the Xiaomi Yi Remote and the R51 "The Lord of Rings" is supported.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_HIDS_C_HARDCODED_H_INCLUDED
#define BLE_HIDS_C_HARDCODED_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_hids_c.h"

/* Defines -------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**@brief HIDS Client event type */
typedef enum
{
    BLE_HIDS_C_HC_DEVICE_UNKNOWN = 0,
    BLE_HIDS_C_HC_DEVICE_XIAOMI,
    BLE_HIDS_C_HC_DEVICE_R51,
    BLE_HIDS_C_HC_DEVICE_CNT
} ble_hids_c_hc_device_t;

/* Exported functions ------------------------------------------------------- */
/** @brief        Function for initializing the hid client module with
 *                hardcoded data
 *
 * @details       This function replaces the @ref ble_hids_c_init  function and
 *                sets the handles according to the selected remote. Because it
 *                is not registering with the database discovery module, it can
 *                be called multiple times to change the type of remote if
 *                necessary, but you have to set the conn_handle manually.
 *
 * @param[in/out] p_ble_hids_c_hc      Pointer to the HID client structure.
 * @param[in]     p_ble_hids_c_hc_init Pointer to the HID initialization structure
 * @param[in]     device_type          the type of device to be initialized
 * @return        NRF_SUCCESS On successful initialization. Otherwise an error code.
 */
uint32_t ble_hids_c_hc_init(ble_hids_c_t * p_ble_hids_c_hc, ble_hids_c_init_t * p_ble_hids_c_hc_init, ble_hids_c_hc_device_t device_type);

/** @brief    Function to parse advertising report to identify if the device is
 *            a supported remote device
 *
 * @param[in] pAdvData      the received advertising packet
 * @return    BLE_HIDS_C_HC_DEVICE_UNKNOWN if advertising report doesn't match to
 *            any supported device, otherwise the detected device.
 */
ble_hids_c_hc_device_t ble_hids_c_hc_is_device(const ble_gap_evt_adv_report_t * pAdvData);

#endif // BLE_HIDS_C_HARDCODED_H_INCLUDED

/**END OF FILE*****************************************************************/
