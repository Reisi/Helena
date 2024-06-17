/**
  ******************************************************************************
  * @file    ble_hids_c.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/12/13
  * @brief   HID over GATT Service Client module
  * @note    This is a to the needs of this project reduced variant, which does
  *          not use ble_db_discovery. The handles need to be supplied manually
  *          with the ble_hids_c_handles_assign() function
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_HIDS_C_H_INCLUDED
#define BLE_HIDS_C_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_gatt_queue.h"

/* Defines -------------------------------------------------------------------*/
#define NUM_OF_REPORT_HANDLES   1

/* Exported types ------------------------------------------------------------*/
/**@brief HIDS Client event type */
typedef enum
{
    //BLE_HIDS_C_EVT_DISCOVERY_COMPLETE = 1,
    //BLE_HIDS_C_EVT_HID_INFO_READ_RESP,
    //BLE_HIDS_C_EVT_REPORT_MAP_READ_RESP,
    BLE_HIDS_C_EVT_REPORT_NOTIFICATION = 4,
    //BLE_HIDS_C_EVT_REPORT_READ_RESP = 5,
    //BLE_HIDS_C_EVT_REPORT_REFERENCE_READ_RESP
} ble_hids_c_evt_type_t;

/**@brief Structure containing the hid information */
//typedef struct
//{
//    uint16_t bcd_hid;
//    uint8_t country_code;
//    uint8_t remote_wake : 1;
//    uint8_t normally_connectable : 1;
//} ble_hids_c_hid_info_t;

//typedef enum
//{
//    BLE_HIDS_C_REPORT_TYPE_INPUT   = 1,
//    BLE_HIDS_C_REPORT_TYPE_OUTPUT  = 2,
//    BLE_HIDS_C_REPORT_TYPE_FEATURE = 3
//} ble_hids_c_report_type_t;

//typedef struct
//{
//    const uint8_t * p_report_map;   /**< report map data */
//    uint16_t        len;            /**< report map length */
//} ble_hids_c_report_map_t;

typedef struct
{
    const uint8_t * p_report;       /**< hid report data */
    uint16_t        len;            /**< length of report */
    uint8_t         index;          /**< index of the report */
} ble_hids_c_report_t;

/*typedef struct
{
    uint8_t                  index;
    uint8_t                  report_id;
    ble_hids_c_report_type_t report_type;
} ble_hids_c_report_ref_t;*/

/**@brief report handles structure */
typedef struct
{
    uint16_t report_handle;             /**< handle of the report characteristic as provided by the BLE stack. */
    uint16_t cccd_handle;               /**< handle of the CCCD of the report characteristic as provided by the BLE stack. */
    //uint16_t report_reference_handle;   /**< handle of the report reference of the report characteristic as provided by the BLE stack. */
} ble_hids_c_report_handles_t;

/**@brief Structure containing the handles related to the HID Service found on the peer. */
typedef struct
{
    //uint16_t                    hid_info_handle;                        /**< Handle of the hid information characteristic as provided by the ble stack. */
    //uint16_t                    report_map_handle;                      /**< Handle of the report map characteristic as provided by the ble stack */
    ble_hids_c_report_handles_t report_handles[NUM_OF_REPORT_HANDLES];  /**< Handles of the report characteristics */
} ble_hids_c_db_t;

/**@brief HID Service Client Event structure */
typedef struct
{
    ble_hids_c_evt_type_t       evt_type;   /**< Type of event */
    uint16_t                    conn_handle;/**< Connection handle relevant to this event.*/
    union
    {
        //ble_hids_c_db_t         hid_db;     /**< Handles related to the HID Service, found on the peer device. This is filled if the evt_type id @ref BLE_HIDS_C_EVT_DISCOVERY_COMPLETE.*/
        //ble_hids_c_hid_info_t   hid_info;   /**< HID information. This field is used for the events @ref BLE_HIDS_C_EVT_HID_INFO_READ_RESP.*/
        //ble_hids_c_report_map_t report_map; /**< report map. This field is used for the events @ref BLE_HIDS_C_EVT_REPORT_MAP_READ_RESP.*/
        ble_hids_c_report_t     report;     /**< report. This field is used for the events @ref BLE_HIDS_C_EVT_REPORT_NOTIFICATION and @ref BLE_HIDS_C_EVT_REPORT_READ_RESP. */
        //ble_hids_c_report_ref_t report_ref; /**< report reference. This field is used for the events @ref BLE_HIDS_C_EVT_REPORT_REFERENCE_READ_RESP. */
    } params;
} ble_hids_c_evt_t;

// Forward declaration of the ble_hids_c_t type
typedef struct ble_hids_c_s ble_hids_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by
 *          the application of this module in order to receive events.
 */
typedef void (* ble_hids_c_evt_handler_t)(ble_hids_c_t * p_ble_hids_c, ble_hids_c_evt_t * p_evt);

/**@brief HID client structure */
struct ble_hids_c_s
{
    uint16_t                    conn_handle;                            /**< Connection handle as provided by the ble stack. */
    ble_hids_c_db_t             peer_hids_db;                           /**< Handles related to the HID on the peer.*/
    ble_srv_error_handler_t     error_handler;                          /**< Application error handler to be called in case of an error. */
    ble_hids_c_evt_handler_t    evt_handler;                            /**< Application event handler to be called when there is an event related to the hid service. */
    ble_gq_inst_t             * p_gq_inst;                              /**< Pointer to BLE GATT Queue instance. */
};

/**@brief HID client initialization structure */
typedef struct
{
    ble_srv_error_handler_t     error_handler;                          /**< Application error handler to be called in case of an error. */
    ble_hids_c_evt_handler_t    evt_handler;                            /**< Application event handler to be called when there is an event related to the hid service. */
    ble_gq_inst_t             * p_gq_inst;                              /**< Pointer to BLE GATT Queue instance. */
} ble_hids_c_init_t;

/* Exported functions ------------------------------------------------------- */
/** @brief        Function for initializing the hid client module
 *
 * @details       This function will register with the DB Discovery module.
 *                There it registers for the HID Service. Doing so will make
 *                the DB Discovery module look for the presence of a HID
 *                Service instance at the peer when a discovery is started.
 *
 * @param[in/out] p_ble_hids_c      Pointer to the HID client structure.
 * @param[in]     p_ble_hids_c_init Pointer to the HID initialization structure
 * @return        NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                            propagates the error code returned by the Database Discovery module API
 *                            @ref ble_db_discovery_evt_register.
 */
uint32_t ble_hids_c_init(ble_hids_c_t * p_ble_hids_c, ble_hids_c_init_t * p_ble_hids_c_init);

/**@brief     Function for handling events from the database discovery module.
 *
 * @details   This function will handle an event from the database discovery module, and determine
 *            if it relates to the discovery of HID service at the peer. If so, it will
 *            call the application's event handler indicating that the HID service has been
 *            discovered at the peer. It also populate the event with the service related
 *            information before providing it to the application.
 *
 * @param[in] p_ble_hids_c Pointer to the HID client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
 //void ble_hids_c_on_db_disc_evt(ble_hids_c_t * p_ble_hids_c, ble_db_discovery_evt_t const * p_evt);

/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function will handle the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the HID Client module, then it uses it to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_hids_c Pointer to the HID client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event.
 */
void ble_hids_c_on_ble_evt(ble_hids_c_t * p_ble_hids_c, ble_evt_t const * p_ble_evt);

/**@brief     Function for reading the HID information characteristic.
 *
 * @param[in] p_ble_hids_c Pointer to the HID client structure.
 *
 * @return    NRF_SUCCESS If the read request was successfully queued to be sent to peer.
 */
//uint32_t ble_hids_c_hid_info_read(ble_hids_c_t * p_ble_hids_c);

/**@brief     Function for reading the report map characteristic.
 *
 * @param[in] p_ble_hids_c Pointer to the HID client structure.
 *
 * @return    NRF_SUCCESS If the read request was successfully queued to be sent to peer.
 */
//uint32_t ble_hids_c_report_map_read(ble_hids_c_t * p_ble_hids_c);

/**@brief     Function for reading the report characteristic.
 *
 * @param[in] p_ble_hids_c Pointer to the HID client structure.
 * @param[in] index        index of the report handle to be read
 *
 * @return    NRF_SUCCESS If the read request was successfully queued to be sent to peer.
 */
//uint32_t ble_hids_c_report_read(ble_hids_c_t * p_ble_hids_c, uint8_t index);

/**@brief     Function for reading the report reference characteristic.
 *
 * @param[in] p_ble_hids_c Pointer to the HID client structure.
 * @param[in] index        index of the report handle to be read
 *
 * @return    NRF_SUCCESS If the read request was successfully queued to be sent to peer.
 */
//uint32_t ble_hids_c_report_reference_read(ble_hids_c_t * p_ble_hids_c, uint8_t index);

/**@brief     Function for requesting the peer to change sending notifications of reports
 *
 * @param[in] p_ble_hids_c Pointer to the HID client structure.
 * @param[in] index        index of the report handle to to change sending notifications
 * @param[in] enable       true to start, false to stop notifications
 *
 * @return    NRF_SUCCESS If the request was successfully queued to be sent to peer.
 */
uint32_t ble_hids_c_report_notif_enable(ble_hids_c_t * p_ble_hids_c, uint8_t index, bool enable);

/**@brief     Function for assigning handles to this instance of hids_c.
 *
 * @details   Call this function when a link has been established with a peer to
 *            associate the link to this instance of the module. This makes it
 *            possible to handle several links and associate each link to a particular
 *            instance of this module. The connection handle and attribute handles are
 *            provided from the discovery event @ref BLE_HIDS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_hids_c   Pointer to the HID client structure instance for associating the link.
 * @param[in] conn_handle    Connection handle associated with the given HID Client Instance.
 * @param[in] p_peer_handles Attribute handles on the HID server you want this HID client to
 *                           interact with.
 */
uint32_t ble_hids_c_handles_assign(ble_hids_c_t *         p_ble_hids_c,
                                  uint16_t                conn_handle,
                                  ble_hids_c_db_t const * p_peer_handles);

#endif /* BLE_HIDS_C_H_INCLUDED */

/**END OF FILE*****************************************************************/
