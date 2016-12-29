/**@file
 *
 * @brief    Com Gateway Service implementation.
 *
 * @details  The Com Gateway Service is a simple GATT based service with a TX and RX
 *           characteristics. Data received from the peer will be passed to the application and the
 *           data received from the application of this service will be sent to the peer as Handle
 *           Value Notifications.
 *
 * @note     The application must propagate S110 SoftDevice events to the Com Gateway Service module
 *           by calling the ble_cgw_on_ble_evt() function from the @ref ble_stack_handler callback.
 */

#ifndef BLE_CGW_H__
#define BLE_CGW_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "comreloaded.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_CGW_SERVICE            0x0001                       /**< The UUID of the Com Gateway Service. */
#define BLE_UUID_CGW_TX_CHARACTERISTIC  0x0002                       /**< The UUID of the TX Characteristic. */
#define BLE_UUID_CGW_RX_CHARACTERISTIC  0x0003                       /**< The UUID of the RX Characteristic. */

// Forward declaration of the ble_cgw_t type.
typedef struct ble_cgw_s ble_cgw_t;

/**@brief Com Gateway Service event handler type. */
typedef void (*ble_cgw_data_handler_t) (ble_cgw_t * p_cgw, com_MessageStruct * p_message_rx);

/**@brief   Com Gateway Service init structure.
 *
 * @details This structure contains the initialization information for the service. The application
 *          needs to fill this structure and pass it to the service using the @ref ble_cgw_init
 *          function.
 */
typedef struct
{
    ble_cgw_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
} ble_cgw_init_t;

/**@brief   Com Gateway Service structure.
 *
 * @details This structure contains status information related to the service.
 */
typedef struct ble_cgw_s
{
    uint8_t                  uuid_type;               /**< UUID type for Com Gateway Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Com Gateway Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic. (as provided by the S110 SoftDevice)*/
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic. (as provided by the S110 SoftDevice)*/
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the S110 SoftDevice). This will be BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_cgw_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
} ble_cgw_t;

/**@brief       Function for initializing the Com Gateway Service.
 *
 * @param[out]  p_cgw       Com Gateway Service structure. This structure will have to be supplied
 *                          by the application. It will be initialized by this function and will
 *                          later be used to identify this particular service instance.
 * @param[in]   p_cgw_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 *              This function returns NRF_ERROR_NULL if either of the pointers p_cgw or p_cgw_init
 *              is NULL.
 */
uint32_t ble_cgw_init(ble_cgw_t * p_cgw, const ble_cgw_init_t * p_cgw_init);

/**@brief       Com Gateway Service BLE event handler.
 *
 * @details     The Com Gateway service expects the application to call this function each time an
 *              event is received from the S110 SoftDevice. This function processes the event if it
 *              is relevant for it and calls the Com Gateway Service event handler of the
 *              application if necessary.
 *
 * @param[in]   p_cgw      Com Gateway Service structure.
 * @param[in]   p_ble_evt  Event received from the S110 SoftDevice.
 */
void ble_cgw_on_ble_evt(ble_cgw_t * p_cgw, ble_evt_t * p_ble_evt);

/**@brief       Function for sending a message to the peer.
 *
 * @details     This function will send the input message as a RX characteristic notification to the
 *              peer.
  *
 * @param[in]   p_cgw          Pointer to the Com Gateway Service structure.
 * @param[in]   p_message_tx   Pointer to the message to be sent
 *
 * @return      NRF_SUCCESS if the CGW Service has successfully requested the S110 SoftDevice to
 *              send the notification. Otherwise an error code.
 *              This function returns NRF_ERROR_INVALID_STATE if the device is not connected to a
 *              peer or if the notification of the RX characteristic was not enabled by the peer.
 *              It returns NRF_ERROR_NULL if the pointer p_cgw is NULL.
 */
uint32_t ble_cgw_send_message(ble_cgw_t * p_cgw, const com_MessageStruct * p_message_tx);

#endif // BLE_CGW_H__

/** @} */
