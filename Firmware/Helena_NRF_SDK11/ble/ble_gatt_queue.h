/**
  ******************************************************************************
  * @file    ble_gatt_queue_c.h
  * @author  Thomas Reisnecker
  * @brief   Gatt queue for client write operation
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_GQ_H_INCLUDED
#define BLE_GQ_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "sdk_errors.h"

/* Configuration -------------------------------------------------------------*/
#define BLE_GQ_TX_BUFFER_SIZE       2  /**< Size of the send buffer. */
#define BLE_GQ_WRITE_MESSAGE_LENGTH 2  /**< Length of the write message for CCCD. */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    BLE_GQ_READ_REQ,      /**< Type identifying that this tx_message is a read request. */
    BLE_GQ_WRITE_REQ      /**< Type identifying that this tx_message is a write request. */
} ble_gq_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
    uint8_t                  gattc_value[BLE_GQ_WRITE_MESSAGE_LENGTH];  /**< The message to write. */
    ble_gattc_write_params_t gattc_params;                              /**< The GATTC parameters for this message. */
} ble_gq_write_params_t;

/**@brief Structure for holding the data that will be transmitted to the connected central.
 */
typedef struct
{
    uint16_t                  conn_handle;  /**< Connection handle to be used when transmitting this message. */
    ble_gq_request_t          type;         /**< Type of message. (read or write). */
    union
    {
        uint16_t              read_handle;  /**< Read request handle. */
        ble_gq_write_params_t write_req;    /**< Write request message. */
    } req;
} ble_gq_message_t;

typedef struct
{
    ble_srv_error_handler_t error_handler;
    ble_gq_message_t        txBuffer[BLE_GQ_TX_BUFFER_SIZE];
    uint8_t                 tx_pending;
    uint8_t                 tx_index;
} ble_gq_inst_t;

/* Exported functions ------------------------------------------------------- */
/** @brief        Function for initializing the gatt queue module
 *
 * @param[in/out] p_gq_inst      Pointer to the gatt queue instance
 * @param[in]     error_handler  the error handler to call on errors.
 * @return        NRF_SUCCESS or NRF_ERROR_NULL
 */
ret_code_t ble_gq_init(ble_gq_inst_t * p_gq_inst, ble_srv_error_handler_t error_handler);

/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @param[in] p_gq_inst Pointer to the gatt queue instance
 */
void ble_gq_on_ble_evt(ble_gq_inst_t * p_gq_inst);

/**@brief     Function for adding a message.
 *
 * @param[in] p_gq_inst Pointer to the gatt queue instance
 * @param[in] p_message message to add
 *
 * @return    NRF_SUCCESS, NRF_ERROR_NULL, or an propagated error
 */
ret_code_t ble_gq_message_add(ble_gq_inst_t * p_gq_inst, ble_gq_message_t const * p_message);

#endif /* BLE_GQ_H_INCLUDED */

/**END OF FILE*****************************************************************/
