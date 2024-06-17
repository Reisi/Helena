/**
  ******************************************************************************
  * @file    ble_hps_discovery_c.h
  * @author  Thomas Reisnecker
  * @brief   discovery for the helen project service
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_HPS_DISC_H_INCLUDED
#define BLE_HPS_DISC_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"
#include "stdint.h"
#include "ble_hps_c.h"

/* Defines -------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    BLE_HPS_DISCOVERY_COMPLETE,     // event indicating that the service discovery is complete
    BLE_HPS_DISCOVERY_SRV_NOT_FOUND,// event indicating that the helen project service was not found at the peer
} ble_hps_discovery_evt_type_t;

typedef struct
{
    ble_hps_discovery_evt_type_t type;          // the type of event
    uint16_t                     conn_handle;   // the connection handle this event is related to
    ble_hps_c_db_t const       * p_db;          // the discovered hps database for event BLE_HPS_DISCOVERY_COMPLETE
} ble_hps_discovery_evt_t;

typedef void (* ble_hps_discovery_evt_handler)(ble_hps_discovery_evt_t const * p_evt);

typedef struct
{
    ble_hps_discovery_evt_handler evt_handler;
    ble_srv_error_handler_t       error_handler;
    ble_gattc_handle_range_t      handle_range;
    ble_hps_c_db_t                hps_db;
    uint8_t                       uuid_type;
    uint16_t                    * p_conn_handles;
} ble_hps_discovery_db_t;

typedef struct
{
    ble_hps_discovery_evt_handler evt_handler;
    ble_srv_error_handler_t       error_handler;
    uint8_t                       uuid_type;
} ble_hps_discovery_init_t;

/* Exported macros ---------------------------------------------------------- */
#define GLUE(X, Y)  X ## Y
#define BLE_HPS_DISC_DEF(_name, _hps_max_clients)                            \
static uint16_t GLUE(_name, _conn_handles)[_hps_max_clients];                \
static ble_hps_discovery_db_t _name =                                        \
{                                                                            \
    .p_conn_handles = GLUE(_name, _conn_handles),                            \
};

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the hps discovery module
 *
 * @param[in] p_hps_discovery  the database buffer used during discovery
 * @param[in] evt_handler      the event handler
 * @return NRF_SUCCESS or NRF_ERROR_NULL
 */
ret_code_t ble_hps_discovery_init(ble_hps_discovery_db_t * p_hps_discovery, uint8_t max_clinets, ble_hps_discovery_init_t const * p_init);

/** @brief function to start a hps discovery
 *
 * @param[in] p_hps_discovery  the database buffer used during discovery
 * @param[in] conn_handle      the connection handle of the peer to start the discovery
 * @return NRF_SUCCESS,
 *         NRF_ERROR_BUSY if buffer is already in use
 */
ret_code_t ble_hps_discovery_start(ble_hps_discovery_db_t * p_hps_discovery, uint16_t conn_handle);

/** @brief function for handling the ble events
 *
 * @param[in] p_hps_discovery  the database buffer used during discovery
 * @param[in] p_ble_evt b      the received ble event
 */
void ble_hps_discovery_on_ble_evt(ble_hps_discovery_db_t * p_hps_discovery, ble_evt_t const * p_ble_evt);


#endif /* BLE_HPS_DISC_H_INCLUDED */

/**END OF FILE*****************************************************************/
