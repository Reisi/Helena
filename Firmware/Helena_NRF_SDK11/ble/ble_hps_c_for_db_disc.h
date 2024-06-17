/**@file
 *
 * @brief    Helen Project Service Client module.
 *
 * @details  This module contains a reduced APIs to just set the mode of a
 *           remote device
 *
 * @note     The application must propagate BLE stack events to this module by
 *           calling ble_hps_c_on_ble_evt().
 *
 */
#ifndef BLE_HPS_C_H_INCLUDED
#define BLE_HPS_C_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt_queue.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"

#define BLE_UUID_HPS_SERVICE    0x0301      /**< The UUID of the Helen Project service */

/**
 * @defgroup hps_c_enums Enumerations
 * @{
 */

/**@brief Helen Project Service Client event type. */
typedef enum
{
    BLE_HPS_C_EVT_DISCOVERY_COMPLETE,   /**< Event indicating that the Light Control Service has been discovered at the peer. */
    BLE_HPS_C_EVT_DISCOVERY_FAILED,     /**< Event indicating that the Light Control Service has not been discovered at the peer. */
    BLE_HPS_C_EVT_CONTROL_POINT_INDIC,  /**< Event indicating that a indication of Light Control Control Point characteristic has been received from peer. */
} ble_hps_c_evt_type_t;

/**@brief Helen Project Service Control Point Commands
 */
typedef enum
{
    BLE_HPS_C_CP_CMD_REQ_MODE          = 1, /**< command to request the current mode of the remote device */
    BLE_HPS_C_CP_CMD_SET_MODE          = 2, /**< command to request to set the mode of the remote device */
    BLE_HPS_C_CP_CMD_REQ_SEARCH        = 3, /**< command to request the start of the search procedure */
    BLE_HPS_C_CP_CMD_REQ_F_RESET       = 5, /**< command to request a factory reset */
    BLE_HPS_C_CP_CMD_SET_MODE_OVERRIDE = 8, /**< command to override current mode */
    BLE_HPS_C_CP_CMD_RESPONSE          = 32 /**< response code */
} ble_hps_c_cpc_t;

/**@brief Helen Project Service Control Point response values
 */
typedef enum
{
    BLE_HPS_C_CP_RV_SUCCESS         = 1,
    BLE_HPS_C_CP_RV_NOT_SUPPORTED   = 2,
    BLE_HPS_C_CP_RV_INVALID         = 3,
    BLE_HPS_C_CP_RV_FAILED          = 4
} ble_hps_c_cprv_t;

/** @} */

/**
 * @defgroup hps_c_structs Structures
 * @{
 */

/**@brief Control Point Response structure */
typedef struct
{
    ble_hps_c_cpc_t  command;               /**< the command this response belongs to */
    ble_hps_c_cprv_t response_value;        /**< response value for this operation */
} ble_hps_c_cp_rsp_t;

/**@brief   Structure containing the handles related to the Helen Project Service found on the peer. */
typedef struct
{
    uint16_t                cp_handle;        /**< Handle of the Control Point Characteristic */
    uint16_t                cp_cccd_handle;   /**< Handle of the CCCD of the Control Point Characteristic */
} ble_hps_c_db_t;

/**@brief Helen Project Service Server specific data */
typedef struct
{
    uint16_t                conn_handle;        /**< connection handle as provided by the softdevice */
    ble_hps_c_db_t          hps_db;             /**< Handles related to HPS on the peer*/
} ble_hps_c_server_spec_t;

/**@brief Helen Project Service Event structure */
typedef struct
{
    ble_hps_c_evt_type_t      evt_type;     /**< type of event */
    union
    {
        uint16_t conn_handle;               /**< This is only filled if the evt_type is @ref BLE_HPS_C_EVT_DISCOVERY_COMPLETE. */
        ble_hps_c_server_spec_t * p_server; /**< pointer to server data this event is related to */
    };
    union
    {
        ble_hps_c_db_t        hps_db;       /**< Helen Control Service related handles found on the peer device. This is filled if the evt_type is @ref BLE_HPS_C_EVT_DISCOVERY_COMPLETE.*/
        ble_hps_c_cp_rsp_t    control_point;/**< control point indication received from the peer. This field will be used for the response to @ref BLE_HPS_C_EVT_CONTROL_POINT_INDIC */
    } data;
} ble_hps_c_evt_t;

/**@brief forward declaration of the Helen Project Client Structure */
typedef struct ble_hps_c_s ble_hps_c_t;

/**@brief Event handler type
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_hps_c_evt_handler_t) (ble_hps_c_t * p_hps_c, ble_hps_c_evt_t * p_evt);

/**@brief Error event handler type
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive error events.
 */
//typedef void (* ble_lcs_c_error_handler_t) (ble_lcs_c_t * p_lcs_c, ble_lcs_c_error_evt_t * p_evt);

/**@brief Helen Project Service Client Structure */
struct ble_hps_c_s
{
    uint8_t                   uuid_type;        /**< the uuid type as delivered by the softdevice */
    ble_hps_c_server_spec_t * p_server;         /**< server specific data */
    ble_hps_c_evt_handler_t   evt_handler;      /**< Event handler to be called for handling events related to the Helen Project Service Client */
    ble_srv_error_handler_t   error_handler;    /**< Error handler to be called for handling error related to the Helen Project Service Client */
    ble_gq_inst_t           * p_gq_inst;        /**< Pointer to BLE GATT Queue instance. */
};

/** @} */

/**
 * @addtogroup hps_c_structs
 * @{
 */

 /**@brief Control Point mode override channel configuration structure */
 typedef struct
{
    void const * p_config;
    uint16_t     size;
} ble_hps_c_cp_channel_config_t;

/**@brief Control Point send command structure */
typedef struct
{
    ble_hps_c_cpc_t command;
    union
    {
        uint8_t                               mode_to_set;      /**< mode to set. This field will be used for the command @ref BLE_HPS_C_CP_CMD_SET_MODE. */
        ble_hps_c_cp_channel_config_t const * p_channel_config; /**< the channel config. This field will be used for the command @ref BLE_HPS_C_CP_CMD_MODE_OVERRIDE. */
    } params;
} ble_hps_c_cp_write_t;

/**@brief Helen Project Service Client initialization structure. */
typedef struct
{
    ble_hps_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Helen Project Service Client module whenever there is an event related to the Helen Project Service. */
    ble_srv_error_handler_t error_handler;/**< Error handler to be called by the Helen Project Service Client module whenever there is an error related to the Helen Project Service. */
    ble_gq_inst_t         * p_gq_inst;    /**< Pointer to BLE GATT Queue instance. */
} ble_hps_c_init_t;

//extern const ble_uuid128_t ble_lcs_c_base_uuid;

#define BLE_HPS_C_DEF(_name, _hps_max_servers)                          \
static ble_hps_c_server_spec_t _name ## _server_data[_hps_max_servers]; \
static ble_hps_c_t _name = { .p_server = _name ## _server_data,};       /*\
NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_HPS_C_BLE_OBSERVER_PRIO,        \
                     ble_hps_c_on_ble_evt, &_name)*/

/** @} */

/**
 * @defgroup hps_c_functions Functions
 * @{
 */

/**@brief      Function for initializing the Helen Project Service Client module.
 *
 * @details    This function will initialize the module and set up Database Discovery to discover
 *             the Generic Attribute. After calling this function, call @ref ble_db_discovery_start
 *             to start discovery.
 *
 * @param[out] p_ble_hps_c      Pointer to the Light Control Service client structure.
 * @param[in]  num_of_servers   Number of available server specific structures
 * @param[in]  p_ble_hps_c_init Pointer to the Light Control Service initialization structure containing
 *                              the initialization information.
 *
 * @retval     NRF_SUCCESS      Operation success.
 * @retval     NRF_ERROR_NULL   A parameter is NULL.
 *                              Otherwise, an error code returned by @ref ble_db_discovery_evt_register.
 */
uint32_t ble_hps_c_init(ble_hps_c_t * p_ble_hps_c, uint8_t num_of_servers, ble_hps_c_init_t * p_ble_hps_c_init);


/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function will handle the BLE events received from the SoftDevice. If the BLE
 *            event is relevant for the Helen Project Client module, then it is used to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @note      This function must be called by the application.
 *
 * @param[in] p_ble_hps_c  Pointer to the Helen Project Service client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event.
 */
void ble_hps_c_on_ble_evt(ble_hps_c_t * p_ble_hps_c, ble_evt_t const * p_ble_evt);


/**@brief   Function for writing to the Helen Project Feature control point.
 *
 * @param[in] p_ble_hps_c  Pointer to the Helen Project Service client structure.
 * @param[in] p_command    pointer to the command structure
 *
 * @retval    NRF_SUCCESS If the write request was successfully queued to be sent to peer.
 */
uint32_t ble_hps_c_cp_write(ble_hps_c_t * p_ble_hps_c, uint16_t conn_handle, ble_hps_c_cp_write_t const * p_command);


/**@brief   Function for requesting the peer to start sending indications of Helen Project
 *          Control Point.
 *
 * @details This function will enable to indication of the Control Point at the peer
 *          by writing to the CCCD of the Control Point Characteristic.
 *
 * @param[in]  p_ble_hps_c  Pointer to the Helen Project Service client structure.
 * @param[in]  enable       true to enable notification, false to disable.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_hps_c_cp_indic_enable(ble_hps_c_t * p_ble_hps_c, uint16_t conn_handle, bool enable);

void ble_hps_c_on_db_disc_evt(ble_hps_c_t * p_ble_hps_c, ble_db_discovery_evt_t const * p_evt);

uint32_t ble_hps_c_handles_assign(ble_hps_c_t * p_ble_hps_c, uint16_t conn_handle, ble_hps_c_db_t * p_peer_handles);

/** @} */ // End tag for Function group.

#endif // BLE_HPS_C_H_INCLUDED

/** @} */ // End tag for the file.
