/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/* This module is an modified version of the original Database Discovery
 * provided by Nordic Semiconductors. The original module stores the
 * information for all registered services in the provided ble_db_discovery_t.
 * Therefore this database needs separate storage for each registered service.
 * This modified version doesn't persistently store this information. After a
 * service discovery is complete and the appropriate service handler was called
 * the database is cleared and reused for the discovery of the next service. It
 * is up to the user to save gatt information if the application needs them.
 */


/**@file
 *
 * @defgroup ble_sdk_lib_db_discovery Database Discovery
 * @{
 * @ingroup  ble_sdk_lib
 * @brief    Database discovery module.
 *
 * @details  This module contains the APIs and types exposed by the DB Discovery module. These APIs
 *           and types can be used by the application to perform discovery of a service and its
 *           characteristics at the peer server. This module can also be used to discover the
 *           desired services in multiple remote devices.
 *           A typical use of this library is described in the figure below.
 *           @image html db_discovery.jpg
 *
 * @warning  The maximum number of characteristics per service that can be discovered by this module
 *           is determined by the number of characteristics in the service structure defined in
 *           ble_gatt_db.h. If the peer has more than the supported number of characteristics, then
 *           the first found will be discovered and any further characteristics will be ignored. No
 *           descriptors other than Client Characteristic Configuration Descriptors will be searched
 *           for at the peer.
 *
 * @note     Presently only one instance of a Primary Service can be discovered by this module. If
 *           there are multiple instances of the service at the peer, only the first instance
 *           of it at the peer is fetched and returned to the application.
 *
 * @note     The application must propagate BLE stack events to this module by calling
 *           ble_db_discovery_on_ble_evt().
 *
 */

#ifndef BLE_DB_DISCOVERY_H__
#define BLE_DB_DISCOVERY_H__

#include <stdint.h>
#include "ble_gattc.h"
#include "ble.h"
#include "nrf_error.h"
#include "ble_srv_common.h"
#include "ble_gatt_db.h"
#include "peer_manager_types.h"

/**
 * @defgroup db_disc_defines Defines
 * @{
 */

#define BLE_DB_DISCOVERY_MAX_SRV          2  /**< Maximum number of services supported by this module. This also indicates the maximum number of users allowed to be registered to this module. (one user per service). */

/** @} */

/**
 * @defgroup db_disc_enums Enumerations
 * @{
 */

/**@brief   Type of the DB Discovery event.
 */
typedef enum
{
    BLE_DB_DISCOVERY_COMPLETE,      /**< Event indicating that the GATT Database discovery is complete. */
    BLE_DB_DISCOVERY_ERROR,         /**< Event indicating that an internal error has occurred in the DB Discovery module. This could typically be because of the SoftDevice API returning an error code during the DB discover.*/
    BLE_DB_DISCOVERY_SRV_NOT_FOUND  /**< Event indicating that the service was not found at the peer.*/
} ble_db_discovery_evt_type_t;

/** @} */

/**
 * @defgroup db_disc_structs Structures
 * @{
 */


/**@brief   Structure for holding the information related to the GATT database at the server.
 *
 * @details This module identifies a remote database. Use one instance of this structure per
 *          connection.
 *
 * @warning This structure must be zero-initialized.
 */
typedef struct
{
    ble_gatt_db_srv_t   service;                             /**< Information related to the current service being discovered. This is intended for internal use during service discovery.*/
    uint16_t            conn_handle;                         /**< Connection handle as provided by the SoftDevice. */
    uint8_t             srv_count;                           /**< Number of services at the peers GATT database.*/
    uint8_t             curr_char_ind;                       /**< Index of the current characteristic being discovered. This is intended for internal use during service discovery.*/
    uint8_t             curr_srv_ind;                        /**< Index of the current service being discovered. This is intended for internal use during service discovery.*/
    bool                discovery_in_progress;               /**< Variable to indicate if there is a service discovery in progress. */
    uint8_t             discoveries_count;                   /**< Number of service discoveries made, both successful and unsuccessful. */
} ble_db_discovery_t;


/**@brief   Structure containing the event from the DB discovery module to the application.
 */
typedef struct
{
    ble_db_discovery_evt_type_t  evt_type;     /**< Type of event. */
    uint16_t                     conn_handle;  /**< Handle of the connection for which this event has occurred. */
    union
    {
        ble_gatt_db_srv_t const* p_discovered_db;  /**< Structure containing the information about the GATT Database at the server. This will be filled when the event type is @ref BLE_DB_DISCOVERY_COMPLETE.*/
        uint32_t                 err_code;         /**< nRF Error code indicating the type of error which occurred in the DB Discovery module. This will be filled when the event type is @ref BLE_DB_DISCOVERY_ERROR. */
    } params;
} ble_db_discovery_evt_t;

/** @} */

/**
 * @defgroup db_disc_types Types
 * @{
 */

/**@brief   DB Discovery event handler type. */
typedef void (* ble_db_discovery_evt_handler_t)(ble_db_discovery_evt_t * p_evt);

/** @} */

/**
 * @addtogroup db_disc_structs
 * @{
 */

/** @} */

/**
 * @defgroup db_disc_functions Functions
 * @{
 */

/**@brief     Function for initializing the DB Discovery module.
 *
 * @retval    NRF_SUCCESS on successful initialization.
 */
uint32_t ble_db_discovery_init(void);


/**@brief Function for closing the DB Discovery module.
 *
 * @details This function will clear up any internal variables and states maintained by the
 *          module. To re-use the module after calling this function, the function @ref
 *          ble_db_discovery_init must be called again.
 *
 * @retval  NRF_SUCCESS                 Operation success.
 */
uint32_t ble_db_discovery_close(void);


/**@brief Function for registering with the DB Discovery module.
 *
 * @details   The application can use this function to inform which service it is interested in
 *            discovering at the server.
 *
 * @param[in] p_uuid             Pointer to the UUID of the service to be discovered at the server.
 * @param[in] evt_handler        Event handler to be called by the DB discovery module when any event
 *                               related to discovery of the registered service occurs.
 *
 * @note      The total number of services that can be discovered by this module is @ref
 *            BLE_DB_DISCOVERY_MAX_SRV. This effectively means that the maximum number of
 *            registrations possible is equal to the @ref BLE_DB_DISCOVERY_MAX_SRV.
 *
 * @retval    NRF_SUCCESS               Operation success.
 * @retval    NRF_ERROR_NULL            When a NULL pointer is passed as input.
 * @retval    NRF_ERROR_INVALID_STATE   If this function is called without calling the
 *                                      @ref ble_db_discovery_init.
 * @retval    NRF_ERROR_NOT_SUPPORTED   The maximum number of registrations allowed by this module
 *                                      has been reached.
 */
uint32_t ble_db_discovery_evt_register(const ble_uuid_t * const             p_uuid,
                                       const ble_db_discovery_evt_handler_t evt_handler);


/**@brief Function for starting the discovery of the GATT database at the server.
 *
 * @warning p_db_discovery structure must be zero-initialized.
 *
 * @param[out] p_db_discovery    Pointer to the DB Discovery structure.
 * @param[in]  conn_handle       The handle of the connection for which the discovery should be
 *                               started.
 *
 * @retval    NRF_SUCCESS               Operation success.
 * @retval    NRF_ERROR_NULL            When a NULL pointer is passed as input.
 * @retval    NRF_ERROR_INVALID_STATE   If this function is called without calling the
 *                                      @ref ble_db_discovery_init, or without calling
 *                                      @ref ble_db_discovery_evt_register.
 * @retval    NRF_ERROR_BUSY            If a discovery is already in progress for the current
 *                                      connection.
 *
 * @return                              This API propagates the error code returned by the
 *                                      SoftDevice API @ref sd_ble_gattc_primary_services_discover.
 */
uint32_t ble_db_discovery_start(ble_db_discovery_t * const p_db_discovery,
                                uint16_t                   conn_handle);

//uint32_t ble_db_discovery_database_get(pm_peer_data_remote_gatt_db_t * p_database);

//uint32_t ble_db_discovery_database_process(uint16_t                              conn_handle,
//                                           const pm_peer_data_remote_gatt_db_t * p_database);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in,out] p_db_discovery Pointer to the DB Discovery structure.
 * @param[in]     p_ble_evt      Pointer to the BLE event received.
 */
void ble_db_discovery_on_ble_evt(ble_db_discovery_t * const p_db_discovery,
                                 const ble_evt_t * const    p_ble_evt);

/** @} */

#endif // BLE_DB_DISCOVERY_H__

/** @} */
