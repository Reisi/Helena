/**@file
 *
 * @brief    Light Control Service Client module.
 *
 * @details  This module contains APIs to read and interact with the Light
 *           Control Service of a remote device.
 *
 * @note     The application must propagate BLE stack events to this module by
 *           calling ble_lcs_c_on_ble_evt().
 *
 */
#ifndef BLE_LCS_C_H_INCLUDED
#define BLE_LCS_C_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"

#define BLE_UUID_LCS_SERVICE 0x0101     /**< The UUID of the light control service */

/**
 * @defgroup lcs_c_enums Enumerations
 * @{
 */

/**@brief Light Control Service Client event type. */
typedef enum
{
    BLE_LCS_C_EVT_DISCOVERY_COMPLETE,   /**< Event indicating that the Light Control Service has been discovered at the peer. */
    BLE_LCS_C_EVT_DISCOVERY_FAILED,     /**< Event indicating that the Light Control Service has not been discovered at the peer. */
    BLE_LCS_C_EVT_MEASUREMENT_NOTIFY,   /**< Event indicating that a notification of the Light Control Measurement characteristic has been received from the peer. */
    BLE_LCS_C_EVT_FEATURE_READ_RESP,    /**< Event indicating that a read response on Light Control Feature characteristic has been received from peer. */
    BLE_LCS_C_EVT_CONTROL_POINT_INDIC,  /**< Event indicating that a indication of Light Control Control Point characteristic has been received from peer. */
} ble_lcs_c_evt_type_t;

/**@breif Light Control Service Control Point Commands
 */
typedef enum
{
    BLE_LCS_C_CP_CMD_REQ_MODE_CNT   = 1,    /**< command to request the max. number of supported modes */
    BLE_LCS_C_CP_CMD_SET_MODE       = 2,    /**< command to set the current mode */
    BLE_LCS_C_CP_CMD_REQ_GRP_CNFG   = 3,    /**< command to request the current group configuration */
    BLE_LCS_C_CP_CMD_CNFG_GROUP     = 4,    /**< command to set the group configuration */
    BLE_LCS_C_CP_CMD_REQ_MODE_CNFG  = 5,    /**< command to request the current mode configuration */
    BLE_LCS_C_CP_CMD_CNFG_MODE      = 6,    /**< command to set a new mode configuration */
    BLE_LCS_C_CP_CMD_REQ_LED_CNFG   = 7,    /**< command to request the led configuration */
    BLE_LCS_C_CP_CMD_CHK_LED_CNFG   = 8,    /**< command to start the procedure to check the led configuration */
    BLE_LCS_C_CP_CMD_REQ_SENS_OFF   = 9,    /**< command to request the sensor offset values */
    BLE_LCS_C_CP_CMD_CALIB_SENS_OFF = 10,   /**< command to initiate a sensor offset calibration */
    BLE_LCS_C_CP_CMD_REQ_LIMITS     = 11,   /**< command to request the current led driver limitations */
    BLE_LCS_C_CP_CMD_SET_LIMITS     = 12,   /**< command to set a new current limit */
    BLE_LCS_C_CP_CMD_RESPONSE       = 32    /**< response code */
} ble_lcs_c_cpc_t;

/**@brief Light Control Service Control Point response values
 */
typedef enum
{
    BLE_LCS_C_CP_RV_SUCCESS         = 1,
    BLE_LCS_C_CP_RV_NOT_SUPPORTED   = 2,
    BLE_LCS_C_CP_RV_INVALID         = 3,
    BLE_LCS_C_CP_RV_FAILED          = 4
} ble_lcs_c_cprv_t;

/** @} */

/**
 * @defgroup lcs_c_structs Structures
 * @{
 */

 /**@brief Light Measurement Flags structure. This contains information which
 *        data fields are present in the light measurement notification package. */
typedef struct
{
    uint16_t intensity_present     : 1;
    uint16_t flood_status_present  : 1;
    uint16_t spot_status_present   : 1;
    uint16_t flood_power_present   : 1;
    uint16_t spot_power_present    : 1;
    uint16_t temperature_present   : 1;
    uint16_t input_voltage_present : 1;
    uint16_t pitch_present         : 1;
} ble_lcs_c_lm_flags_t;

/**@brief Light Control Service Light Setup Flags */
typedef struct
{
    uint8_t flood             : 1;
    uint8_t spot              : 1;
    uint8_t pitchCompensation : 1;
    uint8_t cloned            : 1;
    uint8_t taillight         : 1;
    uint8_t brakelight        : 1;
} ble_lcs_c_light_setup_t;

/**@brief Light Control Service Light Mode structure */
typedef struct
{
    ble_lcs_c_light_setup_t setup;
    uint8_t                 intensity;
} ble_lcs_c_light_mode_t;

 /**@brief Light Measurement status flags structure. This contains the light status flags */
typedef struct
{
    uint8_t overcurrent : 1;
    uint8_t voltage     : 1;
    uint8_t temperature : 1;
    uint8_t dutycycle   : 1;
} ble_lcs_c_lm_status_flags_t;

/**@brief Light Measurement characteristic structure. This structure contains
 *        all data for the Light Measurement characteristic */
typedef struct
{
    ble_lcs_c_lm_flags_t        flags;        /**< flags containing information which data fields are present */
    ble_lcs_c_light_mode_t      mode;         /**< Light Mode */
    ble_lcs_c_lm_status_flags_t flood_status; /**< Flood Status flags */
    ble_lcs_c_lm_status_flags_t spot_status;  /**< Spot Status flags */
    uint16_t                    flood_power;  /**< Flood beam output power in Watt with a resolution of 1/1000 */
    uint16_t                    spot_power;   /**< Spot beam output power in Watt with a resolution of 1/1000 */
    int8_t                      temperature;  /**< Light temperature in degrees Celcius with a resolution of 1 */
    uint16_t                    input_voltage;/**< Input Voltage in Volt with a resolution of 1/1000 */
    int8_t                      pitch;        /**< pitch angle in degree with a resolution of 1 */
} ble_lcs_c_lm_t;

/**@brief Light feature characteristic structure. This contains the supported features */
typedef struct
{
    uint16_t flood_supported                : 1;
    uint16_t spot_supported                 : 1;
    uint16_t pitch_comp_supported           : 1;
    uint16_t mode_change_supported          : 1;
    uint16_t mode_config_supported          : 1;
    uint16_t mode_grouping_supported        : 1;
    uint16_t led_config_check_supported     : 1;
    uint16_t sensor_calibration_supported   : 1;
    uint16_t current_limitation_supported   : 1;
    uint16_t external_taillight_supported   : 1;
    uint16_t external_brake_light_supported : 1;
} ble_lcs_c_lf_t;

/**@brief Control Point mode configuration response data */
typedef struct
{
    uint8_t                       start;        /**< mode number of the first mode in the list */
    uint8_t                       num_of_modes; /**< total number of modes in the list */
    ble_lcs_c_light_mode_t const* pModes;       /**< mode list */
} ble_lcs_c_cp_mcfg_t;

/**@brief Control Point led configuration response data */
typedef struct
{
    uint8_t flood_cnt;                          /**< number of in series connected LEDs an flood side */
    uint8_t spot_cnt;                           /**< number of in series connected LEDs an spot side */
} ble_lcs_c_cp_ledcfg_t;

/**@brief Control Point sensor offset response data */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ble_lcs_c_cp_off_t;

/**@brief Control Point current limit response data */
typedef struct
{
    uint8_t flood;                             /**< current limit in % for flood */
    uint8_t spot;                              /**< current limit in % for spot */
} ble_lcs_c_cp_currlmt_t;

/**@brief Light Control Point Response structure */
typedef struct
{
    ble_lcs_c_cpc_t  command;               /**< the command this response belongs to */
    ble_lcs_c_cprv_t response_value;        /**< response value for this operation */
    union
    {
        uint8_t mode_cnt;                   /**< number of available modes. This field will be used for the response to @ref BLE_LCS_C_CP_CMD_REQ_MODE_CNT */
        uint8_t group_cnt;                  /**< number of groups. This field will be used for the response to @ref BLE_LCS_C_CP_CMD_REQ_GRP_CNFG */
        ble_lcs_c_cp_mcfg_t mode_cfg;       /**< mode configuration. This field will be used for the response to @ref BLE_LCS_C_CP_CMD_REQ_MODE_CNFG */
        ble_lcs_c_cp_ledcfg_t led_cfg;      /**< led configuration. This field will be used for the response to @ref BLE_LCS_C_CP_CMD_REQ_LED_CNFG and @ref BLE_LCS_C_CP_CMD_CHK_LED_CNFG */
        ble_lcs_c_cp_off_t sens_off;        /**< sensor offset. This field will be used for the response to @ref BLE_LCS_C_CP_CMD_REQ_SENS_OFF and @ref BLE_LCS_C_CP_CMD_CALIB_SENS_OFF */
        ble_lcs_c_cp_currlmt_t curr_limit;  /**< current limits. This field will be used for the response to @ref BLE_LCS_C_CP_CMD_REQ_LIMITS */
    } params;
} ble_lcs_c_cp_rsp_t;

/**@brief Light Control Service Server specific data */
typedef struct
{
    uint16_t                conn_handle;        /**< connection handle as provided by the softdevice */
    uint16_t                lcm_handle;         /**< Handle of the Light Control Measurement Characteristic */
    uint16_t                lcm_cccd_handle;    /**< Handle of the CCCD of the Light Control Measurement Characteristic */
    uint16_t                lcf_handle;         /**< Handle of the Light Control Feature Characteristic */
    uint16_t                lccp_handle;        /**< Handle of the Light Control Control Point Characteristic */
    uint16_t                lccp_cccd_handle;   /**< Handle of the CCCD of the Light Control Control Point Characteristic */
} ble_lcs_c_server_spec_t;

/**@brief Light Control Service Event structure */
typedef struct
{
    ble_lcs_c_evt_type_t      evt_type;     /**< type of event */
    ble_lcs_c_server_spec_t * p_server;     /**< pointer to server data this event is related to */
    union
    {
        ble_lcs_c_lm_t        measurement;  /**< light control measurement received from the peer. This field will be used for the response to @ref BLE_LCS_C_EVT_MEASUREMENT_NOTIFY */
        ble_lcs_c_lf_t        feature;      /**< light control feature received from the peer. This field will be used for the response to @ref BLE_LCS_C_EVT_FEATURE_READ_RESP */
        ble_lcs_c_cp_rsp_t    control_point;/**< light control control point indication received from the peer. This field will be used for the response to @ref BLE_LCS_C_EVT_CONTROL_POINT_INDIC */
    } data;
} ble_lcs_c_evt_t;

/** @} */

/**
 * @defgroup gap_c_types Types
 * @{
 */

/**@brief forward declaration of the Light Control Client Structure */
typedef struct ble_lcs_c_s ble_lcs_c_t;

/**@brief Event handler type
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_lcs_c_evt_handler_t) (ble_lcs_c_t * p_lcs_c, ble_lcs_c_evt_t * p_evt);

/**@brief Light Control Service Client Structure */
struct ble_lcs_c_s
{
    uint8_t                   uuid_type;        /**< the uuid type as delivered by the softdevice */
    ble_lcs_c_evt_handler_t   evt_handler;      /**< Event handler to be called for handling events related to the Light Control Service Client */
    ble_lcs_c_server_spec_t * p_server;         /**< server specific data */
};

/** @} */

/**
 * @addtogroup lcs_c_structs
 * @{
 */

/**@brief Light Control Control Point send command structure */
typedef struct
{
    ble_lcs_c_cpc_t command;
    union
    {
        uint8_t                mode_to_set; /**< mode to set. This field will be used for the command @ref BLE_LCS_C_CP_CMD_SET_MODE. */
        uint8_t                group_cnt;   /**< number of group. This field will be used for the command @ref BLE_LCS_C_CP_CMD_CNFG_GROUP. */
        uint8_t                start_mode;  /**< start mode for mode list. This field will be used for the command @ref BLE_LCS_C_CP_CMD_REQ_MODE_CNFG. */
        ble_lcs_c_cp_mcfg_t    mode_cfg;    /**< new list of modes. This field will be used for the command @ref BLE_LCS_C_CP_CMD_CNFG_MODE. */
        ble_lcs_c_cp_currlmt_t curr_limit;  /**< new current limits. This field will be used for the command @ref BLE_LCS_C_CP_CMD_SET_LIMITS. */
    } params;
} ble_lcs_c_cp_write_t;

/**@brief Light Control Service Client initialization structure. */
typedef struct
{
    ble_lcs_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Light Control Service Client module whenever there is an event related to the Light Control Service. */
} ble_lcs_c_init_t;

extern ble_uuid128_t ble_lcs_c_base_uuid;

#define GLUE(X, Y)  X ## Y
#define BLE_LCS_C_DEF(_name, _lcs_max_servers)                              \
static ble_lcs_c_server_spec_t GLUE(_name, _server_data)[_lcs_max_servers]; \
static ble_lcs_c_t _name =                                                  \
{                                                                           \
    .p_server = GLUE(_name, _server_data),                                  \
}

/** @} */

/**
 * @defgroup lcs_c_functions Functions
 * @{
 */

/**@brief      Function for initializing the Light Control Service Client module.
 *
 * @details    This function will initialize the module and set up Database Discovery to discover
 *             the Generic Attribute. After calling this function, call @ref ble_db_discovery_start
 *             to start discovery.
 *
 * @param[out] p_ble_lcs_c      Pointer to the Light Control Service client structure.
 * @param[in]  num_of_servers   Number of available server specific structures
 * @param[in]  p_ble_lcs_c_init Pointer to the Light Control Service initialization structure containing
 *                              the initialization information.
 *
 * @retval     NRF_SUCCESS      Operation success.
 * @retval     NRF_ERROR_NULL   A parameter is NULL.
 *                              Otherwise, an error code returned by @ref ble_db_discovery_evt_register.
 */
uint32_t ble_lcs_c_init(ble_lcs_c_t * p_ble_lcs_c, uint8_t num_of_servers, ble_lcs_c_init_t * p_ble_lcs_c_init);


/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function will handle the BLE events received from the SoftDevice. If the BLE
 *            event is relevant for the Light Control Client module, then it is used to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @note      This function must be called by the application.
 *
 * @param[in] p_ble_lcs_c  Pointer to the Light Control Service client structure.
 * @param[in] p_ble_evt    Pointer to the BLE event.
 */
void ble_lcs_c_on_ble_evt(ble_lcs_c_t * p_ble_lcs_c, ble_evt_t const * p_ble_evt);


/**@brief   Function for requesting the peer to start sending notification of Light Control
 *          Measurement.
 *
 * @details This function will enable to notification of the Light Control Measurement at the peer
 *          by writing to the CCCD of the Light Control Measurement Characteristic.
 *
 * @param[in]  p_ble_lcs_c  Pointer to the Light Control Service client structure.
 * @param[in]  enable       true to enable notification, false to disable.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_lcs_c_lcm_notif_enable(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle, bool enable);


/**@brief   Function for reading the Light Control Feature characteristic.
 *
 * @param[in]  p_ble_lcs_c  Pointer to the Light Control Service client structure.
 *
 * @retval    NRF_SUCCESS If the read request was successfully queued to be sent to peer.
 */
uint32_t ble_lcs_c_feature_read(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle);


/**@brief   Function for writing to the Light Control Feature control point.
 *
 * @param[in] p_ble_lcs_c  Pointer to the Light Control Service client structure.
 * @param[in] p_command    pointer to the command structure
 *
 * @retval    NRF_SUCCESS If the write request was successfully queued to be sent to peer.
 */
uint32_t ble_lcs_c_cp_write(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle, ble_lcs_c_cp_write_t const * p_command);


/**@brief   Function for requesting the peer to start sending indications of Light Control
 *          Control Point.
 *
 * @details This function will enable to indication of the Light Control Control Point at the peer
 *          by writing to the CCCD of the Light Control Control Point Characteristic.
 *
 * @param[in]  p_ble_lcs_c  Pointer to the Light Control Service client structure.
 * @param[in]  enable       true to enable notification, false to disable.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_lcs_c_cp_indic_enable(ble_lcs_c_t * p_ble_lcs_c, uint16_t conn_handle, bool enable);

/** @} */ // End tag for Function group.

#endif // BLE_LCS_C_H_INCLUDED

/** @} */ // End tag for the file.
