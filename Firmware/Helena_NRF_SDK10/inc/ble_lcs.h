/**
  ******************************************************************************
  * @file    ble_lcs.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/08
  * @brief   Light Control Service implementation
  *
  * @details This module implements the Light Control Service with Light
  *          Measurement, Light Control and Light Control Point characteristics.
  *          During initialization it adds the service and characteristics to
  *          the BLE stack database.
  *
  *          If enabled, notifications of the Light Measurements characteristic
  *          is performed when the application calls
  *          ble_lcs_light_measurement_send().
  *
  *          If an event handler is supplied by the application, the Light
  *          Control Service will generate Heart Rate Service events to the
  *          application.
  *
  * @note    The application must propagate BLE stack events to the Light
  *          Control Service module by calling ble_lcs_on_ble_evt() from the
  *          @ref softdevice_handler callback.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_LCS_H_INCLUDED
#define BLE_LCS_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/* Exported defines ----------------------------------------------------------*/
#define BLE_UUID_LCS_SERVICE 0x0101         /**< The UUID of the light control service */

/* Exported types ------------------------------------------------------------*/
/**@brief Light Control Service Event Types */
typedef enum
{
    BLE_LCS_EVT_LM_NOTIVICATION_ENABLED,    /**< Light Measurements notification enabled event */
    BLE_LCS_EVT_LM_NOTIVICATION_DISABLED,   /**< Light Measurements notification disabled event */
} ble_lcs_evt_type_t;

/**@brief Light Control Service Event */
typedef struct
{
    ble_lcs_evt_type_t evt_type;
} ble_lcs_evt_t;

// Forward declaration of the ble_lcs_t type.
typedef struct ble_lcs_s ble_lcs_t;

/**@brief Light Control Service event handler type */
typedef void (*ble_lcs_evt_handler_t)(ble_lcs_t * p_lcs, ble_lcs_evt_t * p_evt);

/**@brief Light Control Service Light Setup Flags */
typedef struct
{
    uint8_t flood             : 1;
    uint8_t spot              : 1;
    uint8_t pitchCompensation : 1;
    uint8_t cloned            : 1;
    uint8_t taillight         : 1;
    uint8_t brakelight        : 1;
} ble_lcs_light_setup_t;

/**@brief Light Control Service Light Mode Types */
/*typedef enum
{
    BLE_LCS_LIGHT_MODE_OFF = 0,
    BLE_LCS_LIGHT_MODE_FLOOD,
    BLE_LCS_LIGHT_MODE_SPOT,
    BLE_LCS_LIGHT_MODE_FLOOD_AND_SPOT,
    BLE_LCS_LIGHT_MODE_FLOOD_PITCH_COMPENSATED,
    BLE_LCS_LIGHT_MODE_SPOT_PITCH_COMPENSATED,
    BLE_LCS_LIGHT_MODE_FLOOD_AND_SPOT_PITCH_COMPENSATED
} ble_lcs_light_mode_type_t;*/

/**@brief Light Control Service Light Mode structure */
typedef struct
{
    ble_lcs_light_setup_t setup;
    uint8_t               intensity;
} ble_lcs_light_mode_t;

/**@brief Light feature structure. This contains the supported features */
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
} ble_lcs_lf_t;

// include control point header here, because its using feature and light mode structures
#include "ble_lcs_ctrlpt.h"

/**@brief Light Control Service init structure. This contains all options and data
 *        needed for initialization of the service */
typedef struct
{
    ble_srv_cccd_security_mode_t    lcs_lm_attr_md; /**< Initial security level for the light measurement characteristic */
    ble_srv_security_mode_t         lcs_lf_attr_md; /**< Initial security level for the light feature characteristic */
    ble_srv_cccd_security_mode_t    lcs_lcp_attr_md;/**< Initial security level for the light control point characteristic */
    ble_lcs_evt_handler_t           evt_handler;    /**< Event handler to be called for handling events in the Light Control Service */
    ble_lcs_ctrlpt_evt_handler_t    cp_evt_handler; /**< Event handler to be called for handling events of the light control control point */
    ble_srv_error_handler_t         error_handler;  /**< Function to be called in case of an error. */
    ble_lcs_lf_t                    features;       /**< supported features by this light */
    ble_lcs_light_mode_t            initial_mode;   /**< Initial mode value */
} ble_lcs_init_t;

/**@brief Light Control Service structure. This contains various status
 *        information for the service */
struct ble_lcs_s
{
    uint8_t                     uuid_type;          /**< UUID type for Light Control Service Base UUID. */
    uint16_t                    conn_handle;        /**< Handle if the current connection (as provided by the BLE stack) */
    uint16_t                    service_handle;     /**< Handle of the Light Control Service (as provided by the BLE stack) */
    ble_gatts_char_handles_t    lm_handles;         /**< Handles related to the Light Measurement characteristic */
    ble_gatts_char_handles_t    lf_handles;         /**< Handles related to the Light feature characteristic */
    ble_lcs_evt_handler_t       evt_handler;        /**< Event handler to be called for handling events in the Light Control Service */
    bool                        is_lm_notfy_enabled;/**< Indicator if light measurement notifications are enabled */
    ble_lcs_lf_t                features;           /**< supported features by this light */
    ble_lcs_ctrlpt_t            ctrl_pt;            /**< data for light control control point */
};

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
} ble_lcs_lm_flags_t;

#define BLE_LCS_LM_FLAGS_MASK           (0x00FF)

/**@brief Light Measurement status flags structure. This contains the light status flags */
typedef struct
{
    uint8_t overcurrent : 1;
    uint8_t voltage     : 1;
    uint8_t temperature : 1;
    uint8_t dutycycle   : 1;
} ble_lcs_lm_status_flags_t;

#define BLE_LCS_LM_STATUS_FLAGS_MASK    (0x0F)

/**@brief Light Measurement characteristic structure. This structure contains
 *        all data for the Light Measurement characteristic */
typedef struct
{
    ble_lcs_lm_flags_t        flags;        /**< flags containing information which data fields are present */
    ble_lcs_light_mode_t      mode;         /**< Light Mode */
    ble_lcs_lm_status_flags_t flood_status; /**< Flood Status flags */
    ble_lcs_lm_status_flags_t spot_status;  /**< Spot Status flags */
    uint16_t                  flood_power;  /**< Flood beam output power in Watt with a resolution of 1/1000 */
    uint16_t                  spot_power;   /**< Spot beam output power in Watt with a resolution of 1/1000 */
    int8_t                    temperature;  /**< Light temperature in degrees Celcius with a resolution of 1 */
    uint16_t                  input_voltage;/**< Input Voltage in Volt with a resolution of 1/1000 */
    int8_t                    pitch;        /**< pitch angle in degree with a resolution of 1 */
} ble_lcs_lm_t;

/* Exported functions ------------------------------------------------------- */
/** @brief Function for initializing the Light Control Service
 *
 * @param[out]  p_lcs       Light Control Service structure. This structure will
 *                          have to be supplied by the application. It will be
 *                          initialized by this function, and will later be used
 *                          to identify this particular service instance
 * @param[in]   p_lcs_init  Information needed to initialize th service
 * @return      NRF_SUCCESS on successful initialization of service, otherwise
 *              an error code
 *
 */
uint32_t ble_lcs_init(ble_lcs_t * p_lcs, const ble_lcs_init_t * p_lcs_init);

/** @brief Function for handling the Applications's BLE stack events.
 *
 * @param[in]   p_lcs       Light Control Service Structure
 * @param[in]   p_ble_evt   Event received from the BLE stack
 */
void ble_lcs_on_ble_evt(ble_lcs_t * p_lcs, ble_evt_t * p_ble_evt);

/** @brief Function for sending light measurements if notifications has been
 *         enabled
 *
 * @details The application can call this function when new measurement data
 *          shall be sent. If notifications has been enabled, the light
 *          measurement data is encoded and sent to the client.
 * @param[in]   p_lcs       Light Control Service Structure
 * @param[in]   p_lcs_lm    New light measurement data
 * @return uint32_t
 *
 */
uint32_t ble_lcs_light_measurement_send(const ble_lcs_t * p_lcs, const ble_lcs_lm_t * p_lcs_lm);

#endif /* BLE_LCS_H_INCLUDED */

/**END OF FILE*****************************************************************/
