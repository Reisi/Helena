/**
  ******************************************************************************
  * @file    ble_lcs_bk_types.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/08
  * @brief   Light Control Bike Type types declarations
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_LCS_BK_TYPES_H_INCLUDED
#define BLE_LCS_BK_TYPES_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/**@brief Light Control Service Bike Light Setup Flags */
typedef struct
{
    uint8_t main_beam          : 1;
    uint8_t extended_main_beam : 1;
    uint8_t high_beam          : 1;
    uint8_t daylight_active    : 1;
    uint8_t taillight          : 1;
    uint8_t brakelight         : 1;
} ble_lcs_bk_light_setup_t;

/**@brief Light Control Service Helmet Light Mode structure */
typedef struct
{
    ble_lcs_bk_light_setup_t setup;
    uint8_t                  main_beam_intensity;
    uint8_t                  high_beam_intensity;
} ble_lcs_bk_light_mode_t;

/**@brief Helmet Light feature structure. This contains the supported features */
typedef struct
{
    uint16_t main_beam_supported            : 1;
    uint16_t extenden_main_beam_supported   : 1;
    uint16_t high_beam_supported            : 1;
    uint16_t daylight_supported             : 1;
    uint16_t mode_change_supported          : 1;
    uint16_t mode_config_supported          : 1;
    uint16_t mode_grouping_supported        : 1;
    uint16_t led_config_check_supported     : 1;
    uint16_t sensor_calibration_supported   : 1;
    uint16_t current_limitation_supported   : 1;
    uint16_t external_taillight_supported   : 1;
    uint16_t external_brake_light_supported : 1;
    uint16_t preferred_mode_supported       : 1;
    uint16_t temporary_mode_supported       : 1;
} ble_lcs_bk_lf_t;

/**@brief Light Measurement Flags structure. This contains information which
 *        data fields are present in the light measurement notification package
 *        corresponding to a Bike Type Light. */
typedef struct
{
    uint16_t main_beam_intensity_present : 1;
    uint16_t main_beam_status_present    : 1;
    uint16_t main_beam_power_present     : 1;
    uint16_t high_beam_intensity_present : 1;
    uint16_t high_beam_status_present    : 1;
    uint16_t high_beam_power_present     : 1;
    uint16_t temperature_present         : 1;
    uint16_t input_voltage_present       : 1;
    uint16_t pitch_present               : 1;
    uint16_t battery_soc_present         : 1;
} ble_lcs_bk_lm_flags_t;

#define BLE_LCS_BK_LM_FLAGS_MASK           (0x03FF)

/**@brief Light Measurement status flags structure. This contains the light
 *        status flags corresponding to a Bike Type Light */
typedef struct
{
    uint8_t overcurrent : 1;
    uint8_t voltage     : 1;
    uint8_t temperature : 1;
    uint8_t dutycycle   : 1;
} ble_lcs_bk_lm_status_flags_t;

#define BLE_LCS_LM_STATUS_FLAGS_MASK    (0x0F)

/**@brief Light Measurement characteristic structure. This structure contains
 *        all data for the Light Measurement characteristic corresponding to a
 *        Bike Type Light */
typedef struct
{
    ble_lcs_bk_lm_flags_t        flags;            /**< flags containing information which data fields are present */
    ble_lcs_bk_light_mode_t      mode;             /**< Light Mode */
    ble_lcs_bk_lm_status_flags_t main_beam_status; /**< Main Beam Status flags */
    ble_lcs_bk_lm_status_flags_t high_beam_status; /**< High Beam Status flags */
    uint16_t                     main_beam_power;  /**< Main beam output power in Watt with a resolution of 1/1000 */
    uint16_t                     high_beam_power;  /**< High beam output power in Watt with a resolution of 1/1000 */
    int8_t                       temperature;      /**< Light temperature in degrees Celcius with a resolution of 1 */
    uint16_t                     input_voltage;    /**< Input Voltage in Volt with a resolution of 1/1000 */
    int8_t                       pitch;            /**< pitch angle in degree with a resolution of 1 */
    uint8_t                      battery_soc;      /**< battery state of charge in percent with a resolution of 1 */
} ble_lcs_bk_lm_t;

#endif // BLE_LCS_BK_TYPES_H_INCLUDED
