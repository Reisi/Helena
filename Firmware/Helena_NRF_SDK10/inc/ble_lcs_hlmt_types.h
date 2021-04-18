/**
  ******************************************************************************
  * @file    ble_lcs_hmlt_types.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/08
  * @brief   Light Control Helmet Type types declarations
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BLE_LCS_HLMT_TYPES_H_INCLUDED
#define BLE_LCS_HLMT_TYPES_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/**@brief Light Control Service Helmet Light Setup Flags */
typedef struct
{
    uint8_t flood             : 1;
    uint8_t spot              : 1;
    uint8_t pitchCompensation : 1;
    uint8_t cloned            : 1;
    uint8_t taillight         : 1;
    uint8_t brakelight        : 1;
} ble_lcs_hlmt_light_setup_t;

/**@brief Light Control Service Helmet Light Mode structure */
typedef struct
{
    ble_lcs_hlmt_light_setup_t setup;
    uint8_t                    intensity;
} ble_lcs_hlmt_light_mode_t;

/**@brief Helmet Light feature structure. This contains the supported features */
typedef struct
{
    uint8_t flood_supported                : 1;
    uint8_t spot_supported                 : 1;
    uint8_t pitch_comp_supported           : 1;
    uint8_t driver_cloning_supported       : 1;
    uint8_t external_taillight_supported   : 1;
    uint8_t external_brake_light_supported : 1;
} ble_lcs_hlmt_lf_t;

/**@brief Light Measurement Flags structure. This contains information which
 *        data fields are present in the light measurement notification package
 *        corresponding to a Helmet Type Light. */
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
    uint16_t battery_soc_present   : 1;
} ble_lcs_hlmt_lm_flags_t;

#define BLE_LCS_HLMT_LM_FLAGS_MASK           (0x01FF)

/**@brief Light Measurement status flags structure. This contains the light
 *        status flags corresponding to a Helmet Type Light */
typedef struct
{
    uint8_t overcurrent : 1;
    uint8_t voltage     : 1;
    uint8_t temperature : 1;
    uint8_t dutycycle   : 1;
} ble_lcs_hlmt_lm_status_flags_t;

#define BLE_LCS_LM_STATUS_FLAGS_MASK    (0x0F)

/**@brief Light Measurement characteristic structure. This structure contains
 *        all data for the Light Measurement characteristic corresponding to a
 *        Helmet Type Light */
typedef struct
{
    ble_lcs_hlmt_lm_flags_t        flags;        /**< flags containing information which data fields are present */
    ble_lcs_hlmt_light_mode_t      mode;         /**< Light Mode */
    ble_lcs_hlmt_lm_status_flags_t flood_status; /**< Flood Status flags */
    ble_lcs_hlmt_lm_status_flags_t spot_status;  /**< Spot Status flags */
    uint16_t                       flood_power;  /**< Flood beam output power in Watt with a resolution of 1/1000 */
    uint16_t                       spot_power;   /**< Spot beam output power in Watt with a resolution of 1/1000 */
    int8_t                         temperature;  /**< Light temperature in degrees Celcius with a resolution of 1 */
    uint16_t                       input_voltage;/**< Input Voltage in Volt with a resolution of 1/1000 */
    int8_t                         pitch;        /**< pitch angle in degree with a resolution of 1 */
    uint8_t                        battery_soc;  /**< battery state of charge in percent with resolution of 1 */
} ble_lcs_hlmt_lm_t;


#endif // BLE_LCS_HLMT_TYPES_H_INCLUDED
