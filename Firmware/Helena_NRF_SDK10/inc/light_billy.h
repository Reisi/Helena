/**
  ******************************************************************************
  * @file    light.h
  * @author  Thomas Reisnecker
  * @brief   Header for light.c module (Billy Edition)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LIGHT_H
#define _LIGHT_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "fpint.h"

/* Exported defines ----------------------------------------------------------*/
#define LIGHT_LEDCONFIG_UNKNOWN UINT8_MAX

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    LIGHT_DRIVERREV10 = 0,      // led driver firmware revision 1.0
    LIGHT_DRIVERREV11,          // rev 1.1, added support for dutycyle readback
    LIGHT_DRIVERREV12,          // rev 1.2, added support for calibration over i2c
    LIGHT_DRIVERREVUNKNOWN
} light_driverRevision_t;

typedef struct
{
    uint8_t mainBeamCount;      // number of leds connected in series
    uint8_t highBeamCount;      // number of leds connected in series
    light_driverRevision_t rev;
} light_driverConfig_t;

typedef struct
{
    bool mainBeam         : 1;  // main beam active
    bool extendedMainBeam : 1;  // not supported, will be ignored
    bool highBeam         : 1;  // high Beam active
} light_modeSetup_t;

typedef struct
{
    light_modeSetup_t setup;    // light setup
    uint8_t intensityMainBeam;  // main beam intensity
    uint8_t intensityHighBeam;  // High beam intensity
} light_mode_t;

typedef struct
{
    bool current     : 1;       // indicating that current limiting is active
    bool voltage     : 1;       // indicating that voltage limiting is active
    bool temperature : 1;       // indicating that temperature limiting is active
    bool dutycycle   : 1;       // indicating that duty cycle limit is active
} light_limiterActive_t;

typedef struct
{
    light_limiterActive_t mainBeam; // status of flood driver
    light_limiterActive_t highBeam; // status of spot driver
    q6_10_t currentMainBeam;        // main beam current in A
    q6_10_t currentHighBeam;        // high beam current in A
    q12_4_t temperature;            // temperature K
    q6_10_t inputVoltage;           // input voltage in V
} light_status_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief initialization function
 *
 * @param[in]   cellCnt     cell count of supply battery
 * @param[out]  pLedConfig  last known LED configuration
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INTERNAL if initialization failed
 */
uint32_t light_Init(uint8_t cellCnt, light_driverConfig_t * pLedConfig);

/** @brief function to enable / disable light functionality
 *
 * @param[in]   enable  true to enable, false to disable
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INVALID_STATE
 *              NRF_ERROR_INTERNAL
 *
 * @note        when light is disabled, no valid data is return by
 *              @ref light_Execute
 */
uint32_t light_Enable(bool enable);

/** @brief target update function
 *
 * @param[in]   pMode       requested light mode
 * @param[out]  ppStatus    actual status of light
 * @return      NRF_SUCCESS
 *              NRF_ERROR_NULL
 *              NRF_ERROR_INVALID_STATE if module is not enabled
 *              NRF_ERROR_INTERNAL
 */
uint32_t light_UpdateTargets(light_mode_t const* pMode, light_status_t const** ppStatus);

/** @brief execute function
 *
 * @note        call this function in the main loop!
 */
void light_Execute(void);

/** @brief Function to check the led configuration
 *
 * @param[out]  pLedConfig configuration of leds
 * @return      NRF_SUCCESS or an error code
 *
 * @note        The configuration will be stored in flash and returned at
 *              future calls of @ref light_Init
 */
uint32_t light_CheckLedConfig(light_driverConfig_t* pLedConfig);

/** @brief Function to get the current limits
 *
 * @param[out]  pMainBeamLimit  current main beam limit
 * @param[out]  pHighBeamLimit  current high beam limit
 * @return      NRF_SUCCESS
 *              NRF_ERROR_NULL
 */
uint32_t light_GetLimits(q8_t* pMainBeamLimit, q8_t* pHighBeamLimit);

/** @brief Function to set the current limits for the leds
 *
 * @param[in]   mainBeamLimit   limit for main beam led
 * @param[in]   highBeamLimit   limit for high beam led
 * @return      NRF_SUCCESS
 *
 * @note        The limits will be stored in flash
 */
uint32_t light_SetLimits(q8_t mainBeamLimit, q8_t highBeamLimit);

#endif /*_LIGHT_H_*/

/**END OF FILE*****************************************************************/
