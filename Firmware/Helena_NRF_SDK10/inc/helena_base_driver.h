/**
  ******************************************************************************
  * @file    helean_base_driver.h
  * @author  Thomas Reisnecker
  * @brief   driver module for helena base i2c driver unit
  *
  * @note    The following functions must be provided
  *          hbd_retVal_t hbd_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t const* pData)
  *          hbd_retVal_t hbd_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t pBuffer)
  *          Additionally hbd_retVal_t has to be defined. This driver only
  *          relayes error codes from the i2c read and write functions.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HELENA_BASE_DRIVER_H_INCLUDED
#define HELENA_BASE_DRIVER_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* platform depending definitions --------------------------------------------*/
#include "i2c.h"
#include "nrf_error.h"

#define HBD_SUCCESS             NRF_SUCCESS
#define HBD_ERROR_NULL          NRF_ERROR_NULL
#define HBD_ERROR_NOT_SUPPORTED NRF_ERROR_NOT_SUPPORTED

typedef uint32_t hbd_retVal_t;

static inline hbd_retVal_t hbd_i2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t const* pData)
{
    return i2c_write(addr, reg, len, pData);
}

static inline hbd_retVal_t hbd_i2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* pBuffer)
{
    return i2c_read(addr, reg, len, pBuffer);
}

/* Exported defines ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef uint8_t q8_t;
typedef uint8_t q1_7_t;
typedef int16_t q13_2_t;
typedef uint16_t q6_10_t;
typedef uint16_t q12_4_t;

/** @brief helena base sleep mode settings
 */
typedef enum
{
    HBD_SLEEP_MODE_OFF = 0,
    HBD_SLEEP_MODE_ON
} hbd_sleepMode_t;

/** @brief helena base sample rate settings
 * @note These setting only have effects when the sleep mode is disabled and
 *       both drivers are set to 0. If at least one driver is active the
 *       sampling of output current and temperature is done at maximum rate
 */
typedef enum
{
    HBD_SAMPLERATE_64SPS = 0,
    HBD_SAMPLERATE_32SPS,
    HBD_SAMPLERATE_16SPS,
    HBD_SAMPLERATE_8SPS,
    HBD_SAMPLERATE_4SPS,
    HBD_SAMPLERATE_2SPS,
    HBD_SAMPLERATE_1SPS,        // default value
    HBD_SAMPLERATE_SPS5,        // 0.5 SPS
    HBD_SAMPLERATE_SPS25,        // 0.25 SPS
    HBD_SAMPLERATE_SPS125       // 0.125 SPS
} hbd_sampleRate_t;

/** @brief helena base configuration structure
*/
typedef struct
{
    hbd_sleepMode_t sleepMode;
    hbd_sampleRate_t sampleRate;
} hbd_config_t;

typedef struct
{
    bool maxDC : 1;             // set if the maximum DC is reached
    bool minDC : 1;             // set if the minimum DC is reached
    q6_10_t current;            // the output current in A, if correctly calibrated
} hbd_current_t;

typedef struct
{
    hbd_current_t currentLeft;
    hbd_current_t currentRight;
    q12_4_t temperature;        // temperature in K
} hbd_samplingData_t;

typedef enum
{
    HBD_FWREV_10 = 0,
    HBD_FWREV_11,
    HBD_FWREV_12,
    HBD_FWREV_UNKOWN
} hbd_firmwareRev_t;

typedef struct
{
    q13_2_t temperatureOffset;
    q1_7_t  gainLeft;
    q1_7_t  gainRight;
} hbd_calibData_t;

/* Exported constants --------------------------------------------------------*/
#define HBD_DC_MAX               254

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/// TODO: init to get the register values at startup

/** @brief function to configure the sleep mode and the sample rate
 *
 * @param[in] pConfig the configuration to set
 * @return    HBD_ERROR_NULL or the return value of hbd_i2c_write
 */
hbd_retVal_t hbd_SetConfig(hbd_config_t const* pConfig);

/** @brief function to read the current sleep mode and the sample rate
 *
 * @param[in] pConfig the configuration to set
 * @return    HBD_SUCCESS, HBD_ERROR_NULL or the return value of hbd_i2c_read
 */
hbd_retVal_t hbd_GetConfig(hbd_config_t* pConfig);

/** @brief function to set the duty cycles
 *
 * @param[in] left  the target current for left driver (0..255 -> 0..100%)
 * @param[in] right the target Current for right driver (0..255 -> 0..100%)
 * @return    HBD_SUCCESS or the return value of hbd_i2c_write
 */
hbd_retVal_t hbd_SetTargetCurrent(q8_t left, q8_t right);

/** @brief function to set the duty cycles
 *
 * @param[out] pLeft  the target current for left driver (0..255 -> 0..100%)
 * @param[out] pRight the target Current for right driver (0..255 -> 0..100%)
 * @return    HBD_SUCCESS, HBD_ERROR_NULL or the return value of hbd_i2c_read
 */
hbd_retVal_t hbd_GetTargetCurrent(q8_t* pLeft, q8_t* pRight);

/** @brief function to get the current sampling data
 *
 * @param[out] pSamplingData    a pointer to the structure, the data will be stored
 * @return     HBD_ERROR_NULL or the return value of hbd_i2c_read;
 */
hbd_retVal_t hbd_ReadSamplingData(hbd_samplingData_t* pSamplingData);

/** @brief function to check the firmware revision of the driver
 *
 * @param[out] pFWRef   the detected firmware revision
 * @return     the return value of hbd_i2c_read;
 */
hbd_retVal_t hbd_GetFirmwareRev(hbd_firmwareRev_t* pFWRef);

/** @brief function to set the duty cycles
 *
 * @param[out] left  the duty cycle for left driver (0..255 -> 0..100%)
 * @param[out] right the duty cycle for right driver (0..255 -> 0..100%)
 * @return    the return value of hbd_i2c_write
 */
hbd_retVal_t hbd_ReadDutyCycles(q8_t* left, q8_t* right);

/** @brief function to read the calibration data
 *
 * @param[out] pCalibData   the received calibration data
 * @return     the return value of hbd_i2c_read;
 */
hbd_retVal_t hbd_GetCalibrationData_t(hbd_calibData_t* pCalibData);

/** @brief function to write new calibration data
 *
 * @param[in] pCalibData    the new calibration data
 * @return    the return value of hbd_i2c_write;
 */
hbd_retVal_t hbd_SetCalibrationData_t(hbd_calibData_t const* pCalibData);

#endif // HELENA_BASE_DRIVER_H_INCLUDED

/**END OF FILE*****************************************************************/
