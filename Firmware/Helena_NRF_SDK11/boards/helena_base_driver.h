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

#include "sdk_errors.h"

/* Configuration -------------------------------------------------------------*/
//#define HDB_FULL_MIRROR // if defined all register are are kept in ram, if not just config and target values

/* Exported defines ----------------------------------------------------------*/
#define HBD_DC_MAX      254
#ifdef HDB_FULL_MIRROR
#define HBD_MIRRORCNT 16
#else
#define HBD_MIRRORCNT 3
#endif // HDB_FULL_MIRROR

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    HBD_SUCCESS             = NRF_SUCCESS,
    HBD_ERROR_COM           = NRF_ERROR_INTERNAL,
    HBD_ERROR_NOT_FOUND     = NRF_ERROR_NOT_FOUND,
    HBD_ERROR_NOT_SUPPORTED = NRF_ERROR_NOT_SUPPORTED,
    HBD_ERROR_INVALID_PARAM = NRF_ERROR_INVALID_PARAM,
    HBD_ERROR_INVALID_STATE = NRF_ERROR_INVALID_STATE,
    HBD_ERROR_NULL          = NRF_ERROR_NULL,
    HBD_ERROR_FORBIDDEN     = NRF_ERROR_FORBIDDEN,
} hbd_retVal_t;

typedef uint8_t q8_t;
typedef uint8_t q1_7_t;
typedef int16_t q13_2_t;
typedef uint16_t q5_11_t;
typedef uint16_t q6_10_t;
typedef uint16_t q12_4_t;

typedef int8_t (*hbd_read_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

typedef int8_t (*hbd_write_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t const *data, uint16_t len);

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
    HDB_FWREV_BILLY,
    HBD_FWREV_UNKOWN
} hbd_firmwareRev_t;

typedef struct
{
    q13_2_t temperatureOffset;
    q1_7_t  gainLeft;
    q1_7_t  gainRight;
} hbd_calibData_t;

typedef struct
{
    uint8_t address;
    hbd_read_t i2cRead;
    hbd_write_t i2cWrite;
} hbd_init_t;

typedef struct
{
    uint8_t address;
    hbd_firmwareRev_t fwRev;
    uint8_t regMirror[HBD_MIRRORCNT];
} hbd_inst_t;

/* Exported functions ------------------------------------------------------- */
/** @brief initialization function
 *
 * @param[in]     pInit  the read and write functions
 * @param[out]    pInst  array of available driver instances
 * @param[out]    pInst  array of sampling data for initial values, NULL if not needed
 * @param[in/out] pCnt   in: available instance structures, out: the number of
 *                       found drivers
 * @param[in]     reset  true if driver should be reset to default setup, false if not
 * @return        HBD_SUCCESS, HBD_ERROR_NULL or HBD_ERROR_NOT_FOUND
 */
hbd_retVal_t hbd_Init(hbd_init_t const* pInit, hbd_inst_t* pInst, hbd_samplingData_t* pData, uint8_t* pCnt, bool reset);


/** @brief function to configure the sleep mode and the sample rate
 *
 * @param[in] pInst   the driver instance
 * @param[in] pConfig the configuration to set
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL or HBD_ERROR_COM
 */
hbd_retVal_t hbd_SetConfig(hbd_inst_t* pInst, hbd_config_t const* pConfig);

/** @brief function to read the current sleep mode and the sample rate
 *
 * @param[in]  pInst   the driver instance
 * @param[out] pConfig the configuration to get
 * @return     HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL or HBD_ERROR_COM
 */
hbd_retVal_t hbd_GetConfig(hbd_inst_t const* pInst, hbd_config_t* pConfig);

/** @brief function to set the duty cycles
 *
 * @param[in] pInst    the driver instance
 * @param[in] voltage  the input voltage
 * @param[in] left     the target current for left driver (0..255 -> 0..100%)
 * @param[in] right    the target Current for right driver (0..255 -> 0..100%)
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL, HBD_ERROR_FORBIDDEN or HBD_ERROR_COM
 */
hbd_retVal_t hbd_SetTargetCurrent(hbd_inst_t* pInst, q5_11_t inputVoltage, q8_t left, q8_t right);

/** @brief function to set the duty cycles
 *
 * @param[in]  pInst   the driver instance
 * @param[out] pLeft  the target current for left driver (0..255 -> 0..100%)
 * @param[out] pRight the target Current for right driver (0..255 -> 0..100%)
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL or HBD_ERROR_COM
 */
hbd_retVal_t hbd_GetTargetCurrent(hbd_inst_t const* pInst, q8_t* pLeft, q8_t* pRight);

/** @brief function to get the current sampling data
 *
 * @param[in]  pInst   the driver instance
 * @param[out] pSamplingData    a pointer to the structure, the data will be stored
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL or HBD_ERROR_COM
 */
hbd_retVal_t hbd_ReadSamplingData(hbd_inst_t* pInst, hbd_samplingData_t* pSamplingData);

/** @brief function to check the firmware revision of the driver
 *
 * @param[in]  pInst   the driver instance
 * @param[out] pFWRef   the detected firmware revision
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL or HBD_ERROR_COM
 */
hbd_retVal_t hbd_GetFirmwareRev(hbd_inst_t const* pInst, hbd_firmwareRev_t* pFWRef);

/** @brief function to set the duty cycles
 *
 * @param[in]  pInst   the driver instance
 * @param[out] left  the duty cycle for left driver (0..255 -> 0..100%)
 * @param[out] right the duty cycle for right driver (0..255 -> 0..100%)
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL,
 *            HBD_ERROR_NOT_SUPPORTED or HBD_ERROR_COM
 */
hbd_retVal_t hbd_ReadDutyCycles(hbd_inst_t* pInst, q8_t* pLeft, q8_t* pRight);

/** @brief function to read the calibration data
 *
 * @param[in]  pInst   the driver instance
 * @param[out] pCalibData   the received calibration data
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL,
 *            HBD_ERROR_NOT_SUPPORTED or HBD_ERROR_COM
 */
hbd_retVal_t hbd_GetCalibrationData(hbd_inst_t const* pInst, hbd_calibData_t* pCalibData);

/** @brief function to write new calibration data
 *
 * @param[in] pInst   the driver instance
 * @param[in] pCalibData    the new calibration data
 * @return    HBD_SUCCESS, HBD_INVALID_STATE, HBD_ERROR_NULL,
 *            HBD_ERROR_NOT_SUPPORTED, HBD_ERROR_INVALID_PARAM or HBD_ERROR_COM
 */
hbd_retVal_t hbd_SetCalibrationData(hbd_inst_t* pInst, hbd_calibData_t const* pCalibData);

#endif // HELENA_BASE_DRIVER_H_INCLUDED

/**END OF FILE*****************************************************************/
