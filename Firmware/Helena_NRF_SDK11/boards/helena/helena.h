/**
  ******************************************************************************
  * @file    helena.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HLN_H_INCLUDED
#define HLN_10_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "board.h"
//#include "pitch_reg.h"
#include "helena_base_driver.h"
#include "ble_KD2.h"
#include "pitch_reg.h"

/* Exported types ------------------------------------------------------------*/
typedef uint16_t q8_8_t;                // fix point integer used for internal target values
typedef uint16_t q6_10_t;               // fix point integer used for power

/// TODO: rename
typedef enum
{
    COMPIN_MODE_NOTUSED = BLE_KD2_COM_PIN_NOT_USED,
    COMPIN_MODE_COM = BLE_KD2_COM_PIN_COM,
    COMPIN_MODE_BUTTON = BLE_KD2_COM_PIN_BUTTON
} hln_comPinMode_t;

typedef struct
{
    q6_10_t outputPower;
    q8_8_t outputLimit;
    prg_optic_t optic;
} hln_channelSetup_t;

/* Exported constants --------------------------------------------------------*/
#define HLN_OUTPUT_POWER_MAX    (27 << 10)

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to retrieve a channel setup
 *
 * @param[out] pSetup   structure to store the setup
 * @param[in]  channel  the desired channel
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_PARAM for invalid channel
 */
ret_code_t hln_GetChannelSetup(hln_channelSetup_t* pSetup, uint8_t channel);

/** @brief function to set a channel setup
 *
 * @param[in]  pSetup   the new setup
 * @param[in]  channel  the desired channel
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INVALID_PARAM for invalid channel
 */
ret_code_t hln_SetChannelSetup(hln_channelSetup_t const* pSetup, uint8_t channel, ds_reportHandler_t resultHandler);

/** @brief function to get the current com pin usage mode
 *
 * @param[out] pMode
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t hln_GetComPinMode(hln_comPinMode_t* pMode);

/** @brief function to set the com pin usage
 *
 * @param[in] mode
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t hln_SetComPinMode(hln_comPinMode_t mode, ds_reportHandler_t resultHandler);

/** @brief function to get the compensation values for the helena driver board
 *
 * @param[out] pComp
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t hln_GetCompensation(hbd_calibData_t* pComp, uint8_t driver);

/** @brief function to set the compensation values for the helena driver board
 *
 * @param[out] pComp
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_FOUND, NRF_ERROR_NOT_SUPPORTED, NRF_ERROR_INVALID_PARAM
 */
 /// TODO: this function uses I2C and should be scheduled in main context
ret_code_t hln_SetCompensation(hbd_calibData_t* pComp, uint8_t driver, ds_reportHandler_t resultHandler);

/** @brief function to check if imu is already calibrated
 *
 * @param[out]  true if calibrated, false if not
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_SUPPORTED
 */
ret_code_t hln_IsImuCalibrated(bool* pIsCalibrated);

/** @brief function to initiate the imu calibration
 *
 * @param[in] resultHandler  the storage result handler
 * @return NRF_SUCCESS
 */
ret_code_t hln_CalibrateImu(ds_reportHandler_t resultHandler);

#endif // KD2_10_H_INCLUDED

/**END OF FILE*****************************************************************/

