/**
  ******************************************************************************
  * @file    board.h
  * @author  Thomas Reisnecker
  * @brief   Header for board module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BOARD_H_INCLUDED
#define BOARD_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
//#include "softdevice_handler.h"
#include "sdk_errors.h"
#include "hmi.h"
#include "channel_types.h"
#include "btle.h"
#include "data_storage.h"
#include "board_config.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t rx;                        // pin used for receiving
    uint32_t tx;                        // pin used for transmitting
} brd_comPins_t;

typedef struct
{
    uint16_t                   channelCount;  // number of channels
    ble_hps_f_ch_size_t const* pChannelTypes; // types of channels (pointer to array with channelCount members)
    brd_comPins_t const*       pComSupported; // the com pins, NULL or COM_PIN_NOT_USED if not available
} brd_features_t;

typedef struct
{
    brd_features_t const*       pFeatures;
    btle_info_t const*          pInfo;
    ble_hps_modes_init_t const* pModes;
    ble_hps_qwr_buffer_t      * pBuf;       // the buffer for queued writes, can be used for other things, too
} brd_info_t;

typedef enum
{
    BRD_PM_OFF = 0, // lowest possible power mode, there will be no wakeup from this mode
    BRD_PM_STANDBY, // "normal" low power mode, light output should be off
    BRD_PM_IDLE,    // operational mode, send status messages to ble at about 1Hz
} brd_powerMode_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the board
 *
 * @param void
 * @return ret_code_t
 *
 */
ret_code_t brd_Init(brd_info_t const* *pInfo);

/** @brief function to set the desired power mode
 *
 * @param[in] newMode  the new mode.
 */
ret_code_t brd_SetPowerMode(brd_powerMode_t newMode);

/** @brief function to set the desired light mode
 *
 * @param[in] newMode  the new mode.
 */
ret_code_t brd_SetLightMode(uint8_t newMode);

/** @brief function to temporary override the channel configuration for the current mode
 *
 * @param[in] pConfig ble_hps_cp_channel_config_t const*
 */
ret_code_t brd_OverrideMode(ble_hps_cp_channel_config_t const* pConfig);

/** @brief execute function that will be called in the main loop
 *
 * @param[in] currentMode   the current mode, in standby and off mode ignore this
 *                          mode and use MM_MODE_OFF
 * @return true if an output is limited, otherwise false
 */
bool brd_Execute(void);

/** @brief function to en-, or disable the status leds
 *
 * @param[in] color  the led type
 * @param     enable
 */
void brd_EnableLed(hmi_ledType_t color, bool enable);

/** @brief function to get the current channel configuration
 *
 * @param[out] ppData   the pointer to the arrays of channel configuration
 * @param[out] pSize    the size of the array
 * @return NRF_SUCCESS, NRF_ERROR_NULL
 */
ret_code_t brd_GetChannelConfig(void const** ppData, uint16_t* pSize);

/** @brief function to set a new channel config
 *
 * @param[in] pData void const* the pointer to the arrays of channel configuration
 * @param[in] size uint16_t     the size of the arrays
 * @param[in] resultHandler     the handler to use to report the result
 * @return    NRF_SUCCESS, NRF_ERROR_INVALID_LENGTH, NRF_ERROR_INVALID_PARAM or an propagated error code *
 */
ret_code_t brd_SetChannelConfig(void const* pData, uint16_t size, ds_reportHandler_t resultHandler);

/** @brief function to set a new device name, if device name changing is not supported just return NRF_SUCCESS
 *
 * @param[in] pNewName char const* the new device name
 * @param[in] resultHandler ds_reportHandler_t
 * @return ret_code_t       the handler to use to report the result
 * @return    NRF_SUCCESS, NRF_ERROR_INVALID_LENGTH or an propagated error code *
 */
ret_code_t brd_SetDeviceName(char const* pNewName, ds_reportHandler_t resultHandler);

/** @brief function to initiate the procedure to delete all internal settings
 *
 * @param[in] resultHandler the handler to call with the result
 * @return NRF_SUCCESS or a propagated error
 */
ret_code_t brd_FactoryReset(ds_reportHandler_t resultHandler);

#endif // BOARD_H_INCLUDED

/**END OF FILE*****************************************************************/
