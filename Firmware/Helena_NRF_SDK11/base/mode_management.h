/**
  ******************************************************************************
  * @file    mode_management.h
  * @author  Thomas Reisnecker
  * @brief   Header for mode management module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODE_MANAGEMENT_H_INCLUDED
#define MODE_MANAGEMENT_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "sdk_errors.h"

#include "hmi.h"
#include "data_storage.h"
#include "ble_hps.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{                           // true if,
    bool ignore        :  1;// mode is ignored and can only be activated by setting mode directly
    bool firstInGroup  :  1;// this is the first mode in this group
    bool lastInGroup   :  1;// this is the last mode in this group
    bool isPrefMode    :  1;// this is the preferred mode
    bool isTempMode    :  1;// this is the temporary mode
    bool isOffMode     :  1;// this is the off mode
    uint16_t dummy     : 10;// reserved for future use
} mm_modeConfig_t;

typedef void (*mm_modeChangedHandler_t)(uint8_t newMode);

typedef struct
{
    mm_modeConfig_t*        pModesMem;  // pointer to the memory for the modes in the gatt table
    ble_hps_qwr_buffer_t*   pBuf;       // the queued write buffer which will be used temporally for storing new received modes
    //uint16_t                memSize;    // the available size for the modes
    mm_modeChangedHandler_t modeHandler;// the handler called when a mode is changed
} mm_init_t;

/* Exported constants --------------------------------------------------------*/
#define MM_NUM_OF_MODES     8               // temporary lcs functions only work with 8!
#define MM_MODE_OFF         255
//#define MM_MODE_SOS         254           // not implemented yet

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the mode management
 *
 * @param[in]  pInit      the initialization structure
 * @param[out] pInitMode  the initial mode
 * @return NRF_SUCCESS
 *         NRF_ERROR_NULL
 */
ret_code_t mm_Init(mm_init_t const* pInit, uint8_t* pInitMode);

/** @brief execute function for the mode management module, to be called in the main loop
 */
void mm_Execute(void);

/** @brief function to enable the mode management module
 *
 * @param[in] enable
 * @return tbd.
 */
//ret_code_t mm_Enable(bool enable);

/** @brief the hmi event handler
 */
ret_code_t mm_HmiEventHandler(hmi_evt_t const* pEvent);

/** @brief function to get the current mode
 *
 * @return the current mode, MM_MODE_OFF in off mode, use mm_GetOffMOde() to
 *         retrieve the actual off mode
 */
uint8_t mm_GetCurrentMode(void);

/** @brief function to get the off mode
 *
 * @return the off mode, MM_MODE_OFF if not used
 */
uint8_t mm_GetOffMode(void);

/** @brief function to get the configuration of a specific mode
 *
 * @param[in] mode the desired mode
 * @return the configuration of the mode, for MM_MODE_OFF either
 *         NULL or the settings of the off mode
 */
//mm_modeConfig_t const* mm_GetModeConfig(uint8_t mode);

ret_code_t mm_GetModeConfigs(mm_modeConfig_t const** ppModeConfigs);

ret_code_t mm_CheckModeConfig(mm_modeConfig_t const* pModeConfig, uint16_t size);

ret_code_t mm_SetModeConfig(mm_modeConfig_t const* pModeConfig, uint16_t size, ds_reportHandler_t resultHandler);

/** @brief function to initiate the procedure to delete all internal settings
 *
 * @param[in] resultHandler the handler to call with the result
 * @return NRF_SUCCESS or a propagated error
 */
ret_code_t mm_FactoryReset(ds_reportHandler_t resultHandler);

/**< temporary functions for lcs */

/*ret_code_t mm_UpdateGroupConfig(ble_lcs_ctrlpt_group_cnfg_t const* pLcs, ds_reportHandler_t resultHandler);

ret_code_t mm_UpdateModeConfig(ble_lcs_ctrlpt_mode_cnfg_t const* pLcs, ds_reportHandler_t resultHandler);

uint8_t mm_GetPrefMode(void);

ret_code_t mm_SetPrefMode(uint8_t mode, ds_reportHandler_t resultHandler);

uint8_t mm_GetTempMode(void);

ret_code_t mm_SetTempMode(uint8_t mode, ds_reportHandler_t resultHandler);*/

#endif // MODE_MANAGEMENT_H_INCLUDED

/**END OF FILE*****************************************************************/
