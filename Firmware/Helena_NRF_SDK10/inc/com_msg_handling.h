/**
  ******************************************************************************
  * @file    com_msg_handling.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/06
  * @brief   header file for communication messages handling
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COM_MSG_HANDLING_H_INCLUDED
#define COM_MSG_HANDLING_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "comreloaded.h"

/* Exported types ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    cmh_LIGHTOFF = 0,
    cmh_LIGHTDIM,
    cmh_LIGHTLOW,
    cmh_LIGHTFULL
} cmh_LightModeEnum;

typedef struct
{
    uint8_t overcurrentError : 1;
    uint8_t voltageError : 1;
    uint8_t temperatureError : 1;
    uint8_t directdriveError : 1;
    cmh_LightModeEnum mode;
    uint16_t current;               /**< current in mA */
    uint16_t temperature;           /**< temperature in 0.1K */
    uint16_t voltage;               /**< voltage in mV */
} cmh_LightStruct;

typedef cmh_LightStruct cmh_HelmetLightStruct;

typedef struct
{
    cmh_LightModeEnum mainBeam;
    cmh_LightModeEnum highBeam;
    cmh_LightModeEnum helmetBeam;
    cmh_LightModeEnum taillight;
} cmh_LightMasterDataStruct;

typedef void (*cmh_LightMasterHandler)(const cmh_LightMasterDataStruct * pMasterData);

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief initialization of com message handling
 *
 * @param[in]   pLightMasterHandler handler for master messages
 * @return      error code of application timer module
 */
uint32_t cmh_Init(cmh_LightMasterHandler pLightMasterHandler);

/** @brief execution handler
 *
 * @note    call this handler in the main loop
 */
void cmh_Execute(void);

/** @brief message handler function
 *
 * @param[in]   pMessageIn  received message
 *
 * @note    call this function on incoming messages
 */
void cmh_ComMessageCheck(const com_MessageStruct * pMessageIn);

/** @brief function to update helmet light data
 *
 * @param[in]   pLight  helmet light data
 * @return      NRF_SUCCESS
 */
uint32_t cmh_UpdateHelmetLight(cmh_HelmetLightStruct* pLight);

/** @brief function to update brake indicator
 *
 * @param[in]   braking  true if brake condition met, otherwise false
 * @return      NRF_SUCCESS
 */
uint32_t cmh_UpdateBrakeIndicator(bool braking);

#endif /* COM_MSG_HANDLING_H_INCLUDED */

/**END OF FILE*****************************************************************/
