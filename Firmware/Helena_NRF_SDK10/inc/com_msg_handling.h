/**
  ******************************************************************************
  * @file    com_msg_handling.h
  * @author  Thomas Reisnecker
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
} cmh_lightMode_t;

typedef struct
{
    uint8_t overcurrentError : 1;
    uint8_t voltageError : 1;
    uint8_t temperatureError : 1;
    uint8_t directdriveError : 1;
    cmh_lightMode_t mode;
    uint8_t cedCnt;
    uint16_t current;               // current in mA
    uint16_t temperature;           // temperature in 0.1K
    uint16_t voltage;               // voltage in mV
} cmh_light_t;

typedef struct
{
    uint8_t overcurrentError : 1;
    uint8_t voltageError : 1;
    uint8_t temperatureError : 1;
    uint8_t directdriveError : 1;
    cmh_lightMode_t mode;
    uint8_t ledCnt;
    uint16_t current;               // current in mA
    uint16_t temperature;           // temperature in 0.1K
    uint16_t voltage;               // voltage in mV
} cmh_taillight_t;

typedef enum
{
    cmh_BCT_LIION = 0,
    cmh_BCT_LIFEPO = 1
} cmh_batteryCellTec_t;

typedef struct
{
    uint8_t currentWarning : 1;
    uint8_t voltageWarning : 1;
    uint8_t temperatureWarning : 1;
    uint8_t balanceActive : 1;
    uint8_t cellCnt;
    cmh_batteryCellTec_t cellTec;
    cmh_lightMode_t mode;
    int16_t current;                // current in mA
    uint16_t voltage;               // voltage in mV
    uint16_t temperature;           // temperature in 0.1K
    uint16_t fullChargeCap;         // full charge capacity in mAh
    uint16_t soc;                   // state of charge in 0.1%
} cmh_battery_t;

typedef struct
{
    cmh_lightMode_t mainBeam;
    cmh_lightMode_t highBeam;
    cmh_lightMode_t helmetBeam;
    cmh_lightMode_t taillight;
} cmh_lightMasterData_t;

typedef void (*cmh_LightMasterHandler_t)(const cmh_lightMasterData_t * pMasterData);

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief initialization of com message handling
 *
 * @param[in]   pLightMasterHandler handler for master messages
 * @return      error code of application timer module
 */
uint32_t cmh_Init(cmh_LightMasterHandler_t lightMasterHandler);

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

#ifdef BILLY
/** @brief function to update main beam data
 *
 * @param[in]   pLight  main beam data
 * @return      NRF_SUCCESS
 */
uint32_t cmh_UpdateMainBeam(cmh_light_t* pLight);

/** @brief function to update high beam data
 *
 * @param[in]   pLight  main beam data
 * @return      NRF_SUCCESS
 */
uint32_t cmh_UpdateHighBeam(cmh_light_t* pLight);

#elif defined HELENA

/** @brief function to update helmet light data
 *
 * @param[in]   pLight  helmet light data
 * @return      NRF_SUCCESS
 */
uint32_t cmh_UpdateHelmetLight(cmh_light_t* pLight);
#endif // HELENA

/** @brief function to enable com based taillight
 *
 * @param[in]   enable  true to enable, false to disable
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INVALID_STATE if an active light master is available
 */
uint32_t cmh_EnableTaillight(bool enable);

/** @brief function to update brake indicator
 *
 * @param[in]   braking  true if brake condition met, otherwise false
 * @return      NRF_SUCCESS
 */
uint32_t cmh_UpdateBrakeIndicator(bool braking);

/** @brief function to retrieve the last known taillight data
 *
 * @param[out] pTaillight the last known taillight data
 * @return     NRF_SUCCESS
 *             NRF_ERROR_NULL
 *             NRF_ERROR_NOT_FOUND if data is unavailable
 */
uint32_t cmh_GetTaillight(cmh_taillight_t* pTaillight);

/** @brief function to retrieve the last known battery data
 *
 * @param[out] pBattery the last known battery data
 * @return     NRF_SUCCESS
 *             NRF_ERROR_NULL
 *             NRF_ERROR_NOT_FOUND if data is unavailable
 */
uint32_t cmh_GetBattery(cmh_battery_t* pBattery);

#endif /* COM_MSG_HANDLING_H_INCLUDED */

/**END OF FILE*****************************************************************/
