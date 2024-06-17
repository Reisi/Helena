/**
  ******************************************************************************
  * @file    limiter.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LIMITER_H_INCLUDED
#define LIMITER_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"

#include "filter.h"

/* Exported types ------------------------------------------------------------*/
typedef int32_t q15_16_t;

typedef enum
{
    LIM_PRIO_HIGH = 0,
    LIM_PRIO_MID = 1,
    LIM_PRIO_LOW = 2,
    LIM_PRIO_CNT = 3
} lim_priority_t;

typedef struct
{
    q15_16_t       fullPower;   // the channels absolute power at 100%
    lim_priority_t priority;    // the priority of this channel
} lim_channelDef_t;

typedef struct
{
    q15_16_t       request;     // the channel relative request
    bool           limiting;    // true if the channel was limited
} lim_channelReq_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to calculate the cell count of a liion battery
 *
 * @note due to the overlapping ranges of full and empty batteries with more
 *       than 3 cells, this automatic detection only works with 1, 2 and 3 cells
 * @param[in] voltage  the battery voltage
 * @return 1, 2, 3 or 0 if automatic detection has failed
 */
uint8_t lim_CalcLiionCellCnt(q15_16_t voltage);

/** @brief function to calculate the relative output power for a given battery
 *         voltage
 *
 * @param[in] voltage  the battery voltage
 * @param[in] cellCnt  the battery pack cell cnt
 * @return    the relative available output power (0 .. 65536 -> 0 .. 1)
 */
q15_16_t lim_CalcLiionVoltageLimit(q15_16_t voltage, uint8_t cellCnt);

/** @brief function to calculate the relative output power for a given
 *         temperature
 *
 * @param[in] temperature  the temperature (in K)
 * @return    the relative available output power (0 .. 65536 -> 0 .. 1)
 */
q15_16_t lim_CalcTemperatureLimit(q15_16_t temperature);

/** @brief function to calculate the channel limits
 *
 * @param[in]     powerAvailable  the available absolute power
 * @param[in]     pChDef          the channel definitions
 * @param[in/out] pChReq          the channel requests
 * @param[in]     chCnt           the channel count
 * @return NRF_SUCCESS, NRF_ERROR_NULL, tbd.
 */
ret_code_t lim_LimitChannels(q15_16_t powerAvailable, lim_channelDef_t const* pChDef,
                             lim_channelReq_t* pChReq, uint8_t chCnt);




/** @brief function to initialize the limiter
 *
 * @param[in]     inputVoltage the initial input voltage
 * @param[in]     temperature  the initial temperature (in K)
 * @param[in/out] pChannels    the channel definitions
 * @param[in]     cnt          the number of channels
 * @return NRF_SUCCESS, NRF_ERROR_INVALID_PARAM
 */
//ret_code_t lim_Init(lim_instance_t* pInst, q15_16_t inputVoltage, q15_16_t temperature);

/** @brief function to feed the limiter with the current input voltage and
 *         temperature
 *
 * @note /// TODO: filter evaluation and recommendations when to call this function
 *
 * @param inputVoltage
 * @param temperature
 */
//void lim_Feed(lim_instance_t* pInst, q15_16_t inputVoltage, q15_16_t temperature, bool idle);

/** @brief function to calculate channel limits
 *
 * @param[in/out] pRequests the relative power requests for each channel
 * @return NRF_SUCCESS, tbd.
 */
//ret_code_t lim_Calc(lim_instance_t* pInst, lim_request_t* pRequests);

#endif // LIMITER_H_INCLUDED

/**END OF FILE*****************************************************************/

