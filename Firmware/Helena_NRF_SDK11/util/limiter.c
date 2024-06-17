/**
  ******************************************************************************
  * @file    limiter.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "limiter.h"
#include "filter.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define CELL_VOLT_MAX   (9 << 15)   // 4.5V (4.5 << 16)
#define CELL_VOLT_MIN   (3 << 16)   // 3.0V
#define CELL_CNT_MAX    3           // max. 3 cells can be detected, because 3
                                    // and 4 cell will have overlapping limits

#define TEMP_MAX        ((65 + 273) << 16)   // fully limited
#define TEMP_MIN        ((49 + 273) << 16)   // limiting start

#define VOLT_MAX        ((3400l << 16) / 1000) // limiting start
#define VOLT_MID        ((3275l << 16) / 1000) // limiting at 50%
#define VOLT_MIN        ((3025l << 16) / 1000) // fully limited

#define UNLIMITED       (1l << 16)


/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/** @brief function to calculate the requested power for a priority
 *
 * @param[in] prio    the priority to calculate
 * @param[in] pChDef  the channel definitions
 * @param[in] pChReq  the channel requests
 * @param[in] chCnt   the number of channels
 * @return    the requested power for this priority
 */
static q15_16_t calcPrioPower(uint8_t prio, lim_channelDef_t const* pChDef, lim_channelReq_t const* pChReq, uint8_t chCnt)
{
    static q15_16_t power = 0;
    power = 0;
    for (uint_fast8_t i = 0; i <chCnt; i++)
    {
        if (pChDef[i].priority == prio)
            power += ((int64_t)pChDef[i].fullPower * pChReq[i].request) >> 16;
    }
    return power;
}

/** @brief function to limit the request for a given priority
 *
 * @param[in]     powerAvailable  the available power
 * @param[in]     prio            the priority to limit
 * @param[in]     pChDef          the channel definitions
 * @param[in/out] pChReq          the channel requests
 * @param[in]     chCnt           the number of channels
 * @return        the remaining power after limiting
 */
static q15_16_t limitPrio(q15_16_t powerAvailable, uint8_t prio,
                          lim_channelDef_t const* pChDef, lim_channelReq_t* pChReq, uint8_t chCnt)
{
    q15_16_t prioPower, requestReduction;

    // calculate the requested power for this priority
    prioPower = calcPrioPower(prio, pChDef, pChReq, chCnt);

    // if requested power is lower than available, there is nothing to limit
    if (prioPower < powerAvailable)
    {
        for (uint_fast8_t i = 0; i < chCnt; i++)
        {
            if (pChDef[i].priority == prio)
                pChReq[i].limiting = false;
        }
        return powerAvailable - prioPower;
    }

    // otherwise reduce requested power
    requestReduction = (int64_t)powerAvailable * UNLIMITED / prioPower;
    for (uint_fast8_t i = 0; i < chCnt; i++)
    {
        if (pChDef[i].priority != prio)
            continue;

        pChReq[i].request = ((int64_t)pChReq[i].request * requestReduction) >> 16;
        pChReq[i].limiting = true;
        powerAvailable -= ((int64_t)pChDef[i].fullPower * pChReq[i].request) >> 16;
    }
    return powerAvailable;
}

/* Public functions ----------------------------------------------------------*/
uint8_t lim_CalcLiionCellCnt(q15_16_t voltage)
{
    for (int_fast8_t cnt = CELL_CNT_MAX; cnt > 0; cnt--)
    {
        if (voltage > CELL_VOLT_MIN * cnt && voltage < CELL_VOLT_MAX * cnt)
            return (uint8_t)cnt;
    }
    return 0;
}

q15_16_t lim_CalcLiionVoltageLimit(q15_16_t voltage, uint8_t cellCnt)
{
    if (cellCnt == 0)
        return 0;

    voltage /= cellCnt;

    if (voltage > CELL_VOLT_MAX || voltage < VOLT_MIN)
        return 0;

    if (voltage < VOLT_MID)
    {
        voltage -= VOLT_MIN;
        return UNLIMITED * voltage / (2 * (VOLT_MID - VOLT_MIN));
    }

    if (voltage < VOLT_MAX)
    {
        voltage -= VOLT_MID;
        return UNLIMITED * voltage / (VOLT_MAX - VOLT_MID);
    }

    return UNLIMITED;
}

q15_16_t lim_CalcTemperatureLimit(q15_16_t temperature)
{
    if (temperature < TEMP_MIN)
        return UNLIMITED;
    if (temperature > TEMP_MAX)
        return 0;

    return UNLIMITED * (TEMP_MAX - temperature) / (TEMP_MAX - TEMP_MIN);
}

ret_code_t lim_LimitChannels(q15_16_t powerAvailable, lim_channelDef_t const* pChDef,
                         lim_channelReq_t* pChReq, uint8_t chCnt)
{
    if (pChDef == NULL || pChReq == NULL)
        return NRF_ERROR_NULL;

    if (chCnt == 0)
        return NRF_ERROR_INVALID_PARAM;

    for (uint_fast8_t i = 0; i < LIM_PRIO_CNT; i++)
    {
        powerAvailable = limitPrio(powerAvailable, i, pChDef, pChReq, chCnt);
    }

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
