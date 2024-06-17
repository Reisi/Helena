/**
  ******************************************************************************
  * @file    filter.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <string.h>

#include "filter.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
int32_t fil_MovAvg(fil_movAvgInst_t* pInst, int32_t actual)
{
    if (pInst == NULL)
        return 0;

    pInst->pBuffer[pInst->wr] = actual;
    pInst->wr = (pInst->wr + 1) % pInst->order;

    int32_t sum = 0;
    for (uint_fast16_t i = 0; i < pInst->order; i++)
    {
        sum += pInst->pBuffer[i];
    }

    return sum / pInst->order;
}

void fil_MovAvgReset(fil_movAvgInst_t* pInst, int32_t initValue)
{
    if (pInst == NULL)
        return;

    pInst->wr = 0;
    for (uint_fast16_t i = 0; i < pInst->order; i++)
    {
        pInst->pBuffer[i] = initValue;
    }
}

void fil_LowPassFeed(fil_lowPassInst_t* pInst, int32_t actual)
{
    if (pInst == NULL)
        return;

    pInst->buffer -= pInst->buffer / pInst->tau;
    pInst->buffer += actual;
}

int32_t fil_LowPassGet(fil_lowPassInst_t const* pInst)
{
    if (pInst == NULL)
        return 0;

    return (int32_t)(pInst->buffer / pInst->tau);
}

void fil_LowPassReset(fil_lowPassInst_t* pInst, int32_t initValue)
{
    pInst->buffer = (int64_t)initValue * pInst->tau;
}

/**END OF FILE*****************************************************************/
