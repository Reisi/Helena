/**
  ******************************************************************************
  * @file    power.c
  * @author  RT
  * @version V1.0
  * @date    15/03/01
  * @brief   power management module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "app_timer.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf_soc.h"
#include "comreloaded.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t pwr_ActiveFlag;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void pwr_SleepManagement()
{
    if (!pwr_ActiveFlag)
    {
        APP_ERROR_CHECK(sd_app_evt_wait());
    }
}

void pwr_SetActiveFlag(uint8_t mask)
{
    CRITICAL_REGION_ENTER();
    pwr_ActiveFlag |= mask;
    CRITICAL_REGION_EXIT();
}

void pwr_ClearActiveFlag(uint8_t mask)
{
    CRITICAL_REGION_ENTER();
    pwr_ActiveFlag &= ~mask;
    CRITICAL_REGION_EXIT();
}

uint32_t pwr_GetResetReason()
{
    static uint32_t resetReason;
    static bool registerRead = false;
    if (registerRead == false)
    {
        resetReason = NRF_POWER->RESETREAS;
        NRF_POWER->RESETREAS = 0xFFFFFFFF;
        registerRead = true;
    }
    return resetReason;
}

/**END OF FILE*****************************************************************/
