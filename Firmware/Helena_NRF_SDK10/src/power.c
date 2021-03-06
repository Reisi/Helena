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
#include "power.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf_soc.h"
#include "app_timer.h"
#include "nrf_adc.h"
#include "custom_board.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
// macros when using one third input scaling and bandgap reference
//#define RESULT_IN_MILLIVOLT(x)  (((uint32_t)x * 5130) >> 8)
#define RESULT_TO_Q6_10(x)      (((uint32_t)x * 5253) >> 8)

// macros when using two third input scaling and VDD/2 reference
//#define RESULT_TO_Q6_10(x)      ((x * 924549ul) >> 16)

#define SINGLE_CELL_MIN_VOLTAGE ((3000l << 10) / 1000)
#define SINGLE_CELL_MAX_VOLTAGE ((4300l << 10) / 1000)


/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t activeFlag;
static pwr_inputVoltage_t voltage;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
void ADC_IRQHandler(void)
{
    nrf_adc_conversion_event_clean();
    voltage.inputVoltage = RESULT_TO_Q6_10(nrf_adc_result_get());
    (void)app_timer_cnt_get(&voltage.timestamp);
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_DISABLED);
}

static nrf_adc_config_input_t gpioToAin(uint32_t pin)
{
    switch (pin)
    {
    case 26:
        return NRF_ADC_CONFIG_INPUT_0;
    case 27:
        return NRF_ADC_CONFIG_INPUT_1;
    case 1:
        return NRF_ADC_CONFIG_INPUT_2;
    case 2:
        return NRF_ADC_CONFIG_INPUT_3;
    case 3:
        return NRF_ADC_CONFIG_INPUT_4;
    case 4:
        return NRF_ADC_CONFIG_INPUT_5;
    case 5:
        return NRF_ADC_CONFIG_INPUT_6;
    case 6:
        return NRF_ADC_CONFIG_INPUT_7;
    default:
        return NRF_ADC_CONFIG_INPUT_DISABLED;
    }
}

static uint8_t getNumberOfCells(q6_10_t supplyVoltage)
{
    for (uint8_t i = 1; i <= 3; i++)
    {
        if (supplyVoltage >= SINGLE_CELL_MIN_VOLTAGE * i &&
            supplyVoltage <= SINGLE_CELL_MAX_VOLTAGE * i)
            return i;
    }
    return 0;
}

/* Public functions ----------------------------------------------------------*/
uint8_t pwr_Init()
{
    nrf_adc_config_t adcConfig =
    {
        .resolution = NRF_ADC_CONFIG_RES_10BIT,
        .scaling =    NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD,
        .reference =  NRF_ADC_CONFIG_REF_VBG
    };
    nrf_adc_configure(&adcConfig);

    int32_t result = nrf_adc_convert_single(gpioToAin(pBoardConfig->analogVin));

    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    sd_nvic_EnableIRQ(ADC_IRQn);

    return getNumberOfCells(RESULT_TO_Q6_10(result));
}

void pwr_SleepManagement()
{
    if (!activeFlag)
    {
        APP_ERROR_CHECK(sd_app_evt_wait());
    }
}

void pwr_SetActiveFlag(pwr_moduleFlags_t module)
{
    uint8_t mask = 1 << module;
    CRITICAL_REGION_ENTER();
    activeFlag |= mask;
    CRITICAL_REGION_EXIT();
}

void pwr_ClearActiveFlag(pwr_moduleFlags_t module)
{
    uint8_t mask = 1 << module;
    CRITICAL_REGION_ENTER();
    activeFlag &= ~mask;
    CRITICAL_REGION_EXIT();
}

uint32_t pwr_StartInputVoltageConversion()
{
    if (nrf_adc_is_busy())
        return NRF_ERROR_INVALID_STATE;

    nrf_adc_input_select(gpioToAin(pBoardConfig->analogVin));
    nrf_adc_start();

    return NRF_SUCCESS;
}

uint32_t pwr_GetInputVoltage(pwr_inputVoltage_t* pVoltage)
{
    if (nrf_adc_is_busy())
        return NRF_ERROR_INVALID_STATE;

    if (pVoltage == NULL)
        return NRF_ERROR_NULL;

    *pVoltage = voltage;

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
