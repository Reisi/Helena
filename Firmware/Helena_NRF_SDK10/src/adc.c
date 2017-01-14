/**
  ******************************************************************************
  * @file    adc.c
  * @author  RT
  * @version V1.0
  * @date    23/02/04
  * @brief   analog digital converter module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_adc.h"
#include "adc.h"
#include "custom_board.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile uint16_t adc_Voltage;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*void ADC_IRQHandler(void)
{
    nrf_adc_conversion_event_clean();
    adc_Voltage = nrf_adc_result_get();
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_DISABLED);
}*/

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

/* Public functions ----------------------------------------------------------*/
/********************************************//**
 * \brief adc initialisation function
 *
 * \note sets reference to 1.2V and MUX to ADC1 to measure input voltage,
 *       activates adc interrupt, adc clock to 125kHz and disable digital input
 *       buffer for ADC1
 * \return void
 *
 ***********************************************/
void adc_Init()
{
    nrf_adc_config_t adcConfig =
    {
        .resolution = NRF_ADC_CONFIG_RES_10BIT,
        .scaling =    NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD,
        .reference =  NRF_ADC_CONFIG_REF_VBG
    };
    nrf_adc_configure(&adcConfig);
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    sd_nvic_EnableIRQ(ADC_IRQn);
}

/********************************************//**
 * \brief adc conversion start function
 *
 * \note enables adc and starts a singleshot conversion
 * \return void
 *
 ***********************************************/
void adc_StartConversion()
{
    nrf_adc_input_select(gpioToAin(pBoardConfig->analogVin));
    nrf_adc_start();
}

/********************************************//**
 * \brief get Voltage function
 *
 * \note returns last Voltage measurement
 * \return uint16_t last conversion result
 *                  0 if no new conversion result is availabe
 *
 ***********************************************/
uint16_t adc_GetVoltage()
{
    return adc_Voltage;
}

/********************************************//**
 * \brief Conversion Complete function
 *
 * \return adc_CONVERSIONONGOING if adc still busy
 *         adc_CONVERSIONCOMPLETE if no conversion is ongoing
 *
 ***********************************************/
adc_ConversionEnum adc_ConversionComplete()
{
    if (nrf_adc_is_busy())
        return adc_CONVERSIONONGOING;
    else
        return adc_CONVERSIONCOMPLETE;
}

/**END OF FILE*****************************************************************/



