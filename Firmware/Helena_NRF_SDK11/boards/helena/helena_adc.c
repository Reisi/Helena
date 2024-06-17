/**
  ******************************************************************************
  * @file    helena_adc.c
  * @author  Thomas Reisnecker
  * @brief   adc module to measure the input voltage
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define BRD_LOG_ENABLED

#ifdef BRD_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BRD_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "helena_adc.h"
#include "app_util_platform.h"
#include "nrf_drv_adc.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static nrf_drv_adc_channel_t vinChannel;
static hln_adc_result_handler_t resultHandler;
//static nrf_adc_value_t result;

/* Private functions ---------------------------------------------------------*/
static q5_11_t dataConvert(nrf_adc_value_t raw)
{
    // (2<<11) /     (1/5.7     *    1/3  / 1.2 * (2<<10)) = 41.01
    //  target   100k/(100k+470k) prescale  vbg  resolution
    return raw * 41;
}

static void adcHandler(nrf_drv_adc_evt_t const *pEvent)
{
    q5_11_t result = dataConvert(pEvent->data.sample.sample);

    if (resultHandler)
        resultHandler(result);
}

static ret_code_t initAdc()
{
    nrf_drv_adc_config_t config = {.interrupt_priority = APP_IRQ_PRIORITY_LOW};

    return nrf_drv_adc_init(&config, adcHandler);
}

static ret_code_t initChannel(uint32_t vinPin, q5_11_t* pVin)
{
    vinChannel.config.config.resolution = NRF_ADC_CONFIG_RES_10BIT;
    vinChannel.config.config.input      = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD;
    vinChannel.config.config.reference  = NRF_ADC_CONFIG_REF_VBG;
    vinChannel.config.config.ain        = nrf_drv_adc_gpio_to_ain(vinPin);

    if (vinChannel.config.config.ain != NRF_ADC_CONFIG_INPUT_DISABLED)
    {
        if (pVin)
        {
            nrf_adc_value_t adcRaw;

            ret_code_t errCode = nrf_drv_adc_sample_convert(&vinChannel, &adcRaw);
            if (errCode != NRF_SUCCESS)
                return errCode;

            *pVin = dataConvert(adcRaw);

            return NRF_SUCCESS;
        }
        else
            return NRF_SUCCESS;
    }
    else
        return NRF_ERROR_INVALID_PARAM;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t hln_adc_Init(hln_adc_init_t const* pInit, q5_11_t* pVin)
{
    /// TODO: vinPin validity check

    if (pInit == NULL || pInit->resultHandler == NULL)
        return NRF_ERROR_NULL;

    ret_code_t errCode;

    errCode = initAdc();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initChannel(pInit->vinPin, pVin);
    if (errCode != NRF_SUCCESS)
        return errCode;

    resultHandler = pInit->resultHandler;

    return NRF_SUCCESS;
}

ret_code_t hln_adc_Sample()
{
    if (nrf_drv_adc_is_busy())
        return NRF_ERROR_BUSY;

    return nrf_drv_adc_sample_convert(&vinChannel, NULL);
}

/*ret_code_t hln_adc_GetResult(q5_11_t* pVin)
{
    if (result == 0)
    {
        return nrf_drv_adc_is_busy() ? NRF_ERROR_BUSY : NRF_ERROR_INVALID_STATE;
    }

    *pVin = dataConvert(result);

    return NRF_SUCCESS;
}*/

/**END OF FILE*****************************************************************/
