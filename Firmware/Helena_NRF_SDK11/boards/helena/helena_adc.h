/**
  ******************************************************************************
  * @file    helena_adc.h
  * @author  Thomas Reisnecker
  * @brief   Header for helena adc module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HLN_ADC_H_INCLUDED
#define HLN_ADC_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef uint16_t q5_11_t;

typedef void (*hln_adc_result_handler_t)(q5_11_t result);

typedef struct
{
    uint32_t                 vinPin;
    hln_adc_result_handler_t resultHandler;
} hln_adc_init_t;

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the adc module
 *
 * @param[in]  pInit  the initialization structure
 * @param[out] pVin   if not null an initial measurement will performed
 * @return NRF_SUCCESS or an error
 */
ret_code_t hln_adc_Init(hln_adc_init_t const* pInit, q5_11_t* pVin);

/** @brief function to start sampling
 *
 * @return NRF_SUCCESS or NRF_ERROR_BUSY if already sampling
 * @note the result is reported with the result handler
 */
ret_code_t hln_adc_Sample(void);

/** @brief function to retrieve the latest sample value
 *
 * @param[out] pVin  the measured input voltage
 * @return NRF_SUCCESS,
 *         NRF_ERROR_BUSY if still sampling or
 *         NRF_ERROR_INVALID_STATE if sampling hasn't been started
 */
//ret_code_t hln_adc_GetResult(q5_11_t* pVin);

#endif // HLN_ADC_INCLUDED

/**END OF FILE*****************************************************************/

