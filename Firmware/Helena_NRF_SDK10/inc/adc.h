/**
  ******************************************************************************
  * @file    adc.h
  * @author  RT
  * @version V1.0
  * @date    23/02/04
  * @brief   Header for adc.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADC_H_
#define _ADC_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
    adc_CONVERSIONONGOING = 0, adc_CONVERSIONCOMPLETE
} adc_ConversionEnum;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
#define RESULT_IN_MILLIVOLT(x)  (((uint32_t)x * 5130) >> 8)

/* Exported functions ------------------------------------------------------- */
void adc_Init(void);
void adc_StartConversion(void);
uint16_t adc_GetVoltage(void);
adc_ConversionEnum adc_ConversionComplete(void);

#endif /*_ADC_H_*/

/**END OF FILE*****************************************************************/
