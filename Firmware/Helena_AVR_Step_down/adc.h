/**
  ******************************************************************************
  * @file    adc.h
  * @author  Thomas Reisnecker
  * @brief   Header for adc module
  *
  * @note    This module handles the analogue to digital converter. The output
  *          values are just filtered and compensated with the given
  *          compensation values. With correct calibrated values the outputs
  *          represents is:
  *          temperature in Kelvin with a resolution of 0.25K
  *          current in relation to maximum value where maximum is defined as
  *          (255 << 4), so one LSD at 3A max. represents ~0.735mA
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADC_H_
#define _ADC_H_

/* Defines -------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
    ADC_MODE_SINGLESHOT = 0,
    ADC_MODE_CONTINUOUS
} adc_mode_t;

typedef enum {
    ADC_DATA_TEMPERATURE = 0,
    ADC_DATA_CURRENTLEFT,
    ADC_DATA_CURRENTRIGHT
} adc_dataType_t;

typedef struct
{
    int16_t TemperatureOffset;  // offset with a resolution of 0.25K
    uint8_t CurrentLeftGain;   // gain in q1_7_t
    uint8_t CurrentRightGain;  // gain in q1_7_t
} adc_compensation_t;

/* Exported constants --------------------------------------------------------*/
#define ADC_COMPENSATION_DEFAULTS   \
{                                   \
    .TemperatureOffset = 0,         \
    .CurrentLeftGain = 255,         \
    .CurrentRightGain = 255         \
}

/* Exported macros -----------------------------------------------------------*/
#define adc_Off()  (PRR & (1<<PRADC))

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize adc
 */
void adc_Init(void);

/** @brief function to set the compensation values
 *
 * @param[in]   pComp   new compensation values
 *
 * @note        This function must be called with valid values prior to the first
 *              conversion to deliver correct values. If it is not called, the
 *              default values are used
 */
void adc_SetCompensation(adc_compensation_t const* pComp);

/** @brief function to set adc in single shot or continuous mode
 *
 * @param[in]   mode    desired mode
 */
void adc_SetMode(adc_mode_t mode);

/** @brief function to start a measurement cycle (one cycle = temperature +
 *         2* current)
 *
 * @note   a finished conversion is indicated in the main_flag by setting bits
 *         SAMPLESTEPDOWNRIGHT, SAMPLESTEPDOWNLEFT and SAMPLETEMPERATURE
 */
void adc_StartCycle(void);

/** @brief function to retrieve a compensated value from the adc
 *
 * @param[in]   dataType    type of data to retrieve
 * @return      the compensated data value in hardware units
 */
uint16_t adc_GetCompensated(adc_dataType_t dataType);

#endif /*_ADC_H_*/

/**END OF FILE*****************************************************************/
