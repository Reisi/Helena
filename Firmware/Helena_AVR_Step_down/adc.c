/**
  ******************************************************************************
  * @file    adc.c
  * @author  RT
  * @version V1.0
  * @date    15/02/01
  * @brief   analog digital converter module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>
#include "main.h"
#include "smps.h"
#include "adc.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint16_t Temperature;
    uint16_t CurrentStepDownLeft;
    uint16_t CurrentStepDownRight;
} adc_data_t;

/* Private macros ------------------------------------------------------------*/
#define adc_StartConversion()   (ADCSRA |= (1<<ADSC))

/* Private defines -----------------------------------------------------------*/
#define TEMPERATURE         0b10001111      // ADMUX settings for temperature measurement
#define CURRSTEPDOWNRIGHT   0b10000011      // ADMUX settings for current measurement right side
#define CURRSTEPDOWNLEFT    0b10000000      // ADMUX settings for current measurement left side

/* Private variables ---------------------------------------------------------*/
static adc_mode_t adc_Mode = ADC_MODE_SINGLESHOT;
static adc_data_t adc_Raw;
static adc_compensation_t adc_Comp = ADC_COMPENSATION_DEFAULTS;

/* Private functions ---------------------------------------------------------*/
/** @brief function disable the adc
 */
static void adc_Disable() {
    ADCSRA &= ~(1<<ADEN);
    power_adc_disable();
}

/** @brief function to enable the adc
 */
static void adc_Enable() {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {power_adc_enable();}
    ADCSRA |= (1<<ADEN);
}

/** @brief adc interrupt service routine
 */
ISR(ADC_vect) {
    uint16_t adc = ADC;

    switch (ADMUX) {
    case TEMPERATURE:
        ADMUX = CURRSTEPDOWNLEFT;
        adc_StartConversion();
        if (adc_Mode == ADC_MODE_SINGLESHOT)
            adc_Raw.Temperature = adc<<2;
        else
            adc_Raw.Temperature = adc_Raw.Temperature - (adc_Raw.Temperature>>2) + adc;
        main_flag |= (1<<SAMPLETEMPERATURE);
        break;
    case CURRSTEPDOWNLEFT:
        ADMUX = CURRSTEPDOWNRIGHT;
        adc_StartConversion();
        if (adc_Mode == ADC_MODE_SINGLESHOT)
            adc_Raw.CurrentStepDownLeft = adc<<2;
        else
            adc_Raw.CurrentStepDownLeft = adc_Raw.CurrentStepDownLeft - (adc_Raw.CurrentStepDownLeft>>2) + adc;
        main_flag |= (1<<SAMPLESTEPDOWNLEFT);
        break;
    case CURRSTEPDOWNRIGHT:
        if (adc_Mode == ADC_MODE_SINGLESHOT) {
            adc_Raw.CurrentStepDownRight = adc<<2;
            adc_Disable();
        }
        else {
            ADMUX = TEMPERATURE;
            adc_StartConversion();
            adc_Raw.CurrentStepDownRight = adc_Raw.CurrentStepDownRight - (adc_Raw.CurrentStepDownRight>>2) + adc;
        }
        main_flag |= (1<<SAMPLESTEPDOWNRIGHT);
        break;
    default:
        break;
    }
}

/* Public functions ----------------------------------------------------------*/
void adc_Init()
{
    ADCSRA = (1<<ADIE)|(1<<ADPS2)|(1<<ADPS0);
    DIDR0 = (1<<ADC0D)|(1<<ADC3D);
    adc_Disable();

    // load compensation values from eeprom
    /*adc_Comp.TemperatureOffset = eeprom_read_word(TEMPOFFS);
    adc_Comp.CurrentLeftGain = eeprom_read_byte(LEFTGAIN);
    adc_Comp.CurrentRightGain = eeprom_read_byte(RIGHTGAIN);

    // default values if eeprom is empty
    if (adc_Comp.TemperatureOffset == 0xFFFF)
        adc_Comp.TemperatureOffset = 0;
    if (adc_Comp.CurrentLeftGain == 0xFF)
        adc_Comp.CurrentLeftGain = 128;
    if (adc_Comp.CurrentRightGain == 0xFF)
        adc_Comp.CurrentRightGain = 128;*/
}

void adc_SetCompensation(adc_compensation_t const* pComp)
{
    adc_Comp = *pComp;
}

void adc_SetMode(adc_mode_t mode)
{
    adc_Mode = mode;
}

void adc_StartCycle()
{
    if (adc_Off()) {
        adc_Enable();
        ADMUX = TEMPERATURE;
        adc_StartConversion();
    }
}

uint16_t adc_GetCompensated(adc_dataType_t compensate) {
    uint16_t i = 0;
    if (compensate == ADC_DATA_TEMPERATURE)
        i = adc_Raw.Temperature + adc_Comp.TemperatureOffset;
    else if (compensate == ADC_DATA_CURRENTLEFT)
        i = (((uint32_t)adc_Raw.CurrentStepDownLeft * adc_Comp.CurrentLeftGain)<<1)>>8;
    else if (compensate == ADC_DATA_CURRENTRIGHT)
        i = (((uint32_t)adc_Raw.CurrentStepDownRight * adc_Comp.CurrentRightGain)<<1)>>8;
    return i;
}

/**END OF FILE*****************************************************************/
