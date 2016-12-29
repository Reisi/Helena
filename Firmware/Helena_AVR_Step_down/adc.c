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
#include <avr/eeprom.h>
#include <util/atomic.h>
#include "main.h"
#include "smps.h"
#include "adc.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    int16_t TemperatureOffset;
    uint8_t StepDownLeftGain;
    uint8_t StepDownRightGain;
} adc_CompensationStruct;

/* Private macros ------------------------------------------------------------*/
#define adc_StartConversion()   (ADCSRA |= (1<<ADSC))

/* Private defines -----------------------------------------------------------*/
#define TEMPERATURE         0b10001111
#define CURRSTEPDOWNRIGHT   0b10000011
#define CURRSTEPDOWNLEFT    0b10000000

/* Private variables ---------------------------------------------------------*/
static adc_ModeEnum adc_Mode = adc_SINGLESHOT;
static adc_DataStruct adc_Raw;
static adc_CompensationStruct adc_Comp;// = {-12,224,217};

/* Private function prototypes -----------------------------------------------*/
static void adc_Disable(void);
static void adc_Enable(void);

/* Private functions ---------------------------------------------------------*/
/********************************************//**
 * \brief adc disable function
 *
 * \return void
 *
 ***********************************************/
static void adc_Disable() {
    ADCSRA &= ~(1<<ADEN);
    power_adc_disable();
}

/********************************************//**
 * \brief adc enable function
 *
 * \return void
 *
 ***********************************************/
static void adc_Enable() {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {power_adc_enable();}
    ADCSRA |= (1<<ADEN);
}

/********************************************//**
 * \brief adc interrupt service routine
 *
 * \return void
 *
 ***********************************************/
ISR(ADC_vect) {
    uint16_t adc = ADC;

    switch (ADMUX) {
    case TEMPERATURE:
        ADMUX = CURRSTEPDOWNLEFT;
        adc_StartConversion();
        if (adc_Mode == adc_SINGLESHOT)
            adc_Raw.Temperature = adc<<2;
        else 
            adc_Raw.Temperature = adc_Raw.Temperature - (adc_Raw.Temperature>>2) + adc;
        main_flag |= (1<<SAMPLETEMPERATURE);
        break;
    case CURRSTEPDOWNLEFT:
        ADMUX = CURRSTEPDOWNRIGHT;
        adc_StartConversion();
        if (adc_Mode == adc_SINGLESHOT)
            adc_Raw.CurrentStepDownLeft = adc<<2;
        else
            adc_Raw.CurrentStepDownLeft = adc_Raw.CurrentStepDownLeft - (adc_Raw.CurrentStepDownLeft>>2) + adc;
        main_flag |= (1<<SAMPLESTEPDOWNLEFT);
        break;
    case CURRSTEPDOWNRIGHT:
        if (adc_Mode == adc_SINGLESHOT) {
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
/********************************************//**
 * \brief adc initialisation function
 *
 * \note sets adc clock to 250kHz, activates adc Interrupt
 * \return void
 *
 ***********************************************/
void adc_Init() {
    ADCSRA = (1<<ADIE)|(1<<ADPS2)|(1<<ADPS0);
    DIDR0 = (1<<ADC0D)|(1<<ADC3D);
    adc_Disable();

    // load compensation values from eeprom
    adc_Comp.TemperatureOffset = eeprom_read_word(TEMPOFFS);
    adc_Comp.StepDownLeftGain = eeprom_read_byte(LEFTGAIN);
    adc_Comp.StepDownRightGain = eeprom_read_byte(RIGHTGAIN);
    
    // default values if eeprom is empty
    if (adc_Comp.TemperatureOffset == 0xFFFF)
        adc_Comp.TemperatureOffset = 0;
    if (adc_Comp.StepDownLeftGain == 0xFF)
        adc_Comp.StepDownLeftGain = 128;
    if (adc_Comp.StepDownRightGain == 0xFF)
        adc_Comp.StepDownRightGain = 128;
}

/********************************************//**
 * \brief function to set adc mode
 *
 * \param Mode: converion mode (adc_SINGLESHOT or adc_CONTINOUS)
 * \return void
 *
 ***********************************************/
void adc_SetMode(adc_ModeEnum Mode) {
    adc_Mode = Mode;
}

/********************************************//**
 * \brief function start adc measurement row
 *
 * \note this function starts temperature measurement, current measurements are
         triggered in ISR. Depending on Conversion Mode the ADC is switched off 
         after last measurement or restarted with first. 
 * \return void
 *
 ***********************************************/
void adc_StartRow() {
    if (adc_Off()) {
        adc_Enable();
        ADMUX = TEMPERATURE;
        adc_StartConversion();
    }
}

/********************************************//**
 * \brief compensation function
 *
 * \note this function compensate offset error for temperature, gain error for
 *       step down and step up current, also calculate step up output current 
 *       from input current measurement
 * \param compensate adc input to be compensated
 * \return void
 *
 ***********************************************/
uint16_t adc_Compensation(adc_CompensationEnum compensate) {
    uint16_t i = 0;
    if (compensate == adc_COMPENSATETEMPERATURE) 
        i = adc_Raw.Temperature + adc_Comp.TemperatureOffset;        
    else if (compensate == adc_COMPENSATESTEPDOWNLEFT) 
        i = (((uint32_t)adc_Raw.CurrentStepDownLeft * adc_Comp.StepDownLeftGain)<<1)>>8;
    else if (compensate == adc_COMPENSATESTEPDOWNRIGHT) 
        i = (((uint32_t)adc_Raw.CurrentStepDownRight * adc_Comp.StepDownRightGain)<<1)>>8;
    return i;
}

/**END OF FILE*****************************************************************/
