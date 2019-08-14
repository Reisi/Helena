/**
  ******************************************************************************
  * @file    smps.c
  * @author  RT
  * @version V1.0
  * @date    15/02/01
  * @brief   switch mode dual led driver module for step down / step up combo
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/power.h>
#include <util/atomic.h>
#include "smps.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define PSU         PB1
#define PSD         PB4
#define DCMAXSD     254
#define DCMIN       25
#define CURRMAXSD   3872

/* Private variables ---------------------------------------------------------*/
static int32_t istepdownleft, istepdownright;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/********************************************//**
 * \brief smps initialisation function
 *
 * \note sets PWM Pins to output and low level, prepares both PWM channels and
         sets maximum value to get 251.969kHz
 * \return void
 *
 ***********************************************/
void smps_Init() {
    PORTB &= ~((1<<PSD)|(1<<PSU));
    DDRB |= (1<<PSD)|(1<<PSU);
    TCCR1 = (1<<PWM1A);
    GTCCR = (1<<PWM1B);
    OCR1C = 0xFE;
    power_timer1_disable();
}

/********************************************//**
 * \brief smps enable function
 *
 * \note turns on timer and PLL, sets DCs to 0 and connects both PWM Pins
 * \return void
 *
 ***********************************************/
void smps_Enable() {
    ATOMIC_BLOCK(ATOMIC_FORCEON) {power_timer1_enable();}
    PLLCSR |= (1<<PLLE);
    while(!(PLLCSR & (1<<PLOCK)));
    PLLCSR |= (1<<PCKE);
    OCR1A = 0xFE;
    OCR1B = 0;
    GTCCR |= (1<<COM1B1);
    TCCR1 |= (1<<COM1A1)|(1<<COM1A0)|(1<<CS10);
}

/********************************************//**
 * \brief smps disable function
 *
 * \return void
 *
 ***********************************************/
void smps_Disable() {
    GTCCR &= ~((1<<COM1B1)|(1<<COM1B0));
    TCCR1 &= ~((1<<COM1A1)|(1<<COM1A0)|(1<<CS13)|(1<<CS12)|(1<<CS11)|(1<<CS10));
    PLLCSR &= ~(1<<PLLE);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {power_timer1_disable();}
    istepdownright = 0;
    istepdownleft = 0;
}

/********************************************//**
 * \brief smps regulator function
 *
 * \param driver: regulator to be updated (smsp_STEPDOWN or smps_STEPUP)
 * \param target: target current
 * \param actual: actual current
 * \return Error: Error value (smps_NOERROR, smps_MINDC or smps_MAXDC)
 *
 ***********************************************/
smps_ErrorEnum smps_Regulator(smps_DriverEnum driver, uint16_t target, uint16_t actual) {
    uint8_t min;
    int16_t p;
    smps_ErrorEnum Error = smps_NOERROR;

    if (driver == smps_STEPDOWNLEFT) {
        if (target == 0) {
            OCR1B = 0;
            istepdownleft = 0;
        }
        else {
            p = (int16_t)target - actual;
            istepdownleft += p;
            if (istepdownleft > ((int32_t)DCMAXSD<<10))
                istepdownleft = ((int32_t)DCMAXSD<<10);
            else if (istepdownleft < 0)
                istepdownleft = 0;
            p = (int16_t)(((istepdownleft>>2) + p)>>8);
            min = 254 - OCR1A;
            min = min < DCMIN ? DCMIN - min : 0;
            if (p > DCMAXSD) {
                OCR1B = DCMAXSD;
                Error = smps_MAXDC;
            }
            else if (p < min) {
                OCR1B = min;
                Error = smps_MINDC;
            }
            else 
                OCR1B = p;
        }
    }
    else if (driver == smps_STEPDOWNRIGHT) {
        if (target == 0) {
            OCR1A = 254;
            istepdownright = 0;
        }
        else {
            p = (int16_t)target - actual;
            istepdownright += p;
            if (istepdownright > ((int32_t)DCMAXSD<<10))
                istepdownright = ((int32_t)DCMAXSD<<10);
            else if (istepdownright < 0)
                istepdownright = 0;
            p = (int16_t)(((istepdownright>>2) + p)>>8);
            min = OCR1B < DCMIN ? DCMIN - OCR1B : 0;
            if (p > DCMAXSD) {
                OCR1A = 254 - DCMAXSD;
                Error = smps_MAXDC;
            }
            else if (p < min) {
                OCR1A = 254 - min;
                Error = smps_MINDC;
            }
            else
                OCR1A = 254 - p;
        }
    }

    return Error;
}

uint8_t smps_GetDutyCycle(smps_DriverEnum driver)
{
    if (driver == smps_STEPDOWNRIGHT)
        return 254 - OCR1A;
    else if (driver == smps_STEPDOWNLEFT)
        return OCR1B;
    return 0;
}

/**END OF FILE*****************************************************************/
