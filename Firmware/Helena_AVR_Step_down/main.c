/**
  ******************************************************************************
  * @file    main.c
  * @author  Thomas Reisnecker
  * @brief   main module for 2 Channel i2c SMPS Module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <avr/io.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "usiTwiSlave.h"
#include "adc.h"
#include "main.h"
#include "smps.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint8_t AdcConfig:4;
    uint8_t Sleep:1;
    uint8_t TargetStepDownLeft;
    uint8_t TargetStepDownRight;
    uint16_t CurrentStepDownLeft;
    uint8_t StatusStepDownLeft;
    uint16_t CurrentStepDownRight;
    uint8_t StatusStepDownRight;
    uint16_t Temperature;
    adc_compensation_t adcComp;
    uint8_t adcXor;
} main_data_t;

/* Private macros ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#ifndef NULL
#define NULL 0
#endif

//#define CALIBRATION     // if this flag is enabled, both drivers are set to
                        // output 0.75A, and the temperature value is stored
                        // to last eeprom address approximately every 5 sec
#ifdef CALIBRATION
#define TEMPSAFECNT     (5*250000/(3*13))
#define TEMPSAFEADDR    (uint16_t*)510
#endif

/* Private variables ---------------------------------------------------------*/
static main_data_t main_Data;

/* Private functions ---------------------------------------------------------*/
static uint8_t calcXor(adc_compensation_t const* pComp)
{
    uint8_t xor = 0;

    xor = pComp->TemperatureOffset & 0x00FF;
    xor ^= (uint8_t)(pComp->TemperatureOffset >> 8);
    xor ^= pComp->CurrentLeftGain;
    xor ^= pComp->CurrentRightGain;

    return xor;
}

static void main_Init() {
    MCUSR &= ~(1<<WDRF);
    wdt_disable();
    ACSR |= (1<<ACD);
    power_timer0_disable();
    main_Data.Sleep = 1;
    main_Data.AdcConfig = WDTO_1S;
    rxbuffer[0] = (main_Data.Sleep<<4) | main_Data.AdcConfig;
    txbuffer[0] = (main_Data.Sleep<<4) | main_Data.AdcConfig;

    int16_t tOffset = eeprom_read_word(TEMPOFFS);
    uint8_t gainLeft = eeprom_read_byte(LEFTGAIN);
    uint8_t gainRight = eeprom_read_byte(RIGHTGAIN);

    if (tOffset == 0xFFFF && gainLeft == 0xFF && gainRight == 0xFF)
    {
        adc_compensation_t def = ADC_COMPENSATION_DEFAULTS;
        main_Data.adcComp = def;
    }
    else
    {
        main_Data.adcComp.TemperatureOffset = tOffset;
        main_Data.adcComp.CurrentLeftGain = gainLeft;
        main_Data.adcComp.CurrentRightGain = gainRight;
    }
    main_Data.adcXor = calcXor(&main_Data.adcComp);

    adc_SetCompensation(&main_Data.adcComp);
}

int main(void) {
    usiTwiSlaveInit(0x3A<<1);
    adc_Init();
    main_Init();
    smps_Init();
    sei();

    while (1) {

#ifdef CALIBRATION
        main_Data.Sleep = 0;
        main_Data.TargetStepDownLeft = 64;
        main_Data.TargetStepDownRight = 64;
#else
        /**
         * receiving twi data handling
         */
        main_Data.AdcConfig = rxbuffer[0] & 0x0F;
        if (rxbuffer[0] & (1<<4))
            main_Data.Sleep = 1;
        else
            main_Data.Sleep = 0;
        main_Data.TargetStepDownLeft = rxbuffer[1];
        main_Data.TargetStepDownRight = rxbuffer[2];
        adc_compensation_t rcvdComp;
        rcvdComp.TemperatureOffset = (int16_t)(((uint16_t)rxbuffer[11] << 8) | rxbuffer[12]);
        rcvdComp.CurrentLeftGain = rxbuffer[13];
        rcvdComp.CurrentRightGain = rxbuffer[14];
        uint8_t xor = rxbuffer[15];
        if (calcXor(&rcvdComp) == xor)
        {
            if (memcmp(&rcvdComp, &main_Data.adcComp, sizeof(adc_compensation_t)) != 0)
            {
                main_Data.adcComp = rcvdComp;
                main_Data.adcXor = xor;
                eeprom_update_word(TEMPOFFS, rcvdComp.TemperatureOffset);
                eeprom_update_byte(LEFTGAIN, rcvdComp.CurrentLeftGain);
                eeprom_update_byte(RIGHTGAIN, rcvdComp.CurrentRightGain);
                adc_SetCompensation(&rcvdComp);
            }
        }

        /**
         * transmitting twi data handling
         */
        ATOMIC_BLOCK(ATOMIC_FORCEON) {txbuffer[0] = (main_Data.Sleep<<4) | main_Data.AdcConfig;}
        txbuffer[1] = main_Data.TargetStepDownLeft;
        txbuffer[2] = main_Data.TargetStepDownRight;
        if (!(USICR & (1<<USIOIE))) {
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                txbuffer[3] = (main_Data.StatusStepDownLeft<<4) | ((main_Data.CurrentStepDownLeft & 0x0FFF)>>8);
                txbuffer[4] = main_Data.CurrentStepDownLeft & 0x00FF;
            }
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                txbuffer[5] = (main_Data.StatusStepDownRight<<4) | ((main_Data.CurrentStepDownRight & 0x0FFF)>>8);
                txbuffer[6] = main_Data.CurrentStepDownRight & 0x00FF;
            }
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                txbuffer[7] = (main_Data.Temperature & 0x0FFF)>>8;
                txbuffer[8] = main_Data.Temperature & 0x00FF;
            }
            txbuffer[9] = smps_GetDutyCycle(smps_STEPDOWNLEFT);
            txbuffer[10] = smps_GetDutyCycle(smps_STEPDOWNRIGHT);
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                txbuffer[11] = main_Data.adcComp.TemperatureOffset >> 8;
                txbuffer[12] = main_Data.adcComp.TemperatureOffset & 0x00FF;
            }
            txbuffer[13] = main_Data.adcComp.CurrentLeftGain;
            txbuffer[14] = main_Data.adcComp.CurrentRightGain;
            txbuffer[15] = main_Data.adcXor;
        }
        main_flag &= ~(1<<USIRXDONE);
#endif // CALIBRATION

        /**
         * Sleep mode checking, turn on or off watchdog with desired repetition
         * rate to trigger analog conversions
         */
        if (main_Data.Sleep == 0) {
            uint8_t i = WDTCR & 0x07;
            if (WDTCR & (1<<WDP3))
                i |= (1<<3);
            if (!(WDTCR & (1<<WDIE)) || (main_Data.AdcConfig != i)) {
                wdt_enable(main_Data.AdcConfig);
                WDTCR |= (1<<WDIE);
            }
        }
        else {
            if (WDTCR & (1<<WDIE)) {
                MCUSR &= ~(1<<WDRF);
                wdt_disable();
            }
        }
        if (main_flag & (1<<WATCHDOG)) {
            main_flag &= ~(1<<WATCHDOG);
            if (adc_Off())
                adc_StartCycle();
        }

        /**
         * Enable smps module if necessary and configure ADC in continuous mode
         */
        if ((main_Data.Sleep == 0) && (main_Data.TargetStepDownLeft || main_Data.TargetStepDownRight) && smps_Off()) {
            smps_Enable();
            adc_SetMode(ADC_MODE_CONTINUOUS);
            if (adc_Off())
                adc_StartCycle();
        }

        /**
         * Disable smps module if necessary and configure ADC in singleshot mode
         */
        if (((main_Data.Sleep != 0) || (!main_Data.TargetStepDownLeft && !main_Data.TargetStepDownRight)) && !smps_Off()) {
            smps_Disable();
            adc_SetMode(ADC_MODE_SINGLESHOT);
        }

        /**
         * compensate analog values and call regulator if necessary
         */
        if (main_flag & (1<<SAMPLETEMPERATURE)) {
            main_flag &= ~(1<<SAMPLETEMPERATURE);
            main_Data.Temperature = adc_GetCompensated(ADC_DATA_TEMPERATURE);
#ifdef CALIBRATION
            static uint16_t cnt = TEMPSAFECNT;
            if (--cnt == 0)
            {
                cnt = TEMPSAFECNT;
                eeprom_update_word(TEMPSAFEADDR, main_Data.Temperature);
            }
#endif // CALIBRATION
        }
        if (main_flag & (1<<SAMPLESTEPDOWNLEFT)) {
            main_flag &= ~(1<<SAMPLESTEPDOWNLEFT);
            main_Data.CurrentStepDownLeft = adc_GetCompensated(ADC_DATA_CURRENTLEFT);
            if (!smps_Off())
                main_Data.StatusStepDownLeft = smps_Regulator(smps_STEPDOWNLEFT, (uint16_t)main_Data.TargetStepDownLeft<<4, main_Data.CurrentStepDownLeft);
        }
        if (main_flag & (1<<SAMPLESTEPDOWNRIGHT)) {
            main_flag &= ~(1<<SAMPLESTEPDOWNRIGHT);
            main_Data.CurrentStepDownRight = adc_GetCompensated(ADC_DATA_CURRENTRIGHT);
            if (!smps_Off())
                main_Data.StatusStepDownRight = smps_Regulator(smps_STEPDOWNRIGHT, (uint16_t)main_Data.TargetStepDownRight<<4, main_Data.CurrentStepDownRight);
        }

        /*main_pComMessage = com_Check();
        if (main_pComMessage != NULL) {
            if ((main_pComMessage->Identifier == 0x00) && (main_pComMessage->Control == 0x03)) {
                if (main_pComMessage->Data[0] & (1<<4))
                    main_Data.Sleep = 1;
                else
                    main_Data.Sleep = 0;
                main_Data.AdcConfig = main_pComMessage->Data[0] & 0x0F;
                main_Data.TargetStepDownLeft = main_pComMessage->Data[1];
                main_Data.TargetStepDownRight = main_pComMessage->Data[2];
            }
            else if ((main_pComMessage->Identifier == 0x01) && (main_pComMessage->Control == 0xF0)) {
                com_MessageStruct Message;
                Message.Identifier = 0x01;
                Message.Control = 0x09;
                Message.Data[0] = main_Data.AdcConfig | (main_Data.Sleep<<4);
                Message.Data[1] = main_Data.TargetStepDownLeft;
                Message.Data[2] = main_Data.TargetStepDownRight;
                Message.Data[3] = (main_Data.StatusStepDownLeft<<4) | ((main_Data.CurrentStepDownLeft & 0x0FFF)>>8);
                Message.Data[4] = main_Data.CurrentStepDownLeft & 0x00FF;
                Message.Data[5] = (main_Data.StatusStepDownRight<<4) | ((main_Data.CurrentStepDownRight & 0x0FFF)>>8);
                Message.Data[6] = main_Data.CurrentStepDownRight & 0x00FF;
                Message.Data[7] = OCR1A;//main_Data.Temperature>>8;
                Message.Data[8] = OCR1B;//main_Data.Temperature & 0x00FF;
                com_Put(&Message);
            }
        }*/

        if (usiCheckForStop() != 0)
            main_flag |= (1<<USIRXDONE);

        /**
         * sleep mode handling (don't use sleep mode if ADC is running,
         * otherwise unwanted conversions may be triggered
         */
        if (adc_Off() && !main_flag && !(USICR & (1<<USIOIE))) {
            cli();
            sleep_enable();
            if (((~PRR) & ((1<<PRTIM0)|(1<<PRTIM1)|(1<<PRADC))) )
                set_sleep_mode(SLEEP_MODE_IDLE);
            else {
                set_sleep_mode(SLEEP_MODE_PWR_DOWN);
                sleep_bod_disable();
            }
            sei();
            sleep_cpu();
            sleep_disable();
        }
    }
}

ISR(WDT_vect) {
    main_flag |= (1<<WATCHDOG);
    WDTCR |= (1<<WDIE);
}

/* Public functions ----------------------------------------------------------*/

/**END OF FILE*****************************************************************/
