/**
  ******************************************************************************
  * @file    main.c
  * @author  RT
  * @version V1.0
  * @date    15/01/31
  * @brief   main module for 2 Channel i2c SMPS Module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <avr/io.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
//#include "comreloaded.h"
#include "usiTwiSlave.h"
#include "adc.h"
#include "main.h"
#include "smps.h"

/* External variables --------------------------------------------------------*/
//extern volatile uint8_t rxbuffer[buffer_size];         // Buffer to write data received from the master
//extern volatile uint8_t txbuffer[buffer_size];			// Transmission buffer to be read from the master

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
} main_DataStruct;

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
static main_DataStruct main_Data;

/* Private function prototypes -----------------------------------------------*/
static void main_Init(void);

/* Private functions ---------------------------------------------------------*/
static void main_Init() {
    MCUSR &= ~(1<<WDRF);
    wdt_disable();
    ACSR |= (1<<ACD);
    power_timer0_disable();
    main_Data.Sleep = 1;
    main_Data.AdcConfig = WDTO_1S;
    rxbuffer[0] = (main_Data.Sleep<<4) | main_Data.AdcConfig;
    txbuffer[0] = (main_Data.Sleep<<4) | main_Data.AdcConfig;
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
         * twi data handling
         */
        main_Data.AdcConfig = rxbuffer[0] & 0x0F;
        if (rxbuffer[0] & (1<<4))
            main_Data.Sleep = 1;
        else
            main_Data.Sleep = 0;
        main_Data.TargetStepDownLeft = rxbuffer[1];
        main_Data.TargetStepDownRight = rxbuffer[2];
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
                adc_StartRow();
        }

        /**
         * Enable smps module if necessary and configure ADC in continous mode
         */
        if ((main_Data.Sleep == 0) && (main_Data.TargetStepDownLeft || main_Data.TargetStepDownRight) && smps_Off()) {
            smps_Enable();
            adc_SetMode(adc_CONTINOUS);
            if (adc_Off())
                adc_StartRow();
        }

        /**
         * Disable smps module if necessary and configure ADC in singleshot mode
         */
        if (((main_Data.Sleep != 0) || (!main_Data.TargetStepDownLeft && !main_Data.TargetStepDownRight)) && !smps_Off()) {
            smps_Disable();
            adc_SetMode(adc_SINGLESHOT);
        }

        /**
         * compensate analoge values and call regulator if necessary
         */
        if (main_flag & (1<<SAMPLETEMPERATURE)) {
            main_flag &= ~(1<<SAMPLETEMPERATURE);
            main_Data.Temperature = adc_Compensation(adc_COMPENSATETEMPERATURE);
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
            main_Data.CurrentStepDownLeft = adc_Compensation(adc_COMPENSATESTEPDOWNLEFT);
            if (!smps_Off())
                main_Data.StatusStepDownLeft = smps_Regulator(smps_STEPDOWNLEFT, (uint16_t)main_Data.TargetStepDownLeft<<4, main_Data.CurrentStepDownLeft);
        }
        if (main_flag & (1<<SAMPLESTEPDOWNRIGHT)) {
            main_flag &= ~(1<<SAMPLESTEPDOWNRIGHT);
            main_Data.CurrentStepDownRight = adc_Compensation(adc_COMPENSATESTEPDOWNRIGHT);
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
