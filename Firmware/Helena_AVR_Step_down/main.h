#ifndef _MAIN_H_
#define _MAIN_H_

#define main_flag GPIOR1
#define SAMPLESTEPDOWNRIGHT 0   //new sample for StepUp Converter available
#define SAMPLESTEPDOWNLEFT  1   //new sample for StepDown Converter available
#define SAMPLETEMPERATURE   2   //new sample for Temperature available
#define USIRXDONE           3
#define WATCHDOG            4
#define COMRXDONE           5

#define TEMPOFFS    (uint16_t*)0
#define LEFTGAIN    (uint8_t*)2
#define RIGHTGAIN   (uint8_t*)3

#endif
