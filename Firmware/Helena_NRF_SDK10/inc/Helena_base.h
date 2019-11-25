/**
  ******************************************************************************
  * @file    Helena_base.h
  * @author  RT
  * @version V1.0
  * @date    23/02/04
  * @brief   constants definitions for Helena_base
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HELENABASE_H_
#define _HELENABASE_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define HELENABASE_ADDRESS              0x3A // this device only has one address
#define HELENABASE_DEFAULT_ADDRESS      0x3A

#define HELENABASE_RA_CONFIG            0x00
#define HELENABASE_RA_TARGETSDL         0x01
#define HELENABASE_RA_TARGETSDR         0x02
#define HELENABASE_RA_STATUSSDL_H       0x03
#define HELENABASE_RA_STATUSSDL_L       0x04
#define HELENABASE_RA_STATUSSDR_H       0x05
#define HELENABASE_RA_STATUSSDR_L       0x06
#define HELENABASE_RA_TEMPERATURE_H     0x07
#define HELENABASE_RA_TEMPERATURE_L     0x08
#define HELENABASE_RA_DUTYCYCLELEFT     0x09    // these registers are only available in v1.1 and greater
#define HELENABASE_RA_DUTYCYCLERIGHT    0x0A
#define HELENABASE_RA_TEMPOFFSET_H      0x0B    // these registers are only available in v1.2 and greater
#define HELENABASE_RA_TEMPOFFSET_L      0x0C
#define HELENABASE_RA_GAINLEFT          0x0D
#define HELENABASE_RA_GAINRIGHT         0x0E
#define HELENABASE_RA_XOR               0x0F

#define HELENABASE_CRA_SLEEP_OFFSET     4
#define HELENABASE_CRA_ADCRATE_OFFSET   0

#define HELENABASE_SLEEP_ENABLE         1
#define HELENABASE_SLEEP_DISABLE        0

#define HELENABASE_ADCRATE_16MS         0
#define HELENABASE_ADCRATE_32MS         1
#define HELENABASE_ADCRATE_64MS         2
#define HELENABASE_ADCRATE_125MS        3
#define HELENABASE_ADCRATE_250MS        4
#define HELENABASE_ADCRATE_500MS        5
#define HELENABASE_ADCRATE_1S           6
#define HELENABASE_ADCRATE_2S           7
#define HELENABASE_ADCRATE_4S           8
#define HELENABASE_ADCRATE_8S           9

#define HELENABASE_ST_STATUS_OFFSET     1
#define HELENABASE_DC_MAX               254

/* Exported macros -----------------------------------------------------------*/
#define REGISTERVALUE_IN_MILLIAMPERE(x)  (((uint32_t)x * 212) >> 8)
#define MILLIAMPERE_IN_TARGETVALUE(x)    (((uint32_t)x * 5571) >> 16)
#define REGISTERVALUE_IN_DECIKELVIN(x)   (((uint32_t)x * 5) >> 1)

/********************************************//**
 * @brief macro to convert temperature register value
 *
 * @param  x raw register value
 * @return temperature in Kelvin in q12_4_t
 ***********************************************/
#define REGISTERVALUE_TO_KELVINQ4(x)     ((uint16_t)((uint32_t)x << 2))

/********************************************//**
 * @brief macro to convert current register value
 *
 * @param  x raw current value
 * @return current in Ampere in q6_10_t
 ***********************************************/
#define REGISTERVALUE_TO_AMPEREQ10(x)    ((uint16_t)((x * 771ul) >> 10))

/********************************************//**
 * @brief macro to convert current to target value
 *
 * @param  x current in Amper in q6_10_t
 * @return target value in uint8_t
 ***********************************************/
#define AMPEREQ10_TO_TARGETVALUE(x)      ((uint8_t)((x * 85ul) >> 10))

/* Exported functions ------------------------------------------------------- */

#endif /* _HELENABASE_H_ */

/**END OF FILE*****************************************************************/
