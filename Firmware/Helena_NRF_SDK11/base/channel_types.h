/**
  ******************************************************************************
  * @file    channel_types.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHANNEL_TYPES_H_INCLUDED
#define CHANNEL_TYPES_H_INCLUDED

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/*typedef enum
{
    CHT_TYPE_USER,
    CHT_TYPE_CURRENT,
    CHT_TYPE_VOLTAGE,
    CHT_TYPE_PWM,
    CHT_TYPE_SWITCH,
} cht_types_t;*/

/*typedef enum
{
    CHT_SPF_USER,
    CHT_SPF_BLINK,          // changing between off and desired value
    CHT_SPF_PULS,           // changing between desired value and max. value
    CHT_SPF_BREAK,          // max. value, when brake condition is detected
    CHT_SPF_PITCHCOMP       // dynamic value depending on pitch angle
} cht_specialFeature_t;*/

typedef struct
{
    uint8_t intensity : 5;  // 1 lsb = 5%
    uint8_t mode      : 3;  // mode depending on user implimentation
} cht_5int3mode_t;

/* Exported constants --------------------------------------------------------*/
#define CHT_53_TO_PERCENT(x)    (x * 5)
#define CHT_PERCENT_TO_53(x)    ((x + 2) / 5)

#endif // CHANNEL_TYPES_H_INCLUDED

/**END OF FILE*****************************************************************/
