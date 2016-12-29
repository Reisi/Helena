/**
  ******************************************************************************
  * @file    adc.h
  * @author  RT
  * @version V1.0
  * @date    15/02/01
  * @brief   Header for adc.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _ADC_H_
#define _ADC_H_

/* Defines -------------------------------------------------------------------*/
#define TEMPOFFS    (uint16_t*)0
#define LEFTGAIN    (uint8_t*)2
#define RIGHTGAIN   (uint8_t*)3

/* Exported types ------------------------------------------------------------*/
typedef enum {
    adc_SINGLESHOT = 0, adc_CONTINOUS
} adc_ModeEnum;

typedef enum {
    adc_COMPENSATETEMPERATURE = 0, adc_COMPENSATESTEPDOWNLEFT, adc_COMPENSATESTEPDOWNRIGHT
} adc_CompensationEnum;

typedef struct {
    uint16_t Temperature;
    uint16_t CurrentStepDownLeft;
    uint16_t CurrentStepDownRight;
} adc_DataStruct;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
#define adc_Off()  (PRR & (1<<PRADC))

/* Exported functions ------------------------------------------------------- */
void adc_Init(void);
void adc_SetMode(adc_ModeEnum);
void adc_StartRow(void);
uint16_t adc_Compensation(adc_CompensationEnum);

#endif /*_ADC_H_*/

/**END OF FILE*****************************************************************/
