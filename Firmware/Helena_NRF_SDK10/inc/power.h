/**
  ******************************************************************************
  * @file    power.h
  * @author  RT
  * @version V1.0
  * @date    01.03.15
  * @brief   Header for power.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _POWER_H_
#define _POWER_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    pwr_ACTIVEHMI = 0,
    pwr_ACTIVELIGHT,
    pwr_ACTIVECOM
} pwr_ActiveFlags;

typedef struct
{
    uint16_t inputVoltage;
    uint32_t timestamp;
} pwr_VoltageStruct;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void pwr_Init(void);
void pwr_SleepManagement(void);
void pwr_SetActiveFlag(uint8_t);
void pwr_ClearActiveFlag(uint8_t);
uint32_t pwr_StartInputVoltageConversion(void);
uint32_t pwr_GetInputVoltage(pwr_VoltageStruct * pVoltage);

#endif /*_POWER_H_*/

/**END OF FILE*****************************************************************/
