/**
  ******************************************************************************
  * @file    smps.h
  * @author  RT
  * @version V1.0
  * @date    15/02/01
  * @brief   Header for smps.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SMPS_H_
#define _SMPS_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum {
    smps_STEPDOWNRIGHT = 0, smps_STEPDOWNLEFT
} smps_DriverEnum;

typedef enum {
    smps_NOERROR = 0, smps_MINDC, smps_MAXDC
} smps_ErrorEnum;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
#define smps_Off()  (PRR & (1<<PRTIM1))

/* Exported functions ------------------------------------------------------- */
void smps_Init(void);
void smps_Enable();
void smps_Disable();
smps_ErrorEnum smps_Regulator(smps_DriverEnum, uint16_t, uint16_t);
uint8_t smps_GetDutyCycle(smps_DriverEnum);

#endif /*__SMPS_H_*/

/**END OF FILE*****************************************************************/
