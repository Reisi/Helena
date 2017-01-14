/**
  ******************************************************************************
  * @file    motion_sensor.h
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/18
  * @brief   header file for helenas motion sensor module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTION_SENSOR_H_INCLUDED
#define MOTION_SENSOR_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef int16_t q15_t;
typedef int16_t q3_12_t;

typedef struct
{
    q3_12_t x;
    q3_12_t y;
    q3_12_t z;
} ms_AccelerationStruct;

typedef struct
{
    q15_t pitch;
    q15_t roll;
    q15_t yaw;
} ms_RotationStruct;

typedef struct
{
    ms_AccelerationStruct acc;
    ms_RotationStruct rot;
    bool isMoving;
} ms_DataStruct;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
uint32_t ms_Init(void);
uint32_t ms_Enable(bool enalbe);
uint32_t ms_FetchData(ms_DataStruct* pData);

#endif /* MOTION_SENSOR_H_INCLUDED */

/**END OF FILE*****************************************************************/
