/**
  ******************************************************************************
  * @file    motion_sensor.h
  * @author  Thomas Reisnecker
  * @brief   header file for helenas motion sensor module
  *
  * @note    This module uses the same orientation convention as used in android
  *          mobile devices:
  *          When the lamp point towards the front then
  *          the x-axis points to the right,
  *          the y-axis points forward and
  *          the z-axis points upwards,
  *          roll is the rotation across the y-axis,
  *          pitch is the rotation across the x-axis and
  *          yaw is the rotation across the z-axis.
  *          The euler angles follow the Z-X'-Y'' convention.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTION_SENSOR_H_INCLUDED
#define MOTION_SENSOR_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include "fpint.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    q3_12_t x;          // x-axis acceleration in g
    q3_12_t y;          // y-axis acceleration in g
    q3_12_t z;          // z-axis acceleration in g
} ms_accelerationData_t;

typedef struct
{
    q15_t roll;         // roll angle in q15_t [0..1]->[0..360°]
    q15_t pitch;        // pitch angle in q15_t [0..1]->[0..360°]
    q15_t yaw;          // yaw angle in q15_t [0..1]->[0..360°]
} ms_rotationData_t;

typedef struct
{
    //ms_accelerationData_t accelBody;
    //ms_rotationData_t rot;
    q15_t pitch;        // pitch angle in q15_t [0..1]->[0..360°]
    bool isBraking;     // indicator if brake situation is detected
    bool isMoving;      // indicator if device is moving
} ms_data_t;

/** @brief handler prototype to be called if movement is detected while module is disabled
 */
typedef void (*ms_LowPowerMovementDetectedHandler_t)(void);

/* Exported constants --------------------------------------------------------*/
#define ACCEL_1G                (1<<12) // 1g for selected full scale range

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the motion sensing module
 *
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INTERNAL for communication and internal errors
 */
uint32_t ms_Init(ms_LowPowerMovementDetectedHandler_t movementHandler);

/** @brief function to enable the motion sensing
 *
 * @param[in]   enable    true to enable, false to disable
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INVALID_STATE if already in requested state
 *              NRF_ERROR_INTERNAL for communication and internal errors
 */
uint32_t ms_Enable(bool enalbe);

/** @brief function to read data
 *
 * @param[out]  pData    a pointer to the data structure
 * @return      NRF_SUCCESS
 *              NRF_ERROR_NULL if pData is 0
 *              NRF_ERROR_INVALID_STATE if module not enabled
 *              NRF_ERROR_INVALID_DATA if no valid data available
 *              NRF_ERROR_INTERNAL for communication and internal errors
 *
 * @note        This function reads the current data of the sensors and
 *              processes them with several algorithms to calculate pitch
 *              angle, brake detection and movement information.
 */
uint32_t ms_GetData(ms_data_t* pData);

/** @brief function to read the current acceleration sensor offset
 *
 * @param[out]  pData   a pointer to the data structure
 * @return      NRF_SUCCESS
 *              NRF_ERROR_NULL if pData is 0
 *              NRF_ERROR_NOT_FOUND if no valid offset data available
 */
uint32_t ms_GetSensorOffset(ms_accelerationData_t* pData);

/** @brief function to calibrate the sensor offset
 *
 * @param[out]  pData   a pointer to the data structure
 * @return      NRF_SUCCESS
 *              NRF_ERROR_INTERNAL for communication and internal errors
 *              error code of fds_write()
 *
 * @note        This function runs a self test of the sensor and the stores
 *              the retrieved values in flash.
 *
 * @note        This function must be called with the sensor either face-up or
 *              face-down (z-axis is parallel to gravity).
 */
uint32_t ms_CalibrateSensorOffset(ms_accelerationData_t* pData);

#endif /* MOTION_SENSOR_H_INCLUDED */

/**END OF FILE*****************************************************************/
