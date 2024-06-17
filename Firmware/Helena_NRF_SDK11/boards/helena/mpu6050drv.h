/**
  ******************************************************************************
  * @file    mpu6050drv.h
  * @author  Thomas Reisnecker
  * @brief   board specific driver for the Invensense MPU-6050 IMU
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MPU6050DRV_H_INCLUDED
#define MPU6050DRV_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include "sdk_errors.h"

/* Exported types ------------------------------------------------------------*/
typedef int16_t  q3_12_t;
typedef int16_t  q15_t;

typedef enum
{
    MPU6050_PWR_OFF,     // off mode
    MPU6050_PWR_STANDBY, // low power mode, only motion is reported
    MPU6050_PWR_ON,      // fully operational, accel and gyro are reported
                         // with ~100Hz, motion as it occurs
} mpu6050drv_powerMode_t;

typedef struct
{
    int16_t accel[3];
    int16_t gyro[3];
} mpu6050drv_sensorOffset_t;

typedef enum
{
    MPU6050_EVT_TYPE_MOTION, // motion has been detected
    MPU6050_EVT_TYPE_DATA,   // new data is available
} mpu6050drv_eventType_t;

typedef struct
{
    q15_t   euler[3];   // pitch, roll and yaw angles in q15_t [0..1]->[0..2pi]
    //q3_12_t accel[3];   // raw x, y and z acceleration (range +-8g)
    //int16_t gyro[3];    // raw x, y and z gyro data (range +-2000 DPS)
} mpu6050drv_data_t;

typedef struct
{
    mpu6050drv_eventType_t type;
    mpu6050drv_data_t      data; // used for event type MPU6050_EVT_TYPE_DATA
} mpu6050drv_event_t;

typedef void (*mpu6050drv_eventHandler_t)(mpu6050drv_event_t const* pEvt);

typedef struct
{
    uint32_t intPin;                          // GPIO pin number int1 is connected to, (0xFFFFFFFF if not connected, not supported)
    signed char const* pOrientation;          // the orientation matrix
    mpu6050drv_sensorOffset_t const* pOffset; // the initial sensor offset data
    mpu6050drv_eventHandler_t handler;        // event handler to report events, must not be NULL
} mpu6050drv_init_t;

/* Exported constants --------------------------------------------------------*/
#define MPU6050DRV_NOINT 0xFFFFFFFF

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize the MPU6050 driver
 *
 * @param[in] pInit  the initialization structure
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_NOT_SUPPORTED or NRF_ERROR_INTERNAL
 */
ret_code_t mpu_Init(mpu6050drv_init_t const* pInit);

/** @brief function to set the driver power mode
 *
 * @param[in] powerMode  the new power mode
 * @return NRF_SUCCESS
 *         NRF_ERROR_INVALID_STATE if already in requested state
 *         NRF_ERROR_INTERNAL for communication errors
 */
ret_code_t mpu_SetPowerMode(mpu6050drv_powerMode_t powerMode);

/** @brief the execute function, call in main loop
 */
void mpu_Execute(void);

/** @brief function to perform a sensor calibration
 *
 * @param[in/out] pOffset in: the old offset values, out: the new ones
 * @return NRF_SUCCESS, NRF_ERROR_NULL, NRF_ERROR_INTERNAL
 */
ret_code_t mpu_CalibrateSensorOffset(mpu6050drv_sensorOffset_t* pOffset);

#endif // mpu6050DRV

/**END OF FILE*****************************************************************/

