/**
  ******************************************************************************
  * @file    motion_sensor.c
  * @author  Thomas Reisnecker
  * @version V1.0
  * @date    16/11/18
  * @brief   helenas motion sensor module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motion_sensor.h"
#include "app_error.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "debug.h"
#include <math.h>
#include "fastmath.h"
#include "i2c.h"
#include "custom_board.h"

/* Private typedef -----------------------------------------------------------*/
typedef float float32_t;
typedef int32_t q30_t;
typedef int16_t q15_t;

/* Private defines -----------------------------------------------------------*/
#define MOVINGTHRESHOLD         (10*131)  // gyro moving threshold in degree per second times hardware units

#define EXTENDED_ERROR_CHECK            /**< activate this flag to get internal error codes */

/* Private macros ------------------------------------------------------------*/
#ifdef EXTENDED_ERROR_CHECK
#define VERIFY_MPU_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        MPU_ERROR_HANDLER(err_code);    \
        return NRF_ERROR_INTERNAL;      \
    }                                   \
} while(0)
#define VERIFY_NRF_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        APP_ERROR_HANDLER(err_code);    \
        return NRF_ERROR_INTERNAL;      \
    }                                   \
} while(0)
#else
#define VERIFY_MPU_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        return NRF_ERROR_INTERNAL;      \
    }                                   \
} while(0)
#define VERIFY_NRF_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        return NRF_ERROR_INTERNAL;      \
    }                                   \
} while(0)
#endif

#define Q15_ANGLE_INVERT(x)     ((1l<<15) - (x))

/* Private variables ---------------------------------------------------------*/
static bool isEnabled;
static const signed char gyro_orientation[9] = {-1,  0,  0,
                                                 0,  0,  1,
                                                 0,  1,  0};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static inline float32_t q15_angle_to_float(q15_t angle)
{
    if (angle > (1>>14))
        angle -= (1l<<15);
    return (float32_t)angle * (float32_t)M_TWOPI / (1<<15);
}

static inline q15_t float_angle_to_q15(float32_t angle)
{
    if (angle < 0)
        angle += (float32_t)M_TWOPI;
    return (q15_t)angle * (1<<15) / (float32_t)M_TWOPI;
}

static void normalizeAcceleration(const ms_RotationStruct* pRot, ms_AccelerationStruct* pAcc)
{
    //static float32_t lowpassyaw;

    arm_matrix_instance_f32 accIn, accOut, rot;
    float32_t accInData[3], accOutData[3], rotData[9];
    float32_t cospitch, sinpitch;
    float32_t cosroll, sinroll;
    //float32_t cosyaw, sinyaw;

    //lowpassyaw = lowpassyaw - (lowpassyaw / 256.0f) + (q15_angle_to_float(pRot->yaw) / 256.0f);

    cospitch = cosf(q15_angle_to_float(pRot->pitch));
    sinpitch = sinf(q15_angle_to_float(pRot->pitch));
    cosroll = cosf(q15_angle_to_float(pRot->roll));
    sinroll = sinf(q15_angle_to_float(pRot->roll));
    //cosyaw = cosf(q15_angle_to_float(pRot->yaw) - lowpassyaw);
    //sinyaw = sinf(q15_angle_to_float(pRot->yaw) - lowpassyaw);

    /*// rotation matrix for pitch compensation only
    rotData[0] = 1.0f;
    rotData[1] = 0.0f;
    rotData[2] = 0.0f;
    rotData[3] = 0.0f;
    rotData[4] = cospitch;
    rotData[5] = -sinpitch;
    rotData[6] = 0.0f;
    rotData[7] = sinpitch;
    rotData[8] = cospitch;
    arm_mat_init_f32(&rot, 3, 3, rotData);*/

    // rotation matrix for pitch and roll compensation
    rotData[0] = cosroll;
    rotData[1] = 0.0f;
    rotData[2] = sinroll;
    rotData[3] = sinpitch * sinroll;
    rotData[4] = cospitch;
    rotData[5] = -sinpitch * cosroll;
    rotData[6] = -cospitch * sinroll;
    rotData[7] = sinpitch;
    rotData[8] = cospitch * cosroll;
    arm_mat_init_f32(&rot, 3, 3, rotData);

    /*// rotation matrix for pitch, roll and yaw compensation
    rotData[0] = cosroll * cosyaw;
    rotData[1] = -cosroll * sinyaw;
    rotData[2] = sinroll;
    rotData[3] = sinpitch * sinroll * cosyaw + cospitch * sinyaw;
    rotData[4] = -sinpitch * sinroll * sinyaw + cospitch * cosyaw;
    rotData[5] = -sinpitch * cosroll;
    rotData[6] = -cospitch * sinroll * cosyaw + sinpitch * cosyaw;
    rotData[7] = cospitch * sinroll * sinyaw + sinpitch * cosyaw;
    rotData[8] = cospitch * cosroll;
    arm_mat_init_f32(&rot, 3, 3, rotData);*/

    accInData[0] = -pAcc->x;
    accInData[1] = pAcc->z;
    accInData[2] = pAcc->y;
    arm_mat_init_f32(&accIn, 3, 1, accInData);

    arm_mat_init_f32(&accOut, 3, 1, accOutData);

    APP_ERROR_CHECK(arm_mat_mult_f32(&rot, &accIn, &accOut));

    pAcc->x = accOutData[0];
    pAcc->y = accOutData[1];
    pAcc->z = accOutData[2];
}

static bool isHeadMoving(const short *pGyro)
{
    long rotation;

    rotation = (long)pGyro[0] * pGyro[0];
    rotation += (long)pGyro[1] * pGyro[1];
    rotation += (long)pGyro[2] * pGyro[2];

    if (rotation > ((long)MOVINGTHRESHOLD * MOVINGTHRESHOLD))
        return true;
    else
        return false;
}

static void quatToEuler(const q30_t *pQuat, q15_t *pEuler)
{
#define MUL2AB(a, b) (((int64_t)a * b) >> 29)    // helper macro for 2*a*b
    int32_t ysqr, i0, i1, i2, i3, i4;

    ysqr = MUL2AB(pQuat[2], pQuat[2]);
    i0   = MUL2AB(pQuat[0], pQuat[1]) - MUL2AB(pQuat[2], pQuat[3]);
    i1   = (1<<30) - (MUL2AB(pQuat[1], pQuat[1]) + ysqr);
    i2   = MUL2AB(pQuat[0], pQuat[2]) - MUL2AB(pQuat[3], pQuat[1]);
    i3   = MUL2AB(pQuat[0], pQuat[3]) - MUL2AB(pQuat[1], pQuat[2]);
    i4   = (1<<30) - (ysqr - MUL2AB(pQuat[3], pQuat[3]));

    /*// pitch
    pEuler[0] = arm_atan2_q15(i0>>16, i1>>16);
    // roll
    if (i2 >= (1<<30)) {pEuler[1] = (1<<13);}
    else if (i2 <= (-1<<30)) {pEuler[1] = (3<<13);}
    else {pEuler[1] = arm_asin_q15(i2>>15);}
    // yaw
    pEuler[2] = arm_atan2_q15(i3>>16, i4>>16);*/

    float32_t i;
    i = atan2f((float32_t)i0, (float32_t)i1);
    if (i < 0) {i += (float32_t)M_TWOPI;}
    i /= (float32_t)M_TWOPI;
    pEuler[0] = i * (1<<15);
    if (i2 >= (1<<30)) {pEuler[1] = (1<<13);}
    else if (i2 <= (-1<<30)) {pEuler[1] = (3<<13);}
    else
    {
        i = asin((float32_t)i2/(1<<30));
        if (i < 0) {i += (float32_t)M_TWOPI;}
        i /= (float32_t)M_TWOPI;
        pEuler[1] = i * (1<<15);
    }
    i = atan2f((float32_t)i3, (float32_t)i4);
    if (i < 0) {i += (float32_t)M_TWOPI;}
    i /= (float32_t)M_TWOPI;
    pEuler[2] = i * (1<<15);
}

static float quatToPitch(const long *quat)
{
    long t1, t2, t3;
    float pitch;

    t1 = inv_q29_mult(quat[1], quat[2]) - inv_q29_mult(quat[0], quat[3]);
    t2 = inv_q29_mult(quat[2], quat[2]) + inv_q29_mult(quat[0], quat[0]) - (1L<<30);
    t3 = inv_q29_mult(quat[2], quat[3]) + inv_q29_mult(quat[0], quat[1]);
    pitch = atan2f((float)t3, sqrtf((float)t1*t1 + (float)t2*t2));
    t2 = inv_q29_mult(quat[3], quat[3]) + inv_q29_mult(quat[0], quat[0]) - (1L<<30);
    if (t2 < 0)
    {
        if (pitch >= 0)
            pitch = (float)M_PI - pitch;
        else
            pitch = -((float)M_PI) - pitch;
    }
    return pitch;
}

/* Public functions ----------------------------------------------------------*/
uint32_t ms_Init()
{
    VERIFY_NRF_ERROR_CODE(i2c_Init());
    VERIFY_MPU_ERROR_CODE(mpu_init(NULL));
    VERIFY_MPU_ERROR_CODE(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
    VERIFY_MPU_ERROR_CODE(mpu_set_lpf(20));
    VERIFY_MPU_ERROR_CODE(dmp_load_motion_driver_firmware());
    VERIFY_MPU_ERROR_CODE(dmp_set_orientation(inv_orientation_matrix_to_scalar(pBoardConfig->gyroOrientation)));
    VERIFY_MPU_ERROR_CODE(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO));
    VERIFY_MPU_ERROR_CODE(dmp_set_fifo_rate(100));
    VERIFY_MPU_ERROR_CODE(mpu_set_dmp_state(1));
    VERIFY_MPU_ERROR_CODE(mpu_set_sensors(0));
    return NRF_SUCCESS;
}

uint32_t ms_Enable(bool enable)
{
    int errCode;

    if (enable == isEnabled)
        return NRF_ERROR_INVALID_STATE;

    if (enable)
        errCode = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    else
        errCode = mpu_set_sensors(0);

    if (errCode == 0)
        isEnabled = enable;

    return errCode ? NRF_ERROR_INTERNAL : NRF_SUCCESS;
}

uint32_t ms_FetchData(ms_DataStruct* pData)
{
    short gyro[3], accel[3], sensors;
    long quat[4];
    unsigned long timestamp;
    unsigned char more;

    if (pData == NULL)
        return NRF_ERROR_NULL;

    if (!isEnabled)
        return NRF_ERROR_INVALID_STATE;

    sensors = INV_WXYZ_QUAT | INV_XYZ_ACCEL;
    do                                      /**< MPU is sampling data with 100Hz, too, but due to unsynchronized */
    {                                       /**< clocks, data is read out until fifo is empty. */
        int error = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
        if (error && error != -3)           /**< ignore unaligned data error (can happen if data is read, */
        {                                   /**< while mpu is writing data into fifo */
            VERIFY_MPU_ERROR_CODE(error);
        }
    }
    while (more != 0);
    if ((sensors & (INV_WXYZ_QUAT | INV_XYZ_ACCEL)) != (INV_WXYZ_QUAT | INV_XYZ_ACCEL))
        return NRF_ERROR_INVALID_DATA;

    long euler[3];
    inv_get_sensor_type_euler(quat, euler);
    for (uint_fast8_t i = 0; i < 3; i++)
    {
        if (euler[i] < 0)
            euler[i] += 360<<16;
    }
    pData->rot.pitch = euler[0]/720;
    pData->rot.roll = euler[1]/720;
    pData->rot.yaw = euler[2]/720;
    pData->acc.x = accel[0];
    pData->acc.y = accel[1];
    pData->acc.z = accel[2];
    normalizeAcceleration(&pData->rot, &pData->acc);
    pData->isMoving = isHeadMoving(gyro);
    /*static uint8_t cnt = 100;
    if (--cnt == 0)
    {
        cnt = 100;
        //SEGGER_RTT_printf(0, "%d, %d, %d\r\n", (int32_t)(pData->rot.pitch*360)/(1<<15), (int32_t)(pData->rot.roll*360)/(1<<15), (int32_t)(pData->rot.yaw*360)/(1<<15));
        //SEGGER_RTT_printf(0, "%d, %d, %d\r\n", gyro[0], gyro[1], gyro[2]);
        SEGGER_RTT_printf(0, "%d, %d, %d\r\n", pData->acc.x, pData->acc.y, pData->acc.z);
    }*/

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
