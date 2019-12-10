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
#include "fds.h"
#include "fastmath.h"
#include "i2c.h"
#include "custom_board.h"
#include "fpint.h"
#include "nrf_drv_gpiote.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    q15_t sinpitch;
    q15_t cospitch;
    q15_t sinroll;
    q15_t cosroll;
    q15_t sinyaw;
    q15_t cosyaw;
} rotationsData_t;

typedef struct
{
    long accel[3];
    long gyro[3];
} biasData_t;

/* Private defines -----------------------------------------------------------*/
#define ACCEL_FULLSCALERANGE    8       // acceleration sensor full scale range

#define MOVINGTHRESHOLD         (10*131)// gyro moving threshold in degree per second times hardware units

#define FDSACCELBIAS            0xACCE  // number identifying accel bias fds data
#define FDSINSTANCE             0x4107  // number identifying fds data for motion sensor module

#define EXTENDED_ERROR_CHECK            // activate this flag to get internal error codes

#define LP_MOTION_INT_THRESH    100     // 100mg
#define LP_MOTION_INT_DURATION  5       // 5ms (trail & error, in reality this results in ~1sec)
#define LP_MOTION_INT_FREQ      5       // 5Hz sampling rate

#define INCL_FILTER_SHIFT       3

/* Private macros ------------------------------------------------------------*/
#ifdef EXTENDED_ERROR_CHECK
#define VERIFY_MPU_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        APP_ERROR_HANDLER(-err_code);   \
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
#define Q15_MULT2(x, y)         ((q15_t)(((int32_t)(x) * (y)) >> 15))
#define Q15_MULT3(x, y, z)      ((q15_t)(((int64_t)(x) * (y) * (z)) >> 30))
#define Q31_MULT2(x, y)         ((q31_t)(((int64_t)(x) * (y)) >> 31))
#define Q29_MULT2(x, y)         ((int32_t)(((int64_t)(x) * (y)) >> 29))
#define Q31_ANGLE_DIFF(a, b)    (((a) - (b)) < -(1<<30) ? INT32_MAX - (b) + (a) : \
                                 ((a) - (b)) > (1<<30) ? INT32_MIN + (a) - (b) :  \
                                 ((a) - (b)))

/* Private variables ---------------------------------------------------------*/
static bool isEnabled;      // indicator if motion sensing is enabled or not
static biasData_t bias;     // sensor bias values
static ms_LowPowerMovementDetectedHandler_t movementIntHandler; // handler to be called , if low power movement interrupt occurred
static int32_t inclination;   // filtered inclination from external sensor, resolution tbd.

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/** @brief fds event handler
 *
 * @details this function checks the results of file operations and sends the
 *          appropriate event response to the ble modules light control service
 */
static void fdsEventHandler(ret_code_t errCode, fds_cmd_id_t cmd, fds_record_id_t recordId, fds_record_key_t recordKey)
{
    // check only events of interest
    if (recordKey.instance == FDSINSTANCE && recordKey.type == FDSACCELBIAS)
    {
        // check if this is an event related to update or write operations
        if (cmd == FDS_CMD_UPDATE || cmd == FDS_CMD_WRITE)
        {
            // run garbage collection if necessary
            if (errCode == NRF_ERROR_NO_MEM)
            {
                errCode = fds_gc();
                if (errCode != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(errCode);
                }
            }
        }
        else
            APP_ERROR_CHECK(errCode);
    }
}

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

/** @brief function to filter the acceleration and gyro data
 *
 * @param[in/out] pAccel pointer to acceleration data
 * @param[in/out] pGyro pointer to gyro data
 *
 * @note    this function runs the acceleration and gyro data through an 2nd
 *          order butterworth low-pass filter with a cut of frequency of
 *          0.1 * fsample
 */
static void filterRawData(short* pAccel, short* pGyro)
{
#define FILTER_NUMOFSTAGES  2
#define FILTER_POSTSHIFT    1

    static q15_t filterCoefficients[] =
    {
        // coefficients for 4nd order butterworth filter with fc = 0.1fs
        632, 0, 1265, 632, 17180, -4852,// b0 Q13(0.0772), 0, b1 Q13(0.154), b2 Q13(0.0772), a1 Q14(1.05), a2 Q14(-0.296)
        1024, 0, 2048, 1024, 21642, -10367// b0 Q14(0.0625), 0, b1 Q14(0.125), b2 Q14(0.0625), a1 Q14(1.32), a2 Q14(-0.633)
    };
    static arm_biquad_casd_df1_inst_q15 accelInstance[3];
    static q15_t accelState[3][4*FILTER_NUMOFSTAGES];
    static arm_biquad_casd_df1_inst_q15 gyroInstance[3];
    static q15_t gyroState[3][4*FILTER_NUMOFSTAGES];

    q15_t accelFiltered[3];
    q15_t gyroFiltered[3];

    // initialize filter instance if not done yet
    if (accelInstance[0].numStages != FILTER_NUMOFSTAGES)
    {
        arm_biquad_cascade_df1_init_q15(&accelInstance[0], FILTER_NUMOFSTAGES,
                                        filterCoefficients, accelState[0], FILTER_POSTSHIFT);
        arm_biquad_cascade_df1_init_q15(&accelInstance[1], FILTER_NUMOFSTAGES,
                                        filterCoefficients, accelState[1], FILTER_POSTSHIFT);
        arm_biquad_cascade_df1_init_q15(&accelInstance[2], FILTER_NUMOFSTAGES,
                                        filterCoefficients, accelState[2], FILTER_POSTSHIFT);
        arm_biquad_cascade_df1_init_q15(&gyroInstance[0], FILTER_NUMOFSTAGES,
                                        filterCoefficients, gyroState[0], FILTER_POSTSHIFT);
        arm_biquad_cascade_df1_init_q15(&gyroInstance[1], FILTER_NUMOFSTAGES,
                                        filterCoefficients, gyroState[1], FILTER_POSTSHIFT);
        arm_biquad_cascade_df1_init_q15(&gyroInstance[2], FILTER_NUMOFSTAGES,
                                        filterCoefficients, gyroState[2], FILTER_POSTSHIFT);
    }

    // run data through filter
    arm_biquad_cascade_df1_q15(&accelInstance[0], &pAccel[0], &accelFiltered[0], 1);
    arm_biquad_cascade_df1_q15(&accelInstance[1], &pAccel[1], &accelFiltered[1], 1);
    arm_biquad_cascade_df1_q15(&accelInstance[2], &pAccel[2], &accelFiltered[2], 1);
    arm_biquad_cascade_df1_q15(&gyroInstance[0], &pGyro[0], &gyroFiltered[0], 1);
    arm_biquad_cascade_df1_q15(&gyroInstance[1], &pGyro[1], &gyroFiltered[1], 1);
    arm_biquad_cascade_df1_q15(&gyroInstance[2], &pGyro[2], &gyroFiltered[2], 1);

    pAccel[0] = accelFiltered[0] << FILTER_POSTSHIFT;
    pAccel[1] = accelFiltered[1] << FILTER_POSTSHIFT;
    pAccel[2] = accelFiltered[2] << FILTER_POSTSHIFT;
    pGyro[0] = gyroFiltered[0] << FILTER_POSTSHIFT;
    pGyro[1] = gyroFiltered[1] << FILTER_POSTSHIFT;
    pGyro[2] = gyroFiltered[2] << FILTER_POSTSHIFT;
}

/** @brief function to rotate the raw acceleration data into helena axis
 *
 * @param[in/out] pAccel pointer to acceleration data
 */
static void rotateRawToHelenaAxis(short* pAccel)
{
    short accelRaw[3] = {pAccel[0], pAccel[1], pAccel[2]};
    pAccel[0] = pBoardConfig->gyroOrientation[0] * accelRaw[0] +
                pBoardConfig->gyroOrientation[1] * accelRaw[1] +
                pBoardConfig->gyroOrientation[2] * accelRaw[2];
    pAccel[1] = pBoardConfig->gyroOrientation[3] * accelRaw[0] +
                pBoardConfig->gyroOrientation[4] * accelRaw[1] +
                pBoardConfig->gyroOrientation[5] * accelRaw[2];
    pAccel[2] = pBoardConfig->gyroOrientation[6] * accelRaw[0] +
                pBoardConfig->gyroOrientation[7] * accelRaw[1] +
                pBoardConfig->gyroOrientation[8] * accelRaw[2];
}

/** @brief function to remove the gravity of the acceleration data
 *
 * @param[in]       pRot    current rotation data
 * @param[in/out]   accel   acceleration data
 * @return void
 *
 */
static void removeGravity(const rotationsData_t* pRot, short* accel)
{
    unsigned short accelSens;
    mpu_get_accel_sens(&accelSens);
    accel[0] -= Q15_MULT3(-pRot->cospitch, pRot->sinroll, (q15_t)accelSens);
    accel[1] -= Q15_MULT2(pRot->sinpitch, (q15_t)accelSens);
    accel[2] -= Q15_MULT3(pRot->cospitch, pRot->cosroll, (q15_t)accelSens);
}

/** @brief function to remove euler and centrifugal acceleration form acceleration data
 *
 * @param[in]       pGyro       current gyro data
 * @param[in]       pPosition   helenas current position in relation to rotation center
 * @param[in/out]   pAccel      acceleration data
 *
 * @note    This function calculates euler and centrifugal accelerations with
 *          the gyro and position data and removes this from the current
 *          acceleration values.
 */
static void removeHeadMovements(const short* pGyro, q15_t* pPosition, short* pAccel)
{
#ifndef ARM_MATH_CM0_FAMILY
#error "pState can not be NULL for non CM0 cpus"
#endif

    static short gyroOld[3];
    arm_matrix_instance_q15 accel, accelCF, accelEuler, matrix, position, i;
    q15_t accelCFData[3], accelEulerData[3], matrixData[9], iData[3];

    arm_mat_init_q15(&matrix, 3, 3, matrixData);
    arm_mat_init_q15(&accelCF, 3, 1, accelCFData);
    arm_mat_init_q15(&accelEuler, 3, 1, accelEulerData);
    arm_mat_init_q15(&position, 3, 1, pPosition);
    arm_mat_init_q15(&i, 3, 1, iData);
    arm_mat_init_q15(&accel, 3, 1, pAccel);

    // remove centrifugal acceleration
    matrixData[0] = Q15_MULT2(pGyro[1], pGyro[1]) + Q15_MULT2(pGyro[2], pGyro[2]);
    matrixData[1] = Q15_MULT2(-pGyro[0], pGyro[1]);
    matrixData[2] = Q15_MULT2(-pGyro[0], pGyro[2]);
    matrixData[3] = matrixData[1];
    matrixData[4] = Q15_MULT2(pGyro[0], pGyro[0]) + Q15_MULT2(pGyro[2], pGyro[2]);
    matrixData[5] = Q15_MULT2(-pGyro[1], pGyro[2]);
    matrixData[6] = matrixData[2];
    matrixData[7] = matrixData[5];
    matrixData[8] = Q15_MULT2(pGyro[0], pGyro[0]) + Q15_MULT2(pGyro[1], pGyro[1]);

    arm_mat_mult_q15(&matrix, &position, &accelCF, NULL);

    /// to do: change converting number into more readable preprocessor constants
    accelCFData[0] = ((q31_t)accelCFData[0] * 63594) >> 12;   // converting result into accel hardware units
    accelCFData[1] = ((q31_t)accelCFData[1] * 63594) >> 12;
    accelCFData[2] = ((q31_t)accelCFData[2] * 63594) >> 12;

    arm_mat_sub_q15(&accel, &accelCF, &i);

    // remove euler acceleration
    matrixData[0] = 0;
    matrixData[1] = pGyro[2] - gyroOld[2];
    matrixData[2] = gyroOld[1] - pGyro[1];
    matrixData[3] = -matrixData[1];
    matrixData[4] = 0;
    matrixData[5] = pGyro[0] - gyroOld[0];
    matrixData[6] = -matrixData[2];
    matrixData[7] = -matrixData[5];
    matrixData[8] = 0;

    arm_mat_mult_q15(&matrix, &position, &accelEuler, NULL);

    /// to do: change converting number into more readable preprocessor constants
    accelEulerData[0] = ((q31_t)accelEulerData[0] * 45546) >> 10;   // convert into hardware units
    accelEulerData[1] = ((q31_t)accelEulerData[1] * 45546) >> 10;
    accelEulerData[2] = ((q31_t)accelEulerData[2] * 45546) >> 10;

    gyroOld[0] = pGyro[0];
    gyroOld[1] = pGyro[1];
    gyroOld[2] = pGyro[2];

    arm_mat_sub_q15(&i, &accelEuler, &accel);
}

/** @brief function to rotate from helena axis into body axis
 *
 * @param[in]       pRot    current rotation data
 * @param[in/out]   pAccel  acceleration data
 */
static void rotateIntoBodyAxis(const rotationsData_t* pRot, short* pAccel)
{
    arm_matrix_instance_q15 accelHead, accelBody, matrix;
    q15_t accelHeadData[3], matrixData[9];

    arm_mat_init_q15(&matrix, 3, 3, matrixData);
    arm_mat_init_q15(&accelHead, 3, 1, accelHeadData);
    arm_mat_init_q15(&accelBody, 3, 1, pAccel);

    accelHeadData[0] = pAccel[0];
    accelHeadData[1] = pAccel[1];
    accelHeadData[2] = pAccel[2];

    matrixData[0] = Q15_MULT2(pRot->cosroll, pRot->cosyaw) + Q15_MULT3(pRot->sinpitch, pRot->sinroll, pRot->sinyaw);
    matrixData[1] = Q15_MULT2(-pRot->cosroll, pRot->sinyaw) + Q15_MULT3(pRot->sinpitch, pRot->sinroll, pRot->cosyaw);
    matrixData[2] = Q15_MULT2(pRot->cospitch, pRot->sinroll);
    matrixData[3] = Q15_MULT2(pRot->cospitch, pRot->sinyaw);
    matrixData[4] = Q15_MULT2(pRot->cospitch, pRot->cosyaw);
    matrixData[5] = -pRot->sinpitch;
    matrixData[6] = Q15_MULT2(-pRot->sinroll, pRot->cosyaw) + Q15_MULT3(pRot->sinpitch, pRot->cosroll, pRot->sinyaw);
    matrixData[7] = Q15_MULT2(pRot->sinroll, pRot->sinyaw) + Q15_MULT3(pRot->sinpitch, pRot->cosroll, pRot->cosyaw);
    matrixData[8] = Q15_MULT2(pRot->cospitch, pRot->cosroll);

    arm_mat_mult_q15(&matrix, &accelHead, &accelBody, NULL);
}

/** @brief function to normalize the acceleration data
 *
 * @param[in]       pRotation   current rotation data
 * @param[in/out]   accel       acceleration data
 * @param[in/out]   gyro        gyro data
 *
 * @details These function function processes the raw acceleration data and
 *          transfers them into the current acceleration appealed to the body.
 */
static void normalizeAcceleration(const short* pRotation, short* accel, short* gyro)
{
    static q31_t yawFilter;                 // long term value of yaw angle

    rotationsData_t rotationData;

    /// to do: change from fixed position to calibrated values or self finding algorithm
    q15_t position[3] = {0, -0.12 * (1<<15), -0.22 * (1<<15)};

    // roll and pitch angles can be used directly, but there is no reference
    // for the yaw angle, so we can only assume, that the mid term direction of
    // view is the riding direction. For this case we can calculate the current
    // yaw angle by using the difference of the current angle to a low-pass
    // filtered value.
    q31_t anglediff = Q31_ANGLE_DIFF(pRotation[2] << 16, yawFilter);
    yawFilter += (anglediff >> 6);  // resulting cut-off frequency 1/(2*pi*2^6*Ta) ~= 0.25Hz
    if (yawFilter < 0)
        yawFilter += INT32_MAX;
    if (anglediff < 0)
        anglediff += INT32_MAX;

    rotationData.sinroll = arm_sin_q15(pRotation[1]);
    rotationData.cosroll = arm_cos_q15(pRotation[1]);
    rotationData.sinpitch = arm_sin_q15(pRotation[0]);
    rotationData.cospitch = arm_cos_q15(pRotation[0]);
    rotationData.sinyaw = arm_sin_q15(anglediff >> 16);
    rotationData.cosyaw = arm_cos_q15(anglediff >> 16);

    filterRawData(accel, gyro);
    rotateRawToHelenaAxis(accel);
    removeGravity(&rotationData, accel);
    removeHeadMovements(gyro, position, accel);
    rotateIntoBodyAxis(&rotationData, accel);
}

/** @brief function to check if head is moving
 *
 * @param[in]   pGyro   gyro data
 * @return      true if head is moving, otherwise false
 */
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

/** @brief function to check if a braking situation is detected
 *
 * @param[in]   pAccel  current body acceleration
 * @return      true if braking situation is detected, otherwise false
 *
 * @details This function checks the braking situation by checking if
 *           - the magnitude of the acceleration is higher than the threshold,
 *           - the acceleration vector is pointing forward with a tolerance of
 *             ANGLEX in x-axis and ANGLEZ in z-axis and
 *           - the angle is not to nervous by checking the sum of the angle
 *             differences over the past data.
 */
static bool isBrakingDetected(const short* pAccel)
{
#define BRAKE_THRESHOLD             (int32_t)(ACCEL_1G * -0.2)
#define BRAKE_ANGLEX                3   // = 1/(tan(alpha)^2), 3 -> +-30°
#define BRAKE_ANGLEZ                8   // = 1/(tan(beta)^2),  8 -> +-19°
#define BRAKE_POSTFILTERSHIFT       3
#define BRAKE_NOISEFILTERCNT        10
#define BRAKE_NOISEANGLE            ((q15_t)((q31_t)30 * (2<<15) / 360))

    static int32_t accelFilter[3];
    static struct
    {
        q15_t angleBetweenVectors[BRAKE_NOISEFILTERCNT];
        uint8_t last;
    } noiseData;
    static int32_t scalarProduct, thisAccelMag, lastAccelMag;
    int32_t thisAccelSquare[3];
    q3_12_t lastAccel[3], thisAccel[3];

    // save last vector
    lastAccel[0] = accelFilter[0] >> BRAKE_POSTFILTERSHIFT;
    lastAccel[1] = accelFilter[1] >> BRAKE_POSTFILTERSHIFT;
    lastAccel[2] = accelFilter[2] >> BRAKE_POSTFILTERSHIFT;

    // run acceleration data through simple low pass filter
    accelFilter[0] = accelFilter[0] - (accelFilter[0] >> BRAKE_POSTFILTERSHIFT) + pAccel[0];
    accelFilter[1] = accelFilter[1] - (accelFilter[1] >> BRAKE_POSTFILTERSHIFT) + pAccel[1];
    accelFilter[2] = accelFilter[2] - (accelFilter[2] >> BRAKE_POSTFILTERSHIFT) + pAccel[2];

    // get actual acceleration vector
    thisAccel[0] = accelFilter[0] >> BRAKE_POSTFILTERSHIFT;
    thisAccel[1] = accelFilter[1] >> BRAKE_POSTFILTERSHIFT;
    thisAccel[2] = accelFilter[2] >> BRAKE_POSTFILTERSHIFT;

    // calculate square values
    thisAccelSquare[0] = thisAccel[0] * thisAccel[0];
    thisAccelSquare[1] = thisAccel[1] * thisAccel[1];
    thisAccelSquare[2] = thisAccel[2] * thisAccel[2];

    // calculate angle between this and last vector for noise determination
    scalarProduct = thisAccel[0] * lastAccel[0] +
                    thisAccel[1] * lastAccel[1] +
                    thisAccel[2] * lastAccel[2];
    thisAccelMag = thisAccelSquare[0] + thisAccelSquare[1] + thisAccelSquare[2];
    arm_sqrt_q31(thisAccelMag >> 1, &thisAccelMag);
    lastAccelMag = lastAccel[0] * lastAccel[0] + lastAccel[1] * lastAccel[1] + lastAccel[2] * lastAccel[2];
    arm_sqrt_q31(lastAccelMag >> 1, &lastAccelMag);
    noiseData.angleBetweenVectors[noiseData.last++] = arm_acos_q15(((int64_t)scalarProduct << 15)/(((int64_t)lastAccelMag * thisAccelMag) >> 30));
    if (noiseData.last >= BRAKE_NOISEFILTERCNT)
        noiseData.last = 0;

    // check if braking
    if ((accelFilter[1] >> BRAKE_POSTFILTERSHIFT) > 0)
        return false;

    // check if deceleration is high enough
    if (thisAccelSquare[0] + thisAccelSquare[1] + thisAccelSquare[2] < BRAKE_THRESHOLD * BRAKE_THRESHOLD)
        return false;

    // check if vector points into valid direction
    if (thisAccelSquare[1] < thisAccelSquare[0] * BRAKE_ANGLEX + thisAccelSquare[2] * BRAKE_ANGLEZ)
        return false;

    // check for noise in angle of vector
    // due to the always positive XZ component the calculated angle will be
    // always between 0 and 180°. Therefore no overflow has to be considered
    q31_t angleSum = 0;
    for (uint_fast8_t i = 0; i < BRAKE_NOISEFILTERCNT; i++)
    {
        angleSum += noiseData.angleBetweenVectors[i];
    }
    if (angleSum > BRAKE_NOISEANGLE)
        return false;

    return true;
}

/** Performs a multiply and shift by 29. These are good functions to write in assembly on
 * with devices with small memory where you want to get rid of the long long which some
 * assemblers don't handle well
 * @param[in] a
 * @param[in] b
 * @return ((long long)a*b)>>29
*/
long inv_q29_mult(long a, long b)
{
    long long temp;
    long result;
    temp = (long long)a *b;
    result = (long)(temp >> 29);
    return result;
}

/**
 *  @brief      Body-to-world frame euler angles.
 *  The euler angles are output with the following convention:
 *  Pitch: -180 to 180
 *  Roll: -90 to 90
 *  Yaw: -180 to 180
 *  @param[in]  quat
 *  @param[out] data        Euler angles in degrees, q16 fixed point.
 *  @return     0 if succesfull.
 */
int inv_get_sensor_type_euler(long *quat, long *data)
{

    long t1, t2, t3;
    long q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
    float values[3];

    q00 = inv_q29_mult(quat[0], quat[0]);
    q01 = inv_q29_mult(quat[0], quat[1]);
    q02 = inv_q29_mult(quat[0], quat[2]);
    q03 = inv_q29_mult(quat[0], quat[3]);
    q11 = inv_q29_mult(quat[1], quat[1]);
    q12 = inv_q29_mult(quat[1], quat[2]);
    q13 = inv_q29_mult(quat[1], quat[3]);
    q22 = inv_q29_mult(quat[2], quat[2]);
    q23 = inv_q29_mult(quat[2], quat[3]);
    q33 = inv_q29_mult(quat[3], quat[3]);

    /* X component of the Ybody axis in World frame */
    t1 = q12 - q03;

    /* Y component of the Ybody axis in World frame */
    t2 = q22 + q00 - (1L << 30);
    values[2] = -atan2f((float) t1, (float) t2) * 180.f / (float) M_PI;

    /* Z component of the Ybody axis in World frame */
    t3 = q23 + q01;
    values[0] =
        atan2f((float) t3,
                sqrtf((float) t1 * t1 +
                      (float) t2 * t2)) * 180.f / (float) M_PI;
    /* Z component of the Zbody axis in World frame */
    t2 = q33 + q00 - (1L << 30);
    if (t2 < 0) {

        if (values[0] >= 0)
            values[0] = 180.f - values[0];
        else
            values[0] = -180.f - values[0];

    }

    /* X component of the Xbody axis in World frame */
    t1 = q11 + q00 - (1L << 30);
    /* Y component of the Xbody axis in World frame */
    t2 = q12 + q03;
    /* Z component of the Xbody axis in World frame */
    t3 = q13 - q02;

    values[1] =
        (atan2f((float)(q33 + q00 - (1L << 30)), (float)(q13 - q02)) *
          180.f / (float) M_PI - 90);
    if (values[1] >= 90)
        values[1] = 180 - values[1];

    if (values[1] < -90)
        values[1] = -180 - values[1];
    data[0] = (long)(values[0] * 65536.f);
    data[1] = (long)(values[1] * 65536.f);
    data[2] = (long)(values[2] * 65536.f);

    return 0;
}

static void loadBias()
{
    uint32_t errCode;

    // register to fds module
    errCode = fds_register(fdsEventHandler);
    APP_ERROR_CHECK(errCode);
    errCode = fds_init();
    APP_ERROR_CHECK(errCode);

    // initialize to zero
    for (uint_fast8_t i = 0; i < 3; i++)
    {
        bias.accel[0] = 0;
        bias.gyro[0] = 0;
    }

    // search if bias data is stored in flash
    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_key_t key = {.type = FDSACCELBIAS, .instance = FDSINSTANCE};

    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode == NRF_SUCCESS)
    {
        // record exists, so open it
        fds_record_t record;
        errCode = fds_open(&descriptor, &record);
        if (errCode == NRF_SUCCESS)
        {
            if(record.header.tl.length_words == SIZEOF_WORDS(bias))
                bias = *(biasData_t*)record.p_data;
            APP_ERROR_CHECK(fds_close(&descriptor));
        }
    }
    else if (errCode != NRF_ERROR_NOT_FOUND)
    {
        APP_ERROR_HANDLER(errCode);
    }
}

static uint32_t applyBias(biasData_t* pBias)
{
    long accel[3], gyro[3];
    unsigned short accelSens;
    float gyroSens;

    if (pBias->accel[0] == 0 && pBias->accel[1] == 0 && pBias->accel[2] == 0 &&
        pBias->gyro[0] == 0 && pBias->gyro[1] == 0 && pBias->gyro[2] == 0)
        return NRF_ERROR_NOT_FOUND;

    //mpu_get_accel_sens(&accelSens);
    //mpu_get_gyro_sens(&gyroSens);
    accelSens = 2048;   // required resolution for offset cancellation
    gyroSens = 32.8f;   // required resolution for offset cancellation

    for (uint_fast8_t i = 0; i < 3; i++)
    {
        accel[i] = (pBias->accel[i] * accelSens) >> 16;
        gyro[i] = ((long)(pBias->gyro[i] * gyroSens)) >> 16;
    }

    if (mpu_set_accel_bias_6050_reg(accel) != 0)
        return NRF_ERROR_INTERNAL;
    if (mpu_set_gyro_bias_reg(gyro) != 0)
        return NRF_ERROR_INTERNAL;
    else
        return NRF_SUCCESS;
}

static void pinChangeHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (movementIntHandler == NULL)
        return;

    if (nrf_gpio_pin_read(pin))
        movementIntHandler();
}

static uint32_t initPinChangeInt()
{
    uint32_t errCode = NRF_SUCCESS;
    nrf_drv_gpiote_in_config_t pinCfg = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);

    if (!nrf_drv_gpiote_is_init())
        errCode = nrf_drv_gpiote_init();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = nrf_drv_gpiote_in_init(pBoardConfig->mpuInt, &pinCfg, pinChangeHandler);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

/* Public functions ----------------------------------------------------------*/
uint32_t ms_Init(ms_LowPowerMovementDetectedHandler_t movementHandler)
{
    VERIFY_NRF_ERROR_CODE(i2c_Init());

    VERIFY_MPU_ERROR_CODE(mpu_init(NULL));
    VERIFY_MPU_ERROR_CODE(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
    VERIFY_MPU_ERROR_CODE(mpu_set_lpf(20));
    VERIFY_MPU_ERROR_CODE(mpu_set_accel_fsr(ACCEL_FULLSCALERANGE));
    VERIFY_MPU_ERROR_CODE(dmp_load_motion_driver_firmware());
    VERIFY_MPU_ERROR_CODE(dmp_set_orientation(inv_orientation_matrix_to_scalar(pBoardConfig->gyroOrientation)));
    VERIFY_MPU_ERROR_CODE(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO));
    VERIFY_MPU_ERROR_CODE(dmp_set_fifo_rate(100));
    VERIFY_MPU_ERROR_CODE(mpu_set_dmp_state(1));
    loadBias();
    applyBias(&bias);

    if (pBoardConfig->mpuInt != 0xFFFFFFFF)
        movementIntHandler = movementHandler;

    if (movementIntHandler)
    {
        VERIFY_NRF_ERROR_CODE(initPinChangeInt());

        VERIFY_MPU_ERROR_CODE(mpu_set_int_latched(0));
        VERIFY_MPU_ERROR_CODE(mpu_set_int_level(0));
        VERIFY_MPU_ERROR_CODE(mpu_lp_motion_interrupt(LP_MOTION_INT_THRESH, LP_MOTION_INT_DURATION, LP_MOTION_INT_FREQ));

        nrf_drv_gpiote_in_event_enable(pBoardConfig->mpuInt, true);
    }
    else
    {
        VERIFY_MPU_ERROR_CODE(mpu_set_sensors(0));
    }

    return NRF_SUCCESS;
}

uint32_t ms_Enable(bool enable)
{
    int errCode;

    if (enable == isEnabled)
        return NRF_ERROR_INVALID_STATE;

    if (movementIntHandler)
    {
        if (enable)
        {
            nrf_drv_gpiote_in_event_disable(pBoardConfig->mpuInt);
            errCode = mpu_lp_motion_interrupt(LP_MOTION_INT_THRESH, LP_MOTION_INT_DURATION, 0);
        }
        else
        {
            errCode = mpu_lp_motion_interrupt(LP_MOTION_INT_THRESH, LP_MOTION_INT_DURATION, LP_MOTION_INT_FREQ);
            nrf_drv_gpiote_in_event_enable(pBoardConfig->mpuInt, true);
        }
    }
    else
    {
        if (enable)
        {
            errCode = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        }
        else
        {
            errCode = mpu_set_sensors(0);
        }
    }

    if (errCode == 0)
        isEnabled = enable;

    return errCode ? NRF_ERROR_INTERNAL : NRF_SUCCESS;
}

uint32_t ms_GetData(ms_data_t* pData)
{
    short gyro[3], accel[3], rotation[3], sensors;
    long quat[4];
    unsigned long timestamp;
    unsigned char more;

    if (pData == NULL)
        return NRF_ERROR_NULL;

    if (!isEnabled)
        return NRF_ERROR_INVALID_STATE;

    // read data
    more = 0xFF;                            // initialize to detect "empty" readings
    do                                      // MPU is sampling data with 100Hz, too, but due to unsynchronized
    {                                       // clocks, data is read out until fifo is empty.
        int error = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
        if (error != -2 &&                  // ignore unaligned data error
            error && more == 0xFF)          // and error, when trying to read empty fifo (returns -1, but more will be set to 0)
        {
            VERIFY_MPU_ERROR_CODE(error);
        }
    }
    while (more != 0);                      // read out data (and discard all but last)
    if ((sensors & (INV_WXYZ_QUAT | INV_XYZ_ACCEL)) != (INV_WXYZ_QUAT | INV_XYZ_ACCEL))
        return NRF_ERROR_INVALID_DATA;      // no valid data available

    long euler[3];
    inv_get_sensor_type_euler(quat, euler); // convert quaternations to euler angles
    for (uint_fast8_t i = 0; i < 3; i++)    // convert from -180°..+180° to 0..360° range
    {
        if (euler[i] < 0)
            euler[i] += 360l<<16;
    }
    rotation[0] = euler[0]/720;             // convert q15_16_t to q3_12_t
    rotation[1] = euler[1]/720;
    rotation[2] = euler[2]/720;

    pData->pitch = rotation[0];             // pitch can be used directly
                                            // inclination is converted to usual q15_t angle representation
    q15_t incl = inclination >> INCL_FILTER_SHIFT;
    pData->inclination = incl < 0 ? incl += (1l << 15) : incl;

    normalizeAcceleration(rotation, accel, gyro);
    pData->isBraking = isBrakingDetected(accel);
    pData->isMoving = isHeadMoving(gyro);

    return NRF_SUCCESS;
}

uint32_t ms_GetSensorOffset(ms_accelerationData_t* pData)
{
    if (bias.accel[0] == 0 && bias.accel[1] == 0 && bias.accel[2] == 0 &&
        bias.gyro[0] == 0 && bias.gyro[1] == 0 && bias.gyro[2] == 0)
        return NRF_ERROR_NOT_FOUND;

    pData->x = bias.accel[0] >> 4;          // convert q15_16_t to q3_12_t
    pData->y = bias.accel[1] >> 4;
    pData->z = bias.accel[2] >> 4;
    return NRF_SUCCESS;
}

uint32_t ms_CalibrateSensorOffset(ms_accelerationData_t* pData)
{
    uint32_t errCode;
    long gyro[3], accel[3];
    int result;
    biasData_t newBias;

    result = mpu_run_self_test(gyro, accel);
    if (result != 7)
        return NRF_ERROR_INTERNAL;

    newBias.accel[0] = accel[0];
    newBias.accel[1] = accel[1];
    newBias.accel[2] = accel[2];
    newBias.gyro[0] = gyro[0] + bias.gyro[0];
    newBias.gyro[1] = gyro[1] + bias.gyro[1];
    newBias.gyro[2] = gyro[2] + bias.gyro[2];

    applyBias(&newBias);

    bias.accel[0] += newBias.accel[0];
    bias.accel[1] += newBias.accel[1];
    bias.accel[2] += newBias.accel[2];
    bias.gyro[0] = newBias.gyro[0];
    bias.gyro[1] = newBias.gyro[1];
    bias.gyro[2] = newBias.gyro[2];

    pData->x = bias.accel[0] >> 4;  // convert q15_16_t to q3_12_t
    pData->y = bias.accel[1] >> 4;
    pData->z = bias.accel[2] >> 4;

    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_chunk_t chunk;
    chunk.p_data = &bias;
    chunk.length_words = SIZEOF_WORDS(bias);

    fds_record_key_t key = {.type = FDSACCELBIAS, .instance = FDSINSTANCE};
    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode == NRF_SUCCESS)
        return fds_update(&descriptor, key, 1, &chunk);
    else if (errCode == NRF_ERROR_NOT_FOUND)
        return fds_write(&descriptor, key, 1, &chunk);
    return errCode;
}

uint32_t ms_ReportInclination(q15_t incl)
{
    if (incl < 0)
        return NRF_ERROR_INVALID_PARAM;

    // Inclination is not stored in usual q15_t (0..1 -> 0°..360°,-1..0 unused).
    // To avoid problems with 360°<->0° overrun, the inclination is mapped into
    // the range of -0.5..0.5 -> -180°..180. Additionally it is internally stored
    // multiplied with 2^INCL_FILTER_SHIFT to minimize precision losses due to
    // filtering and calculations.

    // remap from 0°..360° to -180°..180°
    if (incl >= (1 << 14))
        incl = (1 << 14) - incl;

    inclination -= inclination >> INCL_FILTER_SHIFT;
    inclination += incl;

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
