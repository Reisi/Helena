/**
  ******************************************************************************
  * @file    bmi160drv.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
//#define BRD_LOG_ENABLED

#ifdef BRD_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BRD_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "nrf_drv_gpiote.h"
#include "mpu6050drv.h"
#include "fastmath.h"

/* Defines -------------------------------------------------------------------*/
#define ACCEL_FULLSCALERANGE    8       // acceleration sensor full scale range

#define MOVINGTHRESHOLD         (10*131)// gyro moving threshold in degree per second times hardware units

#define LP_MOTION_INT_THRESH    100     // 100mg
#define LP_MOTION_INT_DURATION  5       // 5ms (trail & error, in reality this results in ~1sec)
#define LP_MOTION_INT_FREQ      5       // 5Hz sampling rate

#define ONESEC_PRELOAD          100

/* Private typedef -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
#define VERIFY_RSLT(x)              \
do                                  \
{                                   \
    if (x != 0)                     \
        return NRF_ERROR_INTERNAL;  \
} while (0)


/* Private variables ---------------------------------------------------------*/
static mpu6050drv_eventHandler_t eventHandler;
static uint32_t intPin;
static mpu6050drv_powerMode_t currentState;
static volatile uint8_t newData;

/* Private read only variables -----------------------------------------------*/

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

/**
 *  @brief      Body-to-world frame euler angles.
 *  The euler angles are output with the following convention:
 *  Pitch: -180 to 180
 *  Roll: -90 to 90
 *  Yaw: -180 to 180
 *  @param[in]  quat
 *  @param[out] data        Euler angles in q15 fixed point.
 *  @return     0 if succesfull.
 */
/*static void inv_get_sensor_type_euler(long *quat, q15_t *data)
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

    // X component of the Ybody axis in World frame
    t1 = q12 - q03;

    // Y component of the Ybody axis in World frame
    t2 = q22 + q00 - (1L << 30);
    values[2] = -atan2f((float) t1, (float) t2);

    // Z component of the Ybody axis in World frame
    t3 = q23 + q01;
    values[0] = atan2f((float) t3, sqrtf((float) t1 * t1 + (float) t2 * t2));

    // Z component of the Zbody axis in World frame
    t2 = q33 + q00 - (1L << 30);
    if (t2 < 0) {

        if (values[0] >= 0)
            values[0] = (float) M_PI - values[0];
        else
            values[0] = (float) -M_PI - values[0];

    }

    // X component of the Xbody axis in World frame
    t1 = q11 + q00 - (1L << 30);
    // Y component of the Xbody axis in World frame
    t2 = q12 + q03;
    // Z component of the Xbody axis in World frame
    t3 = q13 - q02;

    values[1] = atan2f((float)(q33 + q00 - (1L << 30)), (float)(q13 - q02)) - (float) M_PI/2;
    if (values[1] >= (float) M_PI/2)
        values[1] = (float) M_PI - values[1];

    if (values[1] < (float) -M_PI/2)
        values[1] = (float) -M_PI - values[1];

    // convert rad to q15_t
    data[0] = (q15_t)(values[0] * 32768.f / (float) M_PI);
    data[1] = (q15_t)(values[1] * 32768.f / (float) M_PI);
    data[2] = (q15_t)(values[2] * 32768.f / (float) M_PI);
}*/

static void pinChangeHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (!nrf_gpio_pin_read(pin) || eventHandler == NULL)
        return;

    if (currentState == MPU6050_PWR_STANDBY)
    {   // in standby motion event can be sent directly
        mpu6050drv_event_t evt = { .type = MPU6050_EVT_TYPE_MOTION };
        eventHandler(&evt);
    }
    else if (currentState == MPU6050_PWR_ON)
    {   // in on mode, count up interrupts (interrupts are generated with 200 Hz, but fifo rate is only 100Hz
        newData++;
    }
}

static ret_code_t initIntPin(uint32_t pin)
{
    ret_code_t errCode;

    if (!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init();
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[brd]: GPIOTE init failed with error %d.", errCode);
            return errCode;
        }
    }

    nrf_drv_gpiote_in_config_t buttonConfig = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    errCode = nrf_drv_gpiote_in_init(pin, &buttonConfig, &pinChangeHandler);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[but]: GPIOTE input pin init failed with error %d.", errCode);
        return errCode;
    }

    return NRF_SUCCESS;
}

static void enablePinChange(bool enable)
{
    if (enable)
        nrf_drv_gpiote_in_event_enable(intPin, true);
    else
        nrf_drv_gpiote_in_event_disable(intPin);
}

static void getData(q15_t* pEuler, short* pGyro)
{
    short accel[3], sensors;
    long quat[4];
    unsigned long timestamp;
    unsigned char more = 0xFF;
    int errCode;

    if (eventHandler == NULL)
        return;

    do
    {
        errCode = dmp_read_fifo(pGyro, accel, quat, &timestamp, &sensors, &more);
    } while (errCode == 0 && more != 0);

    if (more == 0)
        newData = 0;

    if (errCode != 0)
    {
        //LOG_ERROR("[brd]: reading mpu fifo error %d, more %d", errCode, more);
        return;
    }

    if ((sensors & INV_WXYZ_QUAT) != INV_WXYZ_QUAT)
        return;

    long euler[3];
    inv_get_sensor_type_euler(quat, euler); // convert quaternations to euler angles
    for (uint_fast8_t i = 0; i < 3; i++)    // convert from -180°..+180° to 0..360° range
    {
        if (euler[i] < 0)
            euler[i] += 360l << 16;
    }

    mpu6050drv_event_t evt =
    {
        .type = MPU6050_EVT_TYPE_DATA,
        .data.euler[0] = euler[0]/720,
        .data.euler[1] = euler[1]/720,
        .data.euler[2] = euler[2]/720,
    };
    eventHandler(&evt);
}

/** @brief function to manually check movement, because with mpu driver it is not possible to enable both motion and dmp interrupt
 *
 * @param[in]   pGyro   gyro data
 * @return      true if is moving, otherwise false
 */
static bool isMoving(const short *pGyro)
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

/* Public functions ----------------------------------------------------------*/
ret_code_t mpu_Init(mpu6050drv_init_t const* pInit)
{
    if (pInit == NULL || pInit->handler == NULL || pInit->pOrientation == NULL)
        return NRF_ERROR_NULL;

    if (pInit->intPin == MPU6050DRV_NOINT)
        return NRF_ERROR_NOT_SUPPORTED;

    eventHandler = pInit->handler;
    intPin = pInit->intPin;

    int errCode;
    long offset[3];

    errCode = mpu_init(NULL);
    VERIFY_RSLT(errCode);

    errCode = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    VERIFY_RSLT(errCode);

    errCode = mpu_set_lpf(20);
    VERIFY_RSLT(errCode);

    //errCode = mpu_set_accel_fsr(ACCEL_FULLSCALERANGE);
    //VERIFY_RSLT(errCode);

    errCode = dmp_load_motion_driver_firmware();
    VERIFY_RSLT(errCode);

    errCode = dmp_set_orientation(inv_orientation_matrix_to_scalar(pInit->pOrientation));
    VERIFY_RSLT(errCode);

    errCode = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_CAL_GYRO);
    VERIFY_RSLT(errCode);

    errCode = dmp_set_fifo_rate(100);
    VERIFY_RSLT(errCode);

    errCode = mpu_set_dmp_state(1);
    VERIFY_RSLT(errCode);

    errCode = mpu_set_int_latched(0);
    VERIFY_RSLT(errCode);

    errCode = mpu_set_int_level(0);
    VERIFY_RSLT(errCode);

    errCode = dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
    VERIFY_RSLT(errCode);

    offset[0] = pInit->pOffset->accel[0];
    offset[1] = pInit->pOffset->accel[1];
    offset[2] = pInit->pOffset->accel[2];
    errCode = mpu_set_accel_bias_6050_reg(offset);
    VERIFY_RSLT(errCode);

    offset[0] = pInit->pOffset->gyro[0];
    offset[1] = pInit->pOffset->gyro[1];
    offset[2] = pInit->pOffset->gyro[2];
    errCode = mpu_set_gyro_bias_reg(offset);
    VERIFY_RSLT(errCode);

    errCode = mpu_set_sensors(0);
    VERIFY_RSLT(errCode);

    currentState = MPU6050_PWR_OFF;

    return initIntPin(pInit->intPin);
}

ret_code_t mpu_SetPowerMode(mpu6050drv_powerMode_t powerMode)
{
    int errCode;

    if (powerMode == currentState)
        return NRF_ERROR_INVALID_STATE; // NRF_SUCCESS;

    // disabling interrupts before changing any settings
    enablePinChange(false);

    // leaving standby mode will revert previous mode, therefore separate check is necessary
    if (currentState == MPU6050_PWR_STANDBY)
    {
        //enablePinChange(false);
        LOG_INFO("[brd]: disabling motion interrupt");
        errCode = mpu_lp_motion_interrupt(LP_MOTION_INT_THRESH, LP_MOTION_INT_DURATION, 0);
        VERIFY_RSLT(errCode);
    }

    switch (powerMode)
    {
    case MPU6050_PWR_OFF:
        LOG_INFO("[brd]: disabling imu");
        errCode = mpu_set_sensors(0);
        break;

    case MPU6050_PWR_STANDBY:
        LOG_INFO("[brd]: enabling motion interrupt");
        errCode = mpu_lp_motion_interrupt(LP_MOTION_INT_THRESH, LP_MOTION_INT_DURATION, LP_MOTION_INT_FREQ);
        if (errCode == 0)
            enablePinChange(true);
        break;//return errCode ? NRF_ERROR_INTERNAL : NRF_SUCCESS;

    case MPU6050_PWR_ON:
        LOG_INFO("[brd]: enabling imu");
        errCode = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        if (errCode == 0)
            enablePinChange(true);
        break;
    }

    if (errCode == 0)
        currentState = powerMode;

    return errCode ? NRF_ERROR_INTERNAL : NRF_SUCCESS;
}

void mpu_Execute(void)
{
    if (newData >= 2)    // interrupt is generated with 200Hz, but fifo update rate is 100Hz
    {
        static uint8_t oneSec = ONESEC_PRELOAD;

        short gyro[3];
        q15_t euler[3];

        getData(euler, gyro);
        newData = 0;

        // manually check for movement once a second
        if (oneSec && --oneSec == 0)
        {
            if (eventHandler && isMoving(gyro))
            {
                mpu6050drv_event_t evt = { .type = MPU6050_EVT_TYPE_MOTION };
                eventHandler(&evt);
            }

            oneSec = ONESEC_PRELOAD;
        }
    }
}

ret_code_t mpu_CalibrateSensorOffset(mpu6050drv_sensorOffset_t* pOffset)
{
    int errCode;
    long accel[3], gyro[3];
    mpu6050drv_powerMode_t oldState = currentState;

    if (pOffset == NULL)
        return NRF_ERROR_NULL;

    // writing to bias register has no effect if off, so make sure sensor is running
    if (currentState != MPU6050_PWR_ON)
    {
        ret_code_t errCode = mpu_SetPowerMode(MPU6050_PWR_ON);
        if (errCode != NRF_SUCCESS)
            return errCode;
    }

    // perform self test
    errCode = mpu_run_self_test(gyro, accel);
    //if (errCode != 7)
    //    return NRF_ERROR_INTERNAL;

    // self test returns values in q16 format, but register expect +-1000dps and +-16G
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        accel[i] *= 2048.f; //convert to +-16G
        accel[i] = accel[i] >> 16;
        gyro[i] = (long)(gyro[i] >> 16);
    }

    // apply new biases
    errCode = mpu_set_gyro_bias_reg(gyro);
    VERIFY_RSLT(errCode);
    errCode = mpu_set_accel_bias_6050_reg(accel);
    VERIFY_RSLT(errCode);

    // add new biases to old ones
    pOffset->accel[0] += accel[0];
    pOffset->accel[1] += accel[1];
    pOffset->accel[2] += accel[2];
    pOffset->gyro[0]  += gyro[0];
    pOffset->gyro[1]  += gyro[1];
    pOffset->gyro[2]  += gyro[2];

    // restore old state
    if (oldState != MPU6050_PWR_ON)
        return mpu_SetPowerMode(oldState);

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
