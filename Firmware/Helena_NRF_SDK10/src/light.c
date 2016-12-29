/**
  ******************************************************************************
  * @file    light.c
  * @author  RT
  * @version V1.1
  * @date    16/11/07
  * @brief   light management module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Helena_base.h"
#include "light.h"
#include "app_timer.h"
#include "adc.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h"
//#include "nrf_delay.h"
#include "power.h"
//#include "debug.h"
#include "i2c.h"
//#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define FLOOD               0
#define SPOT                1

#define TIMEBASE_ON         (APP_TIMER_TICKS(10,0))
#define TIMEBASE_IDLE       (APP_TIMER_TICKS(1000,0))

#define VOLTAGEMIN          6000
#define VOLTAGEMAX          8500
#define CURRENTMAX          MILLIAMPERE_IN_TARGETVALUE(2500)
#define TEMPERATUREMAX      3480    // = 75 °C

#define LUTSIZE             256     // size of look up table for pitch<->comp mapping, must be power of two
#define CROSSINGSIZE        16      // size of look up table for crossing between spot and flood, must be power of two
#define PITCHOFFSETSPOT     (155)//((float32_t)0.0296706)
#define PITCHOFFSETFLOOD    (-155)//((float32_t)-0.0296706)

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATEOFF, STATEIDLE, STATEON
} stateEnum;

//typedef float float32_t;
typedef int16_t q15_t;
typedef int16_t q7_8_t;
typedef uint8_t q8_t;

typedef struct
{
    q15_t pitch[LUTSIZE];
    q7_8_t comp[LUTSIZE];
} compensationLUTStruct;

typedef struct
{
    q15_t pitch[CROSSINGSIZE + 1];
    q15_t partSpot[CROSSINGSIZE + 1];
    q15_t partFlood[CROSSINGSIZE + 1];
} crossingLUTStruct;

/* Private macros ------------------------------------------------------------*/
#define EXTENDED_ERROR_CHECK            /**< activate this flag to get internal error codes */

#ifdef EXTENDED_ERROR_CHECK

/*#define VERIFY_MPU_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        MPU_ERROR_HANDLER(err_code);    \
        return NRF_ERROR_INTERNAL;      \
    }                                   \
} while(0)*/

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

/*#define VERIFY_MPU_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        return NRF_ERROR_INTERNAL;      \
    }                                   \
} while(0)*/

#define VERIFY_NRF_ERROR_CODE(err_code) \
do                                      \
{                                       \
    if (err_code != 0)                  \
    {                                   \
        return NRF_ERROR_INTERNAL;      \
    }                                   \
} while(0)

#endif

/* Private variables ---------------------------------------------------------*/
//static volatile uint8_t timebaseFlag;   /**< flags for message sending and timebase generation */
//APP_TIMER_DEF(timebaseTimerId);         /**< timer for timebase */
//static timebaseEnum timebase;           /**< actual timebase */
static const compensationLUTStruct spotLUT =
{
    .pitch =
    {
        24576,24748,24921,25092,25262,25431,25599,25764,25927,26087,26245,
        26399,26551,26699,26844,26986,27124,27258,27389,27516,27639,27759,
        27876,27989,28098,28205,28308,28407,28504,28598,28689,28777,28862,
        28944,29024,29102,29177,29250,29320,29389,29455,29520,29582,29643,
        29702,29759,29814,29868,29921,29972,30021,30070,30117,30162,30207,
        30250,30292,30333,30374,30413,30451,30488,30524,30560,30594,30628,
        30661,30693,30725,30756,30786,30815,30844,30873,30900,30927,30954,
        30980,31005,31030,31055,31079,31102,31125,31148,31170,31192,31213,
        31234,31255,31275,31295,31315,31334,31353,31372,31390,31408,31426,
        31443,31461,31477,31494,31511,31527,31543,31558,31574,31589,31604,
        31619,31633,31648,31662,31676,31690,31704,31717,31730,31743,31756,
        31769,31782,31794,31807,31819,31831,31843,31854,31866,31877,31889,
        31900,31911,31922,31933,31944,31954,31965,31975,31985,31995,32006,
        32016,32025,32035,32045,32054,32064,32073,32083,32092,32101,32110,
        32119,32128,32137,32145,32154,32163,32171,32180,32188,32196,32204,
        32213,32221,32229,32237,32244,32252,32260,32268,32275,32283,32291,
        32298,32305,32313,32320,32327,32335,32342,32349,32356,32363,32370,
        32377,32384,32391,32397,32404,32411,32417,32424,32431,32437,32444,
        32450,32456,32463,32469,32475,32482,32488,32494,32500,32506,32512,
        32518,32524,32530,32536,32542,32548,32554,32560,32566,32571,32577,
        32583,32588,32594,32600,32605,32611,32616,32622,32627,32633,32638,
        32644,32649,32654,32660,32665,32670,32675,32681,32686,32691,32696,
        32701,32706,32712,32717,32722,32727,32732,32737,32742,32747,32752,
        32756,32761,32766
    },
    .comp =
    {
        33,33,33,33,33,34,34,35,35,36,36,37,38,39,40,41,42,43,44,46,47,48,50,
        52,53,55,57,59,61,63,65,67,70,72,74,77,80,82,85,88,91,94,97,100,103,
        107,110,114,117,121,125,128,132,136,141,145,149,153,158,163,167,172,
        177,182,187,192,197,203,208,214,219,225,231,237,243,249,256,262,269,
        275,282,289,296,303,310,318,325,333,341,349,357,365,373,381,390,399,
        408,417,426,435,444,454,464,473,483,494,504,514,525,536,547,558,569,
        581,592,604,616,629,641,654,666,679,692,706,719,733,747,761,776,790,
        805,820,835,851,867,882,899,915,932,949,966,983,1001,1019,1037,1056,
        1074,1093,1113,1132,1152,1172,1193,1213,1235,1256,1278,1300,1322,1345,
        1368,1391,1415,1439,1463,1488,1513,1538,1564,1590,1617,1644,1672,1699,
        1728,1756,1785,1815,1845,1875,1906,1937,1969,2001,2034,2067,2101,2135,
        2170,2205,2240,2277,2313,2351,2389,2427,2466,2506,2546,2587,2628,2670,
        2713,2756,2800,2845,2890,2936,2983,3030,3078,3127,3176,3227,3278,3330,
        3382,3436,3490,3545,3601,3657,3715,3774,3833,3893,3954,4016,4079,4143,
        4208,4274,4342,4410,4479,4549,4620,4692,4766,4840,4916,4993,5071,5151,
        5231,5313,5396,5480,5566,5653,5741,5831,5922,6015,6109,6204
    }
};
static const compensationLUTStruct floodLUT =
{
    .pitch =
    {
        24576,24678,24781,24883,24985,25087,25188,25289,25390,25490,25589,
        25688,25786,25883,25980,26076,26170,26264,26357,26449,26539,26629,
        26718,26805,26892,26977,27061,27144,27225,27306,27385,27463,27540,
        27616,27690,27763,27835,27906,27976,28045,28112,28178,28243,28307,
        28370,28432,28493,28553,28612,28669,28726,28782,28837,28890,28943,
        28995,29046,29097,29146,29195,29242,29289,29335,29381,29425,29469,
        29512,29555,29596,29637,29678,29717,29756,29795,29833,29870,29907,
        29943,29978,30013,30048,30082,30115,30148,30181,30213,30244,30275,
        30306,30336,30366,30395,30424,30452,30480,30508,30535,30562,30589,
        30615,30641,30667,30692,30717,30742,30766,30790,30814,30837,30860,
        30883,30906,30928,30950,30972,30994,31015,31036,31057,31077,31098,
        31118,31138,31157,31177,31196,31215,31234,31253,31271,31290,31308,
        31326,31343,31361,31378,31396,31413,31430,31446,31463,31479,31496,
        31512,31528,31544,31559,31575,31590,31606,31621,31636,31651,31666,
        31680,31695,31709,31724,31738,31752,31766,31780,31793,31807,31821,
        31834,31847,31861,31874,31887,31900,31913,31925,31938,31951,31963,
        31975,31988,32000,32012,32024,32036,32048,32060,32072,32083,32095,
        32107,32118,32129,32141,32152,32163,32174,32185,32196,32207,32218,
        32229,32240,32250,32261,32271,32282,32292,32303,32313,32323,32333,
        32344,32354,32364,32374,32384,32394,32403,32413,32423,32433,32442,
        32452,32461,32471,32480,32490,32499,32509,32518,32527,32536,32545,
        32555,32564,32573,32582,32591,32600,32608,32617,32626,32635,32644,
        32652,32661,32670,32678,32687,32695,32704,32712,32721,32729,32737,
        32746,32754,32762
    },
    .comp =
    {
        125,125,125,125,126,126,126,127,128,129,129,130,131,133,134,135,137,
        138,140,142,143,145,147,149,152,154,156,159,161,164,167,170,173,176,
        179,182,186,189,193,197,200,204,208,213,217,221,226,230,235,240,245,
        250,255,260,265,271,277,282,288,294,300,306,313,319,326,333,339,346,
        354,361,368,376,383,391,399,407,415,424,432,441,450,459,468,477,487,
        496,506,516,526,536,547,557,568,579,590,601,613,624,636,648,661,673,
        685,698,711,724,738,751,765,779,793,808,823,837,853,868,884,899,915,
        932,948,965,982,999,1017,1035,1053,1071,1090,1108,1128,1147,1167,1187,
        1207,1228,1249,1270,1291,1313,1335,1358,1381,1404,1427,1451,1475,1500,
        1525,1550,1575,1601,1628,1655,1682,1709,1737,1766,1794,1823,1853,1883,
        1914,1944,1976,2008,2040,2073,2106,2140,2174,2208,2244,2279,2316,2352,
        2390,2428,2466,2505,2544,2585,2625,2667,2709,2751,2794,2838,2882,2927,
        2973,3020,3067,3114,3163,3212,3262,3313,3364,3416,3469,3523,3578,3633,
        3689,3746,3804,3863,3922,3983,4044,4106,4170,4234,4299,4365,4432,4500,
        4569,4639,4710,4783,4856,4931,5006,5083,5161,5240,5320,5402,5484,5568,
        5653,5740,5828,5917,6007,6099,6193,6287,6384,6481,6581,6681,6783,6887,
        6993,7100,7208,7319,7431,7545,7660,7777,7897,8018
    }
};
static const crossingLUTStruct crossingLUT =
{
    .pitch =
    {
        31424,31498,31572,31647,31721,31795,31870,31944,32018,32093,32167,
        32242,32316,32390,32465,32539,32613
    },
    .partSpot =
    {
        1,1409,2884,4489,6128,7832,9634,11502,13238,16220,19038,21791,24084,
        26247,28148,29786,31195
    },
    .partFlood =
    {
        32767,32767,31850,31490,30605,29753,28869,27918,26477,23921,21037,
        18121,14975,11928,8978,6128,3473
    }
};

static stateEnum lightState;            /**< current light state */
static light_StatusStruct lightStatus;  /**< current light status informations */
static uint8_t targets[2];              /**< current target values */
static volatile bool updateFlag;        /**< flags indication if status has to be updated */
APP_TIMER_DEF(updateTimerId);           /**< timer for timebase */

/* Private functions ---------------------------------------------------------*/
/** @brief handler for timebase timer event
 * */
static void updateTimerHandler(void *pContext)
{
    (void)pContext;
    updateFlag = true;
    pwr_SetActiveFlag(1<<pwr_ACTIVELIGHT);
}

/** @brief function to read data of i2c Step down converter
 *
 * @param[out]  pStatus converter status data
 * @return      NRF_SUCCESS or error code
 *
 */
static uint32_t readConverterData(light_StatusStruct* pStatus)
{
    uint8_t Buffer[6];
    uint32_t errCode;
    uint16_t i;
    static uint32_t temperature;

    errCode = i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_STATUSSDL_H, 6, Buffer);
    if(errCode == NRF_SUCCESS)
    {
        if (Buffer[0]>>4)
            pStatus->flood |= LIGHT_STATUS_DUTYCYCLELIMIT;
        else
            pStatus->flood &= ~LIGHT_STATUS_DUTYCYCLELIMIT;
        i = ((Buffer[0] & 0x0F)<<8) | Buffer[1];
        pStatus->currentFlood = REGISTERVALUE_IN_MILLIAMPERE(i);
        if (Buffer[2]>>4)
            pStatus->spot |= LIGHT_STATUS_DUTYCYCLELIMIT;
        else
            pStatus->spot &= ~LIGHT_STATUS_DUTYCYCLELIMIT;
        i = ((Buffer[2] & 0x0F)<<8) | Buffer[2];
        pStatus->currentSpot = REGISTERVALUE_IN_MILLIAMPERE(i);
        i = (Buffer[4]<<8) | Buffer[5];
        if (temperature == 0)
            temperature = (REGISTERVALUE_IN_DECIKELVIN(i) << 8);
        else
        {
            temperature -= temperature >> 8;
            temperature += REGISTERVALUE_IN_DECIKELVIN(i);
        }
        pStatus->temperature = (temperature + 128) >> 8;
    }
    return errCode;
}

/** @brief Function to limit pitch to 4th Quadrant.
 *
 * @details This function limit the given pitch angle. An angle in the first or
 *          second quadrant will be replaced to an angle of (almost) 0. An
 *          angle in the third quadrant will be mirrored to the fourth quadrant.
 *
 * @param[in]   pitch   input angle
 * @return      limited angle
 */
static q15_t limitPitch(q15_t pitch)
{
    //q15_t convertedPitch;

    //if (pitch < 0)
    //    pitch += (float32_t)M_TWOPI;
    //pitch /= (float32_t)M_TWOPI;
    //convertedPitch = pitch * (1<<15);
    // convert angles in 1. and 2. quadrant to (almost) 0
    if (pitch <= (1<<14))
        pitch = (1<<15) - 1;
    // convert angles in 3. quadrants into 4. quadrant angles
    else if (pitch < (3<<13))
    {
        pitch = (3<<13) - pitch;
        pitch += (3<<13);
    }
    return pitch;
}

/** @brief Function to get target value out of look up table
 *
 * @details This function searches for the best mapping compensation value for
 *          the given pitch angle and multiplies this with the given intensity.
 *          The return value is mapped to a relative factor ([0..0,9961]) of
 *          the maximum output current and can be directly sent to the driver.
 *
 * @param[in] pLut      look up table to search in
 * @param[in] pitch     pitch angle
 * @param[in] intensity light intensity in lux
 * @return              relative output current
 *
 */
static q8_t getTargetFromLUT(const compensationLUTStruct * pLut, q15_t pitch, q7_8_t intensity)
{
    uint_fast8_t i, j;
    int32_t target;

    i = LUTSIZE >> 1;
    j = i >> 1;
    while(j)
    {
        i += pitch > pLut->pitch[i] ? j : -j;
        j >>= 1;
    }
    target = pLut->comp[i];         // get comp value
    target *= intensity;            // include intensity
    target >>= 16;                  // remove fractional digits
    if (target > 255)               // limit to maximum
        return 255;
    if (target < 1)                 // limit to minimum (to prevent blinking led)
        return 1;
    return (uint8_t)target;
}

/** @brief Function to get the intensity distribution when light is in adaptive
 *         mode with spot and flood on.
 *
 * @details This function searches the crossing look up table for best
 *          intensity distribution at current pitch angle.
 *
 * @param[in]   pitch           pitch angle
 * @param[in]   intensity       requested overall intensity
 * @param[out]  pIntensityFlood required intensity for flood
 * @param[out]  pIntensitySpot  required intensity for spot
 */
static void getIntensitiesFromLUT(q15_t pitch, q7_8_t intensity, q7_8_t* pIntensityFlood, q7_8_t* pIntensitySpot)
{
    if (pitch <= crossingLUT.pitch[0])
    {
        *pIntensityFlood = ((int32_t)intensity * crossingLUT.partFlood[0]) >> 15;
        *pIntensitySpot = ((int32_t)intensity * crossingLUT.partSpot[0]) >> 15;;
        return;
    }
    if (pitch >= crossingLUT.pitch[CROSSINGSIZE])
    {
        *pIntensityFlood = ((int32_t)intensity * crossingLUT.partFlood[CROSSINGSIZE]) >> 15;
        *pIntensitySpot = ((int32_t)intensity * crossingLUT.partSpot[CROSSINGSIZE]) >> 15;;
        return;
    }

    int_fast8_t bestMatch, stepSize;
    int_fast16_t frac;
    int32_t value;

    bestMatch = CROSSINGSIZE >> 1;
    stepSize = bestMatch;
    do
    {
        stepSize >>= 1;
        bestMatch += pitch > crossingLUT.pitch[bestMatch] ? stepSize : -stepSize;
    } while (stepSize > 1);
    if (pitch < crossingLUT.pitch[bestMatch])
        bestMatch--;
    frac = (pitch - crossingLUT.pitch[bestMatch]) * (1l<<15) /
                 (crossingLUT.pitch[bestMatch+1] - crossingLUT.pitch[bestMatch]);
    value = crossingLUT.partFlood[bestMatch+1] - crossingLUT.partFlood[bestMatch];
    value *= frac;
    value >>= 15;
    value += crossingLUT.partFlood[bestMatch];
    *pIntensityFlood = (intensity * value) >> 15;
    value = crossingLUT.partSpot[bestMatch+1] - crossingLUT.partSpot[bestMatch];
    value *= frac;
    value >>= 15;
    value += crossingLUT.partSpot[bestMatch];
    *pIntensitySpot = (intensity * value) >> 15;
}

/** @brief function to calculate target currents
 *
 * @param[in]   pMode   mode to configure
 * @param[in]   pitch   pitch in rad, only used in adaptive modes
 * @param[out]  pFlood  target current for flood
 * @param[out]  pSpot   target current for spot
 */
static void calculateTarget(const light_ModeStruct *pMode, q15_t pitch, uint8_t *pFlood, uint8_t *pSpot)
{
    switch (pMode->mode)
    {
    case LIGHT_MODEFLOOD:
        if (pMode->intensity >= 100)
            *pFlood = 255;
        else
            *pFlood = ((uint16_t)pMode->intensity << 8) / 100;
        *pSpot = 0;
        break;
    case LIGHT_MODESPOT:
        *pFlood = 0;
        if (pMode->intensity >= 100)
            *pSpot = 255;
        else
            *pSpot = ((uint16_t)pMode->intensity << 8) / 100;
        break;
    case LIGHT_MODEADAPTIVEFLOOD:
    {
        *pSpot = 0;
        int32_t pitch_off = pitch + PITCHOFFSETFLOOD;
        if (pitch_off >= (1<<15))
            pitch_off -= (1<<15);
        else if (pitch_off < 0)
            pitch_off += (1<<15);
        *pFlood = getTargetFromLUT(&floodLUT, limitPitch((q15_t)pitch_off), (q7_8_t)pMode->intensity << 8);
    }   break;
    case LIGHT_MODEADAPTIVESPOT:
    {
        int32_t pitch_off = pitch + PITCHOFFSETSPOT;
        if (pitch_off >= (1<<15))
            pitch_off -= (1<<15);
        else if (pitch_off < 0)
            pitch_off += (1<<15);
        *pSpot = getTargetFromLUT(&spotLUT, limitPitch((q15_t)pitch_off), (q7_8_t)pMode->intensity << 8);
        *pFlood = 0;
    }   break;
    /*    if (pitch > -0.289899)
            *pFlood = 136;
        else if (pitch < -1.38068)
            *pFlood = 22;
        else
            *pFlood = (uint8_t)(32.0573993483 * powf(-pitch, -1.1670972743));
        *pSpot = 0;
        break;*/
    //case LIGHT_MODEADAPTIVESPOT:
    //    break;
    case LIGHT_MODEADAPTIVEBOTH:
    {
        q7_8_t intensityFlood, intensitySpot;
        getIntensitiesFromLUT(limitPitch(pitch), (q7_8_t)pMode->intensity << 8, &intensityFlood, &intensitySpot);
        int32_t pitch_off = pitch + PITCHOFFSETFLOOD;
        if (pitch_off >= (1<<15))
            pitch_off -= (1<<15);
        else if (pitch_off < 0)
            pitch_off += (1<<15);
        *pFlood = getTargetFromLUT(&floodLUT, limitPitch((q15_t)pitch_off), intensityFlood);
        pitch_off = pitch + PITCHOFFSETSPOT;
        if (pitch_off >= (1<<15))
            pitch_off -= (1<<15);
        else if (pitch_off < 0)
            pitch_off += (1<<15);
        *pSpot = getTargetFromLUT(&spotLUT, limitPitch((q15_t)pitch_off), intensitySpot);
    }   break;

    /*    if (pitch > 0)
            *pFlood = 8;
        else if (pitch > -0.2869439)
            *pFlood = 8 - 128.945*pitch;
        else if (pitch < -1.3183973975)
            *pFlood = 8;
        else
            *pFlood = (uint8_t)(10.9412049589 * powf(-pitch, -1.1326890097));

        if (pitch > -0.018798)
            *pSpot = 204;
        else if (pitch < -0.233807)
            *pSpot = 1;
        else
            *pSpot = (uint8_t)(46.2825728975 * powf(-pitch, -0.4652256144) - 90);
        break;*/
    default:
        *pFlood = 0;
        *pSpot = 0;
        break;
    }
}

/** @brief limiter function
 *
 * @param[in/out]   pStatus light status
 * @param[in/out]   pFlood  flood current
 * @param[in/out]   pSpot   spot current
 */
static void applyLimiter(light_StatusStruct * pStatus, uint8_t *pFlood, uint8_t *pSpot)
{
    uint8_t limit;

    // clear status flags
    pStatus->flood &= ~(LIGHT_STATUS_OVERCURRENT | LIGHT_STATUS_VOLTAGELIMIT | LIGHT_STATUS_TEMPERATURELIMIT);
    pStatus->spot &= ~(LIGHT_STATUS_OVERCURRENT | LIGHT_STATUS_VOLTAGELIMIT | LIGHT_STATUS_TEMPERATURELIMIT);

    // check current limit first
    limit = CURRENTMAX;
    if (*pFlood > limit) {*pFlood = limit; pStatus->flood |= LIGHT_STATUS_OVERCURRENT;}
    if (*pSpot > limit) {*pSpot = limit; pStatus->spot |= LIGHT_STATUS_OVERCURRENT;}

    // check input voltage limit
    if (pStatus->inputVoltage < VOLTAGEMIN || pStatus->inputVoltage > VOLTAGEMAX)
        limit = 0;
    else if (pStatus->inputVoltage >= (VOLTAGEMIN + 1024))
        limit = 255;
    else
        limit = (pStatus->inputVoltage - VOLTAGEMIN) >> 2;
    if (*pFlood > limit) {*pFlood = limit; pStatus->flood |= LIGHT_STATUS_VOLTAGELIMIT;}
    if (*pSpot > limit) {*pSpot = limit; pStatus->spot |= LIGHT_STATUS_VOLTAGELIMIT;}

    // check temperature limit
    if (pStatus->temperature > TEMPERATUREMAX)
        limit = 0;
    else if (pStatus->temperature <= (TEMPERATUREMAX - 256))
        limit = 255;
    else
        limit = (TEMPERATUREMAX - pStatus->temperature);
    if (*pFlood > limit) {*pFlood = limit; pStatus->flood |= LIGHT_STATUS_TEMPERATURELIMIT;}
    if (*pSpot > limit) {*pSpot = limit; pStatus->spot |= LIGHT_STATUS_TEMPERATURELIMIT;}
}

/** @brief helper function to calculate pitch out of quaternion data
 *
 * @param[in]   quat    quaternion data delivered by mpu driver
 * @return      pitch in rad
 */
/*static float quatToPitch(const long *quat)
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
}*/

/** @brief mup initialization function
 *
 * @return  NRF_SUCCESS or an error
 */
/*static uint32_t mpuInit()
{
    VERIFY_MPU_ERROR_CODE(mpu_init(NULL));
    VERIFY_MPU_ERROR_CODE(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
    VERIFY_MPU_ERROR_CODE(mpu_set_lpf(20));
    VERIFY_MPU_ERROR_CODE(dmp_load_motion_driver_firmware());
    VERIFY_MPU_ERROR_CODE(dmp_set_orientation(0b01010100));
    VERIFY_MPU_ERROR_CODE(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL));
    VERIFY_MPU_ERROR_CODE(dmp_set_fifo_rate(100));
    VERIFY_MPU_ERROR_CODE(mpu_set_dmp_state(1));
    VERIFY_MPU_ERROR_CODE(mpu_set_sensors(0));
    mpuEnabled = false;
    return NRF_SUCCESS;
}*/

/* Public functions ----------------------------------------------------------*/
uint32_t light_Init()
{
    // create timer for timebase
    VERIFY_NRF_ERROR_CODE(app_timer_create(&updateTimerId, APP_TIMER_MODE_REPEATED, updateTimerHandler));

    // initialize adc to be able to get input voltage
    adc_Init();

    // initialize TWI interface
    VERIFY_NRF_ERROR_CODE(i2c_Init());

    // wait a bit to give MPU6050 time to power up, then run initialization
    //nrf_delay_ms(100);
    //VERIFY_NRF_ERROR_CODE(mpuInit());

    // enable workaround for TWI lock up in tev2 ICs
    i2c_EnableAutoRecover(true);

    lightState = STATEOFF;

    return NRF_SUCCESS;
}

uint32_t light_Enable(bool enable)
{
    uint8_t i2cBuffer[3];

    if (enable != (lightState == STATEOFF))
        return NRF_ERROR_INVALID_STATE;

    if (enable)
    {
        i2cBuffer[0] = (HELENABASE_SLEEP_DISABLE<<HELENABASE_CRA_SLEEP_OFFSET) | (HELENABASE_ADCRATE_1S<<HELENABASE_CRA_ADCRATE_OFFSET);
        VERIFY_NRF_ERROR_CODE(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, 1, i2cBuffer));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_IDLE, NULL));
        lightState = STATEIDLE;
    }
    else
    {
        i2cBuffer[0] = (HELENABASE_SLEEP_ENABLE<<HELENABASE_CRA_SLEEP_OFFSET) | (HELENABASE_ADCRATE_1S<<HELENABASE_CRA_ADCRATE_OFFSET);
        i2cBuffer[1] = 0;
        i2cBuffer[2] = 0;
        targets[FLOOD] = 0;
        targets[SPOT] = 0;
        VERIFY_NRF_ERROR_CODE(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, 3, i2cBuffer));
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        lightState = STATEOFF;
    }

    return NRF_SUCCESS;
}

uint32_t light_UpdateTargets(const light_ModeStruct* pMode, q15_t pitch, const light_StatusStruct* *ppStatus)
{
    if (pMode == NULL || ppStatus == NULL)
        return NRF_ERROR_NULL;

    if (lightState == STATEOFF)
        return NRF_ERROR_INVALID_STATE;

    // check if timer has to be changed
    if (pMode->mode != LIGHT_MODEOFF && lightState == STATEIDLE)
    {
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_ON, NULL));
        lightState = STATEON;
    }
    else if (pMode->mode == LIGHT_MODEOFF && lightState == STATEON)
    {
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_IDLE, NULL));
        lightState = STATEIDLE;
        targets[FLOOD] = 0;
        targets[SPOT] = 0;
        VERIFY_NRF_ERROR_CODE(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TARGETSDL, 2, targets));
    }

    if (pMode->mode != LIGHT_MODEOFF)
        calculateTarget(pMode, pitch, &targets[FLOOD], &targets[SPOT]);

    // clear error flags if leds are off
    if (pMode->mode == LIGHT_MODEOFF || pMode->mode == LIGHT_MODESPOT || pMode->mode == LIGHT_MODEADAPTIVESPOT)
        lightStatus.flood = 0;
    if (pMode->mode == LIGHT_MODEOFF || pMode->mode == LIGHT_MODEFLOOD || pMode->mode == LIGHT_MODEADAPTIVEFLOOD)
        lightStatus.spot = 0;
    *ppStatus = &lightStatus;

    return NRF_SUCCESS;
}

void light_Execute()
{
    if (!updateFlag)
        return;

    updateFlag = false;
    pwr_ClearActiveFlag(1<<pwr_ACTIVELIGHT);

    adc_StartConversion();                                          /**< start conversion for input voltage */

    APP_ERROR_CHECK(readConverterData(&lightStatus));               /**< read current converter status and data */

    while (adc_ConversionComplete() == adc_CONVERSIONONGOING);      /**< adc conversion should be done until now, if not wait. */
    lightStatus.inputVoltage = RESULT_IN_MILLIVOLT(adc_GetVoltage());

    if (targets[FLOOD] == 0 && targets[SPOT] == 0)                               /**< if light is off, nothing more to do */
        return;

    uint8_t limitedTargets[2];

    limitedTargets[FLOOD] = targets[FLOOD];
    limitedTargets[SPOT] = targets[SPOT];
    applyLimiter(&lightStatus, &limitedTargets[FLOOD], &limitedTargets[SPOT]);

    APP_ERROR_CHECK(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TARGETSDL, 2, limitedTargets));
}

#ifdef NOTUSED
uint32_t light_Execute(const light_ModeStruct* pMode, light_StatusStruct* pStatus)
{
    if (pMode == NULL || pStatus == NULL)
        return NRF_ERROR_NULL;

    //if (pMode->mode != LIGHT_MODEOFF && timebase == TIMEBASEOFF)
    //    return NRF_ERROR_INVALID_STATE;

    if (pMode->mode != LIGHT_MODEOFF && !moduleEnabled)
        return NRF_ERROR_INVALID_STATE;

    // check if light has to be turned on
    //if (timebase == TIMEBASELONG && pMode->mode != LIGHT_MODEOFF)
    if (!mpuEnabled && pMode->mode != LIGHT_MODEOFF)
    {
        //timebase = TIMEBASESHORT;
        //VERIFY_NRF_ERROR_CODE(app_timer_stop(timebaseTimerId));
        //VERIFY_NRF_ERROR_CODE(app_timer_start(timebaseTimerId, TICKSTIMEBASESHORT, NULL));
        VERIFY_MPU_ERROR_CODE(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL));
        mpuEnabled = true;
    }

    // check if light has to be turned off
    //if (timebase == TIMEBASESHORT && pMode->mode == LIGHT_MODEOFF)
    if (mpuEnabled && pMode->mode == LIGHT_MODEOFF)
    {
        uint8_t target[2] = {0,0};
        //timebase = TIMEBASELONG;
        //VERIFY_NRF_ERROR_CODE(app_timer_stop(timebaseTimerId));
        //VERIFY_NRF_ERROR_CODE(app_timer_start(timebaseTimerId, TICKSTIMEBASELONG, NULL));
        VERIFY_NRF_ERROR_CODE(mpu_set_sensors(0));
        mpuEnabled = false;
        VERIFY_NRF_ERROR_CODE(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TARGETSDL, 2, target));
    }

    // cyclic checks (1Hz in off mode, 100Hz in on mode)
    //if (timebaseFlag)
    //{
        enum targetId {FLOOD = 0, SPOT, CNT};

        short gyro[3], accel[3], sensors;
        long quat[4];
        unsigned long timestamp;
        unsigned char more;
        static float32_t pitch;
        uint8_t target[CNT];

//        timebaseFlag = 0;                           /**< clear timebaseflag */
//        pwr_ClearActiveFlag(1<<pwr_ACTIVELIGHT);    /**< and release anti sleep flag */
        adc_StartConversion();                      /**< start conversion for input voltage */
        VERIFY_NRF_ERROR_CODE(readConverterData(pStatus));/**< read current converter status and data */

        //if (timebase == TIMEBASESHORT)              /**< read motion sensor data, if light is on */
        if (mpuEnabled)
        {
            sensors = INV_WXYZ_QUAT;
            do                                      /**< MPU is sampling data with 100Hz, too, but due to unsynchronized */
            {                                       /**< clocks, data is read out until fifo is empty. */
                int error = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
                if (error && error != -3)           /**< ignore unaligned data error (can happen if data is read, */
                {                                   /**< while mpu is writing data into fifo */
                    VERIFY_MPU_ERROR_CODE(error);
                }
            }
            while (more != 0);
            if (sensors != 0)                       /**< calculate pitch if data is available */
                pitch = quatToPitch(quat);
        }
        else
            pitch = -M_PI_2;

        while (adc_ConversionComplete() == adc_CONVERSIONONGOING);  /**< adc conversion should be done until now, if not wait. */
        pStatus->inputVoltage = RESULT_IN_MILLIVOLT(adc_GetVoltage());

        if (pMode->mode == LIGHT_MODEOFF)                               /**< return if leds are off */
            return NRF_SUCCESS;

        calculateTarget(pMode, pitch, &target[FLOOD], &target[SPOT]);/**< now calculate target currents */

        applyLimiter(pStatus, &target[FLOOD], &target[SPOT]);       /**< limit target values if necessary */

        VERIFY_NRF_ERROR_CODE(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TARGETSDL, 2, target)); /**< and finally write targets to driver */
    //}
    return NRF_SUCCESS;
}
#endif
/**END OF FILE*****************************************************************/
