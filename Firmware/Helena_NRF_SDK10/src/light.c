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
#include "power.h"
#include "i2c.h"
#include "fds.h"
#include "nrf_delay.h"
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
#define FLOOD               0
#define SPOT                1

#define TIMEBASE_ON         (APP_TIMER_TICKS(10,0))
#define TIMEBASE_IDLE       (APP_TIMER_TICKS(1000,0))

#define VOLTAGEMIN          6050    // minimum operation input voltage
#define VOLTAGEMAX          8500    // maximum operation input voltage
#define CURRENTMAX          MILLIAMPERE_IN_TARGETVALUE(3000)
#define TEMPERATUREMAX      3480    // = 75 °C

#define LUTSIZE             256     // size of look up table for pitch<->comp mapping, must be power of two
#define CROSSINGSIZE        16      // size of look up table for crossing between spot and flood, must be power of two
#define PITCHOFFSETSPOT     (155)//((float32_t)0.0296706)
#define PITCHOFFSETFLOOD    (-155)//((float32_t)-0.0296706)

#define FDSDRVCONFIG        0x073D      /**< number identifying led configuration fds data */
#define FDSINSTANCE         0x7167      /**< number identifying fds data for light module */

#define LIGHT_LEDCONFIG_UNKNOWN         UINT8_MAX

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
static light_DriverConfigStruct drvConfig;

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
    q15_t pitchSpot;
    q15_t pitchFlood;
    int32_t pitchCalc;

    // if spot and flood leds are present, add pitch offset due to tilted optics
    if (drvConfig.floodCount != 0 && drvConfig.spotCount != 0)
    {
        pitchCalc = pitch + PITCHOFFSETFLOOD;
        if (pitchCalc >= (1<<15))
            pitchCalc -= (1<<15);
        else if (pitchCalc < 0)
            pitchCalc += (1<<15);
        pitchFlood = (q15_t)pitchCalc;
        pitchCalc = pitch + PITCHOFFSETSPOT;
        if (pitchCalc >= (1<<15))
            pitchCalc -= (1<<15);
        else if (pitchCalc < 0)
            pitchCalc += (1<<15);
        pitchSpot = (q15_t)pitchCalc;
    }
    else
    {
        pitchFlood = pitch;
        pitchSpot = pitch;
    }
    pitchFlood = limitPitch(pitchFlood);
    pitchSpot = limitPitch(pitchSpot);

    switch (pMode->mode)
    {
    case LIGHT_MODEFLOOD:
    case LIGHT_MODEFLOODCLONED:
        if (pMode->intensity >= 100)
            *pFlood = 255;
        else
            *pFlood = ((uint16_t)pMode->intensity << 8) / 100;
        if (pMode->mode == LIGHT_MODEFLOODCLONED)
            *pSpot = *pFlood;
        else
            *pSpot = 0;
        break;
    case LIGHT_MODESPOT:
    case LIGHT_MODESPOTCLONED:
        if (pMode->intensity >= 100)
            *pSpot = 255;
        else
            *pSpot = ((uint16_t)pMode->intensity << 8) / 100;
        if (pMode->mode == LIGHT_MODESPOTCLONED)
            *pFlood = *pSpot;
        else
            *pFlood = 0;
        break;
    case LIGHT_MODEFULL:
        *pFlood = 0;
        if (pMode->intensity >= 100)
        {
            *pSpot = 255;
            *pFlood = 255;
        }
        else
        {
            *pSpot = ((uint16_t)pMode->intensity << 8) / 100;
            *pFlood = *pSpot;
        }
        break;
    case LIGHT_MODEFLOODAPC:
    case LIGHT_MODEFLOODAPCCLONED:
        *pFlood = getTargetFromLUT(&floodLUT, pitchFlood, (q7_8_t)pMode->intensity << 8);
        if (pMode->mode == LIGHT_MODEFLOODAPCCLONED)
            *pSpot = *pFlood;
        else
            *pSpot = 0;
        break;
    case LIGHT_MODESPOTAPC:
    case LIGHT_MODESPOTAPCCLONED:
        *pSpot = getTargetFromLUT(&spotLUT, pitchSpot, (q7_8_t)pMode->intensity << 8);
        if (pMode->mode == LIGHT_MODESPOTAPCCLONED)
            *pFlood = *pSpot;
        else
            *pFlood = 0;
        break;
    case LIGHT_MODEFULLAPC:
    {
        q7_8_t intensityFlood, intensitySpot;
        getIntensitiesFromLUT(limitPitch(pitch), (q7_8_t)pMode->intensity << 8, &intensityFlood, &intensitySpot);
        *pFlood = getTargetFromLUT(&floodLUT, pitchFlood, intensityFlood);
        *pSpot = getTargetFromLUT(&spotLUT, pitchSpot, intensitySpot);
    }   break;
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
static void limiter(light_StatusStruct * pStatus, uint8_t *pFlood, uint8_t *pSpot)
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
    if (pStatus->inputVoltage >= VOLTAGEMAX)                // voltage to high
        limit = 0;
    else if (pStatus->inputVoltage >= (VOLTAGEMIN + 1024 - 8))  // no limiting needed
        limit = 255;
    else if (pStatus->inputVoltage >= VOLTAGEMIN)           // limiting needed
        limit = 2 + ((pStatus->inputVoltage - VOLTAGEMIN) >> 2);
    else                                                    // voltage to low
        limit = 2;

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

/** @brief fds event handler
 *
 * @details this function checks the results of file operations and sends the
 *          appropriate event response to the ble modules light control service
 */
static void fdsEventHandler(ret_code_t errCode, fds_cmd_id_t cmd, fds_record_id_t recordId, fds_record_key_t recordKey)
{
    // check only events of interest
    if (recordKey.instance == FDSINSTANCE && recordKey.type == FDSDRVCONFIG)
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

static void validateLedConfig(light_DriverConfigStruct* pLedConfig)
{
    // if both driver led counts are not known, use default setup
    if (pLedConfig->floodCount == LIGHT_LEDCONFIG_UNKNOWN && pLedConfig->spotCount == LIGHT_LEDCONFIG_UNKNOWN)
    {
        pLedConfig->floodCount = 2;
        pLedConfig->spotCount = 1;
    }
    // if only one driver is used, assume that there are two leds connected
    else if (pLedConfig->floodCount == LIGHT_LEDCONFIG_UNKNOWN && pLedConfig->spotCount == 0)
        pLedConfig->floodCount = 2;
    else if (pLedConfig->floodCount == 0 && pLedConfig->spotCount == LIGHT_LEDCONFIG_UNKNOWN)
        pLedConfig->spotCount = 2;
}

static void readLedConfig()
{
    uint32_t errCode;

    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_key_t key = {.type = FDSDRVCONFIG, .instance = FDSINSTANCE};
    light_DriverConfigStruct* pConfig;

    drvConfig.floodCount = LIGHT_LEDCONFIG_UNKNOWN;
    drvConfig.spotCount = LIGHT_LEDCONFIG_UNKNOWN;

    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode == NRF_SUCCESS)
    {
        fds_record_t record;
        errCode = fds_open(&descriptor, &record);
        if (errCode == NRF_SUCCESS)
        {
            pConfig = (light_DriverConfigStruct*)record.p_data;
            drvConfig.floodCount = pConfig->floodCount;
            drvConfig.spotCount = pConfig->spotCount;
        }
        else
            APP_ERROR_HANDLER(errCode);
    }
    else
    {
        if (errCode != NRF_ERROR_NOT_FOUND)
            APP_ERROR_HANDLER(errCode);
    }

    // set default values if led count isn't known
    validateLedConfig(&drvConfig);
}

static void writeLedConfig()
{
    uint32_t errCode;
    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_chunk_t chunk;

    fds_record_key_t key = {.type = FDSDRVCONFIG, .instance = FDSINSTANCE};
    chunk.p_data = &drvConfig;
    chunk.length_words = SIZEOF_WORDS(drvConfig);
    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode == NRF_SUCCESS)
    {
        APP_ERROR_CHECK(fds_update(&descriptor, key, 1, &chunk));
    }
    else if (errCode == NRF_ERROR_NOT_FOUND)
    {
        APP_ERROR_CHECK(fds_write(&descriptor, key, 1, &chunk));
    }
    else
        APP_ERROR_HANDLER(errCode);
}

static light_DriverRevisionEnum driverRevCheck()
{
    uint32_t errCode;
    uint8_t buffer[4];

    // read first two registers to have compare values
    errCode = i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, 2, &buffer[0]);
    if(errCode != NRF_SUCCESS)
    {
        APP_ERROR_HANDLER(errCode);
        return LIGHT_DRIVERREVUNKNOWN;
    }
    // now read duty cycle registers, on rev 1.0 drivers this will result in a read operation of reg 0 and 1
    errCode = i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, 2, &buffer[2]);
    if(errCode != NRF_SUCCESS)
    {
        APP_ERROR_HANDLER(errCode);
        return LIGHT_DRIVERREVUNKNOWN;
    }
    if (buffer[0] == buffer[2] && buffer[1] == buffer[3])
        return LIGHT_DRIVERREV10;
    return LIGHT_DRIVERREV11;
}

/* Public functions ----------------------------------------------------------*/
uint32_t light_Init(light_DriverConfigStruct * pLedConfig)
{
    // create timer for timebase
    VERIFY_NRF_ERROR_CODE(app_timer_create(&updateTimerId, APP_TIMER_MODE_REPEATED, updateTimerHandler));

    // initialize TWI interface
    VERIFY_NRF_ERROR_CODE(i2c_Init());

    // enable workaround for TWI lock up in tev2 ICs
    i2c_EnableAutoRecover(true);

    lightState = STATEOFF;

    // check driver revision;
    drvConfig.rev = driverRevCheck();

    // register to fds module
    VERIFY_NRF_ERROR_CODE(fds_register(fdsEventHandler));
    VERIFY_NRF_ERROR_CODE(fds_init());

    // read led configuration
    readLedConfig();
    *pLedConfig = drvConfig;

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
    if (pMode->mode == LIGHT_MODEOFF ||
        pMode->mode == LIGHT_MODESPOT || pMode->mode == LIGHT_MODESPOTAPC ||
        pMode->mode == LIGHT_MODESPOTCLONED || pMode->mode == LIGHT_MODESPOTAPCCLONED)
        lightStatus.flood = 0;
    if (pMode->mode == LIGHT_MODEOFF ||
        pMode->mode == LIGHT_MODEFLOOD || pMode->mode == LIGHT_MODEFLOODAPC ||
        pMode->mode == LIGHT_MODEFLOODCLONED || pMode->mode == LIGHT_MODEFLOODAPCCLONED)
        lightStatus.spot = 0;
    *ppStatus = &lightStatus;

    return NRF_SUCCESS;
}

void light_Execute()
{
#define LOWPASSTHRESH   100     // threshold in mV, when low pass filter is bypassed
#define sign(x)     (x < 0 ? -1 : 1)
    static uint32_t voltageFiltered;

    if (!updateFlag)
        return;

    updateFlag = false;
    pwr_ClearActiveFlag(1<<pwr_ACTIVELIGHT);

    // check if input voltage value is up to date
    pwr_VoltageStruct voltage;
    if (pwr_GetInputVoltage(&voltage) == NRF_SUCCESS)   // if this fails, conversion is already in progress, no need to check
    {
        uint32_t timestamp;
        (void)app_timer_cnt_get(&timestamp);
        (void)app_timer_cnt_diff_compute(timestamp, voltage.timestamp, &timestamp);
        if (timestamp > TIMEBASE_ON)                    // value to old, get new one
            pwr_StartInputVoltageConversion();
    }

    APP_ERROR_CHECK(readConverterData(&lightStatus));                        // read current converter status and data

    while (pwr_GetInputVoltage(&voltage) != NRF_SUCCESS);                    // wait for conversion to be finished

    int_fast16_t diff = (voltageFiltered >> 4) - (int_fast16_t)voltage.inputVoltage;
    for (uint_fast8_t i = 0; i < 4 ; i++)
    {
        if (abs(diff) < (32 << i) || i == 3)
        {
            voltageFiltered -= (voltageFiltered >> (4 - i));
            voltageFiltered += (voltage.inputVoltage + sign(diff) * ((int16_t)16 << i)) << i;
            break;
        }
    }
    lightStatus.inputVoltage = voltageFiltered >> 4;

    /*if (voltageFiltered == 0)                                                // in case of filter is empty or
        voltageFiltered = voltage.inputVoltage << 4;                         // use input directly
    else if ((voltageFiltered >> 4) > (voltage.inputVoltage + LOWPASSTHRESH))// massive input voltage drop
        voltageFiltered = (voltage.inputVoltage + LOWPASSTHRESH) << 4;       // bypass filter
    else if ((voltageFiltered >> 4) < (voltage.inputVoltage - LOWPASSTHRESH))// massive input voltage increase
        voltageFiltered = (voltage.inputVoltage - LOWPASSTHRESH) << 4;       // bypass filter
    else                                                                     // otherwise use a low pass filter for input voltage
        voltageFiltered = voltageFiltered - (voltageFiltered >> 4) + voltage.inputVoltage;
    lightStatus.inputVoltage = voltageFiltered >> 4;*/

    if (targets[FLOOD] == 0 && targets[SPOT] == 0)              // if light is off, nothing more to do
        return;

    uint8_t limitedTargets[2];

    limitedTargets[FLOOD] = targets[FLOOD];
    limitedTargets[SPOT] = targets[SPOT];
    limiter(&lightStatus, &limitedTargets[FLOOD], &limitedTargets[SPOT]);

    APP_ERROR_CHECK(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TARGETSDL, 2, limitedTargets));
}

uint32_t light_CheckLedConfig(light_DriverConfigStruct* pLedConfig)
{
    uint8_t buffer[3];
    stateEnum oldState;
    uint32_t errCode;

    if (lightState == STATEON)
        return NRF_ERROR_INVALID_STATE;

    // save old state and reset old configuration
    oldState = lightState;
    drvConfig.floodCount = LIGHT_LEDCONFIG_UNKNOWN;
    drvConfig.spotCount = LIGHT_LEDCONFIG_UNKNOWN;

    // turn on led driver with 2*2/3 output current
    buffer[0] = (HELENABASE_SLEEP_DISABLE<<HELENABASE_CRA_SLEEP_OFFSET) | (HELENABASE_ADCRATE_1S<<HELENABASE_CRA_ADCRATE_OFFSET);
    buffer[1] = 171;
    buffer[2] = 171;
    VERIFY_NRF_ERROR_CODE(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, 3, buffer));

    // now wait for ~100ms
    nrf_delay_ms(100);

    // read Converter Data
    errCode = readConverterData(&lightStatus);
    if (errCode == NRF_SUCCESS)
    {
        // check if current is available, if not there is no led connected
        if (lightStatus.currentFlood < 500)
            drvConfig.floodCount = 0;
        if (lightStatus.currentSpot < 500)
            drvConfig.spotCount = 0;

        // read duty cycle registers
        if (drvConfig.rev == LIGHT_DRIVERREV11)
        {
            errCode = i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_DUTYCYCLELEFT, 2, buffer);
            if(errCode == NRF_SUCCESS)
            {
                if (drvConfig.floodCount)
                    drvConfig.floodCount = buffer[0] < (HELENABASE_DC_MAX * 0.75) ? 1 : 2;
                if (drvConfig.spotCount)
                    drvConfig.spotCount = buffer[1] < (HELENABASE_DC_MAX * 0.75) ? 1 : 2;
            }
        }
    }

    // set default values if led count isn't known
    validateLedConfig(&drvConfig);

    pLedConfig->floodCount = drvConfig.floodCount;
    pLedConfig->spotCount = drvConfig.spotCount;

    // save new state
    writeLedConfig();

    // turn of driver
    if (oldState == STATEOFF)
        buffer[0] = (HELENABASE_SLEEP_DISABLE<<HELENABASE_CRA_SLEEP_OFFSET) | (HELENABASE_ADCRATE_1S<<HELENABASE_CRA_ADCRATE_OFFSET);
    else
        buffer[0] = (HELENABASE_SLEEP_ENABLE<<HELENABASE_CRA_SLEEP_OFFSET) | (HELENABASE_ADCRATE_1S<<HELENABASE_CRA_ADCRATE_OFFSET);
    buffer[1] = 0;
    buffer[2] = 0;
    VERIFY_NRF_ERROR_CODE(i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, 3, buffer));

    return errCode;
}

/**END OF FILE*****************************************************************/
