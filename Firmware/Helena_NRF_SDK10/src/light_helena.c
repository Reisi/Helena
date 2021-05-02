/**
  ******************************************************************************
  * @file    light.c
  * @author  Thomas Reisnecker
  * @brief   light management module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Helena_base.h"
#include "helena_base_driver.h"
#include "light_helena.h"
#include "app_timer.h"
#include "power.h"
#include "i2c.h"
#include "fds.h"
#include "nrf_delay.h"
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
//#define FLOOD                   0
//#define SPOT                    1

#define TIMEBASE_ON             (APP_TIMER_TICKS(10,0))     // timebase in on mode
#define TIMEBASE_IDLE           (APP_TIMER_TICKS(1000,0))   // timebase in idle mode

#define VOLTAGEMIN              ((3025ul << 10) / 1000)     // q10_4_t representation of minimal cell voltage
#define VOLTAGEMAX              ((4250ul << 10) / 1000)     // q10_4_t representation of maximal cell voltage

#define NUM_OF_CELL_CONFIGS     3                           // supporting 1, 2 or 3 cells in series
#define NOM_CELL_VOLTAGE        ((3700ul << 10) / 1000)     // q10_4_t representation of nominal cell voltage
#define NOM_LED_VOLTAGE         ((3000ul << 10) / 1000)     // q10_4_t representation of nominal led voltage

#define OUTPUTMAX1CELL          UINT8_MAX                   // max. output (= 3A with correct calibrated driver)
#define OUTPUTMAX2CELL          UINT8_MAX                   // max. output (= 3A with correct calibrated driver
#define OUTPUTMAX3CELL          ((UINT8_MAX * 24) / 30)     // for 3cell out has to be limited to 2.4A
#define OUTPUT_DEFAULT_LIMT     ((UINT8_MAX * 85) / 100)    // default configurable current limit is set to 85%

#define TEMPERATUREMAX          ((75 + 273) << 4)           // 75 °C in q12_4_t K

#define LUTSIZE                 256     // size of look up table for pitch<->comp mapping, must be power of two
#define CROSSINGSIZE            16      // size of look up table for crossing between spot and flood, must be power of two
#define PITCHOFFSETSPOT         155     // 1.7° in q15_t
#define PITCHOFFSETFLOOD        -155    // -1.7° in q15_t

#define FDSDRVCONFIG            0x073D  // number identifying led configuration fds data
#define FDSINSTANCE             0x7167  // number identifying fds data for light module

#define LIGHT_LEDCONFIG_UNKNOWN UINT8_MAX

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    STATEOFF = 0,   // LED driver is shutdown
    STATEIDLE,      // LED driver in idle mode, no leds, but temperature measurement once per second
    STATEON         // at least on led is on, processing at 10msec timebase
} state_t;

typedef struct
{
    q15_t pitch[LUTSIZE];
    q7_8_t comp[LUTSIZE];
} compensationLUT_t;

typedef struct
{
    q15_t pitch[CROSSINGSIZE + 1];
    q15_t partSpot[CROSSINGSIZE + 1];
    q15_t partFlood[CROSSINGSIZE + 1];
} crossingLUT_t;

typedef struct
{
    q8_t flood;
    q8_t spot;
} target_t;

typedef struct
{
    light_driverConfig_t drvConfig;
    target_t currentLimit;
} storageData_t;

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

#define SIZE_IN_WORDS(x)        ((sizeof(x) + 3) / 4)
#define sign(x)                 (x < 0 ? -1 : 1)

/* Private variables ---------------------------------------------------------*/
static const compensationLUT_t spotLUT =
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
static const compensationLUT_t floodLUT =
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
static const crossingLUT_t crossingLUT =
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

static state_t lightState;              // current light state
static light_status_t lightStatus;      // current light status informations
static uint8_t cellCnt;                 // number of in series connected li-ion cells of supply battery
static target_t target, currentLimit;   // current target and limit values
static volatile uint16_t prescale;
APP_TIMER_DEF(updateTimerId);           // timer for timebase
static storageData_t storage;           // information stored into flash
static volatile bool flashInitialized;  // indicating if the flash data storage module is initialized

/* Private functions ---------------------------------------------------------*/
/** @brief handler for timebase timer event
 * */
static void updateTimerHandler(void *pContext)
{
    (void)pContext;
    prescale++;
    pwr_SetActiveFlag(pwr_ACTIVELIGHT);
}

/** @brief function to filter temperature
 *
 * @details   This is a simple 1st order low pass filter with a time constant
 *            of 2.56 sec (at 100 SPS).
 *
 * @param[in] unfiltered    unfiltered temperature
 * @return    the filtered temperature *
 */
static q12_4_t temperatureFilter(q12_4_t unfiltered)
{
    /// TODO: adapt time constant to 1sec update interval
    static q20_12_t filtered;

    if (filtered == 0 || lightState != STATEON)  // don't filter at startup, or if light is off
        filtered = (q20_12_t)unfiltered << 8;
    else
    {
        filtered -= filtered >> 8;
        filtered += unfiltered;
    }

    return filtered >> 8;
}

/** @brief function to filter voltage
 *
 * @details   This is a 1st order low pass filter with an adaptive time
 *            constant (at 100SPS) of 160 (V_diff < 31.25mV), 80 (V_diff <
 *            62.5mV), 40  (V_diff < 125mV) or 20ms  (V_diff >= 125mV).
 *
 * @param[in] unfiltered    unfiltered temperature
 * @return    the filtered temperature *
 */
static q6_10_t voltageFilter(q6_10_t unfiltered)
{
    static q18_14_t filtered;

    // filter input voltage, a dynamic filter is used with high time constant
    // for small changes and low time constant for big changes. This way the
    // limiter does not produce flickering output if the supply voltage drops
    // due to draining battery, but still reacts fast enough on big voltage
    // drops in adaptive mode.
    /// to do: filter and limiter works fine in with 2 cells, but with 1 and 3 cells?
    if (filtered == 0 || lightState != STATEON) // don't filter at startup, or if light is off
        filtered = (q18_14_t)unfiltered << 4;
    else
    {
        int_fast16_t diff = (filtered >> 4) - (int_fast16_t)unfiltered;
        for (uint_fast8_t i = 0; i < 4 ; i++)
        {
            if (abs(diff) < (32 << i) || i == 3)
            {
                filtered -= (filtered >> (4 - i));
                filtered += (unfiltered + sign(diff) * ((int16_t)16 << i)) << i;
                break;
            }
        }
    }

    return filtered >> 4;
}

/** @brief function to read data of i2c Step down converter
 *
 * @param[out]  pStatus converter status data
 * @return      NRF_SUCCESS or error code *
 */
static uint32_t readConverterData(light_status_t* pStatus)
{
    hbd_retVal_t errCode;
    hbd_samplingData_t data;

    errCode = hbd_ReadSamplingData(&data);
    if (errCode == HBD_SUCCESS)
    {
        pStatus->flood.dutycycle = data.currentLeft.maxDC || data.currentLeft.minDC;
        pStatus->spot.dutycycle = data.currentRight.maxDC || data.currentRight.minDC;
        pStatus->currentFlood = data.currentLeft.current;
        pStatus->currentSpot = data.currentRight.current;
        pStatus->temperature = temperatureFilter(data.temperature);
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
static q8_t getTargetFromLUT(const compensationLUT_t* pLut, q15_t pitch, q7_8_t intensity)
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
static void calculateTarget(const light_mode_t *pMode, q15_t pitch, uint8_t *pFlood, uint8_t *pSpot)
{
    q15_t pitchSpot;
    q15_t pitchFlood;
    int32_t pitchCalc;
    q7_8_t illuminanceInLux;

    // if spot and flood leds are present, add pitch offset due to tilted optics
    if (storage.drvConfig.floodCount != 0  && storage.drvConfig.floodCount !=  LIGHT_LEDCONFIG_UNKNOWN &&
        storage.drvConfig.spotCount != 0 && storage.drvConfig.spotCount != LIGHT_LEDCONFIG_UNKNOWN)
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

    // start by setting targets to zero
    *pFlood = 0;
    *pSpot = 0;

    // if pitch compensation is off, intensity is target value
    if (!pMode->setup.pitchCompensation)
    {
        if (pMode->setup.flood || (pMode->setup.spot && pMode->setup.cloned))
            *pFlood = pMode->intensity;
        if (pMode->setup.spot || (pMode->setup.flood && pMode->setup.cloned))
            *pSpot = pMode->intensity;
    }
    else //if (pMode->setup.pitchCompensation)
    {
        illuminanceInLux = pMode->illuminanceInLux > INT8_MAX ? INT8_MAX : pMode->illuminanceInLux;
        illuminanceInLux <<= 8;
        if (pMode->setup.flood && pMode->setup.spot)
        {
            q7_8_t intensityFlood, intensitySpot;
            getIntensitiesFromLUT(limitPitch(pitch), illuminanceInLux, &intensityFlood, &intensitySpot);
            if (storage.drvConfig.floodCount == 1)  // flood intensity is calculated for XHP50, double output if only one led is available
                intensityFlood = intensityFlood >= (1 << 14) ? INT16_MAX : intensityFlood << 1;
            if (storage.drvConfig.spotCount == 2)   // spot intensity is calculated for a single XM-L, halve output if two leds are connected
                intensitySpot >>= 1;
            *pFlood = getTargetFromLUT(&floodLUT, pitchFlood, intensityFlood);
            *pSpot = getTargetFromLUT(&spotLUT, pitchSpot, intensitySpot);
        }
        else
        {
            if (pMode->setup.flood)
            {
                if (storage.drvConfig.floodCount == 1)  // flood intensity is calculated for XHP50, double output if only one led is available
                    illuminanceInLux = illuminanceInLux >= (1 <<14) ? INT16_MAX : illuminanceInLux << 1;
                *pFlood = getTargetFromLUT(&floodLUT, pitchFlood, illuminanceInLux);
                if (pMode->setup.cloned)
                    *pSpot = *pFlood;
            }
            if (pMode->setup.spot)
            {
                if (storage.drvConfig.spotCount == 2)   // spot intensity is calculated for a single XM-L, halve output if two leds are connected
                    illuminanceInLux >>= 1;
                *pSpot = getTargetFromLUT(&spotLUT, pitchSpot, illuminanceInLux);
                if (pMode->setup.cloned)
                    *pFlood = *pSpot;
            }
        }
    }

    // if sos mode is selected, the output is reset to zero using the prescaler counter to generate sos blinking
    if (pMode->setup.sos)
    {
        uint_fast16_t prescl = prescale;

        //   __    __    __    ________    ________    ________    __    __    __      |<----- cut? ---->
        //__|  |__|  |__|  |__|        |__|        |__|        |__|  |__|  |__|  |_______________________
        // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31

        prescl >>= 5;       // prescaler is increased each 10msec, this generates a ticklenght of 0.64sec
        prescl &= 0x001F;   // limit to 0..31

        //   clear >= 24      enable even,     enable 8,      enable 12 and   enable 16
        if (prescl >= 24 || !(prescl & 1 || prescl == 8 || prescl == 12 || prescl == 16))
        {
            *pFlood = 0;
            *pSpot = 0;
        }
    }
}

/** @brief limiter function
 *
 * @param[in/out]   pStatus light status
 * @param[in/out]   pFlood  flood current
 * @param[in/out]   pSpot   spot current
 */
static void limiter(light_status_t * pStatus, uint8_t *pFlood, uint8_t *pSpot)
{
    uint8_t voltageLimit, temperatureLimit;

    // calculate voltage limit
    if (cellCnt == 0)
        voltageLimit = 0;
    else
    {
        uint16_t normVoltage = pStatus->inputVoltage / cellCnt; // normalize input voltage to 1 cell
        if (normVoltage >= VOLTAGEMAX)                          // voltage to high
            voltageLimit = 0;
        else if (normVoltage >= (VOLTAGEMIN + 126 + 127*2))     // above limiting range
            voltageLimit = 255;
        else if (normVoltage >= (VOLTAGEMIN + 126))             // within flat limiting range
            voltageLimit = 128 + ((normVoltage - (VOLTAGEMIN + 126)) >> 1);
        else if (normVoltage >= VOLTAGEMIN)                     // within steep limiting range
            voltageLimit = 2 + (normVoltage - VOLTAGEMIN);
        else                                                    // voltage to low
            voltageLimit = 2;
    }

    // calculate temperature limit
    if (pStatus->temperature > TEMPERATUREMAX)
        temperatureLimit = 0;                                      // temperature is to high, shut of
    else if (pStatus->temperature <= TEMPERATUREMAX - 256)
        temperatureLimit = 255;                                    // no limitation necessary
    else
        temperatureLimit = TEMPERATUREMAX - pStatus->temperature;  // limiting

    // limit flood
    pStatus->flood.current = false;         // clear all flags first
    pStatus->flood.voltage = false;
    pStatus->flood.temperature = false;
    if (*pFlood > currentLimit.flood)
    {
        *pFlood = currentLimit.flood;
        pStatus->flood.current = true;
    }
    if (*pFlood > voltageLimit)
    {
        *pFlood = voltageLimit;
        pStatus->flood.current = false;     // clear flag in case it has been set before
        pStatus->flood.voltage = true;
    }
    if (*pFlood > temperatureLimit)
    {
        *pFlood = temperatureLimit;
        pStatus->flood.current = false;     // clear flag in case it has been set before
        pStatus->flood.voltage = false;     // clear flag in case it has been set before
        pStatus->flood.temperature = true;
    }

    // limit spot
    pStatus->spot.current = false;
    pStatus->spot.voltage = false;
    pStatus->spot.temperature = false;
    if (*pSpot > currentLimit.spot)
    {
        *pSpot = currentLimit.spot;
        pStatus->spot.current = true;
    }
    if (*pSpot > voltageLimit)
    {
        *pSpot = voltageLimit;
        pStatus->spot.current = false;
        pStatus->spot.voltage = true;
    }
    if (*pSpot > temperatureLimit)
    {
        *pSpot = temperatureLimit;
        pStatus->spot.current = false;
        pStatus->spot.voltage = false;
        pStatus->spot.temperature = true;
    }
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

    if (cmd == FDS_CMD_INIT)
        flashInitialized = true;
}

static void validateCurrentLimit(target_t* pLimits)
{
    q8_t designLimit;

    designLimit = cellCnt == 1 ? OUTPUTMAX1CELL :
                  cellCnt == 2 ? OUTPUTMAX2CELL :
                  cellCnt == 3 ? OUTPUTMAX3CELL : 0;

    if (pLimits->flood > designLimit)
        pLimits->flood = designLimit;
    currentLimit.flood = pLimits->flood;

    if (pLimits->spot > designLimit)
        pLimits->spot = designLimit;
    currentLimit.spot = pLimits->spot;
}

static void readStorage()
{
    uint32_t errCode;

    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_key_t key = {.type = FDSDRVCONFIG, .instance = FDSINSTANCE};
    storageData_t* pData;

    // set default values
    storage.drvConfig.floodCount = LIGHT_LEDCONFIG_UNKNOWN;
    storage.drvConfig.spotCount = LIGHT_LEDCONFIG_UNKNOWN;
    storage.currentLimit.flood = OUTPUT_DEFAULT_LIMT;
    storage.currentLimit.spot = OUTPUT_DEFAULT_LIMT;

    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode == NRF_SUCCESS)
    {
        fds_record_t record;
        errCode = fds_open(&descriptor, &record);
        if (errCode == NRF_SUCCESS)
        {
            pData = (storageData_t*)record.p_data;
            storage.drvConfig.floodCount = pData->drvConfig.floodCount;
            storage.drvConfig.spotCount = pData->drvConfig.spotCount;
            // limits has been added supplementary, check size to be sure to not read flimflam from old data
            if (record.header.tl.length_words == SIZE_IN_WORDS(storage))
                storage.currentLimit = pData->currentLimit;
            APP_ERROR_CHECK(fds_close(&descriptor));
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
    //validateLedConfig(&storage.drvConfig);
    validateCurrentLimit(&storage.currentLimit);
}

static void writeStorage()
{
    uint32_t errCode;
    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_chunk_t chunk;

    fds_record_key_t key = {.type = FDSDRVCONFIG, .instance = FDSINSTANCE};
    chunk.p_data = &storage;
    chunk.length_words = SIZE_IN_WORDS(storage);
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

/* Public functions ----------------------------------------------------------*/
uint32_t light_Init(uint8_t supplyCellCnt, light_driverConfig_t* pLedConfig)
{
    nrf_delay_ms(250);  // wait 250ms to let the driver leave the bootloader

    // create timer for timebase
    VERIFY_NRF_ERROR_CODE(app_timer_create(&updateTimerId, APP_TIMER_MODE_REPEATED, updateTimerHandler));

    // initialize TWI interface
    VERIFY_NRF_ERROR_CODE(i2c_Init());

    // enable workaround for TWI lock up in rev2 ICs
    i2c_EnableAutoRecover(true);

    // save cell count
    cellCnt = supplyCellCnt;

    lightState = STATEOFF;

    // register to fds module
    VERIFY_NRF_ERROR_CODE(fds_register(fdsEventHandler));
    VERIFY_NRF_ERROR_CODE(fds_init());
    while (!flashInitialized) {}    // wait for fds

    // read led configuration
    readStorage();

    // check driver revision;
    hbd_firmwareRev_t fwRev;
    VERIFY_NRF_ERROR_CODE(hbd_GetFirmwareRev(&fwRev));
    storage.drvConfig.rev = (light_driverRevision_t)fwRev;
    //storage.drvConfig.rev = driverRevCheck();

    *pLedConfig = storage.drvConfig;

    return NRF_SUCCESS;
}

uint32_t light_Enable(bool enable)
{
    hbd_config_t cfg;

    if (enable != (lightState == STATEOFF))
        return NRF_ERROR_INVALID_STATE;

    if (enable)
    {
        cfg.sleepMode = HBD_SLEEP_MODE_OFF;
        cfg.sampleRate = HBD_SAMPLERATE_1SPS;
        VERIFY_NRF_ERROR_CODE(hbd_SetConfig(&cfg));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_IDLE, NULL));
        lightState = STATEIDLE;
    }
    else
    {
        cfg.sleepMode = HBD_SLEEP_MODE_ON;
        cfg.sampleRate = HBD_SAMPLERATE_1SPS;
        VERIFY_NRF_ERROR_CODE(hbd_SetConfig(&cfg));
        VERIFY_NRF_ERROR_CODE(hbd_SetTargetCurrent(0, 0));
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        lightState = STATEOFF;
    }

    return NRF_SUCCESS;
}

uint32_t light_UpdateTargets(const light_mode_t* pMode, q15_t pitch, const light_status_t* *ppStatus)
{
    if (pMode == NULL || ppStatus == NULL)
        return NRF_ERROR_NULL;

    if (lightState == STATEOFF)
        return NRF_ERROR_INVALID_STATE;

    // check if timer has to be changed
    if ((pMode->setup.flood || pMode->setup.spot || pMode->setup.sos) && lightState == STATEIDLE)
    {
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_ON, NULL));
        lightState = STATEON;
    }
    else if ((!pMode->setup.flood && !pMode->setup.spot && !pMode->setup.sos) && lightState == STATEON)
    {
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_IDLE, NULL));
        lightState = STATEIDLE;
        target.flood = 0;
        target.spot = 0;
        VERIFY_NRF_ERROR_CODE(hbd_SetTargetCurrent(target.flood, target.spot));
    }

    if (pMode->setup.flood || pMode->setup.spot || pMode->setup.sos)
        calculateTarget(pMode, pitch, &target.flood, &target.spot);

    // clear error flags if leds are off
    if (!pMode->setup.flood)
    {
        lightStatus.flood.current = false;
        lightStatus.flood.voltage = false;
        lightStatus.flood.temperature = false;
        lightStatus.flood.dutycycle = false;
    }
    if (!pMode->setup.spot)
    {
        lightStatus.spot.current = false;
        lightStatus.spot.voltage = false;
        lightStatus.spot.temperature = false;
        lightStatus.spot.dutycycle = false;
    }
    *ppStatus = &lightStatus;

    return NRF_SUCCESS;
}

void light_Execute()
{
    static uint16_t lastPrescale;
    uint16_t currentPrescale = prescale;

    if (lastPrescale == currentPrescale)
        return;

    lastPrescale = currentPrescale;
    pwr_ClearActiveFlag(pwr_ACTIVELIGHT);

    // check if input voltage value is up to date
    pwr_inputVoltage_t voltage;
    if (pwr_GetInputVoltage(&voltage) == NRF_SUCCESS)       // if this fails, conversion is already in progress, no need to check
    {
        uint32_t timestamp;
        (void)app_timer_cnt_get(&timestamp);
        (void)app_timer_cnt_diff_compute(timestamp, voltage.timestamp, &timestamp);
        if (timestamp > TIMEBASE_ON)                        // value to old, get new one
            pwr_StartInputVoltageConversion();
    }

    APP_ERROR_CHECK(readConverterData(&lightStatus));       // read current converter status and data while conversion is in progress

    while (pwr_GetInputVoltage(&voltage) != NRF_SUCCESS);   // conversion should be finished, if not wait

    lightStatus.inputVoltage = voltageFilter(voltage.inputVoltage);

    target_t limitedTarget;
    limitedTarget.flood = target.flood;                 // run targets through limiter
    limitedTarget.spot = target.spot;
    limiter(&lightStatus, &limitedTarget.flood, &limitedTarget.spot);
                                                            // and finally send to driver
    APP_ERROR_CHECK(hbd_SetTargetCurrent(limitedTarget.flood, limitedTarget.spot));
}

uint32_t light_CheckLedConfig(light_driverConfig_t* pLedConfig)
{
    hbd_config_t cfg;
    state_t oldState;
    uint32_t errCode;

    if (lightState == STATEON)
        return NRF_ERROR_INVALID_STATE;

    // save old state and reset old configuration
    oldState = lightState;
    storage.drvConfig.floodCount = LIGHT_LEDCONFIG_UNKNOWN;
    storage.drvConfig.spotCount = LIGHT_LEDCONFIG_UNKNOWN;

    // turn on led driver with 2*2/3 output current
    cfg.sleepMode = HBD_SLEEP_MODE_OFF;
    cfg.sampleRate = HBD_SAMPLERATE_1SPS;
    VERIFY_NRF_ERROR_CODE(hbd_SetConfig(&cfg));
    VERIFY_NRF_ERROR_CODE(hbd_SetTargetCurrent(171, 171));

    // now wait for ~100ms
    nrf_delay_ms(100);

    // read Converter Data
    errCode = readConverterData(&lightStatus);
    if (errCode == NRF_SUCCESS)
    {
        // check if current is available, if not there is no led connected
        /// TODO: current on unconnected is not zero, check driver hardware/firmware
        if (lightStatus.currentFlood < 500)
            storage.drvConfig.floodCount = 0;
        if (lightStatus.currentSpot < 500)
            storage.drvConfig.spotCount = 0;

        // read duty cycle registers
        if (storage.drvConfig.rev == LIGHT_DRIVERREV11 || storage.drvConfig.rev == LIGHT_DRIVERREV12)
        {
            target_t dutyCycle;
            errCode = hbd_ReadDutyCycles(&dutyCycle.flood, &dutyCycle.spot);
            if(errCode == NRF_SUCCESS)
            {
                // In theory the output voltage in continuous current mode is V_out = V_in * DC.
                // Due to losses the real output voltage is lower, therefore by ignoring the
                // efficiency the calculated output voltage should be high enough to not deliver
                // a to low count, but not high enough to deliver a to high count.

                uint32_t ledVoltage;

                if (storage.drvConfig.floodCount)
                {
                    ledVoltage = cellCnt * NOM_CELL_VOLTAGE;
                    ledVoltage *= dutyCycle.flood;
                    ledVoltage /= HBD_DC_MAX;
                    storage.drvConfig.floodCount = ledVoltage / NOM_LED_VOLTAGE;
                }
                if (storage.drvConfig.spotCount)
                {
                    ledVoltage = cellCnt * NOM_CELL_VOLTAGE;
                    ledVoltage *= dutyCycle.spot;
                    ledVoltage /= HBD_DC_MAX;
                    storage.drvConfig.spotCount = ledVoltage / NOM_LED_VOLTAGE;
                }
            }
        }
    }

    pLedConfig->floodCount = storage.drvConfig.floodCount;
    pLedConfig->spotCount = storage.drvConfig.spotCount;

    // save new state
    writeStorage();

    // turn of driver
    if (oldState == STATEOFF)
        cfg.sleepMode = HBD_SLEEP_MODE_ON;
    else
        cfg.sleepMode = HBD_SLEEP_MODE_OFF;
    VERIFY_NRF_ERROR_CODE(hbd_SetConfig(&cfg));
    VERIFY_NRF_ERROR_CODE(hbd_SetTargetCurrent(0, 0));

    return errCode;
}

uint32_t light_GetLimits(q8_t* pFloodLimit, q8_t* pSpotLimit)
{
    if (pFloodLimit == NULL || pSpotLimit == NULL)
        return NRF_ERROR_NULL;

    *pFloodLimit = storage.currentLimit.flood;
    *pSpotLimit = storage.currentLimit.spot;

    return NRF_SUCCESS;
}

uint32_t light_SetLimits(q8_t floodLimit, q8_t spotLimit)
{
    storage.currentLimit.flood = floodLimit;
    storage.currentLimit.spot = spotLimit;

    validateCurrentLimit(&storage.currentLimit);

    writeStorage();

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
