/**
  ******************************************************************************
  * @file    helena_base_driver.c
  * @author  Thomas Reisnecker
  * @brief   driver module for helena base i2c led driver
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "helena_base_driver.h"
//#include "helena_base.h"

/* Private typedef -----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define HELENABASE_ADDRESS              0x3A // this device only has one address
#define HELENABASE_DEFAULT_ADDRESS      0x3A

#define HELENABASE_REGISTERCNT_REV10    9
#define HELENABASE_REGISTERCNT_REV11    11
#define HELENABASE_REGISTERCNT_REV12    16

#define HELENABASE_RA_CONFIG            0x00
#define HELENABASE_RA_TARGETSDL         0x01
#define HELENABASE_RA_TARGETSDR         0x02
#define HELENABASE_RA_STATUSSDL_H       0x03
#define HELENABASE_RA_STATUSSDL_L       0x04
#define HELENABASE_RA_STATUSSDR_H       0x05
#define HELENABASE_RA_STATUSSDR_L       0x06
#define HELENABASE_RA_TEMPERATURE_H     0x07
#define HELENABASE_RA_TEMPERATURE_L     0x08
#define HELENABASE_RA_DUTYCYCLELEFT     0x09    // these registers are only available in v1.1 and greater
#define HELENABASE_RA_DUTYCYCLERIGHT    0x0A
#define HELENABASE_RA_TEMPOFFSET_H      0x0B    // these registers are only available in v1.2 and greater
#define HELENABASE_RA_TEMPOFFSET_L      0x0C
#define HELENABASE_RA_GAINLEFT          0x0D
#define HELENABASE_RA_GAINRIGHT         0x0E
#define HELENABASE_RA_XOR               0x0F

#define HELENABASE_CRA_SLEEP_OFFSET     4
#define HELENABASE_CRA_ADCRATE_OFFSET   0

#define HELENABASE_SLEEP_ENABLE         1
#define HELENABASE_SLEEP_DISABLE        0

#define HELENABASE_ADCRATE_16MS         0
#define HELENABASE_ADCRATE_32MS         1
#define HELENABASE_ADCRATE_64MS         2
#define HELENABASE_ADCRATE_125MS        3
#define HELENABASE_ADCRATE_250MS        4
#define HELENABASE_ADCRATE_500MS        5
#define HELENABASE_ADCRATE_1S           6
#define HELENABASE_ADCRATE_2S           7
#define HELENABASE_ADCRATE_4S           8
#define HELENABASE_ADCRATE_8S           9

#define HELENABASE_ST_STATUS_OFFSET     1
#define HELENABASE_DC_MAX               254

/* Private macros ------------------------------------------------------------*/
#define ISINIT()                        \
do                                      \
{                                       \
    if (firmware == HBD_FWREV_UNKOWN)   \
    {                                   \
        hbd_retVal_t retVal;            \
        retVal = initialize();          \
        if (retVal != HBD_SUCCESS)      \
            return retVal;              \
    }                                   \
} while(0);

#define VERIFY_NOT_NULL(param)          \
do                                      \
{                                       \
    if (param == NULL)                  \
        return HBD_ERROR_NULL;          \
} while(0)

#define VERIFY_REV11()                  \
do                                      \
{                                       \
    if (firmware != HBD_FWREV_11 &&     \
        firmware != HBD_FWREV_12)       \
        return HBD_ERROR_NOT_SUPPORTED; \
} while(0)

#define VERIFY_REV12()                  \
do                                      \
{                                       \
    if (firmware != HBD_FWREV_12)       \
        return HBD_ERROR_NOT_SUPPORTED; \
} while(0)

#define RETVAL_CHECK(retVal)            \
do                                      \
{                                       \
    if (retVal != HBD_SUCCESS)          \
        return retVal;                  \
} while(0)

#define REGISTERVALUE_TO_AMPEREQ10(x)   ((uint16_t)(((x) * 771ul) >> 10))
#define REGISTERVALUE_TO_KELVINQ4(x)     ((uint16_t)((uint32_t)(x) << 2))

/* Private variables ---------------------------------------------------------*/
static uint8_t regMirror[HELENABASE_REGISTERCNT_REV12];
static hbd_firmwareRev_t firmware = HBD_FWREV_UNKOWN;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static hbd_retVal_t initialize()
{
    hbd_retVal_t retVal;

    retVal = hbd_i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, sizeof(regMirror), regMirror);
    RETVAL_CHECK(retVal);

    // compare the non existing register in REV 1.0 with the first ones, if
    // the config and target registers are all zero, this comparison will be
    // true for REV 1.1, too.
    if (memcmp(regMirror, &regMirror[HELENABASE_REGISTERCNT_REV10], HELENABASE_REGISTERCNT_REV12 - HELENABASE_REGISTERCNT_REV10) == 0)
        firmware = HBD_FWREV_10;
    // compare the non existing register in REV 1.1 with the first ones
    else if (memcmp(regMirror, &regMirror[HELENABASE_REGISTERCNT_REV11], HELENABASE_REGISTERCNT_REV12 - HELENABASE_REGISTERCNT_REV11) == 0)
        firmware = HBD_FWREV_11;
    // registers are not the same
    else
        firmware = HBD_FWREV_12;

    return HBD_SUCCESS;
}

static void encodeConfig(hbd_config_t const* pConfig, uint8_t* pReg)
{
    *pReg = (pConfig->sleepMode << HELENABASE_CRA_SLEEP_OFFSET) |
            (pConfig->sampleRate << HELENABASE_CRA_ADCRATE_OFFSET);
}

static void decodeConfig(uint8_t pReg, hbd_config_t* pConfig)
{
    pConfig->sleepMode = pReg >> HELENABASE_CRA_SLEEP_OFFSET;
    pConfig->sampleRate = pReg & ((1 << HELENABASE_CRA_SLEEP_OFFSET) - 1);
}

static void decodeSamplingData(uint8_t const* pReg, hbd_samplingData_t* pData)
{
    uint_fast16_t i;
    // left driver current information
    i = ((uint_fast16_t)pReg[0] << 8) | pReg[1];
    pData->currentLeft.maxDC = i & (1<<13);
    pData->currentLeft.minDC = i & (1<<12);
    pData->currentLeft.current = REGISTERVALUE_TO_AMPEREQ10(i & 0x0FFF);
    // right driver current information
    i = ((uint_fast16_t)pReg[2] << 8) | pReg[3];
    pData->currentRight.maxDC = i & (1<<13);
    pData->currentRight.minDC = i & (1<<12);
    pData->currentRight.current = REGISTERVALUE_TO_AMPEREQ10(i & 0x0FFF);
    // temperature information
    i = ((uint_fast16_t)pReg[4] << 8) | pReg[5];
    pData->temperature = REGISTERVALUE_TO_KELVINQ4(i & 0x0FFF);
}

static void encodeCalibrationData(hbd_calibData_t const* pData, uint8_t* pReg)
{
    pReg[0] = (uint8_t)(pData->temperatureOffset >> 8);
    pReg[4] = pReg[0];
    pReg[1] = (uint8_t)(pData->temperatureOffset & 0x00FF);
    pReg[4] ^= pReg[1];
    pReg[2] = pData->gainLeft;
    pReg[4] ^= pReg[2];
    pReg[3] = pData->gainRight;
    pReg[4] ^= pReg[3];
}

static void decodeCalibrationData(hbd_calibData_t* pData)
{
    pData->temperatureOffset = (q13_2_t)((uint_fast16_t)regMirror[HELENABASE_RA_TEMPOFFSET_H] << 8 | regMirror[HELENABASE_RA_TEMPOFFSET_L]);
    pData->gainLeft = regMirror[HELENABASE_RA_GAINLEFT];
    pData->gainRight = regMirror[HELENABASE_RA_GAINRIGHT];
}

/* Public functions ----------------------------------------------------------*/
hbd_retVal_t hbd_SetConfig(hbd_config_t const* pConfig)
{
    ISINIT();
    VERIFY_NOT_NULL(pConfig);

    uint8_t cfg;
    encodeConfig(pConfig, &cfg);

    if (cfg == regMirror[HELENABASE_RA_CONFIG])
        return HBD_SUCCESS;

    hbd_retVal_t retVal;
    retVal = hbd_i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_CONFIG, 1, &cfg);
    RETVAL_CHECK(retVal);

    regMirror[HELENABASE_RA_CONFIG] = cfg;
    return HBD_SUCCESS;
}

hbd_retVal_t hbd_GetConfig(hbd_config_t* pConfig)
{
    ISINIT();
    VERIFY_NOT_NULL(pConfig);

    decodeConfig(regMirror[HELENABASE_RA_CONFIG], pConfig);
    return HBD_SUCCESS;
}

hbd_retVal_t hbd_SetTargetCurrent(q8_t left, q8_t right)
{
    ISINIT();

    if (regMirror[HELENABASE_RA_TARGETSDL] == left &&
        regMirror[HELENABASE_RA_TARGETSDR] == right)
        return HBD_SUCCESS;

    uint8_t buffer[2] = {left, right};
    hbd_retVal_t retVal;
    retVal = hbd_i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TARGETSDL, sizeof(buffer), buffer);
    RETVAL_CHECK(retVal);

    memcpy(&regMirror[HELENABASE_RA_TARGETSDL], buffer, sizeof(buffer));

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_GetTargetCurrent(q8_t* pLeft, q8_t* pRight)
{
    ISINIT();
    VERIFY_NOT_NULL(pLeft);
    VERIFY_NOT_NULL(pRight);

    *pLeft = regMirror[HELENABASE_RA_TARGETSDL];
    *pRight = regMirror[HELENABASE_RA_TARGETSDR];

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_ReadSamplingData(hbd_samplingData_t* pSamplingData)
{
    ISINIT();
    VERIFY_NOT_NULL(pSamplingData);

    hbd_retVal_t retVal;
    retVal = hbd_i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_STATUSSDL_H, 6, &regMirror[HELENABASE_RA_STATUSSDL_H]);
    RETVAL_CHECK(retVal);

    decodeSamplingData(&regMirror[HELENABASE_RA_STATUSSDL_H], pSamplingData);

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_GetFirmwareRev(hbd_firmwareRev_t* pFWRef)
{
    ISINIT();
    VERIFY_NOT_NULL(pFWRef);

    *pFWRef = firmware;

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_ReadDutyCycles(q8_t* pLeft, q8_t* pRight)
{
    ISINIT();
    VERIFY_NOT_NULL(pLeft);
    VERIFY_NOT_NULL(pRight);
    VERIFY_REV11();

    hbd_retVal_t retVal;
    retVal = hbd_i2c_read(HELENABASE_ADDRESS, HELENABASE_RA_DUTYCYCLELEFT, 2, &regMirror[HELENABASE_RA_DUTYCYCLELEFT]);
    RETVAL_CHECK(retVal);

    *pLeft = regMirror[HELENABASE_RA_DUTYCYCLELEFT];
    *pRight = regMirror[HELENABASE_RA_DUTYCYCLERIGHT];

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_GetCalibrationData_t(hbd_calibData_t* pCalibData)
{
    ISINIT();
    VERIFY_NOT_NULL(pCalibData);
    VERIFY_REV12();

    // static information, no need to read
    decodeCalibrationData(pCalibData);

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_SetCalibrationData_t(hbd_calibData_t const* pCalibData)
{
    ISINIT();
    VERIFY_NOT_NULL(pCalibData);
    VERIFY_REV12();

    uint8_t buffer[5];
    encodeCalibrationData(pCalibData, buffer);

    if (memcmp(&regMirror[HELENABASE_RA_TEMPOFFSET_H], buffer, sizeof(buffer)) == 0)
        return HBD_SUCCESS;

    hbd_retVal_t retVal;
    retVal = hbd_i2c_write(HELENABASE_ADDRESS, HELENABASE_RA_TEMPOFFSET_H, sizeof(buffer), buffer);
    RETVAL_CHECK(retVal);

    memcpy(&regMirror[HELENABASE_RA_TEMPOFFSET_H], buffer, sizeof(buffer));

    return HBD_SUCCESS;
}

/**END OF FILE*****************************************************************/
