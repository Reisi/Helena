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
#include "nrf_delay.h"

/* Private typedef -----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define HELENABASE_ADDRESS              0x3A
#define BILLYBASE_ADDRESS               0x39
#define HELENABASE_DEFAULT_ADDRESS      HELENABASE_ADDRESS

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

#define HELENABASE_REGISTER_CNT         16

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

#define HELENABASE_VOLTAGE_MAX          (14 << 11)  // no operation above this voltage
#define HELENABASE_VOLTAGE_LIMIT        (9 << 11)   // limited output current above this voltage
#define HELENABASE_VOLTAGE_MIN          (3 << 11)   // no operation below this voltage
#define HELENABASE_CURRENT_LIMIT        204         // current limit for voltages above HELENABASE_LIMIT_VOLTAGE
#define HELENABASE_CURRENT_MAX          255         // current limit for voltages below HELENABASE_LIMIT_VOLTAGE

/// TODO: verify range!
#define HELENABASE_GAIN_MIN             200         // the minimum gain factor for calibration data

/* Private macros ------------------------------------------------------------*/
/*#define ISINIT()                        \
do                                      \
{                                       \
    if (firmware == HBD_FWREV_UNKOWN)   \
    {                                   \
        hbd_retVal_t retVal;            \
        retVal = initialize();          \
        if (retVal != HBD_SUCCESS)      \
            return retVal;              \
    }                                   \
} while(0);*/

#define READ(addr, reg, data, len)    (read(addr, reg, data, len) ? HBD_ERROR_COM : HBD_SUCCESS)

#define WRITE(addr, reg, data, len)   (write(addr, reg, data, len) ? HBD_ERROR_COM : HBD_SUCCESS)

#define VERIFY_INIT()                   \
do                                      \
{                                       \
    if (read == NULL || write == NULL)  \
        return HBD_ERROR_INVALID_STATE; \
} while(0)

#define VERIFY_INST(pInst)               \
do                                       \
{                                        \
    if (pInst->fwRev == HBD_FWREV_UNKOWN)\
        return HBD_ERROR_INVALID_STATE;  \
} while(0)

#define VERIFY_NOT_NULL(param)          \
do                                      \
{                                       \
    if (param == NULL)                  \
        return HBD_ERROR_NULL;          \
} while(0)

#define VERIFY_REV11(pInst)             \
do                                      \
{                                       \
    if (pInst->fwRev != HBD_FWREV_11 && \
        pInst->fwRev != HBD_FWREV_12)   \
        return HBD_ERROR_NOT_SUPPORTED; \
} while(0)

#define VERIFY_REV12(pInst)             \
do                                      \
{                                       \
    if (pInst->fwRev != HBD_FWREV_12)   \
        return HBD_ERROR_NOT_SUPPORTED; \
} while(0)

#define RETVAL_CHECK(retVal)            \
do                                      \
{                                       \
    if (retVal != HBD_SUCCESS)          \
        return retVal;                  \
} while(0)

#define REGISTERVALUE_TO_AMPEREQ10(x)       ((uint16_t)(((x) * 771ul) >> 10))
#define REGISTERVALUE_TO_AMPEREQ10_BILLY(x) ((uint16_t)(((x) * 3ul) >> 3))
#define REGISTERVALUE_TO_KELVINQ4(x)        ((uint16_t)((uint32_t)(x) << 2))

/* read only variables -------------------------------------------------------*/
static uint8_t baseAdresses[] =
{
    BILLYBASE_ADDRESS,
    HELENABASE_ADDRESS
};

static uint8_t resetValue[3] =
{
    0x16, 0x00, 0x00
};

/* Private variables ---------------------------------------------------------*/
static hbd_read_t read;
static hbd_write_t write;

/* Private functions ---------------------------------------------------------*/
static void decodeSamplingData(hbd_firmwareRev_t fwRev, uint8_t const* pReg, hbd_samplingData_t* pData)
{
    uint_fast16_t i;
    // left driver current information
    i = ((uint_fast16_t)pReg[0] << 8) | pReg[1];
    pData->currentLeft.maxDC = i & (1<<13);
    pData->currentLeft.minDC = i & (1<<12);
    if (fwRev == HDB_FWREV_BILLY)
        pData->currentLeft.current = REGISTERVALUE_TO_AMPEREQ10_BILLY(i & 0x0FFF);
    else
        pData->currentLeft.current = REGISTERVALUE_TO_AMPEREQ10(i & 0x0FFF);
    // right driver current information
    i = ((uint_fast16_t)pReg[2] << 8) | pReg[3];
    pData->currentRight.maxDC = i & (1<<13);
    pData->currentRight.minDC = i & (1<<12);
    if (fwRev == HDB_FWREV_BILLY)
        pData->currentRight.current = REGISTERVALUE_TO_AMPEREQ10_BILLY(i & 0x0FFF);
    else
        pData->currentRight.current = REGISTERVALUE_TO_AMPEREQ10(i & 0x0FFF);
    // temperature information
    i = ((uint_fast16_t)pReg[4] << 8) | pReg[5];
    pData->temperature = REGISTERVALUE_TO_KELVINQ4(i & 0x0FFF);
}

static hbd_retVal_t initialize(hbd_inst_t* pInst, hbd_samplingData_t* pData, uint8_t* pCnt, bool reset)
{
    hbd_retVal_t retVal;
    uint8_t cnt;
    bool powerUpDelay = false;

    cnt = *pCnt;
    *pCnt = 0;

    for (uint_fast8_t i = 0; i < cnt; i++)
    {
        pInst[i].fwRev = HBD_FWREV_UNKOWN;
    }

#ifdef HDB_FULL_MIRROR
    uint8_t* pMirror = pInst->regMirror;
#else
    uint8_t buffer[HELENABASE_REGISTER_CNT];
    uint8_t* pMirror = buffer;
#endif // HDB_FULL_MIRROR

    for (uint_fast8_t i = 0; i < sizeof(baseAdresses) && *pCnt < cnt; i++)
    {
        //uint8_t* pMirror = pInst[*pCnt].regMirror;

        retVal = READ(baseAdresses[i], HELENABASE_RA_CONFIG, pMirror, HELENABASE_REGISTER_CNT);
        // driver has a bootloader which waits 250ms after power up, so if the first read fails,
        // just wait 250ms (only one time) and try again
        if (retVal != HBD_SUCCESS && !powerUpDelay)
        {
            nrf_delay_ms(250);
            retVal = READ(baseAdresses[i], HELENABASE_RA_CONFIG, pMirror, HELENABASE_REGISTER_CNT);
            powerUpDelay = true;
        }
        if (retVal == HBD_SUCCESS)
        {
            pInst[*pCnt].address = baseAdresses[i];

            if (baseAdresses[i] == BILLYBASE_ADDRESS)
                pInst[*pCnt].fwRev = HDB_FWREV_BILLY;
            else if (memcmp(pMirror, &pMirror[HELENABASE_REGISTERCNT_REV10], HELENABASE_REGISTERCNT_REV12 - HELENABASE_REGISTERCNT_REV10) == 0)
                pInst[*pCnt].fwRev = HBD_FWREV_10;
            else if (memcmp(pMirror, &pMirror[HELENABASE_REGISTERCNT_REV11], HELENABASE_REGISTERCNT_REV12 - HELENABASE_REGISTERCNT_REV11) == 0)
                pInst[*pCnt].fwRev = HBD_FWREV_11;
            else
                pInst[*pCnt].fwRev = HBD_FWREV_12;

            if (pData != NULL)
            {
                decodeSamplingData(pInst->fwRev, &pMirror[HELENABASE_RA_STATUSSDL_H], &pData[*pCnt]);
            }

#ifndef HDB_FULL_MIRROR
            memcpy(pInst[*pCnt].regMirror, pMirror, sizeof(pInst[*pCnt].regMirror));
#endif // HDB_FULL_MIRROR

            if (reset)
                (void)WRITE(baseAdresses[i], HELENABASE_RA_CONFIG, resetValue, sizeof(resetValue));

            *pCnt += 1;
        }
    }

    return *pCnt ? HBD_SUCCESS : HBD_ERROR_NOT_FOUND;
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

static bool encodeCalibrationData(hbd_calibData_t const* pData, uint8_t* pReg)
{
    if (pData->gainLeft < HELENABASE_GAIN_MIN || pData->gainRight < HELENABASE_GAIN_MIN)
        return false;

    pReg[0] = (uint8_t)(pData->temperatureOffset >> 8);
    pReg[4] = pReg[0];
    pReg[1] = (uint8_t)(pData->temperatureOffset & 0x00FF);
    pReg[4] ^= pReg[1];
    pReg[2] = pData->gainLeft;
    pReg[4] ^= pReg[2];
    pReg[3] = pData->gainRight;
    pReg[4] ^= pReg[3];

    return true;
}

static void decodeCalibrationData(uint8_t const* pRaw, hbd_calibData_t* pData)
{
    pData->temperatureOffset = (q13_2_t)((uint_fast16_t)pRaw[0] << 8 | pRaw[1]);
    pData->gainLeft = pRaw[2];
    pData->gainRight = pRaw[3];
}

static bool limitDC(q5_11_t voltage, q8_t* pLeft, q8_t* pRight)
{
    q8_t max;
    bool limit = false;

    if (voltage > HELENABASE_VOLTAGE_MAX || voltage < HELENABASE_VOLTAGE_MIN)
        max = 0;
    if (voltage < HELENABASE_VOLTAGE_LIMIT - ((HELENABASE_CURRENT_MAX - HELENABASE_CURRENT_LIMIT) << 4))
        max = HELENABASE_CURRENT_MAX;
    else
        max = HELENABASE_CURRENT_MAX - ((HELENABASE_VOLTAGE_LIMIT - voltage) >> 4);

    if (*pLeft > max)
    {
        *pLeft = max;
        limit = true;
    }

    if (*pRight > max)
    {
        *pRight = max;
        limit = true;
    }

    return limit;
}

/* Public functions ----------------------------------------------------------*/
hbd_retVal_t hbd_Init(hbd_init_t const* pInit, hbd_inst_t* pInst, hbd_samplingData_t* pData, uint8_t* pCnt, bool reset)
{
    VERIFY_NOT_NULL(pInit);
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pCnt);

    read = pInit->i2cRead;
    write = pInit->i2cWrite;

    return initialize(pInst, pData, pCnt, reset);
}

hbd_retVal_t hbd_SetConfig(hbd_inst_t* pInst, hbd_config_t const* pConfig)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pConfig);
    VERIFY_INST(pInst);

    uint8_t cfg;
    encodeConfig(pConfig, &cfg);

    if (cfg == pInst->regMirror[HELENABASE_RA_CONFIG])
        return HBD_SUCCESS;

    hbd_retVal_t retVal;
    retVal = WRITE(pInst->address, HELENABASE_RA_CONFIG, &cfg, 1);
    RETVAL_CHECK(retVal);

    pInst->regMirror[HELENABASE_RA_CONFIG] = cfg;
    return HBD_SUCCESS;
}

hbd_retVal_t hbd_GetConfig(hbd_inst_t const* pInst, hbd_config_t* pConfig)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pConfig);
    VERIFY_INST(pInst);

    decodeConfig(pInst->regMirror[HELENABASE_RA_CONFIG], pConfig);
    return HBD_SUCCESS;
}

hbd_retVal_t hbd_SetTargetCurrent(hbd_inst_t* pInst, q5_11_t inputVoltage, q8_t left, q8_t right)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_INST(pInst);

    if (pInst->regMirror[HELENABASE_RA_TARGETSDL] == left &&
        pInst->regMirror[HELENABASE_RA_TARGETSDR] == right)
        return HBD_SUCCESS;

    bool voltageLimit = limitDC(inputVoltage, &left, &right);
    uint8_t buffer[2] = {left, right};

    hbd_retVal_t retVal;
    retVal = WRITE(pInst->address, HELENABASE_RA_TARGETSDL, buffer, sizeof(buffer));
    RETVAL_CHECK(retVal);

    memcpy(&pInst->regMirror[HELENABASE_RA_TARGETSDL], buffer, sizeof(buffer));

    return voltageLimit ? HBD_ERROR_FORBIDDEN : HBD_SUCCESS;
}

hbd_retVal_t hbd_GetTargetCurrent(hbd_inst_t const* pInst, q8_t* pLeft, q8_t* pRight)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pLeft);
    VERIFY_NOT_NULL(pRight);
    VERIFY_INST(pInst);

    *pLeft = pInst->regMirror[HELENABASE_RA_TARGETSDL];
    *pRight = pInst->regMirror[HELENABASE_RA_TARGETSDR];

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_ReadSamplingData(hbd_inst_t* pInst, hbd_samplingData_t* pSamplingData)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pSamplingData);
    VERIFY_INST(pInst);

#ifdef HDB_FULL_MIRROR
    uint8_t* pData = &pInst->regMirror[HELENABASE_RA_STATUSSDL_H];
#else
    uint8_t buffer[6];
    uint8_t* pData = buffer;
#endif // HDB_FULL_MIRROR

    hbd_retVal_t retVal;
    retVal = READ(pInst->address, HELENABASE_RA_STATUSSDL_H, pData, 6);
    RETVAL_CHECK(retVal);

    decodeSamplingData(pInst->fwRev, pData, pSamplingData);

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_GetFirmwareRev(hbd_inst_t const* pInst, hbd_firmwareRev_t* pFWRef)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pFWRef);
    VERIFY_INST(pInst);

    *pFWRef = pInst->fwRev;

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_ReadDutyCycles(hbd_inst_t* pInst, q8_t* pLeft, q8_t* pRight)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pLeft);
    VERIFY_NOT_NULL(pRight);
    VERIFY_INST(pInst);
    VERIFY_REV11(pInst);

#ifdef HDB_FULL_MIRROR
    uint8_t* pData = &pInst->regMirror[HELENABASE_RA_DUTYCYCLELEFT];
#else
    uint8_t buffer[2];
    uint8_t* pData = buffer;
#endif // HDB_FULL_MIRROR

    hbd_retVal_t retVal;
    retVal = READ(pInst->address, HELENABASE_RA_DUTYCYCLELEFT, pData, 2);
    RETVAL_CHECK(retVal);

    *pLeft = pData[0];
    *pRight = pData[1];

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_GetCalibrationData(hbd_inst_t const* pInst, hbd_calibData_t* pCalibData)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pCalibData);
    VERIFY_INST(pInst);
    VERIFY_REV12(pInst);

#ifdef HDB_FULL_MIRROR
    uint8_t const* pData = &pInst->regMirror[HELENABASE_RA_TEMPOFFSET_H];
#else
    uint8_t buffer[5];
    uint8_t* pData = buffer;

    hbd_retVal_t retVal;
    retVal = READ(pInst->address, HELENABASE_RA_TEMPOFFSET_H, buffer, sizeof(buffer));
    RETVAL_CHECK(retVal);
#endif // HDB_FULL_MIRROR

    decodeCalibrationData(pData, pCalibData);

    return HBD_SUCCESS;
}

hbd_retVal_t hbd_SetCalibrationData(hbd_inst_t* pInst, hbd_calibData_t const* pCalibData)
{
    VERIFY_INIT();
    VERIFY_NOT_NULL(pInst);
    VERIFY_NOT_NULL(pCalibData);
    VERIFY_INST(pInst);
    VERIFY_REV12(pInst);

    uint8_t buffer[5];
    if (!encodeCalibrationData(pCalibData, buffer))
        return HBD_ERROR_INVALID_PARAM;

    if (memcmp(&pInst->regMirror[HELENABASE_RA_TEMPOFFSET_H], buffer, sizeof(buffer)) == 0)
        return HBD_SUCCESS;

    hbd_retVal_t retVal;
    retVal = WRITE(pInst->address, HELENABASE_RA_TEMPOFFSET_H, buffer, sizeof(buffer));
    RETVAL_CHECK(retVal);

#ifdef HDB_FULL_MIRROR
    memcpy(&pInst->regMirror[HELENABASE_RA_TEMPOFFSET_H], buffer, sizeof(buffer));
#endif // HDB_FULL_MIRROR

    return HBD_SUCCESS;
}

/**END OF FILE*****************************************************************/
