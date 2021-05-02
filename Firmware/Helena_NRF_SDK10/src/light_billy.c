/**
  ******************************************************************************
  * @file    light.c
  * @author  Thomas Reisnecker
  * @brief   light management module (Billy Edition)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "helena_base_driver.h"
#include "light_billy.h"
#include "app_timer.h"
#include "power.h"
#include "fds.h"
#include "nrf_delay.h"
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/
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
    q8_t mainBeam;  // main beam current limit
    q8_t highBeam;  // high beam current limit
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
        pStatus->mainBeam.dutycycle = data.currentLeft.maxDC || data.currentLeft.minDC;
        pStatus->highBeam.dutycycle = data.currentRight.maxDC || data.currentRight.minDC;
        pStatus->currentMainBeam = data.currentLeft.current;
        pStatus->currentHighBeam = data.currentRight.current;
        pStatus->temperature = temperatureFilter(data.temperature);
    }

    return errCode;
}

/** @brief function to calculate target currents
 *
 * @param[in]   pMode       mode to configure
 * @param[out]  pMainBeam   target current for main beam
 * @param[out]  pHighBeam   target current for high beam
 */
static void calculateTarget(const light_mode_t *pMode, uint8_t *pMainBeam, uint8_t *pHighBeam)
{
    if (pMode->setup.mainBeam)
        *pMainBeam = pMode->intensityMainBeam;
    else
        *pMainBeam = 0;

    if (pMode->setup.highBeam)
        *pHighBeam = pMode->intensityHighBeam;
    else
        *pHighBeam = 0;
}

/** @brief limiter function
 *
 * @param[in/out]   pStatus     light status
 * @param[in/out]   pMainBeam   main beam current
 * @param[in/out]   pHighBeam   high beam current
 */
static void limiter(light_status_t * pStatus, uint8_t *pMainBeam, uint8_t *pHighBeam)
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

    // limit mainBeam
    pStatus->mainBeam.current = false;         // clear all flags first
    pStatus->mainBeam.voltage = false;
    pStatus->mainBeam.temperature = false;
    if (*pMainBeam > currentLimit.mainBeam)
    {
        *pMainBeam = currentLimit.mainBeam;
        pStatus->mainBeam.current = true;
    }
    if (*pMainBeam > voltageLimit)
    {
        *pMainBeam = voltageLimit;
        pStatus->mainBeam.current = false;     // clear flag in case it has been set before
        pStatus->mainBeam.voltage = true;
    }
    if (*pMainBeam > temperatureLimit)
    {
        *pMainBeam = temperatureLimit;
        pStatus->mainBeam.current = false;     // clear flag in case it has been set before
        pStatus->mainBeam.voltage = false;     // clear flag in case it has been set before
        pStatus->mainBeam.temperature = true;
    }

    // limit highBeam
    pStatus->highBeam.current = false;
    pStatus->highBeam.voltage = false;
    pStatus->highBeam.temperature = false;
    if (*pHighBeam > currentLimit.highBeam)
    {
        *pHighBeam = currentLimit.highBeam;
        pStatus->highBeam.current = true;
    }
    if (*pHighBeam > voltageLimit)
    {
        *pHighBeam = voltageLimit;
        pStatus->highBeam.current = false;
        pStatus->highBeam.voltage = true;
    }
    if (*pHighBeam > temperatureLimit)
    {
        *pHighBeam = temperatureLimit;
        pStatus->highBeam.current = false;
        pStatus->highBeam.voltage = false;
        pStatus->highBeam.temperature = true;
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

    if (pLimits->mainBeam > designLimit)
        pLimits->mainBeam = designLimit;
    currentLimit.mainBeam = pLimits->mainBeam;

    if (pLimits->highBeam > designLimit)
        pLimits->highBeam = designLimit;
    currentLimit.highBeam = pLimits->highBeam;
}

static void readStorage()
{
    uint32_t errCode;

    fds_find_token_t token;
    fds_record_desc_t descriptor;
    fds_record_key_t key = {.type = FDSDRVCONFIG, .instance = FDSINSTANCE};
    storageData_t* pData;

    // set default values
    storage.drvConfig.mainBeamCount = LIGHT_LEDCONFIG_UNKNOWN;
    storage.drvConfig.highBeamCount = LIGHT_LEDCONFIG_UNKNOWN;
    storage.currentLimit.mainBeam = OUTPUT_DEFAULT_LIMT;
    storage.currentLimit.highBeam = OUTPUT_DEFAULT_LIMT;

    errCode = fds_find(key.type, key.instance, &descriptor, &token);
    if (errCode == NRF_SUCCESS)
    {
        fds_record_t record;
        errCode = fds_open(&descriptor, &record);
        if (errCode == NRF_SUCCESS)
        {
            pData = (storageData_t*)record.p_data;
            storage.drvConfig.mainBeamCount = pData->drvConfig.mainBeamCount;
            storage.drvConfig.highBeamCount = pData->drvConfig.highBeamCount;
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

uint32_t light_UpdateTargets(const light_mode_t* pMode, const light_status_t* *ppStatus)
{
    if (pMode == NULL || ppStatus == NULL)
        return NRF_ERROR_NULL;

    if (lightState == STATEOFF)
        return NRF_ERROR_INVALID_STATE;

    // check if timer has to be changed
    if ((pMode->setup.mainBeam || pMode->setup.highBeam) && lightState == STATEIDLE)
    {
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_ON, NULL));
        lightState = STATEON;
    }
    else if ((!pMode->setup.mainBeam && !pMode->setup.highBeam) && lightState == STATEON)
    {
        VERIFY_NRF_ERROR_CODE(app_timer_stop(updateTimerId));
        VERIFY_NRF_ERROR_CODE(app_timer_start(updateTimerId, TIMEBASE_IDLE, NULL));
        lightState = STATEIDLE;
        target.mainBeam = 0;
        target.highBeam = 0;
        VERIFY_NRF_ERROR_CODE(hbd_SetTargetCurrent(target.mainBeam, target.highBeam));
    }

    if (pMode->setup.mainBeam || pMode->setup.highBeam)
        calculateTarget(pMode, &target.mainBeam, &target.highBeam);

    // clear error flags if leds are off
    if (!pMode->setup.mainBeam)
    {
        lightStatus.mainBeam.current = false;
        lightStatus.mainBeam.voltage = false;
        lightStatus.mainBeam.temperature = false;
        lightStatus.mainBeam.dutycycle = false;
    }
    if (!pMode->setup.highBeam)
    {
        lightStatus.highBeam.current = false;
        lightStatus.highBeam.voltage = false;
        lightStatus.highBeam.temperature = false;
        lightStatus.highBeam.dutycycle = false;
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
    limitedTarget.mainBeam = target.mainBeam;                 // run targets through limiter
    limitedTarget.highBeam = target.highBeam;
    limiter(&lightStatus, &limitedTarget.mainBeam, &limitedTarget.highBeam);
                                                            // and finally send to driver
    APP_ERROR_CHECK(hbd_SetTargetCurrent(limitedTarget.mainBeam, limitedTarget.highBeam));
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
    storage.drvConfig.mainBeamCount = LIGHT_LEDCONFIG_UNKNOWN;
    storage.drvConfig.highBeamCount = LIGHT_LEDCONFIG_UNKNOWN;

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
        if (lightStatus.currentMainBeam < 500)
            storage.drvConfig.mainBeamCount = 0;
        if (lightStatus.currentHighBeam < 500)
            storage.drvConfig.highBeamCount = 0;

        // read duty cycle registers
        if (storage.drvConfig.rev == LIGHT_DRIVERREV11 || storage.drvConfig.rev == LIGHT_DRIVERREV12)
        {
            target_t dutyCycle;
            errCode = hbd_ReadDutyCycles(&dutyCycle.mainBeam, &dutyCycle.highBeam);
            if(errCode == NRF_SUCCESS)
            {
                // In theory the output voltage in continuous current mode is V_out = V_in * DC.
                // Due to losses the real output voltage is lower, therefore by ignoring the
                // efficiency the calculated output voltage should be high enough to not deliver
                // a to low count, but not high enough to deliver a to high count.

                uint32_t ledVoltage;

                if (storage.drvConfig.mainBeamCount)
                {
                    ledVoltage = cellCnt * NOM_CELL_VOLTAGE;
                    ledVoltage *= dutyCycle.mainBeam;
                    ledVoltage /= HBD_DC_MAX;
                    storage.drvConfig.mainBeamCount = ledVoltage / NOM_LED_VOLTAGE;
                }
                if (storage.drvConfig.highBeamCount)
                {
                    ledVoltage = cellCnt * NOM_CELL_VOLTAGE;
                    ledVoltage *= dutyCycle.highBeam;
                    ledVoltage /= HBD_DC_MAX;
                    storage.drvConfig.highBeamCount = ledVoltage / NOM_LED_VOLTAGE;
                }
            }
        }
    }

    pLedConfig->mainBeamCount = storage.drvConfig.mainBeamCount;
    pLedConfig->highBeamCount = storage.drvConfig.highBeamCount;

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

uint32_t light_GetLimits(q8_t* pMainBeamLimit, q8_t* pHighBeamLimit)
{
    if (pMainBeamLimit == NULL || pHighBeamLimit == NULL)
        return NRF_ERROR_NULL;

    *pMainBeamLimit = storage.currentLimit.mainBeam;
    *pHighBeamLimit = storage.currentLimit.highBeam;

    return NRF_SUCCESS;
}

uint32_t light_SetLimits(q8_t mainBeamLimit, q8_t highBeamLimit)
{
    storage.currentLimit.mainBeam = mainBeamLimit;
    storage.currentLimit.highBeam = highBeamLimit;

    validateCurrentLimit(&storage.currentLimit);

    writeStorage();

    return NRF_SUCCESS;
}

/**END OF FILE*****************************************************************/
