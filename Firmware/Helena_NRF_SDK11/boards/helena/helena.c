/**
  ******************************************************************************
  * @file    template.c
  * @author  Thomas Reisnecker
  * @brief   board managment module (template)
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define BRD_LOG_ENABLED

#ifdef BRD_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // BRD_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "helena_board.h"
#include "helena_hmi.h"
#include "helena_adc.h"
#include "helena_btle.h"
#include "mode_management.h"
#include "string.h"
#include "helena_base_driver.h"
#include "mpu6050drv.h"
#include "i2c.h"
#include "main.h"
#include "btle.h"
#include "helena.h"
#include "pitch_reg.h"
#include "filter.h"
#include "limiter.h"
//#include "nrf_nvic.h"
#include "nrf_delay.h"
//#include "nrf_nvmc.h"
#include "fstorage.h"
#include "fstorage_config.h"
#include "fstorage_internal_defs.h"
#include "fds_config.h"
#include "nrf.h"

/* macros --------------------------------------------------------------------*/
#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))
#define MAX_OF(a, b)            (((a) > (b)) ? (a) : (b))
#define SIZE_OF_CHANNELS(chcnt) (MM_NUM_OF_MODES * sizeof(cht_5int3mode_t) * chcnt)

/* Private defines -----------------------------------------------------------*/
#define CHANNEL_CNT             4   // if changed, need to be a multiple of 2, don't forget to expand channelTypes and channelDefaults
#define SIZE_OF_MODES           (MM_NUM_OF_MODES * sizeof(mm_modeConfig_t))
#define DRIVER_CNT              (CHANNEL_CNT / 2)
#define MAX_CURRENT             (3 << 10)   // 3A max current for a channel
#define PITCH_OFFSET_MAX        (1 << 12)   // +45°
#define PITCH_OFFSET_MIN        (-1 * PITCH_OFFSET_MAX) // -45°

#define DEFAULT_SET             0           // the default config set, different sets not implemented yet, so 0
#define CHANNELS_VERSION        0           // channel version to identify changes for future extensions
#define SETUP_VERSION           0
#define HELENA_FILE_ID          0x373A      // the file id for this module
#define CHANNELS_RECORD_BASE    0x0101      // the base for channel configuration
#define SETUP_RECORD_BASE       0x0111      // the base for setup
#define QWR_BUFFER_SIZE         MAX_OF(BLE_HPS_MIN_BUF_SIZE(sizeof(modesBuffer)), sizeof(setup_str_t))

#define STORE_IDLE              (ds_reportHandler_t)0xFFFFFFFF

#define DEFAULT_STATE           \
{                               \
    .powerMode = BRD_PM_OFF,    \
    .lightMode = MM_MODE_OFF    \
}

#define TIMER_PERIOD_SLOW       MAIN_TIMER_TICKS(1000)
#define TIMER_PERIOD_FAST       MAIN_TIMER_TICKS(10)

#define ONE_SEC_PRESCALE_FAST  (MAIN_TIMER_TICKS(1000) / TIMER_PERIOD_FAST)
#define ONE_SEC_PRESCALE_SLOW  (MAIN_TIMER_TICKS(1000) / TIMER_PERIOD_SLOW)

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint16_t id;
    uint16_t rev;
} device_info_t;

typedef uint16_t q8_8_t;

/**<  channel types */
 typedef enum
{
    CHANNEL_FLOOD = 0,// the left current driver on the helena base
    CHANNEL_SPOT,     // the right current driver on the helena base
} channelTypes_t;

/**< channel modes */
typedef enum
{
    CHMODE_CONST = 0,
    CHMODE_PITCH,
} channelModes_t;

/**< the structure how the channels are stored in flash */
typedef struct
{
    uint8_t         numOfModesPerChannel;
    uint8_t         version;
    uint16_t        dummy;
    cht_5int3mode_t channels[CHANNEL_CNT][MM_NUM_OF_MODES];
} channels_str_t;

typedef struct
{
    hln_channelSetup_t channelSetup[CHANNEL_CNT];   // 8 bytes each
    char deviceName[MAX_NAME_LENGTH + 1];           // 32 bytes
    hln_comPinMode_t comPinMode;                    // 1
    uint8_t driverCnt;                              // 1
    bool isImuAvailable;                            // 1
    mpu6050drv_sensorOffset_t imuOffset;            // 18 bytes
} setup_t;

typedef struct
{
    uint8_t   numOfChannels;
    uint8_t   version;
    uint16_t  dummy;
    setup_t   setup;
} setup_str_t;

typedef struct
{
    brd_powerMode_t powerMode;
    bool            isMultiOpticMode;
    cht_5int3mode_t ovverrideConfig[CHANNEL_CNT];
    uint8_t         lightMode;
} state_t;

/* function prototypes -------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/
static cht_5int3mode_t const channelDefaults[CHANNEL_CNT][MM_NUM_OF_MODES] =
{
    {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}},
    {{ 5, 0}, { 16, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}},
    {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}},
    {{ 0, 0}, { 0, 0}, { 0, 0}, { 0, 0}, {0,  0}, {0,  0}, {0,  0}, {0,  0}},
};

static setup_t const setupDefaults =
{
    .channelSetup =
    {
        {
            .outputPower = (18 << 10),//0,
            .outputLimit = (218 << 8),//0,
            .optic.type = PRG_TYPE_30,//PRG_TYPE_NA,
            .optic.offset = 0,
        },
        {
            .outputPower = (18 << 10),
            .outputLimit = (218 << 8),
            .optic.type = PRG_TYPE_15,
            .optic.offset = 0,
        },
        {
            .outputPower = 0,
            .outputLimit = 0,
            .optic.type = PRG_TYPE_NA,
            .optic.offset = 0,
        },
        {
            .outputPower = 0,
            .outputLimit = 0,
            .optic.type = PRG_TYPE_NA,
            .optic.offset = 0,
        }
    },
    .deviceName = {'H', 'e', 'l', 'e', 'n', 'a', '\0'},
    .comPinMode = COMPIN_MODE_NOTUSED,
    .driverCnt = 1,
    .isImuAvailable = true,
};


/* Private variables ---------------------------------------------------------*/
static uint8_t              modesBuffer[SIZE_OF_MODES + SIZE_OF_CHANNELS(CHANNEL_CNT)];  // the memory for the modes characteristic
static uint8_t              qwrMemBuffer[QWR_BUFFER_SIZE] __ALIGN(4);
static ble_user_mem_block_t qwrMemBlock =
{
    .p_mem = qwrMemBuffer,
    .len = sizeof(qwrMemBuffer)
};
static ble_hps_qwr_buffer_t qwrBuffer = {.p_block = &qwrMemBlock};          // buffer for queued writes and storage operations

static cht_5int3mode_t*     pChannels = (cht_5int3mode_t*)&modesBuffer[SIZE_OF_MODES];  // pointer to the channels configuration
static setup_t const*       pSetup = &setupDefaults;                        // pointer to the setup

APP_TIMER_DEF(sampleTimer);                                                 // adc sample timer, either off or running at 1Hz or 100Hz
uint16_t                    oneSecPrescale, oneSecPreload;                  // prescaler and reload value to generate a 1Hz timebase
bool                        updateFlag;                                     // flag indicating that adc value has been received and a target update is pending

static hbd_inst_t           helenaInst[DRIVER_CNT];                         // the helena driver board instance
FIL_LOWPASS_DEF(filterVolt, 64);                                            // filter instance for input voltage
FIL_LOWPASS_DEF(filterTemp, 64);                                            // filter instance for temperatire
static q15_t                pitch;                                          // latest pitch angle
static lim_channelDef_t     limiter[CHANNEL_CNT];                           // limiter channel definitions
static uint8_t              batteryCellCnt;                                 // cell count of connected battery
static bool                 isLimiting;                                     // indicator if limiter is active

static ds_reportHandler_t   pendingResultHandler = STORE_IDLE;              // the resultHandler to call after storage operation
static state_t              currentState = DEFAULT_STATE;                   // the current state
static state_t              pendingState = DEFAULT_STATE;                   // the pending state

static volatile bool        eraseInProgress;

/* Board information ---------------------------------------------------------*/
static btle_info_t bleInfo =
{
    .pDevicename = setupDefaults.deviceName,
    .pModelNumber = "Helena",
    .pBoardHWVersion = "n/a",   // as fallback, if detection fails
    .deviceAppearance = BLE_APPEARANCE_GENERIC_CYCLING,
};

static ble_hps_f_ch_size_t const channelTypes[CHANNEL_CNT] =
{
    {   // left(flood) current regulator on 1st helena base driver
        .channel_bitsize     = 8,
        .sp_ftr_bitsize      = 3,
        .channel_description = BLE_HPS_CH_DESC_CURRENT
    },
    {   // right(spot) current regulator on 1st helena base driver
        .channel_bitsize     = 8,
        .sp_ftr_bitsize      = 3,
        .channel_description = BLE_HPS_CH_DESC_CURRENT
    },
    {   // left(flood) current regulator on 2nd helena base driver
        .channel_bitsize     = 8,
        .sp_ftr_bitsize      = 3,
        .channel_description = BLE_HPS_CH_DESC_CURRENT
    },
    {   // right(spot) current regulator on 2nd helena base driver
        .channel_bitsize     = 8,
        .sp_ftr_bitsize      = 3,
        .channel_description = BLE_HPS_CH_DESC_CURRENT
    }
};

static brd_features_t features =
{
    .channelCount = 0,  // will be set after initialization according to the number of detected drivers
    .pChannelTypes = channelTypes
};

static ble_hps_modes_init_t modes =
{
    .p_mode_config = (ble_hps_mode_config_t*)modesBuffer,
    //.total_size = sizeof(modesBuffer) // will be set after initialization according to the number of detected drivers
};

static brd_info_t const brdInfo =
{
    .pFeatures = &features,
    .pInfo = &bleInfo,
    .pModes = &modes,
    .pBuf = &qwrBuffer
};

/* Private functions ---------------------------------------------------------*/
static bool isPageEmpty(uint32_t const* pPage)
{
    for (uint16_t i = 0; i < FS_PAGE_SIZE / sizeof(uint32_t); i ++)
    {
        if (pPage[i] != 0xFFFFFFFF)
        {
            LOG_INFO("[brd]: page not empty, %d", i);
            return false;
        }
    }

    LOG_INFO("[brd]: page empty");
    return true;
}

static bool isChannelSetupValid(hln_channelSetup_t const* pChannelSetup, uint8_t channelCnt)
{
    if (channelCnt > features.channelCount)
        channelCnt = features.channelCount; // don't check channels which are not available

    for (uint8_t i = 0; i < channelCnt; i++)
    {
        if (pChannelSetup[i].outputPower > HLN_OUTPUT_POWER_MAX)
            return false;
        if (pChannelSetup[i].optic.type >= PRG_TYPE_CNT && pChannelSetup[i].optic.type != PRG_TYPE_NA)
            return false;
        if (pChannelSetup[i].optic.offset > PITCH_OFFSET_MAX || pChannelSetup[i].optic.offset < PITCH_OFFSET_MIN)
            return false;
    }

    return true;
}

static bool isComPinModeValid(hln_comPinMode_t mode)
{
    if (mode == COMPIN_MODE_NOTUSED)
        return true;
    if (mode == COMPIN_MODE_COM)
        return true;
    if (mode == COMPIN_MODE_BUTTON)
        return true;
    return false;
}

static bool isSetupValid(setup_t const* pSetup)
{
    if (!isChannelSetupValid(pSetup->channelSetup, ARRAY_SIZE(pSetup->channelSetup)))
        return false;
    if (!isComPinModeValid(pSetup->comPinMode))
        return false;
    if (strlen(pSetup->deviceName) > MAX_NAME_LENGTH)
        return false;
    if (pSetup->driverCnt > DRIVER_CNT)
        return false;

    return true;
}

/// TODO: failsafe if validation fails?
static ret_code_t loadSetup()
{
    ret_code_t         errCode;
    setup_str_t const* pSetupInMem;
    uint16_t           lengthWords;

    errCode = ds_Read(HELENA_FILE_ID, SETUP_RECORD_BASE, (void const**)&pSetupInMem, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(setup_str_t)) &&
            pSetupInMem->numOfChannels == CHANNEL_CNT &&
            isSetupValid(&pSetupInMem->setup))
        {
            pSetup = &pSetupInMem->setup;
            return NRF_SUCCESS;
        }
    }
    else if (errCode != FDS_ERR_NOT_FOUND)
        LOG_ERROR("[brd]: setup record load error %d", errCode);

    return errCode;
}

static void sysEventDispatch(uint32_t sysEvent)
{
    if (sysEvent != NRF_EVT_FLASH_OPERATION_SUCCESS && sysEvent != NRF_EVT_FLASH_OPERATION_ERROR)
        return;

    if (eraseInProgress)
        eraseInProgress = false;
    else
    {   // setup pointer needs to be updated after each write and garbage
        // collections but it is not possible to know when gc is run, so to
        // be safe reload on any flash operation
        ret_code_t errCode = loadSetup();
        if (errCode != NRF_SUCCESS)
        LOG_ERROR("[brd]: error %d loading setup", errCode);
    }
}

SYSEVENT_REGISTER(const main_SysEventHandler_t btle_sysEvent) =
{
    .pHandler = sysEventDispatch
};

/** @brief function for cleaning up storage data from older revisions
 */
static ret_code_t cleanUpOldRevData()
{
    // fds in SDK10 uses different headers, so let's check if user data pages
    // are empty or already initialized and if not delete pages

    for (uint8_t i = 0; i < FDS_VIRTUAL_PAGES; i++)
    {
        uint32_t* pPage = (uint32_t*)((uint32_t)FS_PAGE_END_ADDR - (i + 1) * FS_PAGE_SIZE);
        if (pPage[0] != 0xDEADC0DE && !isPageEmpty(pPage))
        {
            // page contains old garbage, delete
            LOG_INFO("[brd]: deleting page %d", (uint32_t)pPage / FS_PAGE_SIZE);
            ret_code_t errCode = sd_flash_page_erase((uint32_t)pPage / FS_PAGE_SIZE);
            if (errCode != NRF_SUCCESS)
                return errCode;

            eraseInProgress = true;
            while(eraseInProgress) {}
        }
    }

    return NRF_SUCCESS;
}

/** @brief helper to check if a given channel configuration is valid
 *
 * @param[in] pChannels   the configuration to check
 * @param[in] numOfModes  the number of modes for each channel
 * @return true if valid, otherwise false
 */
static bool isChannelConfigValid(cht_5int3mode_t const* pChannels, uint16_t numOfModes, bool isImuPresent)
{
    /// TODO: make this depending on presence of driver board and acceleration sensor

    for (uint8_t i = 0; i < numOfModes; i++)
    {
        for (uint8_t j = 0; j < ARRAY_SIZE(helenaInst); j++)
        {
            // ignore settings if board is not available
            if (helenaInst[j].address != 0)
            {
                uint16_t offsetFlood = (j * 2 + CHANNEL_FLOOD) * numOfModes;
                uint16_t offsetSpot  = (j * 2 + CHANNEL_SPOT)  * numOfModes;

                // intensity needs to be within limits, and special feature requires IMU
                if (pChannels[offsetFlood + i].intensity > CHT_PERCENT_TO_53(100) ||
                    (pChannels[offsetFlood + i].mode == CHMODE_PITCH && !isImuPresent))
                    return false;
                if (pChannels[offsetSpot + i].intensity > CHT_PERCENT_TO_53(100) ||
                    (pChannels[offsetSpot + i].mode == CHMODE_PITCH && !isImuPresent))
                    return false;
            }
        }
    }
    return true;
}

static ret_code_t loadChannels(uint8_t set, bool isImuPresent)
{
    ret_code_t            errCode;
    channels_str_t const* pChannelsInMem;
    uint16_t              lengthWords;

    errCode = ds_Read(HELENA_FILE_ID, CHANNELS_RECORD_BASE + set, (void const**)&pChannelsInMem, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(channels_str_t)) &&
            pChannelsInMem->numOfModesPerChannel == MM_NUM_OF_MODES &&
            isChannelConfigValid(&pChannelsInMem->channels[0][0], pChannelsInMem->numOfModesPerChannel, isImuPresent))
        {
            memcpy(pChannels, pChannelsInMem->channels, SIZE_OF_CHANNELS(CHANNEL_CNT));
            return NRF_SUCCESS;
        }
    }
    else if (errCode != FDS_ERR_NOT_FOUND)
    {
        LOG_ERROR("[brd]: channel record load error %d", errCode);
    }

    // no modes stored yet or invalid, load defaults
    //cht_5int3mode_t channels[CHANNEL_CNT][MM_NUM_OF_MODES] = CHANNEL_DEFAULTS;
    memcpy(pChannels, channelDefaults, SIZE_OF_CHANNELS(CHANNEL_CNT));

    return errCode;
}

static void channelStorageResultHandler(ret_code_t errCode)
{
    LOG_INFO("[brd]: channel storage result %d", errCode);
    qwrBuffer.inUse = false;     // release buffer

    if (pendingResultHandler == STORE_IDLE)
        LOG_ERROR("[brd]: unexpected storage event");
    else if (pendingResultHandler)
        pendingResultHandler(errCode); // relay event to original handler

    pendingResultHandler = STORE_IDLE;
}

static ret_code_t storeChannels(uint8_t set)
{
    ret_code_t errCode;
    void const* pData;
    uint16_t lenghtInWords;

    qwrBuffer.inUse = true;      // mark buffer to be used

    // prepare data
    channels_str_t *pNewChannels = (channels_str_t*)qwrBuffer.p_block->p_mem;
    pNewChannels->version = CHANNELS_VERSION;
    pNewChannels->numOfModesPerChannel = MM_NUM_OF_MODES;
    pNewChannels->dummy = 0;
    memcpy(pNewChannels->channels, pChannels, SIZE_OF_CHANNELS(CHANNEL_CNT));

    pData         = (void const*)pNewChannels;
    lenghtInWords = BYTES_TO_WORDS(sizeof(channels_str_t));

    // execute store operation
    errCode = ds_Write(HELENA_FILE_ID, CHANNELS_RECORD_BASE + set, pData, lenghtInWords, channelStorageResultHandler);

    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[brd]: error %d storing modes", errCode);

        qwrBuffer.inUse = false;
    }

    return errCode;
}

static void setupStorageResultHandler(ret_code_t errCode)
{
    LOG_INFO("[brd]: setup storage result %d", errCode);
    qwrBuffer.inUse = false;     // release buffer

    if (pendingResultHandler == STORE_IDLE)
        LOG_ERROR("[brd]: unexpected storage event");
    else if (pendingResultHandler)
        pendingResultHandler(errCode); // relay event to original handler

    pendingResultHandler = STORE_IDLE;

    // setup is loaded in system event handler
    //if (errCode == NRF_SUCCESS)
    //    (void)loadSetup();
}

static ret_code_t storePreparedSetup(ds_reportHandler_t resultHandler)
{
    ret_code_t errCode;
    void const* pData;
    uint16_t lenghtInWords;

    pData         = (void const*)qwrBuffer.p_block->p_mem;
    lenghtInWords = BYTES_TO_WORDS(sizeof(setup_str_t));

    // execute store operation
    errCode = ds_Write(HELENA_FILE_ID, SETUP_RECORD_BASE, pData, lenghtInWords, setupStorageResultHandler);

    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[brd]: error %d storing setup", errCode);

        qwrBuffer.inUse = false;
    }

    pendingResultHandler = resultHandler;

    return errCode;
}

static void storageExecute()
{
    // check if storing operations are pending
    // only channel storage can be pending, setup changes will be rejected if
    // buffer is already in use, therefor if buffer is free and pending result
    // handler is set it only can be a channel storage request
    if (pendingResultHandler != STORE_IDLE && qwrBuffer.inUse == false)
    {
        LOG_INFO("[brd]: buffer released, storing");
        ret_code_t errCode = storeChannels(DEFAULT_SET);

        if (errCode != NRF_SUCCESS)
        {
            if (pendingResultHandler)
                pendingResultHandler(errCode);
            pendingResultHandler = STORE_IDLE;
            LOG_INFO("[brd]: storing error %d", errCode);
        }
    }
}

static void adcHandler(q5_11_t result)
{
    if (oneSecPreload == ONE_SEC_PRESCALE_SLOW)
        fil_LowPassReset(&filterVolt, result << 5);
    else
        fil_LowPassFeed(&filterVolt, result << 5);

    updateFlag = true;
}

static ret_code_t initAdc(uint32_t vinPin, q5_11_t* pVoltage)
{
    ret_code_t errCode;

    hln_adc_init_t init =
    {
        .vinPin = vinPin,
        .resultHandler = adcHandler
    };

    errCode = hln_adc_Init(&init, pVoltage);

    if (errCode == NRF_SUCCESS)
        fil_LowPassReset(&filterVolt, *pVoltage << 5);

    return errCode;
}

static ret_code_t initI2c(hln_brd_t const* pBoard)
{
    return i2c_Init(pBoard->twiPins.sda, pBoard->twiPins.scl, NRF_TWI_FREQ_400K);
}

/** @brief function to initialize the helena base driver
 *
 * @return NRF_SUCCESS or NRF_ERROR_NOT_FOUND
 */
static ret_code_t initHelenaDriver(uint8_t* pDriverCnt)
{
    hbd_retVal_t errCode;
    hbd_init_t init;
    *pDriverCnt = ARRAY_SIZE(helenaInst);

    init.i2cRead = i2c_Read;
    init.i2cWrite = i2c_Write;

    errCode = hbd_Init(&init, helenaInst, NULL, pDriverCnt, false);
    if (errCode == HBD_SUCCESS)
        return NRF_SUCCESS;
    else
        return NRF_ERROR_NOT_FOUND;
}

void sampleTimerCallback(void * pContext)
{
    (void)pContext;

    ret_code_t errCode = hln_adc_Sample();
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[brd]: error %d starting adc sampling", errCode);

    if (oneSecPrescale)
        oneSecPrescale--;
}

static ret_code_t timerInit()
{
    ret_code_t errCode = app_timer_create(&sampleTimer, APP_TIMER_MODE_REPEATED, &sampleTimerCallback);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[brd]: sample timer create error %d", errCode);
        return errCode;
    }

    return errCode;
}

static void mpuEventHandler(mpu6050drv_event_t const* pEvt)
{
    if (pEvt->type == MPU6050_EVT_TYPE_MOTION)
        (void)main_ResetIdleTimer();

    if (pEvt->type == MPU6050_EVT_TYPE_DATA)
    {
        pitch = pEvt->data.euler[0];
        /*int16_t pitch, roll, yaw;
        pitch = (pEvt->data.euler[0] * 360l) >> 15;
        roll = (pEvt->data.euler[1] * 360l) >> 15;
        yaw = (pEvt->data.euler[2] * 360l) >> 15;

        LOG_INFO("[brd]: p %d, r %d, y %d", pitch, roll, yaw);*/
    }
}

static ret_code_t initMpu(signed char const* pOrientation, uint32_t mpuIntPin, mpu6050drv_sensorOffset_t const* pOffset)
{
    mpu6050drv_init_t init;

    init.intPin = mpuIntPin;
    init.pOrientation = pOrientation;
    init.handler = mpuEventHandler;
    init.pOffset = pOffset;

    return mpu_Init(&init);
}

/**< helper function to get the required power state for the helena base driver */
static hbd_sleepMode_t getDriverState(state_t const* pState)
{
    return pState->powerMode == BRD_PM_IDLE ? HBD_SLEEP_MODE_OFF : HBD_SLEEP_MODE_ON;
}

/**< helper function to determine if any channel in the current light mode needs the imu */
static bool isImuLightMode(cht_5int3mode_t const* pLightModeChannels)
{
    // if imu is not present, no channel should use imu mode (wouldn't pass validation at startup)
    if (!pSetup->isImuAvailable)
        return false;

    for (uint8_t i = 0; i < features.channelCount; i++)
    {
        if (pLightModeChannels[i].mode)
            return true;
    }

    return false;
}

/**< helper function to determine the required imu mode */
static mpu6050drv_powerMode_t getImuState(uint8_t powerMode, cht_5int3mode_t const* pLightModeChannels)
{
    if (!pSetup->isImuAvailable || powerMode == BRD_PM_OFF)
        return MPU6050_PWR_OFF;
    if (powerMode == BRD_PM_STANDBY)
        return MPU6050_PWR_STANDBY;

    // in idle mode state depends if selected light mode uses imu
    return isImuLightMode(pLightModeChannels) ? MPU6050_PWR_ON : MPU6050_PWR_STANDBY;
}

/** @brief helper function to generate an array with the current channels config
 *
 * @param[in]  pState              the state to generate config for
 * @param[out] plightModeChannels  pointer to array with size of CHANNEL_CNT
 */
static void getLightModeChannels(state_t const* pState, cht_5int3mode_t* plightModeChannels)
{
    uint16_t size = sizeof(cht_5int3mode_t) * features.channelCount;
    memset(plightModeChannels, 0, size);

    // if overrideConfig is set, this is the current light mode
    if (memcmp(pState->ovverrideConfig, plightModeChannels, size) != 0)
    {
        memcpy(plightModeChannels, pState->ovverrideConfig, size);
        return;
    }

    // otherwise get the current light mode from channel config
    uint8_t lightMode = pState->lightMode;
    if (lightMode == MM_MODE_OFF)
        lightMode = mm_GetOffMode();
    if (lightMode != MM_MODE_OFF)
    {
        for (uint8_t i = 0; i < features.channelCount; i++)
        {
            plightModeChannels[i] = pChannels[i * MM_NUM_OF_MODES + lightMode];
        }
    }
}

/** @brief helper function to determine if a helena base driver is used
 *
 * @param[in] pChannels pointer to channel config of size 2
 */
static bool isDriverUsed(cht_5int3mode_t const* pChannels)
{
    return pChannels[CHANNEL_FLOOD].intensity || pChannels[CHANNEL_SPOT].intensity;
}

/**< helper function to determine the required timer ticks */
static uint32_t getTimerState(brd_powerMode_t powerMode, cht_5int3mode_t* plightModeChannels)
{
    // timer is off in off mode and standby
    if (powerMode <= BRD_PM_STANDBY)
        return 0;

    // if at least one driver is used, fast timer is necessary (for limiter)
    for (uint8_t i = 0; i < ARRAY_SIZE(helenaInst) && helenaInst[i].address != 0; i++)
    {
        if (isDriverUsed(&plightModeChannels[i * 2]))
            return TIMER_PERIOD_FAST;
    }

    // if no driver is used 1Hz timer is sufficient
    return TIMER_PERIOD_SLOW;
}

/** @brief helper to check if the light mode is a mode which uses pitch
 *  compensation on channels with 15° and 30°
 */
static bool isMultiOptic(cht_5int3mode_t const* pModeChannels)
{
    bool is15 = false, is30 = false;

    if (pModeChannels == NULL)
        return false;

    for (uint8_t i = 0; i < features.channelCount; i++)
    {
        if (pModeChannels[i].mode != CHMODE_PITCH)  // ignore if pitch compensation not used for this mode and channel
            continue;

        if (pSetup->channelSetup[i].optic.type == PRG_TYPE_15)
            is15 = true;
        else if (pSetup->channelSetup[i].optic.type == PRG_TYPE_30)
            is30 = true;
    }

    return is15 && is30;
}

static void limiterInit(q5_11_t voltage)
{
    q15_16_t fullPower = 0;

    for (channelTypes_t i = 0; i < features.channelCount; i++)
    {
        limiter[i].fullPower = (int32_t)pSetup->channelSetup[i].outputPower << 6;
        fullPower += limiter[i].fullPower;
    }

    for (channelTypes_t i = 0; i < features.channelCount; i++)
    {
        // a channel with less than 5% of the total output power gets the
        // highest priority regardless of the configured optic
        if (pSetup->channelSetup[i].outputPower * 20 > fullPower)
            limiter[i].priority = LIM_PRIO_HIGH;
        // spot drivers get mid priority
        else if (pSetup->channelSetup[i].optic.type == PRG_TYPE_15)
            limiter[i].priority = LIM_PRIO_MID;
        // all others get low priority
        else
            limiter[i].priority = LIM_PRIO_LOW;
    }

    batteryCellCnt = lim_CalcLiionCellCnt(voltage << 5);
}

/** @brief function to update states of driver, imu and timer
 *
 * @note this function is called every time the power state, the light mode or the override mode is changed
 */
static void updateState()
{
    cht_5int3mode_t oldLightModeConfig[CHANNEL_CNT], newLightModeConfig[CHANNEL_CNT];
    getLightModeChannels(&currentState, oldLightModeConfig);
    getLightModeChannels(&pendingState, newLightModeConfig);

    // driver state
    hbd_sleepMode_t oldDriverState, newDriverState;
    oldDriverState = getDriverState(&currentState);
    newDriverState = getDriverState(&pendingState);
    if (newDriverState != oldDriverState)
    {
        hbd_config_t config =
        {
            .sleepMode = newDriverState,
            .sampleRate = HBD_SAMPLERATE_1SPS
        };
        for (uint8_t i = 0; i < ARRAY_SIZE(helenaInst) && helenaInst[i].address != 0; i++)
        {
            hbd_retVal_t errCode = hbd_SetConfig(&helenaInst[i], &config);
            if (errCode != HBD_SUCCESS)
                LOG_ERROR("[brd]: error %d setting config for driver %d", errCode, i);
        }
    }

    // imu state
    if (pSetup->isImuAvailable)
    {
        mpu6050drv_powerMode_t oldImuState, newImuState;
        oldImuState = getImuState(currentState.powerMode, oldLightModeConfig);
        newImuState = getImuState(pendingState.powerMode, newLightModeConfig);
        if (newImuState != oldImuState)
        {
            ret_code_t errCode = mpu_SetPowerMode(newImuState);
            if (errCode != NRF_SUCCESS)
                LOG_ERROR("[brd]: error %d setting mpu power mode", errCode);
        }
    }

    // timer state
    uint32_t oldTimerTicks, newTimerTicks;
    oldTimerTicks = getTimerState(currentState.powerMode, oldLightModeConfig);
    newTimerTicks = getTimerState(pendingState.powerMode, newLightModeConfig);
    if (oldTimerTicks != newTimerTicks)
    {
        app_timer_stop(sampleTimer);
        updateFlag = false;

        if (newTimerTicks != 0)
        {
            ret_code_t errCode = app_timer_start(sampleTimer, newTimerTicks, NULL);
            if (errCode != NRF_SUCCESS)
                LOG_ERROR("[brd]: timer start error %d", errCode);
            oneSecPreload = MAIN_TIMER_TICKS(1000) / newTimerTicks;
            oneSecPrescale = oneSecPreload;
            updateFlag = true;
        }
    }

    currentState = pendingState;
    currentState.isMultiOpticMode = isMultiOptic(newLightModeConfig);
}

/**< helper function to generate targets */
static void getTargets(q8_8_t* pTargets, cht_5int3mode_t const* pChannels, bool isMultiOpticMode)
{
    for (uint8_t i = 0; i < features.channelCount; i++)
    {
        int32_t target = (pChannels[i].intensity * (1ul << 14)) / 5;

        if (pChannels[i].mode == CHMODE_PITCH)
        {
            (void)prg_GetComp(pitch, &pSetup->channelSetup[i].optic, &target);
            if (isMultiOpticMode)
                (void)prg_MultiOptic(pitch, &pSetup->channelSetup[i].optic, &target);
        }

        if (target >= UINT16_MAX)
            pTargets[i] = UINT16_MAX;
        else if (target < 0)
            pTargets[i] = 0;
        else
            pTargets[i] = target;
    }
}

static void sendHpsMessage(q5_11_t inputVoltage, hbd_samplingData_t const* pDriverData, uint8_t driverCnt)
{
    if (currentState.powerMode != BRD_PM_IDLE)
        return; // don't send any messages in standby mode

    ret_code_t errCode;
    btle_hpsMeasurement_t message;
    uint32_t power;

    message.mode = mm_GetCurrentMode();
    message.inputVoltage = (1000ul * inputVoltage) >> 11;
    if (driverCnt)
        message.temperature = (int16_t)(pDriverData->temperature >> 4) - 273;   // use temperature from first driver
    else
        message.temperature = INT8_MIN;

    message.outputPower = 0;
    for (uint8_t i = 0; i < driverCnt; i++)
    {
        power = pSetup->channelSetup[i + 0].outputPower;
        power *= pDriverData[i].currentLeft.current;
        power /= MAX_CURRENT;
        power *= 1000;
        power >>= 10;
        message.outputPower += power;

        power = pSetup->channelSetup[i + 1].outputPower;
        power *= pDriverData[i].currentRight.current;
        power /= MAX_CURRENT;
        power *= 1000;
        power >>= 10;
        message.outputPower += power;
    }

    errCode = btle_ReportHpsMeasurements(&message);
    if (errCode != NRF_SUCCESS &&
        errCode != NRF_ERROR_INVALID_STATE && // ignore error if notifications are not enabled
        errCode != BLE_ERROR_NO_TX_PACKETS)   // ignore error if message buffer is full
    {
        LOG_ERROR("[BRD]: error %d sending message", errCode);
    }
}

static void limitTargets(q15_16_t voltage, q15_16_t temp, q8_8_t* pTargets)
{
    ret_code_t errCode;
    lim_channelReq_t targetsLimited[CHANNEL_CNT];
    q15_16_t power = 0, voltLimit, tempLimit;

    // limit to hardware / setup
    for (uint_fast8_t i = 0; i < features.channelCount; i++)
    {
        if (pTargets[i] > pSetup->channelSetup[i].outputLimit)
            pTargets[i] = pSetup->channelSetup[i].outputLimit;
    }

    // calculate power limit
    for (uint_fast8_t i = 0; i < features.channelCount; i++)
    {
        power += limiter[i].fullPower;
    }
    voltLimit = lim_CalcLiionVoltageLimit(voltage, batteryCellCnt);
    tempLimit = lim_CalcTemperatureLimit(temp);

    if (voltLimit < tempLimit)
        power = ((int64_t)power * voltLimit) >> 16;
    else
        power = ((int64_t)power * tempLimit) >> 16;

    // convert and prepare data for limiter
    isLimiting = false;
    for (uint8_t i = 0; i < features.channelCount; i++)
    {
        targetsLimited[i].request = pTargets[i];
        targetsLimited[i].limiting = false;
    }

    // limit
    errCode = lim_LimitChannels(power, limiter, targetsLimited, features.channelCount);
    if (errCode != NRF_SUCCESS)
        LOG_ERROR("[brd]: limiter error %d", errCode);

    // reconvert
    for (uint8_t i = 0; i < features.channelCount; i++)
    {
        pTargets[i] = targetsLimited[i].request;
        isLimiting |= targetsLimited[i].limiting;
    }
}

/** @brief function to check is current mode is using any channel
 *
 * @param[in] pModeChannels channel configurations of current mode
 * @return true if at least one channel has an intensity value of not 0
 */
static bool isModeUsed(cht_5int3mode_t const* pModeChannels)
{
    for (uint8_t i = 0; i < features.channelCount; i++)
    {
        if (pModeChannels[i].intensity)
            return true;
    }
    return false;
}

/** @brief update target function
 *
 * @note all periodic work is done here, it is basically the boards main loop
 */
static void updateTarget()
{
    //LOG_INFO("[brd]: updating...");

    cht_5int3mode_t lightModeConfig[CHANNEL_CNT];
    q8_8_t targets[CHANNEL_CNT];
    hbd_retVal_t hbdCode;
    hbd_samplingData_t helenaData[DRIVER_CNT];
    q15_16_t voltage, temp;

    voltage = fil_LowPassGet(&filterVolt);
    getLightModeChannels(&currentState, lightModeConfig);

    if (features.channelCount)
    {
        // adc and imu sampling not necessary, values are automatically reported through interrupt handler

        // calculate target values
        getTargets(targets, lightModeConfig, currentState.isMultiOpticMode);

        // read driver and adc
        for (uint8_t i = 0; i < ARRAY_SIZE(helenaInst) && helenaInst[i].address != 0; i++)
        {
            hbdCode = hbd_ReadSamplingData(&helenaInst[i], &helenaData[i]);
            if (hbdCode != HBD_SUCCESS && hbdCode != HBD_ERROR_FORBIDDEN)
                LOG_ERROR("[brd]: hbd_ReadSamlingData() error %d for driver %d", hbdCode, i);
        }

        if (oneSecPreload == ONE_SEC_PRESCALE_SLOW)
            fil_LowPassReset(&filterTemp, helenaData[0].temperature << 12);
        else
            fil_LowPassFeed(&filterTemp, helenaData[0].temperature << 12);
        temp = fil_LowPassGet(&filterTemp);


        // run through limiter
        limitTargets(voltage, temp, targets);

        /*LOG_INFO("[brd]: %d, %d, %d, %d, %d, %d",
                 (voltage * 1000) >> 16, (temp >> 16) - 273,
                 lightModeConfig[0].intensity, lightModeConfig[1].intensity,
                 targets[0] >> 8, targets[1] >> 8);*/

        // update driver
        for (uint8_t i = 0; i < ARRAY_SIZE(helenaInst) && helenaInst[i].address != 0; i++)
        {
            hbdCode = hbd_SetTargetCurrent(&helenaInst[i], voltage >> 5, targets[i * 2 + CHANNEL_FLOOD] >> 8, targets[i * 2 + CHANNEL_SPOT] >> 8);
            if (hbdCode != HBD_SUCCESS && hbdCode != HBD_ERROR_FORBIDDEN)
                LOG_ERROR("[brd]: hbd_SetTargetCurrent() error %d for dirver %d", hbdCode, i);
        }
    }

    if (oneSecPrescale == 0)
    {
        // sending message
        sendHpsMessage(voltage >> 5, helenaData, pSetup->driverCnt);

        // without acceleration sensor idle timer is reset when light is used (and not in an active OFF mode)
        if (mm_GetCurrentMode() != MM_MODE_OFF && isModeUsed(lightModeConfig))
            main_ResetIdleTimer();

        oneSecPrescale = oneSecPreload;
    }

    updateFlag = false;
}

static void setBoardInfo(brd_info_t const* *ppInfo, hln_brd_t const* pBoard, setup_t const* pSetup, uint8_t driverCnt)
{
    // brdInfo.pInfo points to bleInfo
    bleInfo.pDevicename = pSetup->deviceName;
    bleInfo.pBoardHWVersion = pBoard->pHwRevString;

    // brdInfo.pFeatures points to features
    // features.channelCount = driverCnt * 2;
    features.pComSupported = pSetup->comPinMode == COMPIN_MODE_COM ? &pBoard->comPins : NULL;

    // brdInfo.pModes points to modes
    modes.total_size = SIZE_OF_MODES + SIZE_OF_CHANNELS(driverCnt * 2);

    *ppInfo = &brdInfo;
}

static bool hasSetupChanged(bool isImuPresent, uint8_t driverCnt)
{
    if (pSetup->isImuAvailable == isImuPresent && pSetup->driverCnt == driverCnt)
        return false;

    if (!qwrBuffer.inUse)
    {
        qwrBuffer.inUse = true;      // mark buffer to be used

        // prepare data
        setup_str_t *pNewSetup = (setup_str_t*)qwrBuffer.p_block->p_mem;
        pNewSetup->version = SETUP_VERSION;
        pNewSetup->numOfChannels = CHANNEL_CNT;
        pNewSetup->dummy = 0;
        memcpy(&pNewSetup->setup, pSetup, sizeof(setup_t));
        pNewSetup->setup.isImuAvailable = isImuPresent;
        pNewSetup->setup.driverCnt = driverCnt;

        ret_code_t errCode = storePreparedSetup(NULL);
        if (errCode != NRF_SUCCESS)
            LOG_ERROR("[brd]: error %d storing changed setup", errCode);
    }
    else
        LOG_ERROR("[brd]: setup changed, but buffer in use");

    return true;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t brd_Init(brd_info_t const* *pInfo)
{
    ret_code_t errCode;
    hln_brd_t const* pBoard;
    uint8_t driverCnt;
    bool setupChanged, isImuPresent;
    q5_11_t voltage;

    if (pInfo == NULL)
        return NRF_ERROR_NULL;

    //*pInfo = &brdInfo;

    errCode = cleanUpOldRevData();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = timerInit();
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = hln_brd_Init(&pBoard);
    if (errCode != NRF_SUCCESS)
        return errCode;
    //bleInfo.pBoardHWVersion = pBoard->pHwRevString;

    errCode = initI2c(pBoard);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initHelenaDriver(&driverCnt);
    if (errCode != NRF_SUCCESS && errCode != NRF_ERROR_NOT_FOUND)
        return errCode;

    features.channelCount = driverCnt * 2;  /// TODO: find better solution. channel count needs to be already set here, otherwise setup verification will fail
    errCode = loadSetup();
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;
    //bleInfo.pDevicename = pSetup->deviceName;

    isImuPresent = initMpu(pBoard->imuCfg.imuOrient, pBoard->imuCfg.intPin, &pSetup->imuOffset) == NRF_SUCCESS;

    errCode = loadChannels(DEFAULT_SET, isImuPresent);
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;

    errCode = hln_hmi_Init(pBoard, pSetup->comPinMode == COMPIN_MODE_BUTTON);
    if (errCode != NRF_SUCCESS)
        return errCode;

    errCode = initAdc(pBoard->analogeVin, &voltage);
    if (errCode != NRF_SUCCESS)
        return errCode;

    setBoardInfo(pInfo, pBoard, pSetup, driverCnt);

    limiterInit(voltage);

    setupChanged = hasSetupChanged(isImuPresent, driverCnt);

    if (setupChanged)
    {
        /// TODO: board features or info need an indicator flag (or maybe a btle function) to send a service change indication
        LOG_INFO("[brd]: setup has changed");
    }

    errCode = hln_btle_Init(isImuPresent);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

ret_code_t brd_SetPowerMode(brd_powerMode_t newMode)
{
    pendingState.powerMode = newMode;

    memset(pendingState.ovverrideConfig, 0, sizeof(pendingState.ovverrideConfig));

    return NRF_SUCCESS;
}

ret_code_t brd_SetLightMode(uint8_t newMode)
{
    pendingState.lightMode = newMode;

    memset(pendingState.ovverrideConfig, 0, sizeof(pendingState.ovverrideConfig));

    return NRF_SUCCESS;
}

ret_code_t brd_OverrideMode(ble_hps_cp_channel_config_t const* pConfig)
{
    if (pConfig->size != features.channelCount)
        return NRF_ERROR_INVALID_LENGTH;

    memcpy(pendingState.ovverrideConfig, pConfig->p_config, sizeof(pendingState.ovverrideConfig));

    return NRF_SUCCESS;
}

bool brd_Execute(void)
{
    storageExecute();
    hln_hmi_Execute();
    mpu_Execute();

    // ignore multi optic indicator for comparison
    if (currentState.powerMode != pendingState.powerMode ||
        currentState.lightMode != pendingState.lightMode ||
        memcmp(&currentState.ovverrideConfig, &pendingState.ovverrideConfig, sizeof(pendingState.ovverrideConfig)) != 0)
        updateState();

    if (updateFlag)
        updateTarget();

    return isLimiting;
}

ret_code_t brd_GetChannelConfig(void const** ppData, uint16_t* pSize)
{
    if (ppData == NULL || pSize == NULL)
        return NRF_ERROR_NULL;

    *ppData = pChannels;
    *pSize = SIZE_OF_CHANNELS(features.channelCount);

    return NRF_SUCCESS;
}

ret_code_t brd_SetChannelConfig(void const* pData, uint16_t size, ds_reportHandler_t resultHandler)
{
    if (pData == NULL)
        return NRF_ERROR_NULL;

    if (size != SIZE_OF_CHANNELS(features.channelCount))
        return NRF_ERROR_INVALID_LENGTH;

    if (!isChannelConfigValid(pData, MM_NUM_OF_MODES, pSetup->isImuAvailable))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(pChannels, pData, SIZE_OF_CHANNELS(features.channelCount));

    pendingResultHandler = resultHandler;

    if (qwrBuffer.inUse)
    {
        LOG_INFO("[brd]: buffer in use");
        return NRF_SUCCESS;
    }
    else
    {
        LOG_INFO("[brd]: buffer free");
        ret_code_t errCode = storeChannels(DEFAULT_SET);
        if (errCode != NRF_SUCCESS)
        {
            pendingResultHandler = STORE_IDLE;
            LOG_INFO("[brd]: storing error %d", errCode);
        }
        return errCode;
    }
}

ret_code_t brd_SetDeviceName(char const* pNewName, ds_reportHandler_t resultHandler)
{
    if (pNewName == NULL)
        return NRF_ERROR_NULL;

    if (MAX_NAME_LENGTH == 0)
        return NRF_ERROR_NOT_SUPPORTED;

    if (strlen(pNewName) > MAX_NAME_LENGTH)
        return NRF_ERROR_INVALID_PARAM;

    if (qwrBuffer.inUse)
        return NRF_ERROR_BUSY;

    qwrBuffer.inUse = true;      // mark buffer to be used

    // prepare data
    setup_str_t *pNewSetup = (setup_str_t*)qwrBuffer.p_block->p_mem;
    pNewSetup->version = SETUP_VERSION;
    pNewSetup->numOfChannels = CHANNEL_CNT;
    pNewSetup->dummy = 0;
    memcpy(&pNewSetup->setup, pSetup, sizeof(setup_t));
    strcpy(pNewSetup->setup.deviceName, pNewName);

    return storePreparedSetup(resultHandler);
}

ret_code_t brd_FactoryReset(ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    // set default channel configuration
    memcpy(pChannels, channelDefaults, SIZE_OF_CHANNELS(CHANNEL_CNT));
    // and default setup
    pSetup = &setupDefaults;

    return ds_Reset(HELENA_FILE_ID, resultHandler);
}

ret_code_t hln_GetChannelSetup(hln_channelSetup_t* pChannelSetup, uint8_t channel)
{
    if (pSetup == NULL)
        return NRF_ERROR_NULL;

    if (channel > features.channelCount)
        return NRF_ERROR_INVALID_PARAM;

    memcpy(pChannelSetup, &pSetup->channelSetup[channel], sizeof(hln_channelSetup_t));

    return NRF_SUCCESS;
}

ret_code_t hln_SetChannelSetup(hln_channelSetup_t const* pChannelSetup, uint8_t channel, ds_reportHandler_t resultHandler)
{
    if (pChannelSetup == NULL)
        return NRF_ERROR_NULL;

    if (qwrBuffer.inUse)
        return NRF_ERROR_BUSY;

    if (channel > features.channelCount || !isChannelSetupValid(pChannelSetup, 1))
        return NRF_ERROR_INVALID_PARAM;

    qwrBuffer.inUse = true;      // mark buffer to be used

    // prepare data
    setup_str_t *pNewSetup = (setup_str_t*)qwrBuffer.p_block->p_mem;
    pNewSetup->version = SETUP_VERSION;
    pNewSetup->numOfChannels = CHANNEL_CNT;
    pNewSetup->dummy = 0;
    memcpy(&pNewSetup->setup, pSetup, sizeof(setup_t));
    memcpy(&pNewSetup->setup.channelSetup[channel], pChannelSetup, sizeof(hln_channelSetup_t));

    return storePreparedSetup(resultHandler);
}

ret_code_t hln_GetComPinMode(hln_comPinMode_t* pMode)
{
    if (pMode == NULL)
        return NRF_ERROR_NULL;

    *pMode = pSetup->comPinMode;

    return NRF_SUCCESS;
}

ret_code_t hln_SetComPinMode(hln_comPinMode_t mode, ds_reportHandler_t resultHandler)
{
    if (qwrBuffer.inUse)
        return NRF_ERROR_BUSY;

    if (!isComPinModeValid(mode))
        return NRF_ERROR_INVALID_PARAM;

    LOG_INFO("[brd]: new com config %d", mode);

    qwrBuffer.inUse = true;      // mark buffer to be used

    // prepare data
    setup_str_t *pNewSetup = (setup_str_t*)qwrBuffer.p_block->p_mem;
    pNewSetup->version = SETUP_VERSION;
    pNewSetup->numOfChannels = CHANNEL_CNT;
    pNewSetup->dummy = 0;
    memcpy(&pNewSetup->setup, pSetup, sizeof(setup_t));
    pNewSetup->setup.comPinMode = mode;

    return storePreparedSetup(resultHandler);
}

/// TODO: Both of these functions are using i2c communication in blocking mode and should not be called in interrupt context
ret_code_t hln_GetCompensation(hbd_calibData_t* pComp, uint8_t driver)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t hln_SetCompensation(hbd_calibData_t* pComp, uint8_t driver, ds_reportHandler_t resultHandler)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t hln_IsImuCalibrated(bool* pIsCalibrated)
{
    if (pIsCalibrated == NULL)
        return NRF_ERROR_NULL;

    if (!pSetup->isImuAvailable)
        return NRF_ERROR_NOT_SUPPORTED;

    *pIsCalibrated = false;
    for (uint8_t i = 0; i < 3; i++)
    {
        if (pSetup->imuOffset.accel[i] != 0 || pSetup->imuOffset.gyro[i] != 0)
        {
            *pIsCalibrated = true;
            break;
        }
    }

    return NRF_SUCCESS;
}

ret_code_t hln_CalibrateImu(ds_reportHandler_t resultHandler)
{
    if (qwrBuffer.inUse)
        return NRF_ERROR_BUSY;

    if (!pSetup->isImuAvailable)
        return NRF_ERROR_NOT_SUPPORTED;

    LOG_INFO("[brd]: calibrating imu");

    mpu6050drv_sensorOffset_t newOffset;
    memcpy(&newOffset, &pSetup->imuOffset, sizeof(mpu6050drv_sensorOffset_t));
    ret_code_t errCode = mpu_CalibrateSensorOffset(&newOffset);
    if (errCode != NRF_SUCCESS)
        return errCode;

    qwrBuffer.inUse = true;      // mark buffer to be used

    // prepare data
    setup_str_t *pNewSetup = (setup_str_t*)qwrBuffer.p_block->p_mem;
    pNewSetup->version = SETUP_VERSION;
    pNewSetup->numOfChannels = CHANNEL_CNT;
    pNewSetup->dummy = 0;
    memcpy(&pNewSetup->setup, pSetup, sizeof(setup_t));
    memcpy(&pNewSetup->setup.imuOffset, &newOffset, sizeof(newOffset));

    return storePreparedSetup(resultHandler);
}

/**END OF FILE*****************************************************************/
