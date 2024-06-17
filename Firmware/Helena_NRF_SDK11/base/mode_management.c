/**
  ******************************************************************************
  * @file    mode management.c
  * @author  Thomas Reisnecker
  * @brief   mode management module
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define MM_LOG_ENABLED

#ifdef MM_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // MM_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include "nordic_common.h"
#include "ble_types.h"
#include "string.h"
#include "mode_management.h"
#include "btle.h"
#include "com_message_handling.h"
#include "debug.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint8_t numOfModes;     // the number of modes stored
    uint8_t version;        // mode version (to convert if changed in future)
    uint16_t dummy;         // dummy bytes for alignment
    mm_modeConfig_t modes[MM_NUM_OF_MODES];
} storageFormat_t;

typedef struct
{
    uint32_t       isValid;     // valid pattern to check if noinit data needs to be initialized
    uint8_t        currentMode; // containing the current mode
    uint8_t        lastMode;    // last mode in case of temporary mode
} mmState_t;

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)       (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/
#define VALID_PATTERN       0x30d334bd
#define STORE_IDLE          (ds_reportHandler_t)0xFFFFFFFF

#define MODES_FILE_ID       0x30de
#define MODES_RECORD_BASE   0x0001

#define MODES_VERSION       0
#define DEFAULT_SET         0

#define MODES_DEFAULTS                          \
{ /* ignore first  last   pref   temp   off   */\
    {false, true,  false, false, false, false}, \
    {false, false, true,  false, false, false}, \
    {false, true,  false, false, false, false}, \
    {false, false, true,  false, false, false}, \
    {true,  true,  false, false, false, false}, \
    {true,  false, false, false, false, false}, \
    {true,  false, false, false, false, false}, \
    {true,  false, true,  false, false, false}, \
}

#define MODES_STR_DEFAULTS                      \
{                                               \
    .numOfModes = MM_NUM_OF_MODES,              \
    .version    = MODES_VERSION,                \
}

/* Private function prototype ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static mm_modeChangedHandler_t modeChangeHandler;
static mmState_t               state __attribute__((section(".noinit")));

//static mm_modeConfig_t         modes[MM_NUM_OF_MODES] = MODES_DEFAULTS;         /// TODO: can pointer to memory for gatt table be used here?
static mm_modeConfig_t*        pModes;

//static storageFormat_t         newModes  __ALIGN(4) = MODES_STR_DEFAULTS;
static ble_hps_qwr_buffer_t*   pBuffer;             // pointer to the shared buffer for queued writes
static ds_reportHandler_t      pendingResultHandler = STORE_IDLE;// the resultHandler to call after storage operation

/* Private functions ---------------------------------------------------------*/
/** @brief function to jump to the next mode
 *
 * @param[in] currentMode
 * @return    the next available mode
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getNextMode(uint8_t currentMode)
{
    uint8_t newMode;

    // in off mode just use the first available mode
    if (currentMode == MM_MODE_OFF) /// TODO: SOS
    {
        newMode = 0;
        while (pModes[newMode].ignore) {newMode++;}
        return newMode;
    }

    newMode = currentMode;
    do
    {
        // if this is not the last, go to the next mode
        if (!pModes[newMode].lastInGroup)
            newMode++;
        // otherwise roll back to the first
        else
        {
            while (!pModes[newMode].firstInGroup) {newMode--;}
        }
        // check if the mode is enabled
        if (!pModes[newMode].ignore)
            return newMode;
    } while (newMode != currentMode); // in case of no available modes in current group

    // if there are no active modes in the current group, just go to the next
    // available mode ignoring group limits
    while (pModes[newMode].ignore)
    {
        if (++newMode >= MM_NUM_OF_MODES)
            newMode = 0;
    }

    return newMode;
}

/** @brief function to jump to the previous mode
 *
 * @param[in] currentMode
 * @return    the previous available mode
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getPreviousMode(uint8_t currentMode)
{
    uint8_t newMode;

    // in off mode just use the first available mode
    if (currentMode == MM_MODE_OFF) /// TODO: SOSO
    {
        newMode = MM_NUM_OF_MODES - 1;
        while (pModes[newMode].ignore) {newMode--;}
        return newMode;
    }

    newMode = currentMode;
    do
    {
        // if this is not the first, go to the previous mode
        if (!pModes[newMode].firstInGroup)
            newMode--;
        // otherwise roll back to the last
        else
        {
            while (!pModes[newMode].lastInGroup) {newMode++;}
        }
        // check if the mode is enabled
        if (!pModes[newMode].ignore)
            return newMode;
    } while (newMode != currentMode); // in case of no available modes in current group

    // if there are no active modes in the current group, just go to the next
    // available mode ignoring group limits
    while (pModes[newMode].ignore)
    {
        if (newMode-- == 0)
            newMode = MM_NUM_OF_MODES - 1;
    }

    return newMode;
}

/**< returns true is the group containing mode has at least one valid member */
static bool isValidGroup(uint8_t mode)
{
    while (!pModes[mode].firstInGroup) {mode--;}
    while (!pModes[mode].lastInGroup)
    {
        if (!pModes[mode++].ignore)
            return true;
    }
    return false;
}

/** @brief function to jump to the next group
 *
 * @param[in] currentMode
 * @return    the correspondent mode in the next group
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getNextGroup(uint8_t currentMode)
{
    uint8_t posInGroup = 0;

    // in off mode start with the first available mode
    if (currentMode == MM_MODE_OFF)  /// TODO: SOS
    {
        currentMode = 0;
        while (pModes[currentMode].ignore)
        {
            currentMode++;
            posInGroup++;
        }
    }
    // if not off, determine the position of the current mode in its group
    else
    {
        while (!pModes[currentMode].firstInGroup)
        {
            currentMode--;
            posInGroup++;
        }
    }

    // now go to the first mode in the next valid group
    do
    {
        while (!pModes[currentMode++].lastInGroup) {}
        if (currentMode >= MM_NUM_OF_MODES)    // roll over to first mode
            currentMode = 0;
    } while (!isValidGroup(currentMode));

    // finally go to the same position in this group (if possible)
    while (posInGroup-- && !pModes[currentMode].lastInGroup) {currentMode++;}

    if (pModes[currentMode].ignore)
        currentMode = getNextMode(currentMode);

    return currentMode;
}

/** @brief function to jump to the previous group
 *
 * @param[in] currentMode
 * @return    the correspondent mode in the previous group
 *
 * @note this function relies on a correct setup with at least one available
 *       mode and correctly set up group boarders, otherwise it will end up in
 *       a dead loop or hard fault
 */
static uint8_t getPreviousGroup(uint8_t currentMode)
{
    uint8_t posInGroup = 0;

    // in off mode start with the first available mode
    if (currentMode == MM_MODE_OFF) /// TODO: SOS
    {
        currentMode = MM_NUM_OF_MODES - 1;
        while (pModes[currentMode].ignore) {currentMode--;}
    }
    // if not off, determine the position of the current mode in its group, referenced to the last member
    else
    {
        while (!pModes[currentMode].lastInGroup)
        {
            currentMode++;
            posInGroup++;
        }
    }

    // now go to the last mode in the previous valid group
    do
    {
        while (!pModes[currentMode--].firstInGroup) {}
        if (currentMode >= MM_NUM_OF_MODES)    // roll over to last mode
            currentMode = MM_NUM_OF_MODES - 1;
    } while (!isValidGroup(currentMode));

    // finally go to the same position in this group (if possible)
    while (posInGroup-- && !pModes[currentMode].firstInGroup) {currentMode--;}

    if (pModes[currentMode].ignore)
        currentMode = getPreviousMode(currentMode);

    return currentMode;
}

static uint8_t getOffMode()
{
    for (uint_fast8_t i = 0; i < MM_NUM_OF_MODES; i++)
    {
        if (pModes[i].isOffMode)
            return i;
    }
    return MM_MODE_OFF;
}

static uint8_t getPrefMode()
{
    for (uint_fast8_t i = 0; i < MM_NUM_OF_MODES; i++)
    {
        if (pModes[i].isPrefMode)
            return i;
    }
    return MM_MODE_OFF;
}

static uint8_t getTempMode()
{
    for (uint_fast8_t i = 0; i < MM_NUM_OF_MODES; i++)
    {
        if (pModes[i].isTempMode)
            return i;
    }
    return MM_MODE_OFF;
}

/*static uint8_t sizeOfGroup(uint8_t mode)
{
    uint8_t size = 1;
    while (!modeCfg.pGeneral[mode--].firstInGroup) {}
    while (!modeCfg.pGeneral[mode++].lastInGroup)
    {
        size++;
    }
    return size;
}*/

static bool isModeConfigValid(mm_modeConfig_t const* pModesToCheck)
{
    int8_t group = 0;
    uint8_t temp = 0, pref = 0, off = 0;
    for (uint_fast8_t i = 0; i < MM_NUM_OF_MODES; i++)
    {
        if (pModesToCheck[i].firstInGroup)
            group++;
        if (pModesToCheck[i].lastInGroup)
            group--;
        if (pModesToCheck[i].isPrefMode)
            pref++;
        if (pModesToCheck[i].isTempMode)
            temp++;
        if (pModesToCheck[i].isOffMode)
            off++;

        if (group > 1 || group < 0 || pref > 1 || temp > 1 || off > 1)
            return false;
    }
    return true;
}

static ret_code_t loadModes(uint8_t set)
{
    ret_code_t             errCode;
    storageFormat_t const* pModesInMem;
    uint16_t               lengthWords;

    errCode = ds_Read(MODES_FILE_ID, MODES_RECORD_BASE + set, (void const**)&pModesInMem, &lengthWords);
    if (errCode == NRF_SUCCESS)
    {
        if (lengthWords == BYTES_TO_WORDS(sizeof(storageFormat_t)) &&
            isModeConfigValid(pModesInMem->modes))
        {
            memcpy(pModes, pModesInMem->modes, sizeof(mm_modeConfig_t) * MM_NUM_OF_MODES);
            return NRF_SUCCESS;
        }
    }
    else if (errCode != FDS_ERR_NOT_FOUND)
    {
        LOG_ERROR("[mm]: mode record load error %d", errCode);
    }

    // no modes stored yet or invalid, load defaults
    mm_modeConfig_t modes[MM_NUM_OF_MODES] = MODES_DEFAULTS;
    memcpy(pModes, modes, sizeof(modes));

    return errCode;
}

static void storageResultHandler(ret_code_t errCode)
{
    LOG_INFO("[mm]: storage result %d", errCode);
    pBuffer->inUse = false;     // release buffer

    if (pendingResultHandler == STORE_IDLE)
    {
        LOG_ERROR("[mm]: unexpected storage event");
    }
    else if (pendingResultHandler)
    {
        pendingResultHandler(errCode); // relay event to original handler
    }

    pendingResultHandler = STORE_IDLE;
}

static ret_code_t storeModes(uint8_t set)
{
    ret_code_t errCode;
    void const* pData;
    uint16_t lenghtInWords;

    pBuffer->inUse = true;      // mark buffer to be used

    // prepare data
    /// TODO: alignment !!!
    storageFormat_t *pNewModes = (storageFormat_t*)pBuffer->p_block->p_mem;
    pNewModes->version = MODES_VERSION;
    pNewModes->numOfModes = MM_NUM_OF_MODES;
    pNewModes->dummy = 0;
    memcpy(pNewModes->modes, pModes, sizeof(mm_modeConfig_t) * MM_NUM_OF_MODES);

    pData         = (void const*)pNewModes;
    lenghtInWords = BYTES_TO_WORDS(sizeof(storageFormat_t));

    // execute store operation
    errCode = ds_Write(MODES_FILE_ID, MODES_RECORD_BASE + set, pData, lenghtInWords, storageResultHandler);

    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[mm]: error %d storing modes", errCode);

        pBuffer->inUse = false;
    }

    return errCode;
}

/*static ret_code_t deleteModes(uint8_t set)
{
    /// HINT: if more sets are used and they should be deleted all use fds_file_delete()

    ret_code_t errCode;
    fds_record_desc_t desc;
    fds_find_token_t  tok = {0};

    errCode = fds_record_find(MODES_FILE_ID, MODES_RECORD_BASE + set, &desc, &tok);
    if (errCode == NRF_SUCCESS)
        errCode = fds_record_delete(&desc);
    else if(errCode == FDS_ERR_NOT_FOUND)
        errCode = NRF_SUCCESS;

    return errCode;
}*/

static bool isValidMode(uint8_t mode)
{
    if (mode == MM_MODE_OFF || /* mode == MM_MODE_SOS || */
        mode < MM_NUM_OF_MODES)
        return true;
    else
        return false;
}

/* Public functions ----------------------------------------------------------*/
ret_code_t mm_Init(mm_init_t const* pInit, uint8_t* pInitMode)
{
    if (pInit == NULL ||
        pInit->modeHandler == NULL ||
        pInit->pModesMem == NULL || pInit->pBuf == NULL ||
        pInitMode == NULL)
        return NRF_ERROR_NULL;

    if (pInit->pBuf->p_block->len < sizeof(storageFormat_t))
        return NRF_ERROR_NO_MEM;

    modeChangeHandler = pInit->modeHandler;
    pModes = pInit->pModesMem;
    pBuffer = pInit->pBuf;

    // check if state is valid, if not initialize
    if (state.isValid != VALID_PATTERN ||
        !isValidMode(state.currentMode) ||
        !isValidMode(state.lastMode))
    {
        state.isValid = VALID_PATTERN;
        state.currentMode = MM_MODE_OFF;
        state.lastMode = MM_MODE_OFF;
    }

    *pInitMode = state.currentMode;

    ret_code_t errCode = loadModes(DEFAULT_SET);
    if (errCode != NRF_SUCCESS && errCode != FDS_ERR_NOT_FOUND)
        return errCode;

    return NRF_SUCCESS;
}

void mm_Execute()
{
    // check if storing operations are pending
    if (pendingResultHandler != STORE_IDLE && pBuffer->inUse == false)
    {
        LOG_INFO("[mm]: buffer released, storing");
        ret_code_t errCode = storeModes(DEFAULT_SET);

        if (errCode != NRF_SUCCESS)
        {
            if (pendingResultHandler)
                pendingResultHandler(errCode);
            pendingResultHandler = STORE_IDLE;
            LOG_INFO("[mm]: storing error %d", errCode);
        }
    }
}

ret_code_t mm_HmiEventHandler(hmi_evt_t const* pEvent)
{
    bool relay = true;
    uint16_t connHandle = BLE_CONN_HANDLE_ALL;

    switch (pEvent->type)
    {
    case HMI_EVT_MODEOFF:
        state.currentMode = MM_MODE_OFF;//getOffMode();
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_MODEPREF:
    {
        uint8_t prefMode = getPrefMode();
        if (state.currentMode == prefMode)
            state.currentMode = MM_MODE_OFF;
        else
            state.currentMode = prefMode;
        state.lastMode = state.currentMode;
    }   break;

    case HMI_EVT_MODETEMP:
    {
        uint8_t tempMode = getTempMode();
        if (tempMode == MM_MODE_OFF)
            break;

        relay = false;
        if (state.currentMode == tempMode)
            state.currentMode = state.lastMode;
        else
            state.currentMode = tempMode;
    }   break;

    case HMI_EVT_MODENUMBER:
        connHandle = pEvent->params.mode.connHandle;
        if (pEvent->params.mode.modeNumber >= MM_NUM_OF_MODES)
            state.currentMode = MM_MODE_OFF;    /// TODO: it's better to relay the received mode (in case of other devices support more modes)
        else
            state.currentMode = pEvent->params.mode.modeNumber;
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_NEXTMODE:
        state.currentMode = getNextMode(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_NEXTGROUP:
        state.currentMode = getNextGroup(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_PREVMODE:
        state.currentMode = getPreviousMode(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    case HMI_EVT_PREVGROUP:
        state.currentMode = getPreviousGroup(state.currentMode);
        state.lastMode = state.currentMode;
        break;

    default:
    case HMI_EVT_MODESOS:
        /// todo:
        break;
    }

    /// TODO: check if mode has changed at all
    if ((state.currentMode == MM_MODE_OFF) && (mm_GetOffMode() != MM_MODE_OFF))
    {
        LOG_INFO("[mm]: new mode %d (%d)", state.currentMode, mm_GetOffMode());
    }
    else
    {
        LOG_INFO("[mm]: new mode %d", state.currentMode);
    }

    if (relay)
    {
        //ret_code_t errCode;
        btle_modeRelay_t relay;

        // if the conn handle is invalid, the mode change was initiated through
        // the wired com, so no relay to com, but to all ble devices
        if (connHandle != BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t errCode = cmh_RelayMode(state.currentMode);
            if (errCode != NRF_SUCCESS &&
                errCode != NRF_ERROR_INVALID_STATE && // no com at all
                errCode != NRF_ERROR_NOT_SUPPORTED)   // no transmitter, just receiving
            {
                LOG_ERROR("[mm]: com relay error %d", errCode);
            }
        }
        else
            connHandle = BLE_CONN_HANDLE_ALL;

        relay.connHandle = connHandle;
        relay.mode = state.currentMode;
        (void)btle_RelayMode(&relay);
    }

    modeChangeHandler(state.currentMode);

    return NRF_SUCCESS;
}

uint8_t mm_GetCurrentMode()
{
    return state.currentMode;
}

uint8_t mm_GetOffMode()
{
    return getOffMode();
}

/*mm_modeConfig_t const* mm_GetModeConfig(uint8_t mode)
{
    if (mode >= ARRAY_SIZE(modes))
        return NULL;
    else
        return &modes[mode];
}*/

ret_code_t mm_GetModeConfigs(mm_modeConfig_t const** ppModeConfigs)
{
    if (ppModeConfigs == NULL)
        return NRF_ERROR_NULL;

    *ppModeConfigs = pModes;

    return NRF_SUCCESS;
}

ret_code_t mm_CheckModeConfig(mm_modeConfig_t const* pModeConfig, uint16_t size)
{
    if (pModeConfig == NULL)
        return NRF_ERROR_NULL;

    if (size != sizeof(mm_modeConfig_t) * MM_NUM_OF_MODES)
        return NRF_ERROR_INVALID_LENGTH;

    return isModeConfigValid(pModeConfig) ? NRF_SUCCESS : NRF_ERROR_INVALID_PARAM;
}

ret_code_t mm_SetModeConfig(mm_modeConfig_t const* pModeConfig, uint16_t size, ds_reportHandler_t resultHandler)
{
    if (pModeConfig == NULL)
        return NRF_ERROR_NULL;

    if (size != sizeof(mm_modeConfig_t) * MM_NUM_OF_MODES)
        return NRF_ERROR_INVALID_LENGTH;

    if (!isModeConfigValid(pModeConfig))
        return NRF_ERROR_INVALID_PARAM;

    memcpy(pModes, pModeConfig, sizeof(mm_modeConfig_t) * MM_NUM_OF_MODES);

    pendingResultHandler = resultHandler;

    if (pBuffer->inUse)
    {
        LOG_INFO("[mm]: buffer in use");
        return NRF_SUCCESS;
    }
    else
    {
        LOG_INFO("[mm]: buffer free");
        ret_code_t errCode = storeModes(DEFAULT_SET);
        if (errCode != NRF_SUCCESS)
        {
            pendingResultHandler = STORE_IDLE;
            LOG_INFO("[mm]: storing error %d", errCode);
        }
        return errCode;
    }
}

ret_code_t mm_FactoryReset(ds_reportHandler_t resultHandler)
{
    if (resultHandler == NULL)
        return NRF_ERROR_NULL;

    // set defaults setting
    mm_modeConfig_t defaults[MM_NUM_OF_MODES] = MODES_DEFAULTS;
    memcpy(pModes, defaults, sizeof(defaults));

    // initiate deletion
    return ds_Reset(MODES_FILE_ID, resultHandler);
}

/**END OF FILE*****************************************************************/
