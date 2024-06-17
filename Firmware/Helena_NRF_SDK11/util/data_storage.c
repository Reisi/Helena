/**
  ******************************************************************************
  * @file    data_storage.c
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* logger configuration ------------------------------------------------------*/
#define DS_LOG_ENABLED

#ifdef DS_LOG_ENABLED
#include "log.h"
#else
#define LOG_ERROR(...)
#define LOG_INFO(...)
#endif // DS_LOG_ENABLED

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>

#include "fds.h"
#include "nrf_log.h"
#include "string.h"
#include "data_storage.h"
#include "debug.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint16_t           fileId;
    uint8_t            cnt;
    ds_reportHandler_t handler;
} userHandlers_t;

/* Private macros ------------------------------------------------------------*/
// id 0x0000 is not invalid, but is used to identify empty handlers and records
#define FILE_ID_FREE    0x0000

#define VERIFY_FILEID(id)               \
{                                       \
    if (id < 0x0001 || id > 0xBFFF)     \
        return NRF_ERROR_INVALID_PARAM; \
}

#define VERIFY_RECORDKEY(key)           \
{                                       \
    if (key < 0x0001 || key > 0xBFFF)   \
        return NRF_ERROR_INVALID_PARAM; \
}

#define VERIFY_PARAM_NOT_NULL(param)    \
{                                       \
    if (param == NULL)                  \
        return NRF_ERROR_NULL;          \
}

#define IS_INIT()                               \
{                                               \
    if (initResult == NRF_ERROR_INVALID_STATE)  \
        init();                                 \
    if (initResult != NRF_SUCCESS)              \
        return initResult;                      \
}

#define ARRAY_SIZE(x)   (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/

/* Private read only variables -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static ret_code_t         initResult = NRF_ERROR_INVALID_STATE;
static userHandlers_t     userHandlers[DS_MAX_USERS];
static fds_record_t       pendingRecords[DS_MAX_USERS];

/* Private functions ---------------------------------------------------------*/
/** @brief function to get the handler structure for a user (identified by its
 *         file ID), use FILE_ID_FREE to get a empty one, returns NULL if not
 *         found or no free one available.
 */
static userHandlers_t* getHandler(uint16_t fileId)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(userHandlers); i++)
    {
        if (userHandlers[i].fileId == fileId)
            return &userHandlers[i];
    }
    return NULL;
}

/** @brief function to send a result. Resets structure if no more events are
 *         pending for this user.
 */
static void sendResult(uint16_t fileId, ret_code_t errCode)
{
    userHandlers_t* pUserHandler = getHandler(fileId);

    //NRF_LOG_INFO("[DS]: result %d for user %d", errCode, fileId);

    if (pUserHandler == NULL)   // no data for this user
        return;

    if (pUserHandler->handler != NULL)  // no handler available
        pUserHandler->handler(errCode);

    // clear if no more events expected for this user
    if (pUserHandler->cnt && --pUserHandler->cnt == 0)
        memset(pUserHandler, 0, sizeof(userHandlers_t));
}

/** @brief function to get the pending records structure for a user (identified
 *         by its file ID), use FILE_ID_FREE to get a empty one, returns NULL
 *         if not found or no free one available.
 */
static fds_record_t* getRecord(uint16_t fileId)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(pendingRecords); i++)
    {
        if (pendingRecords[i].file_id == fileId)
            return &pendingRecords[i];
    }
    return NULL;
}

/** @brief function to write a record, if no more space is available, this
 *         function starts the garbage collection and puts the request to the
 *         pending records
 */
static ret_code_t write(fds_record_t* pRec)
{
    ret_code_t errCode;
    fds_record_desc_t desc;
    fds_find_token_t  tok = {0};

    errCode = fds_record_find(pRec->file_id, pRec->key, &desc, &tok);
    if (errCode == NRF_SUCCESS)
        errCode = fds_record_update(&desc, pRec);
    else if(errCode == FDS_ERR_NOT_FOUND)
        errCode = fds_record_write(&desc, pRec);

    // no more space in flash, run garbage collection and try again
    if (errCode == FDS_ERR_NO_SPACE_IN_FLASH)
    {
        errCode = fds_gc();

        fds_record_t* pPendingRec = getRecord(pRec->file_id);   // race condition if garbage collection is faster?
        if (pPendingRec == NULL)    // no more pending records available
            return NRF_ERROR_BUSY;

        memcpy(pPendingRec, pRec, sizeof(fds_record_t));
    }

    return errCode;
}

/** @brief function to process the pending records after garbage collection has succeeded
 */
static void processPendingRecords()
{
    for (uint32_t i = 0; i < 0; i++)
    {
        if (pendingRecords[i].file_id == FILE_ID_FREE || pendingRecords[i].file_id > 0xBFFF)
            continue;
        ret_code_t errCode = write(&pendingRecords[i]);
        if (errCode != NRF_SUCCESS) /// TODO: handle FDS_ERR_NO_SPACE_IN_QUEUES?
            sendResult(pendingRecords[i].file_id, errCode);
    }
    memset(pendingRecords, 0, sizeof(pendingRecords));
}

/** @brief the event handler
 */
static void fdsEventHandler(fds_evt_t const* pEvt)
{
    ret_code_t errCode = pEvt->result;

    if (pEvt->id == FDS_EVT_INIT)
    {
        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[ds]: fds init error %d in event", errCode);
        }
        initResult = errCode;
    }

    userHandlers_t* pUserHandler = getHandler(pEvt->write.file_id);
    if (pUserHandler == NULL)
        return; // not my busines

    switch (pEvt->id)
    {
    case FDS_EVT_WRITE:
    case FDS_EVT_UPDATE:
        sendResult(pEvt->write.file_id, pEvt->result);
        break;
    case FDS_EVT_DEL_RECORD:
    case FDS_EVT_DEL_FILE:
        sendResult(pEvt->del.file_id, pEvt->result);
        break;
    case FDS_EVT_GC:
        processPendingRecords();
        break;
    default:
        break;
    }
}

static void init()
{
    ret_code_t errCode;

    errCode = fds_register(fdsEventHandler);
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[ds]: fds register error %d", errCode);
        initResult = errCode;
        return;
    }

    errCode = fds_init();
    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("[ds]: fds init error %d", errCode);
        initResult = errCode;
        return;
    }

    while (initResult == NRF_ERROR_INVALID_STATE);
}

/* Public functions ----------------------------------------------------------*/
ret_code_t ds_Read(uint16_t fileId, uint16_t recordKey, void const** ppData, uint16_t* pLengthWords)
{
    IS_INIT();
    VERIFY_FILEID(fileId);
    VERIFY_RECORDKEY(recordKey);
    VERIFY_PARAM_NOT_NULL(ppData);
    VERIFY_PARAM_NOT_NULL(pLengthWords);

    ret_code_t errCode;
    fds_record_desc_t  desc;
    fds_find_token_t   tok = {0};
    fds_flash_record_t rec;

    errCode = fds_record_find(fileId, recordKey, &desc, &tok);
    if (errCode == NRF_SUCCESS)
    {
        // open record
        errCode = fds_record_open(&desc, &rec);
        if (errCode != NRF_SUCCESS)
            return errCode;

        *ppData = rec.p_data;
        *pLengthWords = rec.p_header->tl.length_words;

        errCode = fds_record_close(&desc);
        if (errCode != NRF_SUCCESS)
            return errCode;
    }
    return errCode;
}

ret_code_t ds_Write(uint16_t fileId, uint16_t recordKey, void const* pData, uint16_t lengthWords, ds_reportHandler_t pHandler)
{
    IS_INIT();
    VERIFY_FILEID(fileId);
    VERIFY_RECORDKEY(recordKey);
    VERIFY_PARAM_NOT_NULL(pData);

    ret_code_t         errCode;
    fds_record_t       rec;
    fds_record_chunk_t chunk;
    userHandlers_t* pUserHandler;

    chunk.p_data        = pData;
    chunk.length_words  = lengthWords;

    rec.file_id         = fileId;
    rec.key             = recordKey;
    rec.data.p_chunks   = &chunk;
    rec.data.num_chunks = 1;

    pUserHandler = getHandler(fileId);
    if (pUserHandler == NULL)
        pUserHandler = getHandler(FILE_ID_FREE);
    if (pUserHandler == NULL)
        return NRF_ERROR_BUSY;

    pUserHandler->fileId  = fileId;
    pUserHandler->cnt++;
    pUserHandler->handler = pHandler;

    errCode = write(&rec);

    if (errCode != NRF_SUCCESS)
    {
        if (pUserHandler->cnt && --pUserHandler->cnt == 0)
            memset(pUserHandler, 0, sizeof(userHandlers_t));
    }

    return errCode;
}

ret_code_t ds_Delete(uint16_t fileId, uint16_t recordKey, ds_reportHandler_t pHandler)
{
    IS_INIT();
    VERIFY_FILEID(fileId);
    VERIFY_RECORDKEY(recordKey);

    ret_code_t errCode;
    fds_record_desc_t desc;
    fds_find_token_t  tok = {0};
    userHandlers_t*   pUserHandler;

    pUserHandler = getHandler(fileId);
    if (pUserHandler == NULL)
        pUserHandler = getHandler(FILE_ID_FREE);
    if (pUserHandler == NULL)
        return NRF_ERROR_BUSY;

    pUserHandler->fileId  = fileId;
    pUserHandler->cnt++;
    pUserHandler->handler = pHandler;

    errCode = fds_record_find(fileId, recordKey, &desc, &tok);
    if (errCode == NRF_SUCCESS)
        errCode = fds_record_delete(&desc);
    // if no record exists, result can be sent immediately
    else if(errCode == FDS_ERR_NOT_FOUND)
    {
        errCode = NRF_SUCCESS;
        sendResult(fileId, errCode);
    }

    // if operation failed, remove the event request
    if (errCode != NRF_SUCCESS)
    {
        if (pUserHandler->cnt && --pUserHandler->cnt == 0)
            memset(pUserHandler, 0, sizeof(userHandlers_t));
    }

    return errCode;
}

ret_code_t ds_Reset(uint16_t fileId, ds_reportHandler_t pHandler)
{
    IS_INIT();
    VERIFY_FILEID(fileId);

    ret_code_t errCode;
    userHandlers_t*   pUserHandler;

    pUserHandler = getHandler(fileId);
    if (pUserHandler == NULL)
        pUserHandler = getHandler(FILE_ID_FREE);
    if (pUserHandler == NULL)
        return NRF_ERROR_BUSY;

    pUserHandler->fileId  = fileId;
    pUserHandler->cnt++;
    pUserHandler->handler = pHandler;

    errCode = fds_file_delete(fileId);

    // if operation failed, remove the event request
    if (errCode != NRF_SUCCESS)
    {
        if (pUserHandler->cnt && --pUserHandler->cnt == 0)
            memset(pUserHandler, 0, sizeof(userHandlers_t));
    }

    return errCode;
}

/**END OF FILE*****************************************************************/
