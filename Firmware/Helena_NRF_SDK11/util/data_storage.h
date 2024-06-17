/**
  ******************************************************************************
  * @file    data_storage.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DATA_STORAGE_H_INCLUDED
#define DATA_STORAGE_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include "sdk_errors.h"
#include "fds.h"

/* Exported types ------------------------------------------------------------*/
typedef void (*ds_reportHandler_t)(ret_code_t errCode);

/* Exported constants --------------------------------------------------------*/
#define DS_MAX_USERS    2

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to read data from flash
 *
 * @param[in]  fileId
 * @param[in]  recordKey
 * @param[out] ppData
 * @param[out] pLengthWords
 * @return     NRF_SUCCESS,
 *             NRF_ERROR_INVALID_PARAM for invalid file ids or record keys,
 *             NRF_ERROR_MULL if ppData or pLenghtWords is null,
 *             or propagated errors from the fds module
 */
ret_code_t ds_Read(uint16_t fileId, uint16_t recordKey, void const** ppData, uint16_t* pLengthWords);

/** @brief function to write data to flash
 *
 * @param[in]  fileId
 * @param[in]  recordKey
 * @param[in]  pData
 * @param[in]  lengthWords
 * @param[in]  pHandler     can be NULL, if no result is needed
 * @return     NRF_SUCCESS,
 *             NRF_ERROR_INVALID_PARAM for invalid file ids or record keys,
 *             NRF_ERROR_MULL if pData is null
 *             or propagated errors from the fds module
 */
ret_code_t ds_Write(uint16_t fileId, uint16_t recordKey, void const* pData, uint16_t lengthWords, ds_reportHandler_t pHandler);

/** @brief function to delete a specific record from flash
 *
 * @param[in]  fileId
 * @param[in]  recordKey
 * @param[in]  pHandler     can be NULL, if no result is needed
 * @return     NRF_SUCCESS,
 *             NRF_ERROR_INVALID_PARAM for invalid file ids or record keys,
 *             or propagated errors from the fds module
 */
ret_code_t ds_Delete(uint16_t fileId, uint16_t recordKey, ds_reportHandler_t pHandler);

/** @brief function to delete all content for a user
 *
 * @param[in]  fileId
 * @param[in]  pHandler     can be NULL, if no result is needed
 * @return     NRF_SUCCESS,
 *             NRF_ERROR_INVALID_PARAM for invalid file ids,
 *             or propagated errors from the fds module
 */
ret_code_t ds_Reset(uint16_t fileId, ds_reportHandler_t pHandler);

#endif // DATA_STORAGE_H_INCLUDED

/**END OF FILE*****************************************************************/
