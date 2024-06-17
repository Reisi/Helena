/**
  ******************************************************************************
  * @file    filter.h
  * @author  Thomas Reisnecker
  * @brief   tbd.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FILTER_H_INCLUDED
#define FILTER_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    int32_t* pBuffer;
    uint16_t wr;
    uint16_t order;
} fil_movAvgInst_t;

typedef struct
{
    int64_t buffer;
    uint16_t tau;   // in relation to filter sampling time
} fil_lowPassInst_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/** @brief initialization macro for a moving average filter
 *
 * @param _name
 * @param _order
 */
#define FIL_MOVAVG_DEF(_name, _order)       \
static int32_t _name ## _buffer[_order];    \
static fil_movAvgInst_t _name =             \
{                                           \
    .pBuffer = _name ## _buffer,            \
    .order = _order                         \
};

/** @brief initialization macro for a 1st order low pass filter
 *
 * @param _name
 * @param _tau
 */
#define FIL_LOWPASS_DEF(_name, _tau)       \
static fil_lowPassInst_t _name =           \
{                                          \
    .tau = _tau                            \
};

/* Exported functions ------------------------------------------------------- */
/** @brief moving average filter
 *
 * @param[in] pInst   the filtering instance
 * @param[in] actual  the current value to filter
 * @return  the filter output
 */
int32_t fil_MovAvg(fil_movAvgInst_t* pInst, int32_t actual);

/** @brief function to reset the filter
 *
 * @param[in] pInst the filtering instance
 */
void fil_MovAvgReset(fil_movAvgInst_t* pInst, int32_t initValue);

/** @brief moving average filter feed function
 *
 * @param[in] pInst   the filtering instance
 * @param[in] actual  the current value to filter
 */
void fil_LowPassFeed(fil_lowPassInst_t* pInst, int32_t actual);

/** @brief moving average filter get function
 *
 * @param[in] pInst   the filtering instance
 * @return  the filter output
 */
int32_t fil_LowPassGet(fil_lowPassInst_t const* pInst);

/** @brief function to reset the filter
 *
 * @param[in] pInst the filtering instance
 */
void fil_LowPassReset(fil_lowPassInst_t* pInst, int32_t initValue);

#endif // FILTER_H_INCLUDED

/**END OF FILE*****************************************************************/

