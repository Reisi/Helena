/**
  ******************************************************************************
  * @file    light.h
  * @author  RT
  * @version V1.1
  * @date    16/11/07
  * @brief   Header for light.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LIGHT_H
#define _LIGHT_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported defines ----------------------------------------------------------*/
#define LIGHT_STATUS_OVERCURRENT        (1<<0)
#define LIGHT_STATUS_VOLTAGELIMIT       (1<<1)
#define LIGHT_STATUS_TEMPERATURELIMIT   (1<<2)
#define LIGHT_STATUS_DUTYCYCLELIMIT     (1<<3)

/* Exported types ------------------------------------------------------------*/
typedef int16_t q15_t;

typedef enum
{
    LIGHT_MODEOFF = 0,           /**< light is off */
    LIGHT_MODEFLOOD,             /**< flood active */
    LIGHT_MODESPOT,              /**< spot active */
    LIGHT_MODEBOTH,              /**< flood and spot active */
    LIGHT_MODEADAPTIVEFLOOD,     /**< flood active, with pitch compensation */
    LIGHT_MODEADAPTIVESPOT,      /**< spot active with pitch compensation */
    LIGHT_MODEADAPTIVEBOTH,      /**< flood and spot active with pitch compensation */
    LIGHT_MODECNT
} light_ModeEnum;

typedef struct
{
    light_ModeEnum mode;        /**< mode */
    int8_t intensity;           /**< intensity in % or in lux in adaptive modes */
} light_ModeStruct;

typedef uint8_t light_Status;   /**< light status flag register */

typedef struct
{
    light_Status flood;         /**< status of flood */
    light_Status spot;          /**< status of spot */
    uint16_t currentFlood;      /**< flood current in mA */
    uint16_t currentSpot;       /**< spot current in mA */
    uint16_t temperature;       /**< temperature in 0.1K */
    uint16_t inputVoltage;      /**< input voltage in mV */
} light_StatusStruct;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief initialization function
 *
 * @return      NRF_SUCCESS or an error code
 */
uint32_t light_Init(void);

/** @brief function to enable / disable light functionality
 *
 * @param[in]   enable  true to enable, false to disable
 * @return      NRF_SUCCESS or NRF_ERROR_INVALID_STATE;
 *
 * @note        when light is disabled, no valid data is return by
 *              @ref light_Execute
 */
uint32_t light_Enable(bool enable);

/** @brief target update function
 *
 * @param[in]   pMode       requested light mode
 * @param[in]   pitch       current light pitch angle
 * @param[out]  ppStatus    actual status of light
 * @return      NRF_SUCCESS or an error code
 */
uint32_t light_UpdateTargets(const light_ModeStruct* pMode, q15_t pitch, const light_StatusStruct* *ppStatus);

/** @brief execute function
 *
 * @note        call this function in the main loop!
 */
void light_Execute(void);

#endif /*_LIGHT_H_*/

/**END OF FILE*****************************************************************/
