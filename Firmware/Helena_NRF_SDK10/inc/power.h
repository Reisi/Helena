/**
  ******************************************************************************
  * @file    power.h
  * @author  Thomas Reisnecker
  * @brief   Header for power module, handling sleep modes and input voltage
  *          measuring
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _POWER_H_
#define _POWER_H_

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**< @brief flags for modules capable of preventing the power module to enter
            sleep mode */
typedef enum
{
    pwr_ACTIVEHMI = 0,      // flag reserved for hmi module
    pwr_ACTIVELIGHT,        // flag reserved for light module
    pwr_ACTIVECOM           // flag reserved for com module
} pwr_moduleFlags_t;

typedef uint16_t q6_10_t;   // fix point data type, unsigned 6 integral digits, 10 fractional

/**< @brief input voltage data structure */
typedef struct
{
    q6_10_t inputVoltage;   // input voltage
    uint32_t timestamp;     // timestamp of last measurement
} pwr_inputVoltage_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/********************************************//**
 * @brief initialization function of power module
 *
 * @return the number of assumed li-ion cells connected in series of the
 *         power supply battery, 0 if this fails
 ***********************************************/
uint8_t pwr_Init(void);

/********************************************//**
 * @brief sleep management function
 *
 * @note  this function should be called at the end of the main loop
 ***********************************************/
void pwr_SleepManagement(void);

/********************************************//**
 * @brief function to set the power module into active mode
 *
 * @param[in] module the module requesting active mode
 *
 * @note This function shall be called if any modules needs to prevent entering
 *       sleep mode. Call pwr_ClearActiveFlag to release this lock
 ***********************************************/
void pwr_SetActiveFlag(pwr_moduleFlags_t module);

/********************************************//**
 * @brief function to release a sleep lock
 *
 * @param[in] module the module requesting to clear the active flag
 ***********************************************/
void pwr_ClearActiveFlag(pwr_moduleFlags_t module);

/********************************************//**
 * @brief function to start a supply voltage measurement
 * @return NRF_SUCCESS
 *         NRF_ERROR_INVALID_STATE if measurement already ongoing
 ***********************************************/
uint32_t pwr_StartInputVoltageConversion(void);

/********************************************//**
 * @brief function to get the last known supply voltage
 *
 * @param[out] pVoltage last result including timestamp
 * @return     NRF_SUCCESS
 *             NRF_ERROR_NULL if pVoltage is 0
 *             NRF_ERROR_INVALID_STATE if a measurement is ongoing
 ***********************************************/
uint32_t pwr_GetInputVoltage(pwr_inputVoltage_t* pVoltage);

#endif /*_POWER_H_*/

/**END OF FILE*****************************************************************/
