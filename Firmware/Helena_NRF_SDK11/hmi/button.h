/**
  ******************************************************************************
  * @file    button_evaluation.h
  * @author  Thomas Reisnecker
  *
  * @brief   Button debounce and evaluation module. This module can be used to
  *          debounce and evaluate buttons directly connected to a pin. It uses
  *          the GPIOTE module to detects any pin change. As soon as one pin
  *          changes its state it deactivates the pin change interrupts and
  *          switches to a polling mode using a timer with 10ms timebase.
  *
  *          To automatically evaluate a button call but_EvaluateInit() to
  *          obtain a valid evaluation id. Then call but_DebounceInit() with
  *          this id. The debounced state will then be automatically forwarded.
  *
  *          If you want to use the evaluation without debouncing you have to
  *          report button state changes manually with
  *          but_EvaluateButtonChange().
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BUTTON_H_INCLUDED
#define BUTTON_H_INCLUDED

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "sdk_errors.h"

/* configuration -------------------------------------------------------------*/
/// TODO: put this in sdk_config.h ?
#ifndef BUTTON_DEBOUNCE_CNT
#define BUTTON_DEBOUNCE_CNT 2   // the numbers of buttons to debounce
#endif // BUTTON_DEBOUNCE_CNT

#ifndef BUTTON_EVALUATE_CNT
#define BUTTON_EVALUATE_CNT 4   // number of buttons to evaluate
#endif // BUTTON_EVALUATE_CNT

/* Exported constants --------------------------------------------------------*/
#define BUTTON_PRESSED      1
#define BUTTON_RELEASED     0

/* Exported types ------------------------------------------------------------*/
/** @brief polarity type
 */
typedef enum
{
    BUTTON_ACTIVE_LOW = 0,
    BUTTON_ACTIVE_HIGH
} but_polarity_t;

/** @brief evaluation event types
 */
typedef enum
{
    BUTTON_IS_PRESSED,      // event when button gets pressed
    BUTTON_CLICKSHORT,      // event if button was pressed less than .longClickCnt
    BUTTON_CLICKLONG,       // event if button was pressed longer than .longClickCnt, but shorted than .StillPressedPeriod
    BUTTON_IS_STILL_PRESSED,// event sent every n*.StillPressedPeriod when button is hold
    BUTTON_IS_RELEASED,     // event send only if at least one BUTTON_IS_STILL_PRESSED event was sent
    BUTTON_ABORT            // event when counter reaches .abortCnt
} but_evalEvent_t;

/** @brief handler for debounce reports
 */
typedef void (*but_debounceReport_t)(uint32_t pin, uint8_t newState);

/** @brief handler for evaluated buttons
 */
typedef void (*but_evaluateReport_t)(uint8_t evalId, but_evalEvent_t evt, uint16_t stillPressCnt);

/** @brief debounce initialization structure
 */
typedef struct
{
    uint32_t             pinNo;         // the pin to debounce
    but_debounceReport_t pReportHandler;// the handler to call when stable state is reached, can be NULL whe using evalId
    but_polarity_t       polarity;      // the polarity of the pin
    uint8_t              evalId;        // a valid evaluation id (use but_EvaluateInit) if button should be evaluated, otherwise BUTTON_EVALUATE_CNT
} but_debounceInit_t;

/** @brief evaluation initialization structure
 */
typedef struct
{
    but_evaluateReport_t reportHandler; // the handler to call
    uint16_t longClickCnt;              // the count in 10ms units to decide between long and short clikcs
    uint16_t StillPressedPeriod;        // the period in counts of 10ms units to report pressed events
    uint16_t abortCnt;                  // the count in 10ms units when evaluation should be aborted
} but_evaluateInit_t;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/** @brief function to initialize a pin to debounce
 *
 * @param[in] pInit
 * @return NRF_ERROR_NULL, NRF_ERROR_NO_MEM,
 *         or the propagated error of nrfx_gpiote_in_init
 */
ret_code_t but_DebounceInit(but_debounceInit_t const* pInit);

/** @brief function to initialize a button evaluation
 *
 * @param[in] pInit
 * @return the evaluation id, BUTTON_EVALUATE_CNT if no evaluation structures
 *         are available, or pInit or reportHandler are NULL
 */
uint8_t but_EvaluateInit(but_evaluateInit_t const* pInit);

/** @brief function to manually report button changes to evaluate
 *
 * @param[in] evalId
 * @param[in] newState
 * @return NRF_ERROR_INVALID_PARAM for invalid id
 */
ret_code_t but_EvaluateButtonChange(uint8_t evalId, uint8_t newState);

#endif // BUTTON_EVALUATION_H_INCLUDED

/**END OF FILE*****************************************************************/
