/**
  ******************************************************************************
  * @file    helena_hmi.c
  * @author  Thomas Reisnecker
  * @brief   button and status led handling (template)
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
#include "nrf_gpio.h"

#include "helena_hmi.h"
#include "mode_management.h"
#include "link_management.h"
#include "board.h"
#include "button.h"
//#include "debug.h"

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    BUTEVAL_NONE = 0,   // no event
    BUTEVAL_PRESSED,    // button changed from open to pressed
    BUTEVAL_CLICKSHORT, // button released after short period
    BUTEVAL_CLICKLONG,  // button released after long period
    BUTEVAL_RELEASED,   // button released after a hold event
    BUTEVAL_HOLD2SEC,   // button is hold for 2sec (a BUTEVAL_RELEASED event will follow)
    BUTEVAL_HOLD10SEC,  // button is hold for 10sec (a BUTEVAL_RELEASED event will follow)
    BUTEVAL_ABORT,      // button is hold too long, evaluation is aborted
} buttonEvalEvents_t;

typedef enum
{
    BUT_MAIN = 0,
    BUT_SEC
} buttonType_t;

typedef struct
{
    bool red  : 1;
    bool blue : 1;
} ledState_t;

/* Private macros ------------------------------------------------------------*/
#define ARRAY_SIZE(x)           (sizeof(x)/sizeof(x[0]))

/* Private defines -----------------------------------------------------------*/
#define BUTTONPRESS_LONG        MSEC_TO_UNITS(500, UNIT_10_MS)
#define BUTTONPRESS_HOLD2SEC    MSEC_TO_UNITS(2000, UNIT_10_MS)
#define BUTTONPRESS_HOLD10SEC   MSEC_TO_UNITS(10000, UNIT_10_MS)
#define BUTTONPRESS_HOLDABORT   MSEC_TO_UNITS(30000, UNIT_10_MS)

/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static hln_brd_t const*   pBrdPins;
static uint8_t            buttonIds[2] = {BUTTON_EVALUATE_CNT, BUTTON_EVALUATE_CNT};
static buttonEvalEvents_t buttonEvents[2];  // the last occured button event

/* Private functions ---------------------------------------------------------*/
/** @brief button evaluation handler, just stores the received values, the hmi
 *         events are sent in the main loop
 */
static void buttonEval(uint8_t evalId, but_evalEvent_t evt, uint16_t stillPressCnt)
{
    buttonEvalEvents_t* pButtonEvent = NULL;

    for (uint_fast8_t i = 0; i < ARRAY_SIZE(buttonIds); i++)
    {
        if (evalId == buttonIds[i])
            pButtonEvent = &buttonEvents[i];
    }
    if (pButtonEvent == NULL)
        return;

    switch (evt)
    {
    case BUTTON_CLICKSHORT:
        *pButtonEvent = BUTEVAL_CLICKSHORT;
        break;
    case BUTTON_CLICKLONG:
        *pButtonEvent = BUTEVAL_CLICKLONG;
        break;
    case BUTTON_IS_STILL_PRESSED:
        if (stillPressCnt == BUTTONPRESS_HOLD2SEC)
            *pButtonEvent = BUTEVAL_HOLD2SEC;
        else if (stillPressCnt == BUTTONPRESS_HOLD10SEC)
            *pButtonEvent = BUTEVAL_HOLD10SEC;
        break;
    default:
        break;
    }
}

/**< initialization of the led pins
 */
static void ledsInit()
{
    // configure LED pins and clear LEDs

    if (pBrdPins->ledPins.blue == pBrdPins->ledPins.red)
    {   // rev 2.2/2.3 code
        nrf_gpio_cfg_default(pBrdPins->ledPins.blue);
    }
    else
    {   // rev 2.1 code
        nrf_gpio_cfg_output(pBrdPins->ledPins.red);
        nrf_gpio_pin_clear(pBrdPins->ledPins.red);
        nrf_gpio_cfg_output(pBrdPins->ledPins.blue);
        nrf_gpio_pin_clear(pBrdPins->ledPins.blue);
    }
}

/**< initialization of the button debouncing and evaluation
 */
static ret_code_t buttonInit(bool secondary)
{
    ret_code_t errCode;

    but_evaluateInit_t evalInit = {0};
    evalInit.longClickCnt = BUTTONPRESS_LONG;
    evalInit.StillPressedPeriod = BUTTONPRESS_HOLD2SEC;
    evalInit.abortCnt = BUTTONPRESS_HOLDABORT;
    evalInit.reportHandler = buttonEval;
    buttonIds[BUT_MAIN] = but_EvaluateInit(&evalInit);

    but_debounceInit_t debInit = {0};
    debInit.pinNo = pBrdPins->button;
    debInit.polarity = BUTTON_ACTIVE_LOW;
    debInit.evalId = buttonIds[BUT_MAIN];
    errCode = but_DebounceInit(&debInit);

    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("debounce init error %d", errCode);
    }

    if (!secondary)
        return errCode;

    buttonIds[BUT_SEC] = but_EvaluateInit(&evalInit);
    debInit.pinNo = pBrdPins->comPins.rx;
    debInit.evalId = buttonIds[BUT_SEC];
    if (errCode == NRF_SUCCESS)
        errCode = but_DebounceInit(&debInit);
    else
        (void)but_DebounceInit(&debInit);

    if (errCode != NRF_SUCCESS)
    {
        LOG_ERROR("debounce init error %d", errCode);
    }

    return errCode;
}

#ifdef DEBUG_EXT
/**< bonus function in debug mode. uses RTT to fake button events
 */
static void fakeButtonOverRTT()
{
    char segin[1];
    unsigned numOfBytes = SEGGER_RTT_Read(0, segin, sizeof(segin));
    if (numOfBytes == 1)
    {
        if (segin[0] == '1')            // simulate short click
            buttonEvents[BUT_MAIN] = BUTEVAL_CLICKSHORT;
        else if (segin[0] == '2')       // simulate long click
            buttonEvents[BUT_MAIN] = BUTEVAL_CLICKLONG;
        else if (segin[0] == '3')       // simulate 2s hold
            buttonEvents[BUT_MAIN] = BUTEVAL_HOLD2SEC;
        else if (segin[0] == '4')       // simulate 10s hold
            buttonEvents[BUT_MAIN] = BUTEVAL_HOLD10SEC;
    }
}
#endif

/* Public functions ----------------------------------------------------------*/
ret_code_t hln_hmi_Init(hln_brd_t const* pPins, bool secondaryButton)
{
    ret_code_t errCode;

    if (pPins == NULL)
        return NRF_ERROR_NULL;

    pBrdPins = pPins;

    ledsInit();

    errCode = buttonInit(secondaryButton);
    if (errCode != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

void hln_hmi_Execute()
{
#ifdef DEBUG_EXT
    fakeButtonOverRTT();
#endif // DEBUG_EXT

    if (buttonEvents[BUT_MAIN] != BUTEVAL_NONE)
    {
        uint32_t errCode = NRF_SUCCESS;
        bool isOff = mm_GetCurrentMode() == MM_MODE_OFF;
        lm_scanState_t currentScanState = lm_GetScanningState();

        switch (buttonEvents[BUT_MAIN])
        {
        case BUTEVAL_CLICKSHORT:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTMODE, NULL);
            break;
        case BUTEVAL_CLICKLONG:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTGROUP, NULL);
            break;
        case BUTEVAL_HOLD2SEC:
            if (!isOff)
                errCode = hmi_RequestEvent(HMI_EVT_MODEPREF, NULL);     // preferred mode if on
            else if (currentScanState != LM_SCAN_SEARCH)
                errCode = hmi_RequestEvent(HMI_EVT_SEARCHREMOTE, NULL); // search if off
            else
                errCode = hmi_RequestEvent(HMI_EVT_DELETEBONDS, NULL);  // or delete bonds if already searching
            break;
        case BUTEVAL_HOLD10SEC:
            if (isOff)
                errCode = hmi_RequestEvent(HMI_EVT_FACTORYRESET, NULL);
            else
                errCode = hmi_RequestEvent(HMI_EVT_MODESOS, NULL);
            break;
        default:
            break;
        }

        buttonEvents[BUT_MAIN] = BUTEVAL_NONE;

        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[BRD:] request event error %d", errCode);
        }
    }

    if (buttonEvents[BUT_SEC] != BUTEVAL_NONE)
    {
        uint32_t errCode = NRF_SUCCESS;

        switch (buttonEvents[BUT_SEC])
        {
        case BUTEVAL_CLICKSHORT:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTMODE, NULL);
            break;
        case BUTEVAL_CLICKLONG:
            errCode = hmi_RequestEvent(HMI_EVT_NEXTGROUP, NULL);
            break;
        case BUTEVAL_HOLD2SEC:
            errCode = hmi_RequestEvent(HMI_EVT_MODEPREF, NULL);     // preferred mode
            break;
        default:
            break;
        }

        buttonEvents[BUT_SEC] = BUTEVAL_NONE;

        if (errCode != NRF_SUCCESS)
        {
            LOG_ERROR("[BRD:] request event error %d", errCode);
        }
    }
}

void brd_EnableLed(hmi_ledType_t color, bool enable)
{
    if (pBrdPins == NULL)
        return;

    static ledState_t ledState;

    if (color == HMI_LT_RED)
        ledState.red = enable;
    else if (color == HMI_LT_BLUE)
        ledState.blue = enable;
    else
        return;


    if (pBrdPins->ledPins.blue == pBrdPins->ledPins.red)
    {   // rev 2.2/2.3 code, this rev uses one pin for both led, so without
        // pwm it is not possible to enable both at the same time. Therefore
        // the red led is given the higher priority
        if (ledState.red)
        {
            nrf_gpio_cfg_output(pBrdPins->ledPins.red);
            nrf_gpio_pin_clear(pBrdPins->ledPins.red);
        }
        else if (ledState.blue)
        {
            nrf_gpio_cfg_output(pBrdPins->ledPins.blue);
            nrf_gpio_pin_set(pBrdPins->ledPins.blue);
        }
        else
            nrf_gpio_cfg_default(pBrdPins->ledPins.red);
    }
    else
    {   // rev 2.1 code
        if (ledState.red)
            nrf_gpio_pin_set(pBrdPins->ledPins.red);
        else
            nrf_gpio_pin_clear(pBrdPins->ledPins.red);

        if (ledState.blue)
            nrf_gpio_pin_set(pBrdPins->ledPins.blue);
        else
            nrf_gpio_pin_clear(pBrdPins->ledPins.blue);
    }
}

#undef  NRF_LOG_LEVEL

/**END OF FILE*****************************************************************/
