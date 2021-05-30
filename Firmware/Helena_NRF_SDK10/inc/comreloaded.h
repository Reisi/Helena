#ifndef _COMRELOADED_H_
#define _COMRELOADED_H_

#include <stdint.h>
#include "custom_board.h"

///////////////////// defines /////////////////////////////////////////////////
#define PCOMRX  (pBoardConfig->bdepRX)
#define PCOMTX  (pBoardConfig->bdepTX)

#define com_InputState()   (nrf_gpio_pin_read(PCOMRX))
#define com_OutputState()  ((NRF_GPIO->OUT >> PCOMTX) & 1UL)
#define com_OutputLow()    (nrf_gpio_pin_clear(PCOMTX))
#define com_OutputOD()     (nrf_gpio_pin_set(PCOMTX))
#define com_OutputToggle() (com_OutputState() ? com_OutputLow() : com_OutputOD())
#define com_TXEnable()     (nrf_drv_timer_compare_int_enable(&timerInst, NRF_TIMER_CC_CHANNEL0))
#define com_TXDisable()    (nrf_drv_timer_compare_int_disable(&timerInst, NRF_TIMER_CC_CHANNEL0))
#define com_Off()          (nrf_timer_cc_read(timerInst.p_reg, NRF_TIMER_CC_CHANNEL2))

#define COM_BUFFER  32      //size of fifo output buffer (must be 2^n)

///////////////////// type definitions ////////////////////////////////////////
typedef enum
{COM_NOERROR = 0, COM_FIFOFULL}
com_ErrorEnum;

typedef struct
{
    uint8_t Identifier;
    uint8_t Control;
    uint8_t Data[15];
}
com_MessageStruct;

///////////////////// external function prototypes ////////////////////////////
void com_Init(void);
const com_MessageStruct *com_Check(void);
com_ErrorEnum com_Put(const com_MessageStruct*);
void com_FlushFIFO(void);
void com_OnSysEvt(uint32_t sysEvent);

/*static inline void com_Enable()
{
    power_timer1_enable();
    TCCR1B |= (1<<CS11);
}

static inline void com_Sync()
{
    if (PRR & (1<<PRTIM1))
        com_Enable();
    else
        OCR1B = TCNT1 ^ 64;
}*/

#endif //_COMRELOADED_H_

