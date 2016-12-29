#ifndef _COMRELOADED_H_
#define _COMRELOADED_H_

///////////////////// defines /////////////////////////////////////////////////
#define DDRCOM  DDRB
#define PORTCOM PORTB
#define PCOM    PB0
#define PINCOM  PINB

#define com_InputState()   (PINCOM & (1<<PCOM))
#define com_OutputState()  ((DDRCOM & (1<<PCOM)) ^ (1<<PCOM))
#define com_OutputLow()    (DDRCOM |= (1<<PCOM))
#define com_OutputOD()     (DDRCOM &= ~(1<<PCOM))
#define com_OutputToggle() (DDRCOM ^= (1<<PCOM))
#define com_TXEnable()     (TIMSK |= (1<<OCIE0A))
#define com_TXDisable()    (TIMSK &= ~(1<<OCIE0A))
#define com_Off()          (PRR & (1<<PRTIM0))

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

#endif //_COMRELOADED_H_

