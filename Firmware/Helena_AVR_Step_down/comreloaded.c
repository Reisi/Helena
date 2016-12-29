#include <avr/io.h>
#include <avr/power.h>
#include <string.h>
#include <util/crc16.h> 
#include <avr/interrupt.h>
#include "comreloaded.h"
#include "main.h"

///////////////////// defines /////////////////////////////////////////////////

///////////////////// type definitions ////////////////////////////////////////
typedef struct
{
    uint8_t StuffCounter;
    uint8_t BitCounter;
    uint8_t DataByte;
    uint8_t LastBitSize;
}
com_TransmissionStruct;

typedef struct
{
    uint8_t Buffer[COM_BUFFER];
    uint8_t Read;
    uint8_t Write;
}
com_TransmitFIFOStruct;

///////////////////// internal function prototypes ////////////////////////////
static void com_StartTransmission(void);
static void com_StopTransmission(void);
static void com_Disable(void);
static void com_Receive(void);
static void com_Transmit(void);

///////////////////// local variables /////////////////////////////////////////
static uint8_t com_ReceiveBuffer[18];
static com_TransmitFIFOStruct com_TransmitBuffer;
static com_TransmissionStruct com_TX = {4,144,0,0};
static com_MessageStruct com_InputMessage;
    
///////////////////// external function ///////////////////////////////////////

//---------------------------------------------------------------------------//
// Com Enable function                                                       //
// powers up and start timer used by com interface                           //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
void com_Enable()
{
    power_timer0_enable();
    TCCR0B |= (1<<CS01);
}

//---------------------------------------------------------------------------//
// Com sync function                                                         //
// synchronize timer to incomming messages                                   //
// no input or return values                                                 //
//                                                                           //
// put this function into Pin change isr                                     //
//---------------------------------------------------------------------------//
void com_Sync()
{
    if (PRR & (1<<PRTIM0))
        com_Enable();
    else
        OCR0B = TCNT0 ^ 64;
}


//---------------------------------------------------------------------------//
// Init function                                                             //
// prepares Pin and Timer of Com Interface                                   //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
void com_Init()
{
    PORTCOM &= ~(1<<PCOM);
    DDRCOM &= ~(1<<PCOM);
    
    GIMSK |= (1<<PCIE);                 //activate Pin Change interrupt
    PCMSK |= (1<<PCOM);                  //for COM pin
    
    OCR0A = 127;
    OCR0B = 63;
    TCCR0A |= (1<<WGM01);
    TIMSK |= (1<<OCIE0B);
    power_timer0_disable();
}

//---------------------------------------------------------------------------//
// Com Check function                                                        //
// checks incomming messageg for correct crc value                           //
// no input values                                                           //
// returns value: pointer to received message                                //
//                zero pointer if no message or crc wrong                    //
//---------------------------------------------------------------------------//
const com_MessageStruct *com_Check()
{
    uint8_t crctemp, i;    
    com_MessageStruct *pMessage;

    pMessage = NULL;

    if (main_flag & (1<<COMRXDONE))
    {
        main_flag &= ~(1<<COMRXDONE);
        crctemp = 0xFF;
        for (i = 0; i < 3 + (com_ReceiveBuffer[1] & 0x0F); i++)
            crctemp = _crc_ibutton_update(crctemp, com_ReceiveBuffer[i]);
        if (!crctemp)
        {
            pMessage = &com_InputMessage;
            memcpy(pMessage, com_ReceiveBuffer, 2 + (com_ReceiveBuffer[1] & 0x0F));
        }
    }

    return pMessage;
}

//---------------------------------------------------------------------------//
// Com Put function                                                          //
// puts outgoing messages into FIFO buffer and calculates crc                //
// input value: pointer to MessageStruct to be sent                          //
// returns value: ErrorEnum (FIFOFULL or NOERROR)                            //
//---------------------------------------------------------------------------//
com_ErrorEnum com_Put(const com_MessageStruct *pMessage)
{
    com_ErrorEnum error;
    uint8_t size, space, crctemp, i;
    
    size = 3 + (pMessage->Control & 0x0F);
    if (com_TransmitBuffer.Write >= com_TransmitBuffer.Read)
        space = com_TransmitBuffer.Read + COM_BUFFER - com_TransmitBuffer.Write;
    else
        space = com_TransmitBuffer.Read - com_TransmitBuffer.Write;
    if (space < size)
        error =  COM_FIFOFULL;
    else {
        crctemp = 0xFF;
        for (i = 0; i < size-1; i++) {
            com_TransmitBuffer.Buffer[com_TransmitBuffer.Write] = *(uint8_t *)pMessage;
            pMessage = (com_MessageStruct *)((uint8_t *)pMessage + 1);
            crctemp = _crc_ibutton_update(crctemp, com_TransmitBuffer.Buffer[com_TransmitBuffer.Write++]);
            com_TransmitBuffer.Write &= (COM_BUFFER-1);
        }
        com_TransmitBuffer.Buffer[com_TransmitBuffer.Write++] = crctemp;
        com_TransmitBuffer.Write &= (COM_BUFFER-1);
        error = COM_NOERROR;
    }
    if ( (com_Off()) && (com_InputState()) && (com_TransmitBuffer.Write != com_TransmitBuffer.Read))
        com_StartTransmission();
    return error;
}

//---------------------------------------------------------------------------//
// Com FlushFIFO function                                                    //
// flushs fifo output buffer (usefull if error on bus)                       //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
void com_FlushFIFO()
{
    com_TransmitBuffer.Read = 0;
    com_TransmitBuffer.Write = 0;
}

///////////////////// internal function ///////////////////////////////////////

//---------------------------------------------------------------------------//
// Start Transmission funktion                                               //
// starts a new transmission                                                 //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
static void com_StartTransmission()
{
    com_TX.BitCounter = 0;
    com_TX.StuffCounter = 0;
    com_TX.LastBitSize = (3 + (com_TransmitBuffer.Buffer[(com_TransmitBuffer.Read+1) & (COM_BUFFER-1)] & 0x0F)) << 3;
    com_TX.DataByte = com_TransmitBuffer.Buffer[com_TransmitBuffer.Read];
    com_Enable();
    com_TXEnable();
    com_OutputLow();
}

//---------------------------------------------------------------------------//
// Stop Transmission function                                                //
// stops an ongouing Transmission and resets FIFO read pointer               //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
static void com_StopTransmission()
{
    com_OutputOD();
    com_TXDisable();
    com_TX.BitCounter = 144;
}

//---------------------------------------------------------------------------//
// Com disable function                                                      //
// disables com interface and power downs timer                              //
// no input or return values                                                 //
//---------------------------------------------------------------------------//
static void com_Disable()
{
    TCCR0B &= ~(1<<CS01);
    TIFR = (1<<OCF0B)|(1<<OCF0A);
    OCR0B = 63;
    TCNT0 = 0;
    com_OutputOD();
    power_timer0_disable();
}

//---------------------------------------------------------------------------//
// Com receive function                                                      //
// receives incomming messages                                               //
// no input or return values                                                 //
//                                                                           //
// call from variable output compare timer isr                               //
//---------------------------------------------------------------------------//
static void com_Receive()
{
    static com_TransmissionStruct com_RX = {4,0,0,(1<<PCOM)};
    uint8_t com_Input;

    com_Input = com_InputState();

    if ((com_TX.BitCounter != 144) && (com_Input != com_OutputState()))
        com_StopTransmission();

    if (com_RX.StuffCounter < 4)
    {
        if (com_Input)
        {
            com_RX.DataByte <<= 1;
            com_RX.DataByte++;
        }
        else
            com_RX.DataByte <<= 1;
        if ((com_RX.BitCounter & 0x07) == 0x07)
            com_ReceiveBuffer[com_RX.BitCounter>>3] = com_RX.DataByte;   
        com_RX.BitCounter++;
    }
    else if (com_Input && (com_RX.StuffCounter == 5))
    {
        if (com_RX.BitCounter >= ((3 + (com_ReceiveBuffer[1] & 0x0F)) << 3))
            main_flag |= (1<<COMRXDONE);
        com_RX.BitCounter = 0;
    }
    else if (com_Input && (com_RX.StuffCounter == 8))
    {
        com_Disable();
        com_RX.BitCounter = 0;
        if (com_TransmitBuffer.Write != com_TransmitBuffer.Read)
            com_StartTransmission();
    }

    if (com_Input == com_RX.LastBitSize)
    {
        if (com_RX.StuffCounter < 255)
            com_RX.StuffCounter++;
        else
        {
            com_Disable();
            com_RX.BitCounter = 0;
        }
    }
    else
    {
        com_RX.StuffCounter = 0;
        com_RX.LastBitSize = com_Input;
    }
}

//---------------------------------------------------------------------------//
// Com transmit functioon                                                    //
// transmits outgoing messages                                               //
// no input or return values                                                 //
//                                                                           //
// call from staticle output compare / overflow timer isr                    //
//---------------------------------------------------------------------------//
static void com_Transmit()
{
    if (com_TX.BitCounter < com_TX.LastBitSize)
    {
        if (com_TX.StuffCounter < 4)
        {
            com_TX.DataByte <<= 1;
            if (SREG & (1<<0))
            {
                if (com_OutputState())
                    com_TX.StuffCounter++;
                else
                    com_TX.StuffCounter = 0;
                com_OutputOD();
            }
            else
            {
                if (com_OutputState())
                    com_TX.StuffCounter = 0;
                else
                    com_TX.StuffCounter++;
                com_OutputLow();
            }
            if ((com_TX.BitCounter++ & 0x07) == 0x07)
                com_TX.DataByte = com_TransmitBuffer.Buffer[(com_TransmitBuffer.Read + (com_TX.BitCounter>>3)) & (COM_BUFFER-1)];
        }
        else
        {
            com_TX.StuffCounter = 0;
            com_OutputToggle();
        }
    }
    else
    {
        com_TransmitBuffer.Read = (com_TransmitBuffer.Read + (com_TX.LastBitSize>>3)) & (COM_BUFFER-1);
        com_StopTransmission();
    }
}

///////////////////// interrupt service routines //////////////////////////////
ISR(TIM0_COMPA_vect)
{
    com_Transmit();
}

ISR(TIM0_COMPB_vect)
{
    com_Receive();
}

ISR(PCINT0_vect)                //synchronisation
{
    com_Sync();
}

