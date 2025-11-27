/**
 * EUSART Generated Driver API Header File
 * 
 * @file eusart.c
 * 
 * @ingroup eusart
 * 
 * @brief This is the generated driver implementation file for the EUSART driver using the Enhanced Universal Synchronous and Asynchronous Receiver Transceiver (EUSART) module.
 *
 * @version EUSART Driver Version 3.0.3
*/

/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/
#include "../eusart.h"

/**
  Section: Macro Declarations
*/

/**
  Section: Driver Interface
 */

const uart_drv_interface_t UART = {
    .Initialize = &EUSART_Initialize,
    .Deinitialize = &EUSART_Deinitialize,
    .Read = &EUSART_Read,
    .Write = &EUSART_Write,
    .IsRxReady = &EUSART_IsRxReady,
    .IsTxReady = &EUSART_IsTxReady,
    .IsTxDone = &EUSART_IsTxDone,
    .TransmitEnable = &EUSART_TransmitEnable,
    .TransmitDisable = &EUSART_TransmitDisable,
    .AutoBaudSet = &EUSART_AutoBaudSet,
    .AutoBaudQuery = &EUSART_AutoBaudQuery,
    .BRGCountSet = NULL,
    .BRGCountGet = NULL,
    .BaudRateSet = NULL,
    .BaudRateGet = NULL,
    .AutoBaudEventEnableGet = NULL,
    .ErrorGet = &EUSART_ErrorGet,
    .TxCompleteCallbackRegister = NULL,
    .RxCompleteCallbackRegister = NULL,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = &EUSART_FramingErrorCallbackRegister,
    .OverrunErrorCallbackRegister = &EUSART_OverrunErrorCallbackRegister,
    .ParityErrorCallbackRegister = NULL,
    .EventCallbackRegister = NULL,
};

/**
  Section: EUSART variables
*/
/**
 * @misradeviation{@advisory,19.2}
 * The UART error status necessitates checking the bitfield and accessing the status within the group byte therefore the use of a union is essential.
 */
 /* cppcheck-suppress misra-c2012-19.2 */
static volatile eusart_status_t eusartRxLastError;

/**
  Section: EUSART APIs
*/

static void (*EUSART_FramingErrorHandler)(void) = NULL;
static void (*EUSART_OverrunErrorHandler)(void) = NULL;

static void EUSART_DefaultFramingErrorCallback(void);
static void EUSART_DefaultOverrunErrorCallback(void);


/**
  Section: EUSART  APIs
*/

void EUSART_Initialize(void)
{
    // Set the EUSART module to the options selected in the user interface.

    //ABDEN disabled; WUE disabled; BRG16 16bit_generator; SCKP Non-Inverted; 
    BAUD1CON = 0x48; 
    //ADDEN disabled; CREN enabled; SREN disabled; RX9 8-bit; SPEN enabled; 
    RC1STA = 0x90; 
    //TX9D 0x0; BRGH hi_speed; SENDB sync_break_complete; SYNC asynchronous; TXEN enabled; TX9 8-bit; CSRC client; 
    TX1STA = 0x26; 
    //SPBRGL 68; 
    SP1BRGL = 0x44; 
    //SPBRGH 0; 
    SP1BRGH = 0x0; 

    EUSART_FramingErrorCallbackRegister(EUSART_DefaultFramingErrorCallback);
    EUSART_OverrunErrorCallbackRegister(EUSART_DefaultOverrunErrorCallback);
    eusartRxLastError.status = 0;  

}

void EUSART_Deinitialize(void)
{
    BAUD1CON = 0x00;
    RC1STA = 0x00;
    TX1STA = 0x00;
    SP1BRGL = 0x00;
    SP1BRGH = 0x00;
}

void EUSART_Enable(void)
{
    RC1STAbits.SPEN = 1;

}

void EUSART_Disable(void)
{
    RC1STAbits.SPEN = 0;
}


void EUSART_TransmitEnable(void)
{
    TX1STAbits.TXEN = 1;
}

void EUSART_TransmitDisable(void)
{
    TX1STAbits.TXEN = 0;
}

void EUSART_ReceiveEnable(void)
{
    RC1STAbits.CREN = 1;
}

void EUSART_ReceiveDisable(void)
{
    RC1STAbits.CREN = 0;
}

void EUSART_SendBreakControlEnable(void)
{
    TX1STAbits.SENDB = 1;
}

void EUSART_SendBreakControlDisable(void)
{
    TX1STAbits.SENDB = 0;
}

void EUSART_AutoBaudSet(bool enable)
{
    if(enable)
    {
        BAUD1CONbits.ABDEN = 1;
    }
    else
    {
       BAUD1CONbits.ABDEN = 0; 
    }
}

bool EUSART_AutoBaudQuery(void)
{
return (bool)(!BAUD1CONbits.ABDEN);
}

bool EUSART_IsAutoBaudDetectOverflow(void)
{
    return (bool)BAUD1CONbits.ABDOVF; 
}

void EUSART_AutoBaudDetectOverflowReset(void)
{
    BAUD1CONbits.ABDOVF = 0; 
}

bool EUSART_IsRxReady(void)
{
    return (bool)(PIR3bits.RCIF);
}

bool EUSART_IsTxReady(void)
{
    return (bool)(PIR3bits.TXIF && TX1STAbits.TXEN);
}

bool EUSART_IsTxDone(void)
{
    return TX1STAbits.TRMT;
}

size_t EUSART_ErrorGet(void)
{
    return eusartRxLastError.status;
}

uint8_t EUSART_Read(void)
{
    eusartRxLastError.status = 0;
    if(true == RC1STAbits.OERR)
    {
        eusartRxLastError.oerr = 1;
        if(NULL != EUSART_OverrunErrorHandler)
        {
            EUSART_OverrunErrorHandler();
        }   
    }
    if(true == RC1STAbits.FERR)
    {
        eusartRxLastError.ferr = 1;
        if(NULL != EUSART_FramingErrorHandler)
        {
            EUSART_FramingErrorHandler();
        }   
    }
    return RC1REG;
}

void EUSART_Write(uint8_t txData)
{
    TX1REG = txData;
}

int getch(void)
{
    while(!(EUSART_IsRxReady()))
    {

    }
    return EUSART_Read();
}

void putch(char txData)
{
    while(!(EUSART_IsTxReady()))
    {

    }
    return EUSART_Write(txData);   
}

static void EUSART_DefaultFramingErrorCallback(void)
{
    
}

static void EUSART_DefaultOverrunErrorCallback(void)
{
    //Continuous Receive must be cleared to clear Overrun Error else Rx will not receive upcoming bytes
    RC1STAbits.CREN = 0;
    RC1STAbits.CREN = 1;
}

void EUSART_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART_FramingErrorHandler = callbackHandler;
    }
}

void EUSART_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART_OverrunErrorHandler = callbackHandler;
    }    
}

