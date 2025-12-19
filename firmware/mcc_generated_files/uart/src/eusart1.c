/**
 * EUSART1 Generated Driver API Header File
 * 
 * @file eusart1.c
 * 
 * @ingroup eusart1
 * 
 * @brief This is the generated driver implementation file for the EUSART1 driver using the Enhanced Universal Synchronous and Asynchronous Receiver Transceiver (EUSART) module.
 *
 * @version EUSART1 Driver Version 3.0.3
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
#include "../eusart1.h"

/**
  Section: Macro Declarations
*/

#define EUSART1_TX_BUFFER_SIZE (16U) //buffer size should be 2^n
#define EUSART1_TX_BUFFER_MASK (EUSART1_TX_BUFFER_SIZE - 1U) 

#define EUSART1_RX_BUFFER_SIZE (16U) //buffer size should be 2^n
#define EUSART1_RX_BUFFER_MASK (EUSART1_RX_BUFFER_SIZE - 1U)

/**
  Section: Driver Interface
 */

const uart_drv_interface_t UART = {
    .Initialize = &EUSART1_Initialize,
    .Deinitialize = &EUSART1_Deinitialize,
    .Read = &EUSART1_Read,
    .Write = &EUSART1_Write,
    .IsRxReady = &EUSART1_IsRxReady,
    .IsTxReady = &EUSART1_IsTxReady,
    .IsTxDone = &EUSART1_IsTxDone,
    .TransmitEnable = &EUSART1_TransmitEnable,
    .TransmitDisable = &EUSART1_TransmitDisable,
    .AutoBaudSet = &EUSART1_AutoBaudSet,
    .AutoBaudQuery = &EUSART1_AutoBaudQuery,
    .BRGCountSet = NULL,
    .BRGCountGet = NULL,
    .BaudRateSet = NULL,
    .BaudRateGet = NULL,
    .AutoBaudEventEnableGet = NULL,
    .ErrorGet = &EUSART1_ErrorGet,
    .TxCompleteCallbackRegister = &EUSART1_TxCompleteCallbackRegister,
    .RxCompleteCallbackRegister = &EUSART1_RxCompleteCallbackRegister,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = &EUSART1_FramingErrorCallbackRegister,
    .OverrunErrorCallbackRegister = &EUSART1_OverrunErrorCallbackRegister,
    .ParityErrorCallbackRegister = NULL,
    .EventCallbackRegister = NULL,
};

/**
  Section: EUSART1 variables
*/
static volatile uint8_t eusart1TxHead = 0;
static volatile uint8_t eusart1TxTail = 0;
static volatile uint8_t eusart1TxBufferRemaining;
static volatile uint8_t eusart1TxBuffer[EUSART1_TX_BUFFER_SIZE];

static volatile uint8_t eusart1RxHead = 0;
static volatile uint8_t eusart1RxTail = 0;
static volatile uint8_t eusart1RxCount;
static volatile uint8_t eusart1RxBuffer[EUSART1_RX_BUFFER_SIZE];
/**
 * @misradeviation{@advisory,19.2}
 * The UART error status necessitates checking the bitfield and accessing the status within the group byte therefore the use of a union is essential.
 */
 /* cppcheck-suppress misra-c2012-19.2 */
static volatile eusart1_status_t eusart1RxStatusBuffer[EUSART1_RX_BUFFER_SIZE];

/**
 * @misradeviation{@advisory,19.2}
 * The UART error status necessitates checking the bitfield and accessing the status within the group byte therefore the use of a union is essential.
 */
 /* cppcheck-suppress misra-c2012-19.2 */
static volatile eusart1_status_t eusart1RxLastError;

/**
  Section: EUSART1 APIs
*/

void (*EUSART1_TxInterruptHandler)(void);
/* cppcheck-suppress misra-c2012-8.9 */
static void (*EUSART1_TxCompleteInterruptHandler)(void) = NULL;

void (*EUSART1_RxInterruptHandler)(void);
static void (*EUSART1_RxCompleteInterruptHandler)(void) = NULL;

static void (*EUSART1_FramingErrorHandler)(void) = NULL;
static void (*EUSART1_OverrunErrorHandler)(void) = NULL;

static void EUSART1_DefaultFramingErrorCallback(void);
static void EUSART1_DefaultOverrunErrorCallback(void);

void EUSART1_TransmitISR (void);
void EUSART1_ReceiveISR(void);


/**
  Section: EUSART1  APIs
*/

void EUSART1_Initialize(void)
{
    PIE1bits.RC1IE = 0;   
     EUSART1_RxInterruptHandler = EUSART1_ReceiveISR;   

    PIE1bits.TX1IE = 0; 
    EUSART1_TxInterruptHandler = EUSART1_TransmitISR; 

    // Set the EUSART1 module to the options selected in the user interface.

    //ABDEN disabled; WUE disabled; BRG16 16bit_generator; ABDOVF no_overflow; CKTXP async_noninverted_sync_inverted; RXDTP not_inverted; 
    BAUDCON1 = 0x48; 
    //ADDEN disabled; CREN enabled; SREN disabled; RX9 8-bit; SPEN enabled; 
    RCSTA1 = 0x90; 
    //TX9D 0x0; BRGH hi_speed; SENDB sync_break_complete; SYNC asynchronous; TXEN enabled; TX9 8-bit; CSRC client_mode; 
    TXSTA1 = 0x26; 
    //SPBRG1 160; 
    SPBRG1 = 0xA0; 
    //SPBRGH1 1; 
    SPBRGH1 = 0x1; 

    EUSART1_FramingErrorCallbackRegister(EUSART1_DefaultFramingErrorCallback);
    EUSART1_OverrunErrorCallbackRegister(EUSART1_DefaultOverrunErrorCallback);
    eusart1RxLastError.status = 0;  

    eusart1TxHead = 0;
    eusart1TxTail = 0;
    eusart1TxBufferRemaining = sizeof(eusart1TxBuffer);

    eusart1RxHead = 0;
    eusart1RxTail = 0;
    eusart1RxCount = 0;

    PIE1bits.RC1IE = 1; 
}

void EUSART1_Deinitialize(void)
{
    PIE1bits.RC1IE = 0;    
    PIE1bits.TX1IE = 0; 
    BAUDCON1 = 0x00;
    RCSTA1 = 0x00;
    TXSTA1 = 0x00;
    SPBRG1 = 0x00;
    SPBRGH1 = 0x00;
}

void EUSART1_Enable(void)
{
    RCSTA1bits.SPEN = 1;

}

void EUSART1_Disable(void)
{
    RCSTA1bits.SPEN = 0;
}


void EUSART1_TransmitEnable(void)
{
    TXSTA1bits.TXEN = 1;
}

void EUSART1_TransmitDisable(void)
{
    TXSTA1bits.TXEN = 0;
}

void EUSART1_ReceiveEnable(void)
{
    RCSTA1bits.CREN = 1;
}

void EUSART1_ReceiveDisable(void)
{
    RCSTA1bits.CREN = 0;
}

void EUSART1_SendBreakControlEnable(void)
{
    TXSTA1bits.SENDB = 1;
}

void EUSART1_SendBreakControlDisable(void)
{
    TXSTA1bits.SENDB = 0;
}

void EUSART1_AutoBaudSet(bool enable)
{
    if(enable)
    {
        BAUDCON1bits.ABDEN = 1;
    }
    else
    {
       BAUDCON1bits.ABDEN = 0; 
    }
}

bool EUSART1_AutoBaudQuery(void)
{
return (bool)(!BAUDCON1bits.ABDEN);
}

bool EUSART1_IsAutoBaudDetectOverflow(void)
{
    return (bool)BAUDCON1bits.ABDOVF; 
}

void EUSART1_AutoBaudDetectOverflowReset(void)
{
    BAUDCON1bits.ABDOVF = 0; 
}

void EUSART1_TransmitInterruptEnable(void)
{
    PIE1bits.TX1IE = 1;

}

void EUSART1_TransmitInterruptDisable(void)
{ 
    PIE1bits.TX1IE = 0; 
}

void EUSART1_ReceiveInterruptEnable(void)
{
    PIE1bits.RC1IE = 1;
}
void EUSART1_ReceiveInterruptDisable(void)
{
    PIE1bits.RC1IE = 0; 
}

bool EUSART1_IsRxReady(void)
{
    return (eusart1RxCount ? true : false);
}

bool EUSART1_IsTxReady(void)
{
    return (eusart1TxBufferRemaining ? true : false);
}

bool EUSART1_IsTxDone(void)
{
    return TXSTA1bits.TRMT;
}

size_t EUSART1_ErrorGet(void)
{
    eusart1RxLastError.status = eusart1RxStatusBuffer[(eusart1RxTail) & EUSART1_RX_BUFFER_MASK].status;
    return eusart1RxLastError.status;
}

uint8_t EUSART1_Read(void)
{
    uint8_t readValue  = 0;
    uint8_t tempRxTail;
    
    readValue = eusart1RxBuffer[eusart1RxTail];

    tempRxTail = (eusart1RxTail + 1U) & EUSART1_RX_BUFFER_MASK; // Buffer size of RX should be in the 2^n
    
    eusart1RxTail = tempRxTail;

    PIE1bits.RC1IE = 0; 
    if(0U != eusart1RxCount)
    {
        eusart1RxCount--;
    }
    PIE1bits.RC1IE = 1;
    return readValue;
}

void EUSART1_ReceiveISR(void)
{
    uint8_t regValue;
    uint8_t tempRxHead;

    // use this default receive interrupt handler code
    eusart1RxStatusBuffer[eusart1RxHead].status = 0;

    if(true == RCSTA1bits.OERR)
    {
        eusart1RxStatusBuffer[eusart1RxHead].oerr = 1;
        if(NULL != EUSART1_OverrunErrorHandler)
        {
            EUSART1_OverrunErrorHandler();
        }   
    }   
    if(true == RCSTA1bits.FERR)
    {
        eusart1RxStatusBuffer[eusart1RxHead].ferr = 1;
        if(NULL != EUSART1_FramingErrorHandler)
        {
            EUSART1_FramingErrorHandler();
        }   
    } 
    
    regValue = RCREG1;
    
    tempRxHead = (eusart1RxHead + 1U) & EUSART1_RX_BUFFER_MASK;// Buffer size of RX should be in the 2^n
    if (tempRxHead == eusart1RxTail) 
    {
		// ERROR! Receive buffer overflow 
	} 
    else
    {
        eusart1RxBuffer[eusart1RxHead] = regValue;
		eusart1RxHead = tempRxHead;
		eusart1RxCount++;
	}   

    if(NULL != EUSART1_RxCompleteInterruptHandler)
    {
        (*EUSART1_RxCompleteInterruptHandler)();
    } 
}

void EUSART1_Write(uint8_t txData)
{
    uint8_t tempTxHead;
    
    if(0 == PIE1bits.TX1IE)
    {
        TXREG1 = txData;
    }
    else if(0U < eusart1TxBufferRemaining) // check if at least one byte place is available in TX buffer
    {
       eusart1TxBuffer[eusart1TxHead] = txData;
       tempTxHead = (eusart1TxHead + 1U) & EUSART1_TX_BUFFER_MASK;
       
       eusart1TxHead = tempTxHead;
       PIE1bits.TX1IE = 0; //Critical value decrement
       eusart1TxBufferRemaining--; // one less byte remaining in TX buffer
    }
    else
    {
        //overflow condition; eusart1TxBufferRemaining is 0 means TX buffer is full
    }
    PIE1bits.TX1IE = 1;
}

void EUSART1_TransmitISR(void)
{
    uint8_t tempTxTail;
    // use this default transmit interrupt handler code
    if(sizeof(eusart1TxBuffer) > eusart1TxBufferRemaining) // check if all data is transmitted
    {
       TXREG1 = eusart1TxBuffer[eusart1TxTail];
       tempTxTail = (eusart1TxTail + 1U) & EUSART1_TX_BUFFER_MASK;
       
       eusart1TxTail = tempTxTail;
       eusart1TxBufferRemaining++; // one byte sent, so 1 more byte place is available in TX buffer
    }
    else
    {
        PIE1bits.TX1IE = 0;
    }

    if(NULL != EUSART1_TxCompleteInterruptHandler)
    {
        (*EUSART1_TxCompleteInterruptHandler)();
    }
}

int getch(void)
{
    while(!(EUSART1_IsRxReady()))
    {

    }
    return EUSART1_Read();
}

void putch(char txData)
{
    while(!(EUSART1_IsTxReady()))
    {

    }
    return EUSART1_Write(txData);   
}

static void EUSART1_DefaultFramingErrorCallback(void)
{
    
}

static void EUSART1_DefaultOverrunErrorCallback(void)
{
    //Continuous Receive must be cleared to clear Overrun Error else Rx will not receive upcoming bytes
    RCSTA1bits.CREN = 0;
    RCSTA1bits.CREN = 1;
}

void EUSART1_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART1_FramingErrorHandler = callbackHandler;
    }
}

void EUSART1_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART1_OverrunErrorHandler = callbackHandler;
    }    
}

void EUSART1_RxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       EUSART1_RxCompleteInterruptHandler = callbackHandler; 
    }   
}

void EUSART1_TxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       EUSART1_TxCompleteInterruptHandler = callbackHandler;
    }   
}

