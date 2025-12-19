/**
 * Interrupt Manager Generated Driver File
 *
 * @file interrupt.c
 * 
 * @ingroup interrupt 
 * 
 * @brief This file contains the API implementation for the Interrupt Manager driver.
 * 
 * @version Interrupt Manager Driver Version 1.0.2
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

#include "../../system/interrupt.h"
#include "../../system/system.h"
#include "../pins.h"

void (*INT0_InterruptHandler)(void);
void (*INT1_InterruptHandler)(void);
void (*INT2_InterruptHandler)(void);
void (*INT3_InterruptHandler)(void);

void  INTERRUPT_Initialize (void)
{
    // Enable Interrupt Priority Vectors (16CXXX Compatibility Mode)
    RCONbits.IPEN = 1;

    // Assign peripheral interrupt priority vectors

    // TMRI - high priority
    INTCON2bits.TMR0IP = 1;

    // TXI - high priority
    IPR1bits.TX1IP = 1;

    // RCI - high priority
    IPR1bits.RC1IP = 1;

    // CMPI - high priority
    IPR4bits.CMP1IP = 1;

    // CMPI - high priority
    IPR4bits.CMP2IP = 1;



    // Clear the interrupt flag
    // Set the external interrupt edge detect
    EXT_INT0_InterruptFlagClear();
    EXT_INT0_risingEdgeSet();
    // Set Default Interrupt Handler
    INT0_SetInterruptHandler(INT0_DefaultInterruptHandler);
    // EXT_INT0_InterruptEnable();

    // Clear the interrupt flag
    // Set the external interrupt edge detect
    EXT_INT1_InterruptFlagClear();
    EXT_INT1_risingEdgeSet();
    // Set Default Interrupt Handler
    INT1_SetInterruptHandler(INT1_DefaultInterruptHandler);
    // EXT_INT1_InterruptEnable();

    // Clear the interrupt flag
    // Set the external interrupt edge detect
    EXT_INT2_InterruptFlagClear();
    EXT_INT2_risingEdgeSet();
    // Set Default Interrupt Handler
    INT2_SetInterruptHandler(INT2_DefaultInterruptHandler);
    // EXT_INT2_InterruptEnable();

    // Clear the interrupt flag
    // Set the external interrupt edge detect
    EXT_INT3_InterruptFlagClear();
    EXT_INT3_risingEdgeSet();
    // Set Default Interrupt Handler
    INT3_SetInterruptHandler(INT3_DefaultInterruptHandler);
    // EXT_INT3_InterruptEnable();

}

/**
 * @ingroup interrupt
 * @brief Services the Interrupt Service Routines (ISR) of high-priority enabled interrupts and is called every time a high-priority interrupt is triggered.
 * @pre Interrupt Manager is initialized.
 * @param None.
 * @return None.
 */
void __interrupt() INTERRUPT_InterruptManagerHigh (void)
{
    // interrupt handler
    if(INTCONbits.TMR0IE == 1 && INTCONbits.TMR0IF == 1)
    {
        TMR0_OverflowISR();
    }
    else if(PIE1bits.TX1IE == 1 && PIR1bits.TX1IF == 1)
    {
        EUSART1_TxInterruptHandler();
    }
    else if(PIE1bits.RC1IE == 1 && PIR1bits.RC1IF == 1)
    {
        EUSART1_RxInterruptHandler();
    }
    else if(PIE4bits.CMP1IE == 1 && PIR4bits.CMP1IF == 1)
    {
        CMP1_ISR();
    }
    else if(PIE4bits.CMP2IE == 1 && PIR4bits.CMP2IF == 1)
    {
        CMP2_ISR();
    }
    else
    {
        //Unhandled Interrupt
    }
}


void INT0_ISR(void)
{
    EXT_INT0_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT0_CallBack();    
}


void INT0_CallBack(void)
{
    // Add your custom callback code here
    if(INT0_InterruptHandler)
    {
        INT0_InterruptHandler();
    }
}

void INT0_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT0_InterruptHandler = InterruptHandler;
}

void INT0_DefaultInterruptHandler(void){
    // add your INT0 interrupt custom code
    // or set custom function using INT0_SetInterruptHandler()
}
void INT1_ISR(void)
{
    EXT_INT1_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT1_CallBack();    
}


void INT1_CallBack(void)
{
    // Add your custom callback code here
    if(INT1_InterruptHandler)
    {
        INT1_InterruptHandler();
    }
}

void INT1_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT1_InterruptHandler = InterruptHandler;
}

void INT1_DefaultInterruptHandler(void){
    // add your INT1 interrupt custom code
    // or set custom function using INT1_SetInterruptHandler()
}
void INT2_ISR(void)
{
    EXT_INT2_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT2_CallBack();    
}


void INT2_CallBack(void)
{
    // Add your custom callback code here
    if(INT2_InterruptHandler)
    {
        INT2_InterruptHandler();
    }
}

void INT2_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT2_InterruptHandler = InterruptHandler;
}

void INT2_DefaultInterruptHandler(void){
    // add your INT2 interrupt custom code
    // or set custom function using INT2_SetInterruptHandler()
}
void INT3_ISR(void)
{
    EXT_INT3_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT3_CallBack();    
}


void INT3_CallBack(void)
{
    // Add your custom callback code here
    if(INT3_InterruptHandler)
    {
        INT3_InterruptHandler();
    }
}

void INT3_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT3_InterruptHandler = InterruptHandler;
}

void INT3_DefaultInterruptHandler(void){
    // add your INT3 interrupt custom code
    // or set custom function using INT3_SetInterruptHandler()
}

/**
 End of File
*/
