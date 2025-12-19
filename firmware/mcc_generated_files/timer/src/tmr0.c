/**
 * TMR0 Generated Timer Driver File
 * 
 * @file tmr0.c
 * 
 * @ingroup timerdriver
 * 
 * @brief Timer Driver implementation for the TMR0 driver
 *
 * @version TMR0 Timer Driver Version 2.0.0
 *
 * @version Package Version 2.0.0
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

#include <xc.h>
#include "../tmr0.h"

const struct TIMER_INTERFACE Timer0 = {
    .Initialize = TMR0_Initialize,
    .Deinitialize = TMR0_Deinitialize,
    .Start = TMR0_Start,
    .Stop = TMR0_Stop,
    .PeriodSet = TMR0_PeriodSet,
    .PeriodGet = TMR0_PeriodGet,
    .CounterGet = TMR0_CounterGet,
    .CounterSet = TMR0_CounterSet,
    .MaxCountGet = TMR0_MaxCountGet,
    .TimeoutCallbackRegister = TMR0_OverflowCallbackRegister,
    .Tasks = NULL
};

static volatile uint16_t tmr0PeriodCount;
static void (*TMR0_OverflowCallback)(void);
static void TMR0_DefaultOverflowCallback(void);

/**
  Section: TMR0 APIs
*/ 

void TMR0_Initialize(void)
{
    //Enable 16bit timer mode before assigning value to TMR0H
    T0CONbits.T08BIT = 0;
    TMR0H = 0x3C;                          // Period 100ms; Frequency 500000Hz; Count 15536
    TMR0L = 0xB0;
    
    tmr0PeriodCount = 15536U;
    
    TMR0_OverflowCallbackRegister(TMR0_DefaultOverflowCallback);
    
    INTCONbits.TMR0IF = 0;	   
    INTCONbits.TMR0IE = 1;	

    T0CON = (2 << _T0CON_T0PS_POSN)   // T0PS 1:8
        | (0 << _T0CON_PSA_POSN)   // PSA assigned
        | (1 << _T0CON_T0SE_POSN)   // T0SE Increment_hi_lo
        | (0 << _T0CON_T0CS_POSN)   // T0CS FOSC/4
        | (0 << _T0CON_T08BIT_POSN)   // T08BIT 16-bit
        | (1 << _T0CON_TMR0ON_POSN);  // TMR0ON enabled
}

void TMR0_Deinitialize(void)
{
    T0CONbits.TMR0ON = 0;
    
    INTCONbits.TMR0IF = 0;	   
    INTCONbits.TMR0IE = 0;		
    
    T0CON = 0xFF;
    TMR0H = 0x0;
    TMR0L =0x0;
}

void TMR0_Start(void)
{
    T0CONbits.TMR0ON = 1;
}

void TMR0_Stop(void)
{
    T0CONbits.TMR0ON = 0;
}


uint32_t TMR0_CounterGet(void)
{
    uint16_t counterValue;
    uint8_t counterLowByte;
    uint8_t counterHighByte;

    counterLowByte  = TMR0L;
    counterHighByte = TMR0H;
    counterValue  = ((uint16_t)counterHighByte << 8) + counterLowByte;

    return (uint32_t)counterValue;
}

void TMR0_CounterSet(uint32_t counterValue)
{
    TMR0H = (uint8_t)(counterValue >> 8);
    TMR0L = (uint8_t)(counterValue);
}

void TMR0_PeriodSet(uint32_t periodCount)
{
   tmr0PeriodCount = TMR0_MAX_COUNT - (uint16_t)periodCount;
   TMR0_CounterSet(tmr0PeriodCount);
}

uint32_t TMR0_PeriodGet(void)
{
    return ((uint32_t)TMR0_MAX_COUNT - tmr0PeriodCount);
}

uint32_t TMR0_MaxCountGet(void)
{
    return (uint32_t)TMR0_MAX_COUNT;
}


void TMR0_OverflowISR(void)
{

    //Reload TMR0
    //Write to the Timer0 register
    TMR0H = (uint8_t)(tmr0PeriodCount >> 8);
    TMR0L = (uint8_t)(tmr0PeriodCount);

    if(NULL != TMR0_OverflowCallback)
    {
        TMR0_OverflowCallback();
    }
    INTCONbits.TMR0IF = 0;
}

void TMR0_OverflowCallbackRegister(void (* callbackHandler)(void))
{
    TMR0_OverflowCallback = callbackHandler;
}

static void TMR0_DefaultOverflowCallback(void)
{
    // Default interrupt handler
}