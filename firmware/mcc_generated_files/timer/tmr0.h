/**
 * TMR0 Generated Timer Driver API Header File
 * 
 * @file tmr0.h
 * 
 * @ingroup timerdriver
 * 
 * @brief This file contains the API prototypes and other data types for the TMR0 Timer Driver.
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

#ifndef TMR0_H
#define TMR0_H

#include <stdint.h>
#include <stdbool.h>
#include "timer_interface.h"

/**
 * @misradeviation{@advisory,2.5}
 * MCC Melody drivers provide macros that can be added to an application. 
 * It depends on the application whether a macro is used or not. 
 */

/**
 * @ingroup timerdriver
 * @brief Defines the maximum count of the timer.
 */
#define TMR0_MAX_COUNT (65535U)

/**
 * @ingroup tmr016bit
 * @brief Defines the timer clock frequency in hertz.
 */
 /* cppcheck-suppress misra-c2012-2.5 */
#define TMR0_CLOCK_FREQ (500000UL)

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TMR0_MAX_COUNT
 */
 /* cppcheck-suppress misra-c2012-2.5 */
#define TIMER0_MAX_COUNT TMR0_MAX_COUNT

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TMR0_CLOCK_FREQ.
 */
/* cppcheck-suppress misra-c2012-2.5 */  
#define TIMER0_CLOCK_FREQ TMR0_CLOCK_FREQ

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_Initialize API.
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_Initialize TMR0_Initialize

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_Deinitialize API.
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_Deinitialize TMR0_Deinitialize

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_Start API.
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_Start TMR0_Start

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_Stop API.
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_Stop TMR0_Stop

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_CounterGet API.
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_CounterGet TMR0_CounterGet

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_CounterSet API.
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_CounterSet TMR0_CounterSet

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_PeriodSet API
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_PeriodSet TMR0_PeriodSet

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_PeriodGet API
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_PeriodGet TMR0_PeriodGet

/**
 * @ingroup timerdriver
 * @brief Defines the Custom Name for the \ref TMR0_MaxCountGet API
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_MaxCountGet TMR0_MaxCountGet

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_OverflowISR API
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_OverflowISR TMR0_OverflowISR

/**
 * @ingroup tmr0
 * @brief Defines the Custom Name for the \ref TMR0_OverflowCallbackRegister API
 */
 /* cppcheck-suppress misra-c2012-2.5 */ 
#define Timer0_OverflowCallbackRegister TMR0_OverflowCallbackRegister

/**
 @ingroup tmr0
 @struct TIMER_INTERFACE
 @brief Declares an instance of TIMER_INTERFACE for the TMR0 module
 */
extern const struct TIMER_INTERFACE Timer0;

/**
 * @ingroup tmr0
 * @brief Initializes the Timer0 (TMR0) module.
 *        This routine must be called before any other TMR0 routines.
 * @param None.
 * @return None.
 */
void TMR0_Initialize(void);

/**
 * @ingroup tmr0
 * @brief Deinitializes the TMR0 module.
 * @param None.
 * @return None.
 */
void TMR0_Deinitialize(void);

/**
 * @ingroup tmr0
 * @brief Starts the TMR0 timer.
 * @pre Initialize TMR0 with TMR0_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TMR0_Start(void);

/**
 * @ingroup tmr0
 * @brief Stops the TMR0 timer.
 * @pre Initialize TMR0 with TMR0_Initialize() before calling this API.
 * @param None.
 * @return None.
 */
void TMR0_Stop(void);

/**
 * @ingroup tmr0
 * @brief Returns the current counter value.
 * @pre Initialize TMR0 with TMR0_Initialize() before calling this API.
 * @param None.
 * @return Counter value from the TMR0 register
 */
uint32_t TMR0_CounterGet(void);

/**
 * @ingroup tmr0
 * @brief Sets the counter value.
 * @pre Initialize TMR0 with TMR0_Initialize() before calling this API.
 * @param counterValue - Counter value to be written to the TMR0 register
 * @return None.
 */
void TMR0_CounterSet(uint32_t counterValue);

/**
 * @ingroup tmr0
 * @brief Sets the period count value.
 * @pre Initialize TMR0 with TMR0_Initialize() before calling this API.
 * @param periodCount - Period count value to be written to the TMR0 register
 * @return None.
 */
void TMR0_PeriodSet(uint32_t periodCount);

/**
 * @ingroup tmr0
 * @brief Returns the current period value.
 * @pre Initialize TMR0 with TMR0_Initialize() before calling this API.
 * @param None.
 * @return Period count value
 */
uint32_t TMR0_PeriodGet(void);

/**
 * @ingroup timerdriver
 * @brief Returns the maximum count value.
 * @param None.
 * @return Maximum count value
 */
uint32_t TMR0_MaxCountGet(void);

/**
 * @ingroup tmr0
 * @brief Interrupt Service Routine (ISR) for the TMR0 overflow interrupt.
 * @param None.
 * @return None.
 */
void TMR0_OverflowISR(void);

/**
 * @ingroup tmr0
 * @brief Registers a callback function for the TMR0 overflow event.
 * @param CallbackHandler - Address of the custom callback function
 * @return None.
 */
 void TMR0_OverflowCallbackRegister(void (* CallbackHandler)(void));


/**
 * @}
 */
#endif //TMR0_H