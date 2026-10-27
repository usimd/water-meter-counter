/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.1.1
*/

/*
� [2026] Microchip Technology Inc. and its subsidiaries.

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

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RA5 aliases
#define IO_RA5_TRIS                 TRISAbits.TRISA5
#define IO_RA5_LAT                  LATAbits.LATA5
#define IO_RA5_PORT                 PORTAbits.RA5
#define IO_RA5_WPU                  WPUAbits.
#define IO_RA5_OD                   ODCONAbits.
#define IO_RA5_ANS                  ANSELAbits.ANSEL4
#define IO_RA5_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define IO_RA5_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define IO_RA5_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define IO_RA5_GetValue()           PORTAbits.RA5
#define IO_RA5_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define IO_RA5_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define IO_RA5_SetPullup()          do { WPUAbits. = 1; } while(0)
#define IO_RA5_ResetPullup()        do { WPUAbits. = 0; } while(0)
#define IO_RA5_SetPushPull()        do { ODCONAbits. = 0; } while(0)
#define IO_RA5_SetOpenDrain()       do { ODCONAbits. = 1; } while(0)
#define IO_RA5_SetAnalogMode()      do { ANSELAbits.ANSEL4 = 1; } while(0)
#define IO_RA5_SetDigitalMode()     do { ANSELAbits.ANSEL4 = 0; } while(0)

// get/set RB1 aliases
#define IO_RB1_TRIS                 TRISBbits.TRISB1
#define IO_RB1_LAT                  LATBbits.LATB1
#define IO_RB1_PORT                 PORTBbits.RB1
#define IO_RB1_WPU                  WPUBbits.WPUB1
#define IO_RB1_OD                   ODCONBbits.
#define IO_RB1_ANS                  ANSELBbits.ANSEL8
#define IO_RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define IO_RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define IO_RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define IO_RB1_GetValue()           PORTBbits.RB1
#define IO_RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define IO_RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define IO_RB1_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define IO_RB1_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define IO_RB1_SetPushPull()        do { ODCONBbits. = 0; } while(0)
#define IO_RB1_SetOpenDrain()       do { ODCONBbits. = 1; } while(0)
#define IO_RB1_SetAnalogMode()      do { ANSELBbits.ANSEL8 = 1; } while(0)
#define IO_RB1_SetDigitalMode()     do { ANSELBbits.ANSEL8 = 0; } while(0)

// get/set RB4 aliases
#define S0_TRIS                 TRISBbits.TRISB4
#define S0_LAT                  LATBbits.LATB4
#define S0_PORT                 PORTBbits.RB4
#define S0_WPU                  WPUBbits.WPUB4
#define S0_OD                   ODCONBbits.
#define S0_ANS                  ANCON1bits.ANSEL9
#define S0_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define S0_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define S0_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define S0_GetValue()           PORTBbits.RB4
#define S0_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define S0_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define S0_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define S0_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
// RB4 has no per-pin open-drain control; ANSEL9 uses 1=analog, 0=digital
#define S0_SetPushPull()        do { } while(0)  // No ODCONB bit for RB4
#define S0_SetOpenDrain()       do { } while(0)  // No ODCONB bit for RB4
#define S0_SetAnalogMode()      do { ANCON1bits.ANSEL9 = 1; } while(0)
#define S0_SetDigitalMode()     do { ANCON1bits.ANSEL9 = 0; } while(0)

// get/set RC0 aliases
#define LCB_TRIS                 TRISCbits.TRISC0
#define LCB_LAT                  LATCbits.LATC0
#define LCB_PORT                 PORTCbits.RC0
// RC0 has no WPU, ODCON, or ANSEL bits (SOSC pins are special)
// These macros are intentionally left incomplete - do not use them
#define LCB_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define LCB_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define LCB_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define LCB_GetValue()           PORTCbits.RC0
#define LCB_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define LCB_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
// RC0 has no WPU, ODCON, or ANSEL bits - these are no-ops
#define LCB_SetPullup()          do { } while(0)
#define LCB_ResetPullup()        do { } while(0)
#define LCB_SetPushPull()        do { } while(0)
#define LCB_SetOpenDrain()       do { } while(0)
#define LCB_SetAnalogMode()      do { } while(0)
#define LCB_SetDigitalMode()     do { } while(0)

// get/set RC5 aliases
#define LCA_TRIS                 TRISCbits.TRISC5
#define LCA_LAT                  LATCbits.LATC5
#define LCA_PORT                 PORTCbits.RC5
// RC5 has no WPU, ODCON, or ANSEL bits
// These macros are intentionally left incomplete - do not use them
#define LCA_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define LCA_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define LCA_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define LCA_GetValue()           PORTCbits.RC5
#define LCA_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define LCA_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
// RC5 has no WPU, ODCON, or ANSEL bits - these are no-ops
#define LCA_SetPullup()          do { } while(0)
#define LCA_ResetPullup()        do { } while(0)
#define LCA_SetPushPull()        do { } while(0)
#define LCA_SetOpenDrain()       do { } while(0)
#define LCA_SetAnalogMode()      do { } while(0)
#define LCA_SetDigitalMode()     do { } while(0)

// get/set RC6 aliases
#define IO_RC6_TRIS                 TRISCbits.TRISC6
#define IO_RC6_LAT                  LATCbits.LATC6
#define IO_RC6_PORT                 PORTCbits.RC6
// RC6 has no WPU, ODCON, or ANSEL bits (EUSART TX pin)
// These macros are intentionally left incomplete - do not use them
#define IO_RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define IO_RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define IO_RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define IO_RC6_GetValue()           PORTCbits.RC6
#define IO_RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define IO_RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
// RC6 has no WPU, ODCON, or ANSEL bits - these are no-ops
#define IO_RC6_SetPullup()          do { } while(0)
#define IO_RC6_ResetPullup()        do { } while(0)
#define IO_RC6_SetPushPull()        do { } while(0)
#define IO_RC6_SetOpenDrain()       do { } while(0)
#define IO_RC6_SetAnalogMode()      do { } while(0)
#define IO_RC6_SetDigitalMode()     do { } while(0)

// get/set RC7 aliases
#define IO_RC7_TRIS                 TRISCbits.TRISC7
#define IO_RC7_LAT                  LATCbits.LATC7
#define IO_RC7_PORT                 PORTCbits.RC7
// RC7 has no WPU, ODCON, or ANSEL bits (EUSART RX pin)
// These macros are intentionally left incomplete - do not use them
#define IO_RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define IO_RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define IO_RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define IO_RC7_GetValue()           PORTCbits.RC7
#define IO_RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define IO_RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
// RC7 has no WPU, ODCON, or ANSEL bits - these are no-ops
#define IO_RC7_SetPullup()          do { } while(0)
#define IO_RC7_ResetPullup()        do { } while(0)
#define IO_RC7_SetPushPull()        do { } while(0)
#define IO_RC7_SetOpenDrain()       do { } while(0)
#define IO_RC7_SetAnalogMode()      do { } while(0)
#define IO_RC7_SetDigitalMode()     do { } while(0)

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/
