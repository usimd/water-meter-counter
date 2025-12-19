/**
 * Configuration Bits Generated Driver Source File
 * 
 * @file config_bits.c
 * 
 * @ingroup config_bitsdriver
 * 
 * @brief This file contains the API Implementation for the Device Configuration Bits driver.
 *
 * @version Driver Version 1.0.1
 *
 * @version Package Version 1.0.3
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

// Configuration bits: selected in the GUI

//CONFIG1L
#pragma config RETEN = ON    // VREG Sleep Enable bit->Ultra low-power regulator is Enabled (Controlled by SRETEN bit)
#pragma config INTOSCSEL = HIGH    // LF-INTOSC Low-power Enable bit->LF-INTOSC in High-power mode during Sleep
#pragma config SOSCSEL = HIGH    // SOSC Power Selection and mode Configuration bits->High Power SOSC circuit selected
#pragma config XINST = OFF    // Extended Instruction Set->Disabled

//CONFIG1H
#pragma config FOSC = INTIO2    // Oscillator Selection bits->Internal RC oscillator
#pragma config FCMEN = OFF    // Fail-Safe Clock Monitor Enable bit->Disabled
#pragma config IESO = OFF    // Internal/External Oscillator Switchover bit->Disabled
#pragma config PLLCFG = OFF    // 4X PLL Enable->Disabled

//CONFIG2L
#pragma config PWRTEN = OFF    // Power-up Timer Enable bit->Disabled
#pragma config BOREN = SBORDIS    // Brown-out Reset Enable bits->Enabled in hardware, SBOREN disabled
#pragma config BORV = 0    // Brown Out Reset Voltage bits->3.0V
#pragma config BORPWR = ZPBORMV    // BORMV Power level->ZPBORMV instead of BORMV is selected

//CONFIG2H
#pragma config WDTEN = NOSLP    // Watchdog Timer Enable bits->WDT enabled only while device is active and disabled in Sleep mode; SWDTEN bit disabled
#pragma config WDTPS = 1048576    // Watchdog Timer Postscale Select bits->1:1048576

//CONFIG3H
#pragma config MCLRE = ON    // MCLR Pin Enable bit->MCLR Enabled, RE3 Disabled
#pragma config CANMX = PORTB    // ECAN Mux bit->PORTB
#pragma config MSSPMSK = MSK7    // MSSP address masking->7 Bit address masking mode

//CONFIG4L
#pragma config STVREN = ON    // Stack Full/Underflow Reset Enable bit->Enabled
#pragma config BBSIZ = BB2K    // Boot Block Size Select bit->2K word Boot Block size

//CONFIG5L
#pragma config CP0 = OFF    // Code Protection Block 0->Disabled
#pragma config CP1 = OFF    // Code Protection Block 1->Disabled
#pragma config CP2 = OFF    // Code Protection Block 2->Disabled
#pragma config CP3 = OFF    // Code Protection Block 3->Disabled

//CONFIG5H
#pragma config CPB = OFF    // Boot Block Code Protection bit->Disabled
#pragma config CPD = OFF    // Data EEPROM Code Protection bit->Disabled

//CONFIG6L
#pragma config WRT0 = OFF    // Write Protection Block 0->Disabled
#pragma config WRT1 = OFF    // Write Protection Block 1->Disabled
#pragma config WRT2 = OFF    // Write Protection Block 2->Disabled
#pragma config WRT3 = OFF    // Write Protection Block 3->Disabled

//CONFIG6H
#pragma config WRTC = OFF    // Configuration Register Write Protection bit->Disabled
#pragma config WRTB = OFF    // Boot Block Write Protection bit->Disabled
#pragma config WRTD = OFF    // Data EEPROM Write Protection bit->Disabled

//CONFIG7L
#pragma config EBTR0 = OFF    // Table Read Protection Block 0->Disabled
#pragma config EBTR1 = OFF    // Table Read Protection Block 1->Disabled
#pragma config EBTR2 = OFF    // Table Read Protection Block 2->Disabled
#pragma config EBTR3 = OFF    // Table Read Protection Block 3->Disabled

//CONFIG7H
#pragma config EBTRB = OFF    // Boot Block Table Read Protection bit->OFF

/**
 End of File
*/