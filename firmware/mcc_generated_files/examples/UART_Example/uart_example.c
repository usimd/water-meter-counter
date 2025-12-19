/**
 * UART Write String - Callback Example Driver File
 * 
 * @file uart_example.c
 * 
 * @addtogroup uart-example
 * 
 * @brief This is the generated example implementation for UART Write String in Callback mode
 *
 * @version UART Write String - Callback Example Version 1.0.0
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

/* Use Case 2 Callback implementation. Copy this code to your project source, e.g., to main.c  */
/* ------------------------------------------------------------------

#include "mcc_generated_files/system/system.h"
#include <string.h>

void UART_WriteString(const char *message);

void UART_WriteString(const char *message)
{
    for(int i = 0; i < (int)strlen(message); i++)
    {
        (void) UART.Write(message[i]);
    }
}

void main(void)
{
    const char message[] = "Hello World!\r\n";
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptEnable();
    (void) UART_WriteString(message); 

    while(1)
    {
        ;
    }    
}
------------------------------------------------------------------ */
/**
 End of File
*/