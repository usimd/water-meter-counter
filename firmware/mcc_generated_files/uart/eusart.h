/**
 * EUSART Generated Driver API Header File
 * 
 * @file eusart.h
 * 
 * @defgroup eusart EUSART
 * 
 * @brief This file contains API prototypes and other datatypes for the Enhanced Universal Synchronous and Asynchronous Receiver Transceiver (EUSART) module.
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

#ifndef EUSART_H
#define EUSART_H


#include <stdbool.h>
#include <stdint.h>
/**
    @ingroup eusart 
    @def Standard Input Output functions
    @misradeviation{@required, 21.6} This inclusion is essential for UART module to use Printf function for print the character.
*/
/* cppcheck-suppress misra-c2012-21.6 */
#include <stdio.h>
#include "../system/system.h"
#include "uart_drv_interface.h"

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif


#define UART_interface UART


#define UART_Initialize     EUSART_Initialize
#define UART_Deinitialize   EUSART_Deinitialize
#define UART_Write          EUSART_Write
#define UART_Read           EUSART_Read
#define UART__IsRxReady     EUSART_IsRxReady
#define UART_IsTxReady      EUSART_IsTxReady
#define UART_IsTxDone       EUSART_IsTxDone

#define UART_TransmitEnable       EUSART_TransmitEnable
#define UART_TransmitDisable      EUSART_TransmitDisable
#define UART_AutoBaudSet          EUSART_AutoBaudSet
#define UART_AutoBaudQuery        EUSART_AutoBaudQuery
#define UART_BRGCountSet                (NULL)
#define UART_BRGCountGet                (NULL)
#define UART_BaudRateSet                (NULL)
#define UART_BaudRateGet                (NULL)
#define UART__AutoBaudEventEnableGet    (NULL)
#define UART_ErrorGet             EUSART_ErrorGet

#define UART_TxCompleteCallbackRegister     (NULL)
#define UART_RxCompleteCallbackRegister      (NULL)
#define UART_TxCollisionCallbackRegister  (NULL)
#define UART_FramingErrorCallbackRegister EUSART_FramingErrorCallbackRegister
#define UART_OverrunErrorCallbackRegister EUSART_OverrunErrorCallbackRegister
#define UART_ParityErrorCallbackRegister  (NULL)
#define UART_EventCallbackRegister        (NULL)


/**
 @ingroup eusart
 @struct eusart_status_t
 @brief This is a structure defined for errors in reception of data.
 */
 /**
 * @misradeviation{@advisory,19.2}
 * The UART error status necessitates checking the bitfield and accessing the status within the group byte therefore the use of a union is essential.
 */
 /* cppcheck-suppress misra-c2012-19.2 */
typedef union {
    struct {
        uint8_t perr : 1;     /**<This is a bit field for Parity Error status*/
        uint8_t ferr : 1;     /**<This is a bit field for Framing Error status*/
        uint8_t oerr : 1;     /**<This is a bit field for Overfrun Error status*/
        uint8_t reserved : 5; /**<Reserved*/
    };
    size_t status;            /**<Group byte for status errors*/
}eusart_status_t;





/**
 * @ingroup eusart
 * @brief External object for eusart_interface.
 */
extern const uart_drv_interface_t UART;

/**
 * @ingroup eusart
 * @brief Initializes the EUSART module. This routine is called
 *        only once during system initialization, before calling other APIs.
 * @param None.
 * @return None.
 */
void EUSART_Initialize(void);

/**
 * @ingroup eusart
 * @brief Deinitializes and disables the EUSART module.
 * @param None.
 * @return None.
 */
void EUSART_Deinitialize(void);

/**
 * @ingroup eusart
 * @brief This API enables the EUSART module.     
 * @param None.
 * @return None.
 */
void EUSART_Enable(void);

/**
 * @ingroup eusart
 * @brief This API disables the EUSART module.
 * @param None.
 * @return None.
 */
void EUSART_Disable(void);

/**
 * @ingroup eusart
 * @brief This API enables the EUSART transmitter.
 *        The EUSART must be enabled to send the bytes over to the TX pin.
 * @param None.
 * @return None.
 */
void EUSART_TransmitEnable(void);

/**
 * @ingroup eusart
 * @brief This API disables the EUSART transmitter.
 * @param None.
 * @return None.
 */
void EUSART_TransmitDisable(void);

/**
 * @ingroup eusart
 * @brief This API enables the EUSART receiver.
 *        The EUSART must be enabled to receive the bytes sent by the RX pin.
 * @param None.
 * @return None.
 */
void EUSART_ReceiveEnable(void);

/**
 * @ingroup eusart
 * @brief This API disables the EUSART receiver.
 * @param None.
 * @return None.
 */
void EUSART_ReceiveDisable(void);


/**
 * @ingroup eusart
 * @brief This API enables the EUSART to send a break control. 
 * @param None.
 * @return None.
 */
void EUSART_SendBreakControlEnable(void);

/**
 * @ingroup eusart
 * @brief This API disables the EUSART send break control.
 * @param None.
 * @return None.
 */
void EUSART_SendBreakControlDisable(void);

/**
 * @ingroup eusart
 * @brief This API enables the EUSART AutoBaud Detection (ABR). 
 * @param bool enable.
 * @return None.
 */
void EUSART_AutoBaudSet(bool enable);

/**
 * @ingroup eusart
 * @brief This API reads the EUSART ABR Complete bit.
 * @param None.
 * @return bool.
 */
bool EUSART_AutoBaudQuery(void);

/**
 * @ingroup eusart
 * @brief This API reads the EUSART ABR Overflow bit.
 * @param None.
 * @return None.
 */
bool EUSART_IsAutoBaudDetectOverflow(void);

/**
 * @ingroup eusart
 * @brief This API resets the EUSART ABR Overflow bit.
 * @param None.
 * @return None.
 */
void EUSART_AutoBaudDetectOverflowReset(void);

/**
 * @ingroup eusart
 * @brief This API checks if the EUSART has received available data.
 * @param None.
 * @retval true if EUSART receiver FIFO has a data
 * @retval false EUSART receiver FIFO is empty
 */
bool EUSART_IsRxReady(void);

/**
 * @ingroup eusart
 * @brief This function checks if the EUSART transmitter is ready to accept a data byte.
 * @param None.
 * @retval true if EUSART transmitter FIFO has atleast 1 byte space
 * @retval false if EUSART transmitter FIFO is full
 */
bool EUSART_IsTxReady(void);

/**
 * @ingroup eusart
 * @brief This function returns the status of Transmit Shift Register (TSR).
 * @param None.
 * @retval true if Data completely shifted out from the TSR
 * @retval false if Data is present in Transmit FIFO and/or in TSR
 */
bool EUSART_IsTxDone(void);

/**
 * @ingroup eusart
 * @brief This function receives the error status of the last read byte.
 * @param None.
 * @return Status of the last read byte. See eusart_status_t struct for more details.
 */
size_t EUSART_ErrorGet(void);

/**
 * @ingroup eusart
 * @brief This function reads the 8 bits from the FIFO register receiver.
 * @pre The transfer status must be checked to see if the receiver is not empty
 *      before calling this function. Verify the EUSART_IsRxReady(), before calling this API.
 * @param None.
 * @return 8-bit data from RX FIFO register.
 */
uint8_t EUSART_Read(void);

/**
 * @ingroup eusart
 * @brief This function writes a byte of data to the transmitter FIFO register.
 * @pre The transfer status must be checked to see if the transmitter is ready to accept a byte
 *      before calling this function. Verify the EUSART_IsTxReady() before calling this API.
 * @param txData  - Data byte to write to the TX FIFO.
 * @return None.
 */
void EUSART_Write(uint8_t txData);

/**
 * @ingroup eusart
 * @brief This API registers the function to be called upon framing error.
 * @param callbackHandler - a function pointer which will be called upon framing error condition.
 * @return None.
 */
void EUSART_FramingErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup eusart
 * @brief This API registers the function to be called upon overrun error.
 * @param callbackHandler - a function pointer which will be called upon overrun error condition.
 * @return None.
 */
void EUSART_OverrunErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup eusart
 * @brief This function used to printf support for reads the 8 bits from the FIFO register receiver.
 * @param None.
 * @return 8-bit data from RX FIFO register.
 */
int getch(void);

/**
 * @ingroup eusart
 * @brief This function used to printf support for writes a byte of data to the transmitter FIFO register.
 * @param txData  - Data byte to write to the TX FIFO.
 * @return None.
 */
void putch(char txData);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif  // EUSART_H
