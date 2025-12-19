/**
 * Water Meter Counter Module
 * 
 * @file water_meter.h
 * 
 * @brief This module counts water meter rotations using two LC tank circuits
 *        (CMP1 and CMP2) and outputs an S0 pulse interface.
 *        
 * The LC tank circuits provide resonance detection which transitions when
 * the meter mechanism rotates. The module detects these transitions and
 * generates pulse pulses on the S0 output.
 *
 * @version 1.0.0
 */

#ifndef WATER_METER_H
#define WATER_METER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Water meter state enumeration
 */
typedef enum {
    STATE_LC_A_HIGH,      /**< LC Tank A is in high state */
    STATE_LC_B_HIGH,      /**< LC Tank B is in high state */
    STATE_TRANSITION,     /**< Transition detected between tanks */
    STATE_INVALID         /**< Invalid or unknown state */
} water_meter_state_t;

/**
 * @brief Initialize the water meter counter module
 * 
 * This function initializes the state machine, comparator callbacks,
 * and prepares the S0 output interface.
 */
void WaterMeter_Initialize(void);

/**
 * @brief Process the water meter counter logic
 * 
 * This function should be called periodically in the main loop to
 * process the state machine and generate S0 pulses.
 * 
 * @return true if a full rotation was detected, false otherwise
 */
bool WaterMeter_Process(void);

/**
 * @brief Handle CMP1 (LC Tank A) interrupt
 * 
 * This callback is triggered when the LC Tank A resonance state changes.
 */
void WaterMeter_CMP1_Callback(void);

/**
 * @brief Handle CMP2 (LC Tank B) interrupt
 * 
 * This callback is triggered when the LC Tank B resonance state changes.
 */
void WaterMeter_CMP2_Callback(void);

/**
 * @brief Get the current rotation count
 * 
 * @return Number of full meter rotations detected
 */
uint32_t WaterMeter_GetRotationCount(void);

/**
 * @brief Reset the rotation counter
 */
void WaterMeter_ResetCounter(void);

/**
 * @brief Get current state of LC Tank A
 * 
 * @return true if LC Tank A is in high state, false otherwise
 */
bool WaterMeter_GetLCA_State(void);

/**
 * @brief Get current state of LC Tank B
 * 
 * @return true if LC Tank B is in high state, false otherwise
 */
bool WaterMeter_GetLCB_State(void);

/**
 * @brief Enable deep sleep mode with timer wakeup
 * 
 * Puts the PIC18 into sleep mode and uses TMR0 to wake up periodically
 * to check for water meter rotation. The wakeup period is calculated based
 * on Q_3 flow rate (2.5 m³/h) to ensure no rotation is missed.
 * 
 * Minimum timeout: 1500ms (1.5 seconds) to accommodate maximum flow rate
 * which is 1 liter per 1.44 seconds.
 * 
 * @param timeout_ms Timeout in milliseconds (minimum 1500ms recommended)
 */
void WaterMeter_EnterDeepSleep(uint16_t timeout_ms);

/**
 * @brief Check if we should enter sleep mode
 * 
 * Returns true if system is idle and safe to sleep
 * 
 * @return true if sleep is allowed, false otherwise
 */
bool WaterMeter_CanSleep(void);

/**
 * @brief Update sleep timer (called from TMR0 ISR context)
 * 
 * This function should be called from the TMR0 overflow interrupt handler
 * to decrement the sleep counter and trigger actions when timeout expires.
 * 
 * @return true if sleep timeout has been reached, false otherwise
 */
bool WaterMeter_UpdateSleepTimer(void);

/**
 * @brief Set sleep mode enabled state
 * 
 * @param enabled true to enable sleep mode, false to disable
 */
void WaterMeter_SetSleepEnabled(bool enabled);

/**
 * @brief Get current sleep timeout value
 * 
 * @return Current sleep timeout in milliseconds
 */
uint16_t WaterMeter_GetSleepTimeout(void);

/**
 * @brief Set calibration threshold for LC Tank A metal detection
 * 
 * Oscillation count below this threshold indicates metal is present.
 * Default: ~70% of maximum oscillation count (typically ~40 counts).
 * 
 * @param threshold Oscillation count threshold for metal detection
 */
void WaterMeter_SetLCA_Threshold(uint8_t threshold);

/**
 * @brief Set calibration threshold for LC Tank B metal detection
 * 
 * Oscillation count below this threshold indicates metal is present.
 * Default: ~70% of maximum oscillation count (typically ~40 counts).
 * 
 * @param threshold Oscillation count threshold for metal detection
 */
void WaterMeter_SetLCB_Threshold(uint8_t threshold);

/**
 * @brief Get last measured oscillation count for LC Tank A
 * 
 * Useful for calibration and debugging.
 * 
 * @return Number of oscillations measured in last cycle
 */
uint8_t WaterMeter_GetLCA_OscillationCount(void);

/**
 * @brief Get last measured oscillation count for LC Tank B
 * 
 * Useful for calibration and debugging.
 * 
 * @return Number of oscillations measured in last cycle
 */
uint8_t WaterMeter_GetLCB_OscillationCount(void);

/**
 * @brief Get raw CMP1 comparator output status
 * 
 * @return true if CMP1 output is high, false otherwise
 */
bool WaterMeter_GetCMP1_Output(void);

/**
 * @brief Get raw CMP2 comparator output status
 * 
 * @return true if CMP2 output is high, false otherwise
 */
bool WaterMeter_GetCMP2_Output(void);

/**
 * @brief Run diagnostic test on LC tank hardware
 * 
 * Tests CVref configuration and measures oscillation count ranges.
 * Should be called once at startup to verify hardware is working.
 * Prints results to UART.
 */
void WaterMeter_DiagnosticTest(void);

#endif // WATER_METER_H
/**
 * End of File
 */
