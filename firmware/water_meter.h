/**
 * Water Meter Counter Module Header
 * 
 * Dual LC tank metal detection for water meter disc rotation sensing.
 * 
 * Hardware:
 *   LCA: Drive=RC5, Sense=RB1 (CMP1)
 *   LCB: Drive=RC0, Sense=RA5 (CMP2)
 *   S0:  RB2 (pulse output)
 */

#ifndef WATER_METER_H
#define WATER_METER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize water meter module
 * 
 * Configures comparators, ANSEL bits, drive pins, and registers callbacks.
 * Must be called after SYSTEM_Initialize().
 */
void WaterMeter_Initialize(void);

/**
 * @brief Process water meter readings
 * 
 * Measures both LC tanks and detects metal presence transitions.
 * Raw ADC span is the primary metal-state input; comparator counts are kept
 * as diagnostics.
 * Counts a rotation only after a complete Gray-code cycle returns to air.
 * Generates S0 pulse on forward rotation detection.
 * Should be called periodically from main loop (~100ms interval).
 * 
 * @return true if rotation was detected, false otherwise
 */
bool WaterMeter_Process(void);

/**
 * @brief Get total forward rotation count
 * @return Number of forward disc rotations (LCA leads in the mounted configuration)
 */
uint32_t WaterMeter_GetRotationCount(void);

/**
 * @brief Get forward rotation count
 * @return Number of forward disc rotations (LCA leads in the mounted configuration)
 */
uint32_t WaterMeter_GetForwardCount(void);

/**
 * @brief Get reverse rotation count
 * @return Number of reverse disc rotations (LCB leads in the mounted configuration)
 */
uint32_t WaterMeter_GetReverseCount(void);

/**
 * @brief Get direction of the most recently completed rotation event
 * @return +1 for forward, -1 for reverse, 0 for no event in the last process call
 */
int8_t WaterMeter_GetLastRotationDirection(void);

/**
 * @brief Get current Gray code state
 * @return State 0-3: (LCA_metal<<1) | LCB_metal
 */
uint8_t WaterMeter_GetState(void);

/**
 * @brief Get last LCA oscillation count
 * @return Edge count from last LCA measurement
 */
uint8_t WaterMeter_GetLCA_Count(void);

/**
 * @brief Get last LCB oscillation count
 * @return Edge count from last LCB measurement
 */
uint8_t WaterMeter_GetLCB_Count(void);

/**
 * @brief Get last LCA polled transition count
 * @return Comparator output transitions seen by tight polling during LCA measurement
 */
uint8_t WaterMeter_GetLCA_PolledCount(void);

/**
 * @brief Get current comparator reference setting
 * @return Current CVR value written to CVRCON
 */
uint8_t WaterMeter_GetCVRef(void);

/**
 * @brief Get last raw ADC minimum sample from LCA ring-down capture
 * @return Minimum ADC code seen during the last LCA raw capture
 */
uint16_t WaterMeter_GetLCA_RawMin(void);

/**
 * @brief Get last raw ADC maximum sample from LCA ring-down capture
 * @return Maximum ADC code seen during the last LCA raw capture
 */
uint16_t WaterMeter_GetLCA_RawMax(void);

/**
 * @brief Get last raw ADC span from LCA ring-down capture
 * @return max-min ADC code seen during the last LCA raw capture
 */
uint16_t WaterMeter_GetLCA_RawSpan(void);

/**
 * @brief Get last raw ADC span from LCB ring-down capture
 * @return max-min ADC code seen during the last LCB raw capture
 */
uint16_t WaterMeter_GetLCB_RawSpan(void);

/**
 * @brief Get tracked air baseline for the LCA raw span
 * @return Slowly tracked baseline span for channel A
 */
uint16_t WaterMeter_GetLCA_SpanBaseline(void);

/**
 * @brief Get tracked air baseline for the LCB raw span
 * @return Slowly tracked baseline span for channel B
 */
uint16_t WaterMeter_GetLCB_SpanBaseline(void);

/**
 * @brief Check whether a new CVREF sweep result is available
 * @return true if the latest sweep has not been consumed yet
 */
bool WaterMeter_IsDiagnosticSweepReady(void);

/**
 * @brief Mark the current CVREF sweep results as consumed
 */
void WaterMeter_ClearDiagnosticSweepReady(void);

/**
 * @brief Get number of points in the periodic CVREF sweep
 * @return Number of sweep points currently populated
 */
uint8_t WaterMeter_GetDiagnosticPointCount(void);

/**
 * @brief Get CVREF value for a sweep point
 * @param index Sweep point index
 * @return CVR value for the requested sweep point, or 0 if out of range
 */
uint8_t WaterMeter_GetDiagnosticCVRef(uint8_t index);

/**
 * @brief Get ISR edge count for the A-channel sweep point
 * @param index Sweep point index
 * @return LCA comparator ISR edge count, or 0 if out of range
 */
uint8_t WaterMeter_GetDiagnosticLCA(uint8_t index);

/**
 * @brief Get ISR edge count for the B-channel sweep point
 * @param index Sweep point index
 * @return LCB comparator ISR edge count, or 0 if out of range
 */
uint8_t WaterMeter_GetDiagnosticLCB(uint8_t index);

/**
 * @brief Get polled comparator transition count for the A-channel sweep point
 * @param index Sweep point index
 * @return LCA polled transition count, or 0 if out of range
 */
uint8_t WaterMeter_GetDiagnosticAPl(uint8_t index);

/**
 * @brief Get polled comparator transition count for the B-channel sweep point
 * @param index Sweep point index
 * @return LCB polled transition count, or 0 if out of range
 */
uint8_t WaterMeter_GetDiagnosticBPl(uint8_t index);

/**
 * @brief Get raw ADC span for the A-channel sweep point
 * @param index Sweep point index
 * @return LCA raw ADC span, or 0 if out of range
 */
uint16_t WaterMeter_GetDiagnosticRawSpan(uint8_t index);

/**
 * @brief Get raw ADC span for the B-channel sweep point
 * @param index Sweep point index
 * @return LCB raw ADC span, or 0 if out of range
 */
uint16_t WaterMeter_GetDiagnosticRawSpanB(uint8_t index);

#endif /* WATER_METER_H */
