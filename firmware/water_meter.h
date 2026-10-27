// SPDX-FileCopyrightText: 2025-2026 usimd
// SPDX-License-Identifier: Apache-2.0

/**
 * Water meter counter: dual LC-tank sensing of the meter's half-disc.
 *
 * Hardware (rev2):
 *   Channel A: drive RC5, sense RB1 (C1INB)
 *   Channel B: drive RC0, sense RA5 (C2INB)
 *   S0 output: RB4
 *
 * Each tank is charged through its drive pin and released, and the sense
 * node rings down. Comparator 1 against CVREF gates Timer1, which times how
 * long the ring spends below the threshold; the disc changes that time.
 * The ADC is not used. See rotation_decoder.h for the decoding.
 */

#ifndef WATER_METER_H
#define WATER_METER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t ringdown_a;  /* time below the comparator threshold, 0.25 us */
    uint16_t ringdown_b;
    uint16_t baseline_a;  /* A at boot; A is only compared with itself */
    uint16_t baseline_b;  /* tracked rest level of B */
    bool in_pass_b;       /* B is in its pass level */
} water_meter_status_t;

/**
 * @brief Configure pins and peripherals and learn the air baselines
 * @param wait_for_sample Waits between samples exactly like the main loop
 *                        (B's readings depend on it, e.g. Sleep vs awake)
 *
 * Call after SYSTEM_Initialize() with interrupts enabled. If B is in its
 * pass level at boot, the baseline recovers by tracking and that first pass
 * is not counted.
 */
void WaterMeter_Initialize(void (*wait_for_sample)(void));

/**
 * @brief Measure both tanks once and update the rotation decoder
 *
 * Call every ~70 ms (the decoder was tuned at 70..100 ms; it handles at
 * least ~9 samples per revolution). Forward rotations also emit a 100 ms S0
 * pulse.
 *
 * @return +1 forward rotation, -1 reverse rotation, 0 otherwise
 */
int8_t WaterMeter_Process(void);

uint32_t WaterMeter_GetForwardCount(void);
uint32_t WaterMeter_GetReverseCount(void);

/** @brief Snapshot of the last measurement, for debugging/tuning */
void WaterMeter_GetStatus(water_meter_status_t *status);

/**
 * @brief Whether an S0 pulse is in progress
 *
 * The pulse is timed by TMR0, which stops in sleep: stay awake until it ends.
 */
bool WaterMeter_IsS0PulseActive(void);

#endif /* WATER_METER_H */
