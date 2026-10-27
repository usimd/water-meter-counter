// SPDX-FileCopyrightText: 2025-2026 usimd
// SPDX-License-Identifier: Apache-2.0

/**
 * Rotation decoder: counts revolutions of the meter's half-disc from one
 * ring-down reading of each LC tank per sample.
 *
 * Hardware independent, so it can be compiled and tested on a host.
 *
 * Both channels are comparator times (0.25 us ticks). Channel B alternates
 * between a rest level and a higher pass level once per revolution (rev2
 * bench: ~107 and ~132 ticks). Channel A changes in between, lagging B, and
 * its level varies between mounts, so it is only compared with itself: in
 * forward rotation A is higher at the start of a B pass than at its end
 * (bench: ~69 -> ~57), in reverse the other way round. A pass that enters and
 * leaves over the same edge (a wobble) shows no difference and is not
 * counted.
 */

#ifndef ROTATION_DECODER_H
#define ROTATION_DECODER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Reset the decoder
 * @param baseline_b      Initial rest level of channel B (median of several readings)
 * @param level_a,level_b First readings
 *
 * If B is in a pass at reset, that first pass is not counted.
 */
void RotationDecoder_Init(uint16_t baseline_b, uint16_t level_a, uint16_t level_b);

/**
 * @brief Feed one sample pair
 * @return +1 for a forward revolution, -1 for reverse, 0 otherwise
 *
 * A revolution is reported when the B pass ends.
 */
int8_t RotationDecoder_Update(uint16_t level_a, uint16_t level_b);

/** @brief true during a B pass */
bool RotationDecoder_IsInPass(void);

uint16_t RotationDecoder_GetBaselineB(void);

#endif /* ROTATION_DECODER_H */
