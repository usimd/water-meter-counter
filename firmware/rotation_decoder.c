// SPDX-FileCopyrightText: 2025-2026 usimd
// SPDX-License-Identifier: Apache-2.0

/**
 * Rotation decoder, see rotation_decoder.h.
 *
 * A B pass is a rise above B's tracked rest level. A is not thresholded: its
 * level varies between mounts. Instead the mean A level over the first third
 * of each B pass is compared with the mean over its last third.
 *
 * B thresholds are mid-plateau of the ranges that decoded every pass, forward
 * and time-reversed, in offline replay of the rev2 capture at ~100 ms per
 * sample (enter 6..18, hold 3..10). Replays at a third of the sample rate
 * (~4 m3/h at 1 L per revolution) still decoded correctly.
 */

#include "rotation_decoder.h"

/* B rise above its tracked rest level, in comparator ticks. */
#define B_RISE_ENTER          12U  /* pass starts */
#define B_RISE_HOLD           6U   /* pass continues */
#define B_BASELINE_BAND       3U   /* larger rises do not pull the baseline up */

#define RELEASE_SAMPLES       2U   /* below-hold samples before the pass ends */

/* A is averaged over the first and the last 1/PASS_FRACTION of a B pass,
 * but over at most WINDOW_MAX samples so that a disc parked over B for a
 * long time still compares the edges. */
#define PASS_FRACTION         3U
#define WINDOW_MAX            32U

/* Minimum change of the A mean between the windows, in comparator ticks. A
 * sits on a fixed offset (~56..70 ticks), so the margin is absolute. Bench
 * passes changed by ~12. */
#define DIRECTION_MARGIN      3U

/* On release, the newest RELEASE_SAMPLES samples are already past the B exit. */
#define HISTORY_LENGTH        (WINDOW_MAX + RELEASE_SAMPLES)

static uint16_t a_history[HISTORY_LENGTH];  /* ring of the newest A levels */
static uint8_t history_head;
static uint16_t a_pass_start[WINDOW_MAX];   /* first A levels of the current B pass */
static uint16_t pass_samples;               /* samples since B entry, saturating */

static uint16_t baseline_b;
static bool in_pass;
static bool pass_has_entry;
static uint8_t below_count;

static uint16_t Rise(uint16_t baseline, uint16_t level)
{
    return (level > baseline) ? (uint16_t)(level - baseline) : 0U;
}

/* Median tracker, one count per sample outside a pass. Samples already well
 * above the baseline are ignored so a slowly approaching disc edge cannot
 * drag it toward the pass level. */
static uint16_t UpdateBaseline(uint16_t baseline, uint16_t level)
{
    if (level < baseline) {
        return (uint16_t)(baseline - 1U);
    }
    if ((level > baseline) && (Rise(baseline, level) < B_BASELINE_BAND)) {
        return (uint16_t)(baseline + 1U);
    }
    return baseline;
}

static void RecordPassSample(uint16_t level_a)
{
    if (pass_samples < WINDOW_MAX) {
        a_pass_start[pass_samples] = level_a;
    }
    if (pass_samples < UINT16_MAX) {
        pass_samples++;
    }
}

/* Called on the sample that releases B: compares the A means of the first
 * and last `window` samples of the pass. */
static int8_t DecideDirection(void)
{
    uint16_t pass_length = (uint16_t)(pass_samples - RELEASE_SAMPLES);
    uint16_t window = pass_length / PASS_FRACTION;
    uint32_t start_sum = 0U;
    uint32_t end_sum = 0U;
    uint32_t margin;
    uint8_t index = history_head;
    uint8_t i;

    if (pass_samples == UINT16_MAX) {
        window = WINDOW_MAX;  /* parked for over an hour; the length is unknown */
    } else if (window > WINDOW_MAX) {
        window = WINDOW_MAX;
    } else if (window == 0U) {
        window = 1U;
    }

    for (i = 0U; i < (uint8_t)window; i++) {
        start_sum += a_pass_start[i];
    }

    /* Skip the samples after the exit, then sum the window before it. */
    for (i = 0U; i < (uint8_t)(RELEASE_SAMPLES + window); i++) {
        index = (index == 0U) ? (uint8_t)(HISTORY_LENGTH - 1U) : (uint8_t)(index - 1U);
        if (i >= RELEASE_SAMPLES) {
            end_sum += a_history[index];
        }
    }

    margin = (uint32_t)DIRECTION_MARGIN * window;

    if ((start_sum > end_sum) && ((start_sum - end_sum) >= margin)) {
        return +1;
    }
    if ((end_sum > start_sum) && ((end_sum - start_sum) >= margin)) {
        return -1;
    }
    return 0;
}

void RotationDecoder_Init(uint16_t initial_baseline_b, uint16_t level_a, uint16_t level_b)
{
    uint8_t i;

    for (i = 0U; i < HISTORY_LENGTH; i++) {
        a_history[i] = level_a;
    }
    history_head = 0U;
    pass_samples = 0U;
    baseline_b = initial_baseline_b;
    in_pass = (Rise(baseline_b, level_b) >= B_RISE_ENTER);
    pass_has_entry = false;
    below_count = 0U;
}

int8_t RotationDecoder_Update(uint16_t level_a, uint16_t level_b)
{
    uint16_t rise_b = Rise(baseline_b, level_b);
    int8_t direction = 0;

    a_history[history_head] = level_a;
    history_head = (uint8_t)((history_head + 1U) % HISTORY_LENGTH);

    if (!in_pass) {
        if (rise_b >= B_RISE_ENTER) {
            in_pass = true;
            below_count = 0U;
            pass_has_entry = true;
            pass_samples = 0U;
            RecordPassSample(level_a);
        } else {
            baseline_b = UpdateBaseline(baseline_b, level_b);
        }
        return 0;
    }

    RecordPassSample(level_a);
    if (rise_b >= B_RISE_HOLD) {
        below_count = 0U;
        return 0;
    }

    below_count++;
    if (below_count >= RELEASE_SAMPLES) {
        in_pass = false;
        below_count = 0U;
        if (pass_has_entry) {
            direction = DecideDirection();
        }
        pass_has_entry = false;
    }
    return direction;
}

bool RotationDecoder_IsInPass(void)
{
    return in_pass;
}

uint16_t RotationDecoder_GetBaselineB(void)
{
    return baseline_b;
}
