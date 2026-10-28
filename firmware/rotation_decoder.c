// SPDX-FileCopyrightText: 2025-2026 usimd
// SPDX-License-Identifier: Apache-2.0

/**
 * Rotation decoder, see rotation_decoder.h.
 *
 * A B pass is a rise above B's tracked rest level. A is not thresholded: its
 * level varies between mounts. Instead the mean A level over the first third
 * of each B pass, and of each rest phase between passes, is compared with
 * the mean over its last third.
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

/* The baseline falls by one tick per sample but rises only by one tick per
 * this many samples (~1.4 s). Rising as fast as it falls let it follow B's
 * ramp into a pass at slow flow (B rising <= 1 tick per sample, ~3-4
 * L/min on the bench), so the pass was never entered. Replays slowed down
 * 16x still count every pass; drift of the rest level is far slower. */
#define B_BASELINE_RISE_SAMPLES 16U

#define RELEASE_SAMPLES       2U   /* below-hold samples before the pass ends */

/* A is averaged over the first and the last 1/PASS_FRACTION of a segment,
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

/* Where the disc is within the rest phase (the half revolution between the
 * end of one forward B pass and the start of the next one). LOW is the end
 * where a forward pass exits, HIGH the end where the next forward pass
 * enters. */
#define REST_UNKNOWN          0U
#define REST_LOW              1U
#define REST_HIGH             2U

static uint16_t a_history[HISTORY_LENGTH];  /* ring of the newest A levels */
static uint8_t history_head;
static uint16_t a_segment_start[WINDOW_MAX]; /* first A levels of the current pass or rest */
static uint16_t segment_samples;             /* samples in the current pass or rest, saturating */

static uint16_t baseline_b;
static uint8_t baseline_rise_count;  /* samples that wanted a rise since the last step */
static bool in_pass;
static bool pass_has_entry;
static uint8_t below_count;

static uint8_t rest_start;     /* end at which the current rest phase began */
static uint8_t rest_position;  /* best estimate of the current end */
static int8_t rest_credit;     /* half step already reported for this rest phase */

static uint16_t Rise(uint16_t baseline, uint16_t level)
{
    return (level > baseline) ? (uint16_t)(level - baseline) : 0U;
}

/* Rest level tracker, called once per sample outside a pass: down one tick
 * per sample, up one tick per B_BASELINE_RISE_SAMPLES samples. Samples
 * already well above the baseline are ignored so a disc edge parked near B
 * does not drag it toward the pass level. Settles at the lower edge of the
 * rest noise. */
static void UpdateBaseline(uint16_t level)
{
    if (level < baseline_b) {
        baseline_b--;
        baseline_rise_count = 0U;
    } else if ((level > baseline_b) && (Rise(baseline_b, level) < B_BASELINE_BAND)) {
        baseline_rise_count++;
        if (baseline_rise_count >= B_BASELINE_RISE_SAMPLES) {
            baseline_b++;
            baseline_rise_count = 0U;
        }
    }
}

static void RecordSegmentSample(uint16_t level_a)
{
    if (segment_samples < WINDOW_MAX) {
        a_segment_start[segment_samples] = level_a;
    }
    if (segment_samples < UINT16_MAX) {
        segment_samples++;
    }
}

/* Compares the A mean of the first `window` samples of a segment of
 * `length` samples with the mean of its last `window` samples, which end
 * `skip` samples before the newest one in the history.
 * Returns +1 if A fell, -1 if it rose, 0 if it changed less than the margin. */
static int8_t CompareSegmentEdges(uint16_t length, uint8_t skip)
{
    uint16_t window = length / PASS_FRACTION;
    uint32_t start_sum = 0U;
    uint32_t end_sum = 0U;
    uint32_t margin;
    uint8_t index = history_head;
    uint8_t i;

    if (segment_samples == UINT16_MAX) {
        window = WINDOW_MAX;  /* over an hour; the length is unknown */
    } else if (window > WINDOW_MAX) {
        window = WINDOW_MAX;
    } else if (window == 0U) {
        window = 1U;
    }

    for (i = 0U; i < (uint8_t)window; i++) {
        start_sum += a_segment_start[i];
    }

    for (i = 0U; i < (uint8_t)(skip + window); i++) {
        index = (index == 0U) ? (uint8_t)(HISTORY_LENGTH - 1U) : (uint8_t)(index - 1U);
        if (i >= skip) {
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

/* Half step of a rest phase that started at `start` and is now at `end`. */
static int8_t RestCredit(uint8_t start, uint8_t end)
{
    if ((start == REST_LOW) && (end == REST_HIGH)) {
        return +1;
    }
    if ((start == REST_HIGH) && (end == REST_LOW)) {
        return -1;
    }
    return 0;
}

/* Called on the sample that enters a B pass, before it is recorded. A falls
 * during a forward pass and recovers during the following rest phase, so A
 * rising over the rest means the disc moved forward through it. A rest phase
 * can only be crossed once before the next pass, so a move is accepted only
 * away from the end the disc was estimated at. */
static int8_t DecideRest(void)
{
    int8_t rest_direction;
    int8_t credit;
    int8_t step;

    if (rest_start == REST_UNKNOWN) {
        return 0;
    }

    /* The entry sample is already in the history but not part of the rest. */
    rest_direction = (int8_t)-CompareSegmentEdges(segment_samples, 1U);
    if ((rest_position == REST_LOW) && (rest_direction > 0)) {
        rest_position = REST_HIGH;
    } else if ((rest_position == REST_HIGH) && (rest_direction < 0)) {
        rest_position = REST_LOW;
    }

    credit = RestCredit(rest_start, rest_position);
    step = (int8_t)(credit - rest_credit);
    rest_credit = credit;
    return step;
}

/* Called on the sample that releases B. A decided pass also tells at which
 * end the preceding rest phase ended: a forward pass entered from its HIGH
 * end, a reverse pass from its LOW end. That corrects the tentative rest
 * half step. An undecided pass (the disc turned back inside it) keeps the
 * rest phase open. */
static int8_t DecidePass(void)
{
    int8_t pass_direction;
    int8_t step = 0;

    if (!pass_has_entry) {
        rest_start = REST_UNKNOWN;
        rest_position = REST_UNKNOWN;
        rest_credit = 0;
        return 0;
    }

    pass_direction = CompareSegmentEdges((uint16_t)(segment_samples - RELEASE_SAMPLES),
                                         (uint8_t)RELEASE_SAMPLES);
    if (pass_direction == 0) {
        return 0;
    }

    if (rest_start != REST_UNKNOWN) {
        uint8_t rest_end = (pass_direction > 0) ? REST_HIGH : REST_LOW;

        step = (int8_t)(RestCredit(rest_start, rest_end) - rest_credit);
    }
    step = (int8_t)(step + pass_direction);

    rest_start = (pass_direction > 0) ? REST_LOW : REST_HIGH;
    rest_position = rest_start;
    rest_credit = 0;
    return step;
}

/* Starts the rest segment at the first sample below the hold level, which
 * is RELEASE_SAMPLES - 1 samples before the newest one. */
static void StartRestSegment(void)
{
    uint8_t index = history_head;
    uint8_t i;

    for (i = 0U; i < (uint8_t)RELEASE_SAMPLES; i++) {
        index = (index == 0U) ? (uint8_t)(HISTORY_LENGTH - 1U) : (uint8_t)(index - 1U);
    }
    segment_samples = 0U;
    for (i = 0U; i < (uint8_t)RELEASE_SAMPLES; i++) {
        RecordSegmentSample(a_history[index]);
        index = (uint8_t)((index + 1U) % HISTORY_LENGTH);
    }
}

void RotationDecoder_Init(uint16_t initial_baseline_b, uint16_t level_a, uint16_t level_b)
{
    uint8_t i;

    for (i = 0U; i < HISTORY_LENGTH; i++) {
        a_history[i] = level_a;
    }
    history_head = 0U;
    segment_samples = 0U;
    baseline_b = initial_baseline_b;
    baseline_rise_count = 0U;
    in_pass = (Rise(baseline_b, level_b) >= B_RISE_ENTER);
    pass_has_entry = false;
    below_count = 0U;
    rest_start = REST_UNKNOWN;
    rest_position = REST_UNKNOWN;
    rest_credit = 0;
}

int8_t RotationDecoder_Update(uint16_t level_a, uint16_t level_b)
{
    uint16_t rise_b = Rise(baseline_b, level_b);
    int8_t step = 0;

    a_history[history_head] = level_a;
    history_head = (uint8_t)((history_head + 1U) % HISTORY_LENGTH);

    if (!in_pass) {
        if (rise_b >= B_RISE_ENTER) {
            step = DecideRest();
            in_pass = true;
            below_count = 0U;
            pass_has_entry = true;
            segment_samples = 0U;
        } else {
            UpdateBaseline(level_b);
        }
        RecordSegmentSample(level_a);
        return step;
    }

    RecordSegmentSample(level_a);
    if (rise_b >= B_RISE_HOLD) {
        below_count = 0U;
        return 0;
    }

    below_count++;
    if (below_count >= RELEASE_SAMPLES) {
        in_pass = false;
        below_count = 0U;
        step = DecidePass();
        pass_has_entry = false;
        StartRestSegment();
    }
    return step;
}

bool RotationDecoder_IsInPass(void)
{
    return in_pass;
}

uint16_t RotationDecoder_GetBaselineB(void)
{
    return baseline_b;
}
