// SPDX-FileCopyrightText: 2025-2026 usimd
// SPDX-License-Identifier: Apache-2.0

/**
 * Water meter counter, see water_meter.h.
 */

#include <stddef.h>
#include <xc.h>
#include "mcc_generated_files/system/system.h"
#include "rotation_decoder.h"
#include "water_meter.h"

#define TANK_A  1U
#define TANK_B  2U

#define DRAIN_TIME_US   100U  /* discharge the tank before exciting it */
#define EXCITE_TIME_US  100U  /* L/R is ~1 us, so the coil current is settled */

/* Comparator 1 compares the sense pin with CVREF and its output gates
 * Timer1, which adds up the time the ring-down spends below CVREF. That
 * depends only on the ring-down, not on when the CPU looks at it. The CVR
 * steps (VDD/32 = 156 mV) are where each channel changed most with the disc
 * on the rev2 bench: A 56..70 ticks at 16, B 107..132 at 17. */
#define CVRCON_A              0x90U  /* CVREN, CVRSS = VDD, CVR = 16 */
#define CVRCON_B              0x91U  /* CVR = 17 */
#define CM1CON_A              0x84U  /* CON, CREF = CVREF, CCH = C1INB (RB1), no interrupt */
#define CM1CON_B              0x86U  /* CCH = C2INB (RA5) */
#define T1GCON_COMPARATOR1    0xC2U  /* TMR1GE, T1GPOL high, T1GSS = comparator 1 */
#define T1CON_FOSC4_RD16      0x02U  /* Fosc/4 = 0.25 us per tick, 16-bit access, off */
#define RINGDOWN_WINDOW_TMR3H 8U     /* 8 x 256 x 0.25 us = 512 us */

#define BASELINE_INIT_SAMPLES  7U  /* boot baseline = median, rejects the first-reading spike */

/* S0 pulse length in TMR0 periods (100 ms). S0_Pulse restarts the period,
 * so the pulse lasts exactly this long (EN 62053-31 needs >= 30 ms). One
 * pulse per forward half revolution. A pulse starts only if none was active
 * at the previous sample either, so pulses are >= one sample (~70 ms) apart
 * and at most one starts every ~3 samples (~5 per second); the decoder
 * reports half steps more slowly than that. */
#define S0_PULSE_TICKS  1U
#define S0_PENDING_MAX  UINT8_MAX

static uint16_t ringdown_a;
static uint16_t ringdown_b;
static uint16_t boot_baseline_a;
static uint32_t forward_half_steps;
static uint32_t reverse_half_steps;
static volatile uint8_t s0_ticks_remaining;

/* Reverse half steps not yet made up by forward ones. Forward half steps
 * only queue S0 pulses once it is zero, so the pulses count net forward
 * volume and a wobbling disc does not add pulses. */
static uint32_t reverse_backlog;
static uint8_t s0_pending;
static bool s0_was_active;

static void S0_TimerCallback(void)
{
    if (s0_ticks_remaining > 0U) {
        s0_ticks_remaining--;
        if (s0_ticks_remaining == 0U) {
            S0_LAT = 0;
        }
    }
}

static void S0_Pulse(void)
{
    uint8_t saved_tmr0ie = INTCONbits.TMR0IE;

    INTCONbits.TMR0IE = 0;
    TMR0_PeriodSet(TMR0_PeriodGet());  /* restart a full period now */
    INTCONbits.TMR0IF = 0;
    S0_LAT = 1;
    s0_ticks_remaining = S0_PULSE_TICKS;
    INTCONbits.TMR0IE = saved_tmr0ie;
}

/* Drain, excite, release, and return the time the tank's ring-down spends
 * below CVREF, in 0.25 us ticks. */
static uint16_t MeasureRingDown(uint8_t tank)
{
    uint8_t ticks_low;
    uint8_t saved_pie1 = PIE1;
    uint8_t saved_tmr0ie = INTCONbits.TMR0IE;

    /* Settles during the drain (CVREF 10 us, comparator mode change 10 us). */
    if (tank == TANK_A) {
        CVRCON = CVRCON_A;
        CM1CON = CM1CON_A;
    } else {
        CVRCON = CVRCON_B;
        CM1CON = CM1CON_B;
    }
    T1CON = T1CON_FOSC4_RD16;
    T1GCON = T1GCON_COMPARATOR1;
    T3CON = 0x00U;  /* Fosc/4, 8-bit access so TMR3H reads live */

    if (tank == TANK_A) {
        LCA_TRIS = 0;
        LCA_LAT = 0;
    } else {
        LCB_TRIS = 0;
        LCB_LAT = 0;
    }
    __delay_us(DRAIN_TIME_US);

    if (tank == TANK_A) {
        LCA_LAT = 1;
    } else {
        LCB_LAT = 1;
    }
    __delay_us(EXCITE_TIME_US);

    /* Keep ISRs out of the window so it ends on time. */
    PIE1bits.TX1IE = 0;
    PIE1bits.RC1IE = 0;
    INTCONbits.TMR0IE = 0;

    TMR1H = 0U;
    TMR1L = 0U;
    TMR3H = 0U;
    TMR3L = 0U;
    T1CONbits.TMR1ON = 1;
    T3CONbits.TMR3ON = 1;
    if (tank == TANK_A) {
        LCA_TRIS = 1;
    } else {
        LCB_TRIS = 1;
    }

    while (TMR3H < RINGDOWN_WINDOW_TMR3H) {
    }
    T1CONbits.TMR1ON = 0;
    T3CONbits.TMR3ON = 0;

    /* Drive pins stay low between measurements: an idle tank must not
     * ring along with the other one. */
    if (tank == TANK_A) {
        LCA_TRIS = 0;
        LCA_LAT = 0;
    } else {
        LCB_TRIS = 0;
        LCB_LAT = 0;
    }

    PIE1 = saved_pie1;
    INTCONbits.TMR0IE = saved_tmr0ie;

    /* Off between measurements, also so they draw nothing in Sleep. */
    CM1CON = 0x00U;
    CVRCON = 0x00U;
    T1GCON = 0x00U;
    PIR4bits.CMP1IF = 0;

    ticks_low = TMR1L;  /* latches TMR1H (RD16) */
    return (uint16_t)(((uint16_t)TMR1H << 8U) | ticks_low);
}

static void MeasureBoth(void)
{
    ringdown_b = MeasureRingDown(TANK_B);
    __delay_us(100);
    ringdown_a = MeasureRingDown(TANK_A);
}

/* Median of B over several samples, each taken after wait_for_sample()
 * like in the main loop. */
static uint16_t MedianRingDownB(void (*wait_for_sample)(void))
{
    uint16_t samples[BASELINE_INIT_SAMPLES];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < BASELINE_INIT_SAMPLES; i++) {
        wait_for_sample();
        MeasureBoth();
        samples[i] = ringdown_b;
    }

    for (i = 1U; i < BASELINE_INIT_SAMPLES; i++) {
        uint16_t value = samples[i];

        j = i;
        while ((j > 0U) && (samples[j - 1U] > value)) {
            samples[j] = samples[j - 1U];
            j--;
        }
        samples[j] = value;
    }

    return samples[BASELINE_INIT_SAMPLES / 2U];
}

void WaterMeter_Initialize(void (*wait_for_sample)(void))
{
    uint16_t baseline_b;

    /* ANSEL: 1 = analog. MCC leaves ANCONx at reset defaults. */
    ANCON1bits.ANSEL8 = 1;  /* RB1 = sense A */
    ANCON0bits.ANSEL4 = 1;  /* RA5 = sense B */
    S0_ANS = 0;             /* RB4 = S0 output */

    /* ADC off, input selector on AN0 (RA0, unconnected). The selected pin
     * sources ~2 uA even with the ADC off, which lifts a 165 k sense node
     * by ~0.35 V (two CVR steps); on a sense pin it shifts that comparator
     * channel off its threshold (measured on the rev2 bench). */
    ADCON0 = 0x00U;

    /* The comparators and CVREF configured by MCC stay off;
     * MeasureRingDown enables them for each measurement only. */
    CM1CON = 0x00;
    CM2CON = 0x00;
    CVRCON = 0x00;

    LCA_TRIS = 0;
    LCB_TRIS = 0;
    LCA_LAT = 0;
    LCB_LAT = 0;
    S0_TRIS = 0;
    S0_LAT = 0;

    TMR0_OverflowCallbackRegister(S0_TimerCallback);

    baseline_b = MedianRingDownB(wait_for_sample);
    boot_baseline_a = ringdown_a;
    RotationDecoder_Init(baseline_b, ringdown_a, ringdown_b);
}

static void CountHalfSteps(int8_t step)
{
    uint8_t forward;
    uint8_t reverse;

    if (step > 0) {
        forward = (uint8_t)step;
        forward_half_steps += forward;
        while (forward > 0U) {
            if (reverse_backlog > 0U) {
                reverse_backlog--;
            } else if (s0_pending < S0_PENDING_MAX) {
                s0_pending++;
            }
            forward--;
        }
    } else if (step < 0) {
        reverse = (uint8_t)-step;
        reverse_half_steps += reverse;
        if (reverse_backlog <= (UINT32_MAX - reverse)) {
            reverse_backlog += reverse;
        }
    }
}

int8_t WaterMeter_Process(void)
{
    int8_t step;

    MeasureBoth();
    step = RotationDecoder_Update(ringdown_a, ringdown_b);
    CountHalfSteps(step);

    if ((s0_pending > 0U) && !s0_was_active && !WaterMeter_IsS0PulseActive()) {
        s0_pending--;
        S0_Pulse();
    }
    s0_was_active = WaterMeter_IsS0PulseActive();

    return step;
}

uint32_t WaterMeter_GetForwardHalfSteps(void)
{
    return forward_half_steps;
}

uint32_t WaterMeter_GetReverseHalfSteps(void)
{
    return reverse_half_steps;
}

void WaterMeter_GetStatus(water_meter_status_t *status)
{
    status->ringdown_a = ringdown_a;
    status->ringdown_b = ringdown_b;
    status->baseline_a = boot_baseline_a;
    status->baseline_b = RotationDecoder_GetBaselineB();
    status->in_pass_b = RotationDecoder_IsInPass();
}

bool WaterMeter_IsS0PulseActive(void)
{
    return s0_ticks_remaining != 0U;
}
