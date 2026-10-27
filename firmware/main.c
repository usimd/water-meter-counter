// SPDX-FileCopyrightText: 2025-2026 usimd
// SPDX-License-Identifier: Apache-2.0

/**
 * Water meter counter firmware (PIC18F25K80, rev2 board).
 *
 * Counts rotations of the water meter's half-disc with two LC-tank sensors,
 * emits an S0 pulse on RB4 per forward rotation and logs to UART1
 * (RC6/RC7, 9600 8N1).
 */

#include <stdio.h>
#include "mcc_generated_files/system/system.h"
#include "water_meter.h"

/* The rotation decoder was tuned at 70..100 ms per sample; one measurement
 * takes ~1.5 ms. Between samples the CPU sleeps until the watchdog wakes it
 * (4 ms x WDTPS 1:16 = 64 ms, config_bits.c). While an S0 pulse runs it
 * busy-waits instead, so TMR0 (stopped in Sleep) can end the pulse on time.
 * Both channels are timed in hardware by the comparator, so restarting the
 * HF-INTOSC after Sleep does not affect the readings. */
#define SAMPLE_DELAY_MS  66

/* 1 = print "S <A ticks> <B ticks> <boot A> <baseline B> <P|->" every
 * sample, e.g. to record captures for re-tuning the rotation decoder.
 * Set by the build: default.hex is built with 0, default-debug.hex with 1
 * (cmake/water-meter-counter/default/user.cmake). */
#ifndef DEBUG_STREAM
#define DEBUG_STREAM  0
#endif

/* 0 = busy-wait between samples (CPU awake), 1 = sleep until the watchdog. */
#define LOW_POWER_WAIT  1

static void WaitForNextSample(void)
{
#if LOW_POWER_WAIT
    uint8_t saved_pie1;
    uint8_t saved_tmr0ie;

    if (WaterMeter_IsS0PulseActive()) {
        __delay_ms(SAMPLE_DELAY_MS);
        return;
    }

    /* Only the watchdog may end the sleep: let the UART TX drain and mask
     * the other interrupts that would wake the CPU early. */
    while (PIE1bits.TX1IE || !EUSART1_IsTxDone()) {
    }
    saved_pie1 = PIE1;
    saved_tmr0ie = INTCONbits.TMR0IE;
    PIE1bits.RC1IE = 0;
    INTCONbits.TMR0IE = 0;

    /* Needs WDTEN = ON (SWDTEN control) and WDTPS 1:16 in config_bits.c. */
    WDTCONbits.SWDTEN = 1;
    SLEEP();
    NOP();
    WDTCONbits.SWDTEN = 0;

    PIE1 = saved_pie1;
    INTCONbits.TMR0IE = saved_tmr0ie;
#else
    __delay_ms(SAMPLE_DELAY_MS);
#endif
}

int main(void)
{
    SYSTEM_Initialize();
    INTERRUPT_GlobalInterruptHighEnable();
    INTERRUPT_GlobalInterruptLowEnable();
    INTERRUPT_PeripheralInterruptEnable();

    /* SLEEP() enters Sleep (IDLEN = 0, the reset default). The regulator
     * goes to low-power mode in Sleep; ULP regulator mode stays off
     * (RETEN = OFF, config_bits.c). */
    WDTCONbits.REGSLP = 1;

    printf("\r\nWater meter counter\r\n");
    WaterMeter_Initialize(WaitForNextSample);

    {
        water_meter_status_t status;

        WaterMeter_GetStatus(&status);
        printf("Baseline A=%u B=%u\r\n", status.baseline_a, status.baseline_b);
    }

    while (1) {
        int8_t direction = WaterMeter_Process();

        if (direction != 0) {
            printf("%s fwd=%lu rev=%lu\r\n",
                   (direction > 0) ? "FWD" : "REV",
                   WaterMeter_GetForwardCount(),
                   WaterMeter_GetReverseCount());
        }

#if DEBUG_STREAM
        {
            water_meter_status_t status;

            WaterMeter_GetStatus(&status);
            printf("S %u %u %u %u %c\r\n",
                   status.ringdown_a, status.ringdown_b, status.baseline_a, status.baseline_b,
                   status.in_pass_b ? 'P' : '-');
        }
#endif

        WaitForNextSample();
    }
}
