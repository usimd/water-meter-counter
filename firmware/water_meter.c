/**
 * Water Meter Counter Module Implementation
 * 
 * Dual LC tank metal detection for water meter disc rotation sensing.
 * 
 * Hardware:
 *   LCA: Drive=RC5, Sense=RB1 (CMP1)
 *   LCB: Drive=RC0, Sense=RA5 (CMP2)
 *   S0:  RB2 (pulse output)
 * 
 * Detection principle:
 *   - Metal near an inductor damps the ring-down response.
 *   - Raw ADC span on the sense node is the primary metal-state signal.
 *   - Comparator edge counts remain as diagnostics because bench data showed
 *     they move less reliably than the raw span.
 */

#include "water_meter.h"
#include "mcc_generated_files/system/system.h"
#include <xc.h>

/* Oscillation counters - updated by comparator ISRs
 * Note: With CCH=10 (cross-mapping per datasheet Figure 24-4 Note 2):
 *   CMP1 senses C2INB (RA5) = LCB sense
 *   CMP2 senses C1INB (RB1) = LCA sense
 */
static volatile uint16_t cmp1_count = 0;  /* LCB edges */
static volatile uint16_t cmp2_count = 0;  /* LCA edges */

/* Last measured oscillation counts */
static uint8_t lca_count = 0;
static uint8_t lcb_count = 0;
static uint8_t lca_polled_count = 0;
static uint16_t lca_raw_min = 0;
static uint16_t lca_raw_max = 0;
static uint16_t lca_raw_span = 0;
static uint16_t lcb_raw_span = 0;
static uint16_t lca_span_baseline = 0;
static uint16_t lcb_span_baseline = 0;
static int8_t last_rotation_direction = 0;

/* State tracking for Gray code quadrature decoding
 * State = (LCA_metal << 1) | LCB_metal
 *   0 = 00 = both air
 *   1 = 01 = LCB metal only
 *   2 = 10 = LCA metal only
 *   3 = 11 = both metal
 *
 * Mounted logs showed the real installed pass order is:
 *   A -> AB -> B -> air
 * so forward must be interpreted as 0→2→3→1→0 on this build.
 */
static uint8_t current_state = 0;

/* Rotation counters - separate forward and reverse */
static uint32_t forward_count = 0;
static uint32_t reverse_count = 0;
static uint8_t sequence_direction = 0;
static uint8_t sequence_progress = 0;
static uint8_t air_rearm_count = 0;
static bool sequence_armed = false;
static uint8_t lca_below_threshold_samples = 0;
static uint8_t lcb_below_threshold_samples = 0;

static const uint8_t forward_sequence[5] = {0U, 2U, 3U, 1U, 0U};
static const uint8_t reverse_sequence[5] = {0U, 1U, 3U, 2U, 0U};

/* Timing constants for GPIO HiZ measurement */
#define DRAIN_TIME_US     100   /* Discharge tank before measurement */
#define EXCITE_TIME_US    100   /* Pulse duration to kick tank */
#define MEASURE_TIME_US   500   /* Window to count oscillations */

/* Comparator CVREF still matters for diagnostics, but not for primary state detection. */
#define CVR_OPTIMAL    16U  /* CVref setting for 2.5V threshold */

/* Span-based state detection thresholds.
 * Bench tuning was too conservative once the sensor was mounted.
 * The mounted logs now show tracked baselines around:
 *   A baseline: ~25-28
 *   B baseline: ~29-34
 * with repeated drop excursions of roughly:
 *   A drop: ~7-9
 *   B drop: ~8-13
 *
 * Use drop from the slowly tracked air baseline plus a dominance margin so
 * cross-coupling does not turn a strong A-only or B-only event into a false
 * "both metal" state when entering from air. Once a pass is already in a
 * single-channel state, simultaneous activation should promote to AB so the
 * Gray sequence can complete. Mounted operation also benefits from an
 * asymmetric air-baseline tracker so transient low spans do not desensitize
 * the detector by dragging the reference down.
 */
#define LCA_DROP_ENTER             7U
#define LCB_DROP_ENTER             8U
#define LCA_DROP_JOIN              4U
#define LCB_DROP_JOIN              5U
#define LCA_DROP_HOLD              4U
#define LCB_DROP_HOLD              5U
#define LCA_RELEASE_SAMPLES        2U
#define LCB_RELEASE_SAMPLES        2U
#define DROP_DOMINANCE_ENTER       3U
#define DROP_DOMINANCE_HOLD        2U
#define SPAN_BASELINE_RISE_SHIFT   2U  /* 1/4 update when quiet-air baseline rises */
#define SPAN_BASELINE_TRIM_GUARD   2U  /* Only trim baseline down for tiny steady shifts */
#define ROTATION_REARM_SAMPLES     2U  /* Require 100 ms of stable air before re-arming */

/* ADC-based raw-node capture on the sense pins.
 * This is slower than the actual oscillation frequency, so it does not resolve
 * every cycle. It is still useful because it measures the analog node directly
 * instead of depending on comparator threshold crossings. */
#define ADC_CHANNEL_LCA   8U   /* RB1 = AN8 */
#define ADC_CHANNEL_LCB   4U   /* RA5 = AN4 */
#define ADC_RAW_SAMPLES   16U
/*
 * Raw-node capture reads a very high-impedance, AC-coupled sense node.
 * Use a long acquisition window so the ADC sample capacitor can settle.
 */
#define ADCON2_RAW_CAPTURE 0xBA  /* ADFM=1, ACQT=20TAD, ADCS=Fosc/32 */

/* Periodic CVREF sweep on both channels.
 * Each point stores: ISR count / polled count / raw ADC span. */
#define DIAGNOSTIC_SWEEP_POINT_COUNT 9U
#define DIAGNOSTIC_SWEEP_INTERVAL    20U
static const uint8_t diagnostic_cvr_points[DIAGNOSTIC_SWEEP_POINT_COUNT] = {12U, 13U, 14U, 15U, 16U, 17U, 18U, 19U, 20U};
static uint8_t diagnostic_lca_counts[DIAGNOSTIC_SWEEP_POINT_COUNT] = {0};
static uint8_t diagnostic_lcb_counts[DIAGNOSTIC_SWEEP_POINT_COUNT] = {0};
static uint8_t diagnostic_apl_counts[DIAGNOSTIC_SWEEP_POINT_COUNT] = {0};
static uint8_t diagnostic_bpl_counts[DIAGNOSTIC_SWEEP_POINT_COUNT] = {0};
static uint16_t diagnostic_raw_spans_a[DIAGNOSTIC_SWEEP_POINT_COUNT] = {0};
static uint16_t diagnostic_raw_spans_b[DIAGNOSTIC_SWEEP_POINT_COUNT] = {0};
static bool diagnostic_sweep_ready = false;
static uint8_t diagnostic_loop_divider = 0;
static uint8_t current_cvref = CVR_OPTIMAL;

/* S0 pulse configuration - uses TMR0 (100ms tick).
 * Keep this longer during mounted bring-up so a meter or DMM can catch it. */
static volatile uint8_t s0_ticks_remaining = 0;
#define S0_PULSE_DURATION_TICKS  20  /* 2.0 s pulse (20 x 100ms TMR0 ticks) */

/* TMR0 overflow callback for S0 pulse timing */
static void S0_TimerCallback(void)
{
    if (s0_ticks_remaining > 0) {
        s0_ticks_remaining--;
        if (s0_ticks_remaining == 0) {
            LATBbits.LATB2 = 0;  /* Turn off S0 pulse */
        }
    }
}

/* Start S0 pulse - call when rotation detected */
static void S0_Pulse(void)
{
    LATBbits.LATB2 = 1;  /* Turn on S0 immediately */
    s0_ticks_remaining = S0_PULSE_DURATION_TICKS;  /* TMR0 ISR will turn it off */
}

/* Comparator ISR callbacks */
static void CMP1_Callback(void)
{
    cmp1_count++;
}

static void CMP2_Callback(void)
{
    cmp2_count++;
}

static uint8_t MeasureTankPolled(uint8_t tank);
static uint8_t MeasureTank(uint8_t tank);

static void SetCVRef(uint8_t cvr)
{
    current_cvref = (uint8_t)(cvr & 0x1FU);
    CVRCON = (uint8_t)(0x80U | current_cvref);
    __delay_us(50);
}

static uint16_t SpanDrop(uint16_t baseline, uint16_t span)
{
    return (baseline > span) ? (uint16_t)(baseline - span) : 0U;
}

static uint16_t UpdateBaseline(uint16_t baseline, uint16_t sample)
{
    if (sample >= baseline) {
        return (uint16_t)(((uint32_t)baseline * ((1UL << SPAN_BASELINE_RISE_SHIFT) - 1UL) + (uint32_t)sample + (1UL << (SPAN_BASELINE_RISE_SHIFT - 1U))) >> SPAN_BASELINE_RISE_SHIFT);
    }

    if ((uint16_t)(baseline - sample) <= SPAN_BASELINE_TRIM_GUARD) {
        return (uint16_t)(baseline - 1U);
    }

    return baseline;
}

static bool ApplyTemporalHold(bool instantaneous_active, bool previous_active, uint8_t *below_counter, uint8_t release_samples)
{
    if (instantaneous_active) {
        *below_counter = 0U;
        return true;
    }

    if (!previous_active) {
        *below_counter = 0U;
        return false;
    }

    *below_counter = (uint8_t)(*below_counter + 1U);
    if (*below_counter < release_samples) {
        return true;
    }

    *below_counter = 0U;
    return false;
}

static int8_t ProcessQuadratureSequence(uint8_t previous_state, uint8_t new_state)
{
    const uint8_t *sequence = NULL;

    if (new_state == previous_state) {
        return 0;
    }

    if (sequence_direction == 0U) {
        if (!sequence_armed) {
            return 0;
        }

        if ((previous_state == 0U) && (new_state == 2U)) {
            sequence_direction = 1U;
            sequence_progress = 1U;
        } else if ((previous_state == 0U) && (new_state == 1U)) {
            sequence_direction = 2U;
            sequence_progress = 1U;
        }
        return 0;
    }

    sequence = (sequence_direction == 1U) ? forward_sequence : reverse_sequence;
    if ((sequence_progress < 4U) && (new_state == sequence[sequence_progress + 1U])) {
        sequence_progress++;
        if (sequence_progress == 4U) {
            int8_t direction = (sequence_direction == 1U) ? +1 : -1;

            sequence_direction = 0U;
            sequence_progress = 0U;
            return direction;
        }
        return 0;
    }

    /* Mounted traces often bounce one Gray step backward before resuming.
     * Tolerate that adjacent backtracking instead of discarding the whole pass. */
    if ((sequence_progress > 1U) && (new_state == sequence[sequence_progress - 1U])) {
        sequence_progress--;
        return 0;
    }

    if (new_state == 0U) {
        sequence_direction = 0U;
        sequence_progress = 0U;
        return 0;
    }

    /* Any out-of-order transition means the current pass is ambiguous. */
    sequence_direction = 0U;
    sequence_progress = 0U;
    return 0;
}

static uint8_t DetermineMetalStateFromSpan(uint16_t lca_span, uint16_t lcb_span, uint8_t previous_state)
{
    bool previous_lca_metal = ((previous_state & 2U) != 0U);
    bool previous_lcb_metal = ((previous_state & 1U) != 0U);
    bool pass_in_progress = (previous_state != 0U);
    uint16_t lca_drop = SpanDrop(lca_span_baseline, lca_span);
    uint16_t lcb_drop = SpanDrop(lcb_span_baseline, lcb_span);
    uint16_t lca_threshold = previous_lca_metal ? LCA_DROP_HOLD : (pass_in_progress ? LCA_DROP_JOIN : LCA_DROP_ENTER);
    uint16_t lcb_threshold = previous_lcb_metal ? LCB_DROP_HOLD : (pass_in_progress ? LCB_DROP_JOIN : LCB_DROP_ENTER);
    uint16_t dominance_threshold = ((previous_state == 0U) ? DROP_DOMINANCE_ENTER : DROP_DOMINANCE_HOLD);
    bool lca_active = ApplyTemporalHold((lca_drop >= lca_threshold), previous_lca_metal, &lca_below_threshold_samples, LCA_RELEASE_SAMPLES);
    bool lcb_active = ApplyTemporalHold((lcb_drop >= lcb_threshold), previous_lcb_metal, &lcb_below_threshold_samples, LCB_RELEASE_SAMPLES);

    if (lca_active && lcb_active) {
        if ((previous_state == 1U) || (previous_state == 2U) || (previous_state == 3U)) {
            return 3U;
        }

        if (lca_drop >= (uint16_t)(lcb_drop + dominance_threshold)) {
            return 2U;
        }
        if (lcb_drop >= (uint16_t)(lca_drop + dominance_threshold)) {
            return 1U;
        }
        return 3U;
    }

    if (lca_active) {
        return 2U;
    }
    if (lcb_active) {
        return 1U;
    }
    return 0U;
}

static void ADC_InitializeRawCapture(void)
{
    ADCON1 = 0x00;               /* Vref = VDD/VSS, manual trigger */
    ADCON2 = ADCON2_RAW_CAPTURE; /* Right-justified, long auto acquisition */
    ADCON0 = 0x01;               /* ADON=1, channel will be selected per read */
}

static uint16_t ADC_ReadChannel(uint8_t channel)
{
    ADCON0 = (uint8_t)(((uint8_t)(channel << 2U)) | 0x01U);
    ADCON0bits.GO_nDONE = 1;
    while (ADCON0bits.GO_nDONE) {
    }
    /* PIC18F25K80 ADC is 10-bit; upper bits are not part of the conversion. */
    return ((uint16_t)ADRES) & 0x03FFU;
}

static uint16_t MeasureTankRawADC(uint8_t tank, uint16_t *sample_min_out, uint16_t *sample_max_out)
{
    uint8_t sample_index;
    uint8_t adc_channel = (tank == 1U) ? ADC_CHANNEL_LCA : ADC_CHANNEL_LCB;
    uint16_t sample_min = 0x03FFU;
    uint16_t sample_max = 0U;
    uint8_t saved_pie1 = PIE1;
    uint8_t saved_pie4 = PIE4;
    uint8_t saved_tmr0ie = INTCONbits.TMR0IE;
    uint8_t saved_cm1con = CM1CON;
    uint8_t saved_cm2con = CM2CON;

    /* PHASE 1: DRAIN - discharge tank */
    if (tank == 1U) {
        TRISCbits.TRISC5 = 0;
        LATCbits.LATC5 = 0;
    } else {
        TRISCbits.TRISC0 = 0;
        LATCbits.LATC0 = 0;
    }
    __delay_us(DRAIN_TIME_US);

    /* PHASE 2: EXCITE - pulse HIGH */
    if (tank == 1U) {
        LATCbits.LATC5 = 1;
    } else {
        LATCbits.LATC0 = 1;
    }
    __delay_us(EXCITE_TIME_US);

    /* Quiet unrelated ISRs while sampling the analog node directly. */
    PIE1bits.TX1IE = 0;
    PIE1bits.RC1IE = 0;
    PIE4bits.CMP1IE = 0;
    PIE4bits.CMP2IE = 0;
    INTCONbits.TMR0IE = 0;

    /* Raw capture should sample the pin, not a comparator-loaded mux state. */
    CM1CONbits.CON = 0;
    CM2CONbits.CON = 0;
    __delay_us(5);

    /* PHASE 3: HIZ + SAMPLE - release tank and sample raw node with ADC. */
    if (tank == 1U) {
        PIR4bits.CMP2IF = 0;
        TRISCbits.TRISC5 = 1;
    } else {
        PIR4bits.CMP1IF = 0;
        TRISCbits.TRISC0 = 1;
    }

    /* Throw away the first sample after release so the sample capacitor settles. */
    (void)ADC_ReadChannel(adc_channel);

    for (sample_index = 0U; sample_index < ADC_RAW_SAMPLES; sample_index++) {
        uint16_t sample = ADC_ReadChannel(adc_channel);

        if (sample < sample_min) {
            sample_min = sample;
        }
        if (sample > sample_max) {
            sample_max = sample;
        }
    }

    /* PHASE 4: DRAIN - return to LOW */
    if (tank == 1U) {
        TRISCbits.TRISC5 = 0;
        LATCbits.LATC5 = 0;
    } else {
        TRISCbits.TRISC0 = 0;
        LATCbits.LATC0 = 0;
    }

    CM1CON = saved_cm1con;
    CM2CON = saved_cm2con;
    PIR4bits.CMP1IF = 0;
    PIR4bits.CMP2IF = 0;

    PIE1 = saved_pie1;
    PIE4 = saved_pie4;
    INTCONbits.TMR0IE = saved_tmr0ie;

    if (sample_min_out != NULL) {
        *sample_min_out = sample_min;
    }
    if (sample_max_out != NULL) {
        *sample_max_out = sample_max;
    }

    return (uint16_t)(sample_max - sample_min);
}

static void CaptureDiagnosticSweep(void)
{
    uint8_t point_index;
    uint8_t saved_cvref = current_cvref;

    for (point_index = 0U; point_index < DIAGNOSTIC_SWEEP_POINT_COUNT; point_index++) {
        SetCVRef(diagnostic_cvr_points[point_index]);
        diagnostic_lca_counts[point_index] = MeasureTank(1);
        diagnostic_apl_counts[point_index] = MeasureTankPolled(1);
        diagnostic_raw_spans_a[point_index] = MeasureTankRawADC(1, NULL, NULL);
        __delay_us(100);
        diagnostic_lcb_counts[point_index] = MeasureTank(2);
        diagnostic_bpl_counts[point_index] = MeasureTankPolled(2);
        diagnostic_raw_spans_b[point_index] = MeasureTankRawADC(2, NULL, NULL);
    }

    SetCVRef(saved_cvref);
}

static void RunDiagnosticSweep(void)
{
    CaptureDiagnosticSweep();
    diagnostic_sweep_ready = true;
}

static uint8_t MeasureTankPolled(uint8_t tank)
{
    uint8_t transitions = 0;
    uint8_t last_tick;
    uint8_t tick_delta;
    uint16_t remaining_ticks = (MEASURE_TIME_US / 2U);
    bool last_state;
    uint8_t saved_pie1 = PIE1;
    uint8_t saved_pie4 = PIE4;
    uint8_t saved_tmr0ie = INTCONbits.TMR0IE;

    /* PHASE 1: DRAIN - discharge tank */
    if (tank == 1) {
        TRISCbits.TRISC5 = 0;
        LATCbits.LATC5 = 0;
    } else {
        TRISCbits.TRISC0 = 0;
        LATCbits.LATC0 = 0;
    }
    __delay_us(DRAIN_TIME_US);

    /* PHASE 2: EXCITE - pulse HIGH */
    if (tank == 1) {
        LATCbits.LATC5 = 1;
    } else {
        LATCbits.LATC0 = 1;
    }
    __delay_us(EXCITE_TIME_US);

    /* TMR0 runs from Fosc/4 with 1:8 prescale, so TMR0L advances every ~2 us. */
    /* Disable unrelated ISR noise while sampling comparator output on each TMR0 tick. */
    PIE1bits.TX1IE = 0;
    PIE1bits.RC1IE = 0;
    PIE4bits.CMP1IE = 0;
    PIE4bits.CMP2IE = 0;
    INTCONbits.TMR0IE = 0;

    /* PHASE 3: HIZ + POLL - release and sample comparator output every 2 us TMR0 tick */
    if (tank == 1) {
        PIR4bits.CMP2IF = 0;
        TRISCbits.TRISC5 = 1;
        last_state = CMSTATbits.CMP2OUT;
    } else {
        PIR4bits.CMP1IF = 0;
        TRISCbits.TRISC0 = 1;
        last_state = CMSTATbits.CMP1OUT;
    }

    last_tick = TMR0L;

    while (remaining_ticks > 0U) {
        uint8_t tick = TMR0L;

        tick_delta = (uint8_t)(tick - last_tick);
        if (tick_delta == 0U) {
            continue;
        }

        last_tick = tick;
        if (tick_delta > remaining_ticks) {
            tick_delta = (uint8_t)remaining_ticks;
        }
        remaining_ticks -= tick_delta;

        bool state = (tank == 1) ? CMSTATbits.CMP2OUT : CMSTATbits.CMP1OUT;
        if (state != last_state) {
            if (transitions != UINT8_MAX) {
                transitions++;
            }
            last_state = state;
        }
    }

    /* PHASE 4: DRAIN - return to LOW */
    if (tank == 1) {
        TRISCbits.TRISC5 = 0;
        LATCbits.LATC5 = 0;
    } else {
        TRISCbits.TRISC0 = 0;
        LATCbits.LATC0 = 0;
    }

    /* Drop any comparator flags generated during the polled window before re-enabling IRQs. */
    PIR4bits.CMP1IF = 0;
    PIR4bits.CMP2IF = 0;

    PIE1 = saved_pie1;
    PIE4 = saved_pie4;
    INTCONbits.TMR0IE = saved_tmr0ie;

    return transitions;
}

/**
 * @brief Measure LC tank using GPIO HiZ mode (TI TIDA-010050 style)
 * 
 * Sequence:
 *   1. DRAIN: GPIO LOW (output) - discharge tank
 *   2. EXCITE: GPIO HIGH (output) - pulse to kick tank
 *   3. HIZ: GPIO INPUT - let tank ring freely with no loading
 *   4. COUNT: Comparator interrupts count edge crossings
 *   5. DRAIN: GPIO LOW - ready for next cycle
 * 
 * @param tank 1=LCA (RC5/CMP2), 2=LCB (RC0/CMP1)
 * @return Edge count during measurement window
 */
static uint8_t MeasureTank(uint8_t tank)
{
    volatile uint16_t *edge_counter;
    
    /* Disable UART interrupts during measurement */
    uint8_t saved_pie1 = PIE1;
    PIE1bits.TX1IE = 0;
    PIE1bits.RC1IE = 0;
    
    /* Select comparator based on CCH=10 cross-mapping:
     * CMP1 → RA5 (LCB sense), CMP2 → RB1 (LCA sense) */
    if (tank == 1) {
        edge_counter = &cmp2_count;  /* LCA uses CMP2 */
    } else {
        edge_counter = &cmp1_count;  /* LCB uses CMP1 */
    }
    
    /* PHASE 1: DRAIN - discharge tank */
    if (tank == 1) {
        TRISCbits.TRISC5 = 0;  /* Output */
        LATCbits.LATC5 = 0;    /* LOW */
    } else {
        TRISCbits.TRISC0 = 0;
        LATCbits.LATC0 = 0;
    }
    __delay_us(DRAIN_TIME_US);
    
    /* PHASE 2: EXCITE - pulse HIGH */
    if (tank == 1) {
        LATCbits.LATC5 = 1;
    } else {
        LATCbits.LATC0 = 1;
    }
    __delay_us(EXCITE_TIME_US);
    
    /* PHASE 3: HIZ + COUNT - go directly to input mode */
    *edge_counter = 0;
    if (tank == 1) {
        PIR4bits.CMP2IF = 0;
        TRISCbits.TRISC5 = 1;  /* INPUT = High-Z */
    } else {
        PIR4bits.CMP1IF = 0;
        TRISCbits.TRISC0 = 1;
    }
    
    /* Wait for oscillations */
    __delay_us(MEASURE_TIME_US);
    
    uint8_t edges = (uint8_t)(*edge_counter);
    
    /* PHASE 4: DRAIN - return to LOW */
    if (tank == 1) {
        TRISCbits.TRISC5 = 0;
        LATCbits.LATC5 = 0;
    } else {
        TRISCbits.TRISC0 = 0;
        LATCbits.LATC0 = 0;
    }
    
    /* Restore UART */
    PIE1 = saved_pie1;
    
    return edges;
}

void WaterMeter_Initialize(void)
{
    /* CRITICAL: Sense pins must be in ANALOG mode for comparators to work!
     * ANSEL bit: 0=analog, 1=digital */
    ANCON1 &= ~0x01;       /* ANSEL8=0 → RB1 (AN8) = ANALOG */
    ANCON0 &= ~(1 << 4);   /* ANSEL4=0 → RA5 (AN4) = ANALOG */
    
    /* Configure comparators:
     * - Both enabled, internal output only
     * - CREF=1: CVref on non-inverting (+) input
     * - CCH=10: Cross-mapped channels (see datasheet Note 2)
     *   CMP1 → C2INB (RA5) = LCB
     *   CMP2 → C1INB (RB1) = LCA
     * - EVPOL=11: Interrupt on any edge */
    CM1CON = 0x9E;  /* ON, EVPOL=11, CREF=1, CCH=10 */
    CM2CON = 0x9E;
    
    /* Set CVref to 2.5V (optimal for 330k+330k bias divider) */
    SetCVRef(CVR_OPTIMAL);

    /* Initialize ADC path for comparator-free raw-node capture. */
    ADC_InitializeRawCapture();
    
    /* Register comparator callbacks */
    CMP1_InterruptCallbackRegister(CMP1_Callback);
    CMP2_InterruptCallbackRegister(CMP2_Callback);
    
    /* Enable comparator interrupts */
    PIR4bits.CMP1IF = 0;
    PIR4bits.CMP2IF = 0;
    PIE4bits.CMP1IE = 1;
    PIE4bits.CMP2IE = 1;
    
    /* Configure drive pins as outputs, start LOW */
    TRISCbits.TRISC5 = 0;  /* RC5 = LCA drive */
    TRISCbits.TRISC0 = 0;  /* RC0 = LCB drive */
    LATCbits.LATC5 = 0;
    LATCbits.LATC0 = 0;
    
    /* Configure S0 output (RB2) */
    TRISBbits.TRISB2 = 0;
    LATBbits.LATB2 = 0;
    
    /* Register TMR0 callback for S0 pulse timing */
    TMR0_OverflowCallbackRegister(S0_TimerCallback);
    
    /* Take initial readings to establish air baseline and state. */
    lca_count = MeasureTank(1);
    lca_polled_count = MeasureTankPolled(1);
    lca_raw_span = MeasureTankRawADC(1, &lca_raw_min, &lca_raw_max);
    __delay_us(100);
    lcb_count = MeasureTank(2);
    lcb_raw_span = MeasureTankRawADC(2, NULL, NULL);

    lca_span_baseline = lca_raw_span;
    lcb_span_baseline = lcb_raw_span;
    current_state = DetermineMetalStateFromSpan(lca_raw_span, lcb_raw_span, 0U);
    air_rearm_count = (current_state == 0U) ? ROTATION_REARM_SAMPLES : 0U;
    sequence_armed = (current_state == 0U);
    lca_below_threshold_samples = 0U;
    lcb_below_threshold_samples = 0U;

    /* Capture a boot-time comparator sweep before the normal loop starts.
     * This gives rev2 boards an immediate read on comparator viability. */
    CaptureDiagnosticSweep();
}

bool WaterMeter_Process(void)
{
    bool rotation_detected = false;
    uint8_t previous_state;
    int8_t direction;
    
    /* Measure both LC tanks using GPIO HiZ method */
    lca_count = MeasureTank(1);
    lca_polled_count = MeasureTankPolled(1);
    lca_raw_span = MeasureTankRawADC(1, &lca_raw_min, &lca_raw_max);
    __delay_us(100);  /* Brief gap between measurements */
    lcb_count = MeasureTank(2);
    lcb_raw_span = MeasureTankRawADC(2, NULL, NULL);
    
    /* Determine metal presence from span drop relative to the air baseline. */
    previous_state = current_state;
    last_rotation_direction = 0;

    uint8_t new_state = DetermineMetalStateFromSpan(lca_raw_span, lcb_raw_span, current_state);
    direction = ProcessQuadratureSequence(previous_state, new_state);
    current_state = new_state;
    
    if (direction == +1) {
        forward_count++;
        last_rotation_direction = +1;
        rotation_detected = true;
        sequence_armed = false;
        air_rearm_count = 0U;
        S0_Pulse();
    } else if (direction == -1) {
        reverse_count++;
        last_rotation_direction = -1;
        rotation_detected = true;
        sequence_armed = false;
        air_rearm_count = 0U;
    }

    if (new_state == 0U) {
        lca_span_baseline = UpdateBaseline(lca_span_baseline, lca_raw_span);
        lcb_span_baseline = UpdateBaseline(lcb_span_baseline, lcb_raw_span);

        if (air_rearm_count < ROTATION_REARM_SAMPLES) {
            air_rearm_count++;
        }
        if (air_rearm_count >= ROTATION_REARM_SAMPLES) {
            sequence_armed = true;
        }
    } else {
        air_rearm_count = 0U;
    }

    if (++diagnostic_loop_divider >= DIAGNOSTIC_SWEEP_INTERVAL) {
        diagnostic_loop_divider = 0;
        RunDiagnosticSweep();
    }
    
    return rotation_detected;
}

uint32_t WaterMeter_GetRotationCount(void)
{
    return forward_count;  /* Return forward count as primary */
}

uint32_t WaterMeter_GetForwardCount(void)
{
    return forward_count;
}

uint32_t WaterMeter_GetReverseCount(void)
{
    return reverse_count;
}

int8_t WaterMeter_GetLastRotationDirection(void)
{
    return last_rotation_direction;
}

uint8_t WaterMeter_GetState(void)
{
    return current_state;
}

uint8_t WaterMeter_GetLCA_Count(void)
{
    return lca_count;
}

uint8_t WaterMeter_GetLCB_Count(void)
{
    return lcb_count;
}

uint8_t WaterMeter_GetLCA_PolledCount(void)
{
    return lca_polled_count;
}

uint8_t WaterMeter_GetCVRef(void)
{
    return current_cvref;
}

uint16_t WaterMeter_GetLCA_RawMin(void)
{
    return lca_raw_min;
}

uint16_t WaterMeter_GetLCA_RawMax(void)
{
    return lca_raw_max;
}

uint16_t WaterMeter_GetLCA_RawSpan(void)
{
    return lca_raw_span;
}

uint16_t WaterMeter_GetLCB_RawSpan(void)
{
    return lcb_raw_span;
}

uint16_t WaterMeter_GetLCA_SpanBaseline(void)
{
    return lca_span_baseline;
}

uint16_t WaterMeter_GetLCB_SpanBaseline(void)
{
    return lcb_span_baseline;
}

bool WaterMeter_IsDiagnosticSweepReady(void)
{
    return diagnostic_sweep_ready;
}

void WaterMeter_ClearDiagnosticSweepReady(void)
{
    diagnostic_sweep_ready = false;
}

uint8_t WaterMeter_GetDiagnosticPointCount(void)
{
    return DIAGNOSTIC_SWEEP_POINT_COUNT;
}

uint8_t WaterMeter_GetDiagnosticCVRef(uint8_t index)
{
    if (index >= DIAGNOSTIC_SWEEP_POINT_COUNT) {
        return 0U;
    }
    return diagnostic_cvr_points[index];
}

uint8_t WaterMeter_GetDiagnosticLCA(uint8_t index)
{
    if (index >= DIAGNOSTIC_SWEEP_POINT_COUNT) {
        return 0U;
    }
    return diagnostic_lca_counts[index];
}

uint8_t WaterMeter_GetDiagnosticLCB(uint8_t index)
{
    if (index >= DIAGNOSTIC_SWEEP_POINT_COUNT) {
        return 0U;
    }
    return diagnostic_lcb_counts[index];
}

uint8_t WaterMeter_GetDiagnosticAPl(uint8_t index)
{
    if (index >= DIAGNOSTIC_SWEEP_POINT_COUNT) {
        return 0U;
    }
    return diagnostic_apl_counts[index];
}

uint8_t WaterMeter_GetDiagnosticBPl(uint8_t index)
{
    if (index >= DIAGNOSTIC_SWEEP_POINT_COUNT) {
        return 0U;
    }
    return diagnostic_bpl_counts[index];
}

uint16_t WaterMeter_GetDiagnosticRawSpan(uint8_t index)
{
    if (index >= DIAGNOSTIC_SWEEP_POINT_COUNT) {
        return 0U;
    }
    return diagnostic_raw_spans_a[index];
}

uint16_t WaterMeter_GetDiagnosticRawSpanB(uint8_t index)
{
    if (index >= DIAGNOSTIC_SWEEP_POINT_COUNT) {
        return 0U;
    }
    return diagnostic_raw_spans_b[index];
}
