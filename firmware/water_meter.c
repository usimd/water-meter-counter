/**
 * Water Meter Counter Module Implementation
 * 
 * @file water_meter.c
 * 
 * @brief Implementation of water meter rotation counter using dual LC tank circuits
 */

#include "water_meter.h"
#include "mcc_generated_files/system/system.h"

/* Forward declarations */
static void WaterMeter_TMR0_Callback(void);

/* LC tank drive and sense timing constants */
#define LC_TANK_DRIVE_PULSE_US    100  /* Initial pulse width in microseconds - increased for better excitation */
#define LC_TANK_MEASUREMENT_US    2000  /* Oscillation measurement period in microseconds (2ms) */
#define LC_TANK_SETTLE_US         100  /* Settling time after driving before measurement */

/**
 * @brief Water meter state machine context
 */
typedef struct {
    water_meter_state_t current_state;
    water_meter_state_t previous_state;
    uint32_t rotation_count;
    bool lca_state;              /**< Current state of LC Tank A */
    bool lcb_state;              /**< Current state of LC Tank B */
    uint16_t transition_counter; /**< Counter for detecting complete rotation */
    bool s0_pulse_active;        /**< Flag indicating S0 pulse in progress */
    uint16_t s0_pulse_width;     /**< Counter for S0 pulse width control */
    bool last_valid_transition;  /**< Track last valid transition for rotation detection */
    uint8_t lca_oscillation_count; /**< Oscillation count for LC Tank A */
    uint8_t lcb_oscillation_count; /**< Oscillation count for LC Tank B */
    uint8_t lca_threshold;       /**< Calibrated threshold for LC Tank A metal detection */
    uint8_t lcb_threshold;       /**< Calibrated threshold for LC Tank B metal detection */
} water_meter_context_t;

static water_meter_context_t meter_ctx = {
    .current_state = STATE_INVALID,
    .previous_state = STATE_INVALID,
    .rotation_count = 0,
    .lca_state = false,
    .lcb_state = false,
    .transition_counter = 0,
    .s0_pulse_active = false,
    .s0_pulse_width = 0,
    .last_valid_transition = false,
    .lca_oscillation_count = 0,
    .lcb_oscillation_count = 0,
    .lca_threshold = 18,  /* Calibrated threshold: no-metal=20-25, metal=12-13, threshold=18 */
    .lcb_threshold = 18   /* Calibrated threshold for LC Tank B */
};

/* S0 pulse timing constants */
/* With TMR0 at 100ms period */
#define S0_PULSE_WIDTH_MS  100  /* S0 pulse width in milliseconds */
#define S0_PULSE_COUNTER   1    /* Pulse counter ticks (1 tick = 100ms) */

/* Deep sleep timing constants */
/* Geometry: Disc 7.5mm radius, inductors at 5mm from center, 120° apart */
/* Only half disc is metal. Each inductor sees metal ~60° per 120° rotation sector */
/* Conservative timeout to ensure no rotations are missed accounting for */
/* variable flow rates and inductor positioning (120° offset) */
#define SLEEP_TIMEOUT_MIN_MS   2500  /* Minimum safe timeout (2.5 seconds) */
#define SLEEP_TIMEOUT_DEFAULT_MS 3000 /* Default timeout (3 seconds) */

/* TMR0 configuration: 100ms per interrupt from hardware */
/* With 100ms hardware interrupt, we can directly use it for sleep wakeups */
#define TMR0_INTERRUPT_PERIOD_MS  100   /* Hardware generates interrupt every 100ms */
#define SLEEP_WAKEUP_GRANULE_MS   100   /* Software wakeup granularity in milliseconds */
#define SLEEP_PRESCALER_COUNT     1     /* No software prescaling needed */

typedef struct {
    uint16_t sleep_timeout_ms;   /* Target sleep duration in milliseconds */
    uint16_t sleep_counter;      /* Counter for sleep timing in 100ms units */
    bool sleep_enabled;          /* Sleep mode enabled flag */
    bool can_sleep;              /* Flag indicating system is in idle state */
} sleep_context_t;

static sleep_context_t sleep_ctx = {
    .sleep_timeout_ms = SLEEP_TIMEOUT_DEFAULT_MS,
    .sleep_counter = 0,
    .sleep_enabled = false,
    .can_sleep = true
};

/* Global oscillation counter for ISR context */
static volatile uint8_t oscillation_counter = 0;
static volatile uint8_t measurement_active = 0;

/**
 * @brief Charge LC Tank A and measure oscillation count
 * 
 * Drives the LC tank at RC5 (LCA) and counts oscillations on the sense pin
 * to detect metal presence (damped oscillations indicate metal nearby).
 * 
 * @return Number of oscillations counted during measurement period
 */
static uint8_t WaterMeter_ChargeLCA(void)
{
    oscillation_counter = 0;
    measurement_active = 1;
    
    /* Set pulse input (RC5) as output */
    LCA_SetDigitalOutput();
    LCA_SetLow();
    __delay_us(50);
    
    /* Drive RC5 with pulses to excite the LC tank */
    /* Count oscillations that occur on the sense pin (RA5) during this time */
    
    for (int pulse = 0; pulse < 12; pulse++) {
        /* Drive HIGH for 500µs to excite the tank */
        LCA_SetHigh();
        __delay_us(500);
        
        /* Drive LOW for 500µs - tank ringdown continues oscillating */
        LCA_SetLow();
        __delay_us(500);
    }
    
    /* Stop measurement and ensure clean state for next cycle */
    measurement_active = 0;
    LCA_SetLow();
    
    /* CRITICAL: Set RC5 to INPUT to allow LC tank to discharge naturally */
    /* If RC5 stays as OUTPUT LOW, it actively damps the tank */
    /* Setting to INPUT allows free oscillation decay */
    LCA_SetDigitalInput();
    
    /* Wait for LC tank oscillations to die out completely */
    __delay_us(2000);  /* 2ms settling time */
    
    /* Return accumulated oscillation count */
    uint8_t count = oscillation_counter;
    oscillation_counter = 0;
    return count;
}

/**
 * @brief Charge LC Tank B and measure oscillation count
 * 
 * Drives the LC tank at RC0 (LCB) and counts oscillations on the sense pin
 * to detect metal presence (damped oscillations indicate metal nearby).
 * 
 * @return Number of oscillations counted during measurement period
 */
static uint8_t WaterMeter_ChargeLCB(void)
{
    oscillation_counter = 0;
    measurement_active = 1;
    
    /* Set pulse input (RC0) as output */
    LCB_SetDigitalOutput();
    LCB_SetLow();
    __delay_us(50);
    
    /* Drive RC0 with pulses to excite the LC tank */
    /* Count oscillations that occur on the sense pin (RB1) during this time */
    
    for (int pulse = 0; pulse < 12; pulse++) {
        /* Drive HIGH for 600µs to excite the tank */
        LCB_SetHigh();
        __delay_us(600);
        
        /* Drive LOW for 600µs - tank ringdown continues oscillating */
        LCB_SetLow();
        __delay_us(600);
    }
    
    /* Stop measurement and ensure clean state for next cycle */
    measurement_active = 0;
    LCB_SetLow();
    
    /* CRITICAL: Set RC0 to INPUT to allow LC tank to discharge naturally */
    /* If RC0 stays as OUTPUT LOW, it actively damps the tank */
    /* Setting to INPUT allows free oscillation decay */
    LCB_SetDigitalInput();
    
    /* Wait for LC tank oscillations to die out completely */
    __delay_us(2000);  /* 2ms settling time */
    
    /* Return accumulated oscillation count */
    uint8_t count = oscillation_counter;
    oscillation_counter = 0;
    return count;
}

/**
 * @brief Update the current state based on LC tank oscillation measurements
 * 
 * Charges each LC tank and counts oscillations to detect metal presence.
 * Metal near the tank dampens oscillations (fewer oscillations).
 * State is determined by comparing oscillation count to calibrated threshold.
 */
static void WaterMeter_UpdateState(void)
{
    /* SINGLE-SENSOR MODE: RC0 hardware fault, LCB disabled */
    meter_ctx.lca_oscillation_count = WaterMeter_ChargeLCA();
    meter_ctx.lcb_oscillation_count = 0;  // LCB disabled due to RC0 pin failure
    
    /* Determine state based on oscillation counts */
    /* Lower oscillation count = metal detected (damped oscillations) */
    meter_ctx.lca_state = (meter_ctx.lca_oscillation_count <= meter_ctx.lca_threshold);
    meter_ctx.lcb_state = false;  // LCB permanently disabled
    
    /* Single-sensor state machine: Pulse on each LCA transition */
    static bool last_lca_state = false;
    if (meter_ctx.lca_state != last_lca_state) {
        last_lca_state = meter_ctx.lca_state;
        if (meter_ctx.lca_state) {
            // Metal detected - treat as rotation pulse
            meter_ctx.current_state = STATE_LC_A_HIGH;
        } else {
            meter_ctx.current_state = STATE_INVALID;
        }
    } else {
        meter_ctx.current_state = STATE_INVALID;
    }
}

/**
 * @brief Generate S0 pulse on meter reading
 * 
 * S0 interface typically requires a ~100ms pulse
 */
static void WaterMeter_GenerateS0Pulse(void)
{
    if (!meter_ctx.s0_pulse_active) {
        meter_ctx.s0_pulse_active = true;
        meter_ctx.s0_pulse_width = 0;
        S0_SetHigh();  // Start S0 pulse
    }
}

/**
 * @brief Handle S0 pulse timing
 * 
 * Called from Process() to decrement pulse timer
 */
static void WaterMeter_HandleS0Pulse(void)
{
    if (meter_ctx.s0_pulse_active) {
        meter_ctx.s0_pulse_width++;
        
        // End pulse after specified width
        if (meter_ctx.s0_pulse_width >= S0_PULSE_COUNTER) {
            S0_SetLow();  // End S0 pulse
            meter_ctx.s0_pulse_active = false;
            meter_ctx.s0_pulse_width = 0;
        }
    }
}

/**
 * @brief Detect rotation from LC tank state transitions
 * 
 * Water meter typically has two magnets that trigger LC tank circuits
 * A full rotation requires a specific sequence of state transitions
 */
static bool WaterMeter_DetectRotation(void)
{
    bool rotation_detected = false;
    
    // Check for valid state transition
    // A rotation is detected when we see the proper LC tank resonance sequence
    
    if (meter_ctx.previous_state != STATE_TRANSITION && 
        meter_ctx.current_state == STATE_TRANSITION) {
        // Valid transition detected
        meter_ctx.transition_counter++;
        
        // A full rotation typically involves 2 transition points
        // Adjust this based on actual meter magnet configuration
        if (meter_ctx.transition_counter >= 2) {
            rotation_detected = true;
            meter_ctx.transition_counter = 0;
            meter_ctx.rotation_count++;
            WaterMeter_GenerateS0Pulse();
        }
    } else if (meter_ctx.current_state != STATE_TRANSITION) {
        // Reset transition counter if not in transition state
        meter_ctx.transition_counter = 0;
    }
    
    return rotation_detected;
}

void WaterMeter_Initialize(void)
{
    // Initialize state machine
    meter_ctx.current_state = STATE_INVALID;
    meter_ctx.previous_state = STATE_INVALID;
    meter_ctx.rotation_count = 0;
    meter_ctx.s0_pulse_active = false;
    meter_ctx.s0_pulse_width = 0;
    meter_ctx.transition_counter = 0;
    meter_ctx.last_valid_transition = false;
    
    // Initialize S0 output (RB2)
    S0_SetDigitalOutput();
    S0_SetLow();  // Ensure S0 starts low
    
    // Initialize comparators
    CMP1_Initialize();
    CMP2_Initialize();
    CMP1_Enable();
    CMP2_Enable();
    
    // Register comparator interrupt callbacks
    CMP1_InterruptCallbackRegister(WaterMeter_CMP1_Callback);
    CMP2_InterruptCallbackRegister(WaterMeter_CMP2_Callback);
    
    // Start TMR0 for sleep timing and S0 pulse control
    TMR0_OverflowCallbackRegister(WaterMeter_TMR0_Callback);
    TMR0_Start();
    
    // Read initial state
    WaterMeter_UpdateState();
}

bool WaterMeter_Process(void)
{
    bool rotation_detected = false;
    
    // Store previous state
    meter_ctx.previous_state = meter_ctx.current_state;
    
    // Update current state from comparator outputs
    WaterMeter_UpdateState();
    
    // Detect rotation from state transitions
    rotation_detected = WaterMeter_DetectRotation();
    
    // Handle S0 pulse timing
    WaterMeter_HandleS0Pulse();
    
    return rotation_detected;
}

void WaterMeter_CMP1_Callback(void)
{
    /* CMP1 (LC Tank A) output changed during measurement */
    if (measurement_active) {
        oscillation_counter++;
    }
}

void WaterMeter_CMP2_Callback(void)
{
    /* CMP2 (LC Tank B) output changed during measurement */
    if (measurement_active) {
        oscillation_counter++;
    }
}

uint32_t WaterMeter_GetRotationCount(void)
{
    return meter_ctx.rotation_count;
}

void WaterMeter_ResetCounter(void)
{
    meter_ctx.rotation_count = 0;
}

bool WaterMeter_GetLCA_State(void)
{
    return meter_ctx.lca_state;
}

bool WaterMeter_GetLCB_State(void)
{
    return meter_ctx.lcb_state;
}

/**
 * @brief Enter deep sleep mode with timer-based wakeup
 * 
 * Configures TMR0 interrupt period and puts device into sleep.
 * The device will wake up periodically to check for water meter rotation.
 * 
 * Q_3 = 2.5 m³/h specification:
 * - Maximum flow rate: 2500 L/h
 * - 1 rotation = 1 L
 * - Minimum time per rotation: 1.44 seconds
 * - Safe timeout: 1500ms minimum (1.5 seconds)
 */
void WaterMeter_EnterDeepSleep(uint16_t timeout_ms)
{
    /* Validate timeout is at minimum safe value */
    if (timeout_ms < SLEEP_TIMEOUT_MIN_MS) {
        timeout_ms = SLEEP_TIMEOUT_MIN_MS;
    }
    
    /* Ensure S0 pulse is complete before sleeping */
    if (meter_ctx.s0_pulse_active) {
        return;  /* Cannot sleep while pulse is active */
    }
    
    sleep_ctx.sleep_timeout_ms = timeout_ms;
    sleep_ctx.sleep_counter = 0;
    sleep_ctx.sleep_enabled = true;
    
    /* Configure TMR0 for wake-up timing */
    /* TMR0 generates interrupts every 100ms */
    
    /* Ensure comparators remain enabled during sleep */
    CMP1_Enable();
    CMP2_Enable();
    
    /* Clear any pending TMR0 interrupt */
    INTCONbits.TMR0IF = 0;
    
    /* Enable TMR0 interrupt for periodic wakeup */
    INTCONbits.TMR0IE = 1;
    
#if __DEBUG
    /* Debug build: Use idle loop to allow debugger to function properly */
    /* The TMR0 interrupt will increment sleep_counter every 100ms */
    /* and will set sleep_enabled to false when timeout expires */
    while (sleep_ctx.sleep_enabled) {
        /* Idle here - interrupts are still enabled and will execute */
        /* This allows TMR0 to count down and wake us when timeout expires */
        Nop();
    }
#else
    /* Release build: Use actual hardware sleep for maximum power savings */
    /* The device will wake on TMR0 interrupt */
    Sleep();
#endif
    
    /* Execution continues here when timeout expires or device wakes */
    sleep_ctx.sleep_enabled = false;
}

/**
 * @brief Check if system is in a state safe for sleep
 * 
 * Returns false if:
 * - S0 pulse is currently being generated
 * - A rotation was just detected (give time for pulse to complete)
 * - Device is not in idle state
 * 
 * @return true if safe to sleep, false otherwise
 */
bool WaterMeter_CanSleep(void)
{
    /* Cannot sleep while S0 pulse is active */
    if (meter_ctx.s0_pulse_active) {
        return false;
    }
    
    /* Cannot sleep if comparators show activity */
    if (meter_ctx.current_state != meter_ctx.previous_state) {
        return false;  /* State just changed, give time to settle */
    }
    
    return sleep_ctx.can_sleep;
}

/**
 * @brief Update sleep timer (called from TMR0 ISR context)
 * 
 * This function should be called from the TMR0 overflow interrupt handler
 * to decrement the sleep counter and trigger actions when timeout expires.
 * 
 * @return true if sleep timeout has been reached, false otherwise
 */
bool WaterMeter_UpdateSleepTimer(void)
{
    if (!sleep_ctx.sleep_enabled) {
        return false;
    }
    
    sleep_ctx.sleep_counter++;
    
    /* Check if timeout has been reached */
    if (sleep_ctx.sleep_counter >= sleep_ctx.sleep_timeout_ms) {
        sleep_ctx.sleep_counter = 0;
        return true;  /* Timeout reached, wakeup needed */
    }
    
    return false;
}

/**
 * @brief Set sleep mode enabled state
 * 
 * @param enabled true to enable sleep mode, false to disable
 */
void WaterMeter_SetSleepEnabled(bool enabled)
{
    sleep_ctx.sleep_enabled = enabled;
}

/**
 * @brief Get current sleep timeout value
 * 
 * @return Current sleep timeout in milliseconds
 */
/**
 * @brief Get current sleep timeout value
 * 
 * @return Current sleep timeout in milliseconds
 */
uint16_t WaterMeter_GetSleepTimeout(void)
{
    return sleep_ctx.sleep_timeout_ms;
}

/**
 * @brief Set calibration threshold for LC Tank A
 * 
 * @param threshold Oscillation count threshold (typically 40 for 70% detection)
 */
void WaterMeter_SetLCA_Threshold(uint8_t threshold)
{
    meter_ctx.lca_threshold = threshold;
}

/**
 * @brief Set calibration threshold for LC Tank B
 * 
 * @param threshold Oscillation count threshold (typically 40 for 70% detection)
 */
void WaterMeter_SetLCB_Threshold(uint8_t threshold)
{
    meter_ctx.lcb_threshold = threshold;
}

/**
 * @brief Get last measured oscillation count for LC Tank A
 * 
 * @return Oscillation count from last measurement
 */
uint8_t WaterMeter_GetLCA_OscillationCount(void)
{
    return meter_ctx.lca_oscillation_count;
}

/**
 * @brief Get last measured oscillation count for LC Tank B
 * 
 * @return Oscillation count from last measurement
 */
uint8_t WaterMeter_GetLCB_OscillationCount(void)
{
    return meter_ctx.lcb_oscillation_count;
}

/**
 * @brief TMR0 Interrupt Handler for Sleep Wake-up Timing
 * 
 * This ISR is called every 1ms (TMR0 interrupt period).
 * It decrements the sleep timeout counter and forces wake-up
 * when the timeout expires.
 */
static void WaterMeter_TMR0_Callback(void)
{
    /* Handle sleep timing - TMR0 now interrupts every 100ms */
    if (sleep_ctx.sleep_enabled) {
        sleep_ctx.sleep_counter++;
        
        /* Check if sleep timeout has expired */
        if (sleep_ctx.sleep_counter >= (sleep_ctx.sleep_timeout_ms / SLEEP_WAKEUP_GRANULE_MS)) {
            /* Force device to wake by clearing sleep enable flag */
            sleep_ctx.sleep_enabled = false;
        }
    }
    
    /* Handle S0 pulse timing - now at 100ms resolution */
    if (meter_ctx.s0_pulse_active) {
        meter_ctx.s0_pulse_width++;
        if (meter_ctx.s0_pulse_width >= S0_PULSE_COUNTER) {
            /* Pulse duration complete, deactivate S0 output */
            S0_SetLow();
            meter_ctx.s0_pulse_active = false;
            meter_ctx.s0_pulse_width = 0;
        }
    }
}

/**
 * @brief Get raw CMP1 comparator output status
 * 
 * Returns the direct output of the CMP1 comparator, useful for debugging.
 */
bool WaterMeter_GetCMP1_Output(void) {
    return CMP1_GetOutputStatus();
}

/**
 * @brief Get raw CMP2 comparator output status
 * 
 * Returns the direct output of the CMP2 comparator, useful for debugging.
 */
bool WaterMeter_GetCMP2_Output(void) {
    return CMP2_GetOutputStatus();
}

/**
 * @brief Run diagnostic test on LC tank hardware
 * 
 * Performs several checks:
 * 1. Verifies CVREF module is enabled
 * 2. Measures oscillation count range over 10 cycles
 * 3. Checks if oscillation counts are reasonable
 * 4. Verifies CVref voltage is in expected range
 */
void WaterMeter_DiagnosticTest(void) {
    printf("\n\r=== LC Tank Diagnostic Test ===\r\n");
    
    // Test 1: Verify CVREF enabled
    uint8_t cvr_reg = CVRCON;
    printf("CVRCON Register: 0x%02X\r\n", cvr_reg);
    if (cvr_reg & 0x80) {
        printf("  ✓ CVref module ENABLED\r\n");
        printf("  ✓ CVref value: %d (generates ~%.2f V)\r\n", 
               (cvr_reg & 0x0F), 
               (float)(cvr_reg & 0x0F) * 5.0 / 32.0);
    } else {
        printf("  ✗ ERROR: CVref module DISABLED!\r\n");
    }
    
    // Test 2: Verify comparators are initialized
    printf("\r\nComparator Configuration:\r\n");
    printf("  CM1CON: 0x%02X\r\n", CM1CON);
    printf("  CM2CON: 0x%02X\r\n", CM2CON);
    printf("  Interrupt Status: PIE4=0x%02X, PIR4=0x%02X\r\n", PIE4, PIR4);
    printf("  Global Interrupts: GIE=%d, PEIE=%d\r\n", INTCONbits.GIE, INTCONbits.PEIE);
    printf("\r\nLC Tank Parameters:\r\n");
    printf("  L=1mH, C=2.2nF → f ≈ 107 kHz\r\n");
    printf("  CVref threshold: ~1.87V (CVR=12)\r\n");
    printf("  Measurement window: 6ms → expect ~640 oscillations if undamped\r\n");
    
    // Test output pin configuration
    printf("\r\nOutput Pin Configuration:\r\n");
    printf("  TRISC: 0x%02X\r\n", TRISC);
    printf("  RC5 (LCA) TRIS: %d (0=output, 1=input)\r\n", TRISCbits.TRISC5);
    printf("  RC0 (LCB) TRIS: %d (0=output, 1=input)\r\n", TRISCbits.TRISC0);
    printf("  → CRITICAL: Both should be 0 (outputs) for pulsing to work!\r\n");
    
    // Test RC0 configuration and pulsing capability
    printf("\r\nTesting RC0 (LCB drive) pin configuration:\r\n");
    printf("  TRISC0 (direction): %d (0=output, 1=input)\r\n", TRISCbits.TRISC0);
    printf("  ANSELC (analog): 0x%02X (bit 0 should be 0 for digital)\r\n", *(volatile unsigned char*)0xF1E);
    printf("  RC0 ANSEL bit 0: %d (0=digital, 1=analog)\r\n", (*(volatile unsigned char*)0xF1E) & 0x01);
    
    // Force RC0 to digital output mode
    TRISCbits.TRISC0 = 0;  // Output
    *(volatile unsigned char*)0xF1E &= ~0x01;  // Digital mode (clear ANSELC bit 0)
    
    printf("\r\nAfter forcing digital output mode:\r\n");
    LCB_SetDigitalOutput();
    LCB_SetLow();
    __delay_ms(10);
    printf("  RC0 LOW: LAT=%d, PORT=%d\r\n", LCB_GetValue(), PORTCbits.RC0);
    LCB_SetHigh();
    __delay_ms(10);
    printf("  RC0 HIGH: LAT=%d, PORT=%d (should be LAT=1)\r\n", LCB_GetValue(), PORTCbits.RC0);
    
    // Try direct register write to bypass macro
    LATC |= 0x01;  // Set bit 0 directly
    __delay_ms(10);
    printf("  RC0 after direct LATC write: LATC=0x%02X, PORT=%d\r\n", LATC, PORTCbits.RC0);
    
    // Extended RC0 hardware test
    printf("\r\nRC0 Hardware Test:\r\n");
    LATC &= ~0x01;  // Force LOW
    __delay_ms(10);
    uint8_t rc0_low = PORTCbits.RC0;
    LATC |= 0x01;   // Force HIGH
    __delay_ms(10);
    uint8_t rc0_high = PORTCbits.RC0;
    LATC &= ~0x01;  // Back to LOW
    printf("  Test sequence: LOW->PORT=%d, HIGH->PORT=%d\r\n", rc0_low, rc0_high);
    if (rc0_low == 0 && rc0_high == 0) {
        printf("  ✗ RC0 STUCK LOW - Hardware fault (damaged pin or external short)\r\n");
        printf("  → Measure voltage on RC0 pin - should be 0V if shorted to GND\r\n");
        printf("  → Recommendation: Use different GPIO pin for LCB drive (RC1/RC2/RC3)\r\n");
    } else if (rc0_low == 1 && rc0_high == 1) {
        printf("  ✗ RC0 STUCK HIGH - Check for pull-up or short to VCC\r\n");
    } else if (rc0_low == 0 && rc0_high == 1) {
        printf("  ✓ RC0 works correctly\r\n");
    }
    
    // Check if any peripheral module is controlling RC0
    printf("\r\nChecking peripheral module states:\r\n");
    printf("  SSPCON1 (SPI): 0x%02X (bit 5 SSPEN: %d = %s)\r\n", 
           SSPCON1, (SSPCON1 >> 5) & 0x01, ((SSPCON1 >> 5) & 0x01) ? "SPI ENABLED" : "disabled");
    printf("  T1CON (Timer1): 0x%02X (bit 3 T1OSCEN: %d = %s)\r\n",
           T1CON, (T1CON >> 3) & 0x01, ((T1CON >> 3) & 0x01) ? "OSC ENABLED" : "disabled");
    printf("  CM2CON (CMP2): 0x%02X (bit 2 COE: %d = %s)\r\n",
           CM2CON, (CM2CON >> 2) & 0x01, ((CM2CON >> 2) & 0x01) ? "OUTPUT ENABLED" : "internal");
    
    LCB_SetLow();
    printf("  → If LAT write fails, a peripheral has exclusive control of RC0.\r\n");
    
    // Test RB1 voltage level
    printf("\r\nTesting RB1 (LCB sense) comparator:\r\n");
    printf("  CMP2 output (RB1 vs CVref): %d (0=below 1.72V, 1=above 1.72V)\r\n", WaterMeter_GetCMP2_Output());
    printf("  → If CMP2 is always 1, RB1 is stuck HIGH (>1.72V)\r\n");
    printf("  → Check if LC Tank B circuit exists and is connected to RB1\r\n");
    
    // Test 3: Take 10 measurements and show range
    printf("\r\nMeasuring oscillation counts (10 cycles)...\r\n");
    uint8_t min_lca = 255, max_lca = 0;
    uint8_t min_lcb = 255, max_lcb = 0;
    uint32_t sum_lca = 0, sum_lcb = 0;
    
    // Single test measurement to debug
    printf("DEBUG: Starting single measurement...\r\n");
    printf("DEBUG: GIE=%d, PEIE=%d before measurement\r\n", INTCONbits.GIE, INTCONbits.PEIE);
    printf("DEBUG: CMP1IE=%d, CMP2IE=%d (in PIE4)\r\n", PIE4bits.CMP1IE, PIE4bits.CMP2IE);
    printf("DEBUG: Testing RC5 pulse: about to set HIGH...\r\n");
    LCA_SetHigh();
    printf("DEBUG: RC5 after SetHigh = %d, RA5 pin reads: %d\r\n", LCA_GetValue(), IO_RA5_GetValue());
    LCA_SetLow();
    printf("DEBUG: RC5 after SetLow = %d, RA5 pin reads: %d\r\n", LCA_GetValue(), IO_RA5_GetValue());
    printf("DEBUG: CVref output (RA2) should be HIGH: %d\r\n", PORTAbits.RA2);
    
    for (int i = 0; i < 10; i++) {
        uint8_t cmp1_before = WaterMeter_GetCMP1_Output();
        uint8_t cmp2_before = WaterMeter_GetCMP2_Output();
        
        WaterMeter_UpdateState();
        
        uint8_t cmp1_after = WaterMeter_GetCMP1_Output();
        uint8_t cmp2_after = WaterMeter_GetCMP2_Output();
        
        if (meter_ctx.lca_oscillation_count < min_lca) 
            min_lca = meter_ctx.lca_oscillation_count;
        if (meter_ctx.lca_oscillation_count > max_lca) 
            max_lca = meter_ctx.lca_oscillation_count;
        
        if (meter_ctx.lcb_oscillation_count < min_lcb) 
            min_lcb = meter_ctx.lcb_oscillation_count;
        if (meter_ctx.lcb_oscillation_count > max_lcb) 
            max_lcb = meter_ctx.lcb_oscillation_count;
        
        sum_lca += meter_ctx.lca_oscillation_count;
        sum_lcb += meter_ctx.lcb_oscillation_count;
        
        printf("  [%d] LCA: %d  LCB: %d  (CMP1: %d→%d, CMP2: %d→%d)\r\n", i+1, 
               meter_ctx.lca_oscillation_count, 
               meter_ctx.lcb_oscillation_count,
               cmp1_before, cmp1_after,
               cmp2_before, cmp2_after);
        
        __delay_ms(100);
    }
    
    printf("\r\nResults:\r\n");
    printf("  LCA: min=%d, max=%d, avg=%d\r\n", min_lca, max_lca, (uint8_t)(sum_lca/10));
    printf("  LCB: min=%d, max=%d, avg=%d\r\n", min_lcb, max_lcb, (uint8_t)(sum_lcb/10));
    
    // Health check
    printf("\r\nHealth Status:\r\n");
    if (max_lca < 15 || max_lcb < 15) {
        printf("  ✗ CRITICAL: Oscillation counts too low (<15)\r\n");
        printf("  → Check LC tank hardware (inductor/capacitor)\r\n");
        printf("  → Check CVref is enabled and outputting voltage\r\n");
        printf("  → Check RC5 and RC0 pins are in digital mode\r\n");
    } else if (max_lca < 30 || max_lcb < 30) {
        printf("  ⚠ WARNING: Low oscillation counts (<30)\r\n");
        printf("  → Consider increasing LC_TANK_DRIVE_PULSE_US to 200-300µs\r\n");
        printf("  → Check for noise/EMI on the board\r\n");
    } else {
        printf("  ✓ Good: Oscillation counts are healthy\r\n");
    }
    
    // Check variance
    uint8_t range_lca = max_lca - min_lca;
    uint8_t range_lcb = max_lcb - min_lcb;
    if (range_lca > 15 || range_lcb > 15) {
        printf("  ⚠ WARNING: High variance in measurements (range > 15)\r\n");
        printf("  → Suggests EMI or unstable power supply\r\n");
    } else {
        printf("  ✓ Good: Measurements are stable\r\n");
    }
    
    printf("\r\nTo test metal detection:\r\n");
    printf("  1. Note the oscillation counts above\r\n");
    printf("  2. Bring ferrous metal (iron nail, screwdriver) near inductors\r\n");
    printf("  3. Counts should drop by 20-50%% when metal is close\r\n");
    printf("  4. Current threshold: %d (counts below this = metal detected)\r\n\r\n",
           meter_ctx.lca_threshold);
}

/**
 * End of File
 */

