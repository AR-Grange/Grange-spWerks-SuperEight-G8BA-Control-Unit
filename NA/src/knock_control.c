/**
 * @file    knock_control.c
 * @brief   Module 5 — Knock Detection & Control Implementation
 *
 * Signal path:
 *   Piezo sensor → charge amplifier → bandpass filter (6.8 kHz) →
 *   STM32H743 ADC (DMA) → RMS/peak detection in window → threshold compare
 *
 * Dynamic threshold:
 *   threshold = noise_floor × KNOCK_THRESHOLD_RATIO
 *   Noise floor is learned per-RPM bin during normal operation.
 *
 * Retard strategy:
 *   - Immediate retard on knock event (per-cylinder)
 *   - Gradual advance recovery (one step per clean cycle)
 *   - Bank-level retard when single-cylinder attribution is uncertain
 */

#include "knock_control.h"
#include "crank_cam_sync.h"
#include <string.h>
#include <math.h>

/* ── RusEFI headers ──────────────────────────────────────────────────────── */
#include "adc_inputs.h"

/* ══════════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Knock detection threshold = noise floor × this ratio */
#define KNOCK_THRESHOLD_RATIO   2.5f

/** Noise floor learning rate (exponential moving average α) */
#define NOISE_FLOOR_ALPHA       0.05f

/** Minimum noise floor to prevent false detect when sensor goes quiet */
#define NOISE_FLOOR_MIN_MV      20.0f

/**
 * IIR bandpass filter coefficients for ~6.8 kHz at Fs=44.1 kHz sampling.
 * 2nd-order Butterworth BPF, ±500 Hz bandwidth.
 * (Pre-computed — recalculate if ADC sample rate changes)
 */
#define BPF_B0   0.06120f
#define BPF_B1   0.0f
#define BPF_B2  -0.06120f
#define BPF_A1  -1.87040f
#define BPF_A2   0.87760f

/* Bank cylinder mapping */
#define IS_BANK1(cyl)  ((cyl) < 4u)     /* cylinders 0–3 = Bank 1 */
#define IS_BANK2(cyl)  ((cyl) >= 4u)    /* cylinders 4–7 = Bank 2 */

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile knock_status_t g_knock;

/** IIR filter state (2-sample history per sensor) */
static float s_bpf_x[G8BA_KNOCK_SENSORS][2]; /* input history  */
static float s_bpf_y[G8BA_KNOCK_SENSORS][2]; /* output history */

/*
 * M-2 FIX: previously a single scalar pair, meaning both bank sensors
 * shared one `s_active_window_cyl` / `s_window_peak_mv`.
 *
 * Failure mode: firing order 1-2-7-8-4-5-6-3 puts Bank1 and Bank2
 * cylinders only 90° apart. If B1 cyl-1 window is open (sensor=0)
 * and cyl-7 (Bank2, sensor=1) fires, knock_window_open() for B2 would
 * overwrite the single s_active_window_cyl = cyl-7.  Then when the
 * angle-scheduled close fires for cyl-1, the guard
 * `s_active_window_cyl != cyl_index` would falsely reject it — B1's
 * knock event would be silently dropped and peak state left stale.
 * ADC data from the B2 sensor could also corrupt the B1 peak value.
 *
 * Fix: one pair per sensor (indexed by knock_sensor_id_t).
 */
static volatile int8_t  s_active_window_cyl[G8BA_KNOCK_SENSORS];
static volatile float   s_window_peak_mv[G8BA_KNOCK_SENSORS];

/* Noise floor per RPM bin (16 bins: 0–800, 800–1600 ... 7200–8000 rpm) */
#define NOISE_RPM_BINS  16u
static float s_noise_floor[G8BA_KNOCK_SENSORS][NOISE_RPM_BINS];

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Apply 2nd-order IIR bandpass filter to one sample */
static float bpf_sample(knock_sensor_id_t sensor, float input_mv)
{
    float *x = s_bpf_x[sensor];
    float *y = s_bpf_y[sensor];

    float output = BPF_B0 * input_mv + BPF_B1 * x[0] + BPF_B2 * x[1]
                 - BPF_A1 * y[0] - BPF_A2 * y[1];

    /* Shift history */
    x[1] = x[0]; x[0] = input_mv;
    y[1] = y[0]; y[0] = output;

    return output;
}

/** Determine which knock sensor monitors a given cylinder */
static knock_sensor_id_t sensor_for_cylinder(uint8_t cyl)
{
    return IS_BANK1(cyl) ? KNOCK_SENSOR_B1 : KNOCK_SENSOR_B2;
}

/** Get noise floor for current RPM bin */
static float get_noise_floor(knock_sensor_id_t sensor, rpm_t rpm)
{
    uint8_t bin = (uint8_t)(rpm / 500u);
    if (bin >= NOISE_RPM_BINS) bin = NOISE_RPM_BINS - 1u;
    float floor_val = s_noise_floor[sensor][bin];
    return (floor_val < NOISE_FLOOR_MIN_MV) ? NOISE_FLOOR_MIN_MV : floor_val;
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t knock_init(void)
{
    memset((void *)&g_knock, 0, sizeof(g_knock));
    memset(s_bpf_x, 0, sizeof(s_bpf_x));
    memset(s_bpf_y, 0, sizeof(s_bpf_y));

    /* Initialise noise floors with a safe default */
    for (uint8_t s = 0u; s < G8BA_KNOCK_SENSORS; s++) {
        for (uint8_t b = 0u; b < NOISE_RPM_BINS; b++) {
            s_noise_floor[s][b] = 50.0f; /* 50 mV starting assumption */
        }
    }

    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        cyl_knock_state_t *c = (cyl_knock_state_t *)&g_knock.cylinders[i];
        c->cyl_index         = i;
        c->retard_deg        = 0.0f;
        c->background_noise  = 50.0f;
        c->knock_threshold   = 50.0f * KNOCK_THRESHOLD_RATIO;
        c->clean_cycles      = 0u;
    }

    for (uint8_t s = 0u; s < G8BA_KNOCK_SENSORS; s++) {
        g_knock.sensors[s].id = (knock_sensor_id_t)s;
        g_knock.sensors[s].window_state = KNOCK_WIN_IDLE;
    }

    for (uint8_t s = 0u; s < G8BA_KNOCK_SENSORS; s++) {
        s_active_window_cyl[s] = -1;
        s_window_peak_mv[s]    = 0.0f;
    }

    return G8BA_OK;
}

void knock_window_open(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;

    knock_sensor_id_t sensor = sensor_for_cylinder(cyl_index);
    knock_sensor_state_t *s  = (knock_sensor_state_t *)&g_knock.sensors[sensor];

    /*
     * SAFE-3 FIX: write order matters.  knock_adc_callback() (DMA IRQ) reads
     * window_state and accumulates into s_window_peak_mv only when state is
     * KNOCK_WIN_SAMPLING.  Previous order:
     *     1. window_state = SAMPLING       ← ADC IRQ now starts accumulating
     *     2. active_window_cyl = cyl       ← into the OLD cylinder's tracking
     *     3. peak_mv = 0                   ← then we wipe that work
     *
     * If an ADC sample lands between (1) and (3) it gets credited to the
     * previous cylinder's peak (still indexed by the same sensor) and is
     * then erased.  Worse, if an open were performed for a different cyl
     * on the same sensor (very rare with proper firing-order angles, but
     * possible during sync acquisition), the IRQ would briefly attribute
     * energy to the wrong cylinder.
     *
     * Correct order:
     *   1. clear peak buffer
     *   2. publish active cylinder for this sensor
     *   3. flip state to SAMPLING last — ADC IRQ now sees fresh, consistent
     *      tracking and a zeroed accumulator.
     *
     * This pattern is the standard "publish data first, then flag" idiom.
     * On Cortex-M7 with volatile fields, store ordering at the C level is
     * preserved within a single thread; the IRQ observes whatever the
     * compiler emitted between the writes — the order above is failsafe.
     */
    s_window_peak_mv[sensor]     = 0.0f;                /* per-sensor (M-2) */
    s_active_window_cyl[sensor]  = (int8_t)cyl_index;  /* per-sensor (M-2) */
    s->window_open_angle         = crank_get_angle();
    s->window_state              = KNOCK_WIN_SAMPLING; /* arm last (SAFE-3) */
}

void knock_window_close(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;
    knock_sensor_id_t sensor = sensor_for_cylinder(cyl_index);

    /* Guard: only close the window that was opened for this cylinder.
     * With per-sensor tracking (M-2 fix) this prevents a B2 close from
     * accidentally clearing a concurrently-open B1 window and vice-versa. */
    if (s_active_window_cyl[sensor] != (int8_t)cyl_index) return;

    knock_sensor_state_t *s = (knock_sensor_state_t *)&g_knock.sensors[sensor];

    s->window_state = KNOCK_WIN_PROCESS;
    g_knock.cylinders[cyl_index].peak_level = s_window_peak_mv[sensor]; /* M-2 */
    s_active_window_cyl[sensor] = -1;
}

void knock_adc_callback(knock_sensor_id_t sensor_id, uint16_t raw_counts)
{
    if (sensor_id >= KNOCK_SENSOR_COUNT) return;

    knock_sensor_state_t *s = (knock_sensor_state_t *)&g_knock.sensors[sensor_id];
    if (s->window_state != KNOCK_WIN_SAMPLING) return;

    /* Convert ADC counts to mV (assuming 3.3V ref, 12-bit ADC) */
    float mv = ((float)raw_counts / 4096.0f) * 3300.0f;
    s->raw_mv = mv;

    /* Apply bandpass filter */
    float filtered = bpf_sample(sensor_id, mv);
    s->filtered_mv = filtered;

    /* Track peak absolute value in window — indexed per-sensor (M-2 fix) */
    float abs_filt = fabsf(filtered);
    if (abs_filt > s_window_peak_mv[sensor_id]) {
        s_window_peak_mv[sensor_id] = abs_filt;
    }
}

void knock_process(void)
{
    rpm_t rpm = crank_get_rpm();
    bool any_knock = false;

    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        cyl_knock_state_t *c = (cyl_knock_state_t *)&g_knock.cylinders[i];
        knock_sensor_id_t sensor = sensor_for_cylinder(i);
        knock_sensor_state_t *s  = (knock_sensor_state_t *)&g_knock.sensors[sensor];

        if (s->window_state != KNOCK_WIN_PROCESS) {
            /* This cylinder's window not yet processed — skip */
            continue;
        }

        float noise  = get_noise_floor(sensor, rpm);
        float thresh = noise * KNOCK_THRESHOLD_RATIO;
        c->knock_threshold   = thresh;
        c->background_noise  = noise;

        if (c->peak_level > thresh) {
            /* KNOCK DETECTED */
            c->knock_detected  = true;
            c->knock_count++;
            c->clean_cycles    = 0u;
            c->retard_deg     += G8BA_KNOCK_RETARD_STEP;
            if (c->retard_deg > G8BA_KNOCK_RETARD_MAX) {
                c->retard_deg = G8BA_KNOCK_RETARD_MAX;
            }
            any_knock = true;
        } else {
            /* Clean cycle — slowly advance back */
            c->knock_detected  = false;
            c->clean_cycles++;
            if (c->retard_deg > 0.0f) {
                c->retard_deg -= G8BA_KNOCK_ADVANCE_STEP;
                if (c->retard_deg < 0.0f) c->retard_deg = 0.0f;
            }
        }

        /* Reset window state for next cycle */
        s->window_state = KNOCK_WIN_IDLE;
    }

    /* Compute bank-level worst-case retard */
    float retard_b1 = 0.0f, retard_b2 = 0.0f;
    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        float r = g_knock.cylinders[i].retard_deg;
        if (IS_BANK1(i) && r > retard_b1) retard_b1 = r;
        if (IS_BANK2(i) && r > retard_b2) retard_b2 = r;
    }
    g_knock.total_retard_b1   = retard_b1;
    g_knock.total_retard_b2   = retard_b2;
    g_knock.any_knock_active  = any_knock;
    if (any_knock) g_knock.total_events++;
}

float knock_get_retard(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;
    return g_knock.cylinders[cyl_index].retard_deg;
}

bool knock_is_active(void)
{
    return g_knock.any_knock_active;
}

void knock_update_noise_floor(rpm_t rpm)
{
    uint8_t bin = (uint8_t)(rpm / 500u);
    if (bin >= NOISE_RPM_BINS) bin = NOISE_RPM_BINS - 1u;

    for (uint8_t s = 0u; s < G8BA_KNOCK_SENSORS; s++) {
        float current = g_knock.sensors[s].filtered_mv;
        if (current < 0.0f) current = -current;

        /* EMA update of noise floor */
        s_noise_floor[s][bin] = s_noise_floor[s][bin] * (1.0f - NOISE_FLOOR_ALPHA)
                              + current * NOISE_FLOOR_ALPHA;

        /* Floor must not go below minimum */
        if (s_noise_floor[s][bin] < NOISE_FLOOR_MIN_MV) {
            s_noise_floor[s][bin] = NOISE_FLOOR_MIN_MV;
        }
    }
}

void knock_flush_open_windows(void)
{
    /*
     * R-1 FIX: Without angle-scheduled knock_window_close() the window state
     * stays KNOCK_WIN_SAMPLING indefinitely — knock_process() polls for
     * KNOCK_WIN_PROCESS and never finds it, so retard is never applied.
     *
     * This function is called from thd_fast_ctrl every 5ms as a fallback.
     * For each sensor that still has an open window we:
     *   1. Copy the peak accumulated so far to the cylinder's peak_level.
     *   2. Transition the state to KNOCK_WIN_PROCESS so knock_process() will
     *      evaluate it on the next call (same tick, later in thd_fast_ctrl).
     *   3. Clear the per-sensor tracking variables.
     *
     * The detection window will be shorter than the designed 10°–60° ATDC
     * window, but any knock energy captured before the flush is still used.
     * This is conservative (may miss very late-arriving knock) but safe.
     *
     * Replace with angle-scheduled knock_window_close() per the C-3 design
     * comment in main.c once scheduleByAngle() integration is complete.
     */
    for (uint8_t s = 0u; s < G8BA_KNOCK_SENSORS; s++) {
        knock_sensor_state_t *sens =
            (knock_sensor_state_t *)&g_knock.sensors[s];

        if (sens->window_state != KNOCK_WIN_SAMPLING) continue;

        int8_t active_cyl = s_active_window_cyl[s];
        if (active_cyl < 0 || (uint8_t)active_cyl >= G8BA_CYLINDERS) {
            /* Stale/invalid — reset to idle without processing */
            sens->window_state   = KNOCK_WIN_IDLE;
            s_active_window_cyl[s] = -1;
            s_window_peak_mv[s]    = 0.0f;
            continue;
        }

        /* Commit captured peak and transition to process */
        g_knock.cylinders[(uint8_t)active_cyl].peak_level = s_window_peak_mv[s];
        sens->window_state     = KNOCK_WIN_PROCESS;
        s_active_window_cyl[s] = -1;
    }
}

g8ba_status_t knock_sensor_test(knock_sensor_id_t sensor)
{
    if (sensor >= KNOCK_SENSOR_COUNT) return G8BA_ERR_RANGE;

    knock_sensor_state_t *s = (knock_sensor_state_t *)&g_knock.sensors[sensor];

    /* Check for open circuit (signal < 5 mV) or short (> 3000 mV) */
    if (s->raw_mv < 5.0f || s->raw_mv > 3000.0f) {
        s->sensor_fault = true;
        return G8BA_ERR_SENSOR;
    }
    s->sensor_fault = false;
    return G8BA_OK;
}
