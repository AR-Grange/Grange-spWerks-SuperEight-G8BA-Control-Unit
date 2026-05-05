/**
 * @file    knock_control.h
 * @brief   Module 5 — Knock (Detonation) Detection & Control
 *
 * Two knock sensors (one per bank) detect combustion knock at ~6.8 kHz.
 * Detection window is gated to [G8BA_KNOCK_WINDOW_ATDC … G8BA_KNOCK_WINDOW_END]
 * after each cylinder's TDC, suppressing mechanical noise.
 *
 * Control strategy:
 *   - Fast retard: G8BA_KNOCK_RETARD_STEP per detected event
 *   - Slow advance: G8BA_KNOCK_ADVANCE_STEP per clean cycle (no knock)
 *   - Per-cylinder retard is tracked individually for accurate correction
 *   - Bank-level retard applied when attribution is ambiguous
 */

#ifndef KNOCK_CONTROL_H
#define KNOCK_CONTROL_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Knock sensor assignment */
typedef enum {
    KNOCK_SENSOR_B1 = 0,   /* Bank 1 — cylinders 1,2,3,4               */
    KNOCK_SENSOR_B2 = 1,   /* Bank 2 — cylinders 5,6,7,8               */
    KNOCK_SENSOR_COUNT = 2,
} knock_sensor_id_t;

/** Detection window state */
typedef enum {
    KNOCK_WIN_IDLE     = 0,  /* outside detection window                 */
    KNOCK_WIN_SAMPLING = 1,  /* actively sampling ADC                    */
    KNOCK_WIN_PROCESS  = 2,  /* window closed, processing result         */
} knock_window_state_t;

/** Per-cylinder knock state */
typedef struct {
    uint8_t             cyl_index;
    float               retard_deg;        /* current applied retard (°CA)   */
    float               background_noise;  /* rolling noise floor (counts)   */
    float               peak_level;        /* max signal in last window      */
    float               knock_threshold;   /* dynamic threshold (noise × k)  */
    bool                knock_detected;    /* knock in the last cycle        */
    uint32_t            knock_count;       /* cumulative knock events        */
    uint32_t            clean_cycles;      /* consecutive knock-free cycles  */
} cyl_knock_state_t;

/** Per-sensor state */
typedef struct {
    knock_sensor_id_t   id;
    float               raw_mv;            /* last ADC reading (mV)          */
    float               filtered_mv;       /* bandpass-filtered signal       */
    knock_window_state_t window_state;
    floatdeg_t          window_open_angle; /* crank angle (°) window opened  */
    bool                sensor_fault;      /* open circuit / short detected  */
} knock_sensor_state_t;

/** Module-level knock status */
typedef struct {
    cyl_knock_state_t   cylinders[G8BA_CYLINDERS];
    knock_sensor_state_t sensors[G8BA_KNOCK_SENSORS];
    float               total_retard_b1;   /* worst-case bank 1 retard       */
    float               total_retard_b2;   /* worst-case bank 2 retard       */
    bool                any_knock_active;  /* knock in any cylinder          */
    uint32_t            total_events;
} knock_status_t;

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

extern volatile knock_status_t g_knock;

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise knock detection module.
 *         Configures ADC channels, bandpass filter coefficients,
 *         registers crank-angle event callbacks for window gating.
 * @return G8BA_OK on success.
 */
g8ba_status_t knock_init(void);

/**
 * @brief  Open detection window for cylinder `cyl`.
 *         Called by crank angle scheduler at G8BA_KNOCK_WINDOW_ATDC
 *         after TDC for the firing cylinder.
 * @param  cyl_index   0-based cylinder index
 */
void knock_window_open(uint8_t cyl_index);

/**
 * @brief  Close detection window and evaluate knock level.
 *         Called at G8BA_KNOCK_WINDOW_END after TDC.
 * @param  cyl_index   0-based cylinder index
 */
void knock_window_close(uint8_t cyl_index);

/**
 * @brief  ADC sample complete callback — called from DMA IRQ.
 *         Applies bandpass filter and updates peak level for open window.
 * @param  sensor_id   Which sensor's ADC completed
 * @param  raw_counts  Raw ADC value
 */
void knock_adc_callback(knock_sensor_id_t sensor_id, uint16_t raw_counts);

/**
 * @brief  Process completed window — determine if knock occurred.
 *         Updates per-cylinder retard and noise floor.
 *         Call from TASK_PERIOD_FAST_MS task after window_close.
 */
void knock_process(void);

/**
 * @brief  Return total ignition retard to apply for cylinder `cyl` (°CA).
 *         Combined per-cylinder + bank retard, clamped to G8BA_KNOCK_RETARD_MAX.
 * @param  cyl_index  0-based cylinder index
 * @return Retard amount (positive = retard from base timing)
 */
float knock_get_retard(uint8_t cyl_index);

/**
 * @brief  Return true if knock was detected in any cylinder this cycle.
 */
bool knock_is_active(void);

/**
 * @brief  Update noise floor baseline (call at idle, no load).
 *         Learns background mechanical noise per RPM point.
 * @param  rpm   Current RPM for noise floor mapping
 */
void knock_update_noise_floor(rpm_t rpm);

/**
 * @brief  Flush any sampling windows that are still open to PROCESS state.
 *
 *         This is the fallback close path used until RusEFI's scheduleByAngle()
 *         is wired to call knock_window_close() at G8BA_KNOCK_WINDOW_END.
 *
 *         Without angle-scheduled close, knock_window_open() sets a sensor to
 *         KNOCK_WIN_SAMPLING but it is never transitioned to KNOCK_WIN_PROCESS,
 *         so knock_process() never evaluates the window and no retard is ever
 *         applied — catastrophic knock goes undetected.
 *
 *         Call once per thd_fast_ctrl tick (5ms) BEFORE knock_process().
 *         The peak captured up to the flush point is used for detection;
 *         this is conservative (short window) but failsafe.
 *
 *         Remove this call and replace with angle-scheduled knock_window_close()
 *         once scheduleByAngle() integration is complete (see C-3 fix comment).
 *
 * @note   Safe to call from any thread context.
 */
void knock_flush_open_windows(void);

/**
 * @brief  Sensor self-test — check signal plausibility at rest.
 * @param  sensor  Sensor to test
 * @return G8BA_OK if sensor responding correctly
 */
g8ba_status_t knock_sensor_test(knock_sensor_id_t sensor);

#endif /* KNOCK_CONTROL_H */
