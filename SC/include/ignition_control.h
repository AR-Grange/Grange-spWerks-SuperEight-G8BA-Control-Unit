/**
 * @file    ignition_control.h
 * @brief   Module 3 — Ignition Timing Control
 *
 * Manages coil-on-plug (COP) ignition for 8-cylinder G8BA.
 * Advance angle is computed from:
 *   ADVANCE = BASE_TABLE(rpm, load) + CLT_corr + IAT_corr − KNOCK_retard
 *
 * Dwell is scheduled as a fixed charge window (G8BA_IGN_COIL_DWELL_MS)
 * ending at the computed spark angle (°BTDC).
 *
 * Soft rev-cut alternates cylinders; hard cut removes all spark.
 */

#ifndef IGNITION_CONTROL_H
#define IGNITION_CONTROL_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Ignition mode */
typedef enum {
    IGN_MODE_OFF        = 0,
    IGN_MODE_CRANKING   = 1,   /* fixed advance, all cylinders           */
    IGN_MODE_RUNNING    = 2,   /* table-based, closed-loop corrections   */
    IGN_MODE_SOFT_CUT   = 3,   /* alternating cylinder cut               */
    IGN_MODE_HARD_CUT   = 4,   /* all cylinders disabled                 */
} ign_mode_t;

/** Per-cylinder ignition state */
typedef struct {
    uint8_t     cyl_index;
    floatdeg_t  advance_deg;   /* advance angle (°BTDC), positive = advance */
    floatdeg_t  spark_angle;   /* absolute crank angle of spark event     */
    floatdeg_t  dwell_start;   /* absolute crank angle to begin dwell     */
    bool        coil_active;   /* coil currently charging                 */
    bool        enabled;       /* cylinder allowed to fire                */
    uint32_t    fire_count;    /* number of times this cylinder has fired */
} coil_state_t;

/** Ignition advance breakdown for diagnostics */
typedef struct {
    float   base;          /* table lookup value (°BTDC)                 */
    float   clt_corr;      /* coolant temperature correction             */
    float   iat_corr;      /* inlet air temperature correction           */
    float   knock_retard;  /* knock control retard (negative)            */
    float   total;         /* final commanded advance                    */
} ign_advance_t;

/** Module-level ignition status */
typedef struct {
    coil_state_t    coils[G8BA_CYLINDERS];
    ign_advance_t   advance_breakdown;
    ign_mode_t      mode;
    float           dwell_ms;      /* current coil charge time (ms)      */
    uint8_t         soft_cut_mask; /* bitmask — which cylinders are cut   */
    uint32_t        total_sparks;  /* total spark events since boot       */
} ign_status_t;

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

extern volatile ign_status_t g_ignition;

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise ignition module.
 *         Loads advance table from flash, sets up GPIO for 8 COP coils.
 * @return G8BA_OK on success.
 */
g8ba_status_t ignition_init(void);

/**
 * @brief  Compute ignition advance for current operating point.
 *         Should be called every TASK_PERIOD_MEDIUM_MS.
 * @param  rpm          Current engine speed
 * @param  load_pct     Engine load (MAP/MAF based %)
 * @param  clt_c        Coolant temperature (°C)
 * @param  iat_c        Inlet air temperature (°C)
 * @param  knock_retard Active knock retard from knock module (°)
 * @param[out] out      Calculated advance breakdown
 */
void ignition_calc_advance(rpm_t rpm, float load_pct,
                           float clt_c, float iat_c,
                           float knock_retard,
                           ign_advance_t *out);

/**
 * @brief  Schedule ignition event for cylinder `cyl`.
 *         Called by the crank angle scheduler at the appropriate tooth event.
 * @param  cyl_index   0-based cylinder index
 */
void ignition_schedule_spark(uint8_t cyl_index);

/**
 * @brief  Set ignition operating mode.
 * @param  mode  New mode
 */
void ignition_set_mode(ign_mode_t mode);

/**
 * @brief  Apply per-cylinder soft cut mask (bitmask: bit N = cylinder N).
 *         Masked cylinders skip spark on the next cycle.
 * @param  mask  8-bit cylinder enable mask (0xFF = all enabled)
 */
void ignition_set_soft_cut_mask(uint8_t mask);

/**
 * @brief  Return current advance angle for cylinder `cyl` (°BTDC).
 */
float ignition_get_advance(uint8_t cyl_index);

/**
 * @brief  Adjust dwell time based on battery voltage.
 * @param  vbatt  Battery voltage (V)
 */
void ignition_update_dwell(float vbatt);

/**
 * @brief  ISR-safe coil drive callback — fires when dwell window opens.
 *         Registered with RusEFI angle-based scheduler.
 */
void ignition_coil_on_isr(uint8_t cyl_index);

/**
 * @brief  ISR-safe spark callback — cuts coil current to produce spark.
 *         Registered with RusEFI angle-based scheduler.
 */
void ignition_coil_off_isr(uint8_t cyl_index);

#endif /* IGNITION_CONTROL_H */
