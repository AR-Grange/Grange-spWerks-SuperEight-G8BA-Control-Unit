/**
 * @file    dcvvt_control.h
 * @brief   Module 4 — Dual CVVT (D-CVVT) Variable Valve Timing Control
 *
 * Hyundai D-CVVT controls four independent oil-pressure cam phasers:
 *   - Bank 1 Intake  (B1IN)
 *   - Bank 1 Exhaust (B1EX)
 *   - Bank 2 Intake  (B2IN)
 *   - Bank 2 Exhaust (B2EX)
 *
 * Each phaser is driven by a PWM oil-control solenoid (OCV).
 * Control loop: PID position controller on cam phase error (°CA).
 * Cam phase is measured from cam sensor edges relative to crank TDC.
 *
 * Target positions are looked up from 2D tables (RPM × Load).
 */

#ifndef DCVVT_CONTROL_H
#define DCVVT_CONTROL_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * HW-1 FIX: phaser_id_t moved to g8ba_config.h (brought in by #include above).
 * Removing the definition here prevents duplicate-definition errors and
 * eliminates the need for dcvvt_hw.h to include this file.
 */

/** Direction convention for phase angles:
 *  Intake  phasers: positive = advance (open valves earlier)
 *  Exhaust phasers: positive = retard  (close valves later)
 */

/** PID controller state for a single phaser */
typedef struct {
    float   kp, ki, kd;           /* gains                                */
    float   integrator;           /* accumulated I term                   */
    float   prev_error;           /* previous error for D term            */
    float   output_clamp_max;     /* max duty output (%)                  */
    float   output_clamp_min;     /* min duty output (%)                  */
    float   integrator_max;       /* anti-windup integrator limit         */
} pid_state_t;

/** Single phaser (cam actuator) state */
typedef struct {
    phaser_id_t id;
    floatdeg_t  target_deg;       /* commanded cam phase (°CA)           */
    floatdeg_t  actual_deg;       /* measured cam phase from cam sensor   */
    floatdeg_t  error_deg;        /* target − actual                      */
    float       ocv_duty_pct;     /* OCV solenoid PWM duty (0–100%)       */
    pid_state_t pid;              /* PID controller state                 */
    bool        at_target;        /* within G8BA_CVVT_DEADBAND_DEG        */
    bool        sensor_valid;     /* cam sensor feedback available        */
    uint32_t    hold_lock_count;  /* consecutive cycles at target         */
} phaser_state_t;

/** Module-level CVVT status */
typedef struct {
    phaser_state_t  phasers[PHASER_COUNT];
    bool            enabled;          /* module active (CLT/RPM conditions)*/
    float           oil_temp_c;       /* oil temp for enable logic          */
    float           clt_c;            /* coolant temp for enable logic      */
    rpm_t           rpm;              /* last known RPM for table lookup    */
    float           load_pct;         /* last known load for table lookup   */
    uint32_t        update_count;
} cvvt_status_t;

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

extern volatile cvvt_status_t g_cvvt;

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise D-CVVT module.
 *         Sets up 4 PWM channels for OCV solenoids,
 *         loads target tables from flash, initialises PID states.
 * @return G8BA_OK on success.
 */
g8ba_status_t dcvvt_init(void);

/**
 * @brief  Main CVVT control update — call every TASK_PERIOD_FAST_MS (5ms).
 *         1. Reads cam phase measurements from crank_cam_sync module.
 *         2. Looks up target phases from tables.
 *         3. Runs PID for each phaser.
 *         4. Writes OCV duty cycles to PWM outputs.
 * @param  rpm       Current RPM
 * @param  load_pct  Engine load (%)
 * @param  oil_temp  Oil temperature (°C)
 * @param  clt_c     Coolant temperature (°C) — CVVT disabled below 60 °C
 */
void dcvvt_update(rpm_t rpm, float load_pct, float oil_temp, float clt_c);

/**
 * @brief  Directly command a target phase for a specific phaser.
 *         Overrides table lookup (use for calibration/testing).
 * @param  phaser      Which phaser to command
 * @param  target_deg  Desired phase in °CA
 */
void dcvvt_set_target(phaser_id_t phaser, floatdeg_t target_deg);

/**
 * @brief  Park all phasers to 0° (home position) — call on shutdown/stall.
 *         Sets OCV duty to 0% and waits for mechanical return.
 */
void dcvvt_park_all(void);

/**
 * @brief  Look up target intake cam phase from the VVT target table.
 * @param  phaser   Phaser ID
 * @param  rpm      Engine speed
 * @param  load_pct Engine load
 * @return Target phase in °CA
 */
floatdeg_t dcvvt_lookup_target(phaser_id_t phaser, rpm_t rpm, float load_pct);

/**
 * @brief  Return true if all phasers are at their commanded positions.
 */
bool dcvvt_all_at_target(void);

/**
 * @brief  Return true if CVVT system is enabled for current conditions.
 *         Disabled if: CLT < 60°C, oil temp < 50°C, RPM < 500.
 */
bool dcvvt_is_enabled(void);

/**
 * @brief  Get current phase measurement for specified phaser.
 * @param  phaser  Phaser ID
 * @return Actual cam phase (°CA), NaN if sensor invalid
 */
floatdeg_t dcvvt_get_actual_phase(phaser_id_t phaser);

/**
 * @brief  Emergency hold — maintain current OCV duty, freeze PID.
 *         Called if cam sensor fails mid-run.
 * @param  phaser  Phaser to hold
 */
void dcvvt_hold(phaser_id_t phaser);

/**
 * @brief  Reset PID integrator for a phaser (anti-windup recovery).
 * @param  phaser  Phaser ID
 */
void dcvvt_pid_reset(phaser_id_t phaser);

#endif /* DCVVT_CONTROL_H */
