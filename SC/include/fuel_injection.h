/**
 * @file    fuel_injection.h
 * @brief   Module 2 — Fuel Injection Control
 *
 * Sequential MPI (Multi-Point Injection) for 8-cylinder G8BA.
 * Each cylinder injector is fired once per 720° cycle in firing-order sequence.
 *
 * Pulse width is computed from:
 *   PW = (VE × MAP × IAT_correction × BASE_PW) + INJ_DEAD_TIME
 * where BASE_PW targets stoichiometry (λ=1.0) at the given operating point.
 *
 * Acceleration enrichment, warm-up enrichment, and overrun cut are
 * implemented as multiplicative/additive correction factors.
 */

#ifndef FUEL_INJECTION_H
#define FUEL_INJECTION_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Fuel cut conditions (bit-field) */
typedef enum {
    FCUT_NONE             = 0x00,
    FCUT_OVERRUN          = 0x01,   /* decel fuel cut                          */
    FCUT_REV_LIMIT        = 0x02,   /* hard rev limit                          */
    FCUT_OVERTEMP         = 0x04,   /* CLT protection                          */
    FCUT_LOW_OIL          = 0x08,   /* oil pressure protection                 */
    FCUT_SYNC_LOST        = 0x10,   /* crank/cam sync lost                     */
    FCUT_SOFT_CUT         = 0x20,   /* alternating soft rev cut                */
    FCUT_BOOST_OVERSHOOT  = 0x40,   /* SC: MAP overshoot — boost protection    */
} fuel_cut_flags_t;

/** Per-cylinder fuel state */
typedef struct {
    uint8_t     cyl_index;     /* 0-7                                     */
    us_t        pulse_width_us;/* commanded injection pulse width          */
    us_t        inj_start_us;  /* scheduled injection start timestamp      */
    floatdeg_t  inj_angle;     /* °CA BTDC at which injection starts       */
    bool        active;        /* injector currently open                  */
} injector_state_t;

/** Fuel calculation inputs snapshot */
typedef struct {
    float       map_kpa;       /* manifold absolute pressure               */
    float       iat_c;         /* intake air temperature (°C)              */
    float       clt_c;         /* coolant temperature (°C)                 */
    float       tps_pct;       /* throttle position (%)                    */
    float       tps_dot;       /* TPS rate of change (%/s)                 */
    float       lambda;        /* current lambda (from wideband O2)        */
    float       lambda_target; /* target lambda                            */
    float       vbatt_v;       /* battery voltage (V) — FC-3 FIX: dead-time compensation */
    rpm_t       rpm;
    float       ve_pct;        /* volumetric efficiency from VE table (%)  */
} fuel_inputs_t;

/** Fuel correction factors */
typedef struct {
    float       warmup;        /* coolant-based warm-up enrichment (×)     */
    float       iat;           /* IAT charge density correction (×)        */
    float       accel;         /* acceleration enrichment (×, ≥1.0)        */
    float       closed_loop;   /* lambda closed-loop PID correction (×)    */
    float       baro;          /* barometric pressure compensation (×)     */
    float       total;         /* product of all corrections               */
} fuel_corrections_t;

/** Module-level fuel status */
typedef struct {
    injector_state_t    injectors[G8BA_CYLINDERS];
    fuel_inputs_t       inputs;
    fuel_corrections_t  corrections;
    float               base_pw_us;     /* base pulse width before corrections */
    float               inj_dc_pct;     /* injector duty cycle %              */
    fuel_cut_flags_t    cut_flags;      /* active fuel cut reasons            */
    uint32_t            inj_events;     /* total injection event counter       */
} fuel_status_t;

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

extern volatile fuel_status_t g_fuel;

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise fuel injection module.
 *         Sets up injector outputs, loads VE table from flash,
 *         registers cylinder event callbacks with RusEFI scheduler.
 * @return G8BA_OK on success.
 */
g8ba_status_t fuel_injection_init(void);

/**
 * @brief  Called once per engine cycle (every 720°) by the scheduler task.
 *         Re-calculates all injector pulse widths for the upcoming cycle.
 * @param  inputs  Fresh sensor snapshot
 */
void fuel_calc_cycle(const fuel_inputs_t *inputs);

/**
 * @brief  Schedule injection event for a specific cylinder.
 *         Called from crank angle scheduler when cylinder event fires.
 * @param  cyl_index  0-based cylinder index
 */
void fuel_schedule_injector(uint8_t cyl_index);

/**
 * @brief  Set a fuel cut condition flag.
 * @param  flag  Reason for cut
 */
void fuel_cut_set(fuel_cut_flags_t flag);

/**
 * @brief  Clear a fuel cut condition flag.
 * @param  flag  Reason for cut to clear
 */
void fuel_cut_clear(fuel_cut_flags_t flag);

/**
 * @brief  Return true if any fuel cut is active.
 */
bool fuel_is_cut(void);

/**
 * @brief  Update acceleration enrichment — call every 10ms with new TPS.
 * @param  tps_pct  Current TPS (%)
 * @param  dt_ms    Elapsed time since last call (ms)
 */
void fuel_accel_update(float tps_pct, float dt_ms);

/**
 * @brief  Closed-loop lambda PID step — call every 100ms.
 * @param  lambda_measured  Wideband O2 reading
 * @param  lambda_target    Target lambda
 */
void fuel_lambda_pid_step(float lambda_measured, float lambda_target);

/**
 * @brief  Return current injector duty cycle for cylinder `cyl` (%).
 */
float fuel_get_duty_cycle(uint8_t cyl_index);

/**
 * @brief  Return calculated AFR target for current operating point.
 */
float fuel_get_afr_target(void);

#endif /* FUEL_INJECTION_H */
