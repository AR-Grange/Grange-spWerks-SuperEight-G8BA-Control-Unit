/**
 * @file    boost_control.h
 * @brief   Module 7 - Boost Control for Vortech V-7 YSi-Trim (SC variant)
 *
 * Centrifugal supercharger boost regulation:
 *   - Geometric peak boost is set by pulley ratio (mechanical, not electronic)
 *   - The bypass-control valve (BCV) bleeds excess pressurised charge upstream
 *     of the impeller when the closed-loop PID determines target >= measured
 *   - BCV is a normally-OPEN solenoid driven by PWM at G8BA_BCV_PWM_HZ (40 Hz)
 *       0 % duty   = fully OPEN  (no boost)
 *     100 % duty   = fully CLOSED (full geometric boost - never commanded;
 *                                   we cap at G8BA_BCV_DUTY_MAX_PCT = 95 %)
 *
 * Architecture:
 *   - Boost target table (RPM x TPS%) - selects desired MAP under given load
 *   - PID loop runs in thd_medium_ctrl (10 ms) - lower bandwidth than fuel/ign
 *     because BCV mechanical response is ~25 ms
 *   - boost_safety_check() runs in thd_slow_ctrl (50 ms) and trips
 *     PROT_EVENT_BOOST_OVERSHOOT / PROT_EVENT_BOOST_HARDCUT when MAP exceeds
 *     G8BA_BOOST_OVERSHOOT_KPA / G8BA_BOOST_MAX_KPA respectively
 *
 * Failsafe behaviour:
 *   - On any sensor fault (NaN/Inf MAP), BCV is parked OPEN (0 %) -> no boost
 *   - On boost overshoot, BCV is opened immediately (overrides PID) and
 *     fuel_cut_set(FCUT_BOOST_OVERSHOOT) is asserted
 *   - On crank-sync loss, boost_park() is called from prot_emergency_cut()
 */

#ifndef BOOST_CONTROL_H
#define BOOST_CONTROL_H

#include "g8ba_config.h"

/* ==========================================================================
 * DATA TYPES
 * ========================================================================== */

/** Boost-control mode */
typedef enum {
    BOOST_MODE_OFF      = 0,   /**< BCV fully open - no boost (e.g. cold engine) */
    BOOST_MODE_OPEN_LOOP,      /**< Map-based duty (no PID), used at low RPM     */
    BOOST_MODE_CLOSED_LOOP,    /**< PID actively regulating MAP                  */
    BOOST_MODE_FAULT,          /**< Sensor or overshoot fault - BCV parked open  */
} boost_mode_t;

/** Diagnostic snapshot of boost-control loop */
typedef struct {
    float        map_kpa;          /**< Last measured MAP                       */
    float        target_kpa;       /**< Current target from table               */
    float        error_kpa;        /**< target - measured                       */
    float        bcv_duty_pct;     /**< Commanded BCV duty (0-95 %)             */
    float        pid_p_term;       /**< Diagnostic                              */
    float        pid_i_term;       /**< Diagnostic                              */
    float        pid_d_term;       /**< Diagnostic                              */
    boost_mode_t mode;
    uint16_t     overshoot_ticks;  /**< Counter for persistent-overshoot trip   */
    uint32_t     pid_steps;        /**< Total PID iterations                    */
    bool         hard_cut_active;  /**< true if MAP >= G8BA_BOOST_MAX_KPA        */
} boost_status_t;

/* ==========================================================================
 * MODULE STATE  (read-only external access)
 * ========================================================================== */

extern volatile boost_status_t g_boost;

/* ==========================================================================
 * PUBLIC API
 * ========================================================================== */

/**
 * @brief  Initialise boost-control module.
 *         Configures BCV PWM hardware, validates target table axes,
 *         resets PID state, parks BCV open.
 * @return G8BA_OK on success.
 */
g8ba_status_t boost_init(void);

/**
 * @brief  Look up boost target (kPa absolute) from RPM x TPS table.
 *         Public for unit testing and diagnostics.
 * @param  rpm       Engine speed
 * @param  tps_pct   Throttle position (%)
 * @return Target MAP in kPa absolute (clamped to [100, G8BA_BOOST_TARGET_KPA])
 */
float boost_get_target_kpa(rpm_t rpm, float tps_pct);

/**
 * @brief  Run the boost-control PID step.
 *         Call from thd_medium_ctrl every TASK_PERIOD_MEDIUM_MS.
 *         Updates BCV duty cycle.  Internally selects open-loop vs closed-loop.
 *
 * @param  rpm                Current engine speed
 * @param  tps_pct            Current throttle position
 * @param  map_kpa            Current MAP measurement (absolute, kPa)
 * @param  clt_c              Current coolant temperature (degC) - boost gated below 50degC
 * @param  sync_ok            Crank/cam sync OK flag - boost is forced OFF if false
 * @param  engine_protect_cut true if any engine-protection HARD cut is active
 *                            (CLT_CUT, LOW_OIL, SYNC_LOST) - forces BCV park
 */
void boost_update(rpm_t rpm, float tps_pct, float map_kpa,
                  float clt_c, bool sync_ok, bool engine_protect_cut);

/** Minimum CLT (degC) below which boost is forced OFF (cold-engine protection) */
#define BOOST_CLT_ENABLE_C    50.0f

/** Minimum plausible MAP (kPa) when engine is running - below this is sensor fault */
#define BOOST_MAP_PLAUSIBLE_MIN_KPA   30.0f

/** RPM above which the MAP-plausibility check is enforced (below = cranking, no fault) */
#define BOOST_MAP_PLAUSIBLE_RPM       1000u

/** Persistent ticks of MAP-implausible reading before fuel cut is asserted */
#define BOOST_MAP_FAULT_TICKS         3u   /* 3 x 10 ms = 30 ms */

/**
 * @brief  Check for boost overshoot - call from thd_slow_ctrl every 50 ms.
 *         Maintains a persistence counter; trips fuel-cut when overshoot
 *         lasts >= G8BA_BOOST_OVER_TICKS slow ticks.
 *
 *         If MAP >= G8BA_BOOST_MAX_KPA at any single check, fuel_cut_set
 *         (FCUT_BOOST_OVERSHOOT) is asserted IMMEDIATELY (no debounce).
 *
 * @param  map_kpa  Current MAP measurement
 * @return true if a hard cut is active (MAP > G8BA_BOOST_MAX_KPA),
 *         false otherwise.
 */
bool boost_safety_check(float map_kpa);

/**
 * @brief  Force BCV fully open (0 % duty) and set BOOST_MODE_OFF.
 *         Called from emergency paths (sync loss, engine stall).
 */
void boost_park(void);

/**
 * @brief  Return current BCV commanded duty (%).
 */
float boost_get_bcv_duty(void);

/**
 * @brief  Return current boost mode.
 */
boost_mode_t boost_get_mode(void);

#endif /* BOOST_CONTROL_H */
