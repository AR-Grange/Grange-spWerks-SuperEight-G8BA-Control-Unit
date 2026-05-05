/**
 * @file    g8ba_config.h
 * @brief   Hyundai Tau 4.6 G8BA V8 — Master Engine Configuration (SC variant)
 *
 * Hardware: RusEFI Proteus V0.4+
 * RTOS:     ChibiOS/RT
 * Target:   STM32H743 (Proteus)
 *
 * Forced-induction setup:
 *   Supercharger: Vortech V-7 YSi-Trim (centrifugal, gear-driven)
 *   Intercooler : Air-to-water, front-mount heat exchanger
 *   Boost target: ~7 PSI peak (≈ 148 kPa absolute) — calibrated on dyno
 *   Boost cut   : 250 kPa absolute (G8BA_BOOST_MAX_KPA) — hardware safe limit
 *   Pistons     : Forged, dished, 9.0:1 static CR (was 10.4:1 NA)
 *   Injectors   : 850 cc/min @ 300 kPa (was 650 cc/min NA)
 *   Fuel        : 98 RON pump (E85 capable with separate calibration)
 *
 * Firing order : 1-2-7-8-4-5-6-3
 * Trigger wheel: 36-2 VR on crankshaft
 * Cam sensors  : Hall effect × 4 (B1 IN, B1 EX, B2 IN, B2 EX)
 */

#ifndef G8BA_CONFIG_H
#define G8BA_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* ── RusEFI / ChibiOS includes ─────────────────────────────────────────── */
#include "ch.h"           /* ChibiOS RTOS core          */
#include "hal.h"          /* ChibiOS HAL                */
#include "efi_gpio.h"     /* RusEFI GPIO abstraction    */
#include "sensor.h"       /* RusEFI sensor API          */
#include "engine.h"       /* RusEFI engine object       */

/* ══════════════════════════════════════════════════════════════════════════
 * BUILD VARIANT IDENTIFIER
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_VARIANT_SC                /**< Compiled with supercharger support */
#define G8BA_VARIANT_NAME    "G8BA-SC-Vortech-V7-YSi"

/* ══════════════════════════════════════════════════════════════════════════
 * ENGINE MECHANICAL CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_CYLINDERS          8
#define G8BA_DISPLACEMENT_CC    4627u          /* cm³                        */
#define G8BA_BORE_MM            92.0f          /* mm                         */
#define G8BA_STROKE_MM          87.0f          /* mm                         */
/*
 * SC-1: compression ratio LOWERED from 10.4 → 9.0 (forged dished pistons).
 * Required for safe boost operation on 98 RON.  Mismatch with the actual
 * installed pistons will cause severe knock under boost.
 */
#define G8BA_COMPRESSION_RATIO  9.0f
#define G8BA_CRANK_ANGLE_CYCLE  720.0f         /* degrees per full cycle     */
#define G8BA_FIRING_INTERVAL    90.0f          /* °CA between events (720/8) */

/**
 * Firing order encoded as 0-based cylinder indices.
 * Physical cylinders: 1-2-7-8-4-5-6-3 → indices: 0,1,6,7,3,4,5,2
 * Bank 1: cyl 1,2,3,4  (indices 0-3)
 * Bank 2: cyl 5,6,7,8  (indices 4-7)
 */
#define G8BA_FIRING_ORDER       {0u, 1u, 6u, 7u, 3u, 4u, 5u, 2u}
#define G8BA_FIRING_ORDER_STR   "1-2-7-8-4-5-6-3"

/* ══════════════════════════════════════════════════════════════════════════
 * TRIGGER WHEEL — 36-2 VR CRANK
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_TRIGGER_TEETH_TOTAL    36u        /* nominal tooth count        */
#define G8BA_TRIGGER_TEETH_MISSING  2u         /* missing teeth for sync     */
#define G8BA_TRIGGER_TEETH_PHYSICAL 34u        /* actual teeth present       */
#define G8BA_TRIGGER_TOOTH_ANGLE    (360.0f / G8BA_TRIGGER_TEETH_TOTAL)  /* 10° */
#define G8BA_TRIGGER_GAP_ANGLE      (G8BA_TRIGGER_TOOTH_ANGLE * (G8BA_TRIGGER_TEETH_MISSING + 1u))

/** Crank angle of cylinder #1 TDC compression stroke (calibrate on dyno) */
#define G8BA_TDC_CYL1_OFFSET_DEG   114.0f

/* ══════════════════════════════════════════════════════════════════════════
 * REV LIMITS  (SC-2: lowered to protect rotating assembly under boost loads)
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_RPM_MAX                7500u      /* hard rev limit (rpm)       */
#define G8BA_RPM_SOFT_CUT           7300u      /* soft cut start (rpm)       */
#define G8BA_RPM_IDLE_TARGET        850u       /* warm idle target (slightly raised for SC drag) */
#define G8BA_RPM_CRANK              400u       /* cranking threshold (rpm)   */

/* ══════════════════════════════════════════════════════════════════════════
 * FUEL SYSTEM  (SC-3: injectors upsized 650 → 850 cc/min)
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_INJECTOR_FLOW_CC_MIN   850.0f     /* cc/min @ 300 kPa — bigger to feed boost */
#define G8BA_FUEL_PRESSURE_KPA      300.0f     /* rail pressure (kPa gauge)  */
#define G8BA_STOICH_AFR             14.7f      /* lambda 1.0 for gasoline    */
#define G8BA_INJ_DEAD_TIME_US       650u       /* injector dead time (µs) — placeholder, see fuel_map dead-time table */
#define G8BA_INJ_MIN_PW_US          900u       /* min pulse width — bigger injector → lift the floor slightly */
#define G8BA_INJ_MAX_PW_US          25000u     /* maximum pulse width (µs)   */

/* ══════════════════════════════════════════════════════════════════════════
 * IGNITION SYSTEM
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_IGN_COIL_DWELL_MS      3.5f       /* coil dwell time (ms)       */
/*
 * SC-4: Maximum advance LOWERED from 45° → 35° (boost dramatically narrows
 * the safe MBT window).  ignition_map base table has been re-derived for SC.
 */
#define G8BA_IGN_ADVANCE_MAX        35.0f      /* max advance (°BTDC)        */
#define G8BA_IGN_ADVANCE_CRANK      5.0f       /* cranking advance (°BTDC)   */
#define G8BA_IGN_ADVANCE_IDLE       12.0f      /* idle advance (°BTDC)       */

/* ══════════════════════════════════════════════════════════════════════════
 * D-CVVT PARAMETERS
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_CVVT_BANKS             2u         /* Bank1, Bank2               */
#define G8BA_CVVT_CAMS_PER_BANK     2u         /* intake + exhaust           */
#define G8BA_CVVT_TOTAL_ACTUATORS   4u         /* 4 VVT solenoids            */
#define G8BA_CVVT_PWM_HZ            250u       /* solenoid PWM frequency     */
/*
 * SC-5: Intake max advance REDUCED from 50° → 35° to limit valve overlap
 * under boost (overlap pushes pressurised charge straight out the exhaust →
 * lost boost, lost cylinder fill, raw HC out the tailpipe).
 */
#define G8BA_CVVT_IN_ADVANCE_MAX    35.0f      /* intake max advance (°CA)   */
#define G8BA_CVVT_EX_RETARD_MAX     30.0f      /* exhaust max retard (°CA)   */
#define G8BA_CVVT_DEADBAND_DEG      1.5f       /* position deadband (°CA)    */
#define G8BA_CVVT_KP                3.5f       /* PID proportional gain      */
#define G8BA_CVVT_KI                0.08f      /* PID integral gain          */
#define G8BA_CVVT_KD                0.30f      /* PID derivative gain        */

/* ══════════════════════════════════════════════════════════════════════════
 * KNOCK CONTROL  (SC-6: more aggressive — boost is unforgiving on detonation)
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_KNOCK_SENSORS          2u         /* one per bank               */
#define G8BA_KNOCK_FREQ_HZ          6800u      /* characteristic frequency   */
#define G8BA_KNOCK_RETARD_STEP      2.5f       /* deg/event (was 1.5 NA) — pull harder */
#define G8BA_KNOCK_ADVANCE_STEP     0.25f      /* deg/clean (was 0.3 NA) — recover slower */
#define G8BA_KNOCK_RETARD_MAX       12.0f      /* max total retard (was 10 NA) */
#define G8BA_KNOCK_WINDOW_ATDC      8.0f       /* window start (was 10 NA) — earlier capture */
#define G8BA_KNOCK_WINDOW_END       65.0f      /* window end (was 60 NA)     */

/* ══════════════════════════════════════════════════════════════════════════
 * BOOST CONTROL — Vortech V-7 YSi-Trim                                  (SC-7 NEW)
 * ══════════════════════════════════════════════════════════════════════════
 *
 * The V-7 YSi is a centrifugal head; pulley-ratio sets the geometric peak
 * boost, electronic control trims via a bypass valve (BCV) that bleeds
 * pressurised charge back upstream of the impeller when target is met.
 * BCV is a normally-OPEN solenoid: 0 % duty = fully open (no boost),
 * 100 % duty = fully closed (full geometric boost).
 *
 * Boost target table (RPM × TPS%) is in boost_control.c.  This file owns
 * only the safety limits and PID gains.
 */

#define G8BA_BOOST_TARGET_KPA       148.0f     /* nominal target (≈ 7 PSI absolute) */
#define G8BA_BOOST_OVERSHOOT_KPA    180.0f     /* triggers protection (≈ 11.5 PSI)  */
#define G8BA_BOOST_MAX_KPA          250.0f     /* hard cut (≈ 21.7 PSI)             */

/** PID gains for boost-control PID on the BCV duty cycle.
 *  Tuned conservatively — start here, tune on dyno. */
#define G8BA_BOOST_KP               1.20f      /* %duty per kPa error        */
#define G8BA_BOOST_KI               0.08f      /* integral, %duty/(kPa·s)    */
#define G8BA_BOOST_KD               0.40f      /* derivative                 */

/** BCV PWM frequency and duty clamps */
#define G8BA_BCV_PWM_HZ             40u        /* solenoid mech response ≈ 25 ms */
#define G8BA_BCV_DUTY_MIN_PCT       0.0f
#define G8BA_BCV_DUTY_MAX_PCT       95.0f      /* leave 5% headroom — never fully closed */

/** Boost-control authority gate: only active above this RPM (no boost off-idle) */
#define G8BA_BOOST_ENABLE_RPM       2500u

/** Boost overshoot fault: if MAP > OVERSHOOT_KPA for > this many ticks (50 ms),
 *  raise PROT_EVENT_BOOST_OVERSHOOT and start fuel/ign trim. */
#define G8BA_BOOST_OVER_TICKS       4u         /* 4 × 50 ms = 200 ms persistent overshoot */

/* ══════════════════════════════════════════════════════════════════════════
 * ENGINE PROTECTION THRESHOLDS  (some tightened for SC duty)
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_CLT_WARN_C             100.0f     /* coolant warning (was 105 NA) */
#define G8BA_CLT_PROTECT_C          108.0f     /* power reduction (was 112 NA) */
#define G8BA_CLT_CUTOFF_C           115.0f     /* fuel cut (was 118 NA)        */
#define G8BA_OIL_PRESS_MIN_KPA      180.0f     /* min oil pressure (kPa) — raised for SC bearing load */
#define G8BA_OIL_PRESS_WARN_KPA     230.0f     /* oil press warning (kPa)    */
#define G8BA_OIL_TEMP_WARN_C        125.0f     /* oil temp warning (was 130 NA) */
#define G8BA_OIL_TEMP_MAX_C         140.0f     /* oil temp cutoff (was 145 NA)  */

/* ══════════════════════════════════════════════════════════════════════════
 * INTERCOOLER (informational — not directly read by control code)
 * ══════════════════════════════════════════════════════════════════════════ */

#define G8BA_IAT_BOOST_LIMIT_C      65.0f      /* IAT above this raises PROT_EVENT_IAT_HOT */

/* ══════════════════════════════════════════════════════════════════════════
 * TASK SCHEDULING (ChibiOS thread periods)
 * ══════════════════════════════════════════════════════════════════════════ */

#define TASK_PERIOD_FAST_MS         5u         /* 200 Hz — knock/CVVT ctrl   */
#define TASK_PERIOD_MEDIUM_MS       10u        /* 100 Hz — fuel/ign update + boost PID */
#define TASK_PERIOD_SLOW_MS         50u        /* 20 Hz  — protection/diag   */
#define TASK_PERIOD_VERYLOW_MS      250u       /* 4 Hz   — logging/telemetry */

/* Thread stack sizes — 1 kB minimum for threads with FPU + function calls */
#define STACK_CRANK_SYNC            1024u
#define STACK_FUEL_INJ              1024u
#define STACK_IGNITION              1024u
#define STACK_CVVT                  1024u
#define STACK_KNOCK                 1024u
#define STACK_PROTECTION            768u
#define STACK_BOOST                 768u       /* SC-7: boost-control thread shares stack family */

/* ══════════════════════════════════════════════════════════════════════════
 * FUEL CALCULATION PHYSICAL CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

#define AIR_DENSITY_REF_G_CC        0.001204f
#define FUEL_DENSITY_G_CC           0.720f
#define MAP_REF_KPA                 100.0f

/* ══════════════════════════════════════════════════════════════════════════
 * CAM PHASE CONFIRMATION PARAMETERS
 * ══════════════════════════════════════════════════════════════════════════ */

#define CAM_PHASE_CONFIRM_WINDOW_DEG  30.0f
#define CAM_PHASE_MIN_CONFIRMS        2u

/* ══════════════════════════════════════════════════════════════════════════
 * COMMON TYPE ALIASES
 * ══════════════════════════════════════════════════════════════════════════ */

typedef float   floatdeg_t;
typedef float   floatms_t;
typedef float   floatv_t;
typedef uint32_t rpm_t;
typedef uint32_t us_t;

/** Standardized return codes for all G8BA modules */
typedef enum {
    G8BA_OK           = 0,
    G8BA_ERR_SENSOR   = -1,
    G8BA_ERR_RANGE    = -2,
    G8BA_ERR_TIMEOUT  = -3,
    G8BA_ERR_SYNC     = -4,
    G8BA_ERR_OVERTEMP = -5,
    G8BA_ERR_LOWPRESS = -6,
    G8BA_ERR_HW       = -7,
    G8BA_ERR_BOOST    = -8,   /* SC-7: boost-overshoot or BCV fault */
} g8ba_status_t;

/** D-CVVT oil-control-valve channel identifiers (match cam_id_t 0-based) */
typedef enum {
    PHASER_B1_INTAKE  = 0,
    PHASER_B1_EXHAUST = 1,
    PHASER_B2_INTAKE  = 2,
    PHASER_B2_EXHAUST = 3,
    PHASER_COUNT      = 4,
} phaser_id_t;

#endif /* G8BA_CONFIG_H */
