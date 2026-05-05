/**
 * @file    fuel_map.h
 * @brief   Fuel Calibration Tables — G8BA + Vortech V-7 YSi-Trim (SC variant)
 *
 * This module owns all fuel calibration data and lookup functions.
 * fuel_injection.c calls into this module for VE lookups and corrections.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Hardware baseline (SC)
 * ──────────────────────────────────────────────────────────────────────────
 *   Injectors  : 850 cc/min @ 300 kPa (was 650 cc/min NA)
 *   Fuel press : 300 kPa gauge (rail) — no FPR upgrade
 *   Fuel type  : 98 RON unleaded (stoich 14.7:1)
 *   Target λ   : 0.95 cruise / 0.83 WOT under boost (richer than NA — cooling)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Base pulse-width derivation (FUELMAP_BASE_PW_MS)  [SC re-derived]
 * ──────────────────────────────────────────────────────────────────────────
 *   At: 100 % VE, 100 kPa MAP, 20 °C IAT, λ=1.00, 850 cc/min injectors
 *
 *   Vcyl       = 4627 / 8 = 578.375 cc
 *   air_mass   = 578.375 × 0.001204 = 0.6965 g
 *   fuel_mass  = 0.6965 / 14.7      = 0.04737 g
 *   fuel_vol   = 0.04737 / 0.720    = 0.06580 cc
 *
 *   inj_flow   = 850 cc/min = 850/60000 cc/ms = 0.014167 cc/ms
 *   BASE_PW    = 0.06580 / 0.014167 ≈ 4.645 ms   (was 6.074 ms with 650 cc/min)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Pulse-width formula (in fuel_map_calc)
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   PW = BASE_PW × (VE/100) × (MAP/MAP_ref) × CLT_corr × IAT_corr
 *                × λ_corr   + dead_time_ms
 *
 *   With MAP/MAP_ref now scaling beyond 1.0 (boost), the same formula
 *   handles forced-induction operating points seamlessly.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Table axes  [SC: MAP axis EXTENDED from 105 kPa → 250 kPa]
 * ──────────────────────────────────────────────────────────────────────────
 *   RPM  : 16 points — 600 … 7500
 *   MAP  : 14 points — 30  … 250 kPa  (NA had 10 points 20…105)
 *   VBatt: 9 points  — 8.0 … 16.0 V
 *   CLT  : 13 points — −20 … 100 °C
 *   IAT  : 13 points — −20 … 100 °C  (extended for hot post-SC charge)
 */

#ifndef FUEL_MAP_H
#define FUEL_MAP_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE DIMENSIONS
 * ══════════════════════════════════════════════════════════════════════════ */

#define FUELMAP_RPM_POINTS     16u
#define FUELMAP_MAP_POINTS     14u            /* SC: was 10 */
#define FUELMAP_VBATT_POINTS    9u
#define FUELMAP_CLT_POINTS     13u
#define FUELMAP_IAT_POINTS     13u            /* SC: was 11 (axis extended to 100°C) */

/* ══════════════════════════════════════════════════════════════════════════
 * INJECTOR CONSTANTS (850 cc/min racing injectors — SC upgrade)
 * ══════════════════════════════════════════════════════════════════════════ */

#define FUELMAP_INJ_FLOW_CC_MIN     850.0f
#define FUELMAP_INJ_FLOW_CC_MS      (FUELMAP_INJ_FLOW_CC_MIN / 60000.0f)

/**
 * Base pulse width (ms) at 100 % VE, 100 kPa MAP, 20 °C IAT, λ=1.00.
 * SC re-derivation: 4.645 ms (was 6.074 ms NA with 650 cc/min).
 */
#define FUELMAP_BASE_PW_MS          4.645f

/**
 * Minimum effective open time (ms) — bigger injectors have a longer
 * non-linear region; raise the floor accordingly.
 *   FUELMAP_MIN_OPEN_MS(0.7) + max_dead_time(2.2 ms) = 2.9 ms > 0.9 ms ✓
 *   (G8BA_INJ_MIN_PW_US is 900 µs in SC config)
 */
#define FUELMAP_MIN_OPEN_MS         0.7f

/**
 * Maximum pulse width cap (ms).  At redline (7500 RPM) one full 720°
 * cycle = 16 ms — leave margin so duty cycle stays below ~85 %.
 */
#define FUELMAP_MAX_PW_MS           14.0f

/* SC-FM-1: Compile-time guard — FUELMAP_MAX_PW_MS must stay within hardware. */
#if ((uint32_t)(FUELMAP_MAX_PW_MS * 1000.0f + 0.5f)) > G8BA_INJ_MAX_PW_US
#  error "FUELMAP_MAX_PW_MS exceeds G8BA_INJ_MAX_PW_US — update one of them"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct {
    float  rpm;
    float  map_kpa;
    float  vbatt_v;
    float  clt_c;
    float  iat_c;
    float  lambda_corr;
} fuel_map_inputs_t;

typedef struct {
    float  ve_pct;
    float  dead_time_ms;
    float  pw_cmd_ms;
    float  pw_effective_ms;
    float  clt_corr;
    float  iat_corr;
} fuel_map_result_t;

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE AXIS DECLARATIONS (defined in fuel_map.c)
 * ══════════════════════════════════════════════════════════════════════════ */

extern const uint16_t FUELMAP_RPM_AXIS[FUELMAP_RPM_POINTS];
extern const uint8_t  FUELMAP_MAP_AXIS[FUELMAP_MAP_POINTS];
extern const float    FUELMAP_VBATT_AXIS[FUELMAP_VBATT_POINTS];
extern const int8_t   FUELMAP_CLT_AXIS[FUELMAP_CLT_POINTS];
extern const int8_t   FUELMAP_IAT_AXIS[FUELMAP_IAT_POINTS];

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API  (signatures unchanged from NA — caller code stays the same)
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t fuel_map_init(void);
void          fuel_map_calc(const fuel_map_inputs_t *in, fuel_map_result_t *out);
float         fuel_map_get_ve(float rpm, float map_kpa);
float         fuel_map_get_dead_time_ms(float vbatt_v);
float         fuel_map_get_clt_correction(float clt_c);
float         fuel_map_get_iat_correction(float iat_c);

#endif /* FUEL_MAP_H */
