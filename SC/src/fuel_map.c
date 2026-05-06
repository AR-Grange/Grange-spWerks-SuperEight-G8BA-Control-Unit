/**
 * @file    fuel_map.c
 * @brief   Fuel Calibration Tables - G8BA + Vortech V-7 YSi-Trim (SC variant)
 *
 * --------------------------------------------------------------------------
 * SC calibration baseline (requires dyno verification before use)
 * --------------------------------------------------------------------------
 *   Engine    : Tau 4.6 G8BA V8, FORGED 9.0:1 dished pistons, MLS HG
 *   Blower    : Vortech V-7 YSi-Trim, 3.40" pulley -> ~7 PSI peak boost
 *   Cooler    : Air-to-water, ~70 degC IAT delta drop at peak load
 *   Fuel      : 98 RON premium pump fuel
 *   Injectors : 850 cc/min high-flow MPI, 300 kPa rail
 *   IAT ref   : 20 degC (293 K)
 *
 * VE TABLE VALUES BELOW BOOST (<= 100 kPa) are first-pass estimates carried
 * over from NA calibration with conservative scaling.  Boost rows
 * (>100 kPa) are estimated from typical centrifugal-SC behaviour:
 *   - VE rises from ~95 % (NA WOT) to ~110 % at 1.0 bar boost
 *   - Above ~180 kPa absolute, intercooler heat-soak and impeller efficiency
 *     loss begin to flatten VE around 105-108 %
 *   - Peak VE typically near 5000-5500 RPM under boost
 *
 * MUST be replaced with measured values from wide-band O2 logging on the
 * dyno before any sustained WOT operation.
 *
 * --------------------------------------------------------------------------
 * lambda targets (informational - applied via lambda_target in caller)
 * --------------------------------------------------------------------------
 *   Cruise / part-throttle (MAP < 90 kPa)         : lambda = 0.95-1.00
 *   Off-boost WOT (MAP 90-100 kPa)                : lambda = 0.88
 *   Light boost (MAP 100-140 kPa)                 : lambda = 0.85
 *   Full boost (MAP > 140 kPa)                    : lambda = 0.80-0.83  (cooling)
 */

#include "fuel_map.h"

/* ==========================================================================
 * TABLE AXES
 * ========================================================================== */

/** Engine speed axis (RPM) - 16 points; rev limit pulled to 7500 in SC */
const uint16_t FUELMAP_RPM_AXIS[FUELMAP_RPM_POINTS] = {
     600u, 800u, 1000u, 1500u, 2000u, 2500u, 3000u, 3500u,
    4000u, 4500u, 5000u, 5500u, 6000u, 6500u, 7000u, 7500u
};

/**
 * MAP axis (kPa absolute) - 14 points, 30...250 kPa.
 * Below 100 kPa: same density as NA table for low-load resolution.
 * 100...180 kPa: dense (10 kPa steps) - main boost operating region.
 * 180...250 kPa: coarser - high-boost region, mostly safety margin.
 */
const uint8_t FUELMAP_MAP_AXIS[FUELMAP_MAP_POINTS] = {
    30u, 50u, 70u, 90u, 100u,           /* off-boost  */
   110u, 120u, 130u, 140u, 150u,        /* light boost (~1.5-7 PSI gauge) */
   170u, 190u, 220u, 250u                /* full boost / safety           */
};

/** Battery voltage axis (V) for injector dead-time lookup - 9 points */
const float FUELMAP_VBATT_AXIS[FUELMAP_VBATT_POINTS] = {
    8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f
};

/** CLT axis (degC) for warm-up enrichment - 13 points */
const int8_t FUELMAP_CLT_AXIS[FUELMAP_CLT_POINTS] = {
    -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};

/** IAT axis (degC) - 13 points, EXTENDED to 100 degC for hot post-SC charge */
const int8_t FUELMAP_IAT_AXIS[FUELMAP_IAT_POINTS] = {
    -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};

/* ==========================================================================
 * VE TABLE  [16 RPM x 14 MAP]  - SC re-tuned with boost columns
 * ==========================================================================
 *
 * Units: percent (whole number).
 * MAP axis does NOT scale the values - MAP is applied separately in PW calc.
 *
 * Columns:  MAP  30  50  70  90 100 110 120 130 140 150 170 190 220 250 kPa
 * ========================================================================== */
static const uint8_t s_ve_table[FUELMAP_RPM_POINTS][FUELMAP_MAP_POINTS] = {
    /* RPM  600 */ { 76, 76, 77, 79, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80 }, /* idle, no boost */
    /* RPM  800 */ { 77, 77, 78, 80, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82 },
    /* RPM 1000 */ { 78, 78, 79, 82, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85 },
    /* RPM 1500 */ { 79, 79, 81, 85, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88 },
    /* RPM 2000 */ { 80, 80, 82, 87, 91, 92, 93, 93, 93, 92, 91, 90, 88, 86 }, /* boost barely active */
    /* RPM 2500 */ { 81, 82, 84, 89, 93, 95, 97, 98, 99, 99, 98, 97, 95, 92 }, /* boost building   */
    /* RPM 3000 */ { 82, 84, 87, 92, 96, 99,101,103,104,104,103,102, 99, 96 },
    /* RPM 3500 */ { 83, 86, 90, 94, 98,101,103,105,107,107,106,105,102, 99 },
    /* RPM 4000 */ { 84, 87, 91, 96, 99,102,105,107,108,109,108,107,104,100 },
    /* RPM 4500 */ { 85, 88, 92, 97,101,103,106,108,110,110,110,108,105,101 }, /* peak VE region */
    /* RPM 5000 */ { 85, 88, 92, 97,100,103,105,107,109,110,110,109,106,102 },
    /* RPM 5500 */ { 84, 87, 91, 96, 98,101,103,105,107,108,108,107,104,100 },
    /* RPM 6000 */ { 82, 85, 89, 93, 95, 98,100,102,104,105,105,104,101, 97 },
    /* RPM 6500 */ { 80, 83, 87, 90, 92, 95, 97, 99,101,102,102,101, 98, 94 },
    /* RPM 7000 */ { 78, 81, 85, 87, 89, 92, 94, 96, 98, 99, 99, 98, 95, 91 },
    /* RPM 7500 */ { 75, 78, 82, 84, 85, 88, 90, 92, 94, 95, 95, 94, 91, 87 }, /* limiter approaching */
};

/* ==========================================================================
 * INJECTOR DEAD-TIME TABLE  [9 points] - same physics as NA, larger absolute
 * values for the bigger injector solenoid.
 * ========================================================================== */
static const uint16_t s_dead_time_us[FUELMAP_VBATT_POINTS] = {
    /*  8.0 V */ 2400u,
    /*  9.0 V */ 1950u,
    /* 10.0 V */ 1550u,
    /* 11.0 V */ 1250u,
    /* 12.0 V */ 1000u,
    /* 13.0 V */  840u,
    /* 14.0 V */  720u,  /* typical running voltage */
    /* 15.0 V */  640u,
    /* 16.0 V */  580u,
};

/* ==========================================================================
 * CLT CORRECTION TABLE  [13 points]  (same as NA - coolant warm-up physics
 * is independent of induction type)
 * ========================================================================== */
static const uint8_t s_clt_corr_pct[FUELMAP_CLT_POINTS] = {
    /* -20 degC */ 140u,
    /* -10 degC */ 132u,
    /*   0 degC */ 125u,
    /*  10 degC */ 118u,
    /*  20 degC */ 112u,
    /*  30 degC */ 108u,
    /*  40 degC */ 105u,
    /*  50 degC */ 103u,
    /*  60 degC */ 102u,
    /*  70 degC */ 101u,
    /*  80 degC */ 100u,
    /*  90 degC */ 100u,
    /* 100 degC */ 100u,
};

/* ==========================================================================
 * IAT CORRECTION TABLE  [13 points] - extended to 100 degC
 *   corr(T) = round(29300 / (T_IAT + 273))    (293 K reference at 20 degC)
 * SC: post-intercooler IAT can briefly hit 80-95 degC on long pulls.
 * ========================================================================== */
static const uint8_t s_iat_corr_pct[FUELMAP_IAT_POINTS] = {
    /* -20 degC */ 116u,
    /* -10 degC */ 111u,
    /*   0 degC */ 107u,
    /*  10 degC */ 104u,
    /*  20 degC */ 100u,   /* REFERENCE POINT (matches BASE_PW)             */
    /*  30 degC */  97u,
    /*  40 degC */  94u,
    /*  50 degC */  91u,
    /*  60 degC */  88u,
    /*  70 degC */  85u,
    /*  80 degC */  83u,
    /*  90 degC */  81u,   /* SC region - IAT at sustained boost */
    /* 100 degC */  78u,
};

/* ==========================================================================
 * PRIVATE HELPERS
 * ========================================================================== */

static float lerp(float y0, float y1, float frac)
{
    return y0 + frac * (y1 - y0);
}

static float ve_bilinear(float rpm, float map_kpa)
{
    uint8_t ri = 0u, mi = 0u;

    for (uint8_t i = 0u; i < FUELMAP_RPM_POINTS - 1u; i++) {
        if (rpm < (float)FUELMAP_RPM_AXIS[i + 1u]) { ri = i; break; }
        ri = FUELMAP_RPM_POINTS - 2u;
    }

    for (uint8_t j = 0u; j < FUELMAP_MAP_POINTS - 1u; j++) {
        if (map_kpa < (float)FUELMAP_MAP_AXIS[j + 1u]) { mi = j; break; }
        mi = FUELMAP_MAP_POINTS - 2u;
    }

    float rpm_span = (float)(FUELMAP_RPM_AXIS[ri + 1u] - FUELMAP_RPM_AXIS[ri]);
    float map_span = (float)(FUELMAP_MAP_AXIS[mi + 1u] - FUELMAP_MAP_AXIS[mi]);

    float rfrac = 0.0f, mfrac = 0.0f;

    if (rpm_span > 0.0f) {
        rfrac = (rpm - (float)FUELMAP_RPM_AXIS[ri]) / rpm_span;
        if (rfrac < 0.0f) rfrac = 0.0f;
        if (rfrac > 1.0f) rfrac = 1.0f;
    }
    if (map_span > 0.0f) {
        mfrac = (map_kpa - (float)FUELMAP_MAP_AXIS[mi]) / map_span;
        if (mfrac < 0.0f) mfrac = 0.0f;
        if (mfrac > 1.0f) mfrac = 1.0f;
    }

    float v00 = (float)s_ve_table[ri    ][mi    ];
    float v10 = (float)s_ve_table[ri + 1u][mi    ];
    float v01 = (float)s_ve_table[ri    ][mi + 1u];
    float v11 = (float)s_ve_table[ri + 1u][mi + 1u];

    float lo = lerp(v00, v01, mfrac);
    float hi = lerp(v10, v11, mfrac);
    return lerp(lo, hi, rfrac);
}

static float interp1d_u16(const float *x_axis, const uint16_t *y_vals,
                           uint8_t count, float x)
{
    if (x <= x_axis[0]) return (float)y_vals[0];
    if (x >= x_axis[count - 1u]) return (float)y_vals[count - 1u];

    for (uint8_t i = 0u; i < count - 1u; i++) {
        if (x < x_axis[i + 1u]) {
            float span = x_axis[i + 1u] - x_axis[i];
            float frac = (span > 0.0f) ? ((x - x_axis[i]) / span) : 0.0f;
            return lerp((float)y_vals[i], (float)y_vals[i + 1u], frac);
        }
    }
    return (float)y_vals[count - 1u];
}

static float interp1d_corr(const int8_t *x_axis, const uint8_t *y_pct,
                             uint8_t count, float x)
{
    if (x <= (float)x_axis[0]) return (float)y_pct[0] * 0.01f;
    if (x >= (float)x_axis[count - 1u]) return (float)y_pct[count - 1u] * 0.01f;

    for (uint8_t i = 0u; i < count - 1u; i++) {
        float x0 = (float)x_axis[i];
        float x1 = (float)x_axis[i + 1u];
        if (x < x1) {
            float span = x1 - x0;
            float frac = (span > 0.0f) ? ((x - x0) / span) : 0.0f;
            return lerp((float)y_pct[i] * 0.01f,
                        (float)y_pct[i + 1u] * 0.01f, frac);
        }
    }
    return (float)y_pct[count - 1u] * 0.01f;
}

/* ==========================================================================
 * PUBLIC IMPLEMENTATION
 * ========================================================================== */

g8ba_status_t fuel_map_init(void)
{
    /* Axis monotonicity check - ALL builds (FM-2 retained from NA). */
    for (uint8_t i = 0u; i < FUELMAP_RPM_POINTS - 1u; i++) {
        if (FUELMAP_RPM_AXIS[i] >= FUELMAP_RPM_AXIS[i + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    for (uint8_t j = 0u; j < FUELMAP_MAP_POINTS - 1u; j++) {
        if (FUELMAP_MAP_AXIS[j] >= FUELMAP_MAP_AXIS[j + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    /* SC-FM-2: also validate IAT, CLT, and Vbatt axes (added because their
     * sizes changed from NA - a typo would silently corrupt enrichment). */
    for (uint8_t k = 0u; k < FUELMAP_CLT_POINTS - 1u; k++) {
        if (FUELMAP_CLT_AXIS[k] >= FUELMAP_CLT_AXIS[k + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    for (uint8_t k = 0u; k < FUELMAP_IAT_POINTS - 1u; k++) {
        if (FUELMAP_IAT_AXIS[k] >= FUELMAP_IAT_AXIS[k + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    for (uint8_t k = 0u; k < FUELMAP_VBATT_POINTS - 1u; k++) {
        if (FUELMAP_VBATT_AXIS[k] >= FUELMAP_VBATT_AXIS[k + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    return G8BA_OK;
}

float fuel_map_get_ve(float rpm, float map_kpa)
{
    return ve_bilinear(rpm, map_kpa);
}

float fuel_map_get_dead_time_ms(float vbatt_v)
{
    float dt_us = interp1d_u16(FUELMAP_VBATT_AXIS, s_dead_time_us,
                                FUELMAP_VBATT_POINTS, vbatt_v);
    return dt_us * 0.001f;
}

float fuel_map_get_clt_correction(float clt_c)
{
    return interp1d_corr(FUELMAP_CLT_AXIS, s_clt_corr_pct,
                          FUELMAP_CLT_POINTS, clt_c);
}

float fuel_map_get_iat_correction(float iat_c)
{
    return interp1d_corr(FUELMAP_IAT_AXIS, s_iat_corr_pct,
                          FUELMAP_IAT_POINTS, iat_c);
}

void fuel_map_calc(const fuel_map_inputs_t *in, fuel_map_result_t *out)
{
    if ((in == NULL) || (out == NULL)) return;

    float ve_pct = ve_bilinear(in->rpm, in->map_kpa);
    out->ve_pct  = ve_pct;

    float dt_ms       = fuel_map_get_dead_time_ms(in->vbatt_v);
    out->dead_time_ms = dt_ms;

    float clt_corr = fuel_map_get_clt_correction(in->clt_c);
    float iat_corr = fuel_map_get_iat_correction(in->iat_c);
    out->clt_corr  = clt_corr;
    out->iat_corr  = iat_corr;

    /* Lambda correction allowed range - same band as NA */
    float lambda_corr = in->lambda_corr;
    if (lambda_corr < 0.80f) lambda_corr = 0.80f;
    if (lambda_corr > 1.20f) lambda_corr = 1.20f;

    float pw_eff_ms = FUELMAP_BASE_PW_MS
                    * (ve_pct  * 0.01f)
                    * (in->map_kpa / MAP_REF_KPA)
                    * clt_corr
                    * iat_corr
                    * lambda_corr;

    if (pw_eff_ms < FUELMAP_MIN_OPEN_MS) {
        pw_eff_ms = FUELMAP_MIN_OPEN_MS;
    }
    out->pw_effective_ms = pw_eff_ms;

    float pw_cmd = pw_eff_ms + dt_ms;

    if (pw_cmd > FUELMAP_MAX_PW_MS) {
        pw_cmd = FUELMAP_MAX_PW_MS;
    }
    out->pw_cmd_ms = pw_cmd;
}
