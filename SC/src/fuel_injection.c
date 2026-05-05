/**
 * @file    fuel_injection.c
 * @brief   Module 2 — Fuel Injection Control Implementation
 *
 * Sequential MPI injection scheduling:
 *   Each cylinder is injected once per 720° cycle.
 *   Injection starts INJ_ANGLE_BTDC degrees before TDC.
 *   Pulse width is recalculated every cycle from the VE table.
 *
 * [C-1 FIX]  Base pulse-width formula corrected (previous omitted density terms
 *             → 600× overestimate).
 * [FC-1 FIX] Injector flow corrected: 440 → 650 cc/min (stock→racing).
 *             BASE_PW_MS changed from computed-inline (~8.97 ms) to
 *             FUELMAP_BASE_PW_MS (6.074 ms) — ends 47.7% over-fueling.
 * [FC-2 FIX] Internal VE table, CLT/IAT correction functions removed.
 *             All calibration data now sourced from fuel_map.c (sole owner).
 * [FC-3 FIX] Injector dead-time is now voltage-compensated via
 *             fuel_map_get_dead_time_ms() — was fixed 600 µs.
 * [H-2 FIX]  cut_flags protected by mutex — written from multiple threads.
 * [H-5 FIX]  Lambda closed-loop includes both P and I terms.
 */

#include "fuel_injection.h"
#include "fuel_map.h"            /* FC-2 FIX: fuel calibration data module  */
#include "crank_cam_sync.h"
#include <string.h>
#include <math.h>

/* ── RusEFI headers ──────────────────────────────────────────────────────── */
#include "fuel_math.h"
#include "injector_model.h"
#include "sensor.h"
#include "scheduler.h"

/* ══════════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Injection timing: start this many °CA before cylinder TDC */
#define INJ_ANGLE_BTDC          340.0f

/** Lambda PID gains */
#define LAMBDA_KP               0.10f
#define LAMBDA_KI               0.02f
#define LAMBDA_CL_MAX           1.25f   /* max enrichment  (×1.25 = +25%) */
#define LAMBDA_CL_MIN           0.75f   /* max enleanment  (×0.75 = -25%) */

/** Acceleration enrichment parameters */
#define ACCEL_TPS_THRESHOLD     5.0f    /* %/s — below this, no enrichment */
#define ACCEL_DECAY_RATE        0.90f   /* per 10ms tick                   */

/*
 * FC-2 FIX: Internal VE table and correction functions removed.
 *
 * Previously this module maintained a private 16×16 VE table (RPM×MAP),
 * warmup_correction(), and iat_correction() — all duplicating calibration data
 * that is owned by fuel_map.c.  Two sources of truth caused silent divergence:
 *   - VE table: 16×16 here (MAP 20-170 kPa)  vs  16×10 in fuel_map.c (20-105 kPa)
 *   - CLT enrichment: 1.60× max here         vs  1.40× max in fuel_map.c
 *   - Dead-time: fixed 600 µs here           vs  voltage-compensated in fuel_map.c
 *
 * All calibration data now comes from fuel_map.c via:
 *   fuel_map_get_ve()              — bilinear VE lookup
 *   fuel_map_get_clt_correction()  — CLT warm-up enrichment
 *   fuel_map_get_iat_correction()  — IAT charge-density correction
 *   fuel_map_get_dead_time_ms()    — voltage-compensated injector dead-time
 *   FUELMAP_BASE_PW_MS             — base PW constant for 650 cc/min injectors
 */

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile fuel_status_t g_fuel;

/** H-2: mutex protecting g_fuel.cut_flags from concurrent read-modify-write */
static mutex_t  s_fuel_cut_mutex;

/** H-5: Lambda PID state — integrator and last proportional correction */
static float s_lambda_integrator   = 0.0f;  /* accumulated I term          */
static float s_lambda_p_term       = 0.0f;  /* current cycle P term        */

static float s_accel_enrichment    = 0.0f;
static float s_tps_prev            = 0.0f;

/* FC-2 FIX: ve_lookup(), warmup_correction(), iat_correction() removed.
 * See fuel_map.c: fuel_map_get_ve(), fuel_map_get_clt_correction(),
 * fuel_map_get_iat_correction() provide these calculations with consistent
 * calibration data (650 cc/min injectors, 1.40× max CLT enrichment, etc.). */

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t fuel_injection_init(void)
{
    memset((void *)&g_fuel, 0, sizeof(g_fuel));

    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        g_fuel.injectors[i].cyl_index = i;
        g_fuel.injectors[i].inj_angle = INJ_ANGLE_BTDC;
        g_fuel.injectors[i].active    = false;
    }

    g_fuel.corrections.closed_loop = 1.0f;
    g_fuel.corrections.warmup      = 1.0f;
    g_fuel.corrections.iat         = 1.0f;
    g_fuel.corrections.accel       = 1.0f;
    g_fuel.corrections.baro        = 1.0f;

    s_lambda_integrator = 0.0f;
    s_lambda_p_term     = 0.0f;

    /* H-2: initialise mutex for cut_flags */
    chMtxObjectInit(&s_fuel_cut_mutex);

    return G8BA_OK;
}

void fuel_calc_cycle(const fuel_inputs_t *inputs)
{
    if (inputs == NULL) return;

    /* Cache inputs */
    g_fuel.inputs = *inputs;

    /* Fuel cut check — read under lock to avoid partial state */
    chMtxLock(&s_fuel_cut_mutex);
    bool cut_active = (g_fuel.cut_flags != FCUT_NONE);
    chMtxUnlock(&s_fuel_cut_mutex);

    if (cut_active) {
        for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
            g_fuel.injectors[i].pulse_width_us = 0u;
        }
        return;
    }

    /* 1. VE lookup — FC-2 FIX: delegate to fuel_map module */
    float ve = fuel_map_get_ve((float)inputs->rpm, inputs->map_kpa);
    g_fuel.inputs.ve_pct = ve;

    /* 2. Base pulse width — FC-1/FC-2 FIX:
     *
     * Use FUELMAP_BASE_PW_MS (6.074 ms, for 650 cc/min racing injectors).
     *
     * Previous code computed BASE_PW inline from G8BA_INJECTOR_FLOW_CC_MIN
     * which was 440 cc/min (stock injector spec) → BASE_PW = 8.97 ms.
     * With actual 650 cc/min injectors installed, this produced 47.7% over-
     * fueling on every injection event (AFR ≈ 9.9 instead of 14.7 at stoich).
     *
     * FUELMAP_BASE_PW_MS is defined in fuel_map.h:
     *   At 100% VE, 100 kPa MAP, 20°C IAT, λ=1.00, 650 cc/min → 6.074 ms.
     * The MAP_RATIO below scales it to actual manifold pressure.
     */
    const float MAP_RATIO = inputs->map_kpa / MAP_REF_KPA;
    if (MAP_RATIO <= 0.0f) return;   /* sensor failure guard */

    float base_us = FUELMAP_BASE_PW_MS * (ve / 100.0f) * MAP_RATIO * 1000.0f;
    g_fuel.base_pw_us = base_us;

    /* 3. Corrections — FC-2/FC-4 FIX: use fuel_map calibration tables.
     *
     * Removed warmup_correction() (max 1.60×) and iat_correction() (273.15 K
     * reference) from this module. fuel_map_get_clt_correction() (max 1.40×)
     * and fuel_map_get_iat_correction() (293 K reference, table-based) are
     * the authoritative calibrated values — consistent with FUELMAP_BASE_PW_MS.
     */
    g_fuel.corrections.warmup      = fuel_map_get_clt_correction(inputs->clt_c);
    g_fuel.corrections.iat         = fuel_map_get_iat_correction(inputs->iat_c);
    g_fuel.corrections.accel       = 1.0f + s_accel_enrichment;

    /*
     * H-5 FIX: closed_loop = 1.0 + P_term + I_term
     * P_term is recomputed each step in fuel_lambda_pid_step()
     * and feeds into the per-cycle correction.
     */
    float cl = 1.0f + s_lambda_p_term + s_lambda_integrator;
    if (cl > LAMBDA_CL_MAX) cl = LAMBDA_CL_MAX;
    if (cl < LAMBDA_CL_MIN) cl = LAMBDA_CL_MIN;
    g_fuel.corrections.closed_loop = cl;

    g_fuel.corrections.total = g_fuel.corrections.warmup
                             * g_fuel.corrections.iat
                             * g_fuel.corrections.accel
                             * g_fuel.corrections.closed_loop
                             * g_fuel.corrections.baro;

    /* 4. Final pulse width per cylinder */

    /*
     * FC-3 FIX: voltage-compensated injector dead-time.
     *
     * Previous: (us_t)pw_f + G8BA_INJ_DEAD_TIME_US  (flat 600 µs, all voltages)
     *
     * Problem: Injector solenoid opening time depends on drive voltage.
     * At 14 V (normal): dead-time ≈ 520 µs.
     * At 11 V (cranking / low charge): dead-time ≈ 1 800 µs.
     * Using 600 µs at 11 V under-declares 1 200 µs of dead-time per injection
     * → effective PW is 1.2 ms SHORTER than commanded → lean on every cold start.
     *
     * fuel_map_get_dead_time_ms() interpolates the battery-voltage table
     * (range: 8 V→16 V, dead-time 2.2 ms→0.52 ms) for accurate compensation.
     * Computed once before the cylinder loop — same Vbatt for all 8 injectors.
     */
    const float dead_time_us = fuel_map_get_dead_time_ms(inputs->vbatt_v) * 1000.0f;

    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        /*
         * C-1 + guard: base_us * corrections could theoretically be
         * negative if a correction factor is negative (corrupted state).
         * Guard before cast to uint32_t to prevent wraparound to max value.
         */
        float pw_f = base_us * g_fuel.corrections.total;
        if (pw_f < 0.0f) pw_f = 0.0f;

        us_t pw = (us_t)(pw_f + dead_time_us);

        /* Hard clamp */
        if (pw < G8BA_INJ_MIN_PW_US) pw = G8BA_INJ_MIN_PW_US;
        if (pw > G8BA_INJ_MAX_PW_US) pw = G8BA_INJ_MAX_PW_US;

        g_fuel.injectors[i].pulse_width_us = pw;

        /* Duty cycle (compute once, all cylinders identical for MPI) */
        if (i == 0u && inputs->rpm > 0u) {
            /* 720° cycle time = 2 rev / RPM × 60s/min × 1e6 µs/s */
            float cycle_us = 2.0f * 60.0f * 1.0e6f / (float)inputs->rpm;
            g_fuel.inj_dc_pct = 100.0f * (float)pw / cycle_us;
        }
    }

    g_fuel.inj_events++;
}

void fuel_schedule_injector(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;

    /*
     * R-4 FIX: exclude FCUT_SOFT_CUT from the fuel cut check.
     *
     * Previously `cut_flags != FCUT_NONE` caused a full 8-cylinder fuel cut
     * whenever FCUT_SOFT_CUT was set by the soft rev limiter. The "soft" cut
     * is designed to be an ignition-only cut: ignition_set_soft_cut_mask()
     * already alternates a cylinder bitmask each tick, cutting ~50% of
     * combustion events to gently limit RPM.
     *
     * Cutting fuel simultaneously:
     *   (a) makes the soft cut a de facto hard cut — no different in effect
     *       from FCUT_REV_LIMIT.
     *   (b) sends unburnt fuel through the exhaust during the ignition-cut
     *       cycles, risking catalytic converter overheating.
     *
     * Correct behaviour: FCUT_SOFT_CUT → skip fuel injection only for
     * cylinders whose ignition is also cut this cycle. Because
     * fuel_schedule_injector() is called per-cylinder and is not currently
     * wired to the ignition mask (that would require cross-module coupling),
     * the simplest failsafe is to NOT cut fuel on FCUT_SOFT_CUT at all and
     * rely solely on the ignition mask to achieve the RPM reduction.
     * Unburnt charge from unmisted cylinders during the brief soft-cut window
     * is preferable to lean combustion spikes or exhaust damage.
     *
     * Hard cuts (FCUT_OVERTEMP, FCUT_LOW_OIL, FCUT_SYNC_LOST, FCUT_REV_LIMIT)
     * continue to cut all fuel as before.
     */
    chMtxLock(&s_fuel_cut_mutex);
    bool cut_active = (g_fuel.cut_flags & ~FCUT_SOFT_CUT) != FCUT_NONE;
    chMtxUnlock(&s_fuel_cut_mutex);

    if (cut_active) return;

    injector_state_t *inj = (injector_state_t *)&g_fuel.injectors[cyl_index];
    inj->active = true;

    /*
     * RusEFI angle-based scheduler integration:
     * scheduleByAngle(&inj_start_event[cyl_index],
     *                 inj->inj_angle,
     *                 (schfunc_t)injector_fire_callback,
     *                 (void *)(uintptr_t)cyl_index);
     */
}

void fuel_cut_set(fuel_cut_flags_t flag)
{
    /* H-2 FIX: protect read-modify-write with mutex */
    chMtxLock(&s_fuel_cut_mutex);
    g_fuel.cut_flags |= flag;
    chMtxUnlock(&s_fuel_cut_mutex);
}

void fuel_cut_clear(fuel_cut_flags_t flag)
{
    /* H-2 FIX: protect read-modify-write with mutex */
    chMtxLock(&s_fuel_cut_mutex);
    g_fuel.cut_flags &= ~flag;
    chMtxUnlock(&s_fuel_cut_mutex);
}

bool fuel_is_cut(void)
{
    chMtxLock(&s_fuel_cut_mutex);
    bool result = (g_fuel.cut_flags != FCUT_NONE);
    chMtxUnlock(&s_fuel_cut_mutex);
    return result;
}

void fuel_accel_update(float tps_pct, float dt_ms)
{
    /* Guard against zero or invalid dt */
    if (dt_ms < 0.001f) return;

    float tps_dot = (tps_pct - s_tps_prev) / (dt_ms / 1000.0f); /* %/s */
    s_tps_prev = tps_pct;

    if (tps_dot > ACCEL_TPS_THRESHOLD) {
        float add = (tps_dot - ACCEL_TPS_THRESHOLD) * 0.005f;
        s_accel_enrichment += add;
        if (s_accel_enrichment > 0.50f) s_accel_enrichment = 0.50f;
    } else {
        s_accel_enrichment *= ACCEL_DECAY_RATE;
        if (s_accel_enrichment < 0.001f) s_accel_enrichment = 0.0f;
    }
}

void fuel_lambda_pid_step(float lambda_measured, float lambda_target)
{
    /*
     * H-5 FIX: both P and I terms are now implemented.
     *
     * P term: immediate proportional response — corrects within 1 cycle.
     * I term: accumulated correction — removes steady-state offset.
     *
     * The closed-loop correction applied in fuel_calc_cycle() is:
     *   closed_loop = 1.0 + P + I  (clamped to [LAMBDA_CL_MIN, LAMBDA_CL_MAX])
     */
    float error = lambda_target - lambda_measured;

    /* P term: responds immediately to error this cycle */
    s_lambda_p_term = error * LAMBDA_KP;

    /* I term: accumulated, with anti-windup clamping */
    s_lambda_integrator += error * LAMBDA_KI;
    if (s_lambda_integrator >  (LAMBDA_CL_MAX - 1.0f))
        s_lambda_integrator =  (LAMBDA_CL_MAX - 1.0f);
    if (s_lambda_integrator < -(1.0f - LAMBDA_CL_MIN))
        s_lambda_integrator = -(1.0f - LAMBDA_CL_MIN);

    /* Clamp total P+I to prevent over-correction if P is large */
    float total = s_lambda_p_term + s_lambda_integrator;
    if (total >  (LAMBDA_CL_MAX - 1.0f)) s_lambda_p_term =  (LAMBDA_CL_MAX - 1.0f) - s_lambda_integrator;
    if (total < -(1.0f - LAMBDA_CL_MIN)) s_lambda_p_term = -(1.0f - LAMBDA_CL_MIN) - s_lambda_integrator;
}

float fuel_get_duty_cycle(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;
    return g_fuel.inj_dc_pct;
}

float fuel_get_afr_target(void)
{
    return G8BA_STOICH_AFR; /* extend with RPM/load-based AFR table on dyno */
}
