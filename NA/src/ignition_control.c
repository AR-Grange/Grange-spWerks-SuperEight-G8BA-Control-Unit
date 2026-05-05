/**
 * @file    ignition_control.c
 * @brief   Module 3 — Ignition Timing Control Implementation
 *
 * [IG-1 FIX] Internal 16×16 advance table removed; delegates to
 *             ign_map_get_base_advance() (ignition_map.c — sole owner).
 * [IG-1 FIX] Per-cylinder knock retard applied via knock_get_retard(i).
 *             Previously all 8 cylinders received the same bank-level
 *             worst-case retard — now each cylinder is retarded only by
 *             its own accumulated retard.
 *
 * Dwell scheduling:
 *   dwell_start_angle = spark_angle − dwell_angle
 *   where dwell_angle = (dwell_ms × rpm / 60000) × 360°
 */

#include "ignition_control.h"
#include "ignition_map.h"        /* IG-1 FIX: advance table (sole owner)       */
#include "knock_control.h"       /* IG-1 FIX: per-cylinder knock retard        */
#include "crank_cam_sync.h"
#include <string.h>
#include <math.h>

/* ── RusEFI headers ──────────────────────────────────────────────────────── */
#include "scheduler.h"
#include "efi_gpio.h"

/*
 * IG-1 FIX: Internal advance table removed.
 *
 * Previously this module maintained a private 16×16 advance table (RPM×load%)
 * duplicating calibration data owned by ignition_map.c (16×10, RPM×MAP kPa).
 * Two separate tables caused inconsistency: different values, different axes.
 *
 * ignition_map.c is now the SOLE owner of base advance data.
 * Base advance is obtained via ign_map_get_base_advance(rpm, map_kpa).
 *
 * NOTE: In this ECU, load_pct passed to ignition_calc_advance() is computed
 * as (map_kpa / 100 × 100 = map_kpa) — numerically equal to MAP in kPa.
 * It is passed directly to ign_map_get_base_advance() which expects kPa.
 * If load_pct is ever decoupled from MAP (e.g., MAF-based), this call site
 * must be updated to pass actual map_kpa as a separate parameter.
 */

/* Dwell vs battery voltage table (3 points) */
static const float DWELL_VBATT[] = {11.0f, 12.0f, 14.5f};
static const float DWELL_MS[]    = {4.5f,  3.5f,  2.8f };

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile ign_status_t g_ignition;

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/* advance_table_lookup() removed — IG-1 FIX: use ign_map_get_base_advance() */

/** CLT correction — retard when cold, slight advance when hot (to max) */
static float clt_advance_corr(float clt_c)
{
    if (clt_c < 20.0f)  return -5.0f;   /* cold retard */
    if (clt_c < 60.0f)  return -2.0f;
    if (clt_c > 100.0f) return -3.0f;   /* overtemp protection */
    return 0.0f;
}

/** IAT correction — retard timing as charge gets hotter (knock risk) */
static float iat_advance_corr(float iat_c)
{
    if (iat_c < 20.0f) return  1.0f;
    if (iat_c < 40.0f) return  0.0f;
    if (iat_c < 60.0f) return -1.0f;
    if (iat_c < 80.0f) return -2.5f;
    return -4.0f;   /* > 80°C */
}

/** Compute dwell duration from battery voltage via interpolation */
static float dwell_from_vbatt(float vbatt)
{
    if (vbatt <= DWELL_VBATT[0]) return DWELL_MS[0];
    if (vbatt >= DWELL_VBATT[2]) return DWELL_MS[2];
    for (uint8_t i = 0u; i < 2u; i++) {
        if (vbatt < DWELL_VBATT[i+1u]) {
            float frac = (vbatt - DWELL_VBATT[i])
                       / (DWELL_VBATT[i+1u] - DWELL_VBATT[i]);
            return DWELL_MS[i] + frac * (DWELL_MS[i+1u] - DWELL_MS[i]);
        }
    }
    return DWELL_MS[1];
}

/** Convert ms dwell to crank degrees at current RPM */
static floatdeg_t dwell_ms_to_deg(float dwell_ms, rpm_t rpm)
{
    if (rpm == 0u) return 30.0f; /* fallback at stall */
    return dwell_ms * (float)rpm * 360.0f / 60000.0f;
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t ignition_init(void)
{
    memset((void *)&g_ignition, 0, sizeof(g_ignition));

    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        g_ignition.coils[i].cyl_index = i;
        g_ignition.coils[i].enabled   = true;
    }

    g_ignition.mode              = IGN_MODE_OFF;
    g_ignition.dwell_ms          = G8BA_IGN_COIL_DWELL_MS;
    g_ignition.soft_cut_mask     = 0xFFu; /* all cylinders enabled */

    return G8BA_OK;
}

void ignition_calc_advance(rpm_t rpm, float load_pct,
                           float clt_c, float iat_c,
                           float knock_retard,
                           ign_advance_t *out)
{
    if (out == NULL) return;

    float base   = 0.0f;
    float clt    = 0.0f;
    float iat    = 0.0f;
    float knock  = -knock_retard;  /* retard is positive, subtract here */

    switch (g_ignition.mode) {
    case IGN_MODE_CRANKING:
        base = G8BA_IGN_ADVANCE_CRANK;
        break;

    case IGN_MODE_RUNNING:
    case IGN_MODE_SOFT_CUT:
        /*
         * IG-1 FIX: call ign_map_get_base_advance() (ignition_map.c sole owner).
         * load_pct = map_kpa numerically in this ECU (see table-removal comment).
         */
        base  = ign_map_get_base_advance((float)rpm, load_pct);
        clt   = clt_advance_corr(clt_c);
        iat   = iat_advance_corr(iat_c);
        break;

    case IGN_MODE_HARD_CUT:
    case IGN_MODE_OFF:
    default:
        /* No spark */
        out->base = out->clt_corr = out->iat_corr = out->knock_retard = 0.0f;
        out->total = 0.0f;
        return;
    }

    /*
     * IG-1 FIX: bank-level total for output struct / diagnostic only.
     * Actual per-cylinder timing is computed individually in the coil loop below.
     */
    float base_advance = base + clt + iat;   /* without knock */
    float total        = base_advance + knock;
    if (total > G8BA_IGN_ADVANCE_MAX) total = G8BA_IGN_ADVANCE_MAX;
    if (total < 0.0f)                 total = 0.0f;

    out->base         = base;
    out->clt_corr     = clt;
    out->iat_corr     = iat;
    out->knock_retard = knock_retard;
    out->total        = total;

    /*
     * H-2 FIX: Propagate to all cylinder coil states.
     *
     * Problem: ignition_coil_on_isr / ignition_coil_off_isr read
     * c->dwell_start and c->spark_angle from interrupt context while
     * this thread loop writes them.  A torn write (ISR fires between
     * writing spark_angle and dwell_start) would schedule dwell from
     * the old position with a new spark angle — coil over-charge or
     * misfired spark.
     *
     * Fix: compute ALL new values into locals first, then write the
     * safety-critical angles inside chSysLock() so the ISR either
     * sees the old complete set OR the new complete set — never a mix.
     *
     * Write order (most critical field last):
     *   1. advance_deg  (diagnostic only — not read by ISR)
     *   2. dwell_start  (ISR uses this to schedule coil_on)
     *   3. spark_angle  (ISR uses this to schedule coil_off)
     *   4. enabled      (ISR checks this before scheduling)
     */
    floatdeg_t dwell_deg = dwell_ms_to_deg(g_ignition.dwell_ms, rpm);

    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        floatdeg_t tdc = crank_tdc_angle(i);

        /*
         * IG-1 FIX: per-cylinder knock retard from knock_control module.
         *
         * Previously all 8 cylinders received the same bank-level worst-case
         * retard passed from main.c (g_knock.total_retard_b1/b2 maximum).
         * If cylinder 3 alone knocked at 10°, ALL 8 cylinders were retarded 10°.
         *
         * Now each cylinder is retarded by only its own accumulated knock retard.
         * Cylinders that did not knock are unaffected — preserving power output
         * on the non-knocking cylinders.  The knock_retard parameter above
         * (bank-level) is retained only for the diagnostic output struct.
         */
        float cyl_retard = knock_get_retard(i);
        float total_i    = base_advance - cyl_retard;
        if (total_i > G8BA_IGN_ADVANCE_MAX) total_i = G8BA_IGN_ADVANCE_MAX;
        if (total_i < 0.0f)                 total_i = 0.0f;

        /* Compute locally */
        floatdeg_t new_spark = tdc - total_i;
        if (new_spark < 0.0f) new_spark += 720.0f;

        floatdeg_t new_dwell = new_spark - dwell_deg;
        if (new_dwell < 0.0f) new_dwell += 720.0f;

        bool new_enabled = (g_ignition.soft_cut_mask & (1u << i)) != 0u;

        /* Write atomically — ISR cannot fire between these stores */
        chSysLock();
        coil_state_t *c = (coil_state_t *)&g_ignition.coils[i];
        c->advance_deg = total_i;   /* per-cylinder actual advance (diagnostic) */
        c->dwell_start = new_dwell; /* before spark_angle — see order note       */
        c->spark_angle = new_spark;
        c->enabled     = new_enabled;
        chSysUnlock();
    }

    g_ignition.advance_breakdown = *out;
}

void ignition_schedule_spark(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;

    /*
     * SAFE-2 FIX (defense in depth): also check g_ignition.mode.  Even with
     * the corrected ignition_set_mode() that propagates enabled flags, we
     * want a final firewall here so any stale enabled bit (e.g. from a coil
     * state struct being initialised before a mode transition completes)
     * cannot reach the angle scheduler during HARD_CUT or OFF.
     */
    ign_mode_t mode = g_ignition.mode;
    if (mode == IGN_MODE_OFF || mode == IGN_MODE_HARD_CUT) return;

    coil_state_t *c = (coil_state_t *)&g_ignition.coils[cyl_index];
    if (!c->enabled) return;

    /*
     * RusEFI integration:
     * scheduleByAngle(&coil_on_event,  c->dwell_start, ignition_coil_on_isr,  cyl_index);
     * scheduleByAngle(&coil_off_event, c->spark_angle,  ignition_coil_off_isr, cyl_index);
     */
}

void ignition_set_mode(ign_mode_t mode)
{
    g_ignition.mode = mode;

    /*
     * SAFE-2 FIX: propagate mode → coils[].enabled immediately.
     *
     * Previously the function updated only soft_cut_mask, leaving coils[].enabled
     * stale until the next ignition_calc_advance() pass.  But the OFF and HARD_CUT
     * branches of ignition_calc_advance() return EARLY and never touch enabled,
     * so once we transitioned RUNNING → HARD_CUT the per-cylinder enable flags
     * stayed `true`.  ignition_schedule_spark() only checks c->enabled (not
     * g_ignition.mode), so a pending angle-scheduled fire-event from RusEFI
     * could still drive the coil during a hard-cut — defeating the rev-limit
     * and the sync-loss emergency cut.
     *
     * Fix: clear mask AND every coils[].enabled atomically here on entry to
     * any non-firing mode; restore both on entry to a firing mode so the next
     * angle event is dispatched correctly.
     */
    if (mode == IGN_MODE_HARD_CUT || mode == IGN_MODE_OFF) {
        g_ignition.soft_cut_mask = 0x00u;
        chSysLock();
        for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
            ((coil_state_t *)&g_ignition.coils[i])->enabled = false;
        }
        chSysUnlock();
    } else if (mode == IGN_MODE_RUNNING || mode == IGN_MODE_CRANKING) {
        g_ignition.soft_cut_mask = 0xFFu;
        chSysLock();
        for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
            ((coil_state_t *)&g_ignition.coils[i])->enabled = true;
        }
        chSysUnlock();
    }
    /* IGN_MODE_SOFT_CUT: caller drives ignition_set_soft_cut_mask() with
     * the alternating mask; do not touch enabled here. */
}

void ignition_set_soft_cut_mask(uint8_t mask)
{
    g_ignition.soft_cut_mask = mask;
    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        ((coil_state_t *)&g_ignition.coils[i])->enabled =
            (mask & (1u << i)) != 0u;
    }
}

float ignition_get_advance(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;
    return g_ignition.coils[cyl_index].advance_deg;
}

void ignition_update_dwell(float vbatt)
{
    g_ignition.dwell_ms = dwell_from_vbatt(vbatt);
}

void ignition_coil_on_isr(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;
    /* Drive coil GPIO HIGH to begin charging
     * efiSetPinValue(coil_pins[cyl_index], true); */
    ((coil_state_t *)&g_ignition.coils[cyl_index])->coil_active = true;
}

void ignition_coil_off_isr(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;
    /* Drive coil GPIO LOW — current collapse produces spark
     * efiSetPinValue(coil_pins[cyl_index], false); */
    ((coil_state_t *)&g_ignition.coils[cyl_index])->coil_active = false;
    ((coil_state_t *)&g_ignition.coils[cyl_index])->fire_count++;
    ((ign_status_t *)&g_ignition)->total_sparks++;
}
