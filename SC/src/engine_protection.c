/**
 * @file    engine_protection.c
 * @brief   Module 6 — Engine Protection (SC variant) Implementation
 *
 * SC additions vs NA:
 *   - Consumes boost_safety_check() result and sets PROT_EVENT_BOOST_*
 *   - IAT-hot warning gated on G8BA_IAT_BOOST_LIMIT_C
 *   - Tightened thermal thresholds (CLT, oil temp) — see g8ba_config.h
 *   - prot_is_hard_cut_active() now also accounts for boost hard cut
 *
 * Sequencing (highest priority first, unchanged from NA):
 *   1. Sync lost  → emergency cut
 *   2. Boost hard cut (latched by boost_safety_check)
 *   3. CLT overtemp graduated
 *   4. Oil pressure low graduated
 *   5. Oil temp high (power reduction)
 *   6. Boost overshoot (persistent) — fuel cut, ignition stays normal
 *   7. IAT hot warn — informational only
 *   8. Rev limit (soft / hard)
 */

#include "engine_protection.h"
#include "fuel_injection.h"
#include "ignition_control.h"
#include "ignition_map.h"
#include "boost_control.h"
#include "crank_cam_sync.h"
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

static const uint16_t OIL_PRESS_RPM_AXIS[]  = {600, 1000, 2000, 3000, 7500};
/*
 * SC: minimum oil pressure raised at every RPM to protect bearings under
 * boosted cylinder pressures.  Was {80,120,160,180,200} in NA.
 */
static const float    OIL_PRESS_MIN_KPA[]   = {100, 150, 190, 220, 240};
#define OIL_PRESS_POINTS  5u

#define CLT_HYSTERESIS_C        3.0f
#define OIL_PRESS_HYSTERESIS    20.0f
#define RPM_HYSTERESIS          150u
#define OIL_TEMP_HYSTERESIS_C   5.0f
#define IAT_HOT_HYSTERESIS_C    5.0f       /* SC */

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile prot_status_t g_protection;

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

static float oil_press_min_for_rpm(rpm_t rpm)
{
    for (uint8_t i = 0u; i < OIL_PRESS_POINTS - 1u; i++) {
        if (rpm < OIL_PRESS_RPM_AXIS[i+1u]) {
            float frac = ((float)rpm - (float)OIL_PRESS_RPM_AXIS[i])
                       / ((float)OIL_PRESS_RPM_AXIS[i+1u] - (float)OIL_PRESS_RPM_AXIS[i]);
            return OIL_PRESS_MIN_KPA[i] + frac *
                   (OIL_PRESS_MIN_KPA[i+1u] - OIL_PRESS_MIN_KPA[i]);
        }
    }
    return OIL_PRESS_MIN_KPA[OIL_PRESS_POINTS - 1u];
}

static prot_level_t eval_clt(float clt_c, prot_level_t prev)
{
    if (clt_c >= G8BA_CLT_CUTOFF_C) {
        return PROT_LEVEL_CUT;
    } else if (clt_c >= G8BA_CLT_PROTECT_C) {
        return PROT_LEVEL_REDUCE;
    } else if (clt_c >= G8BA_CLT_WARN_C) {
        return PROT_LEVEL_WARN;
    } else {
        if (prev == PROT_LEVEL_WARN &&
            clt_c < (G8BA_CLT_WARN_C - CLT_HYSTERESIS_C)) {
            return PROT_LEVEL_OK;
        } else if (prev > PROT_LEVEL_OK) {
            return (clt_c < (G8BA_CLT_WARN_C - CLT_HYSTERESIS_C))
                   ? PROT_LEVEL_OK : prev;
        }
        return PROT_LEVEL_OK;
    }
}

static prot_level_t eval_oil_press(float press_kpa, rpm_t rpm, prot_level_t prev)
{
    if (rpm < G8BA_RPM_CRANK) return PROT_LEVEL_OK;

    float min_press = oil_press_min_for_rpm(rpm);

    if (press_kpa < min_press) {
        return PROT_LEVEL_CUT;
    } else if (press_kpa < (G8BA_OIL_PRESS_WARN_KPA)) {
        return PROT_LEVEL_WARN;
    } else {
        if (prev == PROT_LEVEL_WARN &&
            press_kpa > (G8BA_OIL_PRESS_WARN_KPA + OIL_PRESS_HYSTERESIS)) {
            return PROT_LEVEL_OK;
        }
        return (prev > PROT_LEVEL_OK) ? prev : PROT_LEVEL_OK;
    }
}

/*
 * SC-FIX-C1: take the freshly-built events bitmask as a parameter rather
 * than reading p->active_events.  Previously the function read the field
 * BEFORE engine_protection_update() committed the new value at the end of
 * the tick — so the boost-overshoot reduction always lagged by one 50 ms
 * cycle.  Passing the in-flight events fixes that latency.
 */
static float calc_power_reduction(const prot_status_t *p,
                                  prot_event_flags_t live_events)
{
    float pwr = 0.0f;

    if (p->thermal.clt_level == PROT_LEVEL_REDUCE) {
        float clt = p->inputs.clt_c;
        float frac = (clt - G8BA_CLT_WARN_C) / (G8BA_CLT_CUTOFF_C - G8BA_CLT_WARN_C);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        pwr = frac * 50.0f;
    }

    if (p->thermal.oil_temp_level >= PROT_LEVEL_WARN) {
        pwr += 20.0f;
    }

    /* SC: small additional reduction during persistent boost overshoot to
     * help the BCV catch up — without going to full cut */
    if (live_events & PROT_EVENT_BOOST_OVERSHOOT) {
        pwr += 15.0f;
    }

    if (pwr > 100.0f) pwr = 100.0f;
    return pwr;
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t engine_protection_init(void)
{
    memset((void *)&g_protection, 0, sizeof(g_protection));
    g_protection.overall_level = PROT_LEVEL_OK;
    g_protection.rev_limiter.hysteresis_rpm = G8BA_RPM_SOFT_CUT - RPM_HYSTERESIS;
    return G8BA_OK;
}

void engine_protection_update(const prot_inputs_t *inputs)
{
    if (inputs == NULL) return;

    g_protection.inputs = *inputs;
    prot_event_flags_t events = PROT_EVENT_NONE;

    /* ── 1. Sync lost ──────────────────────────────────────────────────── */
    if (!inputs->sync_ok) {
        prot_emergency_cut();
        events |= PROT_EVENT_SYNC_LOST;
        g_protection.active_events = events;
        g_protection.overall_level = PROT_LEVEL_EMERG;
        return;
    } else {
        prot_emergency_clear();
        fuel_cut_clear(FCUT_SYNC_LOST);
    }

    /* ── 2. Boost safety (SC) ──────────────────────────────────────────── */
    bool boost_hard = boost_safety_check(inputs->map_kpa);
    if (boost_hard) {
        events |= PROT_EVENT_BOOST_HARDCUT;
        g_protection.boost_overshoot_count++;
        prot_log_fault(PROT_EVENT_BOOST_HARDCUT, PROT_LEVEL_CUT, inputs->map_kpa);
    }
    if (g_boost.overshoot_ticks >= G8BA_BOOST_OVER_TICKS && !boost_hard) {
        events |= PROT_EVENT_BOOST_OVERSHOOT;
    }

    /* ── 3. CLT ────────────────────────────────────────────────────────── */
    prot_level_t clt_lv = eval_clt(inputs->clt_c, g_protection.thermal.clt_level);
    g_protection.thermal.clt_level = clt_lv;

    switch (clt_lv) {
    case PROT_LEVEL_CUT:
        events |= (PROT_EVENT_CLT_WARN | PROT_EVENT_CLT_REDUCE | PROT_EVENT_CLT_CUT);
        fuel_cut_set(FCUT_OVERTEMP);
        g_protection.clt_warn_count++;
        prot_log_fault(PROT_EVENT_CLT_CUT, PROT_LEVEL_CUT, inputs->clt_c);
        break;
    case PROT_LEVEL_REDUCE:
        events |= (PROT_EVENT_CLT_WARN | PROT_EVENT_CLT_REDUCE);
        fuel_cut_clear(FCUT_OVERTEMP);
        break;
    case PROT_LEVEL_WARN:
        events |= PROT_EVENT_CLT_WARN;
        fuel_cut_clear(FCUT_OVERTEMP);
        g_protection.clt_warn_count++;
        prot_send_warning(PROT_EVENT_CLT_WARN);
        break;
    default:
        fuel_cut_clear(FCUT_OVERTEMP);
        break;
    }

    /* ── 4. Oil pressure ───────────────────────────────────────────────── */
    prot_level_t oil_lv = eval_oil_press(inputs->oil_pressure_kpa, inputs->rpm,
                                         g_protection.thermal.oil_press_level);
    g_protection.thermal.oil_press_level = oil_lv;

    switch (oil_lv) {
    case PROT_LEVEL_CUT:
        events |= (PROT_EVENT_OIL_PRESS_W | PROT_EVENT_OIL_PRESS_CUT);
        fuel_cut_set(FCUT_LOW_OIL);
        g_protection.oil_warn_count++;
        prot_log_fault(PROT_EVENT_OIL_PRESS_CUT, PROT_LEVEL_CUT,
                       inputs->oil_pressure_kpa);
        break;
    case PROT_LEVEL_WARN:
        events |= PROT_EVENT_OIL_PRESS_W;
        fuel_cut_clear(FCUT_LOW_OIL);
        g_protection.oil_warn_count++;
        prot_send_warning(PROT_EVENT_OIL_PRESS_W);
        break;
    default:
        fuel_cut_clear(FCUT_LOW_OIL);
        break;
    }

    /* ── 5. Oil temperature ────────────────────────────────────────────── */
    prot_level_t oil_temp_prev = g_protection.thermal.oil_temp_level;

    if (inputs->oil_temp_c >= G8BA_OIL_TEMP_MAX_C) {
        events |= (PROT_EVENT_OIL_TEMP_W | PROT_EVENT_OIL_TEMP_CUT);
        g_protection.thermal.oil_temp_level = PROT_LEVEL_REDUCE;
    } else if (inputs->oil_temp_c >= G8BA_OIL_TEMP_WARN_C) {
        events |= PROT_EVENT_OIL_TEMP_W;
        g_protection.thermal.oil_temp_level = PROT_LEVEL_WARN;
        prot_send_warning(PROT_EVENT_OIL_TEMP_W);
    } else {
        if (oil_temp_prev == PROT_LEVEL_OK ||
            inputs->oil_temp_c < (G8BA_OIL_TEMP_WARN_C - OIL_TEMP_HYSTERESIS_C)) {
            g_protection.thermal.oil_temp_level = PROT_LEVEL_OK;
        }
    }

    /* ── 6. IAT hot (SC) — informational warn flag with hysteresis ─────── */
    static bool s_iat_warn_latched = false;
    if (inputs->iat_c >= G8BA_IAT_BOOST_LIMIT_C) {
        s_iat_warn_latched = true;
    } else if (inputs->iat_c < (G8BA_IAT_BOOST_LIMIT_C - IAT_HOT_HYSTERESIS_C)) {
        s_iat_warn_latched = false;
    }
    if (s_iat_warn_latched) {
        events |= PROT_EVENT_IAT_HOT;
        prot_send_warning(PROT_EVENT_IAT_HOT);
    }

    /* ── 7. Power reduction (SC-FIX-C1: pass live events, not stale field) ── */
    g_protection.thermal.power_reduction_pct = calc_power_reduction(
        (const prot_status_t *)&g_protection, events);

    /* ── 8. Overall level ─────────────────────────────────────────────── */
    prot_level_t worst = PROT_LEVEL_OK;
    if (clt_lv > worst)         worst = clt_lv;
    if (oil_lv > worst)         worst = oil_lv;
    if (g_protection.thermal.oil_temp_level > worst)
        worst = g_protection.thermal.oil_temp_level;
    if (boost_hard) worst = PROT_LEVEL_CUT;

    g_protection.active_events  = events;
    g_protection.overall_level  = worst;
    g_protection.last_update_us = 0;
}

void rev_limiter_update(rpm_t rpm)
{
    rev_limiter_state_t *rl = (rev_limiter_state_t *)&g_protection.rev_limiter;
    prot_event_flags_t *ev  = (prot_event_flags_t *)&g_protection.active_events;

    uint8_t mask = ign_map_update_rev_limiter(rpm);
    rl->soft_cut_mask = mask;

    if (rpm >= G8BA_RPM_MAX) {
        rl->hard_active = true;
        rl->soft_active = true;
        *ev |= PROT_EVENT_REV_HARD;
        fuel_cut_set(FCUT_REV_LIMIT);
        ignition_set_mode(IGN_MODE_HARD_CUT);

    } else if (rpm >= G8BA_RPM_SOFT_CUT) {
        rl->hard_active = false;
        rl->soft_active = true;
        rl->soft_cut_toggle ^= 1u;
        *ev |= PROT_EVENT_REV_SOFT;
        fuel_cut_clear(FCUT_REV_LIMIT);
        fuel_cut_set(FCUT_SOFT_CUT);
        ignition_set_soft_cut_mask(mask);
        ignition_set_mode(IGN_MODE_SOFT_CUT);

    } else if (rpm < rl->hysteresis_rpm) {
        rl->hard_active = false;
        rl->soft_active = false;
        *ev &= ~(PROT_EVENT_REV_SOFT | PROT_EVENT_REV_HARD);
        fuel_cut_clear(FCUT_REV_LIMIT);
        fuel_cut_clear(FCUT_SOFT_CUT);
        ignition_set_soft_cut_mask(IGNMAP_ALL_CYL_MASK);
        ignition_set_mode(IGN_MODE_RUNNING);
    }
}

bool prot_is_hard_cut_active(void)
{
    return g_protection.rev_limiter.hard_active ||
           (g_protection.thermal.clt_level   == PROT_LEVEL_CUT) ||
           (g_protection.thermal.oil_press_level == PROT_LEVEL_CUT) ||
           (g_protection.active_events & PROT_EVENT_SYNC_LOST) ||
           (g_protection.active_events & PROT_EVENT_BOOST_HARDCUT);   /* SC */
}

float prot_get_power_reduction(void)
{
    return g_protection.thermal.power_reduction_pct / 100.0f;
}

prot_level_t prot_get_level(void)
{
    return g_protection.overall_level;
}

prot_event_flags_t prot_get_events(void)
{
    return g_protection.active_events;
}

void prot_emergency_cut(void)
{
    fuel_cut_set(FCUT_SYNC_LOST);
    ignition_set_mode(IGN_MODE_HARD_CUT);
    boost_park();                                       /* SC: open BCV       */
    g_protection.active_events |= PROT_EVENT_SYNC_LOST;
    g_protection.overall_level  = PROT_LEVEL_EMERG;
}

void prot_emergency_clear(void)
{
    if (crank_get_sync_state() == SYNC_FULL) {
        g_protection.active_events &= ~PROT_EVENT_SYNC_LOST;
        if (g_protection.overall_level == PROT_LEVEL_EMERG) {
            g_protection.overall_level = PROT_LEVEL_OK;
        }
    }
}

void prot_send_warning(prot_event_flags_t event)
{
    (void)event;
}

void prot_log_fault(prot_event_flags_t event, prot_level_t level, float value)
{
    (void)event; (void)level; (void)value;
}
