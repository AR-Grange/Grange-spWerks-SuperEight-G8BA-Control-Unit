/**
 * @file    ignition_map.c
 * @brief   Ignition Calibration Tables — G8BA + Vortech V-7 YSi-Trim (SC variant)
 *
 * SC re-tune notes:
 *   - 9.0:1 dished forged pistons → MBT timing slightly higher than 10.4:1
 *     would have allowed under boost, but still well below NA peak values.
 *   - Boost rows (>100 kPa) pulled to 14–22° peak based on typical V8/SC
 *     dyno data; further pull-back below 5000 RPM where cylinder pressure
 *     peaks earlier.
 *   - Per-cylinder knock retard (knock_control) provides closed-loop trim.
 *   - All values are first-pass — DYNO VERIFICATION REQUIRED before WOT use.
 */

#include "ignition_map.h"
#include "g8ba_config.h"
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════════
 * TABLE AXES
 * ══════════════════════════════════════════════════════════════════════════ */

const uint16_t IGNMAP_RPM_AXIS[IGNMAP_RPM_POINTS] = {
     600u, 800u, 1000u, 1500u, 2000u, 2500u, 3000u, 3500u,
    4000u, 4500u, 5000u, 5500u, 6000u, 6500u, 7000u, 7500u
};

/** MAP axis (kPa absolute) — 14 points; matches fuel_map MAP axis exactly */
const uint8_t IGNMAP_MAP_AXIS[IGNMAP_MAP_POINTS] = {
    30u, 50u, 70u, 90u, 100u,
   110u, 120u, 130u, 140u, 150u,
   170u, 190u, 220u, 250u
};

/* ══════════════════════════════════════════════════════════════════════════
 * BASE ADVANCE TABLE  [16 RPM × 14 MAP]  — degrees BTDC, uint8_t
 * ══════════════════════════════════════════════════════════════════════════
 *
 * Boost retard rationale (per 10 kPa above 100 kPa, very rough rule of thumb):
 *   - Light boost (100–140 kPa) : pull ~1° per 10 kPa
 *   - Heavy boost (140–200 kPa) : pull ~1.5° per 10 kPa
 *   - Above 200 kPa             : pull ~2° per 10 kPa  (knock-limited)
 *
 * Columns:  MAP  30  50  70  90 100 110 120 130 140 150 170 190 220 250 kPa
 * ══════════════════════════════════════════════════════════════════════════ */
static const uint8_t s_adv_table[IGNMAP_RPM_POINTS][IGNMAP_MAP_POINTS] = {
    /* RPM  600 */ { 14, 12,  8,  6,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5 },
    /* RPM  800 */ { 18, 14, 11,  8,  6,  6,  6,  6,  6,  6,  6,  6,  6,  6 },
    /* RPM 1000 */ { 22, 16, 12, 10,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7 },
    /* RPM 1500 */ { 26, 20, 15, 12,  9,  9,  9,  9,  9,  9,  9,  9,  9,  9 },
    /* RPM 2000 */ { 30, 22, 17, 14, 11, 10,  9,  8,  7,  7,  7,  7,  7,  7 },
    /* RPM 2500 */ { 34, 26, 20, 16, 13, 12, 11, 10,  9,  8,  8,  8,  8,  8 },
    /* RPM 3000 */ { 35, 28, 22, 18, 15, 14, 13, 12, 10,  9,  8,  8,  8,  8 },
    /* RPM 3500 */ { 35, 30, 24, 20, 17, 16, 15, 13, 12, 10,  9,  8,  8,  8 },
    /* RPM 4000 */ { 35, 31, 25, 21, 19, 17, 16, 14, 13, 11, 10,  9,  9,  9 },
    /* RPM 4500 */ { 35, 32, 26, 23, 21, 19, 17, 16, 14, 12, 11, 10, 10,  9 },
    /* RPM 5000 */ { 35, 33, 27, 24, 22, 20, 18, 17, 15, 13, 12, 11, 10, 10 },
    /* RPM 5500 */ { 35, 33, 27, 24, 23, 21, 19, 17, 16, 14, 13, 12, 11, 10 },
    /* RPM 6000 */ { 35, 32, 26, 23, 22, 20, 18, 17, 15, 14, 13, 12, 11, 10 },
    /* RPM 6500 */ { 34, 31, 25, 22, 21, 19, 17, 16, 14, 13, 12, 11, 10, 10 },
    /* RPM 7000 */ { 33, 30, 24, 21, 20, 18, 16, 15, 13, 12, 11, 10, 10,  9 },
    /* RPM 7500 */ { 32, 28, 22, 20, 18, 16, 14, 13, 12, 11, 10,  9,  9,  8 },
};

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile ign_cyl_knock_t g_ign_knock[G8BA_CYLINDERS];

static struct {
    ign_rev_state_t state;
    uint8_t         mask_toggle;
} s_rev;

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

static float lerp(float y0, float y1, float frac)
{
    return y0 + frac * (y1 - y0);
}

static float adv_bilinear(float rpm, float map_kpa)
{
    uint8_t ri = 0u, mi = 0u;

    for (uint8_t i = 0u; i < IGNMAP_RPM_POINTS - 1u; i++) {
        if (rpm < (float)IGNMAP_RPM_AXIS[i + 1u]) { ri = i; break; }
        ri = IGNMAP_RPM_POINTS - 2u;
    }

    for (uint8_t j = 0u; j < IGNMAP_MAP_POINTS - 1u; j++) {
        if (map_kpa < (float)IGNMAP_MAP_AXIS[j + 1u]) { mi = j; break; }
        mi = IGNMAP_MAP_POINTS - 2u;
    }

    float rpm_span = (float)(IGNMAP_RPM_AXIS[ri + 1u] - IGNMAP_RPM_AXIS[ri]);
    float map_span = (float)(IGNMAP_MAP_AXIS[mi + 1u] - IGNMAP_MAP_AXIS[mi]);

    float rfrac = 0.0f, mfrac = 0.0f;

    if (rpm_span > 0.0f) {
        rfrac = (rpm - (float)IGNMAP_RPM_AXIS[ri]) / rpm_span;
        if (rfrac < 0.0f) rfrac = 0.0f;
        if (rfrac > 1.0f) rfrac = 1.0f;
    }
    if (map_span > 0.0f) {
        mfrac = (map_kpa - (float)IGNMAP_MAP_AXIS[mi]) / map_span;
        if (mfrac < 0.0f) mfrac = 0.0f;
        if (mfrac > 1.0f) mfrac = 1.0f;
    }

    float v00 = (float)s_adv_table[ri    ][mi    ];
    float v10 = (float)s_adv_table[ri + 1u][mi    ];
    float v01 = (float)s_adv_table[ri    ][mi + 1u];
    float v11 = (float)s_adv_table[ri + 1u][mi + 1u];

    float lo = lerp(v00, v01, mfrac);
    float hi = lerp(v10, v11, mfrac);
    return lerp(lo, hi, rfrac);
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t ign_map_init(void)
{
    memset((void *)g_ign_knock, 0, sizeof(g_ign_knock));

    s_rev.state       = IGNMAP_REV_OFF;
    s_rev.mask_toggle = 0u;

    /* Axis monotonicity check — all builds (IG-3 retained from NA) */
    for (uint8_t i = 0u; i < IGNMAP_RPM_POINTS - 1u; i++) {
        if (IGNMAP_RPM_AXIS[i] >= IGNMAP_RPM_AXIS[i + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    for (uint8_t j = 0u; j < IGNMAP_MAP_POINTS - 1u; j++) {
        if (IGNMAP_MAP_AXIS[j] >= IGNMAP_MAP_AXIS[j + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }

    return G8BA_OK;
}

float ign_map_get_base_advance(float rpm, float map_kpa)
{
    float adv = adv_bilinear(rpm, map_kpa);
    if (adv < 0.0f) adv = 0.0f;
    return adv;
}

float ign_map_get_advance(uint8_t cyl_index, float rpm, float map_kpa)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;

    float base    = adv_bilinear(rpm, map_kpa);
    float retard  = g_ign_knock[cyl_index].retard_deg;

    float net = base - retard;

    if (net < 0.0f)                  net = 0.0f;
    if (net > G8BA_IGN_ADVANCE_MAX)  net = G8BA_IGN_ADVANCE_MAX;

    return net;
}

void ign_map_knock_event(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;

    ign_cyl_knock_t *k = &g_ign_knock[cyl_index];

    float new_retard = k->retard_deg + IGNMAP_KNOCK_RETARD_STEP_DEG;
    if (new_retard > IGNMAP_KNOCK_MAX_RETARD_DEG) {
        new_retard = IGNMAP_KNOCK_MAX_RETARD_DEG;
    }

    k->retard_deg   = new_retard;
    k->knock_events++;
    k->clean_cycles = 0u;
}

void ign_map_clean_cycle(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return;

    ign_cyl_knock_t *k = &g_ign_knock[cyl_index];

    if (k->retard_deg <= 0.0f) {
        k->retard_deg = 0.0f;
        return;
    }

    /* Critical section around RMW (IG-1 retained from NA) */
    chSysLock();
    float new_retard = k->retard_deg - IGNMAP_KNOCK_ADVANCE_STEP_DEG;
    if (new_retard < 0.0f) new_retard = 0.0f;
    k->retard_deg = new_retard;
    chSysUnlock();

    k->clean_cycles++;
}

uint8_t ign_map_update_rev_limiter(rpm_t rpm)
{
    uint8_t mask;

    if (rpm >= IGNMAP_HARD_CUT_RPM) {
        s_rev.state = IGNMAP_REV_HARD;
        mask = 0x00u;

    } else if (rpm >= IGNMAP_SOFT_CUT_RPM) {
        s_rev.state = IGNMAP_REV_SOFT;
        s_rev.mask_toggle ^= 1u;
        mask = (s_rev.mask_toggle != 0u) ? IGNMAP_SOFT_CUT_MASK_B
                                          : IGNMAP_SOFT_CUT_MASK_A;

    } else if (rpm < (IGNMAP_SOFT_CUT_RPM - IGNMAP_SOFT_CUT_HYST_RPM)) {
        s_rev.state       = IGNMAP_REV_OFF;
        s_rev.mask_toggle = 0u;
        mask = IGNMAP_ALL_CYL_MASK;

    } else {
        if (s_rev.state == IGNMAP_REV_SOFT) {
            mask = (s_rev.mask_toggle != 0u) ? IGNMAP_SOFT_CUT_MASK_B
                                              : IGNMAP_SOFT_CUT_MASK_A;
        } else {
            mask = IGNMAP_ALL_CYL_MASK;
        }
    }

    return mask;
}

ign_rev_state_t ign_map_get_rev_state(void)
{
    return s_rev.state;
}

float ign_map_get_cyl_retard(uint8_t cyl_index)
{
    if (cyl_index >= G8BA_CYLINDERS) return 0.0f;
    return g_ign_knock[cyl_index].retard_deg;
}
