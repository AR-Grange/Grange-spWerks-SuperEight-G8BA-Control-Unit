/**
 * @file    crank_cam_sync.c
 * @brief   Module 1 — Crank/Cam Synchronization Implementation
 *
 * 36-2 trigger wheel decoding:
 *   - RusEFI handles raw VR comparator → digital edge conversion
 *   - We register a tooth callback to maintain our position counter
 *   - Gap detection (2 missing teeth = 20° silence) triggers sync reference
 *   - Cam Hall edge confirms phase (compression vs exhaust TDC)
 *
 * Crank angle computation:
 *   angle_360 = tooth_index × 10°  (each tooth = 360°/36 = 10°)
 *   angle_720 = angle_360 + phase_offset (0° or 360°, from cam)
 */

#include "crank_cam_sync.h"
#include <string.h>
#include <math.h>     /* fabsf() — used in cam phase window check */

/* ── RusEFI internal headers (adjust path to your RusEFI tree) ─────────── */
#include "trigger_central.h"
#include "engine_configuration.h"
#include "os_util.h"    /* getTimeNowUs() */

/* ══════════════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Firing order LUT: firing_order[event_index] = cylinder_index (0-based) */
static const uint8_t FIRING_ORDER[G8BA_CYLINDERS] = G8BA_FIRING_ORDER;

/**
 * TDC crank angles (absolute, 0–720°) for each cylinder in firing order.
 * Cylinder 0 (cyl #1) TDC is the reference angle G8BA_TDC_CYL1_OFFSET_DEG.
 * Each subsequent firing event is offset by G8BA_FIRING_INTERVAL.
 */
static floatdeg_t s_tdc_angles[G8BA_CYLINDERS];

/** RPM low-pass filter time constant (lower = faster, higher = smoother) */
#define RPM_LPF_ALPHA   0.25f

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile engine_position_t g_engine_pos;
volatile cam_state_t       g_cam[CAM_COUNT];

static uint32_t s_sync_count          = 0u;
static uint8_t  s_event_index         = 0u;    /* position in firing order  */
static bool     s_phase_high          = false; /* cam phase offset applied  */

/* H-3: cam phase confirmation state — requires 2 consecutive matches */
static uint8_t  s_phase_confirm_count    = 0u;
static bool     s_phase_high_candidate   = false;

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/** Precompute TDC crank angles for all 8 cylinders */
static void compute_tdc_table(void)
{
    for (uint8_t i = 0u; i < G8BA_CYLINDERS; i++) {
        /* Firing event i fires G8BA_FIRING_INTERVAL × i after cyl-1 TDC */
        floatdeg_t angle = G8BA_TDC_CYL1_OFFSET_DEG
                         + ((floatdeg_t)i * G8BA_FIRING_INTERVAL);
        /* Wrap to 0–720 */
        while (angle >= G8BA_CRANK_ANGLE_CYCLE) {
            angle -= G8BA_CRANK_ANGLE_CYCLE;
        }
        /* Store indexed by cylinder (0-7), not event index */
        s_tdc_angles[FIRING_ORDER[i]] = angle;
    }
}

/** Compute RPM from tooth-to-tooth period */
static rpm_t period_to_rpm(us_t period_us)
{
    if (period_us == 0u) return 0u;
    /* One tooth = 10° = 10/360 revolution
     * RPM = (10/360) / (period_s) × 60
     *      = 10 × 60 / (360 × period_s)
     *      = 1,666,667 / period_us  (with period in µs) */
    return (rpm_t)(1666667uL / period_us);
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t crank_cam_sync_init(void)
{
    memset((void *)&g_engine_pos, 0, sizeof(g_engine_pos));
    memset((void *)g_cam, 0, sizeof(g_cam));

    /* Assign cam channel IDs */
    for (cam_id_t i = 0; i < CAM_COUNT; i++) {
        g_cam[i].id = i;
    }

    compute_tdc_table();

    g_engine_pos.sync_state = SYNC_LOST;
    s_sync_count = 0u;
    s_event_index = 0u;

    /* ── Configure RusEFI trigger ─────────────────────────────────────── */
    /*
     * engineConfiguration->trigger.type = TT_TOOTHED_WHEEL_36_2;
     * engineConfiguration->trigger.customTotalToothCount = 36;
     * engineConfiguration->trigger.customSkippedToothCount = 2;
     * engineConfiguration->vvtMode[0] = VVT_SINGLE_TOOTH; (B1 intake cam)
     * engineConfiguration->vvtMode[1] = VVT_SINGLE_TOOTH; (B1 exhaust cam)
     * ... etc for B2
     *
     * RusEFI will call our registered tooth callback after its own decoding.
     * Registration: addTriggerEventListener(&crank_tooth_callback, "g8ba_crank")
     */

    return G8BA_OK;
}

void crank_tooth_callback(uint8_t tooth_index, us_t timestamp_us)
{
    /*
     * Called from RusEFI TriggerCentral ISR after each decoded crank tooth.
     * tooth_index: 0 = first tooth after gap, 1–33 = subsequent teeth.
     * This function must be ISR-safe: no malloc, no mutex, no blocking.
     */
    us_t prev_ts   = g_engine_pos.last_tooth_us;
    us_t period_us = (prev_ts > 0u) ? (timestamp_us - prev_ts) : 0u;

    g_engine_pos.last_tooth_us  = timestamp_us;
    g_engine_pos.tooth_period_us = period_us;
    g_engine_pos.sync_tooth     = tooth_index;

    /* Crank angle within 360° revolution */
    floatdeg_t angle_360 = (floatdeg_t)tooth_index * G8BA_TRIGGER_TOOTH_ANGLE;
    g_engine_pos.crank_angle_360 = angle_360;

    /* Absolute 720° angle depends on phase (confirmed by cam sensor) */
    if (g_engine_pos.phase_confirmed) {
        g_engine_pos.crank_angle_720 = angle_360 + (s_phase_high ? 360.0f : 0.0f);
        if (g_engine_pos.sync_state == SYNC_PARTIAL) {
            g_engine_pos.sync_state = SYNC_FULL;
            s_sync_count++;
        }
    } else {
        /* Phase unknown — stay in PARTIAL, use 360° position only */
        g_engine_pos.crank_angle_720 = angle_360;
    }

    /* Update RPM (raw and filtered) */
    if (period_us > 0u) {
        rpm_t raw_rpm = period_to_rpm(period_us);
        g_engine_pos.rpm = raw_rpm;

        /* Exponential moving average for filtered RPM */
        float filt = (float)g_engine_pos.rpm_filtered;
        filt = filt + RPM_LPF_ALPHA * ((float)raw_rpm - filt);
        g_engine_pos.rpm_filtered = (rpm_t)filt;
    }

    /* Advance cylinder counter at TDC reference tooth */
    if (tooth_index == 0u && g_engine_pos.sync_state == SYNC_FULL) {
        s_event_index = (s_event_index + 1u) % G8BA_CYLINDERS;
        g_engine_pos.current_cylinder = FIRING_ORDER[s_event_index];
    }

    /* If gap was detected by RusEFI (tooth_index resets to 0), mark partial */
    if (tooth_index == 0u && g_engine_pos.sync_state == SYNC_LOST) {
        g_engine_pos.sync_state = SYNC_PARTIAL;
    }
}

void cam_edge_callback(cam_id_t cam, us_t timestamp_us)
{
    /*
     * Hall sensor rising edge callback.
     * Cam sensor pulse reference angle (from datasheet) is known relative
     * to cylinder #1 TDC. We compare actual crank angle at cam edge vs
     * expected to determine phase offset.
     *
     * For G8BA, intake cam tooth fires near 360° ATDC of cyl #1.
     * If crank_angle_360 is near expected → we are in compression stroke (phase=0)
     * Otherwise → phase = 360° offset.
     */

    if (cam >= CAM_COUNT) return;

    floatdeg_t crank_now = g_engine_pos.crank_angle_360;
    floatdeg_t elapsed_since_gap = crank_now; /* angle since gap tooth */

    g_cam[cam].last_edge_us = timestamp_us;
    g_cam[cam].valid        = true;

    /* Measure cam phase: difference from expected tooth position */
    /* Expected angle for B1 intake cam is approximately 360° into cycle */
    /* (calibrate this value on actual engine) */
    static const floatdeg_t CAM_EXPECTED_ANGLE[CAM_COUNT] = {
        [CAM_B1_INTAKE]  = 25.0f,   /* °CA after gap tooth (calibrate) */
        [CAM_B1_EXHAUST] = 185.0f,
        [CAM_B2_INTAKE]  = 205.0f,
        [CAM_B2_EXHAUST] = 15.0f,
    };

    floatdeg_t raw_phase = elapsed_since_gap - CAM_EXPECTED_ANGLE[cam];
    /* Normalise to ±180° */
    while (raw_phase >  180.0f) raw_phase -= 360.0f;
    while (raw_phase < -180.0f) raw_phase += 360.0f;

    g_cam[cam].phase_deg = raw_phase;

    /* H-3: Confirm engine phase using Bank 1 intake cam
     *
     * Safety requirement: phase determination must be validated by
     * (a) cam edge arriving within ±CAM_PHASE_CONFIRM_WINDOW_DEG of the
     *     known expected crank angle, AND
     * (b) CAM_PHASE_MIN_CONFIRMS consecutive identical conclusions.
     *
     * A wrong phase (off by 360°) would fire all 8 cylinders at the
     * wrong TDC (exhaust instead of compression) — catastrophic.
     */
    if (cam == CAM_B1_INTAKE && !g_engine_pos.phase_confirmed) {
        float expected = CAM_EXPECTED_ANGLE[CAM_B1_INTAKE];
        float deviation = crank_now - expected;

        /* Normalise deviation to ±180° */
        while (deviation >  180.0f) deviation -= 360.0f;
        while (deviation < -180.0f) deviation += 360.0f;

        if (fabsf(deviation) > CAM_PHASE_CONFIRM_WINDOW_DEG) {
            /*
             * Cam edge arrived outside expected window — likely noise or
             * misfire. Reset confirmation counter and DO NOT update phase.
             */
            s_phase_confirm_count  = 0u;
            return;
        }

        /* Edge is within window — determine phase candidate */
        bool candidate = (crank_now > 180.0f);

        if (s_phase_confirm_count == 0u) {
            /* First valid edge: record candidate, await confirmation */
            s_phase_high_candidate = candidate;
            s_phase_confirm_count  = 1u;
        } else if (candidate == s_phase_high_candidate) {
            /* Consistent with previous — increment confirmation count */
            s_phase_confirm_count++;
            if (s_phase_confirm_count >= CAM_PHASE_MIN_CONFIRMS) {
                /* PHASE LOCKED */
                s_phase_high                = s_phase_high_candidate;
                g_engine_pos.phase_confirmed = true;
                s_phase_confirm_count        = 0u; /* reset for re-lock */
            }
        } else {
            /*
             * Inconsistent with previous conclusion — could be sensor
             * noise mid-cycle. Reset and start over from this candidate.
             */
            s_phase_high_candidate = candidate;
            s_phase_confirm_count  = 1u;
        }
    }
}

floatdeg_t crank_get_angle(void)
{
    return g_engine_pos.crank_angle_720;
}

rpm_t crank_get_rpm(void)
{
    return g_engine_pos.rpm_filtered;
}

sync_state_t crank_get_sync_state(void)
{
    return g_engine_pos.sync_state;
}

floatdeg_t crank_tdc_angle(uint8_t cyl)
{
    if (cyl >= G8BA_CYLINDERS) return 0.0f;
    return s_tdc_angles[cyl];
}

uint8_t crank_next_cylinder(void)
{
    uint8_t next_event = (s_event_index + 1u) % G8BA_CYLINDERS;
    return FIRING_ORDER[next_event];
}

g8ba_status_t cam_get_phase(cam_id_t cam, floatdeg_t *out)
{
    if (cam >= CAM_COUNT || out == NULL) return G8BA_ERR_RANGE;
    if (!g_cam[cam].valid)              return G8BA_ERR_SENSOR;
    *out = g_cam[cam].phase_deg;
    return G8BA_OK;
}

void crank_cam_sync_reset(void)
{
    g_engine_pos.sync_state     = SYNC_LOST;
    g_engine_pos.phase_confirmed = false;
    g_engine_pos.rpm            = 0u;
    g_engine_pos.rpm_filtered   = 0u;
    g_engine_pos.current_cylinder = 0u;
    s_event_index               = 0u;
    s_phase_high                = false;
    s_phase_confirm_count       = 0u;       /* H-3: reset confirmation state */
    s_phase_high_candidate      = false;

    for (cam_id_t i = 0; i < CAM_COUNT; i++) {
        g_cam[i].valid    = false;
        g_cam[i].phase_deg = 0.0f;
    }
}

uint32_t crank_sync_count(void)
{
    return s_sync_count;
}
