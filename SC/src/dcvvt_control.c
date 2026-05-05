/**
 * @file    dcvvt_control.c
 * @brief   Module 4 — D-CVVT Control Implementation
 *
 * PID position control for 4 cam phasers.
 * OCV PWM output: 0% = full retard (park), 50% = hold, 100% = full advance.
 * Actual cam phase is read from g_cam[] populated by crank_cam_sync.
 *
 * Enable conditions:
 *   - CLT > 60°C (oil must be warm enough to build pressure)
 *   - Oil temp > 50°C
 *   - RPM > 500 (idle and above)
 *   - Sync state == SYNC_FULL
 */

#include "dcvvt_control.h"
#include "crank_cam_sync.h"
#include <string.h>
#include <math.h>
#include <float.h>

#include "dcvvt_hw.h"

/* ══════════════════════════════════════════════════════════════════════════
 * VVT TARGET TABLES (RPM × Load %)
 * 8×8 simplified — expand to 16×16 after dyno calibration
 *
 * Intake advance: positive = advance (°CA)
 * Exhaust retard: positive = retard  (°CA, from fully-retarded home)
 * ══════════════════════════════════════════════════════════════════════════ */

#define VVT_TABLE_ROWS  8u
#define VVT_TABLE_COLS  8u

static const uint16_t VVT_RPM_AXIS[VVT_TABLE_ROWS] =
    {800, 1500, 2000, 3000, 4000, 5000, 6000, 7800};
static const float VVT_LOAD_AXIS[VVT_TABLE_COLS] =
    {10, 25, 40, 55, 70, 85, 100, 115};

/* Intake cam advance target (°CA) */
static const float VVT_IN_TARGET[VVT_TABLE_ROWS][VVT_TABLE_COLS] = {
/*       10   25   40   55   70   85  100  115 */
/* 800 */{0,   5,   8,  10,  10,  10,  10,   0},
/*1500 */{0,   8,  14,  18,  20,  18,  15,   5},
/*2000 */{0,  10,  18,  24,  28,  25,  20,   8},
/*3000 */{0,  12,  22,  30,  35,  32,  25,  10},
/*4000 */{0,  14,  24,  32,  40,  38,  30,  12},
/*5000 */{0,  12,  22,  30,  38,  40,  35,  15},
/*6000 */{0,  10,  18,  26,  34,  38,  38,  18},
/*7800 */{0,   5,  12,  20,  28,  32,  35,  20},
};

/* Exhaust cam retard target (°CA from home) */
static const float VVT_EX_TARGET[VVT_TABLE_ROWS][VVT_TABLE_COLS] = {
/*       10   25   40   55   70   85  100  115 */
/* 800 */{0,   3,   5,   5,   5,   5,   5,   0},
/*1500 */{0,   5,  10,  12,  12,  10,   8,   0},
/*2000 */{0,   8,  14,  16,  16,  14,  10,   0},
/*3000 */{0,  10,  16,  20,  20,  18,  14,   0},
/*4000 */{0,  10,  16,  20,  22,  20,  16,   5},
/*5000 */{0,   8,  14,  18,  20,  20,  18,   8},
/*6000 */{0,   5,  10,  14,  18,  18,  18,  10},
/*7800 */{0,   0,   5,  10,  14,  16,  16,  12},
};

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE
 * ══════════════════════════════════════════════════════════════════════════ */

volatile cvvt_status_t g_cvvt;

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

static float vvt_table_lookup(const float table[VVT_TABLE_ROWS][VVT_TABLE_COLS],
                               rpm_t rpm, float load_pct)
{
    uint8_t ri = VVT_TABLE_ROWS - 2u;
    for (uint8_t i = 0u; i < VVT_TABLE_ROWS - 1u; i++) {
        if ((float)rpm < (float)VVT_RPM_AXIS[i+1u]) { ri = i; break; }
    }
    uint8_t ci = VVT_TABLE_COLS - 2u;
    for (uint8_t i = 0u; i < VVT_TABLE_COLS - 1u; i++) {
        if (load_pct < VVT_LOAD_AXIS[i+1u]) { ci = i; break; }
    }

    float rf = ((float)rpm - (float)VVT_RPM_AXIS[ri])
             / ((float)VVT_RPM_AXIS[ri+1u] - (float)VVT_RPM_AXIS[ri]);
    float lf = (load_pct - VVT_LOAD_AXIS[ci])
             / (VVT_LOAD_AXIS[ci+1u] - VVT_LOAD_AXIS[ci]);

    if (rf < 0.0f) rf = 0.0f; if (rf > 1.0f) rf = 1.0f;
    if (lf < 0.0f) lf = 0.0f; if (lf > 1.0f) lf = 1.0f;

    return table[ri][ci]     * (1-rf)*(1-lf)
         + table[ri+1u][ci]  * rf*(1-lf)
         + table[ri][ci+1u]  * (1-rf)*lf
         + table[ri+1u][ci+1u] * rf*lf;
}

static void pid_init(pid_state_t *pid, float kp, float ki, float kd)
{
    pid->kp              = kp;
    pid->ki              = ki;
    pid->kd              = kd;
    pid->integrator      = 0.0f;
    pid->prev_error      = 0.0f;
    pid->output_clamp_max = 95.0f;
    pid->output_clamp_min =  5.0f;
    pid->integrator_max  = 30.0f;
}

static float pid_step(pid_state_t *pid, float error, float dt_s)
{
    /* Proportional */
    float p = pid->kp * error;

    /* Integral with anti-windup */
    pid->integrator += pid->ki * error * dt_s;
    if (pid->integrator >  pid->integrator_max) pid->integrator =  pid->integrator_max;
    if (pid->integrator < -pid->integrator_max) pid->integrator = -pid->integrator_max;

    /* Derivative (on measurement, not error, to avoid derivative kick) */
    float d = 0.0f;
    if (dt_s > 0.0f) {
        d = pid->kd * (error - pid->prev_error) / dt_s;
    }
    pid->prev_error = error;

    float output = p + pid->integrator + d + 50.0f; /* 50% = hold position */

    /* Clamp output */
    if (output > pid->output_clamp_max) output = pid->output_clamp_max;
    if (output < pid->output_clamp_min) output = pid->output_clamp_min;

    return output;
}

static void write_ocv_duty(phaser_id_t phaser, float duty_pct)
{
    dcvvt_hw_set_duty(phaser, duty_pct);
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC IMPLEMENTATION
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t dcvvt_init(void)
{
    /* Initialise TIM3/TIM4 hardware — all channels start at 0% (park) */
    g8ba_status_t hw_st = dcvvt_hw_init();
    if (hw_st != G8BA_OK) return hw_st;

    memset((void *)&g_cvvt, 0, sizeof(g_cvvt));

    for (phaser_id_t i = 0; i < PHASER_COUNT; i++) {
        phaser_state_t *p = (phaser_state_t *)&g_cvvt.phasers[i];
        p->id            = i;
        p->target_deg    = 0.0f;
        p->actual_deg    = 0.0f;
        p->ocv_duty_pct  = 50.0f;
        p->sensor_valid  = false;
        pid_init(&p->pid, G8BA_CVVT_KP, G8BA_CVVT_KI, G8BA_CVVT_KD);
    }

    g_cvvt.enabled = false;

    return G8BA_OK;
}

void dcvvt_update(rpm_t rpm, float load_pct, float oil_temp, float clt_c)
{
    g_cvvt.rpm        = rpm;
    g_cvvt.load_pct   = load_pct;
    g_cvvt.oil_temp_c = oil_temp;
    g_cvvt.clt_c      = clt_c;   /* M-1: store CLT so dcvvt_is_enabled can use it */

    bool now_enabled = dcvvt_is_enabled();

    /*
     * HW-5 FIX: Reset PID integrators on disabled → enabled transition.
     *
     * When CVVT is disabled (cold engine, stall, or oil-temp gate), the
     * PID integrators are not written but retain their values from the last
     * active run.  Without a reset, the first PID step after re-enable begins
     * with a non-zero integrator, producing an immediate output spike that can
     * slam the cam phaser to full advance or full retard before settling.
     *
     * Calling dcvvt_pid_reset() on each phaser before the first control cycle
     * ensures the integrator starts from zero.  The proportional term alone
     * drives the first correction, which is the correct racing-tuned behaviour
     * (fast step response without integrator wind-up artefacts).
     */
    if (now_enabled && !g_cvvt.enabled) {
        for (phaser_id_t i = 0u; i < PHASER_COUNT; i++) {
            dcvvt_pid_reset(i);
        }
    }

    g_cvvt.enabled = now_enabled;

    if (!g_cvvt.enabled) {
        dcvvt_park_all();
        return;
    }

    const float DT_S = (float)TASK_PERIOD_FAST_MS / 1000.0f;

    for (phaser_id_t id = 0; id < PHASER_COUNT; id++) {
        phaser_state_t *p = (phaser_state_t *)&g_cvvt.phasers[id];

        /* Update target from table */
        p->target_deg = dcvvt_lookup_target(id, rpm, load_pct);

        /* Read actual phase from crank_cam_sync */
        floatdeg_t measured = 0.0f;
        cam_id_t cam = (cam_id_t)id;  /* phaser_id_t maps to cam_id_t */
        g8ba_status_t ret = cam_get_phase(cam, &measured);
        p->sensor_valid = (ret == G8BA_OK);

        if (!p->sensor_valid) {
            /* Sensor fault — hold current duty to maintain position */
            dcvvt_hold(id);
            continue;
        }

        p->actual_deg = measured;
        p->error_deg  = p->target_deg - p->actual_deg;

        /* Deadband: within ±G8BA_CVVT_DEADBAND_DEG, hold at 50% */
        if (fabsf(p->error_deg) < G8BA_CVVT_DEADBAND_DEG) {
            p->ocv_duty_pct = 50.0f;
            p->at_target    = true;
            p->hold_lock_count++;
        } else {
            p->ocv_duty_pct = pid_step(&p->pid, p->error_deg, DT_S);
            p->at_target    = false;
            p->hold_lock_count = 0u;
        }

        write_ocv_duty(id, p->ocv_duty_pct);
    }

    g_cvvt.update_count++;
}

void dcvvt_set_target(phaser_id_t phaser, floatdeg_t target_deg)
{
    if (phaser >= PHASER_COUNT) return;

    /* Clamp to physical limits */
    float max_deg = (phaser == PHASER_B1_INTAKE || phaser == PHASER_B2_INTAKE)
                  ? G8BA_CVVT_IN_ADVANCE_MAX
                  : G8BA_CVVT_EX_RETARD_MAX;

    if (target_deg < 0.0f)    target_deg = 0.0f;
    if (target_deg > max_deg) target_deg = max_deg;

    ((phaser_state_t *)&g_cvvt.phasers[phaser])->target_deg = target_deg;
}

void dcvvt_park_all(void)
{
    /*
     * HW-4 FIX: Use direct register write path for the hardware output.
     *
     * The previous implementation called write_ocv_duty(i, 0.0f) which goes
     * through float clamping + CCR arithmetic.  On emergency / stall paths
     * (called from thd_cyl_events timeout or engine_protection), avoiding
     * float is safer and faster.  dcvvt_hw_park_all() writes CCR=0 to all
     * four TIM registers with no floating-point dependency.
     */
    for (phaser_id_t i = 0u; i < PHASER_COUNT; i++) {
        ((phaser_state_t *)&g_cvvt.phasers[i])->target_deg   = 0.0f;
        ((phaser_state_t *)&g_cvvt.phasers[i])->ocv_duty_pct = 0.0f;
    }
    dcvvt_hw_park_all();
}

floatdeg_t dcvvt_lookup_target(phaser_id_t phaser, rpm_t rpm, float load_pct)
{
    float raw = 0.0f;
    switch (phaser) {
    case PHASER_B1_INTAKE:
    case PHASER_B2_INTAKE:
        raw = vvt_table_lookup(VVT_IN_TARGET, rpm, load_pct);
        if (raw > G8BA_CVVT_IN_ADVANCE_MAX) raw = G8BA_CVVT_IN_ADVANCE_MAX;
        break;
    case PHASER_B1_EXHAUST:
    case PHASER_B2_EXHAUST:
        raw = vvt_table_lookup(VVT_EX_TARGET, rpm, load_pct);
        if (raw > G8BA_CVVT_EX_RETARD_MAX) raw = G8BA_CVVT_EX_RETARD_MAX;
        break;
    default:
        break;
    }
    return (raw < 0.0f) ? 0.0f : raw;
}

bool dcvvt_all_at_target(void)
{
    for (phaser_id_t i = 0; i < PHASER_COUNT; i++) {
        if (!g_cvvt.phasers[i].at_target) return false;
    }
    return true;
}

bool dcvvt_is_enabled(void)
{
    /*
     * M-1 FIX: the original code declared `float clt = 0.0f` and then
     * immediately discarded it with `(void)clt`, so the CLT > 60°C
     * enable condition (documented in both the file header and the .h
     * doxygen) was never evaluated — CVVT could activate on a cold engine
     * before the oil reached operating viscosity.
     *
     * CLT is now passed in via dcvvt_update() and stored in g_cvvt.clt_c.
     */
    return (g_cvvt.clt_c      > 60.0f)   &&   /* oil needs warm coolant   */
           (g_cvvt.oil_temp_c > 50.0f)   &&   /* oil viscosity gate       */
           (g_cvvt.rpm        >= 500u)    &&   /* idle and above           */
           (crank_get_sync_state() == SYNC_FULL);
}

floatdeg_t dcvvt_get_actual_phase(phaser_id_t phaser)
{
    if (phaser >= PHASER_COUNT) return 0.0f;
    return g_cvvt.phasers[phaser].actual_deg;
}

void dcvvt_hold(phaser_id_t phaser)
{
    /*
     * H-4 FIX: the original body was `(void)p` — a complete no-op.
     *
     * Consequence: when a cam sensor fails mid-run, dcvvt_update() calls
     * dcvvt_hold() and then `continue`s to the next phaser, leaving the
     * OCV PWM hardware at whatever duty it happened to be at from the
     * previous PID update. Because write_ocv_duty() is the only path that
     * drives the PWM peripheral, and it was never called here, the phaser
     * would drift freely under oil pressure rather than holding position.
     *
     * Fix: re-assert the frozen duty value back to the PWM peripheral.
     * This is idempotent — calling write_ocv_duty with the same value
     * every cycle keeps the solenoid at the last known good position
     * without making any PID changes.
     */
    if (phaser >= PHASER_COUNT) return;
    phaser_state_t *p = (phaser_state_t *)&g_cvvt.phasers[phaser];
    write_ocv_duty(phaser, p->ocv_duty_pct);
}

void dcvvt_pid_reset(phaser_id_t phaser)
{
    if (phaser >= PHASER_COUNT) return;
    pid_state_t *pid = (pid_state_t *)&g_cvvt.phasers[phaser].pid;
    pid->integrator  = 0.0f;
    pid->prev_error  = 0.0f;
}
