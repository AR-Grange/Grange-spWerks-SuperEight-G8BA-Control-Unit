/**
 * @file    boost_control.c
 * @brief   Module 7 - Boost Control (Vortech V-7 YSi-Trim) Implementation
 *
 * Closed-loop PID drives the bypass-control valve (BCV) PWM duty so that
 * measured MAP tracks a target derived from engine speed and throttle position.
 *
 * Hardware: STM32H743 TIM5 CH1 -> GPIOA PA0 (AF2), 40 Hz PWM.
 *           (PA0 chosen because TIM3/TIM4 are owned by D-CVVT.)
 *
 * Safety priorities (highest first):
 *   1. boost_safety_check() - hard-cut if MAP >= G8BA_BOOST_MAX_KPA (250 kPa)
 *   2. NaN/Inf guard in boost_update() - park BCV open on sensor fault
 *   3. sync_ok=false -> mode=OFF, BCV open
 *   4. Boost only enabled above G8BA_BOOST_ENABLE_RPM (2500) - prevents
 *      off-idle stumble and protects torque converter / clutch
 *   5. PID integrator clamped (anti-windup)
 */

#include "boost_control.h"
#include "fuel_injection.h"   /* FCUT_BOOST_OVERSHOOT (added in SC engine_protection) */
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"

/* ==========================================================================
 * BOOST TARGET TABLE  (RPM x TPS%)  -  kPa absolute
 * ==========================================================================
 *
 * Below G8BA_BOOST_ENABLE_RPM (2500) and below ~30 % TPS the table returns
 * 100 kPa (atmospheric) so the BCV stays open.  Above that, target ramps
 * smoothly up to G8BA_BOOST_TARGET_KPA (148 kPa ~ 7 PSI) at WOT and high RPM.
 *
 * Conservative starting point - re-tune on dyno.
 * ========================================================================== */

#define BOOST_TGT_RPM_PTS   8u
#define BOOST_TGT_TPS_PTS   8u

static const uint16_t s_tgt_rpm_axis[BOOST_TGT_RPM_PTS] = {
    1500u, 2500u, 3000u, 3500u, 4000u, 5000u, 6000u, 7500u
};
static const uint8_t s_tgt_tps_axis[BOOST_TGT_TPS_PTS] = {
    0u, 10u, 20u, 30u, 50u, 70u, 90u, 100u
};

/** Target MAP (kPa absolute).  Rows = RPM, columns = TPS%. */
static const uint8_t s_target_kpa[BOOST_TGT_RPM_PTS][BOOST_TGT_TPS_PTS] = {
    /*  TPS% 0  10  20  30  50  70  90 100 */
    /*1500*/{100,100,100,100,100,100,100,100},  /* below enable RPM - atmospheric */
    /*2500*/{100,100,100,105,110,115,120,120},
    /*3000*/{100,100,100,108,118,128,135,135},
    /*3500*/{100,100,100,110,125,135,142,142},
    /*4000*/{100,100,100,112,130,140,146,148},
    /*5000*/{100,100,100,113,132,142,148,148},
    /*6000*/{100,100,100,113,132,142,148,148},
    /*7500*/{100,100,100,110,128,138,144,144},  /* slight back-off near limiter */
};

/* ==========================================================================
 * MODULE STATE
 * ========================================================================== */

volatile boost_status_t g_boost;

/** PID internal state */
static struct {
    float integrator;
    float prev_error;
} s_pid;

/** Mutex protects boost_update() vs boost_safety_check() vs boost_park()
 *  cross-thread writes to g_boost.bcv_duty_pct / g_boost.mode. */
static mutex_t s_boost_mutex;

/* ==========================================================================
 * BCV HARDWARE - TIM5 CH1 on PA0 (AF2), 40 Hz PWM
 *
 *   APB1 timer clock : 200 MHz
 *   PSC = 999  ->  200 MHz / 1000 = 200 kHz timer tick
 *   ARR = 4999 ->  200 kHz / 5000 = 40 Hz period
 * ========================================================================== */

#define BCV_TIM             TIM5
#define BCV_PSC             999u
#define BCV_ARR             4999u

/* Compile-time check of frequency math */
#if (200000000UL / ((BCV_PSC + 1u) * (BCV_ARR + 1u))) != G8BA_BCV_PWM_HZ
#  error "BCV PSC/ARR combination does not yield G8BA_BCV_PWM_HZ"
#endif

static g8ba_status_t bcv_hw_init(void)
{
    /* Enable GPIOA clock */
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
    __DSB();

    /* Enable TIM5 clock (APB1) */
    RCC->APB1LENR |= RCC_APB1LENR_TIM5EN;
    __DSB();

    /* PA0: alternate function (0b10 in MODER bits [1:0]) */
    GPIOA->MODER  = (GPIOA->MODER  & ~(0x3u << 0u)) | (0x2u << 0u);
    /* PA0: AF2 in AFRL[3:0] */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFu << 0u)) | (0x2u << 0u);
    /* PA0: very-high speed (0b11 in OSPEEDR bits [1:0]) */
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(0x3u << 0u)) | (0x3u << 0u);
    /* PA0: no pull (0b00 - explicit) */
    GPIOA->PUPDR  &= ~(0x3u << 0u);

    /* Configure TIM5 */
    BCV_TIM->CR1  = 0u;
    BCV_TIM->PSC  = BCV_PSC;
    BCV_TIM->ARR  = BCV_ARR;
    /* PWM mode 1 + preload on CH1: OC1M=110 (bits[6:4]), OC1PE=1 (bit 3) -> 0x68 */
    BCV_TIM->CCMR1 = 0x68u;
    BCV_TIM->CCR1  = 0u;                 /* start at 0% - BCV fully open    */
    BCV_TIM->CCER  = 0x1u;               /* CC1E = 1                         */
    BCV_TIM->EGR   = TIM_EGR_UG;
    BCV_TIM->SR    = 0u;
    BCV_TIM->CR1   = 0x81u;              /* ARPE | CEN                       */

    /* Verify CEN was actually set (peripheral responded) */
    if ((BCV_TIM->CR1 & TIM_CR1_CEN) == 0u) {
        return G8BA_ERR_HW;
    }
    return G8BA_OK;
}

/** Write BCV duty cycle (0-100 %).  NaN/Inf -> fully open (0 %).  */
static void bcv_set_duty(float duty_pct)
{
    if (!isfinite(duty_pct)) {
        BCV_TIM->CCR1 = 0u;
        return;
    }
    if (duty_pct < G8BA_BCV_DUTY_MIN_PCT) duty_pct = G8BA_BCV_DUTY_MIN_PCT;
    if (duty_pct > G8BA_BCV_DUTY_MAX_PCT) duty_pct = G8BA_BCV_DUTY_MAX_PCT;

    uint32_t ccr = (uint32_t)(duty_pct * (float)(BCV_ARR + 1u) * 0.01f);
    if (ccr > BCV_ARR) ccr = BCV_ARR;
    BCV_TIM->CCR1 = ccr;
}

/* ==========================================================================
 * PRIVATE HELPERS
 * ========================================================================== */

static float lerp(float y0, float y1, float frac)
{
    return y0 + frac * (y1 - y0);
}

/** 2-D bilinear interpolation on the boost target table */
static float target_lookup(float rpm, float tps_pct)
{
    uint8_t ri = 0u, ti = 0u;

    for (uint8_t i = 0u; i < BOOST_TGT_RPM_PTS - 1u; i++) {
        if (rpm < (float)s_tgt_rpm_axis[i + 1u]) { ri = i; break; }
        ri = BOOST_TGT_RPM_PTS - 2u;
    }
    for (uint8_t j = 0u; j < BOOST_TGT_TPS_PTS - 1u; j++) {
        if (tps_pct < (float)s_tgt_tps_axis[j + 1u]) { ti = j; break; }
        ti = BOOST_TGT_TPS_PTS - 2u;
    }

    float rspan = (float)(s_tgt_rpm_axis[ri + 1u] - s_tgt_rpm_axis[ri]);
    float tspan = (float)(s_tgt_tps_axis[ti + 1u] - s_tgt_tps_axis[ti]);

    float rfrac = (rspan > 0.0f) ? (rpm - (float)s_tgt_rpm_axis[ri]) / rspan : 0.0f;
    float tfrac = (tspan > 0.0f) ? (tps_pct - (float)s_tgt_tps_axis[ti]) / tspan : 0.0f;

    if (rfrac < 0.0f) rfrac = 0.0f; if (rfrac > 1.0f) rfrac = 1.0f;
    if (tfrac < 0.0f) tfrac = 0.0f; if (tfrac > 1.0f) tfrac = 1.0f;

    float v00 = (float)s_target_kpa[ri    ][ti    ];
    float v10 = (float)s_target_kpa[ri + 1u][ti    ];
    float v01 = (float)s_target_kpa[ri    ][ti + 1u];
    float v11 = (float)s_target_kpa[ri + 1u][ti + 1u];

    float lo = lerp(v00, v01, tfrac);
    float hi = lerp(v10, v11, tfrac);
    return lerp(lo, hi, rfrac);
}

/* ==========================================================================
 * PUBLIC IMPLEMENTATION
 * ========================================================================== */

g8ba_status_t boost_init(void)
{
    memset((void *)&g_boost, 0, sizeof(g_boost));
    s_pid.integrator = 0.0f;
    s_pid.prev_error = 0.0f;

    chMtxObjectInit(&s_boost_mutex);

    /* Validate target table axes (BC-1 safety check at boot) */
    for (uint8_t i = 0u; i < BOOST_TGT_RPM_PTS - 1u; i++) {
        if (s_tgt_rpm_axis[i] >= s_tgt_rpm_axis[i + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }
    for (uint8_t j = 0u; j < BOOST_TGT_TPS_PTS - 1u; j++) {
        if (s_tgt_tps_axis[j] >= s_tgt_tps_axis[j + 1u]) {
            return G8BA_ERR_RANGE;
        }
    }

    /* Initialise BCV hardware - must succeed (peripheral readback verified) */
    g8ba_status_t hw_st = bcv_hw_init();
    if (hw_st != G8BA_OK) return hw_st;

    g_boost.mode         = BOOST_MODE_OFF;
    g_boost.bcv_duty_pct = 0.0f;
    g_boost.target_kpa   = MAP_REF_KPA;   /* 100 kPa */
    bcv_set_duty(0.0f);                    /* explicitly open BCV at boot     */

    return G8BA_OK;
}

float boost_get_target_kpa(rpm_t rpm, float tps_pct)
{
    if (!isfinite(tps_pct)) tps_pct = 0.0f;
    if (tps_pct < 0.0f)     tps_pct = 0.0f;
    if (tps_pct > 100.0f)   tps_pct = 100.0f;

    float t = target_lookup((float)rpm, tps_pct);

    /* Hard clamp to project safety limit */
    if (t < MAP_REF_KPA)             t = MAP_REF_KPA;
    if (t > G8BA_BOOST_TARGET_KPA)   t = G8BA_BOOST_TARGET_KPA;
    return t;
}

/** Persistence counter for MAP-implausible-low fault (SAFE-SC-1).
 *  Cleared whenever MAP becomes plausible OR engine is below the gate RPM. */
static uint8_t s_map_fault_ticks = 0u;

void boost_update(rpm_t rpm, float tps_pct, float map_kpa,
                  float clt_c, bool sync_ok, bool engine_protect_cut)
{
    /* -- Failsafe gates (highest priority first) ------------------------- */

    /* Sync lost -> emergency park */
    if (!sync_ok) {
        boost_park();
        s_map_fault_ticks = 0u;
        return;
    }

    /* Engine-protection hard cut active (CLT/oil/sync) -> park BCV.
     * SAFE-SC-3: previously the BCV was left at the last PID duty, which
     * could mean fully closed while fuel was being cut for low oil.  The
     * residual plenum pressure delays engine-deceleration.  Park ensures
     * the bypass dumps charge immediately. */
    if (engine_protect_cut) {
        boost_park();
        s_map_fault_ticks = 0u;
        return;
    }

    /* NaN/Inf inputs -> fault */
    if (!isfinite(map_kpa) || !isfinite(tps_pct) || !isfinite(clt_c)) {
        chMtxLock(&s_boost_mutex);
        g_boost.mode = BOOST_MODE_FAULT;
        g_boost.bcv_duty_pct = 0.0f;
        chMtxUnlock(&s_boost_mutex);
        bcv_set_duty(0.0f);
        s_pid.integrator = 0.0f;
        s_pid.prev_error = 0.0f;
        s_map_fault_ticks = 0u;
        return;
    }

    /*
     * SAFE-SC-1: MAP sensor wire-break / pull-down fault detection.
     *
     * Failure scenario without this check:
     *   1. MAP sensor ground wire breaks -> ADC reads ~0 mV -> 0 kPa
     *   2. Engine actually IS in boost (e.g. 200 kPa) - driver standing on it
     *   3. boost_update() sees error = target(148) - 0 = +148
     *   4. PID drives BCV to G8BA_BCV_DUTY_MAX_PCT (95%) -> BCV closes ->
     *      max geometric boost from supercharger
     *   5. boost_safety_check() ALSO sees MAP=0 -> never trips
     *   6. Real cylinder pressure climbs uncontrolled -> catastrophic failure
     *
     * Detection: when engine is running above BOOST_MAP_PLAUSIBLE_RPM,
     * MAP must be >= BOOST_MAP_PLAUSIBLE_MIN_KPA (30 kPa).  Anything below
     * is a sensor fault - park BCV and assert FCUT_BOOST_OVERSHOOT after
     * a short debounce.
     */
    if (rpm >= BOOST_MAP_PLAUSIBLE_RPM && map_kpa < BOOST_MAP_PLAUSIBLE_MIN_KPA) {
        if (s_map_fault_ticks < UINT8_MAX) s_map_fault_ticks++;
        if (s_map_fault_ticks >= BOOST_MAP_FAULT_TICKS) {
            fuel_cut_set(FCUT_BOOST_OVERSHOOT);
            chMtxLock(&s_boost_mutex);
            g_boost.mode = BOOST_MODE_FAULT;
            g_boost.bcv_duty_pct = 0.0f;
            chMtxUnlock(&s_boost_mutex);
            bcv_set_duty(0.0f);
            s_pid.integrator = 0.0f;
            return;
        }
    } else {
        if (s_map_fault_ticks > 0u) s_map_fault_ticks--;
    }

    /* If hard-cut latched by safety check earlier, force BCV open (overrides PID) */
    if (g_boost.hard_cut_active) {
        chMtxLock(&s_boost_mutex);
        g_boost.mode = BOOST_MODE_FAULT;
        g_boost.bcv_duty_pct = 0.0f;
        chMtxUnlock(&s_boost_mutex);
        bcv_set_duty(0.0f);
        s_pid.integrator = 0.0f;
        return;
    }

    /* -- Mode selection -------------------------------------------------- */
    boost_mode_t want;
    if (rpm < G8BA_BOOST_ENABLE_RPM) {
        want = BOOST_MODE_OFF;
    } else if (clt_c < BOOST_CLT_ENABLE_C) {
        /* SAFE-SC-2: cold-engine boost lockout - protects bearings + intercooler
         * water pump (often electric, may not be primed when engine is cold) */
        want = BOOST_MODE_OFF;
    } else {
        want = BOOST_MODE_CLOSED_LOOP;
    }

    /* Reset PID on mode transition into closed-loop (anti-windup carryover) */
    boost_mode_t was = g_boost.mode;
    if (want == BOOST_MODE_CLOSED_LOOP && was != BOOST_MODE_CLOSED_LOOP) {
        s_pid.integrator = 0.0f;
        s_pid.prev_error = 0.0f;
    }

    g_boost.map_kpa = map_kpa;
    g_boost.mode    = want;

    if (want == BOOST_MODE_OFF) {
        chMtxLock(&s_boost_mutex);
        g_boost.bcv_duty_pct = 0.0f;
        g_boost.target_kpa   = MAP_REF_KPA;
        g_boost.error_kpa    = 0.0f;
        chMtxUnlock(&s_boost_mutex);
        bcv_set_duty(0.0f);
        return;
    }

    /* -- PID step (closed-loop) ------------------------------------------ */
    float target = boost_get_target_kpa(rpm, tps_pct);
    float error  = target - map_kpa;       /* positive = need MORE boost  */
    const float DT_S = (float)TASK_PERIOD_MEDIUM_MS / 1000.0f;

    /* P term */
    float p = G8BA_BOOST_KP * error;

    /* I term with anti-windup: integrator clamped to [-40, +60] %duty equiv */
    s_pid.integrator += G8BA_BOOST_KI * error * DT_S;
    if (s_pid.integrator >  60.0f) s_pid.integrator =  60.0f;
    if (s_pid.integrator < -40.0f) s_pid.integrator = -40.0f;

    /* D term on error (target steps are rare -> no derivative-kick risk) */
    float d = (DT_S > 0.0f) ? G8BA_BOOST_KD * (error - s_pid.prev_error) / DT_S
                            : 0.0f;
    s_pid.prev_error = error;

    /* Closed-loop duty: more positive error -> close BCV more (raise duty) */
    float duty = p + s_pid.integrator + d;

    /*
     * SC-FIX-C2: clamp the output and BACK OUT the latest integrator step
     * if we hit saturation in the same direction as the error.  This is the
     * standard "back-calculation" anti-windup - without it, the integrator
     * (already clamped to +/-60) can sit at +60 long after the error sign
     * reverses, producing a sluggish recovery (overshoot in the opposite
     * direction).  Hard +/-[40, 60] limit on integrator remains the absolute
     * last-resort guard.
     */
    if (duty > G8BA_BCV_DUTY_MAX_PCT) {
        if (error > 0.0f) {
            s_pid.integrator -= G8BA_BOOST_KI * error * DT_S;
            if (s_pid.integrator >  60.0f) s_pid.integrator =  60.0f;
            if (s_pid.integrator < -40.0f) s_pid.integrator = -40.0f;
        }
        duty = G8BA_BCV_DUTY_MAX_PCT;
    }
    if (duty < G8BA_BCV_DUTY_MIN_PCT) {
        if (error < 0.0f) {
            s_pid.integrator -= G8BA_BOOST_KI * error * DT_S;
            if (s_pid.integrator >  60.0f) s_pid.integrator =  60.0f;
            if (s_pid.integrator < -40.0f) s_pid.integrator = -40.0f;
        }
        duty = G8BA_BCV_DUTY_MIN_PCT;
    }

    chMtxLock(&s_boost_mutex);
    g_boost.target_kpa   = target;
    g_boost.error_kpa    = error;
    g_boost.bcv_duty_pct = duty;
    g_boost.pid_p_term   = p;
    g_boost.pid_i_term   = s_pid.integrator;
    g_boost.pid_d_term   = d;
    g_boost.pid_steps++;
    chMtxUnlock(&s_boost_mutex);

    bcv_set_duty(duty);
}

bool boost_safety_check(float map_kpa)
{
    /* Sensor-fault treatment: NaN/Inf -> force park, leave any latched cut alone */
    if (!isfinite(map_kpa)) {
        boost_park();
        return g_boost.hard_cut_active;   /* preserve previous latch state    */
    }

    /*
     * SAFE-SC-4: implausibly-low MAP must NOT decrement the overshoot counter
     * nor clear a latched fuel cut.
     *
     * Failure scenario without this guard:
     *   1. Real MAP overshoot at 250 kPa -> hard_cut_active=true, fuel_cut set,
     *      ticks=4
     *   2. MAP sensor wire breaks one tick later -> ADC reads 0
     *   3. 0 is finite and < OVERSHOOT(180), so we'd take the `else` branch:
     *        - decrement ticks every tick
     *        - after 4 ticks (200 ms) ticks reach 0 AND 0 < 170 -> CLEAR cut!
     *   4. Meanwhile boost_update's SAFE-SC-1 sets the SAME cut on its 10 ms
     *      cadence -> the cut would oscillate ON/OFF, allowing fuel pulses
     *      into a still-boosted engine.
     *
     * Fix: low-plausible MAP is treated as "unknown" - leave counters and
     * cut state untouched.  SAFE-SC-1 in boost_update is the authoritative
     * sensor-fault path; this function only acts on plausible readings.
     */
    if (map_kpa < BOOST_MAP_PLAUSIBLE_MIN_KPA) {
        return g_boost.hard_cut_active;
    }

    /* Immediate hard-cut: any plausible sample at or above MAX -> fuel cut now */
    if (map_kpa >= G8BA_BOOST_MAX_KPA) {
        g_boost.hard_cut_active = true;
        g_boost.overshoot_ticks = G8BA_BOOST_OVER_TICKS; /* latch high */
        fuel_cut_set(FCUT_BOOST_OVERSHOOT);
        bcv_set_duty(0.0f);                   /* slam BCV open immediately   */
        return true;
    }

    /* Persistent overshoot above OVERSHOOT_KPA (plausible readings only) */
    if (map_kpa >= G8BA_BOOST_OVERSHOOT_KPA) {
        if (g_boost.overshoot_ticks < UINT16_MAX) {
            g_boost.overshoot_ticks++;
        }
        if (g_boost.overshoot_ticks >= G8BA_BOOST_OVER_TICKS) {
            fuel_cut_set(FCUT_BOOST_OVERSHOOT);
            return true;
        }
    } else {
        /* Plausible MAP and below OVERSHOOT - decay counter & maybe clear */
        if (g_boost.overshoot_ticks > 0u) g_boost.overshoot_ticks--;
        if (g_boost.overshoot_ticks == 0u && map_kpa < (G8BA_BOOST_OVERSHOOT_KPA - 10.0f)) {
            g_boost.hard_cut_active = false;
            fuel_cut_clear(FCUT_BOOST_OVERSHOOT);
        }
    }

    return g_boost.hard_cut_active;
}

void boost_park(void)
{
    chMtxLock(&s_boost_mutex);
    g_boost.mode         = BOOST_MODE_OFF;
    g_boost.bcv_duty_pct = 0.0f;
    g_boost.target_kpa   = MAP_REF_KPA;
    g_boost.error_kpa    = 0.0f;
    chMtxUnlock(&s_boost_mutex);

    s_pid.integrator = 0.0f;
    s_pid.prev_error = 0.0f;

    bcv_set_duty(0.0f);
}

float boost_get_bcv_duty(void)
{
    return g_boost.bcv_duty_pct;
}

boost_mode_t boost_get_mode(void)
{
    return g_boost.mode;
}
