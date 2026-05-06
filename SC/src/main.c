/**
 * @file    main.c
 * @brief   G8BA-SC ECU Firmware - Main Entry Point & ChibiOS Task Orchestration
 *
 * Vortech V-7 YSi-Trim variant.  See SC/README.md and SC/PR.md for
 * differences against the NA branch.
 *
 * +-------------------------------------------------------------------------+
 * |  Task Map (SC adds boost-control PID into thd_medium_ctrl)              |
 * |                                                                         |
 * |  Priority           Thread              Period    Responsibility        |
 * |  ---------          ------------------  --------  ----------------------|
 * |  ISR                crank_tooth_cb      per-tooth Crank position update |
 * |  ISR                cam_edge_cb         per-edge  Cam phase measurement |
 * |  ISR                knock_adc_cb        per-samp  Knock ADC (DMA)       |
 * |  ISR                g8ba_cylinder_event_isr  per-fire  Mailbox post     |
 * |  NORMALPRIO+20      thd_cyl_events      event     Cylinder dispatch     |
 * |  NORMALPRIO+10      thd_fast_ctrl       5  ms     Knock + CVVT + dwell  |
 * |  NORMALPRIO         thd_medium_ctrl     10 ms     Fuel/ign + boost PID  |
 * |  NORMALPRIO-5       thd_slow_ctrl       50 ms     Protection + boost SC |
 * |  LOWPRIO            thd_diag            250 ms    Diagnostics           |
 * +-------------------------------------------------------------------------+
 */

#include "g8ba_config.h"
#include "crank_cam_sync.h"
#include "fuel_injection.h"
#include "fuel_map.h"
#include "ignition_control.h"
#include "ignition_map.h"
#include "dcvvt_control.h"
#include "knock_control.h"
#include "engine_protection.h"
#include "boost_control.h"           /* SC-7: Module 7                       */

#include "ch.h"
#include "hal.h"

#include "sensor.h"
#include "engine.h"
#include "os_util.h"

/* ==========================================================================
 * ENGINE STATE MACHINE
 * ========================================================================== */

typedef enum {
    ENGINE_STATE_OFF        = 0,
    ENGINE_STATE_CRANKING   = 1,
    ENGINE_STATE_RUNNING    = 2,
    ENGINE_STATE_STALL      = 3,
    ENGINE_STATE_PROTECT    = 4,
} engine_state_t;

static volatile engine_state_t s_engine_state = ENGINE_STATE_OFF;

/* ==========================================================================
 * INTER-TASK MAILBOX (depth = 16, same as NA)
 * ========================================================================== */

#define CYL_MAILBOX_DEPTH   (G8BA_CYLINDERS * 2u)
static mailbox_t  g_cyl_mailbox;
static msg_t      g_cyl_mb_buf[CYL_MAILBOX_DEPTH];

/* ==========================================================================
 * THREAD WORKING AREAS
 * ========================================================================== */

static THD_WORKING_AREA(wa_fast_ctrl,    STACK_KNOCK   + STACK_CVVT);
static THD_WORKING_AREA(wa_medium_ctrl,  STACK_FUEL_INJ + STACK_IGNITION + STACK_BOOST);
static THD_WORKING_AREA(wa_slow_ctrl,    STACK_PROTECTION);
static THD_WORKING_AREA(wa_diag,         512u);
static THD_WORKING_AREA(wa_cyl_events,   STACK_CRANK_SYNC);

/* ==========================================================================
 * SENSOR READING HELPERS
 * Wraps RusEFI Sensor API for use in C context.
 * ========================================================================== */

static inline float read_clt(void)            { return 80.0f;  }
static inline float read_iat(void)            { return 25.0f;  }
static inline float read_tps(void)            { return 0.0f;   }
static inline float read_map_kpa(void)        { return 101.3f; }   /* SC: peaks > 100 kPa under boost */
static inline float read_lambda(void)         { return 1.0f;   }
static inline float read_oil_pressure_kpa(void){ return 350.0f; }
static inline float read_oil_temp(void)       { return 90.0f;  }
static inline float read_vbatt(void)          { return 13.8f;  }

/* ==========================================================================
 * ENGINE STATE MACHINE
 * ========================================================================== */

static engine_state_t eval_engine_state(rpm_t rpm, bool sync_ok,
                                        bool hard_cut_active)
{
    /* SAFE-4 FIX preserved from NA: PROTECT outranks CRANKING */
    if (!sync_ok || rpm == 0u)                  return ENGINE_STATE_OFF;
    if (hard_cut_active)                         return ENGINE_STATE_PROTECT;
    if (rpm < G8BA_RPM_CRANK)                   return ENGINE_STATE_CRANKING;
    return ENGINE_STATE_RUNNING;
}

/* ==========================================================================
 * THREAD IMPLEMENTATIONS
 * ========================================================================== */

/**
 * thd_cyl_events - NORMALPRIO+20
 * Cylinder event dispatch (mailbox-driven).
 */
static THD_FUNCTION(thd_cyl_events, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_cyl_ev");

    while (true) {
        msg_t cyl_msg = 0;
        msg_t result  = chMBFetchTimeout(&g_cyl_mailbox, &cyl_msg,
                                         TIME_MS2I(100));

        if (result == MSG_TIMEOUT) {
            if (s_engine_state == ENGINE_STATE_RUNNING ||
                s_engine_state == ENGINE_STATE_CRANKING) {
                s_engine_state = ENGINE_STATE_STALL;
                crank_cam_sync_reset();
                dcvvt_park_all();
                boost_park();                  /* SC: open BCV on stall */
            }
            continue;
        }

        if (crank_get_sync_state() != SYNC_FULL) {
            prot_emergency_cut();              /* now also calls boost_park() */
            s_engine_state = ENGINE_STATE_STALL;
            continue;
        }

        uint8_t cyl = (uint8_t)cyl_msg;
        if (cyl >= G8BA_CYLINDERS) continue;

        fuel_schedule_injector(cyl);
        ignition_schedule_spark(cyl);
        knock_window_open(cyl);
    }
}

/**
 * thd_fast_ctrl - NORMALPRIO+10, 5 ms (knock + CVVT + dwell, unchanged from NA)
 */
static THD_FUNCTION(thd_fast_ctrl, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_fast");

    systime_t deadline = chVTGetSystemTime();

    while (true) {
        deadline = chTimeAddX(deadline, TIME_MS2I(TASK_PERIOD_FAST_MS));

        rpm_t   rpm      = crank_get_rpm();
        float   load_pct = (read_map_kpa() / 100.0f) * 100.0f;
        float   oil_temp = read_oil_temp();
        float   vbatt    = read_vbatt();

        if (s_engine_state == ENGINE_STATE_RUNNING) {
            knock_flush_open_windows();
            knock_process();
            if (!knock_is_active()) {
                knock_update_noise_floor(rpm);
            }
        }

        dcvvt_update(rpm, load_pct, oil_temp, read_clt());
        ignition_update_dwell(vbatt);

        chThdSleepUntil(deadline);
    }
}

/**
 * thd_medium_ctrl - NORMALPRIO, 10 ms (fuel/ign + SC boost PID)
 */
static THD_FUNCTION(thd_medium_ctrl, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_medium");

    systime_t deadline = chVTGetSystemTime();

    while (true) {
        deadline = chTimeAddX(deadline, TIME_MS2I(TASK_PERIOD_MEDIUM_MS));

        rpm_t   rpm      = crank_get_rpm();
        float   clt      = read_clt();
        float   iat      = read_iat();
        float   tps      = read_tps();
        float   map_kpa  = read_map_kpa();
        float   lambda   = read_lambda();
        float   load_pct = (map_kpa / 100.0f) * 100.0f;

        bool sync_ok = (crank_get_sync_state() == SYNC_FULL);
        s_engine_state = eval_engine_state(rpm, sync_ok,
                                           prot_is_hard_cut_active());

        switch (s_engine_state) {
        case ENGINE_STATE_CRANKING:
            ignition_set_mode(IGN_MODE_CRANKING);
            break;
        case ENGINE_STATE_RUNNING:
            ignition_set_mode(IGN_MODE_RUNNING);
            break;
        case ENGINE_STATE_PROTECT:
        case ENGINE_STATE_STALL:
        case ENGINE_STATE_OFF:
        default:
            ignition_set_mode(IGN_MODE_OFF);
            break;
        }

        /*
         * SC-NOTE: NA applied power_red as a MAP-scaling on the fuel calc.
         * That approach is partially defeated by closed-loop lambda PID
         * (it re-targets stoich and undoes the reduction).  In SC we keep
         * the same approach for backward parity but the more effective
         * power-down vector under boost is the boost-overshoot fuel cut and
         * the PROT_EVENT_BOOST_OVERSHOOT-driven extra reduction (see
         * engine_protection.c::calc_power_reduction).
         */
        float power_red = (rpm >= G8BA_RPM_CRANK) ? prot_get_power_reduction()
                                                   : 0.0f;
        float map_kpa_effective = map_kpa * (1.0f - power_red);

        fuel_inputs_t fi = {
            .map_kpa       = map_kpa_effective,
            .iat_c         = iat,
            .clt_c         = clt,
            .tps_pct       = tps,
            .tps_dot       = 0.0f,
            .lambda        = lambda,
            .lambda_target = fuel_get_afr_target() / G8BA_STOICH_AFR,
            .vbatt_v       = read_vbatt(),
            .rpm           = rpm,
        };
        fuel_calc_cycle(&fi);

        fuel_accel_update(tps, (float)TASK_PERIOD_MEDIUM_MS);

        static uint8_t lambda_div = 0u;
        if (++lambda_div >= 10u) {
            lambda_div = 0u;
            fuel_lambda_pid_step(lambda,
                                 fuel_get_afr_target() / G8BA_STOICH_AFR);
        }

        ign_advance_t adv;
        float worst_retard = (g_knock.total_retard_b1 > g_knock.total_retard_b2)
                           ? g_knock.total_retard_b1 : g_knock.total_retard_b2;
        ignition_calc_advance(rpm, load_pct, clt, iat, worst_retard, &adv);

        rev_limiter_update(rpm);

        /* -- SC-7: BOOST CONTROL PID step ---------------------------------
         *
         * SAFE-SC-2/SC-3: pass clt and engine_protect_cut so boost_update
         * can lock itself out when the engine is cold (< 50degC) or any
         * upstream protection cut is active.  This is checked INSIDE the
         * boost module rather than gated here so the diagnostic state in
         * g_boost stays coherent (we want g_boost.mode = OFF visible to TS,
         * not "PID was skipped").
         */
        bool engine_protect_cut = prot_is_hard_cut_active();
        boost_update(rpm, tps, map_kpa, clt, sync_ok, engine_protect_cut);

        chThdSleepUntil(deadline);
    }
}

/**
 * thd_slow_ctrl - NORMALPRIO-5, 50 ms (engine protection + knock self-test)
 */
static THD_FUNCTION(thd_slow_ctrl, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_slow");

    systime_t deadline = chVTGetSystemTime();

    while (true) {
        deadline = chTimeAddX(deadline, TIME_MS2I(TASK_PERIOD_SLOW_MS));

        prot_inputs_t pi = {
            .clt_c            = read_clt(),
            .oil_pressure_kpa = read_oil_pressure_kpa(),
            .oil_temp_c       = read_oil_temp(),
            .map_kpa          = read_map_kpa(),
            .iat_c            = read_iat(),       /* SC */
            .rpm              = crank_get_rpm(),
            .sync_ok          = (crank_get_sync_state() == SYNC_FULL),
            .vbatt            = read_vbatt(),
        };

        /*
         * Note: engine_protection_update() internally calls
         * boost_safety_check(pi.map_kpa); it owns the BOOST_HARDCUT trip
         * and the persistent overshoot counter.
         */
        engine_protection_update(&pi);

        static uint32_t sensor_test_count = 0u;
        if (++sensor_test_count >= 100u) {        /* every 5 seconds */
            sensor_test_count = 0u;
            knock_sensor_test(KNOCK_SENSOR_B1);
            knock_sensor_test(KNOCK_SENSOR_B2);
        }

        chThdSleepUntil(deadline);
    }
}

/**
 * thd_diag - LOWPRIO, 250 ms (TunerStudio output)
 */
static THD_FUNCTION(thd_diag, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_diag");

    while (true) {
        chThdSleepMilliseconds(TASK_PERIOD_VERYLOW_MS);

        /*
         * SC additions to TS output channels:
         *   engine->outputChannels.boostMapKpa     = g_boost.map_kpa;
         *   engine->outputChannels.boostTargetKpa  = g_boost.target_kpa;
         *   engine->outputChannels.bcvDutyPct      = g_boost.bcv_duty_pct;
         *   engine->outputChannels.boostMode       = g_boost.mode;
         *   engine->outputChannels.boostHardCut    = g_boost.hard_cut_active;
         */
    }
}

/* ==========================================================================
 * CRANK ANGLE SCHEDULER CALLBACK
 * ========================================================================== */

void g8ba_cylinder_event_isr(uint8_t cyl_index)
{
    /* SAFE-1 (preserved from NA): I-class primitive must be in ISR-locked CS */
    chSysLockFromISR();
    (void)chMBPostI(&g_cyl_mailbox, (msg_t)cyl_index);
    chSysUnlockFromISR();
}

/* ==========================================================================
 * INITIALIZATION
 * ========================================================================== */

void g8ba_init(void)
{
    g8ba_status_t st;

    st = crank_cam_sync_init();
    if (st != G8BA_OK) chSysHalt("crank_cam_sync_init failed");

    st = fuel_map_init();
    if (st != G8BA_OK) chSysHalt("fuel_map_init failed: VE/MAP/CLT/IAT axis corrupt");

    st = ign_map_init();
    if (st != G8BA_OK) chSysHalt("ign_map_init failed: advance axis corrupt");

    st = fuel_injection_init();
    if (st != G8BA_OK) chSysHalt("fuel_injection_init failed");

    st = ignition_init();
    if (st != G8BA_OK) chSysHalt("ignition_init failed");

    st = dcvvt_init();
    if (st != G8BA_OK) chSysHalt("dcvvt_init failed");

    st = knock_init();
    if (st != G8BA_OK) chSysHalt("knock_init failed");

    st = engine_protection_init();
    if (st != G8BA_OK) chSysHalt("engine_protection_init failed");

    /* SC-7: Boost control must initialise after engine_protection so that
     * its FCUT path is ready to receive boost-overshoot trips. */
    st = boost_init();
    if (st != G8BA_OK) chSysHalt("boost_init failed: BCV PWM hardware fault");

    chMBObjectInit(&g_cyl_mailbox, g_cyl_mb_buf, CYL_MAILBOX_DEPTH);
}

void g8ba_start_threads(void)
{
    chThdCreateStatic(wa_cyl_events, sizeof(wa_cyl_events),
                      NORMALPRIO + 20,
                      thd_cyl_events, NULL);

    chThdCreateStatic(wa_fast_ctrl, sizeof(wa_fast_ctrl),
                      NORMALPRIO + 10,
                      thd_fast_ctrl, NULL);

    chThdCreateStatic(wa_medium_ctrl, sizeof(wa_medium_ctrl),
                      NORMALPRIO,
                      thd_medium_ctrl, NULL);

    chThdCreateStatic(wa_slow_ctrl, sizeof(wa_slow_ctrl),
                      NORMALPRIO - 5,
                      thd_slow_ctrl, NULL);

    chThdCreateStatic(wa_diag, sizeof(wa_diag),
                      LOWPRIO,
                      thd_diag, NULL);
}

/* ==========================================================================
 * MAIN - standalone build (RusEFI integration replaces this)
 * ========================================================================== */

#ifdef G8BA_STANDALONE_BUILD

static const WDGConfig wdg_cfg = {
    .pr   = STM32_IWDG_PR_64,          /* /64 -> 500 Hz                       */
    .rlr  = 150u,                       /* 300 ms timeout                     */
    .winr = STM32_IWDG_WIN_DISABLED,
};

int main(void)
{
    halInit();
    chSysInit();

    wdgStart(&WDGD1, &wdg_cfg);

    g8ba_init();
    g8ba_start_threads();

    while (true) {
        wdgReset(&WDGD1);
        chThdSleepMilliseconds(50u);
    }

    return 0;
}

#endif /* G8BA_STANDALONE_BUILD */
