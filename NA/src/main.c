/**
 * @file    main.c
 * @brief   G8BA ECU Firmware — Main Entry Point & ChibiOS Task Orchestration
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  Task Map                                                               │
 * │                                                                         │
 * │  Priority   Thread              Period    Responsibility                │
 * │  ─────────  ──────────────────  ────────  ────────────────────────      │
 * │  ISR        crank_tooth_cb      per-tooth Crank position update         │
 * │  ISR        cam_edge_cb         per-edge  Cam phase measurement         │
 * │  ISR        knock_adc_cb        per-samp  Knock ADC (DMA complete)      │
 * │  REALTIME   thd_crank_events    event     Cylinder event dispatch       │
 * │  HIGHPRIO   thd_fast_ctrl       5  ms     Knock process + CVVT ctrl     │
 * │  NORMALPRIO thd_medium_ctrl     10 ms     Fuel/ign recalc, rev limiter  │
 * │  LOWPRIO    thd_slow_ctrl       50 ms     Engine protection evaluation  │
 * │  LOWPRIO    thd_diag            250 ms    Diagnostics + TunerStudio     │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * RusEFI integration notes:
 *   - This file replaces/extends rusefi's main_trigger_callback.cpp
 *   - Call g8ba_init() from RusEFI's initEngineController()
 *   - Register crank_tooth_callback with RusEFI's TriggerCentral listener
 *   - Fuel/ignition scheduling integrates with RusEFI's angle scheduler
 */

#include "g8ba_config.h"
#include "crank_cam_sync.h"
#include "fuel_injection.h"
#include "fuel_map.h"          /* FM-3 FIX: table sanity check at startup */
#include "ignition_control.h"
#include "ignition_map.h"      /* FM-3 FIX: knock retard init, table sanity check */
#include "dcvvt_control.h"
#include "knock_control.h"
#include "engine_protection.h"

/* ── ChibiOS ────────────────────────────────────────────────────────────── */
#include "ch.h"
#include "hal.h"

/* ── RusEFI ─────────────────────────────────────────────────────────────── */
#include "sensor.h"
#include "engine.h"
#include "os_util.h"

/* ══════════════════════════════════════════════════════════════════════════
 * ENGINE STATE MACHINE
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    ENGINE_STATE_OFF        = 0,
    ENGINE_STATE_CRANKING   = 1,
    ENGINE_STATE_RUNNING    = 2,
    ENGINE_STATE_STALL      = 3,
    ENGINE_STATE_PROTECT    = 4,  /* protection cutoff active */
} engine_state_t;

static volatile engine_state_t s_engine_state = ENGINE_STATE_OFF;

/* ══════════════════════════════════════════════════════════════════════════
 * INTER-TASK MAILBOX  (replaces old event-flag scheme — C-2 fix)
 *
 * A ChibiOS mailbox passes the cyl_index from the angle-scheduler ISR to
 * thd_cyl_events. This eliminates the C-2 bug where cyl_index was discarded
 * by the ISR and the thread had to fall back on the stale
 * g_engine_pos.current_cylinder (updated only 2× per 720° instead of 8×).
 *
 * Depth = G8BA_CYLINDERS × 2 (16 slots) to absorb brief pre-emption bursts
 * without losing events (chMBPostI never blocks — it returns MSG_TIMEOUT if
 * the buffer is full, which is handled gracefully by the ISR discarding the
 * oldest-safe entry).
 * ══════════════════════════════════════════════════════════════════════════ */

#define CYL_MAILBOX_DEPTH   (G8BA_CYLINDERS * 2u)
static mailbox_t  g_cyl_mailbox;
static msg_t      g_cyl_mb_buf[CYL_MAILBOX_DEPTH];

/* ══════════════════════════════════════════════════════════════════════════
 * THREAD WORKING AREAS
 * ══════════════════════════════════════════════════════════════════════════ */

static THD_WORKING_AREA(wa_fast_ctrl,    STACK_KNOCK   + STACK_CVVT);
static THD_WORKING_AREA(wa_medium_ctrl,  STACK_FUEL_INJ + STACK_IGNITION);
static THD_WORKING_AREA(wa_slow_ctrl,    STACK_PROTECTION);
static THD_WORKING_AREA(wa_diag,         512u);
static THD_WORKING_AREA(wa_cyl_events,   STACK_CRANK_SYNC);

/* ══════════════════════════════════════════════════════════════════════════
 * SENSOR READING HELPERS
 * Wraps RusEFI Sensor API for use in C context.
 * ══════════════════════════════════════════════════════════════════════════ */

static inline float read_clt(void)
{
    /* C++ RusEFI: Sensor::get(SensorType::Clt).value_or(20.0f) */
    return 80.0f; /* placeholder — replace with actual RusEFI C sensor call */
}

static inline float read_iat(void)
{
    return 25.0f; /* placeholder */
}

static inline float read_tps(void)
{
    return 0.0f; /* placeholder */
}

static inline float read_map_kpa(void)
{
    return 101.3f; /* placeholder */
}

static inline float read_lambda(void)
{
    return 1.0f; /* placeholder */
}

static inline float read_oil_pressure_kpa(void)
{
    return 350.0f; /* placeholder */
}

static inline float read_oil_temp(void)
{
    return 90.0f; /* placeholder */
}

static inline float read_vbatt(void)
{
    return 13.8f; /* placeholder */
}

/* ══════════════════════════════════════════════════════════════════════════
 * ENGINE STATE MACHINE
 * ══════════════════════════════════════════════════════════════════════════ */

static engine_state_t eval_engine_state(rpm_t rpm, bool sync_ok,
                                        bool hard_cut_active)
{
    /*
     * SAFE-4 FIX: protection takes priority over cranking.
     *
     * Previous order:
     *   1. !sync_ok || rpm == 0  → OFF
     *   2. rpm < CRANK           → CRANKING
     *   3. hard_cut_active       → PROTECT
     * Failure case: a hard-cut condition that asserts during the cranking
     * window (e.g. low oil pressure detected on the very first start, or a
     * sync-loss-while-still-spinning-down with rpm < 400) fell through to
     * CRANKING in step 2, never reaching the PROTECT check.  The state
     * machine then commanded ignition_set_mode(IGN_MODE_CRANKING) which
     * applies the cranking-advance and turns the coils back ON, even though
     * a protective fuel-cut was set in parallel.  Net result: spark with no
     * fuel after a few revolutions — undesirable, and inconsistent with the
     * documented intent of PROTECT (full ignition off).
     *
     * Correct order: hard_cut_active is the highest-priority transition
     * (after the no-sync OFF case) — protection always wins over normal
     * operating modes.
     */
    if (!sync_ok || rpm == 0u)                  return ENGINE_STATE_OFF;
    if (hard_cut_active)                         return ENGINE_STATE_PROTECT;
    if (rpm < G8BA_RPM_CRANK)                   return ENGINE_STATE_CRANKING;
    return ENGINE_STATE_RUNNING;
}

/* ══════════════════════════════════════════════════════════════════════════
 * THREAD IMPLEMENTATIONS
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * thd_cyl_events — REALTIME priority
 * Waits for cylinder-ready event from crank angle scheduler ISR,
 * then dispatches fuel injection and ignition scheduling for that cylinder.
 *
 * This thread has no periodic sleep — it wakes purely on hardware events.
 */
static THD_FUNCTION(thd_cyl_events, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_cyl_ev");

    while (true) {
        /*
         * C-2 FIX: Fetch cyl_index from the mailbox posted by the ISR.
         *
         * Original code used chEvtWaitAnyTimeout() and then read
         * g_engine_pos.current_cylinder, which is only updated at gap
         * tooth (tooth_index == 0) — at most twice per 720° cycle.
         * This meant 6 of 8 fuel/ignition events went to the wrong cylinder.
         *
         * Fix: the ISR posts the exact cyl_index it received from RusEFI's
         * angle scheduler. The 100ms timeout remains as stall detection.
         */
        msg_t cyl_msg = 0;
        msg_t result  = chMBFetchTimeout(&g_cyl_mailbox, &cyl_msg,
                                         TIME_MS2I(100));

        if (result == MSG_TIMEOUT) {
            /* No cylinder events for 100ms — engine has stalled */
            if (s_engine_state == ENGINE_STATE_RUNNING ||
                s_engine_state == ENGINE_STATE_CRANKING) {
                s_engine_state = ENGINE_STATE_STALL;
                crank_cam_sync_reset();
                dcvvt_park_all();
            }
            continue;
        }

        /* Validate sync before dispatching — sync may have been lost between
         * when the ISR fired and when this thread was scheduled. */
        if (crank_get_sync_state() != SYNC_FULL) {
            prot_emergency_cut();
            s_engine_state = ENGINE_STATE_STALL;
            continue;
        }

        uint8_t cyl = (uint8_t)cyl_msg;
        if (cyl >= G8BA_CYLINDERS) continue;  /* ISR safety guard */

        /* Schedule fuel injection for this cylinder */
        fuel_schedule_injector(cyl);

        /* Schedule ignition events (dwell + spark) */
        ignition_schedule_spark(cyl);

        /*
         * C-3 FIX: Knock window MUST be gated by crank angle, not time.
         *
         * Problem: the old code called knock_window_close() from thd_fast_ctrl
         * every 5ms. At 7800 RPM the knock window (10°–60° ATDC) spans only
         * 1.07ms; a 5ms timer fires up to 4.6× after the window should have
         * closed, leaving the detector armed into the next cylinder's power
         * stroke and generating false knock events.
         *
         * Correct implementation via RusEFI angle scheduler:
         *   scheduleByAngle(&s_knock_open_ev[cyl],
         *                   crank_tdc_angle(cyl) + G8BA_KNOCK_WINDOW_ATDC,
         *                   knock_window_open,  cyl);
         *   scheduleByAngle(&s_knock_close_ev[cyl],
         *                   crank_tdc_angle(cyl) + G8BA_KNOCK_WINDOW_END,
         *                   knock_window_close, cyl);
         *
         * knock_window_open() and knock_window_close() are ISR-safe (no
         * blocking, no malloc). knock_process() remains in thd_fast_ctrl
         * where it polls KNOCK_WIN_PROCESS state — no timing constraint.
         *
         * Until scheduleByAngle() is wired: open at the cylinder event angle
         * (a conservative approximation — the window may capture a few degrees
         * before TDC, but will not bleed into an adjacent cylinder).
         */
        knock_window_open(cyl);
        /*
         * scheduleByAngle(&s_knock_close_ev[cyl],
         *                 crank_tdc_angle(cyl) + G8BA_KNOCK_WINDOW_END,
         *                 knock_window_close, cyl);
         */
    }
}

/**
 * thd_fast_ctrl — HIGHPRIO, 5ms period
 * Time-critical control loops: knock processing and CVVT position control.
 * Must complete within TASK_PERIOD_FAST_MS to maintain control bandwidth.
 */
static THD_FUNCTION(thd_fast_ctrl, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_fast");

    systime_t deadline = chVTGetSystemTime();

    while (true) {
        deadline = chTimeAddX(deadline, TIME_MS2I(TASK_PERIOD_FAST_MS));

        rpm_t   rpm      = crank_get_rpm();
        float   load_pct = (read_map_kpa() / 100.0f) * 100.0f; /* MAP-based */
        float   oil_temp = read_oil_temp();
        float   vbatt    = read_vbatt();

        /* ── Knock control ──────────────────────────────────────────────── */
        if (s_engine_state == ENGINE_STATE_RUNNING) {
            /*
             * R-1 FIX: Flush any open sampling windows before processing.
             *
             * knock_window_open() is called in thd_cyl_events, but
             * knock_window_close() (angle-scheduled) is not yet wired —
             * the scheduleByAngle() call is commented out in the C-3 block.
             * Without a close, window_state stays KNOCK_WIN_SAMPLING and
             * knock_process() (which checks for KNOCK_WIN_PROCESS) never
             * acts — leaving the engine with zero knock retard at all times.
             *
             * knock_flush_open_windows() transitions any SAMPLING windows
             * to PROCESS state so knock_process() can evaluate them below.
             * Remove this call once the angle-scheduled close is wired.
             *
             * C-3 NOTE: The window captured here may be shorter than the
             * designed 10°–60° ATDC window, but all knock energy seen
             * between the cylinder event and this 5ms tick is preserved.
             */
            knock_flush_open_windows();
            knock_process();

            /* Update noise floor during normal operation */
            if (!knock_is_active()) {
                knock_update_noise_floor(rpm);
            }
        }

        /* ── D-CVVT control ─────────────────────────────────────────────── */
        /* M-1 fix propagated: pass CLT so dcvvt_is_enabled() can gate on it */
        dcvvt_update(rpm, load_pct, oil_temp, read_clt());

        /* ── Dwell adjustment ───────────────────────────────────────────── */
        ignition_update_dwell(vbatt);

        /* Sleep until next deadline (absolute-time sleep for jitter control) */
        chThdSleepUntil(deadline);
    }
}

/**
 * thd_medium_ctrl — NORMALPRIO, 10ms period
 * Fuel and ignition recalculation, rev limiter evaluation, lambda PID.
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

        /* ── Engine state evaluation ─────────────────────────────────────── */
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

        /* ── Fuel calculation ────────────────────────────────────────────── */
        /*
         * R-2 FIX: Apply thermal power reduction to MAP input.
         *
         * prot_get_power_reduction() returns 0.0–1.0 when CLT or oil temp
         * exceeds warning thresholds (linear 0–50% from CLT_WARN to CLT_CUT,
         * plus 20% flat for oil temp warning). Previously this value was
         * calculated in engine_protection.c but never consumed — the engine
         * ran at full power from 105 °C warning all the way to 118 °C hard cut.
         *
         * Reducing map_kpa proportionally lowers both the VE table lookup
         * pressure and the base pulse width in a consistent, self-correcting
         * way: closed-loop lambda will maintain stoich at the reduced power
         * level without lean/rich excursions.
         *
         * Only apply when above idle RPM (protection module already gates
         * on RPM, but guard here for safety).
         */
        float power_red = (rpm >= G8BA_RPM_CRANK) ? prot_get_power_reduction()
                                                   : 0.0f;
        float map_kpa_effective = map_kpa * (1.0f - power_red);

        fuel_inputs_t fi = {
            .map_kpa       = map_kpa_effective,
            .iat_c         = iat,
            .clt_c         = clt,
            .tps_pct       = tps,
            .tps_dot       = 0.0f,   /* calculated internally by accel update */
            .lambda        = lambda,
            .lambda_target = fuel_get_afr_target() / G8BA_STOICH_AFR,
            .vbatt_v       = read_vbatt(),   /* FC-3 FIX: dead-time compensation */
            .rpm           = rpm,
        };
        fuel_calc_cycle(&fi);

        /* ── Acceleration enrichment update ─────────────────────────────── */
        fuel_accel_update(tps, (float)TASK_PERIOD_MEDIUM_MS);

        /* ── Lambda closed-loop (100ms cadence — every 10th call) ────────── */
        static uint8_t lambda_div = 0u;
        if (++lambda_div >= 10u) {
            lambda_div = 0u;
            fuel_lambda_pid_step(lambda,
                                 fuel_get_afr_target() / G8BA_STOICH_AFR);
        }

        /* ── Ignition advance calculation ───────────────────────────────── */
        ign_advance_t adv;
        /* Use worst-case bank retard for safety (all cylinders get it) */
        float worst_retard = (g_knock.total_retard_b1 > g_knock.total_retard_b2)
                           ? g_knock.total_retard_b1 : g_knock.total_retard_b2;
        ignition_calc_advance(rpm, load_pct, clt, iat, worst_retard, &adv);

        /* ── Rev limiter ─────────────────────────────────────────────────── */
        rev_limiter_update(rpm);

        chThdSleepUntil(deadline);
    }
}

/**
 * thd_slow_ctrl — LOWPRIO, 50ms period
 * Engine protection evaluation, sensor plausibility checks.
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
            .rpm              = crank_get_rpm(),
            .sync_ok          = (crank_get_sync_state() == SYNC_FULL),
            .vbatt            = read_vbatt(),
        };

        engine_protection_update(&pi);

        /* Knock sensor self-test at startup and periodically */
        static uint32_t sensor_test_count = 0u;
        if (++sensor_test_count >= 100u) { /* every 5 seconds */
            sensor_test_count = 0u;
            knock_sensor_test(KNOCK_SENSOR_B1);
            knock_sensor_test(KNOCK_SENSOR_B2);
        }

        chThdSleepUntil(deadline);
    }
}

/**
 * thd_diag — IDLEPRIO, 250ms period
 * Non-critical diagnostics, TunerStudio output channel updates.
 * Lowest priority — will not interfere with real-time tasks.
 */
static THD_FUNCTION(thd_diag, arg)
{
    (void)arg;
    chRegSetThreadName("g8ba_diag");

    while (true) {
        chThdSleepMilliseconds(TASK_PERIOD_VERYLOW_MS);

        /*
         * Update RusEFI output channels for TunerStudio monitoring:
         *
         * engine->outputChannels.rpm              = crank_get_rpm();
         * engine->outputChannels.knockLevel       = g_knock.total_retard_b1;
         * engine->outputChannels.ignitionAdvance  = g_ignition.coils[0].advance_deg;
         * engine->outputChannels.injectorDutyCycle = g_fuel.inj_dc_pct;
         * engine->outputChannels.vvtPositionB1IN  = g_cvvt.phasers[PHASER_B1_INTAKE].actual_deg;
         * engine->outputChannels.vvtPositionB1EX  = g_cvvt.phasers[PHASER_B1_EXHAUST].actual_deg;
         * engine->outputChannels.vvtPositionB2IN  = g_cvvt.phasers[PHASER_B2_INTAKE].actual_deg;
         * engine->outputChannels.vvtPositionB2EX  = g_cvvt.phasers[PHASER_B2_EXHAUST].actual_deg;
         * engine->outputChannels.engineProtection = g_protection.overall_level;
         *
         * CAN broadcast (if CAN bus configured):
         * can_tx_engine_status();
         */
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * CRANK ANGLE SCHEDULER CALLBACK
 * Called by RusEFI's angle-based scheduler when a cylinder event fires.
 * Context: interrupt-safe (called from TriggerCentral ISR or fast timer)
 * ══════════════════════════════════════════════════════════════════════════ */

void g8ba_cylinder_event_isr(uint8_t cyl_index)
{
    /*
     * C-2 FIX: post cyl_index to the thread mailbox.
     *
     * Original: (void)cyl_index discarded the index supplied by RusEFI's
     * angle scheduler. The thread then read g_engine_pos.current_cylinder,
     * which is only updated at gap-tooth (tooth_index == 0) — at most twice
     * per 720° cycle instead of 8×. This meant 6 of every 8 fuel/ignition
     * events were dispatched to the WRONG cylinder.
     *
     * SAFE-1 FIX: chMBPostI() is an *I-class* ChibiOS primitive; it must be
     * invoked inside an ISR critical section established by
     * chSysLockFromISR()/chSysUnlockFromISR().  The previous implementation
     * called it bare from ISR context — in DEBUG builds the kernel state
     * check (CH_DBG_SYSTEM_STATE_CHECK) asserts; in release builds the
     * mailbox internal counters can be corrupted if a higher-priority ISR
     * preempts mid-update, leading to lost or duplicated cylinder events.
     *
     * If the mailbox is temporarily full (burst during pre-emption) the
     * event is dropped (chMBPostI returns MSG_TIMEOUT) — preferred over
     * blocking in an ISR.
     */
    chSysLockFromISR();
    (void)chMBPostI(&g_cyl_mailbox, (msg_t)cyl_index);
    chSysUnlockFromISR();
}

/* ══════════════════════════════════════════════════════════════════════════
 * INITIALIZATION
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise all G8BA ECU modules.
 *         Call this from RusEFI's initEngineController() after the
 *         hardware abstraction layer (HAL) is configured.
 */
void g8ba_init(void)
{
    /*
     * R-3 FIX: check every module init return value.
     *
     * Previously all six inits were called with return values discarded.
     * A failed init (e.g. ADC configuration fault, NULL pointer in HAL) left
     * the system appearing to start normally while the affected module was in
     * an undefined state — potentially commanding open injectors, floating
     * coils, or uninitialized protection thresholds.
     *
     * Failsafe response: if any module fails, set all fuel cuts and force
     * ignition off before the threads start. The watchdog will reboot the MCU
     * after 300ms if the error persists (no threads = no wdgReset).
     *
     * In a full RusEFI integration, replace chSysHalt() with a RusEFI fault
     * reporting call (firmwareError()) to log the fault code before halting.
     */
    g8ba_status_t st;

    st = crank_cam_sync_init();  /* must be first — others depend on position */
    if (st != G8BA_OK) chSysHalt("crank_cam_sync_init failed");

    /* FM-3 FIX: initialise calibration-table modules before control modules */
    st = fuel_map_init();
    if (st != G8BA_OK) chSysHalt("fuel_map_init failed: VE/MAP axis corrupt");

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

    /* Initialise ChibiOS mailbox for ISR→thread cylinder-index delivery (C-2) */
    chMBObjectInit(&g_cyl_mailbox, g_cyl_mb_buf, CYL_MAILBOX_DEPTH);
}

/**
 * @brief  Start all ChibiOS threads.
 *         Call after g8ba_init() and chSysInit().
 *
 * Thread priority table:
 *   HIGHPRIO   = NORMALPRIO + 10  (time-critical, never blocks long)
 *   NORMALPRIO = 64               (standard tasks)
 *   LOWPRIO    = NORMALPRIO - 5   (background tasks)
 *   IDLEPRIO   = 1                (diagnostics)
 */
void g8ba_start_threads(void)
{
    /* Cylinder event dispatcher — highest user-thread priority */
    chThdCreateStatic(wa_cyl_events, sizeof(wa_cyl_events),
                      NORMALPRIO + 20,
                      thd_cyl_events, NULL);

    /* Fast control (knock + CVVT) */
    chThdCreateStatic(wa_fast_ctrl, sizeof(wa_fast_ctrl),
                      NORMALPRIO + 10,
                      thd_fast_ctrl, NULL);

    /* Medium control (fuel + ignition + rev limit) */
    chThdCreateStatic(wa_medium_ctrl, sizeof(wa_medium_ctrl),
                      NORMALPRIO,
                      thd_medium_ctrl, NULL);

    /* Slow control (engine protection) */
    chThdCreateStatic(wa_slow_ctrl, sizeof(wa_slow_ctrl),
                      NORMALPRIO - 5,
                      thd_slow_ctrl, NULL);

    /* Diagnostics + TunerStudio output (lowest priority) */
    chThdCreateStatic(wa_diag, sizeof(wa_diag),
                      LOWPRIO,
                      thd_diag, NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * MAIN — Entry point when running as standalone (non-RusEFI) build
 * In normal RusEFI integration, main() is owned by RusEFI itself.
 * This main() is provided for unit testing / simulator builds only.
 * ══════════════════════════════════════════════════════════════════════════ */

#ifdef G8BA_STANDALONE_BUILD

/*
 * H-7 FIX: Watchdog configuration.
 *
 * STM32H7 IWDG runs on the 32 kHz LSI oscillator.
 *   PR = 64  →  32000 / 64  = 500 ticks/second
 *   RLR = 150 → 150 / 500  = 300ms timeout
 *
 * The main loop feeds the watchdog every 50ms — well within the timeout.
 * A locked-up thread (stuck in an infinite loop or blocked ISR) will
 * cause the MCU to reset within 300ms.
 *
 * The watchdog was previously commented out (wdgReset left as a comment),
 * providing zero protection against firmware hangs in a safety-critical
 * ECU. This is now enabled unconditionally in standalone builds.
 */
static const WDGConfig wdg_cfg = {
    .pr   = STM32_IWDG_PR_64,          /* prescaler: /64 → 500 Hz */
    .rlr  = 150u,                       /* reload: 150 × 2ms = 300ms timeout */
    .winr = STM32_IWDG_WIN_DISABLED,    /* no early-window restriction */
};

int main(void)
{
    /* ChibiOS board and HAL initialisation */
    halInit();
    chSysInit();

    /* H-7: Start hardware watchdog before anything else — if init hangs,
     * the MCU resets cleanly rather than staying in an undefined state. */
    wdgStart(&WDGD1, &wdg_cfg);

    /* G8BA ECU module initialisation */
    g8ba_init();

    /* Start all control threads */
    g8ba_start_threads();

    /*
     * Main thread becomes the watchdog feeder.
     * All real work happens in the spawned threads above.
     * If any thread deadlocks and starves this loop for > 300ms, the
     * watchdog fires a full MCU reset — a safe-state for an ECU.
     */
    while (true) {
        wdgReset(&WDGD1);               /* H-7 FIX: watchdog feed enabled */
        chThdSleepMilliseconds(50u);
    }

    return 0; /* unreachable */
}

#endif /* G8BA_STANDALONE_BUILD */
