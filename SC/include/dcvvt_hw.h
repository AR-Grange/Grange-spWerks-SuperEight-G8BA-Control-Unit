/**
 * @file    dcvvt_hw.h
 * @brief   D-CVVT Hardware Driver — STM32H743 TIM3/TIM4 OCV PWM
 *
 * Maps the four oil-control-valve (OCV) solenoid channels to STM32H743
 * general-purpose timers with direct register access (no HAL).
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Timer / GPIO assignment
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   Phaser             Timer    Channel   GPIO   AF
 *   ─────────────────  ──────   ───────   ────   ──
 *   PHASER_B1_INTAKE   TIM3     CH1       PB4    AF2
 *   PHASER_B1_EXHAUST  TIM3     CH2       PB5    AF2
 *   PHASER_B2_INTAKE   TIM4     CH1       PB6    AF2
 *   PHASER_B2_EXHAUST  TIM4     CH2       PB7    AF2
 *
 *   All four OCV solenoids are active-high, positive logic PWM.
 *   Duty 0% = spring-return (full retard / park).
 *   Duty 50% = hydraulic equilibrium (hold current phase).
 *   Duty 100% = full advance (maximum oil flow in advance direction).
 *
 * ──────────────────────────────────────────────────────────────────────────
 * PWM timing (250 Hz — matches G8BA_CVVT_PWM_HZ in g8ba_config.h)
 * ──────────────────────────────────────────────────────────────────────────
 *
 *   APB1 timer clock : 200 MHz  (STM32H743, HCLK=400 MHz, APB1 /2 × TMR×2)
 *   Prescaler (PSC)  : 99       (÷100 → 2 MHz timer tick)
 *   Auto-reload (ARR): 7999     (÷8000 → 250 Hz period)
 *   Period           : 4 ms
 *
 *   Duty cycle formula:
 *     CCR = (uint32_t)(duty_pct * (float)(DCVVT_HW_ARR + 1u) * 0.01f)
 *     e.g.: 50% → 50 × 8000 × 0.01 = 4000  →  4000/8000 = 50.0% ✓
 *
 *   Special CCR values:
 *     DCVVT_HW_CCR_PARK  =    0  (0% — spring return)
 *     DCVVT_HW_CCR_HOLD  = 4000  (50% — hold position)
 *     DCVVT_HW_CCR_FULL  = 7999  (99.99% — max advance)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Safety behaviour
 * ──────────────────────────────────────────────────────────────────────────
 *   - dcvvt_hw_init() parks all channels (CCR=0), then verifies TIM3/TIM4
 *     responded by reading back CR1.CEN; returns G8BA_ERR_HW on failure.
 *   - dcvvt_hw_set_duty() clamps duty to [0, 100] and guards against NaN/Inf;
 *     fail-safes to spring-return (0%) on invalid floating-point input.
 *   - Both timers run in edge-aligned PWM mode 1 with preload enable.
 *   - ARR preload is enabled (ARPE) — period updates are glitch-free.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Layering
 * ──────────────────────────────────────────────────────────────────────────
 *   HW-1 FIX: this file previously included dcvvt_control.h solely for
 *   phaser_id_t.  That created an upward dependency (hw layer → control layer)
 *   and a latent circular-include risk.  phaser_id_t is now defined in
 *   g8ba_config.h (included below), resolving the layering violation.
 */

#ifndef DCVVT_HW_H
#define DCVVT_HW_H

#include "g8ba_config.h"   /* g8ba_status_t, phaser_id_t (HW-1 FIX) */

/* ══════════════════════════════════════════════════════════════════════════
 * CLOCK AND TIMER CONSTANTS
 * ══════════════════════════════════════════════════════════════════════════ */

/** APB1 timer clock after PLL and prescaler (Hz).
 *  STM32H743: HCLK=400 MHz, APB1 clock = HCLK/4 = 100 MHz,
 *  APB1 timer clock = APB1 × 2 = 200 MHz. */
#define DCVVT_HW_TIM_CLK_HZ     200000000UL

/** Timer prescaler: divides input clock by (PSC+1) = 100 → 2 MHz tick. */
#define DCVVT_HW_PSC            99u

/**
 * Auto-reload register value: (ARR+1) = 8000 → 250 Hz PWM period.
 *
 * HW-0 FIX: was 9999 (200 Hz), contradicting G8BA_CVVT_PWM_HZ = 250 Hz.
 * Corrected: 200 MHz / (100 × 8000) = 250 Hz.
 */
#define DCVVT_HW_ARR            7999u

/** PWM frequency (Hz) — must match G8BA_CVVT_PWM_HZ in g8ba_config.h. */
#define DCVVT_HW_FREQ_HZ        250u

/* Compile-time cross-check: hardware freq must match project config */
#if DCVVT_HW_FREQ_HZ != G8BA_CVVT_PWM_HZ
#  error "DCVVT_HW_FREQ_HZ does not match G8BA_CVVT_PWM_HZ — update one of them"
#endif

/* Compile-time verification: PSC and ARR must give the target frequency */
#if (DCVVT_HW_TIM_CLK_HZ / ((DCVVT_HW_PSC + 1u) * (DCVVT_HW_ARR + 1u))) != DCVVT_HW_FREQ_HZ
#  error "DCVVT_HW_PSC / DCVVT_HW_ARR combination does not yield DCVVT_HW_FREQ_HZ"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * CCR SENTINEL VALUES
 * ══════════════════════════════════════════════════════════════════════════ */

/** CCR value for 0% duty — spring returns phaser to full retard. */
#define DCVVT_HW_CCR_PARK       0u

/**
 * CCR value for exactly 50% duty — hydraulic equilibrium, holds phase.
 *
 * HW-6 FIX: was (ARR+1)/2 - 1 = 3999 (49.99%), inconsistent with the
 * actual CCR produced by dcvvt_hw_set_duty(50.0f) which uses (ARR+1)
 * in its formula giving CCR = 4000 (50.00%).
 * Corrected to (ARR+1)/2 so sentinel and runtime formula agree.
 */
#define DCVVT_HW_CCR_HOLD       ((DCVVT_HW_ARR + 1u) / 2u)   /* 4000 */

/** CCR value for ~100% duty — maximum oil flow, full advance. */
#define DCVVT_HW_CCR_FULL       DCVVT_HW_ARR                  /* 7999 */

/* Sanity checks on sentinel values */
#if DCVVT_HW_CCR_HOLD != 4000u
#  error "DCVVT_HW_CCR_HOLD must be (ARR+1)/2 — check ARR value"
#endif
#if DCVVT_HW_CCR_FULL != 7999u
#  error "DCVVT_HW_CCR_FULL must equal DCVVT_HW_ARR"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * GPIO / ALTERNATE FUNCTION DEFINITIONS
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * All four OCV pins are on GPIOB with AF2 (TIM3/TIM4 mapping).
 *
 *   PB4  — TIM3_CH1 — PHASER_B1_INTAKE   (AF2)
 *   PB5  — TIM3_CH2 — PHASER_B1_EXHAUST  (AF2)
 *   PB6  — TIM4_CH1 — PHASER_B2_INTAKE   (AF2)
 *   PB7  — TIM4_CH2 — PHASER_B2_EXHAUST  (AF2)
 */

/** GPIO alternate function index for TIM3/TIM4 on PB4-PB7. */
#define DCVVT_HW_GPIO_AF        2u

/** GPIOB pin numbers for the four OCV channels. */
#define DCVVT_HW_PIN_B1_IN      4u   /**< PB4 — TIM3 CH1 — B1 Intake  */
#define DCVVT_HW_PIN_B1_EX      5u   /**< PB5 — TIM3 CH2 — B1 Exhaust */
#define DCVVT_HW_PIN_B2_IN      6u   /**< PB6 — TIM4 CH1 — B2 Intake  */
#define DCVVT_HW_PIN_B2_EX      7u   /**< PB7 — TIM4 CH2 — B2 Exhaust */

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise OCV PWM hardware.
 *
 *         - Enables GPIOB and TIM3/TIM4 peripheral clocks via RCC.
 *         - Configures PB4-PB7 as alternate-function push-pull outputs,
 *           very-high-speed, AF2 (TIM3/TIM4).
 *         - Programs TIM3 and TIM4: PSC=99, ARR=7999, PWM mode 1,
 *           output preload enabled, counter auto-reload preload enabled.
 *         - Starts both timers (CR1.CEN=1).
 *         - All four channels start at 0% duty (DCVVT_HW_CCR_PARK).
 *         - Reads back TIM3->CR1 and TIM4->CR1 to verify peripherals
 *           responded; returns G8BA_ERR_HW if CR1.CEN is not set.
 *
 * @return G8BA_OK      — both timers initialised and running.
 *         G8BA_ERR_HW  — TIM3 or TIM4 did not respond (clock not enabled).
 *
 * @note   Must be called before dcvvt_update() or dcvvt_hw_set_duty().
 *         Caller (dcvvt_init) must ensure chSysInit() + halInit() completed.
 */
g8ba_status_t dcvvt_hw_init(void);

/**
 * @brief  Write OCV duty cycle to the hardware PWM channel.
 *
 *         Converts duty_pct to a CCR register value and writes it to
 *         the appropriate TIMx channel compare register.
 *
 *         NaN/Inf safety: if duty_pct is not a finite number the function
 *         parks the channel (0% duty) and returns immediately, preventing
 *         undefined behaviour from the float-to-integer conversion.
 *
 * @param  phaser    Which OCV channel to drive (PHASER_B1_INTAKE … B2_EXHAUST)
 * @param  duty_pct  Desired duty cycle (%). Clamped to [0.0, 100.0].
 *
 * @note   ISR-safe: CCR write is a single 32-bit register store.
 *         No lock required for a single-writer / single-reader channel.
 */
void dcvvt_hw_set_duty(phaser_id_t phaser, float duty_pct);

/**
 * @brief  Park a single OCV channel to 0% duty (spring-return).
 *         Direct register write — no floating-point arithmetic.
 *         Use on fault or emergency shutdown paths.
 *
 * @param  phaser  Which channel to park.
 */
void dcvvt_hw_park(phaser_id_t phaser);

/**
 * @brief  Park all four OCV channels to 0% duty.
 *         Direct register write — no floating-point arithmetic.
 *         Called by dcvvt_park_all() on engine stall / CVVT disable.
 */
void dcvvt_hw_park_all(void);

#endif /* DCVVT_HW_H */
