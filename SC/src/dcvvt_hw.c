/**
 * @file    dcvvt_hw.c
 * @brief   D-CVVT Hardware Driver — STM32H743 TIM3/TIM4 OCV PWM
 *
 * Direct register implementation — no STM32 HAL.
 * CMSIS device header (stm32h743xx.h) provides TIM_TypeDef, RCC_TypeDef,
 * GPIO_TypeDef struct definitions and the peripheral base addresses
 * (TIM3, TIM4, GPIOB, RCC).  These are brought in through hal.h which
 * ChibiOS includes as part of its STM32 HAL port.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * PWM frequency: 250 Hz (HW-0 FIX — was 200 Hz)
 * ──────────────────────────────────────────────────────────────────────────
 *   APB1 timer clock : 200 MHz
 *   PSC = 99  → timer tick = 200 MHz / 100  = 2 MHz
 *   ARR = 7999 → period    = 2 MHz   / 8000 = 250 Hz (4 ms)
 *
 * ──────────────────────────────────────────────────────────────────────────
 * Register writes that need explanation
 * ──────────────────────────────────────────────────────────────────────────
 *
 * CCMR1 (Capture/Compare Mode Register 1):
 *   OC1M[2:0] = 0b110 (bits [6:4])  → PWM mode 1 (active while CNT < CCR)
 *   OC1PE     = 1     (bit  3)       → Output compare 1 preload enable
 *   OC2M[2:0] = 0b110 (bits [14:12])→ PWM mode 1 for CH2
 *   OC2PE     = 1     (bit  11)      → Output compare 2 preload enable
 *
 *   Combined value: 0x6868u
 *     bits 6,5,4 = 110 → CH1 PWM mode 1   (0x0060)
 *     bit  3     = 1   → CH1 preload       (0x0008)
 *     bits 14,13,12 = 110 → CH2 PWM mode 1(0x6000)
 *     bit  11    = 1   → CH2 preload       (0x0800)
 *     Total: 0x6868
 *
 * CCER (Capture/Compare Enable Register):
 *   CC1E = 1 (bit 0) → CH1 output enable
 *   CC2E = 1 (bit 4) → CH2 output enable
 *   Value: 0x0011u
 *
 * CR1 (Control Register 1):
 *   ARPE = 1 (bit 7) → Auto-reload preload enable (smooth ARR updates)
 *   CEN  = 1 (bit 0) → Counter enable (start timer)
 *   Value: 0x0081u
 *
 * EGR (Event Generation Register):
 *   UG = 1 (bit 0) → Force update event, loads PSC/ARR into shadow regs.
 *   Self-clears immediately after write.
 *
 * RCC_APB1LENR:
 *   TIM3EN (bit 1) and TIM4EN (bit 2) enable APB1 peripheral clocks.
 *
 * GPIOB_MODER:
 *   2 bits per pin.  Mode 0b10 = Alternate Function.
 *   Pins 4-7: bits [15:8].  Set all four to AF (0b10 each = 0xAA << 8).
 *
 * GPIOB_AFRL (AFR[0]):
 *   4 bits per pin, covering pins 0-7.
 *   AF2 = 0x2.  Pins 4-7 occupy bits [31:16].
 *   AFRL for pins 4-7 = (2<<16)|(2<<20)|(2<<24)|(2<<28) = 0x22220000u
 *
 * GPIOB_OSPEEDR:
 *   2 bits per pin.  0b11 = Very-high speed.
 *   Pins 4-7: bits [15:8].  0xFF << 8 = 0xFF00 = four × 0b11.
 */

#include "hal.h"             /* ChibiOS HAL — brings in stm32h743xx.h, GPIO, RCC */
#include <math.h>            /* isfinite() — HW-3 NaN guard                       */
#include "dcvvt_hw.h"

/* ══════════════════════════════════════════════════════════════════════════
 * INTERNAL HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * CCMR1 value enabling PWM mode 1 with preload for both CH1 and CH2.
 *   OC1M=0b110 → bits [6:4], OC1PE → bit 3
 *   OC2M=0b110 → bits [14:12], OC2PE → bit 11
 */
#define CCMR1_PWM_CH1_CH2   0x6868u

/*
 * CCER value enabling CH1 and CH2 outputs (active-high, no inversion).
 *   CC1E (bit 0) = 1, CC2E (bit 4) = 1
 */
#define CCER_CH1_CH2_EN     0x0011u

/*
 * CR1 value: ARPE (bit 7) + CEN (bit 0) — start timer, enable ARR preload.
 */
#define CR1_ARPE_CEN        0x0081u

/* ──────────────────────────────────────────────────────────────────────────
 * gpio_config_af2_pb4_pb7
 *   Configure GPIOB pins 4-7 as alternate function (AF2), very-high speed,
 *   push-pull, no pull-up/down.
 * ──────────────────────────────────────────────────────────────────────────
 */
static void gpio_config_af2_pb4_pb7(void)
{
    /* Enable GPIOB peripheral clock (AHB4) */
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
    __DSB();   /* data sync barrier — ensure clock enable is visible */

    /*
     * MODER: set pins 4-7 to Alternate Function (0b10 each).
     * Pins 4-7 occupy MODER bits [15:8] (2 bits × 4 pins).
     * Clear those 8 bits, then OR in 0xAA = 10101010b (four × 0b10).
     */
    GPIOB->MODER = (GPIOB->MODER & ~(0xFFu << 8u)) | (0xAAu << 8u);

    /*
     * AFRL (AFR[0]): assign AF2 to pins 4-7.
     * Each pin occupies 4 bits; pins 4-7 are in bits [31:16].
     * AF2 = 0x2 per pin → 0x22220000 for all four pins.
     */
    GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(0xFFFF0000u)) | (0x22220000u);

    /*
     * OSPEEDR: set pins 4-7 to Very-high speed (0b11 each).
     * Pins 4-7 occupy OSPEEDR bits [15:8].
     * 0xFF = 11111111b = four × 0b11.
     */
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(0xFFu << 8u)) | (0xFFu << 8u);

    /*
     * PUPDR: pins 4-7 — no pull (0b00 each, reset default, but be explicit).
     */
    GPIOB->PUPDR &= ~(0xFFu << 8u);
}

/* ──────────────────────────────────────────────────────────────────────────
 * tim_init_pwm_250hz
 *   Common TIM initialisation for 250 Hz PWM on CH1 + CH2.
 *   Called for both TIM3 (Bank1) and TIM4 (Bank2).
 * ──────────────────────────────────────────────────────────────────────────
 */
static void tim_init_pwm_250hz(TIM_TypeDef *tim)
{
    /* Ensure timer is stopped before configuration */
    tim->CR1  = 0u;

    /* Prescaler: divide 200 MHz by 100 → 2 MHz timer tick */
    tim->PSC  = DCVVT_HW_PSC;

    /* Auto-reload: 2 MHz / 8000 → 250 Hz (HW-0 FIX: was 9999 → 200 Hz) */
    tim->ARR  = DCVVT_HW_ARR;

    /* PWM mode 1 + output preload for CH1 and CH2 */
    tim->CCMR1 = CCMR1_PWM_CH1_CH2;

    /* Start all channels at 0% (spring-return / park) */
    tim->CCR1 = DCVVT_HW_CCR_PARK;
    tim->CCR2 = DCVVT_HW_CCR_PARK;

    /* Enable CH1 and CH2 outputs */
    tim->CCER = CCER_CH1_CH2_EN;

    /* Force update event — loads PSC and ARR into their shadow registers */
    tim->EGR  = TIM_EGR_UG;

    /* Clear update interrupt flag set by EGR write (not using interrupts,
     * but keep the status register clean) */
    tim->SR   = 0u;

    /* Enable timer: ARR preload on, counter running */
    tim->CR1  = CR1_ARPE_CEN;
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t dcvvt_hw_init(void)
{
    /* Enable TIM3 and TIM4 peripheral clocks on APB1L bus */
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN | RCC_APB1LENR_TIM4EN;
    __DSB();   /* wait for clock enable to propagate before register access */

    /* Configure GPIOB PB4-PB7 as TIM3/TIM4 AF2 outputs */
    gpio_config_af2_pb4_pb7();

    /* Initialise TIM3 — Bank1 (B1_IN on CH1, B1_EX on CH2) */
    tim_init_pwm_250hz(TIM3);

    /* Initialise TIM4 — Bank2 (B2_IN on CH1, B2_EX on CH2) */
    tim_init_pwm_250hz(TIM4);

    /*
     * HW-2 FIX: Verify that both timers responded to configuration.
     *
     * After tim_init_pwm_250hz(), CR1 should have CEN=1 (bit 0) set.
     * If the APB1 clock enable failed (e.g., RCC peripheral gate stuck),
     * the register bus returns 0x0000 and CR1.CEN remains clear.
     *
     * This is a lightweight sanity check, not a full self-test.
     * A more complete test (verify ARR/PSC readback) can be added on dyno.
     */
    if (((TIM3->CR1 & TIM_CR1_CEN) == 0u) ||
        ((TIM4->CR1 & TIM_CR1_CEN) == 0u)) {
        return G8BA_ERR_HW;
    }

    return G8BA_OK;
}

/* ─────────────────────────────────────────────────────────────────────────
 * dcvvt_hw_set_duty
 * ─────────────────────────────────────────────────────────────────────────
 * Map:
 *   PHASER_B1_INTAKE  → TIM3->CCR1
 *   PHASER_B1_EXHAUST → TIM3->CCR2
 *   PHASER_B2_INTAKE  → TIM4->CCR1
 *   PHASER_B2_EXHAUST → TIM4->CCR2
 *
 * CCR formula: CCR = (uint32_t)(duty_pct * (ARR+1) * 0.01f)
 *   Using (ARR+1) instead of ARR ensures 50% duty → CCR = 4000 exactly,
 *   consistent with DCVVT_HW_CCR_HOLD = 4000.  (HW-6 FIX)
 * ─────────────────────────────────────────────────────────────────────────
 */
void dcvvt_hw_set_duty(phaser_id_t phaser, float duty_pct)
{
    /*
     * HW-3 FIX: NaN/Inf guard.
     *
     * If duty_pct is NaN or ±Inf (propagated from a cam sensor fault through
     * the PID chain), the clamp comparisons below are silently skipped
     * (NaN comparisons always evaluate to false).  The subsequent
     * float→integer cast of NaN is undefined behaviour in C — on ARM Cortex-M7
     * it typically saturates to 0 or 0x7FFFFFFF, but both outcomes are
     * wrong (0 = unexpected park; max = latch at full advance).
     *
     * Fail-safe action: park the channel (spring-return, 0% duty).
     * The control loop will recover on the next cycle if sensor data is valid.
     */
    if (!isfinite(duty_pct)) {
        dcvvt_hw_park(phaser);
        return;
    }

    /* Clamp duty to valid range */
    if (duty_pct < 0.0f)   duty_pct = 0.0f;
    if (duty_pct > 100.0f) duty_pct = 100.0f;

    /*
     * Convert to CCR count.
     * Formula: CCR = duty_pct / 100 × (ARR+1)
     *   → duty_pct × (ARR+1) × 0.01
     * At 50%: 50 × 8000 × 0.01 = 4000.0 → CCR=4000 (50.0%) ✓
     * At 100%: 100 × 8000 × 0.01 = 8000 → capped to ARR=7999 (99.99%) ✓
     */
    uint32_t ccr = (uint32_t)(duty_pct * (float)(DCVVT_HW_ARR + 1u) * 0.01f);

    /* Guard: cap at ARR in case of floating-point rounding overshoot */
    if (ccr > DCVVT_HW_ARR) ccr = DCVVT_HW_ARR;

    switch (phaser) {
        case PHASER_B1_INTAKE:   TIM3->CCR1 = ccr; break;
        case PHASER_B1_EXHAUST:  TIM3->CCR2 = ccr; break;
        case PHASER_B2_INTAKE:   TIM4->CCR1 = ccr; break;
        case PHASER_B2_EXHAUST:  TIM4->CCR2 = ccr; break;
        default:
            /* Invalid phaser index — do nothing; caller must validate */
            break;
    }
}

void dcvvt_hw_park(phaser_id_t phaser)
{
    switch (phaser) {
        case PHASER_B1_INTAKE:   TIM3->CCR1 = DCVVT_HW_CCR_PARK; break;
        case PHASER_B1_EXHAUST:  TIM3->CCR2 = DCVVT_HW_CCR_PARK; break;
        case PHASER_B2_INTAKE:   TIM4->CCR1 = DCVVT_HW_CCR_PARK; break;
        case PHASER_B2_EXHAUST:  TIM4->CCR2 = DCVVT_HW_CCR_PARK; break;
        default:
            break;
    }
}

void dcvvt_hw_park_all(void)
{
    TIM3->CCR1 = DCVVT_HW_CCR_PARK;
    TIM3->CCR2 = DCVVT_HW_CCR_PARK;
    TIM4->CCR1 = DCVVT_HW_CCR_PARK;
    TIM4->CCR2 = DCVVT_HW_CCR_PARK;
}
