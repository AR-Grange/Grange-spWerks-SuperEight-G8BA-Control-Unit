# G8BA-SC ECU Firmware

**Hyundai Tau 4.6 G8BA V8 + Vortech V-7 YSi-Trim Centrifugal Supercharger**
**RusEFI-based Racing ECU Firmware**

---

> ## Disclaimer
>
> - **This software is NOT designed or calibrated for use on public roads.**
>   It is developed exclusively for racing vehicles on a closed circuit or private
>   property. Use on public roads is prohibited. The user assumes full responsibility.
>
> - **This firmware has NOT undergone complete testing (bench, dyno, or real-vehicle
>   validation).** Professional step-by-step verification is required before installation
>   on any vehicle. All calibration values (VE table, ignition advance, boost target,
>   TDC offset, etc.) **must** be corrected through dyno measurement before operation.
>
> - **Forced induction amplifies every calibration error.** Running the SC firmware
>   on an engine that does NOT have the required hardware (forged 9.0:1 pistons,
>   850 cc/min injectors, intercooler, knock sensors, MAP sensor rated to 300 kPa)
>   will destroy the engine on the first WOT pull.

---

## Table of Contents

1. [What's Different from NA](#whats-different-from-na)
2. [Supercharger Setup](#supercharger-setup)
3. [Hardware Requirements](#hardware-requirements)
4. [Software Architecture](#software-architecture)
5. [Module Breakdown](#module-breakdown)
6. [RusEFI Integration](#rusefi-integration)
7. [Required Calibration](#required-calibration)
8. [File Structure](#file-structure)
9. [Not Yet Implemented](#not-yet-implemented)

---

## What's Different from NA

The SC branch is a **fork of the NA branch** with the following design changes
(see `PR.md` for the full diff narrative):

| Area | NA | SC |
|------|----|----|
| Static compression | 10.4:1 | **9.0:1** (forged dished pistons) |
| Injectors | 650 cc/min | **850 cc/min** |
| Soft-/hard-cut RPM | 7,600 / 7,800 | **7,300 / 7,500** |
| MAP table axis | 20-105 kPa (10 pts) | **30-250 kPa (14 pts)** |
| IAT table axis | -20-80 °C (11 pts) | **-20-100 °C (13 pts)** |
| VE peak | 102 % @ 4500 RPM, 105 kPa | **110 % @ 4500 RPM, 150 kPa (boost)** |
| Max ignition advance | 45° | **35°** |
| Knock retard step | +1.5° / event | **+2.5° / event (more aggressive)** |
| Knock max retard | 10° | **12°** |
| CLT thresholds | 105 / 112 / 118 °C | **100 / 108 / 115 °C (tighter)** |
| Oil temp thresholds | 130 / 145 °C | **125 / 140 °C** |
| Oil pressure floor | 80-200 kPa (RPM) | **100-240 kPa (RPM, raised for SC bearing load)** |
| Boost control | - | **Module 7 (boost_control), PID on BCV PWM** |
| Hard boost cut | - | **MAP ≥ 250 kPa → instant fuel cut** |
| BCV PWM hardware | - | **TIM5 CH1 → PA0 (AF2), 40 Hz** |

---

## Supercharger Setup

| Parameter | Value |
|-----------|-------|
| Make / Model | **Vortech V-7 YSi-Trim** |
| Type | Centrifugal, gear-driven head |
| Drive | Crank pulley → 8-rib serpentine → input shaft |
| Pulley target boost | ~7 PSI peak (≈ 148 kPa absolute) |
| Hard cut threshold | 250 kPa absolute (≈ 21.7 PSI) |
| Bypass-control valve (BCV) | Normally-OPEN solenoid, PWM-driven |
| BCV duty convention | 0 % = fully open (no boost), 95 % cap = max regulation authority |
| BCV PWM frequency | 40 Hz (matches solenoid mechanical bandwidth ~25 ms) |
| Intercooler | Air-to-water, front-mount heat exchanger |
| Race fuel | 98 RON (E85 capable with separate calibration) |

The Vortech V-7 YSi was chosen because:

- **Centrifugal boost curve matches the engine's high-RPM character** - boost
  rises with shaft speed, peaking near the engine's torque/power peak.
- **High adiabatic efficiency** (~78 % at peak) - minimal IAT rise vs. Roots/screw.
- **Low parasitic loss at low RPM** - preserves throttle response off-boost.
- **Front-mount packaging** - does not occupy the V8 valley; preserves D-CVVT access.
- **Gear-driven head** - no internal lubrication concerns shared with engine oil.

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| ECU Board | RusEFI Proteus V0.4+ |
| MCU | STM32H743 (ARM Cortex-M7, 480 MHz) |
| RTOS | ChibiOS/RT |
| CVVT PWM Timers | TIM3 (Bank 1), TIM4 (Bank 2) - APB1 bus, 200 MHz clock |
| **BCV PWM Timer (SC)** | **TIM5 CH1** - APB1 bus, 200 MHz clock |
| CVVT GPIO | GPIOB PB4-PB7 (AF2, 250 Hz PWM) |
| **BCV GPIO (SC)** | **GPIOA PA0 (AF2, 40 Hz PWM)** |
| Knock Sensor ADC | 1 sensor per bank × 2 (characteristic frequency 6,800 Hz) |
| **MAP Sensor (SC)** | **3-bar absolute (rated to 300 kPa) - replaces 1-bar NA sensor** |

### Mechanical preconditions

- **Forged dished pistons, 9.0:1 static CR** (was 10.4:1 stock)
- **MLS multi-layer steel head gasket** (replaces stock composite)
- **ARP head studs** (additional clamp load for MLS)
- **Upgraded fuel pump** capable of 5,200 cc/min total flow at 400 kPa
- **3-bar MAP sensor** in plenum (1-bar NA sensor saturates above 100 kPa)
- **BCV solenoid** (e.g. GFB G-Force III or equivalent, 12 V, ≥ 100 mA hold)

---

## Software Architecture

### ChibiOS Thread Layout

| Thread | Priority | Period | Responsibility |
|--------|----------|--------|----------------|
| ISR - crank_tooth_cb | Interrupt | Per tooth | Crank position update |
| ISR - cam_edge_cb | Interrupt | Per edge | Cam phase measurement |
| ISR - knock_adc_cb | Interrupt | Per ADC sample | Knock ADC via DMA |
| ISR - g8ba_cylinder_event_isr | Interrupt | Per fire event | Posts cyl_index to mailbox |
| thd_cyl_events | NORMALPRIO + 20 | Event-driven | Cylinder dispatch (fuel + ign + knock window open) |
| thd_fast_ctrl | NORMALPRIO + 10 | 5 ms | Knock processing + CVVT control + dwell update |
| thd_medium_ctrl | NORMALPRIO | 10 ms | **Fuel/ign recalc, lambda PID, rev limiter, boost PID** |
| thd_slow_ctrl | NORMALPRIO - 5 | 50 ms | Engine protection (incl. boost safety check) + knock self-test |
| thd_diag | LOWPRIO | 250 ms | Diagnostics + TunerStudio output |

The SC variant adds the boost-control PID step into `thd_medium_ctrl` (10 ms cadence
matches the BCV solenoid's mechanical response time of ~25 ms - running the PID
faster would only chase noise) and the boost overshoot/hard-cut check into
`thd_slow_ctrl` via `boost_safety_check()`.

### ISR → Thread Communication

Cylinder events are passed from `g8ba_cylinder_event_isr()` (invoked by RusEFI's
angle scheduler) to `thd_cyl_events` through a ChibiOS mailbox (`g_cyl_mailbox`,
depth `G8BA_CYLINDERS × 2 = 16`). The ISR uses `chSysLockFromISR()` /
`chSysUnlockFromISR()` around the `chMBPostI()` call (SAFE-1, preserved from NA).

---

## Module Breakdown

### Module 1 - Crank/Cam Synchronisation (`crank_cam_sync`)

Identical to NA - 36-2 VR crank wheel decoder + Hall-cam phase confirmation.
No SC-specific changes.

### Module 2 - Fuel Injection (`fuel_injection` + `fuel_map`)

- VE-based air-mass calculation (formula identical to NA)
- **VE table EXTENDED**: 16 RPM × **14 MAP points** (30-250 kPa)
- **Injector dead-time**: re-tabulated for 850 cc/min injector (longer absolute times)
- **CLT warm-up enrichment**: same physics as NA (independent of induction type)
- **IAT charge-density correction**: extended to 100 °C for hot post-SC charge
- Closed-loop lambda PID correction (P + I terms, same as NA)
- Pulse-width clamp: minimum 900 µs / maximum 14 ms (was 800 µs / 25 ms NA)
- `FUELMAP_BASE_PW_MS = 4.645 ms` (re-derived for 850 cc/min; was 6.074 ms NA)
- New fuel-cut flag: `FCUT_BOOST_OVERSHOOT` (0x40)

### Module 3 - Ignition Control (`ignition_control` + `ignition_map`)

- **Advance table EXTENDED**: 16 RPM × **14 MAP points**, with boost rows pulled
  back significantly (peak 35° NA → ~10° at 250 kPa boost)
- `G8BA_IGN_ADVANCE_MAX = 35°` (was 45° NA)
- CLT / IAT / dwell logic unchanged from NA
- Per-cylinder knock retard via `knock_get_retard()` - **more aggressive** in SC:
  `G8BA_KNOCK_RETARD_STEP = 2.5°` (was 1.5 NA), `_MAX = 12°` (was 10 NA)
- Soft-/hard-cut RPM thresholds lowered (7,300 / 7,500 vs 7,600 / 7,800 NA)
- ISR safety: SAFE-2 fix preserved (mode propagates to `coils[].enabled` inside
  `chSysLock()` and `ignition_schedule_spark()` re-checks mode as defense-in-depth)

### Module 4 - D-CVVT Control (`dcvvt_control` + `dcvvt_hw`)

- 4-channel PID identical to NA - same hardware (TIM3 / TIM4)
- **Intake max advance reduced from 50° → 35°** to limit valve overlap under boost
  (`G8BA_CVVT_IN_ADVANCE_MAX = 35.0f`)
- Exhaust retard max unchanged (30°)
- VVT target tables unchanged in SC v1; re-tune for boost regions in v2

### Module 5 - Knock Control (`knock_control`)

- Same hardware path: 2 sensors, 6,800 Hz BPF, dynamic noise floor
- **Detection window widened**: 8°-65° ATDC (was 10°-60° NA)
- **Retard steps more aggressive**: +2.5° per event, -0.25° per clean cycle
- **Max retard**: 12° (was 10° NA) - more headroom under boost detonation
- Per-sensor active-window tracking (M-2 fix preserved from NA)
- Window-open ordering (SAFE-3 fix preserved from NA)

### Module 6 - Engine Protection (`engine_protection`)

| Condition | Warning | Reduce | Cutoff |
|-----------|---------|--------|--------|
| Coolant temp (CLT) | **100 °C** | **108 °C** | **115 °C** (fuel cut) |
| Oil pressure | **230 kPa** | - | **100-240 kPa (RPM-dep)** |
| Oil temperature | **125 °C** (5 °C hyst) | **140 °C** | - |
| **IAT (post-IC) (SC)** | **65 °C** (warn-only) | - | - |
| **Boost overshoot (SC)** | - | **180 kPa** (persistent → fuel cut) | **250 kPa** (instant cut) |
| Over-rev | 7,300 RPM (soft cut) | - | 7,500 RPM (hard cut) |

State machine evaluates priority: **OFF → PROTECT → CRANKING → RUNNING** (SAFE-4
preserved from NA). `prot_is_hard_cut_active()` now also returns true for boost
hard-cut. `prot_emergency_cut()` additionally calls `boost_park()` to fully open
the BCV on sync loss.

### Module 7 - Boost Control (`boost_control`) - **NEW IN SC**

- **Boost target table**: 8 RPM × 8 TPS% (1500-7500 RPM × 0-100 %)
- **PID closed loop** on MAP error → BCV duty cycle (`G8BA_BOOST_KP/KI/KD`)
- **Open-loop floor**: BCV held open (0 %) below `G8BA_BOOST_ENABLE_RPM = 2500`
- **PWM hardware**: STM32H743 TIM5 CH1 → GPIOA PA0 (AF2), 40 Hz
- **Anti-windup**: integrator clamped to [-40, +60] %duty equivalent
- **NaN/Inf guard**: any non-finite MAP/TPS input → BCV park OPEN, mode FAULT
- **Boost safety**:
  - `boost_safety_check()` runs in `thd_slow_ctrl` (50 ms)
  - MAP ≥ 250 kPa → IMMEDIATE `fuel_cut_set(FCUT_BOOST_OVERSHOOT)` + slam BCV open
  - MAP ≥ 180 kPa for 200 ms persistent → same cut, with hysteresis on recovery

---

## RusEFI Integration

```c
/* 1. Initialisation - call from RusEFI initEngineController() */
g8ba_init();

/* 2. Thread start - call after chSysInit() */
g8ba_start_threads();

/* 3. Crank / cam / knock / cylinder-event callbacks - same as NA */

/* 4. Replace sensor stubs in main.c */
// read_clt()           → Sensor::getOrZero(SensorType::Clt)
// read_iat()           → Sensor::getOrZero(SensorType::Iat)
// read_tps()           → Sensor::getOrZero(SensorType::Tps1)
// read_map_kpa()       → Sensor::getOrZero(SensorType::Map)   [3-bar sensor!]
// read_lambda()        → Sensor::getOrZero(SensorType::Lambda1)
// read_oil_pressure_kpa() → Sensor::getOrZero(SensorType::OilPressure)
// read_oil_temp()      → Sensor::getOrZero(SensorType::OilTemp)
// read_vbatt()         → Sensor::getOrZero(SensorType::BatteryVoltage)

/* 5. TunerStudio channels (SC adds) */
// engine->outputChannels.boostMapKpa     = g_boost.map_kpa;
// engine->outputChannels.boostTargetKpa  = g_boost.target_kpa;
// engine->outputChannels.bcvDutyPct      = g_boost.bcv_duty_pct;
// engine->outputChannels.boostMode       = g_boost.mode;
// engine->outputChannels.boostHardCut    = g_boost.hard_cut_active;
```

---

## Required Calibration

The following items **must** be verified and corrected by a qualified engineer
using a dyno before any real-vehicle operation:

| Parameter | Initial Value | Notes |
|-----------|---------------|-------|
| `G8BA_TDC_CYL1_OFFSET_DEG` | 114.0 ° | Measure on dyno - wrong value = global timing error |
| VE table boost rows (`fuel_map.c`, MAP > 100 kPa) | 100-110 % estimated | Tune with wideband O2 |
| Ignition boost rows (`ignition_map.c`) | 8-22° estimated | Pull until knock is suppressed with margin |
| Boost target table (`boost_control.c`, `s_target_kpa`) | 100-148 kPa | Begin conservative, raise after fuel/timing is safe |
| Boost PID (`G8BA_BOOST_KP/KI/KD`) | 1.20 / 0.08 / 0.40 | Tune step-response behaviour on dyno |
| Knock noise floor | learned dynamically | Background noise per RPM bin must be re-learned for SC engine |
| CVVT VVT target tables (`dcvvt_control.c`) | inherited from NA | Re-tune boost rows to reduce overlap |
| Pulley ratio (mechanical, on Vortech) | 3.40" | Determines geometric peak boost; cap with smaller pulley if needed |

---

## File Structure

```
Grange-SuperEight-G8BA-Control-Unit/
└── SC/
    ├── README.md              # This file
    ├── PR.md                  # Detailed NA→SC change log
    ├── include/
    │   ├── g8ba_config.h          # Master config (SC variant - CR 9.0, boost limits)
    │   ├── crank_cam_sync.h       # Module 1: crank/cam sync (unchanged from NA)
    │   ├── fuel_injection.h       # Module 2: + FCUT_BOOST_OVERSHOOT flag
    │   ├── fuel_map.h             # Module 2: MAP axis 30-250 kPa, 850 cc/min injectors
    │   ├── ignition_control.h     # Module 3: unchanged from NA
    │   ├── ignition_map.h         # Module 3: MAP axis 30-250 kPa, retarded boost rows
    │   ├── dcvvt_control.h        # Module 4: unchanged from NA
    │   ├── dcvvt_hw.h             # Module 4: unchanged from NA
    │   ├── knock_control.h        # Module 5: unchanged from NA
    │   ├── engine_protection.h    # Module 6: + boost / IAT-hot events
    │   └── boost_control.h        # Module 7 NEW: Vortech V-7 boost control API
    └── src/
        ├── main.c                 # ChibiOS threads + boost PID call
        ├── crank_cam_sync.c       # unchanged from NA
        ├── fuel_injection.c       # unchanged from NA (consumes FCUT_BOOST_OVERSHOOT via flag mask)
        ├── fuel_map.c             # SC tables (16 × 14)
        ├── ignition_control.c     # unchanged from NA
        ├── ignition_map.c         # SC tables (16 × 14)
        ├── dcvvt_control.c        # unchanged from NA
        ├── dcvvt_hw.c             # unchanged from NA
        ├── knock_control.c        # unchanged from NA
        ├── engine_protection.c    # + boost safety / IAT-hot / tightened thresholds
        └── boost_control.c        # Module 7 NEW: PID + BCV PWM + safety check
```

The legacy `crank_sync.c/h` files from NA are **NOT** carried over - the SC tree
starts clean.

---

## Not Yet Implemented

The following items are absent from the current codebase and are required before
real-vehicle deployment:

- **Sensor reads**: All sensor functions return placeholder values. Must be
  replaced with RusEFI `Sensor::getOrZero()` calls. **Critically: `read_map_kpa()`
  must be wired to a 3-bar (300 kPa) MAP sensor**, not a 1-bar.
- **Build system**: No `CMakeLists.txt` or `Makefile` exists.
- **TunerStudio configuration**: `tunerstudio/g8ba_sc.ini` not yet written
  (must include `boostMapKpa`, `boostTargetKpa`, `bcvDutyPct`, `boostMode` channels).
- **Cylinder event angle scheduler**: same gap as NA - `scheduleByAngle()` integration
  incomplete, time-based fallback in use.
- **Knock window close scheduler**: same gap as NA - using `knock_flush_open_windows()`
  fallback in `thd_fast_ctrl`.
- **Boost target table dyno re-tune**: current values are conservative estimates.
- **VVT target table boost re-tune**: NA tables carried over without modification.

---

*This project is experimental racing-only software. Use on public roads or by
unqualified personnel is strongly discouraged. Forced induction amplifies all risks.*
