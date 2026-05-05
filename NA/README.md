# G8BA ECU Firmware

**Hyundai Tau 4.6 G8BA V8 - RusEFI-based Racing ECU Firmware**

---

> ## Disclaimer
>
> - **This software is NOT designed or calibrated for use on public roads.**
>   It is developed exclusively for racing vehicles on a closed circuit or private property.
>   Use on public roads is prohibited. The user assumes full responsibility for any consequences.
>
> - **This firmware has NOT undergone complete testing (bench, dyno, or real-vehicle validation).**
>   Professional step-by-step verification is required before installation on any vehicle.
>   All calibration values (VE table, ignition advance, TDC offset, etc.) **must** be corrected
>   through dyno measurement before operation.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Engine Specifications](#engine-specifications)
3. [Hardware Requirements](#hardware-requirements)
4. [Software Architecture](#software-architecture)
5. [Module Breakdown](#module-breakdown)
6. [RusEFI Integration](#rusefi-integration)
7. [Required Calibration](#required-calibration)
8. [File Structure](#file-structure)
9. [Not Yet Implemented](#not-yet-implemented)

---

## Project Overview

The G8BA ECU firmware is a custom engine management system for racing vehicles equipped
with the Hyundai Tau 4.6 G8BA V8 engine. It runs on [RusEFI Proteus V0.4+](https://rusefi.com/)
hardware under the ChibiOS/RT real-time operating system.

Key features:
- Sequential MPI fuel injection
- Coil-on-plug (COP) independent ignition
- D-CVVT 4-channel independent cam phaser control (intake + exhaust × 2 banks)
- Per-cylinder independent knock detection and ignition retard
- Graduated engine protection (CLT / oil pressure / oil temperature / over-rev)

---

## Engine Specifications

| Parameter | Value |
|-----------|-------|
| Engine Code | Hyundai Tau G8BA |
| Displacement | 4,627 cc |
| Configuration | 90° V8 DOHC |
| Bore × Stroke | 92.0 mm × 87.0 mm |
| Compression Ratio | 10.4 : 1 |
| Firing Order | 1-2-7-8-4-5-6-3 |
| Bank Layout | Bank 1: Cylinders 1-4 / Bank 2: Cylinders 5-8 |
| Fuel | 98 RON unleaded (stoich AFR 14.7 : 1) |
| Target λ | 1.00 (idle/cruise) / 0.88 (WOT above 4,500 RPM) |
| Rev Limit (soft cut) | 7,600 RPM |
| Rev Limit (hard cut) | 7,800 RPM |

### Injector Specifications

| Parameter | Value |
|-----------|-------|
| Manufacturer/Model | Siemens/Continental high-flow MPI |
| Flow Rate | 650 cc/min @ 300 kPa |
| Fuel Rail Pressure | 300 kPa (gauge) |
| Base Pulse Width | 6.074 ms (100 % VE, 100 kPa MAP, 20 °C IAT, λ=1.00) |
| Minimum Pulse Width | 800 µs |
| Maximum Pulse Width | 25,000 µs |

### Trigger System

| Parameter | Value |
|-----------|-------|
| Crank Sensor | 36-2 Variable Reluctance (VR) |
| Physical Teeth | 34 (10 °/tooth) |
| Cam Sensors | Hall effect × 4 (B1 intake/exhaust, B2 intake/exhaust) |
| Cylinder 1 TDC Offset | 114.0 ° (**must be calibrated on dyno**) |

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| ECU Board | RusEFI Proteus V0.4+ |
| MCU | STM32H743 (ARM Cortex-M7, 480 MHz) |
| RTOS | ChibiOS/RT |
| CVVT PWM Timers | TIM3 (Bank 1), TIM4 (Bank 2) - APB1 bus, 200 MHz clock |
| CVVT GPIO | GPIOB PB4-PB7 (AF2, 250 Hz PWM) |
| Knock Sensor ADC | 1 sensor per bank × 2 (characteristic frequency 6,800 Hz) |

---

## Software Architecture

### ChibiOS Thread Layout

| Thread | Priority | Period | Responsibility |
|--------|----------|--------|----------------|
| ISR - crank_tooth_cb | Interrupt | Per tooth | Crank position update |
| ISR - cam_edge_cb | Interrupt | Per edge | Cam phase measurement |
| ISR - knock_adc_cb | Interrupt | Per ADC sample | Knock ADC via DMA |
| ISR - g8ba_cylinder_event_isr | Interrupt | Per fire event | Posts cyl_index to mailbox |
| thd_cyl_events | NORMALPRIO + 20 | Event-driven | Cylinder event dispatch (fuel + ign + knock window open) |
| thd_fast_ctrl | NORMALPRIO + 10 | 5 ms | Knock processing + CVVT control + dwell update |
| thd_medium_ctrl | NORMALPRIO | 10 ms | Fuel/ignition recalc, lambda PID (every 10th call), rev limiter |
| thd_slow_ctrl | NORMALPRIO − 5 | 50 ms | Engine protection evaluation + knock sensor self-test (5 s) |
| thd_diag | LOWPRIO | 250 ms | Diagnostics + TunerStudio output |

### ISR → Thread Communication

Cylinder events are passed from `g8ba_cylinder_event_isr()` (invoked by RusEFI's angle
scheduler) to `thd_cyl_events` through a ChibiOS mailbox (`g_cyl_mailbox`,
depth `G8BA_CYLINDERS × 2 = 16`). The ISR uses `chSysLockFromISR() / chSysUnlockFromISR()`
around the `chMBPostI()` call (SAFE-1) - required for ChibiOS *I*-class primitives.
Keeping fuel and ignition scheduling out of interrupt context preserves real-time determinism
while ensuring the correct cylinder index reaches the dispatcher (C-2 fix).

---

## Module Breakdown

### Module 1 - Crank/Cam Synchronisation (`crank_cam_sync`)

- 36-2 VR crank wheel decoder (missing-tooth gap detection)
- RPM calculation from inter-tooth interval timing
- Cam Hall-sensor phase measurement (°CA relative to crank TDC)
- Per-cylinder TDC angle provider

### Module 2 - Fuel Injection (`fuel_injection` + `fuel_map`)

- VE-based air mass calculation
- **VE table**: 16 RPM × 10 MAP points (600-7,800 RPM × 20-105 kPa), bilinear interpolation
- **Injector dead-time**: battery-voltage compensation (8-16 V, 9 points, 2.2-0.52 ms)
- **CLT warm-up enrichment**: up to 1.40× correction at −20 °C
- **IAT charge-density correction**: 11 points (−20-80 °C)
- Closed-loop lambda PID correction
- Pulse-width clamp: minimum 800 µs / maximum 25,000 µs

### Module 3 - Ignition Control (`ignition_control` + `ignition_map`)

- **Ignition advance table**: 16 RPM × 10 MAP points, bilinear interpolation
- CLT correction: cold −5 ° / overtemp −3 °
- IAT correction: up to −4 ° above 80 °C
- **Per-cylinder independent knock retard** (active path):
  `ignition_calc_advance()` reads `knock_get_retard(cyl)` from `knock_control` -
  retard accumulated at +1.5 °/knock, recovery −0.3 °/clean cycle, max 10 ° (see Module 5).
  Cylinders that did not knock retain full base advance.
- `ignition_map` also exposes a parallel `ign_map_knock_event()` / `ign_map_clean_cycle()`
  API with separate constants (+2.0 °/−0.5 °, max 15 °) and `g_ign_knock[]` state.
  These functions exist for future per-cylinder integration but are NOT currently called -
  knock retard flows exclusively through `g_knock.cylinders[].retard_deg`.
- Dwell time: battery-voltage compensation (11-14.5 V → 4.5-2.8 ms)
- Soft cut: alternating cylinder-cut at 7,600 RPM (masks 0x69 / 0x96, firing-order-aware,
  owned by `ignition_map`; `engine_protection` delegates via `ign_map_update_rev_limiter()`)
- Hard cut: all cylinders disabled at 7,800 RPM
- ISR safety:
  - `chSysLock()` guards atomic dwell/spark-angle writes to coil state (H-2)
  - `ignition_set_mode(OFF/HARD_CUT)` propagates to `coils[].enabled = false` inside the
    same critical section, and `ignition_schedule_spark()` re-checks `g_ignition.mode`
    as a defense-in-depth firewall (SAFE-2). Without this, an angle-scheduled fire-event
    pending in RusEFI could still drive the coil after a hard cut.

### Module 4 - D-CVVT Control (`dcvvt_control` + `dcvvt_hw`)

- 4-channel independent PID position control (B1 intake/exhaust, B2 intake/exhaust)
- **VVT target tables**: 8 × 8 (RPM × load %) - expand to 16 × 16 after dyno calibration
- **Intake**: up to 50 °CA advance; **Exhaust**: up to 30 °CA retard
- **PWM**: STM32H743 TIM3/TIM4 direct register control, 250 Hz

  | Register | Value | Effect |
  |----------|-------|--------|
  | PSC | 99 | 200 MHz ÷ 100 = 2 MHz timer tick |
  | ARR | 7999 | 2 MHz ÷ 8000 = 250 Hz period |
  | GPIO | PB4-PB7, AF2 | TIM3 CH1/CH2, TIM4 CH1/CH2 |

- PID gains: KP = 3.5, KI = 0.08, KD = 0.30 (tuned for fast racing transient response)
- Deadband: ±1.5 °CA; NaN/Inf guard; hardware init verification (CR1.CEN readback)
- Enable conditions: CLT ≥ 60 °C, oil temp ≥ 50 °C, RPM ≥ 500

### Module 5 - Knock Control (`knock_control`)

- 1 knock sensor per bank (2 total)
- 2nd-order IIR bandpass filter centred at 6,800 Hz (@ 44.1 kHz ADC sample rate)
- **Detection window**: 10 °-60 ° ATDC per cylinder (suppresses mechanical noise outside this band)
- **Dynamic threshold**: per-RPM noise floor learning (16 RPM bins × 2 sensors)
- **Retard** (from `g8ba_config.h`): +1.5 ° per knock event, −0.3 °/clean cycle recovery, max 10 °
- Independent per-cylinder retard tracking; per-sensor active-window state (M-2 fix)
  prevents bank-1 and bank-2 windows from corrupting each other when cylinders only
  90 ° apart fire on opposite banks
- **Window-open ordering** (SAFE-3): `knock_window_open()` clears the per-sensor peak
  buffer and publishes the active cylinder index BEFORE flipping `window_state` to
  `KNOCK_WIN_SAMPLING`. The DMA ADC IRQ only accumulates after the state transitions,
  so an in-flight sample cannot be credited to the previous cylinder.

### Module 6 - Engine Protection (`engine_protection`)

| Condition | Warning | Protection (power reduction) | Cutoff |
|-----------|---------|------------------------------|--------|
| Coolant temp (CLT) | 105 °C | 112 °C (graduated reduction) | 118 °C (fuel cut) |
| Oil pressure | 200 kPa | - | 150 kPa (fuel cut, RPM-dependent) |
| Oil temperature | 130 °C (5 °C hysteresis) | 145 °C | - |
| Over-rev | 7,600 RPM (soft cut) | - | 7,800 RPM (hard cut) |

Engine state machine (`eval_engine_state` in `main.c`) evaluates priority in the order:
**OFF → PROTECT → CRANKING → RUNNING** (SAFE-4). `hard_cut_active` outranks `rpm < CRANK`,
so a protection condition triggered during a cranking attempt routes through PROTECT
(ignition off) rather than re-entering CRANKING mode (which would re-enable the coils).

Power reduction (`prot_get_power_reduction()`) returns 0.0-1.0 based on CLT and oil
temperature warning levels. `thd_medium_ctrl` applies it by scaling MAP into the fuel
calculation. **Caveat:** with closed-loop lambda PID active, the resulting AFR is
re-targeted to stoich, which partially defeats the intended power reduction. A future
revision should switch to ignition retard or partial-cylinder fuel cut for thermal
power-down.

---

## RusEFI Integration

The following wiring steps are required for on-vehicle deployment:

```c
/* 1. Initialisation - call from RusEFI initEngineController() */
g8ba_init();

/* 2. Thread start - call after chSysInit() */
g8ba_start_threads();

/* 3. Crank callback - register with RusEFI TriggerCentral listener */
// crank_tooth_callback → TriggerCentral listener

/* 4. Cam callback - register for VVT cam inputs */
// cam_edge_callback → cam input handler

/* 5. Knock ADC callback - register as DMA complete IRQ handler */
// knock_adc_callback → DMA complete handler

/* 6. Cylinder event ISR - fires from RusEFI angle scheduler */
// g8ba_cylinder_event_isr() → angle scheduler

/* 7. Replace sensor stubs (currently return fixed placeholder values) */
// read_clt()   → Sensor::getOrZero(SensorType::Clt)
// read_map()   → Sensor::getOrZero(SensorType::Map)
// read_iat()   → Sensor::getOrZero(SensorType::Iat)
// read_tps()   → Sensor::getOrZero(SensorType::Tps1)
// read_vbatt() → Sensor::getOrZero(SensorType::BatteryVoltage)

/* 8. TunerStudio channels - update in thd_diag */
// engine->outputChannels.* update
```

---

## Required Calibration

The following items **must** be verified and corrected by a qualified engineer using a dyno
or equivalent equipment before any real-vehicle operation:

| Parameter | Current Value | Notes |
|-----------|---------------|-------|
| `G8BA_TDC_CYL1_OFFSET_DEG` | 114.0 ° | **Measure on dyno - wrong value causes global timing error** |
| VE table (`fuel_map.c`) | Initial estimate | Tune with wideband O2 lambda logging |
| Ignition advance table (`ignition_map.c`) | Initial estimate | Optimise for peak torque + knock safety margin |
| Knock threshold (`knock_control.c`) | Dynamic learned | Background noise floor must be learned per engine |
| CVVT VVT target tables (`dcvvt_control.c`) | Initial estimate (8×8) | Optimise for torque/power; expand to 16×16 post-dyno |
| PID gains (`G8BA_CVVT_KP/KI/KD`) | KP=3.5, KI=0.08, KD=0.30 | Adjust to actual hydraulic response of phaser system |

---

## File Structure

```
Grange-SuperEight-G8BA-Control-Unit/
└── NA/
    ├── README.md              # This file
    ├── include/
    │   ├── g8ba_config.h          # Master config: all constants, types, task periods
    │   ├── crank_cam_sync.h       # Module 1: crank/cam synchronisation API (active)
    │   ├── crank_sync.h           # Legacy alternative crank-sync API - NOT WIRED IN
    │   ├── fuel_injection.h       # Module 2: fuel injection API
    │   ├── fuel_map.h             # Module 2: fuel calibration table API (sole owner)
    │   ├── ignition_control.h     # Module 3: ignition control API
    │   ├── ignition_map.h         # Module 3: ignition advance table API (sole owner)
    │   ├── dcvvt_control.h        # Module 4: D-CVVT control API
    │   ├── dcvvt_hw.h             # Module 4: STM32H743 PWM hardware driver API
    │   ├── knock_control.h        # Module 5: knock detection/control API
    │   └── engine_protection.h    # Module 6: engine protection API
    └── src/
        ├── main.c                 # ChibiOS threads, ISR wiring, g8ba_init()
        ├── crank_cam_sync.c       # 36-2 decoder, cam phase, RPM calculation (active)
        ├── crank_sync.c           # Legacy alternative implementation - NOT WIRED IN
        ├── fuel_injection.c       # VE-based fuel calculation, lambda PID
        ├── fuel_map.c             # VE / CLT / IAT / dead-time tables (sole owner)
        ├── ignition_control.c     # Advance calculation, dwell, soft/hard cut
        ├── ignition_map.c         # Advance table, rev limiter logic (sole owner)
        ├── dcvvt_control.c        # CVVT PID position control, 8×8 VVT tables
        ├── dcvvt_hw.c             # STM32H743 TIM3/TIM4 direct-register PWM
        ├── knock_control.c        # BPF filter, dynamic threshold, per-cylinder retard
        └── engine_protection.c    # CLT / oil / rev-limit graduated protection
```

> **Note on legacy crank-sync files:** `crank_sync.c/h` is an alternative, more-defensive
> 36-2 decoder implementation that was developed in parallel with `crank_cam_sync.c/h`.
> `main.c` currently includes and uses `crank_cam_sync.*` only; `crank_sync.*` is dead
> code (no symbol clash; symbols are uniquely prefixed). Either remove the legacy files
> or migrate to them after a calibrated comparison - do not leave both indefinitely.

---

## Not Yet Implemented

The following items are absent from the current codebase and are required before
real-vehicle deployment:

- **Sensor reads**: All sensor functions (`read_clt()`, etc.) return hardcoded placeholder
  values. Must be replaced with RusEFI `Sensor::getOrZero()` C++ API calls.
- **Build system**: No `CMakeLists.txt` or `Makefile` exists.
- **TunerStudio configuration**: `tunerstudio/g8ba.ini` (gauge and table definitions) not yet written.
- **Cylinder event angle scheduler**: `scheduleByAngle()` RusEFI integration incomplete
  (currently using time-based fallback).
- **Knock window close scheduler**: `knock_window_close()` angle-scheduling not yet wired
  (currently using 5 ms polling fallback in `thd_fast_ctrl`).

---

*This project is experimental racing-only software. Use on public roads or by unqualified
personnel is strongly discouraged.*
