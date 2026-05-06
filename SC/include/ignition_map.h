/**
 * @file    ignition_map.h
 * @brief   Ignition Calibration Tables - G8BA + Vortech V-7 YSi-Trim (SC variant)
 *
 * SC differences vs NA:
 *   - MAP axis extended from 105 kPa -> 250 kPa (10 -> 14 points)
 *   - Base advance values significantly retarded above 100 kPa
 *   - Maximum advance clamped to G8BA_IGN_ADVANCE_MAX = 35deg (was 45deg NA)
 *   - Soft-cut mask scheme unchanged
 *
 * Per-cylinder knock retard scheme: identical to NA (per-cyl tracking via
 * knock_control.c -> knock_get_retard()).  IGNMAP_KNOCK_* constants below
 * still describe the unused parallel API (same caveat as NA - see README).
 */

#ifndef IGNITION_MAP_H
#define IGNITION_MAP_H

#include "g8ba_config.h"

/* ==========================================================================
 * TABLE DIMENSIONS
 * ========================================================================== */

#define IGNMAP_RPM_POINTS   16u
#define IGNMAP_MAP_POINTS   14u                /* SC: was 10 */

/* ==========================================================================
 * KNOCK RETARD CONSTANTS  (parallel ignition_map API - see header note above)
 * ========================================================================== */

#define IGNMAP_KNOCK_RETARD_STEP_DEG    2.5f   /* SC: more aggressive (was 2.0 NA) */
#define IGNMAP_KNOCK_ADVANCE_STEP_DEG   0.4f   /* SC: slower recovery (was 0.5 NA) */
#define IGNMAP_KNOCK_MAX_RETARD_DEG     17.0f  /* SC: more headroom (was 15 NA)    */

/* ==========================================================================
 * REV LIMITER CONSTANTS
 * ========================================================================== */

#define IGNMAP_SOFT_CUT_RPM     G8BA_RPM_SOFT_CUT    /* 7300 RPM in SC */
#define IGNMAP_HARD_CUT_RPM     G8BA_RPM_MAX          /* 7500 RPM in SC */
#define IGNMAP_SOFT_CUT_HYST_RPM   150u

#define IGNMAP_SOFT_CUT_MASK_A  0x69u
#define IGNMAP_SOFT_CUT_MASK_B  0x96u
#define IGNMAP_ALL_CYL_MASK     0xFFu

/* ==========================================================================
 * DATA TYPES
 * ========================================================================== */

typedef struct {
    float    retard_deg;
    uint16_t knock_events;
    uint16_t clean_cycles;
} ign_cyl_knock_t;

typedef enum {
    IGNMAP_REV_OFF = 0,
    IGNMAP_REV_SOFT,
    IGNMAP_REV_HARD,
} ign_rev_state_t;

extern volatile ign_cyl_knock_t g_ign_knock[G8BA_CYLINDERS];
extern const uint16_t IGNMAP_RPM_AXIS[IGNMAP_RPM_POINTS];
extern const uint8_t  IGNMAP_MAP_AXIS[IGNMAP_MAP_POINTS];

/* ==========================================================================
 * PUBLIC API
 * ========================================================================== */

g8ba_status_t ign_map_init(void);
float         ign_map_get_base_advance(float rpm, float map_kpa);
float         ign_map_get_advance(uint8_t cyl_index, float rpm, float map_kpa);
void          ign_map_knock_event(uint8_t cyl_index);
void          ign_map_clean_cycle(uint8_t cyl_index);
uint8_t       ign_map_update_rev_limiter(rpm_t rpm);
ign_rev_state_t ign_map_get_rev_state(void);
float         ign_map_get_cyl_retard(uint8_t cyl_index);

#endif /* IGNITION_MAP_H */
