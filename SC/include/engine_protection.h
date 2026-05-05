/**
 * @file    engine_protection.h
 * @brief   Module 6 — Engine Protection (SC variant — adds boost-overshoot cut)
 *
 * SC additions vs NA:
 *   - PROT_EVENT_BOOST_OVERSHOOT  flag (persistent MAP > 180 kPa)
 *   - PROT_EVENT_BOOST_HARDCUT    flag (instant MAP > 250 kPa)
 *   - PROT_EVENT_IAT_HOT          flag (post-intercooler IAT > 65 °C)
 *   - All thresholds tightened to reflect higher engine duty cycle
 *
 * Sequencing: boost safety check is OWNED by boost_control.c (called from
 * thd_slow_ctrl); engine_protection_update() merely consumes the result and
 * raises the corresponding flags + records the fault in g_protection.
 */

#ifndef ENGINE_PROTECTION_H
#define ENGINE_PROTECTION_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    PROT_LEVEL_OK      = 0,
    PROT_LEVEL_WARN    = 1,
    PROT_LEVEL_REDUCE  = 2,
    PROT_LEVEL_CUT     = 3,
    PROT_LEVEL_EMERG   = 4,
} prot_level_t;

typedef enum {
    PROT_EVENT_NONE             = 0x0000,
    PROT_EVENT_CLT_WARN         = 0x0001,
    PROT_EVENT_CLT_REDUCE       = 0x0002,
    PROT_EVENT_CLT_CUT          = 0x0004,
    PROT_EVENT_OIL_PRESS_W      = 0x0008,
    PROT_EVENT_OIL_PRESS_CUT    = 0x0010,
    PROT_EVENT_OIL_TEMP_W       = 0x0020,
    PROT_EVENT_OIL_TEMP_CUT     = 0x0040,
    PROT_EVENT_REV_SOFT         = 0x0080,
    PROT_EVENT_REV_HARD         = 0x0100,
    PROT_EVENT_SYNC_LOST        = 0x0200,
    PROT_EVENT_SENSOR_FAULT     = 0x0400,
    /* SC additions */
    PROT_EVENT_BOOST_OVERSHOOT  = 0x0800,   /* Persistent overshoot       */
    PROT_EVENT_BOOST_HARDCUT    = 0x1000,   /* Instant ≥250 kPa cut       */
    PROT_EVENT_IAT_HOT          = 0x2000,   /* IAT above SC limit         */
} prot_event_flags_t;

typedef struct {
    float       clt_c;
    float       oil_pressure_kpa;
    float       oil_temp_c;
    float       map_kpa;
    float       iat_c;             /* SC: now consumed for PROT_EVENT_IAT_HOT */
    rpm_t       rpm;
    bool        sync_ok;
    float       vbatt;
} prot_inputs_t;

typedef struct {
    bool        soft_active;
    bool        hard_active;
    uint8_t     soft_cut_mask;
    uint8_t     soft_cut_toggle;
    rpm_t       hysteresis_rpm;
} rev_limiter_state_t;

typedef struct {
    prot_level_t    clt_level;
    prot_level_t    oil_temp_level;
    prot_level_t    oil_press_level;
    float           power_reduction_pct;
} thermal_state_t;

typedef struct {
    prot_inputs_t       inputs;
    prot_event_flags_t  active_events;
    prot_level_t        overall_level;
    rev_limiter_state_t rev_limiter;
    thermal_state_t     thermal;
    uint32_t            clt_warn_count;
    uint32_t            oil_warn_count;
    uint32_t            boost_overshoot_count;   /* SC */
    us_t                last_update_us;
} prot_status_t;

extern volatile prot_status_t g_protection;

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

g8ba_status_t engine_protection_init(void);
void          engine_protection_update(const prot_inputs_t *inputs);
void          rev_limiter_update(rpm_t rpm);
bool          prot_is_hard_cut_active(void);
float         prot_get_power_reduction(void);
prot_level_t  prot_get_level(void);
prot_event_flags_t prot_get_events(void);
void          prot_emergency_cut(void);
void          prot_emergency_clear(void);
void          prot_send_warning(prot_event_flags_t event);
void          prot_log_fault(prot_event_flags_t event, prot_level_t level, float value);

#endif /* ENGINE_PROTECTION_H */
