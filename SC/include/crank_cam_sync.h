/**
 * @file    crank_cam_sync.h
 * @brief   Module 1 — Crank/Cam Synchronization
 *
 * Decodes 36-2 VR crank wheel and Hall-effect cam sensors to determine:
 *   - Absolute crank position (°CA, 0–720)
 *   - Engine phase (compression vs exhaust TDC)
 *   - Per-cylinder event scheduling base
 *
 * Integration with RusEFI trigger subsystem:
 *   RusEFI handles raw tooth ISR and gap detection via its TriggerCentral.
 *   This module registers callbacks and exposes a clean engine-position API.
 */

#ifndef CRANK_CAM_SYNC_H
#define CRANK_CAM_SYNC_H

#include "g8ba_config.h"

/* ══════════════════════════════════════════════════════════════════════════
 * DATA TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

/** Synchronization state machine states */
typedef enum {
    SYNC_LOST       = 0,   /* no sync, awaiting gap detection          */
    SYNC_PARTIAL    = 1,   /* gap found, cam phase not confirmed        */
    SYNC_FULL       = 2,   /* crank + cam phase confirmed, running      */
} sync_state_t;

/** Cam channel identifiers (D-CVVT has 4 independent cam phasers) */
typedef enum {
    CAM_B1_INTAKE  = 0,
    CAM_B1_EXHAUST = 1,
    CAM_B2_INTAKE  = 2,
    CAM_B2_EXHAUST = 3,
    CAM_COUNT      = 4,
} cam_id_t;

/** Engine position snapshot — updated every tooth event */
typedef struct {
    floatdeg_t  crank_angle_720;    /* absolute angle 0–720 °CA           */
    floatdeg_t  crank_angle_360;    /* crank angle 0–360 °CA              */
    rpm_t       rpm;                /* instantaneous RPM                   */
    rpm_t       rpm_filtered;       /* low-pass filtered RPM               */
    uint8_t     sync_tooth;         /* tooth number since gap (0-33)       */
    sync_state_t sync_state;        /* current synchronization state       */
    bool        phase_confirmed;    /* cam sensor confirmed phase          */
    uint8_t     current_cylinder;   /* cylinder index (0-7) next to fire   */
    us_t        last_tooth_us;      /* timestamp of last tooth (µs)        */
    us_t        tooth_period_us;    /* last tooth-to-tooth period (µs)     */
} engine_position_t;

/** Cam sensor state */
typedef struct {
    cam_id_t    id;
    floatdeg_t  phase_deg;          /* measured cam phase (°CA)           */
    floatdeg_t  target_phase_deg;   /* commanded cam phase (°CA)          */
    us_t        last_edge_us;       /* timestamp of last cam edge          */
    bool        valid;              /* sensor reading is valid             */
} cam_state_t;

/* ══════════════════════════════════════════════════════════════════════════
 * MODULE STATE (read-only external access)
 * ══════════════════════════════════════════════════════════════════════════ */

extern volatile engine_position_t g_engine_pos;
extern volatile cam_state_t       g_cam[CAM_COUNT];

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialize crank/cam sync module.
 *         Configures RusEFI trigger type to TT_TOOTHED_WHEEL_36_2,
 *         registers tooth and sync callbacks, initialises state.
 * @return G8BA_OK on success.
 */
g8ba_status_t crank_cam_sync_init(void);

/**
 * @brief  Callback invoked by RusEFI TriggerCentral on every crank tooth.
 *         Called from interrupt context — must be ISR-safe (no blocking).
 * @param  tooth_index  0-based tooth number (gap = tooth 0)
 * @param  timestamp_us Tooth arrival timestamp in microseconds
 */
void crank_tooth_callback(uint8_t tooth_index, us_t timestamp_us);

/**
 * @brief  Callback invoked by RusEFI on cam sensor rising edge.
 * @param  cam   Which cam channel fired
 * @param  timestamp_us Edge timestamp in microseconds
 */
void cam_edge_callback(cam_id_t cam, us_t timestamp_us);

/**
 * @brief  Return current absolute crank angle (0–720°).
 *         Thread-safe — uses atomic read.
 */
floatdeg_t crank_get_angle(void);

/**
 * @brief  Return current engine RPM (filtered).
 */
rpm_t crank_get_rpm(void);

/**
 * @brief  Return current sync state.
 */
sync_state_t crank_get_sync_state(void);

/**
 * @brief  Calculate crank angle at which cylinder `cyl` reaches TDC compression.
 * @param  cyl  Cylinder index 0-7 (maps via firing order)
 * @return Absolute crank angle 0–720 °CA
 */
floatdeg_t crank_tdc_angle(uint8_t cyl);

/**
 * @brief  Return next cylinder index (0-7) that will fire.
 */
uint8_t crank_next_cylinder(void);

/**
 * @brief  Get cam phase measurement for specified cam channel.
 * @param  cam   Cam channel
 * @param  out   Pointer to receive phase in °CA
 * @return G8BA_OK or G8BA_ERR_SENSOR if signal lost
 */
g8ba_status_t cam_get_phase(cam_id_t cam, floatdeg_t *out);

/**
 * @brief  Reset sync state — called after engine stall or error.
 */
void crank_cam_sync_reset(void);

/**
 * @brief  Diagnostic: how many consecutive sync confirmations since start.
 */
uint32_t crank_sync_count(void);

#endif /* CRANK_CAM_SYNC_H */
