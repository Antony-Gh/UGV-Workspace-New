/**
 * @file    config.h
 * @brief   Competition profile configuration for the UGV.
 *
 * Provides static, compile-time profiles for each supported competition
 * (IGVC, IRC, UGVC, URC, ERC) plus a runtime selector via CAN 0x120.
 *
 * Each profile defines:
 *   - Speed limits (min/max in m/s)
 *   - Heartbeat timeout (ms)
 *   - LED patterns for TELEOP and AUTONOMOUS modes
 *   - Wheel radius (m) for velocity ↔ linear speed conversion
 *
 * No dynamic allocation. Profiles stored in flash (const-qualified).
 *
 * @author  UGV Firmware Team
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ──────────────────── Competition Identifiers ─────────────────── */

typedef enum {
  COMP_IGVC = 0, /**< IGVC 2026 — 1–5 mph (0.45–2.24 m/s)         */
  COMP_IRC = 1,  /**< IRC 2026  — no strict speed limits           */
  COMP_UGVC = 2, /**< UGVC 2025 — 2–10 km/h (0.56–2.78 m/s)       */
  COMP_URC = 3,  /**< URC 2025  — no strict speed limits           */
  COMP_ERC = 4,  /**< ERC 2025  — no strict speed limits           */
  COMP_COUNT = 5
} Competition_t;

/* ──────────────────── Profile Structure ───────────────────────── */

/**
 * @brief Configuration profile for one competition.
 *
 * Speed limits in meters per second.
 * LED pattern indices correspond to LightMode_t values.
 */
typedef struct {
  float speed_min_mps;           /**< Minimum allowed speed (m/s), 0 = none   */
  float speed_max_mps;           /**< Maximum allowed speed (m/s)             */
  float wheel_radius_m;          /**< Wheel radius for vel→speed conversion   */
  uint32_t heartbeat_timeout_ms; /**< Comms timeout before FAULT (ms)       */
  uint8_t led_pattern_teleop;    /**< LightMode_t for teleop mode             */
  uint8_t led_pattern_auto;      /**< LightMode_t for autonomous mode         */
} CompetitionConfig_t;

/* ──────────────────── Public API ──────────────────────────────── */

/**
 * @brief  Initialise the config module with the default competition.
 *         Defaults to COMP_IGVC.
 */
void Config_Init(void);

/**
 * @brief  Switch to a different competition profile.
 * @param  comp  Competition identifier (0–4).
 * @return 0 on success, -1 if comp is out of range.
 *
 * Also updates the fault module heartbeat timeout.
 */
int Config_Apply(Competition_t comp);

/**
 * @brief  Get the currently active competition profile.
 * @return Pointer to the active CompetitionConfig_t (read-only).
 */
const CompetitionConfig_t *Config_Get(void);

/**
 * @brief  Get the currently active competition ID.
 * @return Competition_t enum value.
 */
Competition_t Config_GetActiveComp(void);

/**
 * @brief  Convert a linear speed (m/s) to wheel angular velocity (rad/s).
 * @param  speed_mps  Linear speed in meters per second.
 * @return Angular velocity in rad/s using the active profile's wheel radius.
 */
float Config_SpeedToRadPerSec(float speed_mps);

/**
 * @brief  Convert wheel angular velocity (rad/s) to linear speed (m/s).
 * @param  rad_per_sec  Angular velocity in rad/s.
 * @return Linear speed in meters per second.
 */
float Config_RadPerSecToSpeed(float rad_per_sec);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
