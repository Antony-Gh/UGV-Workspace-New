/**
 * @file    config.c
 * @brief   Competition profile configuration implementation.
 *
 * Stores 5 competition profiles in flash (const-qualified).
 * Runtime selection via Config_Apply() — triggered by CAN 0x120.
 *
 * Speed conversion uses the active profile's wheel radius:
 *   ω (rad/s) = v (m/s) / r (m)
 *   v (m/s)   = ω (rad/s) × r (m)
 */

#include "config.h"
#include "fault.h"

/* ──────────────── External References ─────────────────────────── */

extern Fault_State_t g_fault_state;

/* ──────────────── Competition Profiles (flash) ───────────────── */

/**
 * Built-in profiles.  Indexed by Competition_t enum.
 *
 * Speed references:
 *   IGVC  : 1–5 mph   → 0.45–2.24 m/s
 *   IRC   : no strict limit → 0–3.0 m/s (conservative)
 *   UGVC  : 2–10 km/h → 0.56–2.78 m/s
 *   URC   : no strict limit → 0–3.0 m/s
 *   ERC   : no strict limit → 0–2.5 m/s
 */
static const CompetitionConfig_t s_profiles[COMP_COUNT] = {
    /* ── IGVC 2026 ──────────────────────────────────────────────── */
    {
        .speed_min_mps = 0.45f, /* 1 mph minimum                */
        .speed_max_mps = 2.24f, /* 5 mph maximum                */
        .wheel_radius_m = 0.1f, /* 100 mm radius — tune to HW   */
        .heartbeat_timeout_ms = 300U,
        .led_pattern_teleop = 2U, /* LIGHT_TELEOP                 */
        .led_pattern_auto = 3U,   /* LIGHT_AUTONOMOUS             */
    },
    /* ── IRC 2026 ───────────────────────────────────────────────── */
    {
        .speed_min_mps = 0.0f, /* No minimum                   */
        .speed_max_mps = 3.0f, /* Conservative cap             */
        .wheel_radius_m = 0.1f,
        .heartbeat_timeout_ms = 500U, /* Longer timeout for terrain   */
        .led_pattern_teleop = 2U,
        .led_pattern_auto = 3U,
    },
    /* ── UGVC 2025 ──────────────────────────────────────────────── */
    {
        .speed_min_mps = 0.56f, /* 2 km/h minimum               */
        .speed_max_mps = 2.78f, /* 10 km/h maximum              */
        .wheel_radius_m = 0.1f,
        .heartbeat_timeout_ms = 300U,
        .led_pattern_teleop = 2U,
        .led_pattern_auto = 3U,
    },
    /* ── URC 2025 ───────────────────────────────────────────────── */
    {
        .speed_min_mps = 0.0f,
        .speed_max_mps = 3.0f,
        .wheel_radius_m = 0.1f,
        .heartbeat_timeout_ms = 300U,
        .led_pattern_teleop = 2U, /* Blue solid                   */
        .led_pattern_auto = 3U,   /* Red solid                    */
    },
    /* ── ERC 2025 ───────────────────────────────────────────────── */
    {
        .speed_min_mps = 0.0f,
        .speed_max_mps = 2.5f,
        .wheel_radius_m = 0.1f,
        .heartbeat_timeout_ms = 400U,
        .led_pattern_teleop = 2U,
        .led_pattern_auto = 3U,
    },
};

/* ──────────────── Module State ────────────────────────────────── */

/** Currently active competition (runtime-selectable). */
static Competition_t s_active_comp = COMP_IGVC;

/* ──────────────── Public Functions ─────────────────────────────── */

void Config_Init(void) {
  s_active_comp = COMP_IGVC;

  /* Apply default heartbeat timeout to fault module */
  g_fault_state.heartbeat_timeout_ms =
      s_profiles[COMP_IGVC].heartbeat_timeout_ms;
}

int Config_Apply(Competition_t comp) {
  if ((uint8_t)comp >= (uint8_t)COMP_COUNT) {
    return -1; /* Invalid profile */
  }

  s_active_comp = comp;

  /* Update fault module heartbeat timeout from new profile */
  g_fault_state.heartbeat_timeout_ms = s_profiles[comp].heartbeat_timeout_ms;

  return 0;
}

const CompetitionConfig_t *Config_Get(void) {
  return &s_profiles[s_active_comp];
}

Competition_t Config_GetActiveComp(void) { return s_active_comp; }

float Config_SpeedToRadPerSec(float speed_mps) {
  float r = s_profiles[s_active_comp].wheel_radius_m;
  if (r <= 0.001f) {
    r = 0.1f; /* Fallback: 100 mm */
  }
  return speed_mps / r;
}

float Config_RadPerSecToSpeed(float rad_per_sec) {
  return rad_per_sec * s_profiles[s_active_comp].wheel_radius_m;
}
