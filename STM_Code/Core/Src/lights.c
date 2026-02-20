/**
 * @file    lights.c
 * @brief   Multi-color LED indicator controller implementation.
 *
 * Non-blocking flash patterns driven by HAL_GetTick().
 * Called from the main super-loop at >= 20 Hz.
 *
 * Flash timing:
 *   2 Hz flash → 250 ms ON / 250 ms OFF  (FAULT)
 *   5 Hz flash → 100 ms ON / 100 ms OFF  (ESTOP)
 *   1 Hz flash → 500 ms ON / 500 ms OFF  (WAYPOINT)
 */

#include "lights.h"

/* ──────────────── Module State ────────────────────────────────── */

static LightMode_t s_mode = LIGHT_OFF;
static uint32_t s_last_toggle_tick = 0U;
static bool s_flash_state = false; /**< true = LED on during flash  */

/* ──────────────── Helper: set raw GPIO ────────────────────────── */

static inline void led_red(bool on) {
  HAL_GPIO_WritePin(LIGHT_RED_PORT, LIGHT_RED_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void led_green(bool on) {
  HAL_GPIO_WritePin(LIGHT_GREEN_PORT, LIGHT_GREEN_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void led_blue(bool on) {
  HAL_GPIO_WritePin(LIGHT_BLUE_PORT, LIGHT_BLUE_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ──────────────── Flash Helper ────────────────────────────────── */

/**
 * @brief  Check if the flash state should toggle (non-blocking).
 * @param  period_ms  Full flash period in milliseconds (ON + OFF).
 * @return true if flash state changed.
 */
static bool flash_tick(uint32_t period_ms) {
  uint32_t now = HAL_GetTick();
  uint32_t half = period_ms / 2U;

  if ((now - s_last_toggle_tick) >= half) {
    s_last_toggle_tick = now;
    s_flash_state = !s_flash_state;
    return true;
  }
  return false;
}

/* ──────────────── Public Functions ─────────────────────────────── */

void Lights_Init(void) {
  /* All LEDs off at startup */
  led_red(false);
  led_green(false);
  led_blue(false);
  HAL_GPIO_WritePin(LIGHT_LASER_PORT, LIGHT_LASER_PIN, GPIO_PIN_RESET);

  s_mode = LIGHT_OFF;
  s_last_toggle_tick = HAL_GetTick();
  s_flash_state = false;
}

void Lights_SetMode(LightMode_t mode) {
  if (mode != s_mode) {
    s_mode = mode;
    /* Reset flash timing on mode change */
    s_last_toggle_tick = HAL_GetTick();
    s_flash_state = false;
  }
}

LightMode_t Lights_GetMode(void) { return s_mode; }

void Lights_Update(void) {
  switch (s_mode) {

  /* ── All off ───────────────────────────────────────────── */
  case LIGHT_OFF:
    led_red(false);
    led_green(false);
    led_blue(false);
    break;

  /* ── Solid blue — power on ─────────────────────────────── */
  case LIGHT_POWER_ON:
    led_red(false);
    led_green(false);
    led_blue(true);
    break;

  /* ── Solid blue — teleop mode (URC blue) ───────────────── */
  case LIGHT_TELEOP:
    led_red(false);
    led_green(false);
    led_blue(true);
    break;

  /* ── Solid red — autonomous mode (URC red) ─────────────── */
  case LIGHT_AUTONOMOUS:
    led_red(true);
    led_green(false);
    led_blue(false);
    break;

  /* ── Flashing green — waypoint arrival (1 Hz) ──────────── */
  case LIGHT_WAYPOINT_ARRIVAL:
    flash_tick(1000U); /* 500 ms ON / 500 ms OFF */
    led_red(false);
    led_green(s_flash_state);
    led_blue(false);
    break;

  /* ── 2 Hz red flash — recoverable fault ────────────────── */
  case LIGHT_FAULT:
    flash_tick(500U); /* 250 ms ON / 250 ms OFF */
    led_red(s_flash_state);
    led_green(false);
    led_blue(false);
    break;

  /* ── 5 Hz red flash — E-STOP ───────────────────────────── */
  case LIGHT_ESTOP:
    flash_tick(200U); /* 100 ms ON / 100 ms OFF */
    led_red(s_flash_state);
    led_green(false);
    led_blue(false);
    break;

  default:
    /* Unknown mode — turn off for safety */
    led_red(false);
    led_green(false);
    led_blue(false);
    break;
  }
}

void Lights_SetLaser(bool on) {
  HAL_GPIO_WritePin(LIGHT_LASER_PORT, LIGHT_LASER_PIN,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Lights_AllOff(void) {
  s_mode = LIGHT_OFF;
  led_red(false);
  led_green(false);
  led_blue(false);
  HAL_GPIO_WritePin(LIGHT_LASER_PORT, LIGHT_LASER_PIN, GPIO_PIN_RESET);
}
