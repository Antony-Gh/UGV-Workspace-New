/**
 * @file    lights.h
 * @brief   Multi-color LED indicator controller for the UGV.
 *
 * Drives RGB LEDs and an optional laser pointer GPIO to indicate
 * system mode and state.  Patterns are non-blocking, toggled by
 * tick-based counters in Lights_Update().
 *
 * Competition LED requirements:
 *   IGVC  — Yellow ON = power, flashing = moving
 *   UGVC  — Solid = power ON, flashing = autonomous
 *   URC   — Red = autonomous, Blue = teleop, Flashing Green = waypoint
 *   ERC   — Per requirements appendix
 *
 * GPIO pin assignments (adjust to PCB):
 *   Red   : PF0   Green : PF1   Blue  : PF2   Laser : PF3
 *
 * @author  UGV Firmware Team
 */

#ifndef LIGHTS_H
#define LIGHTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ──────────────────── GPIO Configuration ──────────────────────── */

#ifndef LIGHT_RED_PORT
#define LIGHT_RED_PORT GPIOF
#endif
#ifndef LIGHT_RED_PIN
#define LIGHT_RED_PIN GPIO_PIN_0
#endif

#ifndef LIGHT_GREEN_PORT
#define LIGHT_GREEN_PORT GPIOF
#endif
#ifndef LIGHT_GREEN_PIN
#define LIGHT_GREEN_PIN GPIO_PIN_1
#endif

#ifndef LIGHT_BLUE_PORT
#define LIGHT_BLUE_PORT GPIOF
#endif
#ifndef LIGHT_BLUE_PIN
#define LIGHT_BLUE_PIN GPIO_PIN_2
#endif

#ifndef LIGHT_LASER_PORT
#define LIGHT_LASER_PORT GPIOF
#endif
#ifndef LIGHT_LASER_PIN
#define LIGHT_LASER_PIN GPIO_PIN_3
#endif

/* ──────────────────── Light Modes ─────────────────────────────── */

/**
 * @brief Operating modes for the LED controller.
 *
 * Each mode defines which LEDs are active and their flash pattern.
 */
typedef enum {
  LIGHT_OFF = 0,              /**< All LEDs off                          */
  LIGHT_POWER_ON = 1,         /**< Solid blue — system powered           */
  LIGHT_TELEOP = 2,           /**< Solid blue — teleop mode (URC)        */
  LIGHT_AUTONOMOUS = 3,       /**< Solid red  — autonomous mode (URC)    */
  LIGHT_WAYPOINT_ARRIVAL = 4, /**< Flashing green — waypoint reached    */
  LIGHT_FAULT = 5,            /**< 2 Hz red flash — recoverable fault    */
  LIGHT_ESTOP = 6,            /**< 5 Hz red flash — emergency stop       */
} LightMode_t;

/* ──────────────────── Public API ──────────────────────────────── */

/**
 * @brief  Initialise LED GPIO outputs (all OFF).
 */
void Lights_Init(void);

/**
 * @brief  Set the current light mode.
 * @param  mode  LightMode_t value.
 *
 * Takes effect on the next Lights_Update() call.
 */
void Lights_SetMode(LightMode_t mode);

/**
 * @brief  Get the current light mode.
 * @return Current LightMode_t.
 */
LightMode_t Lights_GetMode(void);

/**
 * @brief  Periodic update — call from super-loop (~20 Hz or faster).
 *
 * Drives LEDs according to the current mode.  Flash patterns use
 * HAL_GetTick() and are completely non-blocking.
 */
void Lights_Update(void);

/**
 * @brief  Enable or disable the laser pointer GPIO (UGVC).
 * @param  on  true = laser ON, false = OFF.
 */
void Lights_SetLaser(bool on);

/**
 * @brief  Force all LEDs + laser off immediately (called from E-STOP ISR).
 *
 * ISR-safe: no blocking, no HAL_Delay.
 */
void Lights_AllOff(void);

#ifdef __cplusplus
}
#endif

#endif /* LIGHTS_H */
