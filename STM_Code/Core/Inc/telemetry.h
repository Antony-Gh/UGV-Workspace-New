/**
 * @file    telemetry.h
 * @brief   Periodic telemetry transmission for the UGV — expanded channels.
 *
 * Sends three CAN frames at different intervals from the main loop:
 *   0x300 – Motor telemetry (50 ms)
 *   0x310 – Odometry        (100 ms)
 *   0x320 – Battery          (200 ms)
 *
 * Uses HAL_GetTick() — no blocking delays.
 *
 * @author  UGV Firmware Team
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fault.h"
#include "motor.h"
#include "safety.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ──────────────── Configuration ───────────────────────────────── */

/** Telemetry interval for motor data (0x300) in milliseconds. */
#define TELEMETRY_MOTOR_INTERVAL_MS 50U

/** Telemetry interval for odometry (0x310) in milliseconds. */
#define TELEMETRY_ODOM_INTERVAL_MS 100U

/** Telemetry interval for battery (0x320) in milliseconds. */
#define TELEMETRY_BATT_INTERVAL_MS 200U

/* ──────────────── ADC Configuration ──────────────────────────── */

/**
 * ADC scale factors for battery monitoring.
 * Adjust these to match your voltage divider and current sensor.
 *
 * Example:  12-bit ADC, 3.3V ref, 1:11 voltage divider for 36V battery
 *   ADC_to_mV = (raw / 4095) × 3300 × 11
 *
 * Example:  ACS712 current sensor, 185 mV/A, Vref/2 zero-current
 *   ADC_to_mA = ((raw / 4095) × 3300 - 1650) / 0.185
 */
#define ADC_VREF_MV 3300U         /**< ADC reference voltage (mV)       */
#define ADC_RESOLUTION 4095U      /**< 12-bit ADC max value             */
#define BATT_VDIV_RATIO 11U       /**< Voltage divider ratio            */
#define BATT_ISENSE_MV_PER_A 185U /**< Current sensor sensitivity       */
#define BATT_ISENSE_ZERO_MV 1650U /**< Current sensor zero-current mV   */

/* ──────────────── Public API ──────────────────────────────────── */

/**
 * @brief  Initialise telemetry timing and ADC handle.
 * @param  hadc  Pointer to ADC handle for battery monitoring (ADC1).
 */
void Telemetry_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief  Call from main loop — sends telemetry on staggered schedules.
 * @param  motors       Array of MOTOR_COUNT motors (read velocities).
 * @param  fault_state  Pointer to fault state (read fault flags).
 *
 * Non-blocking: returns immediately if no interval has elapsed.
 * Handles 0x300 (motor), 0x310 (odometry), and 0x320 (battery) frames.
 */
void Telemetry_Update(const Motor_t motors[MOTOR_COUNT],
                      const Fault_State_t *fault_state);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
