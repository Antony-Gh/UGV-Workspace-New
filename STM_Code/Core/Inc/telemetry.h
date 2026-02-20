/**
 * @file    telemetry.h
 * @brief   Periodic telemetry transmission for the UGV.
 *
 * Sends CAN 0x300 at a 50 ms interval from the main loop.
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
#include <stdint.h>


/* ──────────────── Configuration ───────────────────────────────── */

/** Telemetry transmission interval in milliseconds. */
#define TELEMETRY_INTERVAL_MS 50U

/* ──────────────── Public API ──────────────────────────────────── */

/**
 * @brief  Initialise telemetry timing.
 */
void Telemetry_Init(void);

/**
 * @brief  Call from main loop — sends telemetry if 50 ms have elapsed.
 * @param  motors       Array of MOTOR_COUNT motors (read velocities).
 * @param  fault_state  Pointer to fault state (read fault flags).
 *
 * Non-blocking: returns immediately if interval has not elapsed.
 */
void Telemetry_Update(const Motor_t motors[MOTOR_COUNT],
                      const Fault_State_t *fault_state);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
