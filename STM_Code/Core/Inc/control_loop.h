/**
 * @file    control_loop.h
 * @brief   1 kHz deterministic control loop for the UGV.
 *
 * Triggered by TIM6 update interrupt (1 ms period).
 * Executes the full motor-control pipeline:
 *   1. Debug GPIO high
 *   2. Read encoders → compute velocity
 *   3. Apply velocity commands from CAN (clamped to speed limits)
 *   4. Accumulate odometry (distance + heading)
 *   5. Run PID for each motor
 *   6. Write PWM outputs
 *   7. Run fault checks
 *   8. Update safety state machine
 *   9. Debug GPIO low
 *
 * Worst-case execution must be < 200 µs.
 *
 * @author  UGV Firmware Team
 */

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fault.h"
#include "motor.h"
#include "stm32h7xx_hal.h"

/* ──────────────── Odometry Configuration ─────────────────────── */

/**
 * Wheel track width (distance between left and right wheel centres)
 * in meters.  Used for differential heading estimate.
 * Adjust to match your UGV geometry.
 */
#ifndef CONTROL_LOOP_TRACK_WIDTH_M
#define CONTROL_LOOP_TRACK_WIDTH_M 0.40f
#endif

/* ──────────────── Public API ──────────────────────────────────── */

/**
 * @brief  Initialise and start the 1 kHz control loop (TIM6).
 * @param  htim6        Pointer to TIM6 HAL handle.
 * @param  motors       Pointer to motor array (MOTOR_COUNT elements).
 * @param  fault_state  Pointer to global fault state.
 */
void ControlLoop_Init(TIM_HandleTypeDef *htim6, Motor_t *motors,
                      Fault_State_t *fault_state);

/**
 * @brief  TIM6 period-elapsed callback — called from HAL ISR.
 * @param  htim  Timer handle (verified to be TIM6 inside).
 *
 * Connect this to HAL_TIM_PeriodElapsedCallback().
 */
void ControlLoop_TimerCallback(TIM_HandleTypeDef *htim);

/**
 * @brief  Get accumulated odometry values.
 * @param  distance_mm   Output: total distance in millimeters (int32).
 * @param  heading_cdeg  Output: heading in centidegrees (int16).
 *
 * Values are reset to zero on system reset.
 * Thread-safe: reads are atomic on Cortex-M7 for aligned 32-bit values.
 */
void ControlLoop_GetOdometry(int32_t *distance_mm, int16_t *heading_cdeg);

/**
 * @brief  Reset accumulated odometry to zero.
 */
void ControlLoop_ResetOdometry(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
