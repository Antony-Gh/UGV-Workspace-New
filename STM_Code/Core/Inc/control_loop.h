/**
 * @file    control_loop.h
 * @brief   1 kHz deterministic control loop for the UGV.
 *
 * Triggered by TIM6 update interrupt (1 ms period).
 * Executes the full motor-control pipeline:
 *   1. Debug GPIO high
 *   2. Read encoders → compute velocity
 *   3. Apply velocity commands from CAN
 *   4. Run PID for each motor
 *   5. Write PWM outputs
 *   6. Run fault checks
 *   7. Update safety state machine
 *   8. Debug GPIO low
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

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOOP_H */
