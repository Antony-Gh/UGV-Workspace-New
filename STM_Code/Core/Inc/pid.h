/**
 * @file    pid.h
 * @brief   Velocity PID controller with anti-windup for UGV motor control.
 *
 * Fixed-timestep (1 ms) PID implementation.
 * - Integral anti-windup via clamping
 * - Output clamped to [0, 100] % PWM duty cycle
 * - No dynamic allocation
 *
 * @author  UGV Firmware Team
 */

#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ───────────────────────── Configuration ───────────────────────── */

/** Fixed control loop timestep in seconds (1 kHz → 1 ms). */
#define PID_DT  0.001f

/** Default output limits (PWM duty %). */
#define PID_OUTPUT_MIN  0.0f
#define PID_OUTPUT_MAX  100.0f

/* ───────────────────────── Data Structures ─────────────────────── */

/**
 * @brief PID controller state.
 *
 * All fields are in engineering units (rad/s for velocity control).
 * The integral term is bounded by ±integral_limit to prevent windup.
 */
typedef struct {
    /* Gains */
    float Kp;               /**< Proportional gain              */
    float Ki;               /**< Integral gain                  */
    float Kd;               /**< Derivative gain                */

    /* State */
    float integral;          /**< Accumulated integral term      */
    float prev_error;        /**< Previous-cycle error           */

    /* Limits */
    float integral_limit;    /**< Anti-windup: max |integral|    */
    float output_min;        /**< Minimum output (typically 0)   */
    float output_max;        /**< Maximum output (typically 100) */
} PID_Controller;

/* ────────────────────────── Public API ─────────────────────────── */

/**
 * @brief  Initialise a PID controller with the given gains and limits.
 * @param  pid             Pointer to PID controller instance.
 * @param  kp              Proportional gain.
 * @param  ki              Integral gain.
 * @param  kd              Derivative gain.
 * @param  integral_limit  Anti-windup clamp for integral accumulator.
 */
void PID_Init(PID_Controller *pid,
              float kp, float ki, float kd,
              float integral_limit);

/**
 * @brief  Compute one PID iteration.
 * @param  pid       Pointer to PID controller instance.
 * @param  setpoint  Desired value (rad/s).
 * @param  measured  Current measured value (rad/s).
 * @return Clamped control output in [output_min, output_max].
 *
 * Must be called at a fixed 1 ms interval (PID_DT).
 */
float PID_Compute(PID_Controller *pid, float setpoint, float measured);

/**
 * @brief  Reset PID internal state (integral & derivative memory).
 * @param  pid  Pointer to PID controller instance.
 */
void PID_Reset(PID_Controller *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
