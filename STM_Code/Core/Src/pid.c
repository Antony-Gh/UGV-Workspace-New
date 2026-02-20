/**
 * @file    pid.c
 * @brief   Velocity PID controller implementation.
 *
 * Fixed-timestep discrete PID with:
 *   - Integral anti-windup (clamping)
 *   - Output saturation [0, 100] %
 *   - Derivative-on-error (standard form)
 *
 * Called once per control cycle (1 ms) from the TIM6 ISR.
 */

#include "pid.h"

/* ─────────────────── Helper: float clamp ──────────────────────── */

static inline float clampf(float value, float lo, float hi) {
  if (value < lo)
    return lo;
  if (value > hi)
    return hi;
  return value;
}

/* ────────────────────── Public Functions ───────────────────────── */

/**
 * @brief Initialise a PID controller.
 *
 * Sets gains and limits; zeros internal state.
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit) {
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;

  pid->integral_limit = integral_limit;
  pid->output_min = PID_OUTPUT_MIN;
  pid->output_max = PID_OUTPUT_MAX;
}

/**
 * @brief Compute one PID cycle.
 *
 * Algorithm (discrete, fixed Δt = PID_DT):
 *   error       = setpoint − measured
 *   P           = Kp × error
 *   integral   += Ki × error × Δt       (clamped to ±integral_limit)
 *   derivative  = Kd × (error − prev_error) / Δt
 *   output      = clamp(P + I + D, output_min, output_max)
 *
 * @return Clamped PWM duty percentage [0, 100].
 */
float PID_Compute(PID_Controller *pid, float setpoint, float measured) {
  float error = setpoint - measured;

  /* --- Proportional --- */
  float p_term = pid->Kp * error;

  /* --- Integral with anti-windup --- */
  pid->integral += pid->Ki * error * PID_DT;
  pid->integral =
      clampf(pid->integral, -pid->integral_limit, pid->integral_limit);

  /* --- Derivative --- */
  float d_term = pid->Kd * (error - pid->prev_error) / PID_DT;
  pid->prev_error = error;

  /* --- Sum & clamp output --- */
  float output = p_term + pid->integral + d_term;
  output = clampf(output, pid->output_min, pid->output_max);

  return output;
}

/**
 * @brief Reset PID state (call on mode change or fault recovery).
 */
void PID_Reset(PID_Controller *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}
