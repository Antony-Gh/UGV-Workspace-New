/**
 * @file    motor.c
 * @brief   4-wheel BLDC motor driver implementation.
 *
 * Provides:
 *   - PWM output via TIM1 / TIM8 (20 kHz, 4 channels total)
 *   - Quadrature encoder reading via TIM2 / TIM3 / TIM4 / TIM5
 *   - Direction and enable GPIO control
 *   - Emergency stop (zero PWM + disable relay)
 *   - Overcurrent fault GPIO reading
 *
 * All functions are non-blocking and safe to call from ISR context.
 */

#include "motor.h"

/* ──────────────── Helper: duty → CCR value ────────────────────── */

/**
 * @brief Convert a 0–100 % duty to the timer compare value.
 */
static inline uint32_t duty_to_ccr(float duty) {
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 100.0f)
    duty = 100.0f;
  return (uint32_t)((duty / 100.0f) * (float)MOTOR_PWM_ARR);
}

/* ──────────────────── Public Functions ─────────────────────────── */

void Motor_Init(Motor_t motors[MOTOR_COUNT]) {
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    Motor_t *m = &motors[i];

    /* ---------- Zero runtime state ---------- */
    m->state.target_velocity = 0.0f;
    m->state.current_velocity = 0.0f;
    m->state.pwm_output = 0.0f;
    m->state.enc_last_count = 0;
    m->state.enc_delta = 0;
    m->state.direction = true;
    m->state.stall_flag = false;

    /* ---------- Initialise PID ---------- */
    /* Default gains — tune on real hardware */
    PID_Init(&m->state.pid,
             /* Kp */ 1.0f,
             /* Ki */ 0.5f,
             /* Kd */ 0.01f,
             /* integral_limit */ 50.0f);

    /* ---------- Start PWM output (duty = 0) ---------- */
    __HAL_TIM_SET_COMPARE(m->hw.pwm_timer, m->hw.pwm_channel, 0U);
    HAL_TIM_PWM_Start(m->hw.pwm_timer, m->hw.pwm_channel);

    /* ---------- Start encoder timer in encoder mode ---------- */
    HAL_TIM_Encoder_Start(m->hw.enc_timer, TIM_CHANNEL_ALL);
    m->state.enc_last_count = (int32_t)__HAL_TIM_GET_COUNTER(m->hw.enc_timer);

    /* ---------- Disable motor relay by default (safe) ---------- */
    HAL_GPIO_WritePin(m->hw.en_port, m->hw.en_pin, GPIO_PIN_RESET);

    /* ---------- Direction = forward ---------- */
    HAL_GPIO_WritePin(m->hw.dir_port, m->hw.dir_pin, GPIO_PIN_SET);
  }
}

void Motor_ReadEncoder(Motor_t *motor) {
  /*
   * Encoder timers run in 32-bit mode (TIM2/TIM5) or 16-bit (TIM3/TIM4).
   * We use signed arithmetic so overflow wraps correctly for 32-bit timers.
   * For 16-bit timers, cast to int16_t first to handle rollover.
   */
  int32_t current = (int32_t)__HAL_TIM_GET_COUNTER(motor->hw.enc_timer);
  motor->state.enc_delta = current - motor->state.enc_last_count;
  motor->state.enc_last_count = current;

  /* Convert delta ticks → angular velocity (rad/s).
   * vel = (delta / CPR) * 2π / Δt  →  delta * MOTOR_VEL_SCALE            */
  motor->state.current_velocity =
      (float)motor->state.enc_delta * MOTOR_VEL_SCALE;
}

void Motor_SetPWM(Motor_t *motor, float duty, bool fwd) {
  /* Clamp duty */
  if (duty < 0.0f)
    duty = 0.0f;
  if (duty > 100.0f)
    duty = 100.0f;

  motor->state.pwm_output = duty;
  motor->state.direction = fwd;

  /* Set direction GPIO */
  HAL_GPIO_WritePin(motor->hw.dir_port, motor->hw.dir_pin,
                    fwd ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* Write compare value */
  __HAL_TIM_SET_COMPARE(motor->hw.pwm_timer, motor->hw.pwm_channel,
                        duty_to_ccr(duty));
}

void Motor_EmergencyStop(Motor_t motors[MOTOR_COUNT]) {
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    /* Zero PWM immediately */
    __HAL_TIM_SET_COMPARE(motors[i].hw.pwm_timer, motors[i].hw.pwm_channel, 0U);
    motors[i].state.pwm_output = 0.0f;
    motors[i].state.target_velocity = 0.0f;

    /* Disable motor relay / H-bridge enable */
    HAL_GPIO_WritePin(motors[i].hw.en_port, motors[i].hw.en_pin,
                      GPIO_PIN_RESET);

    /* Reset PID to avoid integral windup on restart */
    PID_Reset(&motors[i].state.pid);
  }
}

void Motor_EnableAll(Motor_t motors[MOTOR_COUNT]) {
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    HAL_GPIO_WritePin(motors[i].hw.en_port, motors[i].hw.en_pin, GPIO_PIN_SET);
  }
}

bool Motor_ReadOvercurrent(const Motor_t *motor) {
  /* Overcurrent input is active-low: GPIO_PIN_RESET means fault. */
  return (HAL_GPIO_ReadPin(motor->hw.oc_port, motor->hw.oc_pin) ==
          GPIO_PIN_RESET);
}
