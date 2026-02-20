/**
 * @file    motor.h
 * @brief   4-wheel BLDC motor driver interface for the UGV.
 *
 * Each motor has:
 *   - A PWM output channel (20 kHz on TIM1/TIM8)
 *   - An encoder input   (TIM2/TIM3/TIM4/TIM5 in encoder mode)
 *   - A direction GPIO pin
 *   - An enable GPIO pin (H-bridge enable / motor relay)
 *   - An overcurrent fault input GPIO
 *
 * PWM duty is expressed as 0–100 % and mapped to the timer ARR range.
 *
 * @author  UGV Firmware Team
 */

#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include <stdbool.h>
#include <stdint.h>


/* For HAL timer/GPIO handles */
#include "stm32h7xx_hal.h"

/* ──────────────────── Configuration ───────────────────────────── */

/** Number of motors on the platform. */
#define MOTOR_COUNT 4U

/** Encoder counts per revolution (adjust per your encoder). */
#define MOTOR_ENCODER_CPR 1024U

/**
 * Velocity conversion factor:  rad/s = (delta_ticks / CPR) * 2π / Δt
 * Pre-computed for Δt = 1 ms.
 */
#define MOTOR_VEL_SCALE                                                        \
  ((2.0f * 3.14159265f) / ((float)MOTOR_ENCODER_CPR * PID_DT))

/* PWM timer auto-reload value – set to match 20 kHz.
 * For a 240 MHz timer clock: ARR = 240 000 000 / 20 000 − 1 = 11999.
 * Adjust if timer clock differs.                                        */
#define MOTOR_PWM_ARR 11999U

/* ──────────────────── Data Structures ─────────────────────────── */

/** Hardware descriptor for one motor channel. */
typedef struct {
  TIM_HandleTypeDef *pwm_timer; /**< PWM timer handle (TIM1/TIM8)        */
  uint32_t pwm_channel;         /**< HAL channel (TIM_CHANNEL_x)         */

  TIM_HandleTypeDef *enc_timer; /**< Encoder timer handle (TIM2/3/4/5)   */

  GPIO_TypeDef *dir_port; /**< Direction GPIO port                 */
  uint16_t dir_pin;       /**< Direction GPIO pin                  */

  GPIO_TypeDef *en_port; /**< Enable/relay GPIO port              */
  uint16_t en_pin;       /**< Enable/relay GPIO pin               */

  GPIO_TypeDef *oc_port; /**< Overcurrent fault input port        */
  uint16_t oc_pin;       /**< Overcurrent fault input pin         */
} Motor_HW_t;

/** Run-time state for one motor. */
typedef struct {
  /* Velocity control */
  float target_velocity;  /**< Desired velocity (rad/s)            */
  float current_velocity; /**< Measured velocity (rad/s)           */
  float pwm_output;       /**< Current PWM duty (0–100 %)          */
  PID_Controller pid;     /**< PID controller instance             */

  /* Encoder tracking */
  int32_t enc_last_count; /**< Previous encoder counter snapshot   */
  int32_t enc_delta;      /**< Ticks since last sample             */

  /* Direction: true = forward, false = reverse */
  bool direction;

  /* Fault flags (latched by fault module) */
  bool stall_flag; /**< Overcurrent detected                */
} Motor_State_t;

/** Combined motor descriptor (hardware + state). */
typedef struct {
  Motor_HW_t hw;
  Motor_State_t state;
} Motor_t;

/* ──────────────────── Public API ──────────────────────────────── */

/**
 * @brief  Initialise all motors: start PWM outputs & encoder timers.
 * @param  motors  Array of MOTOR_COUNT Motor_t, pre-filled with HW config.
 */
void Motor_Init(Motor_t motors[MOTOR_COUNT]);

/**
 * @brief  Read encoder hardware and compute velocity for one motor.
 * @param  motor  Pointer to Motor_t.
 *
 * Updates motor->state.enc_delta and motor->state.current_velocity.
 */
void Motor_ReadEncoder(Motor_t *motor);

/**
 * @brief  Write PWM duty cycle and direction for one motor.
 * @param  motor  Pointer to Motor_t.
 * @param  duty   Duty cycle 0–100 %.
 * @param  fwd    true = forward, false = reverse.
 */
void Motor_SetPWM(Motor_t *motor, float duty, bool fwd);

/**
 * @brief  Emergency stop: zero PWM and disable motor relay for ALL motors.
 * @param  motors  Array of MOTOR_COUNT Motor_t.
 */
void Motor_EmergencyStop(Motor_t motors[MOTOR_COUNT]);

/**
 * @brief  Enable motor relay / H-bridge for ALL motors.
 * @param  motors  Array of MOTOR_COUNT Motor_t.
 */
void Motor_EnableAll(Motor_t motors[MOTOR_COUNT]);

/**
 * @brief  Read the overcurrent fault input GPIO for one motor.
 * @param  motor  Pointer to Motor_t.
 * @return true if overcurrent is asserted (active-low assumed).
 */
bool Motor_ReadOvercurrent(const Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
