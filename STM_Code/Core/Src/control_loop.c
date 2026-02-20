/**
 * @file    control_loop.c
 * @brief   1 kHz deterministic control loop — enhanced with speed limits and
 * odometry.
 *
 * All time-critical work is executed inside the TIM6 update ISR.
 * The loop body must complete in < 200 µs worst-case.
 *
 * Flow (every 1 ms):
 *   1. Toggle debug GPIO HIGH          — oscilloscope timing start
 *   2. Read 4 encoder counters         — compute wheel velocities
 *   3. Latch CAN velocity commands     — clamp to competition speed limits
 *   4. Accumulate odometry             — distance + differential heading
 *   5. Run PID for each motor          — compute PWM duty
 *   6. Apply PWM duty & direction      — write to timer CCR & GPIOs
 *   7. Read overcurrent GPIOs          — collect fault inputs
 *   8. Run Fault_Check()               — update fault bitmask
 *   9. Run Safety_Update()             — advance state machine
 *  10. Toggle debug GPIO LOW           — oscilloscope timing end
 */

#include "control_loop.h"
#include "can_comm.h"
#include "config.h"
#include "pid.h"
#include "safety.h"
#include <math.h>

/* ──────────────── Module-Level Pointers ───────────────────────── */

static TIM_HandleTypeDef *s_htim6 = NULL;
static Motor_t *s_motors = NULL;
static Fault_State_t *s_fault_state = NULL;

/* ──────────────── Odometry State ──────────────────────────────── */

/**
 * Accumulated odometry — updated every 1 ms in ISR.
 * distance is accumulated in meters (float), converted to mm on read.
 * heading is accumulated in radians (float), converted to cdeg on read.
 */
static volatile float s_odom_distance_m = 0.0f;
static volatile float s_odom_heading_rad = 0.0f;

/* ──────────────── Private: Speed Clamp ───────────────────────── */

/**
 * @brief  Clamp a velocity command (rad/s) to competition speed limits.
 * @param  cmd_rad_s  Desired velocity in rad/s (can be negative).
 * @return Clamped velocity in rad/s, preserving sign.
 *
 * Uses the active competition config's speed_max_mps converted to rad/s.
 * speed_min_mps is enforced only if the commanded speed is non-zero
 * (to allow zero / stop commands).
 */
static float clamp_velocity(float cmd_rad_s) {
  const CompetitionConfig_t *cfg = Config_Get();

  float max_rad = Config_SpeedToRadPerSec(cfg->speed_max_mps);
  float min_rad = Config_SpeedToRadPerSec(cfg->speed_min_mps);

  float abs_cmd = fabsf(cmd_rad_s);
  float sign = (cmd_rad_s >= 0.0f) ? 1.0f : -1.0f;

  /* Allow zero (stop) commands */
  if (abs_cmd < 0.001f) {
    return 0.0f;
  }

  /* Enforce minimum speed if configured */
  if (min_rad > 0.0f && abs_cmd < min_rad) {
    abs_cmd = min_rad;
  }

  /* Enforce maximum speed */
  if (abs_cmd > max_rad) {
    abs_cmd = max_rad;
  }

  return abs_cmd * sign;
}

/* ──────────────── Public Functions ─────────────────────────────── */

void ControlLoop_Init(TIM_HandleTypeDef *htim6, Motor_t *motors,
                      Fault_State_t *fault_state) {
  s_htim6 = htim6;
  s_motors = motors;
  s_fault_state = fault_state;

  /* Zero odometry */
  s_odom_distance_m = 0.0f;
  s_odom_heading_rad = 0.0f;

  /* Start TIM6 in interrupt mode — generates 1 ms update events */
  HAL_TIM_Base_Start_IT(htim6);
}

void ControlLoop_TimerCallback(TIM_HandleTypeDef *htim) {
  /* Only respond to TIM6 */
  if (htim->Instance != TIM6)
    return;

  /* ── 1. Debug timing — start ─────────────────────────────── */
  Safety_DebugToggle();

  /* ── 2. Read encoders & compute velocity ─────────────────── */
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    Motor_ReadEncoder(&s_motors[i]);
  }

  /* ── 3. Latch velocity commands from CAN (with speed clamping) */
  if (g_can_velocity_cmd_new && Safety_IsActive()) {
    /*
     * Convert int16 scaled values to float rad/s, clamp to config
     * speed limits, and apply.
     * Order: [0]=FL, [1]=FR, [2]=RL, [3]=RR
     */
    float cmd[4];
    cmd[0] = (float)g_can_velocity_cmd.velocity_fl / CAN_VELOCITY_SCALE;
    cmd[1] = (float)g_can_velocity_cmd.velocity_fr / CAN_VELOCITY_SCALE;
    cmd[2] = (float)g_can_velocity_cmd.velocity_rl / CAN_VELOCITY_SCALE;
    cmd[3] = (float)g_can_velocity_cmd.velocity_rr / CAN_VELOCITY_SCALE;

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
      float clamped = clamp_velocity(cmd[i]);
      s_motors[i].state.target_velocity = fabsf(clamped);
      s_motors[i].state.direction = (clamped >= 0.0f);
    }

    g_can_velocity_cmd_new = false;
  }

  /* If NOT active, zero all targets */
  if (!Safety_IsActive()) {
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
      s_motors[i].state.target_velocity = 0.0f;
    }
  }

  /* ── 4. Accumulate odometry ──────────────────────────────── */
  {
    const CompetitionConfig_t *cfg = Config_Get();
    float r = cfg->wheel_radius_m;

    /*
     * Average left/right velocities for distance and heading.
     * Left  = average of FL (0) and RL (2)
     * Right = average of FR (1) and RR (3)
     *
     * v_linear = (v_left + v_right) / 2   (angular rad/s)
     * ω_yaw    = (v_right - v_left) / track_width
     *
     * Distance increment: v_linear × r × dt
     * Heading increment:  ω_yaw × dt
     */
    float v_left = (s_motors[0].state.current_velocity +
                    s_motors[2].state.current_velocity) *
                   0.5f;
    float v_right = (s_motors[1].state.current_velocity +
                     s_motors[3].state.current_velocity) *
                    0.5f;

    float v_linear = (v_left + v_right) * 0.5f; /* rad/s average */
    float omega_yaw =
        (v_right - v_left) * r / CONTROL_LOOP_TRACK_WIDTH_M; /* rad/s */

    /* Integrate over 1 ms (PID_DT = 0.001) */
    s_odom_distance_m += fabsf(v_linear) * r * PID_DT;
    s_odom_heading_rad += omega_yaw * PID_DT;
  }

  /* ── 5. Run PID for each motor ───────────────────────────── */
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    float duty =
        PID_Compute(&s_motors[i].state.pid, s_motors[i].state.target_velocity,
                    fabsf(s_motors[i].state.current_velocity));
    s_motors[i].state.pwm_output = duty;
  }

  /* ── 6. Apply PWM duty & direction ───────────────────────── */
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    Motor_SetPWM(&s_motors[i], s_motors[i].state.pwm_output,
                 s_motors[i].state.direction);
  }

  /* ── 7. Collect fault inputs ─────────────────────────────── */
  float pwm_duty[4];
  int32_t enc_delta[4];
  bool overcurrent[4];

  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    pwm_duty[i] = s_motors[i].state.pwm_output;
    enc_delta[i] = s_motors[i].state.enc_delta;
    overcurrent[i] = Motor_ReadOvercurrent(&s_motors[i]);
  }

  /* ── 8. Run fault detection ──────────────────────────────── */
  Fault_Check(s_fault_state, HAL_GetTick(), pwm_duty, enc_delta, overcurrent);

  /* ── 9. Safety state machine ─────────────────────────────── */
  Safety_Update(s_motors, s_fault_state);

  /* ── 10. Debug timing — end ──────────────────────────────── */
  Safety_DebugToggle();
}

void ControlLoop_GetOdometry(int32_t *distance_mm, int16_t *heading_cdeg) {
  /* Convert accumulated float values to integer representations.
   * Reads are effectively atomic on Cortex-M7 for aligned 32-bit floats. */
  *distance_mm = (int32_t)(s_odom_distance_m * 1000.0f);

  /* Convert radians to centidegrees: rad × (180/π) × 100 = rad × 5729.578 */
  *heading_cdeg = (int16_t)(s_odom_heading_rad * 5729.578f);
}

void ControlLoop_ResetOdometry(void) {
  s_odom_distance_m = 0.0f;
  s_odom_heading_rad = 0.0f;
}
