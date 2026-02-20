/**
 * @file    control_loop.c
 * @brief   1 kHz deterministic control loop implementation.
 *
 * All time-critical work is executed inside the TIM6 update ISR.
 * The loop body must complete in < 200 µs worst-case.
 *
 * Flow (every 1 ms):
 *   1. Toggle debug GPIO HIGH          — oscilloscope timing start
 *   2. Read 4 encoder counters         — compute wheel velocities
 *   3. Latch CAN velocity commands     — update motor setpoints
 *   4. Run PID for each motor          — compute PWM duty
 *   5. Apply PWM duty & direction      — write to timer CCR & GPIOs
 *   6. Read overcurrent GPIOs          — collect fault inputs
 *   7. Run Fault_Check()               — update fault bitmask
 *   8. Run Safety_Update()             — advance state machine
 *   9. Toggle debug GPIO LOW           — oscilloscope timing end
 */

#include "control_loop.h"
#include "can_comm.h"
#include "pid.h"
#include "safety.h"
#include <math.h>


/* ──────────────── Module-Level Pointers ───────────────────────── */

static TIM_HandleTypeDef *s_htim6 = NULL;
static Motor_t *s_motors = NULL;
static Fault_State_t *s_fault_state = NULL;

/* ──────────────── Public Functions ─────────────────────────────── */

void ControlLoop_Init(TIM_HandleTypeDef *htim6, Motor_t *motors,
                      Fault_State_t *fault_state) {
  s_htim6 = htim6;
  s_motors = motors;
  s_fault_state = fault_state;

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

  /* ── 3. Latch velocity commands from CAN ─────────────────── */
  if (g_can_velocity_cmd_new && Safety_GetState() == SYSTEM_ACTIVE) {
    /*
     * Convert int16 scaled values to float rad/s and apply.
     * Order: [0]=FL, [1]=FR, [2]=RL, [3]=RR
     */
    float cmd[4];
    cmd[0] = (float)g_can_velocity_cmd.velocity_fl / CAN_VELOCITY_SCALE;
    cmd[1] = (float)g_can_velocity_cmd.velocity_fr / CAN_VELOCITY_SCALE;
    cmd[2] = (float)g_can_velocity_cmd.velocity_rl / CAN_VELOCITY_SCALE;
    cmd[3] = (float)g_can_velocity_cmd.velocity_rr / CAN_VELOCITY_SCALE;

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
      s_motors[i].state.target_velocity = fabsf(cmd[i]);
      s_motors[i].state.direction = (cmd[i] >= 0.0f);
    }

    g_can_velocity_cmd_new = false;
  }

  /* If NOT active, zero all targets */
  if (Safety_GetState() != SYSTEM_ACTIVE) {
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
      s_motors[i].state.target_velocity = 0.0f;
    }
  }

  /* ── 4. Run PID for each motor ───────────────────────────── */
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    float duty =
        PID_Compute(&s_motors[i].state.pid, s_motors[i].state.target_velocity,
                    fabsf(s_motors[i].state.current_velocity));
    s_motors[i].state.pwm_output = duty;
  }

  /* ── 5. Apply PWM duty & direction ───────────────────────── */
  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    Motor_SetPWM(&s_motors[i], s_motors[i].state.pwm_output,
                 s_motors[i].state.direction);
  }

  /* ── 6. Collect fault inputs ─────────────────────────────── */
  float pwm_duty[4];
  int32_t enc_delta[4];
  bool overcurrent[4];

  for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
    pwm_duty[i] = s_motors[i].state.pwm_output;
    enc_delta[i] = s_motors[i].state.enc_delta;
    overcurrent[i] = Motor_ReadOvercurrent(&s_motors[i]);
  }

  /* ── 7. Run fault detection ──────────────────────────────── */
  Fault_Check(s_fault_state, HAL_GetTick(), pwm_duty, enc_delta, overcurrent);

  /* ── 8. Safety state machine ─────────────────────────────── */
  Safety_Update(s_motors, s_fault_state);

  /* ── 9. Debug timing — end ───────────────────────────────── */
  Safety_DebugToggle();
}
