/**
 * @file    fault.c
 * @brief   Fault detection implementation.
 *
 * Called every 1 ms from the control loop to check:
 *   1. Heartbeat timeout  (>300 ms)
 *   2. Motor stall        (overcurrent flag while PWM active)
 *   3. Encoder failure    (no change for 100 consecutive cycles while driving)
 *   4. CAN bus error      (set externally via Fault_SetCANError)
 */

#include "fault.h"
#include <string.h>

/* ────────────────────── Public Functions ───────────────────────── */

void Fault_Init(Fault_State_t *state) {
  memset(state, 0, sizeof(Fault_State_t));
  state->active_faults = FAULT_NONE;
  state->last_heartbeat_tick = 0U; /* Will be set on first heartbeat */
}

void Fault_Check(Fault_State_t *state, uint32_t current_tick,
                 const float pwm_duty[4], const int32_t encoder_delta[4],
                 const bool overcurrent[4]) {
  /* ── 1. Heartbeat timeout ─────────────────────────────────── */
  /*    Only check if we have ever received a heartbeat.        */
  if (state->last_heartbeat_tick != 0U) {
    uint32_t elapsed = current_tick - state->last_heartbeat_tick;
    if (elapsed > FAULT_HEARTBEAT_TIMEOUT_MS) {
      state->active_faults |= FAULT_HEARTBEAT_TIMEOUT;
    } else {
      state->active_faults &= ~FAULT_HEARTBEAT_TIMEOUT;
    }
  }

  /* ── 2. Motor stall (overcurrent) ────────────────────────── */
  {
    bool any_stall = false;
    for (uint8_t i = 0U; i < 4U; i++) {
      state->motor_overcurrent_flag[i] = overcurrent[i];
      if (overcurrent[i] && (pwm_duty[i] > 0.0f)) {
        any_stall = true;
      }
    }
    if (any_stall)
      state->active_faults |= FAULT_MOTOR_STALL;
    else
      state->active_faults &= ~FAULT_MOTOR_STALL;
  }

  /* ── 3. Encoder failure ──────────────────────────────────── */
  {
    bool any_encoder_fail = false;
    for (uint8_t i = 0U; i < 4U; i++) {
      if (pwm_duty[i] > FAULT_ENCODER_PWM_THRESHOLD) {
        /* Motor is being driven — encoder should be moving */
        if (encoder_delta[i] == 0) {
          state->encoder_fail_counter[i]++;
          if (state->encoder_fail_counter[i] >= FAULT_ENCODER_FAIL_COUNT) {
            any_encoder_fail = true;
          }
        } else {
          state->encoder_fail_counter[i] = 0U;
        }
      } else {
        /* Motor not driven — reset counter */
        state->encoder_fail_counter[i] = 0U;
      }
    }
    if (any_encoder_fail)
      state->active_faults |= FAULT_ENCODER_FAILURE;
    else
      state->active_faults &= ~FAULT_ENCODER_FAILURE;
  }
}

void Fault_HeartbeatReceived(Fault_State_t *state, uint32_t current_tick) {
  state->last_heartbeat_tick = current_tick;
  state->active_faults &= ~FAULT_HEARTBEAT_TIMEOUT;
}

void Fault_SetCANError(Fault_State_t *state, bool error) {
  if (error)
    state->active_faults |= FAULT_CAN_ERROR;
  else
    state->active_faults &= ~FAULT_CAN_ERROR;
}

uint8_t Fault_GetActive(const Fault_State_t *state) {
  return state->active_faults;
}

void Fault_ClearAll(Fault_State_t *state) {
  state->active_faults = FAULT_NONE;
  for (uint8_t i = 0U; i < 4U; i++) {
    state->encoder_fail_counter[i] = 0U;
    state->motor_overcurrent_flag[i] = false;
  }
}
