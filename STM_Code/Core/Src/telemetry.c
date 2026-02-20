/**
 * @file    telemetry.c
 * @brief   Periodic telemetry transmission implementation.
 *
 * Called from the main super-loop.
 * Uses tick-based timing (HAL_GetTick) — never blocks.
 */

#include "telemetry.h"
#include "can_comm.h"
#include "stm32h7xx_hal.h"

/* ──────────────── Module-Level State ──────────────────────────── */

static uint32_t s_last_tx_tick = 0U;

/* ──────────────── Public Functions ─────────────────────────────── */

void Telemetry_Init(void) { s_last_tx_tick = HAL_GetTick(); }

void Telemetry_Update(const Motor_t motors[MOTOR_COUNT],
                      const Fault_State_t *fault_state) {
  uint32_t now = HAL_GetTick();

  if ((now - s_last_tx_tick) < TELEMETRY_INTERVAL_MS) {
    return; /* Not time yet */
  }

  s_last_tx_tick = now;

  CAN_Comm_SendTelemetry(
      motors[0].state.current_velocity, motors[1].state.current_velocity,
      motors[2].state.current_velocity, motors[3].state.current_velocity,
      (uint8_t)Safety_GetState(), Fault_GetActive(fault_state));
}
