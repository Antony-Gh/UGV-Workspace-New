/**
 * @file    safety.c
 * @brief   Safety state machine implementation.
 *
 * State transitions are explicit — no hidden jumps.
 * E-STOP is handled in EXTI ISR for minimum latency.
 * All motor shutdowns go through Motor_EmergencyStop().
 */

#include "safety.h"

/* ──────────────── Module-Level State ──────────────────────────── */

/** Current system state (file-scope, accessed only via API). */
static volatile SystemState_t s_state = SYSTEM_INIT;

/* ──────────────────── Public Functions ─────────────────────────── */

void Safety_Init(void) {
  s_state = SYSTEM_INIT;

  /* Contactor OFF at boot (safe default) */
  HAL_GPIO_WritePin(CONTACTOR_GPIO_PORT, CONTACTOR_GPIO_PIN, GPIO_PIN_RESET);
}

void Safety_SetReady(void) {
  if (s_state == SYSTEM_INIT) {
    s_state = SYSTEM_READY;
  }
}

void Safety_Update(Motor_t motors[MOTOR_COUNT], Fault_State_t *fault_state) {
  uint8_t faults = Fault_GetActive(fault_state);

  switch (s_state) {
  /* ───── SYSTEM_INIT ─────────────────────────────────── */
  case SYSTEM_INIT:
    /* Waiting for Safety_SetReady() — do nothing here */
    break;

  /* ───── SYSTEM_READY ────────────────────────────────── */
  case SYSTEM_READY:
    /*
     * Transition to ACTIVE once we receive the first heartbeat.
     * The heartbeat flag is cleared by Fault_HeartbeatReceived(),
     * so if heartbeat timeout is NOT set, we know we have comms.
     */
    if ((faults & FAULT_HEARTBEAT_TIMEOUT) == 0U &&
        fault_state->last_heartbeat_tick != 0U) {
      /* Enable motor power relay */
      Safety_EnableContactor();
      Motor_EnableAll(motors);
      s_state = SYSTEM_ACTIVE;
    }
    break;

  /* ───── SYSTEM_ACTIVE ───────────────────────────────── */
  case SYSTEM_ACTIVE:
    if (faults != FAULT_NONE) {
      /* Any fault → disable motors, enter FAULT state */
      Motor_EmergencyStop(motors);
      Safety_DisableContactor();
      s_state = SYSTEM_FAULT;
    }
    break;

  /* ───── SYSTEM_FAULT ────────────────────────────────── */
  case SYSTEM_FAULT:
    /* Motors are already stopped.
     * Stay here until Safety_RequestReset() is called,
     * or escalate to ESTOP if hardware E-STOP fires.    */
    Motor_EmergencyStop(motors); /* Reinforce — belt & suspenders */
    break;

  /* ───── SYSTEM_ESTOP ────────────────────────────────── */
  case SYSTEM_ESTOP:
    /* Hard lockout — only Safety_RequestReset() can exit.
     * Keep motors disabled continuously.                  */
    Motor_EmergencyStop(motors);
    Safety_DisableContactor();
    break;

  default:
    /* Should never reach here — treat as ESTOP */
    s_state = SYSTEM_ESTOP;
    Motor_EmergencyStop(motors);
    Safety_DisableContactor();
    break;
  }
}

void Safety_ESTOP_IRQ(Motor_t motors[MOTOR_COUNT]) {
  /*
   * Called from EXTI ISR (falling edge on ESTOP_GPIO_PIN).
   * Must be fast, non-blocking, and ISR-safe.
   */

  /* Immediate motor shutdown */
  Motor_EmergencyStop(motors);

  /* Kill power relay */
  HAL_GPIO_WritePin(CONTACTOR_GPIO_PORT, CONTACTOR_GPIO_PIN, GPIO_PIN_RESET);

  /* Enter ESTOP state — this is a one-way transition until manual reset */
  s_state = SYSTEM_ESTOP;
}

bool Safety_RequestReset(void) {
  if (s_state == SYSTEM_ESTOP || s_state == SYSTEM_FAULT) {
    s_state = SYSTEM_INIT;
    return true;
  }
  return false;
}

SystemState_t Safety_GetState(void) { return s_state; }

void Safety_EnableContactor(void) {
  HAL_GPIO_WritePin(CONTACTOR_GPIO_PORT, CONTACTOR_GPIO_PIN, GPIO_PIN_SET);
}

void Safety_DisableContactor(void) {
  HAL_GPIO_WritePin(CONTACTOR_GPIO_PORT, CONTACTOR_GPIO_PIN, GPIO_PIN_RESET);
}

void Safety_DebugToggle(void) {
  HAL_GPIO_TogglePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN);
}
