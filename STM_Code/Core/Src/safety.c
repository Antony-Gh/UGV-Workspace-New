/**
 * @file    safety.c
 * @brief   Safety state machine implementation — enhanced for
 * multi-competition.
 *
 * State transitions are explicit — no hidden jumps.
 * E-STOP is handled in EXTI ISR for minimum latency.
 * Both mechanical (PC13) and wireless (PG0) E-STOP pins route here.
 * All motor shutdowns go through Motor_EmergencyStop().
 * Battery isolation relay is opened in FAULT and ESTOP states.
 */

#include "safety.h"
#include "lights.h"

/* ──────────────── Module-Level State ──────────────────────────── */

/** Current system state (file-scope, accessed only via API). */
static volatile SystemState_t s_state = SYSTEM_INIT;

/* ──────────────────── Public Functions ─────────────────────────── */

void Safety_Init(void) {
  s_state = SYSTEM_INIT;

  /* Contactor OFF at boot (safe default) */
  HAL_GPIO_WritePin(CONTACTOR_GPIO_PORT, CONTACTOR_GPIO_PIN, GPIO_PIN_RESET);

  /* Isolation relay OFF at boot (disconnected = safe) */
  HAL_GPIO_WritePin(ISO_RELAY_GPIO_PORT, ISO_RELAY_GPIO_PIN, GPIO_PIN_RESET);
}

void Safety_SetReady(void) {
  if (s_state == SYSTEM_INIT) {
    s_state = SYSTEM_READY;
    /* Set lights to power-on indicator */
    Lights_SetMode(LIGHT_POWER_ON);
  }
}

bool Safety_SetMode(bool autonomous) {
  /* Mode switch only valid from active states (TELEOP or AUTONOMOUS) */
  if (s_state != SYSTEM_TELEOP && s_state != SYSTEM_AUTONOMOUS) {
    return false;
  }

  if (autonomous) {
    s_state = SYSTEM_AUTONOMOUS;
    Lights_SetMode(LIGHT_AUTONOMOUS);
  } else {
    s_state = SYSTEM_TELEOP;
    Lights_SetMode(LIGHT_TELEOP);
  }

  return true;
}

bool Safety_IsActive(void) {
  return (s_state == SYSTEM_TELEOP || s_state == SYSTEM_AUTONOMOUS);
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
     * Transition to TELEOP (default) once we receive the first heartbeat.
     * The heartbeat flag is cleared by Fault_HeartbeatReceived(),
     * so if heartbeat timeout is NOT set, we know we have comms.
     */
    if ((faults & FAULT_HEARTBEAT_TIMEOUT) == 0U &&
        fault_state->last_heartbeat_tick != 0U) {
      /* Enable motor power relay + isolation relay */
      Safety_EnableContactor();
      Safety_EnableIsolation();
      Motor_EnableAll(motors);
      s_state = SYSTEM_TELEOP;
      Lights_SetMode(LIGHT_TELEOP);
    }
    break;

  /* ───── SYSTEM_TELEOP ───────────────────────────────── */
  case SYSTEM_TELEOP:
    if (faults != FAULT_NONE) {
      /* Any fault → disable motors, enter FAULT state */
      Motor_EmergencyStop(motors);
      Safety_DisableContactor();
      Safety_IsolateBAT();
      s_state = SYSTEM_FAULT;
      Lights_SetMode(LIGHT_FAULT);
    }
    break;

  /* ───── SYSTEM_AUTONOMOUS ───────────────────────────── */
  case SYSTEM_AUTONOMOUS:
    if (faults != FAULT_NONE) {
      Motor_EmergencyStop(motors);
      Safety_DisableContactor();
      Safety_IsolateBAT();
      s_state = SYSTEM_FAULT;
      Lights_SetMode(LIGHT_FAULT);
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
    Safety_IsolateBAT();
    break;

  default:
    /* Should never reach here — treat as ESTOP */
    s_state = SYSTEM_ESTOP;
    Motor_EmergencyStop(motors);
    Safety_DisableContactor();
    Safety_IsolateBAT();
    Lights_SetMode(LIGHT_ESTOP);
    break;
  }
}

void Safety_ESTOP_IRQ(Motor_t motors[MOTOR_COUNT]) {
  /*
   * Called from EXTI ISR — triggered by EITHER:
   *   - Mechanical E-STOP  (PC13 falling edge)
   *   - Wireless E-STOP    (PG0  falling edge)
   * Must be fast, non-blocking, and ISR-safe.
   */

  /* Immediate motor shutdown */
  Motor_EmergencyStop(motors);

  /* Kill motor power relay */
  HAL_GPIO_WritePin(CONTACTOR_GPIO_PORT, CONTACTOR_GPIO_PIN, GPIO_PIN_RESET);

  /* Open battery isolation relay (URC requirement) */
  HAL_GPIO_WritePin(ISO_RELAY_GPIO_PORT, ISO_RELAY_GPIO_PIN, GPIO_PIN_RESET);

  /* Force all lights off, then set ESTOP pattern */
  Lights_AllOff();

  /* Enter ESTOP state — one-way transition until manual reset */
  s_state = SYSTEM_ESTOP;
}

bool Safety_RequestReset(void) {
  if (s_state == SYSTEM_ESTOP || s_state == SYSTEM_FAULT) {
    s_state = SYSTEM_INIT;
    Lights_SetMode(LIGHT_OFF);
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

void Safety_EnableIsolation(void) {
  HAL_GPIO_WritePin(ISO_RELAY_GPIO_PORT, ISO_RELAY_GPIO_PIN, GPIO_PIN_SET);
}

void Safety_IsolateBAT(void) {
  HAL_GPIO_WritePin(ISO_RELAY_GPIO_PORT, ISO_RELAY_GPIO_PIN, GPIO_PIN_RESET);
}

void Safety_DebugToggle(void) {
  HAL_GPIO_TogglePin(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN);
}
