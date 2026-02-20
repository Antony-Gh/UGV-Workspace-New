/**
 * @file    safety.h
 * @brief   Safety state machine and E-STOP handling for the UGV.
 *
 * States:
 *   SYSTEM_INIT   – Initialising peripherals
 *   SYSTEM_READY  – All peripherals up, awaiting first heartbeat
 *   SYSTEM_ACTIVE – Motors enabled, velocity commands accepted
 *   SYSTEM_FAULT  – Recoverable fault detected, motors disabled
 *   SYSTEM_ESTOP  – Hardware E-STOP asserted, requires manual reset
 *
 * Transition rules:
 *   INIT   → READY   : all init complete
 *   READY  → ACTIVE  : first heartbeat received
 *   ACTIVE → FAULT   : heartbeat timeout / stall / encoder fail / CAN error
 *   ACTIVE → ESTOP   : hardware E-STOP GPIO
 *   FAULT  → READY   : faults cleared + reset command
 *   FAULT  → ESTOP   : hardware E-STOP GPIO
 *   ESTOP  → INIT    : manual reset only
 *
 * @author  UGV Firmware Team
 */

#ifndef SAFETY_H
#define SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fault.h"
#include "motor.h"
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


/* ──────────────────── System States ───────────────────────────── */

typedef enum {
  SYSTEM_INIT = 0,
  SYSTEM_READY = 1,
  SYSTEM_ACTIVE = 2,
  SYSTEM_FAULT = 3,
  SYSTEM_ESTOP = 4
} SystemState_t;

/* ──────────────────── Configuration ───────────────────────────── */

/**
 * E-STOP GPIO — directly to an EXTI-capable pin.
 * Overrideable at compile time.
 */
#ifndef ESTOP_GPIO_PORT
#define ESTOP_GPIO_PORT GPIOC
#endif
#ifndef ESTOP_GPIO_PIN
#define ESTOP_GPIO_PIN GPIO_PIN_13 /* Nucleo-144 user button as test */
#endif

/**
 * Battery contactor / motor power relay GPIO.
 */
#ifndef CONTACTOR_GPIO_PORT
#define CONTACTOR_GPIO_PORT GPIOD
#endif
#ifndef CONTACTOR_GPIO_PIN
#define CONTACTOR_GPIO_PIN GPIO_PIN_0
#endif

/**
 * Debug GPIO for timing measurement.
 */
#ifndef DEBUG_GPIO_PORT
#define DEBUG_GPIO_PORT GPIOB
#endif
#ifndef DEBUG_GPIO_PIN
#define DEBUG_GPIO_PIN GPIO_PIN_0
#endif

/* ──────────────────── Public API ──────────────────────────────── */

/**
 * @brief  Initialise the safety system.
 *         Sets state to SYSTEM_INIT, configures contactor GPIO off.
 */
void Safety_Init(void);

/**
 * @brief  Transition from SYSTEM_INIT to SYSTEM_READY.
 *         Call once all peripheral init is complete.
 */
void Safety_SetReady(void);

/**
 * @brief  Periodic safety update — call every 1 ms from control loop.
 * @param  motors       Pointer to motor array (used for emergency stop).
 * @param  fault_state  Pointer to fault state (used to check faults).
 *
 * Handles state transitions based on fault flags and heartbeat.
 */
void Safety_Update(Motor_t motors[MOTOR_COUNT], Fault_State_t *fault_state);

/**
 * @brief  E-STOP ISR handler — call from EXTI callback.
 * @param  motors  Pointer to motor array.
 *
 * Immediately zeros PWM, disables contactor, enters SYSTEM_ESTOP.
 * This function is ISR-safe (no blocking, no printf).
 */
void Safety_ESTOP_IRQ(Motor_t motors[MOTOR_COUNT]);

/**
 * @brief  Request manual reset from ESTOP or FAULT state.
 * @return true if reset was successful (transitions to SYSTEM_INIT).
 */
bool Safety_RequestReset(void);

/**
 * @brief  Get the current system state.
 * @return Current SystemState_t.
 */
SystemState_t Safety_GetState(void);

/**
 * @brief  Enable the battery contactor / motor power relay.
 */
void Safety_EnableContactor(void);

/**
 * @brief  Disable the battery contactor / motor power relay.
 */
void Safety_DisableContactor(void);

/**
 * @brief  Toggle the debug GPIO pin (for oscilloscope timing).
 */
void Safety_DebugToggle(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_H */
