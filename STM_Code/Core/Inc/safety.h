/**
 * @file    safety.h
 * @brief   Safety state machine, dual E-STOP, and battery isolation for the
 * UGV.
 *
 * States:
 *   SYSTEM_INIT       – Initialising peripherals
 *   SYSTEM_READY      – All peripherals up, awaiting first heartbeat
 *   SYSTEM_TELEOP     – Teleoperation mode — motors enabled, operator control
 *   SYSTEM_AUTONOMOUS – Autonomous mode — motors enabled, nav computer control
 *   SYSTEM_FAULT      – Recoverable fault detected, motors disabled
 *   SYSTEM_ESTOP      – Hardware E-STOP asserted, requires manual reset
 *
 * Transition rules:
 *   INIT       → READY       : all init complete
 *   READY      → TELEOP      : first heartbeat received (default mode)
 *   TELEOP     → AUTONOMOUS  : CAN mode-select command (0x110)
 *   AUTONOMOUS → TELEOP      : CAN mode-select command (0x110)
 *   TELEOP     → FAULT       : heartbeat timeout / stall / encoder fail / CAN
 * error AUTONOMOUS → FAULT       : heartbeat timeout / stall / encoder fail /
 * CAN error TELEOP     → ESTOP       : hardware E-STOP GPIO (mechanical or
 * wireless) AUTONOMOUS → ESTOP       : hardware E-STOP GPIO (mechanical or
 * wireless) FAULT      → READY       : faults cleared + CAN reset command
 * (0x130) FAULT      → ESTOP       : hardware E-STOP GPIO ESTOP      → INIT :
 * manual reset only
 *
 * Dual E-STOP:
 *   Mechanical : PC13 — EXTI falling edge, NVIC priority 0,0
 *   Wireless   : PG0  — EXTI falling edge, NVIC priority 0,1
 *
 * Battery isolation relay: PD5 — opened on E-STOP and FAULT.
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
  SYSTEM_TELEOP = 2,     /**< Active — teleoperation mode              */
  SYSTEM_AUTONOMOUS = 3, /**< Active — autonomous navigation mode      */
  SYSTEM_FAULT = 4,
  SYSTEM_ESTOP = 5
} SystemState_t;

/* ──────────────────── Configuration ───────────────────────────── */

/* --- Mechanical E-STOP (primary) --- */
#ifndef ESTOP_GPIO_PORT
#define ESTOP_GPIO_PORT GPIOC
#endif
#ifndef ESTOP_GPIO_PIN
#define ESTOP_GPIO_PIN GPIO_PIN_13
#endif

/* --- Wireless E-STOP (secondary) --- */
#ifndef ESTOP_WIRELESS_GPIO_PORT
#define ESTOP_WIRELESS_GPIO_PORT GPIOG
#endif
#ifndef ESTOP_WIRELESS_GPIO_PIN
#define ESTOP_WIRELESS_GPIO_PIN GPIO_PIN_0
#endif

/* --- Battery contactor / motor power relay --- */
#ifndef CONTACTOR_GPIO_PORT
#define CONTACTOR_GPIO_PORT GPIOD
#endif
#ifndef CONTACTOR_GPIO_PIN
#define CONTACTOR_GPIO_PIN GPIO_PIN_4
#endif

/* --- Battery isolation relay (URC power disconnect) --- */
#ifndef ISO_RELAY_GPIO_PORT
#define ISO_RELAY_GPIO_PORT GPIOD
#endif
#ifndef ISO_RELAY_GPIO_PIN
#define ISO_RELAY_GPIO_PIN GPIO_PIN_5
#endif

/* --- Debug GPIO for timing measurement --- */
#ifndef DEBUG_GPIO_PORT
#define DEBUG_GPIO_PORT GPIOB
#endif
#ifndef DEBUG_GPIO_PIN
#define DEBUG_GPIO_PIN GPIO_PIN_0
#endif

/* ──────────────────── Public API ──────────────────────────────── */

/**
 * @brief  Initialise the safety system.
 *         Sets state to SYSTEM_INIT, contactor + isolation relay OFF.
 */
void Safety_Init(void);

/**
 * @brief  Transition from SYSTEM_INIT to SYSTEM_READY.
 *         Call once all peripheral init is complete.
 */
void Safety_SetReady(void);

/**
 * @brief  Switch between TELEOP and AUTONOMOUS modes.
 * @param  autonomous  true = AUTONOMOUS, false = TELEOP.
 * @return true if mode switch was accepted (only valid in active states).
 *
 * Called from CAN RX handler when 0x110 mode-select is received.
 */
bool Safety_SetMode(bool autonomous);

/**
 * @brief  Check if the system is in an active state (TELEOP or AUTONOMOUS).
 * @return true if state is SYSTEM_TELEOP or SYSTEM_AUTONOMOUS.
 */
bool Safety_IsActive(void);

/**
 * @brief  Periodic safety update — call every 1 ms from control loop.
 * @param  motors       Pointer to motor array (used for emergency stop).
 * @param  fault_state  Pointer to fault state (used to check faults).
 *
 * Handles state transitions based on fault flags and heartbeat.
 */
void Safety_Update(Motor_t motors[MOTOR_COUNT], Fault_State_t *fault_state);

/**
 * @brief  E-STOP ISR handler — call from EXTI callback for EITHER pin.
 * @param  motors  Pointer to motor array.
 *
 * Immediately zeros PWM, disables contactor, opens isolation relay,
 * enters SYSTEM_ESTOP.  ISR-safe (no blocking, no printf).
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
 * @brief  Enable the battery isolation relay (close relay = power on).
 */
void Safety_EnableIsolation(void);

/**
 * @brief  Disable the battery isolation relay (open relay = power off).
 *         Called in E-STOP and FAULT states.
 */
void Safety_IsolateBAT(void);

/**
 * @brief  Toggle the debug GPIO pin (for oscilloscope timing).
 */
void Safety_DebugToggle(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_H */
