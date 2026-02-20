/**
 * @file    can_comm.h
 * @brief   CAN-FD communication interface for the UGV.
 *
 * Message IDs:
 *   0x100 – Velocity Command  (RPi → STM, 4 × int16 scaled velocities)
 *   0x200 – Heartbeat          (RPi → STM, 1 byte counter)
 *   0x300 – Telemetry           (STM → RPi, motor + system state)
 *   0x400 – Fault Report        (STM → RPi, fault bitmask)
 *
 * Transport: FDCAN1, 1 Mbps nominal / 5 Mbps data phase.
 * RX is interrupt-driven; TX is called from main loop (telemetry)
 * or from fault handler.
 *
 * @author  UGV Firmware Team
 */

#ifndef CAN_COMM_H
#define CAN_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


/* ──────────────── CAN Message IDs ─────────────────────────────── */

#define CAN_ID_VELOCITY_CMD 0x100U
#define CAN_ID_HEARTBEAT 0x200U
#define CAN_ID_TELEMETRY 0x300U
#define CAN_ID_FAULT_REPORT 0x400U

/* ──────────────── Velocity Scaling ────────────────────────────── */

/**
 * Velocities are transmitted as int16_t scaled by this factor.
 * Real velocity (rad/s) = raw_int16 / VELOCITY_SCALE.
 */
#define CAN_VELOCITY_SCALE 100.0f

/* ──────────────── Message Payloads (packed) ───────────────────── */

/** 0x100 – Velocity command from Raspberry Pi (8 bytes). */
typedef struct __attribute__((packed)) {
  int16_t velocity_fl; /**< Front-left  motor (× VELOCITY_SCALE) */
  int16_t velocity_fr; /**< Front-right motor                     */
  int16_t velocity_rl; /**< Rear-left   motor                     */
  int16_t velocity_rr; /**< Rear-right  motor                     */
} CAN_VelocityCmd_t;

/** 0x200 – Heartbeat from Raspberry Pi (1 byte). */
typedef struct __attribute__((packed)) {
  uint8_t counter; /**< Incrementing sequence number          */
} CAN_Heartbeat_t;

/** 0x300 – Telemetry to Raspberry Pi (12 bytes). */
typedef struct __attribute__((packed)) {
  int16_t velocity_fl;  /**< Measured front-left  (× SCALE)       */
  int16_t velocity_fr;  /**< Measured front-right                  */
  int16_t velocity_rl;  /**< Measured rear-left                    */
  int16_t velocity_rr;  /**< Measured rear-right                   */
  uint8_t system_state; /**< SystemState_t enum value              */
  uint8_t fault_flags;  /**< Active fault bitmask                  */
  uint16_t reserved;    /**< Padding / future use                  */
} CAN_Telemetry_t;

/** 0x400 – Fault report to Raspberry Pi (2 bytes). */
typedef struct __attribute__((packed)) {
  uint8_t fault_flags;  /**< Active fault bitmask                  */
  uint8_t system_state; /**< Current SystemState_t                 */
} CAN_FaultReport_t;

/* ──────────────── Shared RX Data ──────────────────────────────── */

/**
 * @brief Latest received velocity command (written in RX ISR).
 *
 * Declared extern here; defined in can_comm.c.
 * Read by the control loop every 1 ms.
 * Updated atomically (single 64-bit write on Cortex-M7).
 */
extern volatile CAN_VelocityCmd_t g_can_velocity_cmd;

/** Flag set by RX ISR when a new velocity command arrives. */
extern volatile bool g_can_velocity_cmd_new;

/* ──────────────── Public API ──────────────────────────────────── */

/**
 * @brief  Initialise FDCAN1 peripheral: filters, bit timing, interrupts.
 * @param  hfdcan  Pointer to HAL FDCAN handle (must be pre-configured).
 */
void CAN_Comm_Init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief  FDCAN RX FIFO0 callback — called from HAL ISR.
 * @param  hfdcan    HAL FDCAN handle.
 * @param  RxFifo0ITs  Interrupt flags.
 *
 * Parses 0x100 (velocity) and 0x200 (heartbeat) messages.
 * Must be called inside HAL_FDCAN_RxFifo0Callback().
 */
void CAN_Comm_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief  Transmit a telemetry frame (0x300).
 * @param  vel_fl … vel_rr   Measured wheel velocities (rad/s).
 * @param  system_state      Current SystemState_t value.
 * @param  fault_flags       Active fault bitmask.
 */
void CAN_Comm_SendTelemetry(float vel_fl, float vel_fr, float vel_rl,
                            float vel_rr, uint8_t system_state,
                            uint8_t fault_flags);

/**
 * @brief  Transmit a fault report frame (0x400).
 * @param  fault_flags    Active fault bitmask.
 * @param  system_state   Current SystemState_t value.
 */
void CAN_Comm_SendFault(uint8_t fault_flags, uint8_t system_state);

#ifdef __cplusplus
}
#endif

#endif /* CAN_COMM_H */
