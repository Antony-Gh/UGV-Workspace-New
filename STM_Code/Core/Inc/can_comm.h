/**
 * @file    can_comm.h
 * @brief   CAN-FD communication interface for the UGV.
 *
 * Message IDs:
 *   RX (RPi → STM):
 *     0x100 – Velocity Command    (4 × int16 scaled velocities)
 *     0x110 – Mode Select         (TELEOP / AUTONOMOUS switch)
 *     0x120 – Config Profile      (competition selection + speed override)
 *     0x130 – Reset Command       (fault/ESTOP recovery request)
 *     0x200 – Heartbeat           (incrementing counter)
 *
 *   TX (STM → RPi):
 *     0x300 – Telemetry           (motor velocities + system state)
 *     0x310 – Odometry            (distance + heading from encoders)
 *     0x320 – Battery Telemetry   (voltage, current, temperature)
 *     0x400 – Fault Report        (fault bitmask + system state)
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

/* RX messages (Navigation Computer → Motor Controller) */
#define CAN_ID_VELOCITY_CMD 0x100U   /**< 4-wheel velocity setpoints     */
#define CAN_ID_MODE_SELECT 0x110U    /**< TELEOP / AUTONOMOUS switch     */
#define CAN_ID_CONFIG_PROFILE 0x120U /**< Competition profile selection   */
#define CAN_ID_RESET_CMD 0x130U      /**< Fault/ESTOP reset request      */
#define CAN_ID_HEARTBEAT 0x200U      /**< Keep-alive counter             */

/* TX messages (Motor Controller → Navigation Computer) */
#define CAN_ID_TELEMETRY 0x300U    /**< Motor telemetry + state        */
#define CAN_ID_ODOMETRY 0x310U     /**< Encoder-based odometry         */
#define CAN_ID_BATTERY 0x320U      /**< Battery voltage/current/temp   */
#define CAN_ID_FAULT_REPORT 0x400U /**< Fault bitmask + state          */

/* ──────────────── Velocity Scaling ────────────────────────────── */

/**
 * Velocities are transmitted as int16_t scaled by this factor.
 * Real velocity (rad/s) = raw_int16 / VELOCITY_SCALE.
 */
#define CAN_VELOCITY_SCALE 100.0f

/* ──────────────── Message Payloads (packed) ───────────────────── */

/** 0x100 – Velocity command from nav computer (8 bytes). */
typedef struct __attribute__((packed)) {
  int16_t velocity_fl; /**< Front-left  motor (× VELOCITY_SCALE) */
  int16_t velocity_fr; /**< Front-right motor                     */
  int16_t velocity_rl; /**< Rear-left   motor                     */
  int16_t velocity_rr; /**< Rear-right  motor                     */
} CAN_VelocityCmd_t;

/** 0x110 – Mode select command (1 byte). */
typedef struct __attribute__((packed)) {
  uint8_t mode; /**< 0 = TELEOP, 1 = AUTONOMOUS               */
} CAN_ModeCmd_t;

/** 0x120 – Config profile command (2 bytes). */
typedef struct __attribute__((packed)) {
  uint8_t competition; /**< Competition_t enum (0–4)              */
  uint8_t reserved;    /**< Future use / padding                  */
} CAN_ConfigCmd_t;

/** 0x130 – Reset command (1 byte). */
typedef struct __attribute__((packed)) {
  uint8_t reset_key; /**< Must be 0xA5 to confirm reset           */
} CAN_ResetCmd_t;

/** Reset confirmation key — prevents accidental resets. */
#define CAN_RESET_KEY 0xA5U

/** 0x200 – Heartbeat from nav computer (1 byte). */
typedef struct __attribute__((packed)) {
  uint8_t counter; /**< Incrementing sequence number          */
} CAN_Heartbeat_t;

/** 0x300 – Telemetry to nav computer (12 bytes). */
typedef struct __attribute__((packed)) {
  int16_t velocity_fl;  /**< Measured front-left  (× SCALE)       */
  int16_t velocity_fr;  /**< Measured front-right                  */
  int16_t velocity_rl;  /**< Measured rear-left                    */
  int16_t velocity_rr;  /**< Measured rear-right                   */
  uint8_t system_state; /**< SystemState_t enum value              */
  uint8_t fault_flags;  /**< Active fault bitmask                  */
  uint8_t comp_id;      /**< Active Competition_t enum             */
  uint8_t reserved;     /**< Padding / future use                  */
} CAN_Telemetry_t;

/** 0x310 – Odometry to nav computer (8 bytes). */
typedef struct __attribute__((packed)) {
  int32_t distance_mm;  /**< Total distance in millimeters         */
  int16_t heading_cdeg; /**< Heading in centidegrees (0.01°)       */
  uint16_t reserved;    /**< Padding / future use                  */
} CAN_Odometry_t;

/** 0x320 – Battery telemetry to nav computer (8 bytes). */
typedef struct __attribute__((packed)) {
  uint16_t voltage_mv; /**< Battery voltage in millivolts         */
  int16_t current_ma;  /**< Battery current in milliamps (signed) */
  int16_t temp_cdeg;   /**< Temperature in centidegrees C         */
  uint16_t reserved;   /**< Padding / future use                  */
} CAN_BatteryTelemetry_t;

/** 0x400 – Fault report to nav computer (2 bytes). */
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
 * Parses 0x100, 0x110, 0x120, 0x130, 0x200 messages.
 * Must be called inside HAL_FDCAN_RxFifo0Callback().
 */
void CAN_Comm_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief  Transmit a telemetry frame (0x300).
 * @param  vel_fl … vel_rr   Measured wheel velocities (rad/s).
 * @param  system_state      Current SystemState_t value.
 * @param  fault_flags       Active fault bitmask.
 * @param  comp_id           Active Competition_t value.
 */
void CAN_Comm_SendTelemetry(float vel_fl, float vel_fr, float vel_rl,
                            float vel_rr, uint8_t system_state,
                            uint8_t fault_flags, uint8_t comp_id);

/**
 * @brief  Transmit an odometry frame (0x310).
 * @param  distance_mm  Total accumulated distance in millimeters.
 * @param  heading_cdeg Heading in centidegrees.
 */
void CAN_Comm_SendOdometry(int32_t distance_mm, int16_t heading_cdeg);

/**
 * @brief  Transmit a battery telemetry frame (0x320).
 * @param  voltage_mv   Battery voltage in millivolts.
 * @param  current_ma   Battery current in milliamps (signed).
 * @param  temp_cdeg    Temperature in centidegrees Celsius.
 */
void CAN_Comm_SendBattery(uint16_t voltage_mv, int16_t current_ma,
                          int16_t temp_cdeg);

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
