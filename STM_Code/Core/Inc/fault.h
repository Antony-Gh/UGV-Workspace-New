/**
 * @file    fault.h
 * @brief   Fault detection and reporting for the UGV.
 *
 * Monitors:
 *   - Heartbeat timeout (>300 ms without CAN 0x200)
 *   - Motor stall (overcurrent flag while PWM active)
 *   - Encoder failure (no count change while PWM > threshold)
 *   - CAN bus error (HAL error flag)
 *
 * Fault status is exposed as a bitmask and can be transmitted via CAN.
 *
 * @author  UGV Firmware Team
 */

#ifndef FAULT_H
#define FAULT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>


/* ─────────────────── Fault Bit Definitions ────────────────────── */

#define FAULT_NONE 0x00U
#define FAULT_HEARTBEAT_TIMEOUT 0x01U /**< No heartbeat for >300 ms     */
#define FAULT_MOTOR_STALL 0x02U       /**< Overcurrent flag asserted    */
#define FAULT_ENCODER_FAILURE 0x04U   /**< Encoder stuck while driving  */
#define FAULT_CAN_ERROR 0x08U         /**< CAN bus / protocol error     */

/* ───────────────────── Configuration ──────────────────────────── */

/** Heartbeat timeout threshold in milliseconds. */
#define FAULT_HEARTBEAT_TIMEOUT_MS 300U

/**
 * Minimum PWM duty (%) to consider a motor "actively driven"
 * when checking for encoder failure.
 */
#define FAULT_ENCODER_PWM_THRESHOLD 5.0f

/**
 * Number of consecutive 1 ms cycles with zero encoder delta
 * while PWM is active before declaring encoder failure.
 */
#define FAULT_ENCODER_FAIL_COUNT 100U

/* ─────────────────── Fault State Struct ────────────────────────── */

typedef struct {
  uint8_t active_faults;            /**< Bitmask of active faults      */
  uint32_t last_heartbeat_tick;     /**< HAL tick of last heartbeat RX */
  uint16_t encoder_fail_counter[4]; /**< Per-motor encoder fail count  */
  bool motor_overcurrent_flag[4];   /**< Per-motor overcurrent input   */
} Fault_State_t;

/* ─────────────────── Public API ───────────────────────────────── */

/**
 * @brief  Initialise fault detection state.
 * @param  state  Pointer to fault state structure.
 */
void Fault_Init(Fault_State_t *state);

/**
 * @brief  Run all fault checks (call every 1 ms from control loop).
 * @param  state           Pointer to fault state.
 * @param  current_tick    Current HAL_GetTick() value.
 * @param  pwm_duty        Array[4] of current PWM duty percentages.
 * @param  encoder_delta   Array[4] of encoder tick deltas this cycle.
 * @param  overcurrent     Array[4] of overcurrent GPIO states (true = fault).
 */
void Fault_Check(Fault_State_t *state, uint32_t current_tick,
                 const float pwm_duty[4], const int32_t encoder_delta[4],
                 const bool overcurrent[4]);

/**
 * @brief  Record reception of a heartbeat message (resets timeout).
 * @param  state         Pointer to fault state.
 * @param  current_tick  Current HAL_GetTick() value.
 */
void Fault_HeartbeatReceived(Fault_State_t *state, uint32_t current_tick);

/**
 * @brief  Set or clear the CAN error fault flag.
 * @param  state   Pointer to fault state.
 * @param  error   true = CAN error active; false = cleared.
 */
void Fault_SetCANError(Fault_State_t *state, bool error);

/**
 * @brief  Return the active fault bitmask.
 * @param  state  Pointer to fault state.
 * @return Bitmask of FAULT_xxx flags.
 */
uint8_t Fault_GetActive(const Fault_State_t *state);

/**
 * @brief  Clear all fault flags (call after fault recovery / reset).
 * @param  state  Pointer to fault state.
 */
void Fault_ClearAll(Fault_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_H */
